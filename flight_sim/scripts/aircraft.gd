extends RefCounted
## Body and world axes match the MATLAB model: x left, y backward, z up.
const Propulsion = preload("res://scripts/propulsion.gd")
const Catalog = preload("res://scripts/aircraft_catalog.gd")
const CLEARANCE := 0.12
const TO_GODOT := Basis(Vector3(-1, 0, 0), Vector3(0, 0, 1), Vector3(0, 1, 0))
const TO_FLU := Basis(Vector3(0, 1, 0), Vector3(-1, 0, 0), Vector3(0, 0, 1))
var cfg: Dictionary
var physics: Dictionary
var propulsion
var mass: float
var inertia: Basis
var inv_inertia: Basis
var arms: Array[Vector3] = []
var feet: Array[Vector3] = []
var thrust := PackedFloat64Array([0, 0, 0, 0])
var position := Vector3(0, 0, CLEARANCE)
var velocity := Vector3.ZERO
var orientation := Quaternion.IDENTITY
var omega := Vector3.ZERO
var acceleration := Vector3.ZERO
var specific_force := Vector3(0, 0, 9.80665)
var wind_world := Vector3.ZERO
var crashed := false
var contact := true
var drag_enabled := true
var ground_effect_enabled := true
var ground_contact_enabled := true
var configuration_error := ""
var profile_id := Catalog.DEFAULT_ID
var clearance := CLEARANCE

func _init(load_measured := true, selected_id := Catalog.DEFAULT_ID) -> void:
	profile_id = selected_id
	var entry := Catalog.find(profile_id)
	if entry.is_empty():
		# Stay usable but prevent arming with a silently substituted aircraft.
		configuration_error = "Unknown aircraft profile: " + profile_id
		entry = Catalog.find(Catalog.DEFAULT_ID)
	cfg = JSON.parse_string(FileAccess.get_file_as_string(entry.aircraft_path))
	physics = JSON.parse_string(FileAccess.get_file_as_string(entry.physics_path))
	clearance = float(cfg.get("ground_clearance_m", CLEARANCE))
	position.z = clearance
	mass = cfg.mass_kg
	var rows: Array = cfg.inertia_kg_m2
	inertia = Basis(vec(rows[0]), vec(rows[1]), vec(rows[2])).transposed()
	inv_inertia = inertia.inverse()
	for m in cfg.motors_m:
		var arm := vec(m) - vec(cfg.cg_m)
		arms.append(arm)
		feet.append(Vector3(arm.x, arm.y, 0) * float(physics.contact.footprint_motor_arm_scale) + Vector3(0, 0, -clearance))
	propulsion = Propulsion.new(cfg, physics.motor, load_measured, entry.curve_path)
	if configuration_error == "": configuration_error = propulsion.error
	wind_world = vec(physics.aerodynamics.wind_world_m_s)
	ground_effect_enabled = physics.ground_effect.enabled

static func vec(a: Array) -> Vector3:
	return Vector3(a[0], a[1], a[2])

func reset() -> void:
	position = Vector3(0, 0, clearance)
	velocity = Vector3.ZERO
	orientation = Quaternion.IDENTITY
	omega = Vector3.ZERO
	propulsion.reset()
	thrust.fill(0)
	crashed = false
	contact = true
	acceleration = Vector3.ZERO
	specific_force = Vector3(0, 0, cfg.gravity)

func crash() -> void:
	crashed = true
	velocity = Vector3.ZERO
	omega = Vector3.ZERO
	propulsion.reset()
	thrust.fill(0)

func prime_motors(commands: PackedFloat64Array) -> void:
	propulsion.set_steady(commands)
	update_thrust_readout()

func aerodynamic_wrench(air_velocity_body: Vector3, rates: Vector3, speeds: PackedFloat64Array) -> Array[Vector3]:
	if not drag_enabled:
		return [Vector3.ZERO, Vector3.ZERO]
	var aero: Dictionary = physics.aerodynamics
	# Positive diagonal CdA makes force oppose air-relative motion in every attitude.
	var pressure_arm := vec(aero.center_of_pressure_relative_to_cg_m)
	var local_body_velocity := air_velocity_body + rates.cross(pressure_arm)
	var force := -0.5 * float(aero.air_density_kg_m3) * local_body_velocity.length() * vec(aero.body_cd_area_m2) * local_body_velocity
	var moment := pressure_arm.cross(force)
	moment -= vec(aero.angular_drag_Nm_s2) * rates * rates.abs()
	for i in range(4):
		# Each rotor sees translation plus its own tangential velocity during rotation.
		var local_velocity := air_velocity_body + rates.cross(arms[i])
		var rotor_force := -float(aero.rotor_in_plane_drag_N_s_per_m_at_full_speed) * speeds[i] * Vector3(local_velocity.x, local_velocity.y, 0)
		force += rotor_force
		moment += arms[i].cross(rotor_force)
	return [force, moment]

func ground_multiplier(i: int, p: Vector3, rotation: Basis) -> float:
	if not ground_effect_enabled:
		return 1.0
	var settings: Dictionary = physics.ground_effect
	var alignment := rotation.z.z
	var minimum: float = settings.minimum_upward_alignment
	if alignment <= minimum:
		return 1.0
	var height := maxf((p + rotation * arms[i]).z, float(cfg.propeller_radius_m) * 0.5)
	var ratio := float(cfg.propeller_radius_m) / (4.0 * height)
	var gain := minf(float(settings.maximum_thrust_multiplier), 1.0 / (1.0 - ratio*ratio))
	var blend := smoothstep(minimum, 1.0, alignment)
	return 1.0 + blend * (gain - 1.0)

func wrench(p: Vector3, rotation: Basis, v: Vector3, rates: Vector3, speeds: PackedFloat64Array) -> Array[Vector3]:
	var aero := aerodynamic_wrench(rotation.transposed() * (v - wind_world), rates, speeds)
	var force: Vector3 = aero[0]
	var moment: Vector3 = aero[1]
	for i in range(4):
		var static_force: float = propulsion.max_thrust * speeds[i] * speeds[i]
		var rotor_force := Vector3(0, 0, static_force * ground_multiplier(i, p, rotation))
		force += rotor_force
		moment += arms[i].cross(rotor_force)
		# Ground effect changes lift for a given rotor state; it doesn't directly
		# multiply shaft torque. Unmeasured aerodynamic torque uses the old Q/T.
		moment.z += float(cfg.spin_sign[i]) * propulsion.reaction_torque(static_force)
	return [force, moment]

func step(commands: PackedFloat64Array, dt: float) -> void:
	if crashed:
		crash()
		return
	if dt <= 0 or configuration_error != "":
		return
	var before_velocity := velocity
	var rotation := Basis(orientation)
	var gravity := Vector3(0, 0, -cfg.gravity)
	var mid_speed: PackedFloat64Array = propulsion.evolve(commands, dt)
	# Explicit midpoint integration evaluates forces at the intermediate attitude,
	# airspeed and rates, rather than using the previous orientation for all thrust.
	var first := wrench(position, rotation, velocity, omega, mid_speed)
	var a0 := rotation * first[0] / mass + gravity
	var w0 := inv_inertia * (first[1] - omega.cross(inertia * omega))
	var mid_rate := omega + w0 * (dt * 0.5)
	var mid_q := advance_attitude(orientation, omega, dt * 0.5)
	var mid_rotation := Basis(mid_q)
	var mid_velocity := velocity + a0 * (dt * 0.5)
	var middle := wrench(position + velocity * (dt * 0.5), mid_rotation, mid_velocity, mid_rate, mid_speed)
	position += mid_velocity * dt
	velocity += (mid_rotation * middle[0] / mass + gravity) * dt
	omega += inv_inertia * (middle[1] - mid_rate.cross(inertia * mid_rate)) * dt
	orientation = advance_attitude(orientation, mid_rate, dt)
	contact = false
	if ground_contact_enabled:
		solve_ground()
	acceleration = (velocity - before_velocity) / dt
	specific_force = Basis(orientation).transposed() * (acceleration - gravity)
	update_thrust_readout()
	if not position.is_finite() or not velocity.is_finite() or not omega.is_finite() or not orientation.is_finite():
		reset()
		crash()

static func advance_attitude(q: Quaternion, rates: Vector3, dt: float) -> Quaternion:
	if rates.length_squared() < 1e-18:
		return q
	return (q * Quaternion(rates.normalized(), rates.length() * dt)).normalized()

func update_thrust_readout() -> void:
	for i in range(4):
		thrust[i] = propulsion.max_thrust * propulsion.speed[i] * propulsion.speed[i] * ground_multiplier(i, position, Basis(orientation))

func apply_impulse(impulse_world: Vector3, arm_body: Vector3, rotation: Basis) -> void:
	velocity += impulse_world / mass
	omega += inv_inertia * arm_body.cross(rotation.transposed() * impulse_world)

func inverse_effective_mass(direction_world: Vector3, arm_body: Vector3, rotation: Basis) -> float:
	var direction_body := rotation.transposed() * direction_world
	return 1.0 / mass + direction_body.dot((inv_inertia * arm_body.cross(direction_body)).cross(arm_body))

func solve_ground() -> void:
	var rotation := Basis(orientation)
	var active: Array[int] = []
	var penetration := 0.0
	for i in range(feet.size()):
		var r := rotation * feet[i]
		var point := position + r
		if point.z <= 0.0005: # 0.5 mm contact skin avoids alternating support points at rest.
			active.append(i)
			penetration = maxf(penetration, -point.z)
			var closing := (velocity + (rotation * omega).cross(r)).z
			if closing < -float(physics.contact.crash_normal_speed_m_s):
				position.z += penetration
				crash()
				contact = true
				return
	# A disk tipped into the ground counts as a prop strike, independently of CG height.
	var disk_drop := float(cfg.propeller_radius_m) * sqrt(maxf(0, 1-rotation.z.z*rotation.z.z))
	for r in arms:
		if (position + rotation*r).z - disk_drop <= 0:
			crash()
			contact = true
			return
	if active.is_empty():
		return
	contact = true
	position.z += penetration
	var normal_impulses := PackedFloat64Array([0,0,0,0])
	var friction_impulses: Array[Vector3] = [Vector3.ZERO,Vector3.ZERO,Vector3.ZERO,Vector3.ZERO]
	var normal := Vector3(0,0,1)
	for iteration in range(int(physics.contact.solver_iterations)):
		for i in active:
			var point_velocity := velocity + rotation * omega.cross(feet[i])
			var old_normal := normal_impulses[i]
			normal_impulses[i] = maxf(0, old_normal - point_velocity.z / inverse_effective_mass(normal, feet[i], rotation))
			apply_impulse(normal * (normal_impulses[i]-old_normal), feet[i], rotation)
			point_velocity = velocity + rotation * omega.cross(feet[i])
			var tangent := Vector3(point_velocity.x,point_velocity.y,0)
			var candidate := friction_impulses[i]
			if tangent.length_squared() > 1e-16:
				var direction := tangent.normalized()
				candidate -= tangent / inverse_effective_mass(direction, feet[i], rotation)
			# The friction bound must shrink if the normal load shrinks, even
			# when the current tangential velocity is already effectively zero.
			candidate = candidate.limit_length(float(physics.contact.friction_coefficient)*normal_impulses[i])
			apply_impulse(candidate - friction_impulses[i], feet[i], rotation)
			friction_impulses[i] = candidate

func render_transform() -> Transform3D:
	return Transform3D(TO_GODOT * Basis(orientation) * TO_GODOT.transposed(), TO_GODOT * position)
