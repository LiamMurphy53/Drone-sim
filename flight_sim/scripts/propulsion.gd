extends RefCounted
## Steady thrust map and normalized rotor dynamics, independent of aircraft motion.
var max_thrust := 13.5
var torque_ratio := 0.013
var tau_up := 0.05
var tau_down := 0.05
var response_model := "rotor_speed"
var speed := PackedFloat64Array([0, 0, 0, 0])
var curve: Array = []
var source := "Original linear thrust estimate"
var error := ""

func _init(aircraft: Dictionary, settings: Dictionary, load_measured := true, curve_path := "res://config/propulsion_curve.json") -> void:
	max_thrust = aircraft.max_thrust_N
	torque_ratio = aircraft.torque_per_thrust_m
	tau_up = float(settings.spin_up_tau_s) if settings.get("spin_up_tau_s") != null else float(aircraft.motor_tau_s)
	tau_down = float(settings.spin_down_tau_s) if settings.get("spin_down_tau_s") != null else float(aircraft.motor_tau_s)
	response_model = settings.get("response_model", "rotor_speed")
	if not is_finite(tau_up) or not is_finite(tau_down) or tau_up <= 0 or tau_down <= 0 or response_model not in ["rotor_speed", "legacy_thrust"]:
		error = "Invalid motor response settings in config/physics.json"
	if error == "" and load_measured and FileAccess.file_exists(curve_path):
		error = load_curve(JSON.parse_string(FileAccess.get_file_as_string(curve_path)))

func load_curve(data: Variant) -> String:
	# Validate the entire document before replacing the current model.
	if not data is Dictionary or data.get("schema_version") != 1:
		return "Invalid propulsion curve schema. Reimport the measurements."
	var points: Variant = data.get("points")
	if not points is Array or points.size() < 2:
		return "A propulsion curve needs at least two points."
	var previous_command := -1.0
	var previous_thrust := -1.0
	var has_torque: bool = points[0] is Dictionary and points[0].has("torque_Nm")
	var previous_torque := -1.0
	for point in points:
		if not point is Dictionary:
			return "Invalid propulsion curve point."
		for field in ["command", "thrust_N"]:
			if not point.get(field) is float and not point.get(field) is int:
				return "Curve values must be finite numbers."
			if not is_finite(float(point[field])):
				return "Curve values must be finite numbers."
		var command: float = point.command
		var thrust: float = point.thrust_N
		if command < 0 or command > 1 or command <= previous_command or thrust < 0 or thrust < previous_thrust:
			return "Commands must increase from 0 to 1; thrust must be nonnegative and monotonic."
		if point.has("torque_Nm") != has_torque:
			return "Supply torque for every curve point or none."
		if has_torque:
			var torque: Variant = point.torque_Nm
			if (not torque is float and not torque is int) or not is_finite(float(torque)) or float(torque) < 0:
				return "Torque values must be finite and nonnegative."
			if thrust == previous_thrust and float(torque) != previous_torque:
				return "A thrust plateau must have the same torque at both ends."
			previous_torque = torque
		previous_command = command
		previous_thrust = thrust
	if float(points[0].command) != 0 or float(points[-1].command) != 1 or float(points[0].thrust_N) != 0 or float(points[-1].thrust_N) <= 0:
		return "Curve must include zero command/zero thrust and a full-command measurement."
	if has_torque and float(points[0].torque_Nm) != 0:
		return "Zero thrust must have zero torque."
	curve = points.duplicate(true)
	max_thrust = float(points[-1].thrust_N)
	source = str(data.get("source", "Measured thrust curve"))
	return ""

func static_thrust(command: float) -> float:
	command = clampf(command, 0, 1)
	if curve.is_empty():
		return command * max_thrust
	for i in range(1, curve.size()):
		if command <= float(curve[i].command):
			var t := (command - float(curve[i-1].command)) / (float(curve[i].command) - float(curve[i-1].command))
			return lerpf(float(curve[i-1].thrust_N), float(curve[i].thrust_N), t)
	return max_thrust

func reaction_torque(static_force: float) -> float:
	if curve.is_empty() or not curve[0].has("torque_Nm"):
		return static_force * torque_ratio
	static_force = clampf(static_force, 0, max_thrust)
	for i in range(1, curve.size()):
		if static_force <= float(curve[i].thrust_N):
			var span := float(curve[i].thrust_N) - float(curve[i-1].thrust_N)
			var t := (static_force-float(curve[i-1].thrust_N))/span if span > 0 else 0.0
			return lerpf(float(curve[i-1].torque_Nm), float(curve[i].torque_Nm), t)
	return float(curve[-1].torque_Nm)

func evolve(commands: PackedFloat64Array, dt: float) -> PackedFloat64Array:
	# Return the midpoint speed for force integration; retain end-of-step speed.
	var midpoint := speed.duplicate()
	for i in range(4):
		var target := sqrt(static_thrust(commands[i]) / max_thrust)
		var tau := tau_up if target >= speed[i] else tau_down
		if response_model == "legacy_thrust":
			var old_squared := speed[i] * speed[i]
			midpoint[i] = sqrt(lerpf(old_squared, target*target, 1-exp(-dt*0.5/tau)))
			speed[i] = sqrt(lerpf(old_squared, target*target, 1-exp(-dt/tau)))
		else:
			midpoint[i] = lerpf(speed[i], target, 1-exp(-dt*0.5/tau))
			speed[i] = lerpf(speed[i], target, 1-exp(-dt/tau))
	return midpoint

func set_steady(commands: PackedFloat64Array) -> void:
	for i in range(4):
		speed[i] = sqrt(static_thrust(commands[i]) / max_thrust)

func reset() -> void:
	speed.fill(0)
