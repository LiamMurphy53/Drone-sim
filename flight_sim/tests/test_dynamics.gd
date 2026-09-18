extends SceneTree
const Aircraft = preload("res://scripts/aircraft.gd")
const Propulsion = preload("res://scripts/propulsion.gd")
var OFF := PackedFloat64Array([0,0,0,0])
var FULL := PackedFloat64Array([1,1,1,1])
var failures := 0
var checks := 0

func check(ok: bool, message: String) -> void:
	checks += 1
	if ok: print("PASS ", message)
	else:
		push_error("FAIL " + message)
		failures += 1

func _initialize() -> void:
	test_motors()
	test_aerodynamics()
	test_ground_effect()
	test_integration()
	test_contacts()
	print("Dynamics checks: ", checks, "; failures: ", failures)
	quit(1 if failures else 0)

func test_motors() -> void:
	var a = Aircraft.new(false)
	var p = a.propulsion
	check(p.error == "", "Default propulsion configuration is valid")
	p.evolve(FULL, .05)
	check(absf(p.max_thrust * p.speed[0]*p.speed[0] - 13.5*pow(1-exp(-1),2)) < 1e-9, "Motor start evolves rotor speed, with squared-speed thrust")
	p.set_steady(FULL)
	p.evolve(OFF, .05)
	check(absf(p.max_thrust * p.speed[0]*p.speed[0] - 13.5*exp(-2)) < 1e-9, "Motor coast-down retains decaying thrust")
	p.tau_down = .1
	p.set_steady(FULL)
	p.evolve(OFF, .05)
	check(absf(p.speed[0]-exp(-.5)) < 1e-9, "Separate spin-down time is honored")
	check(absf(p.static_thrust(.37)-13.5*.37)<1e-9, "Unmeasured steady thrust preserves the legacy mapping")
	p.reset()
	for i in range(500): p.evolve(PackedFloat64Array([.1,.3,.6,1]), .002)
	check(absf(p.speed[2]*p.speed[2]-.6)<1e-8 and p.speed[3]<=1, "Rotor response converges without overshoot")
	var curve := {"schema_version":1,"source":"test fixture only","points":[
		{"command":0.0,"thrust_N":0.0,"torque_Nm":0.0},
		{"command":0.5,"thrust_N":4.0,"torque_Nm":0.08},
		{"command":1.0,"thrust_N":16.0,"torque_Nm":0.2}]}
	check(p.load_curve(curve)=="" and p.max_thrust==16, "Measured thrust replaces the provisional maximum")
	check(absf(p.static_thrust(.75)-10)<1e-9 and absf(p.static_thrust(.25)-2)<1e-9, "Measured thrust uses interpolation between neighboring readings")
	check(absf(p.reaction_torque(10)-.14)<1e-9, "Measured shaft torque follows the instantaneous rotor state")
	var invalid := curve.duplicate(true)
	invalid.points[1].thrust_N = -1
	check(p.load_curve(invalid)!="" and p.static_thrust(.75)==10, "Invalid measurements cannot partially replace a working curve")
	for point in curve.points: point.erase("torque_Nm")
	p.load_curve(curve)
	check(absf(p.reaction_torque(10)-.13)<1e-9, "Absent torque data retains the explicitly provisional Q/T estimate")
	var bad = Propulsion.new(a.cfg,{"spin_up_tau_s":0},false)
	check(bad.error!="", "Invalid motor response time is rejected")

func test_aerodynamics() -> void:
	var a = Aircraft.new(false)
	# Nonzero pressure center also exercises its rotational velocity and moment.
	a.physics.aerodynamics.center_of_pressure_relative_to_cg_m = [.02,-.01,.03]
	var maximum_power := -INF
	for v in [Vector3(8,-3,2), Vector3(-1,6,-4), Vector3.ZERO]:
		for w in [Vector3(2,-4,3), Vector3(-5,1,-2), Vector3.ZERO]:
			var loads: Array[Vector3] = a.aerodynamic_wrench(v,w,PackedFloat64Array([.2,.7,.5,1]))
			maximum_power = maxf(maximum_power, loads[0].dot(v)+loads[1].dot(w))
	check(maximum_power<1e-6, "Body and rotor drag never add mechanical energy in still air")
	a.physics.aerodynamics.center_of_pressure_relative_to_cg_m = [0,0,0]
	var edge: Array[Vector3] = a.aerodynamic_wrench(Vector3(0,10,0),Vector3.ZERO,OFF)
	var belly: Array[Vector3] = a.aerodynamic_wrench(Vector3(0,0,10),Vector3.ZERO,OFF)
	check(belly[0].length()>edge[0].length()*3, "Broadside motion experiences more body drag than edge-on motion")
	var slow: Array[Vector3] = a.aerodynamic_wrench(Vector3(0,5,0),Vector3.ZERO,OFF)
	check(absf(edge[0].length()/slow[0].length()-4)<1e-5, "Body drag increases quadratically with airspeed")
	var spinning: Array[Vector3] = a.aerodynamic_wrench(Vector3.ZERO,Vector3(0,0,3),FULL)
	var stopped: Array[Vector3] = a.aerodynamic_wrench(Vector3.ZERO,Vector3(0,0,3),OFF)
	check(spinning[1].z<stopped[1].z and spinning[1].z<0, "Rotor-local airflow resists rotation even with no CG translation")
	a.wind_world = Vector3(4,-3,0)
	var moving: Array[Vector3] = a.wrench(Vector3(0,0,20),Basis.IDENTITY,a.wind_world,Vector3.ZERO,OFF)
	check(moving[0].length()<1e-8, "Matching wind velocity eliminates translational air loads")
	var still: Array[Vector3] = a.wrench(Vector3(0,0,20),Basis.IDENTITY,Vector3.ZERO,Vector3.ZERO,OFF)
	check(still[0].dot(a.wind_world)>0, "A stationary aircraft is pushed in the wind direction")

func test_ground_effect() -> void:
	var a = Aircraft.new(false)
	var near: float = a.ground_multiplier(0,Vector3(0,0,.12),Basis.IDENTITY)
	var far: float = a.ground_multiplier(0,Vector3(0,0,20),Basis.IDENTITY)
	var clipped: float = a.ground_multiplier(0,Vector3.ZERO,Basis.IDENTITY)
	check(near>far and near>1 and far<1.00001, "Ground effect decays with each rotor's height")
	check(clipped<=1.2 and clipped>=1, "Ground effect remains bounded close to the surface")
	check(a.ground_multiplier(0,Vector3(0,0,.12),Basis(Vector3.RIGHT,PI))==1, "Inverted rotors receive no ground-effect boost")
	var tilted := Basis(Vector3.UP,.3)
	check(absf(a.ground_multiplier(0,Vector3(0,0,.15),tilted)-a.ground_multiplier(1,Vector3(0,0,.15),tilted))>.001, "Tilt gives low and high rotors different ground proximity")
	a.drag_enabled = false
	var close: Array[Vector3] = a.wrench(Vector3(0,0,.12),Basis.IDENTITY,Vector3.ZERO,Vector3.ZERO,PackedFloat64Array([.5,0,0,0]))
	var high: Array[Vector3] = a.wrench(Vector3(0,0,20),Basis.IDENTITY,Vector3.ZERO,Vector3.ZERO,PackedFloat64Array([.5,0,0,0]))
	check(close[0].z>high[0].z and absf(close[1].z-high[1].z)<1e-8, "Ground effect increases lift without inventing extra reaction torque")

func trajectory(dt: float):
	var a = Aircraft.new(false)
	a.ground_contact_enabled = false
	a.ground_effect_enabled = false
	a.position = Vector3(0,0,50)
	a.velocity = Vector3(8,-4,2)
	a.omega = Vector3(.3,-.2,.5)
	var command := PackedFloat64Array([.16,.17,.15,.14])
	for i in range(roundi(.5/dt)): a.step(command,dt)
	return a

func test_integration() -> void:
	var a = Aircraft.new(false)
	a.drag_enabled = false
	a.ground_effect_enabled = false
	a.ground_contact_enabled = false
	a.position.z = 1000
	a.omega = Vector3(2,-3,5)
	var initial_energy: float = .5*a.omega.dot(a.inertia*a.omega)
	var initial_momentum: Vector3 = a.inertia*a.omega
	for i in range(5000): a.step(OFF,.002)
	var energy: float = .5*a.omega.dot(a.inertia*a.omega)
	var momentum: Vector3 = Basis(a.orientation)*(a.inertia*a.omega)
	check(absf(energy-initial_energy)/initial_energy<.001, "Ten-second torque-free rotation conserves rotational energy")
	check((momentum-initial_momentum).length()/initial_momentum.length()<.001, "Ten-second torque-free rotation conserves world angular momentum")
	var coarse = trajectory(.004)
	var nominal = trajectory(.002)
	var fine = trajectory(.001)
	var coarse_error: float = (coarse.velocity-fine.velocity).length()+(coarse.omega-fine.omega).length()
	var nominal_error: float = (nominal.velocity-fine.velocity).length()+(nominal.omega-fine.omega).length()
	check(nominal_error<coarse_error*.5, "Halving the integration step reduces trajectory error")
	check((nominal.position-fine.position).length()<.001 and nominal.orientation.angle_to(fine.orientation)<.002, "500 Hz and 1000 Hz trajectories agree within 1 mm and 0.12 degrees over the test maneuver")

func test_contacts() -> void:
	var a = Aircraft.new(false)
	for i in range(2500): a.step(OFF,.002)
	check(not a.crashed and absf(a.position.z-Aircraft.CLEARANCE)<1e-6 and a.omega.length()<1e-5, "Five-second ground rest stays still without artificial leveling")
	check(absf(a.specific_force.length()-a.cfg.gravity)<.001, "Ground contact supports weight without spurious accelerometer pulses")
	a.reset()
	a.position.z = .2
	a.velocity = Vector3(.4,0,-.3)
	for i in range(1000): a.step(OFF,.002)
	check(not a.crashed and a.contact and a.velocity.length()<.001, "A gentle sliding landing settles through normal impulses and friction")
	a.reset()
	a.velocity = Vector3(2,1,-.5)
	var impact_energy: float = .5*a.mass*a.velocity.length_squared()
	a.solve_ground()
	var remaining_energy: float = .5*a.mass*a.velocity.length_squared()+.5*a.omega.dot(a.inertia*a.omega)
	check(remaining_energy<=impact_energy+1e-6, "Inelastic sliding contact does not add kinetic energy")
	a.reset()
	a.position.z = .125
	a.velocity.z = -4
	a.step(OFF,.002)
	check(a.crashed, "Hard landing is detected from contact-point impact speed")
	a.reset()
	a.orientation = Quaternion(Vector3.RIGHT,PI)
	a.position.z = 1
	for i in range(500): a.step(OFF,.002)
	check(a.crashed and a.position.z>-.05, "Inverted ground collision is detected at the rotor disks")
