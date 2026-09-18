extends SceneTree
const Aircraft = preload("res://scripts/aircraft.gd")
var failures := 0
func check(condition: bool, text: String) -> void:
	if condition: print("PASS ",text)
	else:
		push_error("FAIL " + text)
		failures += 1
func _initialize() -> void:
	var a = Aircraft.new(false)
	a.ground_effect_enabled = false
	a.propulsion.response_model = "legacy_thrust"
	a.drag_enabled = false
	check(absf(a.mass-0.808819)<1e-8,"Original MATLAB mass")
	check(absf(a.cfg.max_thrust_N-13.5)<1e-8,"Original thrust limit")
	check(absf(a.inertia.x.x-0.002639)<1e-9,"Original inertia units")
	a.position.z = 100
	for i in range(500): a.step(PackedFloat64Array([0,0,0,0]),0.002)
	check(absf(a.velocity.z+9.80665)<0.001,"One-second free fall")
	check(absf(a.position.z-(100-9.80665/2))<0.01,"Free-fall position")
	check(a.specific_force.length()<0.001,"Accelerometer reads zero in free fall")
	a.reset()
	a.position.z = 20
	# Exact zero-moment trim follows CG offsets, including yaw reaction torque.
	var weight: float = a.mass*a.cfg.gravity
	var y_front: float = a.arms[0].y
	var y_rear: float = a.arms[2].y
	var front: float = weight*y_rear/(y_rear-y_front)
	var rear: float = weight-front
	var dx: float = a.cfg.cg_m[0]
	var difference: float = dx*weight/(0.106252+0.093252)
	var trim := PackedFloat64Array([(front+difference)/2,(front-difference)/2,(rear+difference)/2,(rear-difference)/2])
	var command := PackedFloat64Array()
	for value in trim: command.append(value/a.cfg.max_thrust_N)
	a.prime_motors(command)
	for i in range(2500): a.step(command,0.002)
	check(a.omega.length()<0.001,"Asymmetric frame holds trimmed hover without a controller")
	check(absf(a.position.z-20)<0.01,"Five-second hover maintains altitude")
	check(absf(a.orientation.length()-1)<1e-6,"Quaternion remains normalized")
	a.reset()
	a.position.z = 20
	a.step(PackedFloat64Array([1,0,0,0]),0.002)
	check(a.omega.x<0 and a.omega.y<0 and a.omega.z>0,"FL motor produces expected pitch, roll, and yaw signs")
	check(absf(a.thrust[0]-13.5*(1-exp(-.002/.05)))<1e-6,"Original 50 ms motor response")
	check(a.render_transform().basis.determinant()>0.999,"Rendering transform preserves handedness")
	a.reset()
	for i in range(100): a.step(PackedFloat64Array([0,0,0,0]),.002)
	check(not a.crashed and absf(a.position.z-Aircraft.CLEARANCE)<1e-6,"Ground support at rest")
	check(absf(a.specific_force.z-9.80665)<.001,"Supported accelerometer reads 1 g")
	a.position.z = 3
	for i in range(500): a.step(PackedFloat64Array([0,0,0,0]),.002)
	check(a.crashed,"Hard ground impact registers a crash")
	print("Physics failures: ", failures)
	quit(1 if failures else 0)
