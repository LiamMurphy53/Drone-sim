extends SceneTree
const Aircraft = preload("res://scripts/aircraft.gd")
const Catalog = preload("res://scripts/aircraft_catalog.gd")
var failures := 0
var count := 0

func check(ok: bool, message: String) -> void:
	count += 1
	if ok: print("PASS ",message)
	else:
		push_error("FAIL "+message)
		failures += 1

func _initialize() -> void:
	call_deferred("run")

func run() -> void:
	var entries := Catalog.entries()
	check(entries.size()==2 and entries[0].name=="GoPro Drone" and entries[1].name=="DJI FPV", "Both named aircraft are registered")
	check(entries[0].curve_path!=entries[1].curve_path and entries[0].physics_path!=entries[1].physics_path, "Each aircraft owns separate propulsion measurements and physics settings")
	var dji = Aircraft.new(false,"dji_fpv")
	check(absf(dji.mass-.795)<1e-8 and absf(dji.cfg.propeller_radius_m-.0673)<1e-8, "DJI mass and propeller diameter match the cited specifications")
	check(absf((dji.arms[0]-dji.arms[3]).length()-.245)<1e-6, "Estimated motor layout preserves DJI's published diagonal")
	check(dji.inertia.determinant()>0 and dji.inertia.x.x>0 and dji.inertia.y.y>0, "DJI estimated inertia is positive definite")
	dji.ground_effect_enabled = false
	dji.drag_enabled = false
	dji.position.z = 20
	var hover: float = dji.mass*dji.cfg.gravity/(4*dji.propulsion.max_thrust)
	var commands := PackedFloat64Array([hover,hover,hover,hover])
	dji.prime_motors(commands)
	for i in range(2500): dji.step(commands,.002)
	check(absf(dji.position.z-20)<.001 and dji.omega.length()<.001, "DJI symmetric model maintains an unforced trimmed hover")
	dji.reset()
	for i in range(2500): dji.step(PackedFloat64Array([0,0,0,0]),.002)
	check(not dji.crashed and absf(dji.position.z-dji.clearance)<1e-6 and dji.omega.length()<1e-5, "DJI rests stably on its own landing footprint")
	var invalid = Aircraft.new(false,"missing")
	check(invalid.configuration_error!="", "An unknown profile raises an arming-blocking configuration error")
	var scene = load("res://main.tscn").instantiate()
	root.add_child(scene)
	scene.flight_clock.mutex.lock()
	scene.aircraft.position.z = 5
	scene.flight_clock.mutex.unlock()
	# Exercise the actual scene callback with its renderer/main thread stalled.
	OS.delay_msec(120)
	scene.flight_clock.stop()
	check(scene.rate_steps >= 30 and scene.aircraft.position.z < 4.99, "Scene physics continues during a stalled frame")
	scene.set_process(false)
	scene.set_physics_process(false)
	scene.select_aircraft(0,false)
	scene.arm_request = true
	scene.aircraft.position.z = 20
	scene.aircraft.velocity = Vector3(3,4,5)
	scene.aircraft.omega = Vector3.ONE
	scene.pilot.keyboard_throttle = .8
	scene.link.motors.fill(.8)
	scene.select_aircraft(1,false)
	check(scene.aircraft.profile_id=="dji_fpv" and scene.aircraft_select.selected==1, "Dropdown changes the active physics model and selection together")
	check(not scene.arm_request and scene.pilot.keyboard_throttle==0 and scene.link.motors[0]==0, "Changing aircraft clears arming, keyboard throttle, and stale motor commands")
	check(scene.aircraft.velocity==Vector3.ZERO and scene.aircraft.omega==Vector3.ZERO and scene.aircraft.propulsion.speed[0]==0, "Changing aircraft resets motion and rotor state")
	check(absf(scene.aircraft.position.z-scene.aircraft.clearance)<1e-6 and scene.reset_until>Time.get_ticks_msec(), "Changing aircraft returns to its launch height with a disarm interval")
	check("Approximate" in scene.aircraft_description.text, "DJI approximation is identified in the flight interface")
	var children: int = scene.drone.get_child_count()
	for i in range(5):
		scene.select_aircraft(0,false)
		scene.select_aircraft(1,false)
	check(scene.drone.get_child_count()==children, "Repeated switches rebuild visuals without accumulating meshes")
	scene.select_aircraft(0,false)
	check(scene.aircraft.cfg.name=="GoPro Drone" and absf(scene.aircraft.mass-.808819)<1e-8, "Switching back restores the original GoPro model")
	scene.free()
	print("Aircraft profile checks: ",count,"; failures: ",failures)
	quit(1 if failures else 0)
