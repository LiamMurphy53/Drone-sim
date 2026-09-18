extends SceneTree
const FlightClock = preload("res://scripts/flight_clock.gd")
var flight_clock := FlightClock.new()
var finished := false
## Continuous airborne throttle chops, release braking and full-power punches.
const Aircraft = preload("res://scripts/aircraft.gd")
const Link = preload("res://scripts/betaflight.gd")
var aircraft
var link
var elapsed := 0.0
var failures := 0
var last_second := -1
var was_low := false
var low_started := 0.0
var low_intervals := 0
var low_settling_rate := 0.0
var punch_delays: Array[float] = []
var punch_started := 0.0
var was_punching := false
var punch_seen := false
var unexpected_disarm := false
var lost_connection := false
var peak_rate := 0.0
var peak_time := 0.0
var recovery_rate := 0.0
var powered_roll := 0.0
var minimum_altitude := INF
var takeoff := .42
var cruise := .28

func _initialize() -> void:
	var arguments := OS.get_cmdline_user_args()
	aircraft = Aircraft.new(false, arguments[0] if not arguments.is_empty() else "gopro_drone")
	if aircraft.profile_id == "dji_fpv":
		takeoff = .70
		cruise = .46
	link = Link.new()
	Engine.max_fps = int(arguments[1]) if arguments.size() > 1 else 60
	print("Immediate throttle and AirMode test: ", aircraft.cfg.name, " at ", Engine.max_fps, " FPS")
	call_deferred("start_clock")

func _flight_step(dt: float) -> bool:
	if finished: return false
	elapsed += dt
	var controls := {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}
	var arm := elapsed > 4.5 and elapsed < 29
	if elapsed >= 6 and elapsed < 10: controls.throttle = takeoff
	if elapsed >= 10 and elapsed < 29: controls.throttle = cruise
	# Pilot steering through normal RC channels establishes upright flight.
	# It is absent during low-throttle braking and the final settling window.
	if (elapsed >= 6 and elapsed < 11) or (elapsed >= 15 and elapsed < 18) or (elapsed >= 22.5 and elapsed < 25):
		var up_body: Vector3 = Basis(aircraft.orientation).transposed() * Vector3(0,0,1)
		controls.roll = clampf(-.5 * up_body.x, -.25, .25)
		controls.pitch = clampf(-.5 * up_body.y, -.25, .25)
	if elapsed >= 12.1 and elapsed < 12.5: controls.roll = .15
	if elapsed >= 19.6 and elapsed < 20: controls.pitch = .15
	var low := (elapsed >= 12.5 and elapsed < 14.5) or (elapsed >= 20 and elapsed < 22)
	if low:
		controls.throttle = 0.0 if elapsed < 13.5 else (.01 if elapsed < 14.5 else .005)
		if not was_low:
			low_started = elapsed
			low_intervals += 1
	var punching := (elapsed >= 14.5 and elapsed < 14.8) or (elapsed >= 22 and elapsed < 22.3)
	if punching:
		controls.throttle = 1.0
		controls.roll = 0.0
		controls.pitch = 0.0
		if not was_punching:
			punch_started = elapsed
			punch_seen = false
	if elapsed >= 25.5 and elapsed < 25.75: controls.roll = .15
	link.update(aircraft, controls, arm, dt)
	var commands: PackedFloat64Array = link.motors if link.armed and arm else PackedFloat64Array([0,0,0,0])
	aircraft.step(commands, dt)
	if low and elapsed - low_started >= 1.3:
		low_settling_rate = maxf(low_settling_rate, aircraft.omega.length())
	if punching and not punch_seen:
		var maximum := 0.0
		for motor in link.raw_motors: maximum = maxf(maximum, motor)
		if maximum >= .95:
			punch_delays.append(elapsed - punch_started)
			punch_seen = true
	was_low = low
	was_punching = punching
	if elapsed >= 8 and elapsed < 28.9:
		unexpected_disarm = unexpected_disarm or not link.armed
		lost_connection = lost_connection or not link.connected
		if aircraft.omega.length() > peak_rate:
			peak_rate = aircraft.omega.length()
			peak_time = elapsed
		minimum_altitude = minf(minimum_altitude, aircraft.position.z)
	if elapsed >= 25.55 and elapsed < 25.75: powered_roll += -aircraft.omega.y * dt
	if elapsed >= 28 and elapsed < 28.5: recovery_rate = maxf(recovery_rate, aircraft.omega.length())
	if int(elapsed) != last_second:
		last_second = int(elapsed)
		print("t=",last_second," throttle=",controls.throttle," armed=",link.armed," z=",aircraft.position.z," omega=",aircraft.omega," motors=",link.motors)
	if elapsed > 30:
		print("Low-throttle settling=", low_settling_rate, "; punch delays=", punch_delays, "; peak/recovery=", peak_rate, " at ", peak_time, " / ", recovery_rate)
		check(not lost_connection, "Continuous controller connection")
		check(not unexpected_disarm, "Throttle chops keep the armed state")
		check(not aircraft.crashed and minimum_altitude > 1, "Flight remains airborne without contact")
		check(low_intervals == 2 and low_settling_rate < .2, "Centering sticks brakes rotation during both low-throttle intervals")
		check(punch_delays.size() == 2, "Both throttle punches produce full-scale motor authority")
		if punch_delays.size() == 2:
			check(punch_delays[0] < .05 and punch_delays[1] < .05, "Native motor commands reach 95% within 50 ms of a full-throttle step")
		check(peak_rate < 5, "Throttle restoration stays bounded")
		check(powered_roll > .01, "Powered steering resumes after throttle is restored")
		check(recovery_rate < .2, "Rotation settles after powered steering")
		check(not link.armed, "Disarms on request")
		print("Immediate throttle and AirMode failures: ",failures)
		finished = true
		call_deferred("finish_test")
	return false

func check(ok: bool, label: String) -> void:
	if ok: print("PASS ", label)
	else:
		push_error("FAIL " + label)
		failures += 1

func start_clock() -> void:
	if flight_clock.start(_flight_step) != OK:
		push_error("Could not start the steady flight clock")
		quit(1)

func finish_test() -> void:
	flight_clock.stop()
	link.close()
	quit(1 if failures else 0)
