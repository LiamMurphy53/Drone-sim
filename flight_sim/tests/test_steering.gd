extends SceneTree
const FlightClock = preload("res://scripts/flight_clock.gd")
var flight_clock := FlightClock.new()
var finished := false
## Actual Betaflight and motor-driven motion with zero collective throttle.
## Each airborne fixture steers, then releases to brake through real motors.
## The passive reference proves this is active braking, not natural drag.
const Aircraft = preload("res://scripts/aircraft.gd")
const Link = preload("res://scripts/betaflight.gd")
const CASE_SECONDS := 2.6
const START_TIME := 10.0
const CASES := [
	["roll", 1.0, "right roll"], ["roll", -1.0, "left roll"],
	["pitch", -1.0, "pitch up"], ["pitch", 1.0, "pitch down"],
	["yaw", 1.0, "right yaw"], ["yaw", -1.0, "left yaw"]]
var aircraft
var passive
var link
var elapsed := 0.0
var case_index := -1
var response := 0.0
var motor_peak := 0.0
var braking_motor_peak := 0.0
var settling_rate := 0.0
var peak_rate := 0.0
var response_delay := -1.0
var early_rotation := 0.0
var rate_at_100ms := 0.0
var lost_arm := false
var lost_connection := false
var checked_cases := 0
var failures := 0
var cruise := .28

func _initialize() -> void:
	var arguments := OS.get_cmdline_user_args()
	aircraft = Aircraft.new(false, arguments[0] if not arguments.is_empty() else "gopro_drone")
	if aircraft.profile_id == "dji_fpv": cruise = .46
	link = Link.new()
	Engine.max_fps = int(arguments[1]) if arguments.size() > 1 else 60
	print("Zero-throttle steering and release braking: ", aircraft.cfg.name, " at ", Engine.max_fps, " FPS")
	call_deferred("start_clock")

func finish_case() -> void:
	print(CASES[case_index][2], ": signed rotation=", response, " rad; motor peak=", motor_peak)
	print("RESPONSE ", JSON.stringify({"axis":CASES[case_index][2], "delay_ms":response_delay * 1000, "rotation_150ms_deg":rad_to_deg(early_rotation), "rate_100ms_deg_s":rad_to_deg(rate_at_100ms)}))
	check(response > .03, CASES[case_index][2] + " responds in the requested direction without throttle")
	# Observable onset and early motion catch a return to the soft v5 stick
	# curve. These limits are for the provisional models and a 35% stick step.
	var yaw_axis: bool = CASES[case_index][0] == "yaw"
	var max_delay := .17 if yaw_axis else .10
	var minimum_rotation := .48 if yaw_axis else 2.0
	if aircraft.profile_id == "gopro_drone":
		max_delay = .11 if yaw_axis else .08
		minimum_rotation = 1.4 if yaw_axis else (3.2 if case_index == 2 else 5.2)
	check(response_delay >= 0 and response_delay < max_delay, CASES[case_index][2] + " begins promptly after stick movement")
	check(rad_to_deg(early_rotation) > minimum_rotation, CASES[case_index][2] + " has a firm response within 150 ms")
	check(motor_peak > .01, CASES[case_index][2] + " is produced by actual motor commands")
	print("BRAKE ", CASES[case_index][2], " settled=", settling_rate, " passive=", passive.omega.length(), " motors=", braking_motor_peak)
	check(settling_rate < .2, CASES[case_index][2] + " stops rotating after centering at zero throttle")
	check(passive.omega.length() > .3 and aircraft.omega.length() < passive.omega.length() * .2, CASES[case_index][2] + " brakes substantially faster than a passive coast")
	check(braking_motor_peak > .01, CASES[case_index][2] + " brakes using real motor outputs")
	if not yaw_axis:
		var up: Vector3 = Basis(aircraft.orientation) * Vector3(0,0,1)
		check(up.angle_to(Vector3(0,0,1)) > .1, CASES[case_index][2] + " stops rotation without auto-leveling")
	check(not aircraft.crashed and aircraft.position.z > 1, CASES[case_index][2] + " stays airborne")
	checked_cases += 1

func _flight_step(dt: float) -> bool:
	if finished: return false
	elapsed += dt
	var controls := {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}
	var end_time := START_TIME + CASE_SECONDS * CASES.size()
	var arm := elapsed > 4.5 and elapsed < end_time
	var testing := elapsed >= START_TIME and elapsed < end_time
	var phase := 0.0
	if elapsed >= 5 and elapsed < START_TIME:
		controls.throttle = cruise
		var up: Vector3 = Basis(aircraft.orientation).transposed() * Vector3(0,0,1)
		controls.roll = clampf(-.5 * up.x, -.25, .25)
		controls.pitch = clampf(-.5 * up.y, -.25, .25)
	if testing:
		var next_case := mini(int((elapsed - START_TIME) / CASE_SECONDS), CASES.size() - 1)
		if next_case != case_index:
			if case_index >= 0: finish_case()
			case_index = next_case
			# Independent initial condition, never an in-maneuver correction.
			aircraft = Aircraft.new(false, aircraft.profile_id)
			aircraft.position = Vector3(0, 0, 60)
			passive = null
			response = 0.0
			motor_peak = 0.0
			braking_motor_peak = 0.0
			settling_rate = 0.0
			response_delay = -1.0
			early_rotation = 0.0
			rate_at_100ms = 0.0
		phase = elapsed - START_TIME - case_index * CASE_SECONDS
		# Reset controller trim with the physical fixture. AirMode retains I
		# while armed, so teleporting just the plant would inject stale trim.
		arm = phase >= .4
		controls.throttle = [0.0, .005, .01][case_index % 3]
		if phase >= .8 and phase < 1.4:
			controls[CASES[case_index][0]] = .35 * CASES[case_index][1]
		if phase >= 1.4 and passive == null:
			passive = Aircraft.new(false, aircraft.profile_id)
			passive.position = aircraft.position
			passive.velocity = aircraft.velocity
			passive.orientation = aircraft.orientation
			passive.omega = aircraft.omega
			passive.propulsion.speed = aircraft.propulsion.speed.duplicate()
			passive.contact = aircraft.contact
	link.update(aircraft, controls, arm, dt)
	var commands: PackedFloat64Array = link.motors if link.armed and arm else PackedFloat64Array([0,0,0,0])
	aircraft.step(commands, dt)
	if testing:
		if phase >= .7:
			if not link.armed and not lost_arm: print("ARM LOST at ", elapsed, " flags=", link.arming_flags)
			lost_arm = lost_arm or not link.armed
		lost_connection = lost_connection or not link.connected
		peak_rate = maxf(peak_rate, aircraft.omega.length())
		var rates := {"roll":-aircraft.omega.y, "pitch":aircraft.omega.x, "yaw":-aircraft.omega.z}
		if phase >= .8 and phase < 1.4:
			# Include the initial motor impulse, not just the later steady hold.
			for value in commands: motor_peak = maxf(motor_peak, value)
			var signed_rate: float = rates[CASES[case_index][0]] * CASES[case_index][1]
			if response_delay < 0 and signed_rate >= .2: response_delay = phase - .8
			if phase < .95: early_rotation += signed_rate * dt
			if phase < .9: rate_at_100ms = signed_rate
		if phase >= .9 and phase < 1.4:
			response += rates[CASES[case_index][0]] * CASES[case_index][1] * dt
		if phase >= 1.4:
			passive.step(PackedFloat64Array([0,0,0,0]), dt)
			for value in commands: braking_motor_peak = maxf(braking_motor_peak, value)
		if phase >= 1.9: settling_rate = maxf(settling_rate, aircraft.omega.length())
	if not testing and case_index >= 0 and checked_cases < CASES.size(): finish_case()
	if elapsed > end_time + 1:
		check(checked_cases == 6, "All six directions exercised")
		check(not lost_connection and not lost_arm, "Connection and arming survive zero-throttle steering and braking")
		check(peak_rate < 3, "Moderate steering remains bounded on every axis")
		check(not link.armed, "Disarms on request")
		print("Zero-throttle steering failures: ", failures)
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
