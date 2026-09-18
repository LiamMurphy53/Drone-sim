extends SceneTree
## Actual Betaflight and motor-driven motion with zero collective throttle.
## Each isolated direction starts from rest in the air. The separate throttle
## scenario exercises uninterrupted takeoff, coast and powered recovery.
const Aircraft = preload("res://scripts/aircraft.gd")
const Link = preload("res://scripts/betaflight.gd")
const CASE_SECONDS := 2.2
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
var raw_coast_peak := 0.0
var applied_coast_peak := 0.0
var passive_error := 0.0
var passive_attitude_matches := true
var peak_rate := 0.0
var coast_started := 0.0
var before_counter := 0.0
var after_counter := 0.0
var response_delay := -1.0
var early_rotation := 0.0
var rate_at_100ms := 0.0
var lost_arm := false
var lost_connection := false
var checked_cases := 0
var failures := 0

func _initialize() -> void:
	var arguments := OS.get_cmdline_user_args()
	aircraft = Aircraft.new(false, arguments[0] if not arguments.is_empty() else "gopro_drone")
	link = Link.new()
	Engine.max_fps = int(arguments[1]) if arguments.size() > 1 else 60
	print("Zero-throttle steering: ", aircraft.cfg.name, " at ", Engine.max_fps, " FPS")

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
	check(before_counter > .1 and after_counter < before_counter - .2, CASES[case_index][2] + " responds to countersteering at zero throttle")
	check(not aircraft.crashed and aircraft.position.z > 1, CASES[case_index][2] + " stays airborne")
	checked_cases += 1

func _physics_process(dt: float) -> bool:
	elapsed += dt
	var controls := {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}
	var end_time := 6.0 + CASE_SECONDS * CASES.size()
	var arm := elapsed > 4 and elapsed < end_time
	var testing := elapsed >= 6 and elapsed < end_time
	var phase := 0.0
	if testing:
		var next_case := mini(int((elapsed - 6) / CASE_SECONDS), CASES.size() - 1)
		if next_case != case_index:
			if case_index >= 0: finish_case()
			case_index = next_case
			# Independent initial condition, never an in-maneuver correction.
			aircraft = Aircraft.new(false, aircraft.profile_id)
			aircraft.position = Vector3(0, 0, 30)
			passive = null
			response = 0.0
			motor_peak = 0.0
			before_counter = 0.0
			after_counter = 0.0
			response_delay = -1.0
			early_rotation = 0.0
			rate_at_100ms = 0.0
		phase = elapsed - 6 - case_index * CASE_SECONDS
		controls.throttle = [0.0, .005, .01][case_index % 3]
		if phase >= .4 and phase < 1.0:
			controls[CASES[case_index][0]] = .35 * CASES[case_index][1]
		# Release to coast, then oppose the existing rotation using the sticks.
		if phase >= 1.3 and phase < 1.6:
			controls[CASES[case_index][0]] = -.35 * CASES[case_index][1]
			passive = null
		if ((phase >= 1.0 and phase < 1.3) or phase >= 1.6) and passive == null:
			coast_started = elapsed
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
		lost_arm = lost_arm or not link.armed
		lost_connection = lost_connection or not link.connected
		peak_rate = maxf(peak_rate, aircraft.omega.length())
		var rates := {"roll":-aircraft.omega.y, "pitch":aircraft.omega.x, "yaw":-aircraft.omega.z}
		if phase >= .4 and phase < 1.0:
			var signed_rate: float = rates[CASES[case_index][0]] * CASES[case_index][1]
			if response_delay < 0 and signed_rate >= .2: response_delay = phase - .4
			if phase < .55: early_rotation += signed_rate * dt
			if phase < .5: rate_at_100ms = signed_rate
		if phase >= .5 and phase < 1.0:
			response += rates[CASES[case_index][0]] * CASES[case_index][1] * dt
			for value in commands: motor_peak = maxf(motor_peak, value)
		if phase >= 1.2 and phase < 1.3: before_counter = rates[CASES[case_index][0]] * CASES[case_index][1]
		if phase >= 1.5 and phase < 1.6: after_counter = rates[CASES[case_index][0]] * CASES[case_index][1]
		if (phase >= 1.0 and phase < 1.3) or phase >= 1.6:
			passive.step(PackedFloat64Array([0,0,0,0]), dt)
			passive_error = maxf(passive_error, (aircraft.position-passive.position).length() + (aircraft.velocity-passive.velocity).length() + (aircraft.omega-passive.omega).length())
			passive_attitude_matches = passive_attitude_matches and aircraft.orientation.is_equal_approx(passive.orientation)
			for value in commands: applied_coast_peak = maxf(applied_coast_peak, value)
			if elapsed - coast_started >= .2:
				for value in link.raw_motors: raw_coast_peak = maxf(raw_coast_peak, value)
	if not testing and case_index >= 0 and checked_cases < CASES.size(): finish_case()
	if elapsed > end_time + 1:
		print("Coast raw/applied: ", raw_coast_peak, " / ", applied_coast_peak, "; passive error: ", passive_error, "; peak rate: ", peak_rate)
		check(checked_cases == 6, "All six directions exercised")
		check(not lost_connection and not lost_arm, "Connection and arming survive zero-throttle steering")
		check(applied_coast_peak == 0.0, "Centering sticks immediately releases motor authority")
		check(raw_coast_peak < .000001, "Betaflight itself stops correcting after sticks are centered")
		check(passive_error < .00001 and passive_attitude_matches, "Releasing steering preserves a physical coast with no attitude correction")
		check(peak_rate < 3, "Moderate steering remains bounded on every axis")
		check(not link.armed, "Disarms on request")
		print("Zero-throttle steering failures: ", failures)
		link.close()
		quit(1 if failures else 0)
	return false

func check(ok: bool, label: String) -> void:
	if ok: print("PASS ", label)
	else:
		push_error("FAIL " + label)
		failures += 1
