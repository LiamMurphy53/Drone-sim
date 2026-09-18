extends SceneTree
const FlightClock = preload("res://scripts/flight_clock.gd")
var flight_clock := FlightClock.new()
var finished := false
## Large-command tracking, reversal and release through native Betaflight.
## Independent airborne fixtures avoid ground impacts concealing rate errors.
const Aircraft = preload("res://scripts/aircraft.gd")
const Link = preload("res://scripts/betaflight.gd")
const CASE_SECONDS := 4.5
const START_TIME := 10.0
const CASE_COUNT := 14
const DIRECTIONS := [Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1), Vector3(.8,-.8,.8)]
var aircraft
var link
var elapsed := 0.0
var case_index := -1
var failures := 0
var checked := 0
var error_squared := 0.0
var samples := 0
var settling_peak := 0.0
var peak_rate := 0.0
var missed_link := false
var missed_arm := false
var packet_age_peak := 0.0
var target := Vector3.ZERO
var cruise := .28
var worst_error := 0.0
var rapid_min := 0.0
var rapid_max := 0.0
var peak_phase := 0.0
var peak_axes := Vector3.ZERO

func _initialize() -> void:
	var args := OS.get_cmdline_user_args()
	aircraft = Aircraft.new(false, args[0] if not args.is_empty() else "gopro_drone")
	if aircraft.profile_id == "dji_fpv": cruise = .46
	link = Link.new()
	Engine.max_fps = int(args[1]) if args.size() > 1 else 60
	print("Aggressive control test: ", aircraft.cfg.name, " at ", Engine.max_fps, " FPS")
	call_deferred("start_clock")

func expected_rate(stick: Vector3) -> Vector3:
	# Pinned v6 Betaflight rate curve, independently compared with body motion.
	var rate := Vector3.ZERO
	for axis in range(3):
		var s := stick[axis]
		rate[axis] = deg_to_rad(200.0 * (1.0 if axis < 2 else .9) * s * (.95 + .05 * pow(absf(s), 3)) / (1.0 - absf(s) * (.6 if axis < 2 else .5)))
	return rate

func finish_case() -> void:
	var error := sqrt(error_squared / maxf(samples, 1)) / maxf(target.length(), .01)
	print("CASE ", case_index, " throttle group=", int(case_index / 4), " axis=", case_index % 4,
		" tracking error=", error, " settling=", settling_peak, " peak=", peak_rate, " at=", peak_phase,
		" peak RPY=", peak_axes, " motor age ms=", packet_age_peak)
	if case_index < 12:
		check(samples > 100 and error < .30, "Large inputs and reversals track commanded rates, case " + str(case_index))
	else:
		check(rapid_min < -.3 and rapid_max > .3, "Rapid reversals produce motion in both directions, case " + str(case_index))
	check(peak_rate < 1.5 * target.length() + .25, "Large inputs avoid excessive rate overshoot, case " + str(case_index))
	check(settling_peak < .5, "Rotation settles after release at any throttle, case " + str(case_index))
	check(not aircraft.crashed, "Maneuver remains airborne, case " + str(case_index))
	worst_error = maxf(worst_error, error)
	checked += 1

func _flight_step(dt: float) -> bool:
	if finished: return false
	elapsed += dt
	var end_time := START_TIME + CASE_COUNT * CASE_SECONDS
	var testing := elapsed >= START_TIME and elapsed < end_time
	var arm := elapsed > 4.5 and elapsed < end_time
	var controls := {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}
	var phase := 0.0
	var stick := Vector3.ZERO
	# Establish real powered flight first, including Betaflight's normal
	# takeoff checks, before the independent airborne stress fixtures.
	if elapsed >= 5 and elapsed < START_TIME:
		controls.throttle = cruise
		var up: Vector3 = Basis(aircraft.orientation).transposed() * Vector3(0,0,1)
		controls.roll = clampf(-.5 * up.x, -.25, .25)
		controls.pitch = clampf(-.5 * up.y, -.25, .25)
	if testing:
		var next_case := mini(int((elapsed - START_TIME) / CASE_SECONDS), CASE_COUNT - 1)
		if next_case != case_index:
			if case_index >= 0: finish_case()
			case_index = next_case
			aircraft = Aircraft.new(false, aircraft.profile_id)
			# Full-throttle inverted holds can descend hundreds of metres. Keep
			# ground contact out of this rate-control test, not out of the plant.
			aircraft.position = Vector3(0,0,1000)
			error_squared = 0.0
			samples = 0
			settling_peak = 0.0
			peak_rate = 0.0
			peak_phase = 0.0
			peak_axes = Vector3.ZERO
			packet_age_peak = 0.0
			rapid_min = 0.0
			rapid_max = 0.0
		phase = elapsed - START_TIME - case_index * CASE_SECONDS
		if phase >= .6: controls.throttle = cruise if case_index >= 12 else [0.0, cruise, 1.0][int(case_index / 4)]
		if phase >= 1.2 and phase < 2.0: stick = DIRECTIONS[case_index % 4]
		if phase >= 2.0 and phase < 2.8: stick = -DIRECTIONS[case_index % 4]
		if case_index >= 12:
			var direction: Vector3 = DIRECTIONS[0 if case_index == 12 else 3]
			target = expected_rate(direction)
			if phase >= 1.2 and phase < 2.8:
				stick = direction * (1.0 if int((phase - 1.2) / .125) % 2 == 0 else -1.0)
		controls.roll = stick.x
		controls.pitch = stick.y
		controls.yaw = stick.z
	link.update(aircraft, controls, arm, dt)
	var commands: PackedFloat64Array = link.motors if link.armed and arm else PackedFloat64Array([0,0,0,0])
	aircraft.step(commands, dt)
	if testing:
		if not link.armed and not missed_arm: print("ARM LOST at ", elapsed, " flags=", link.arming_flags)
		missed_arm = missed_arm or not link.armed
		missed_link = missed_link or not link.connected
		packet_age_peak = maxf(packet_age_peak, (Time.get_ticks_usec() - link.last_motor_us) / 1000.0)
		var rates: Vector3 = Vector3(-aircraft.omega.y, aircraft.omega.x, -aircraft.omega.z)
		if rates.length() > peak_rate:
			peak_rate = rates.length()
			peak_phase = phase
			peak_axes = rates
		if case_index < 12 and ((phase >= 1.7 and phase < 2.0) or (phase >= 2.5 and phase < 2.8)):
			target = expected_rate(stick)
			error_squared += (rates - target).length_squared()
			samples += 1
		if case_index >= 12 and phase >= 1.2 and phase < 2.8:
			var projected := rates.dot(target.normalized())
			rapid_min = minf(rapid_min, projected)
			rapid_max = maxf(rapid_max, projected)
		if phase >= 4.0: settling_peak = maxf(settling_peak, aircraft.omega.length())
	if not testing and case_index >= 0 and checked < CASE_COUNT: finish_case()
	if elapsed > end_time + 1:
		check(checked == CASE_COUNT, "All high-command maneuvers completed")
		check(not missed_arm and not missed_link, "No loss of controller connection or arming")
		check(not link.armed, "Disarms on request")
		print("Worst relative tracking error: ", worst_error, "; aggressive test failures: ", failures)
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
