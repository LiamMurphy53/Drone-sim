extends SceneTree
## Native Betaflight regression for centered-stick coast and power return.
## The reference plant follows the same physical state with zero motor commands.
const Aircraft = preload("res://scripts/aircraft.gd")
const Link = preload("res://scripts/betaflight.gd")
var aircraft
var passive
var link
var elapsed := 0.0
var failures := 0
var last_second := -1
var was_coasting := false
var coast_started := 0.0
var coast_intervals := 0
var off_command_peak := 0.0
var off_raw_peak := 0.0
var residual_thrust := 0.0
var passive_error := 0.0
var passive_attitude_matches := true
var coast_travel := 0.0
var coast_origin := Vector3.ZERO
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
	print("Centered-stick coast test: ", aircraft.cfg.name, " at ", Engine.max_fps, " FPS")

func start_passive_reference() -> void:
	passive = Aircraft.new(false, aircraft.profile_id)
	passive.position = aircraft.position
	passive.velocity = aircraft.velocity
	passive.orientation = aircraft.orientation
	passive.omega = aircraft.omega
	passive.propulsion.speed = aircraft.propulsion.speed.duplicate()
	passive.contact = aircraft.contact
	coast_origin = aircraft.position
	coast_started = elapsed
	coast_intervals += 1

func _physics_process(dt: float) -> bool:
	elapsed += dt
	var controls := {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}
	var arm := elapsed > 4 and elapsed < 29
	if elapsed >= 6 and elapsed < 10: controls.throttle = takeoff
	if elapsed >= 10 and elapsed < 29: controls.throttle = cruise
	# Pilot steering through normal RC channels establishes upright flight.
	# It is absent in both coast intervals and the final recovery window.
	if (elapsed >= 6 and elapsed < 11) or (elapsed >= 15 and elapsed < 18) or (elapsed >= 22.5 and elapsed < 25):
		var up_body: Vector3 = Basis(aircraft.orientation).transposed() * Vector3(0,0,1)
		controls.roll = clampf(-.5 * up_body.x, -.25, .25)
		controls.pitch = clampf(-.5 * up_body.y, -.25, .25)
	if elapsed >= 11 and elapsed < 11.25: controls.roll = .15
	if elapsed >= 19 and elapsed < 19.25: controls.pitch = .15
	var coasting := (elapsed >= 12.5 and elapsed < 14.5) or (elapsed >= 20 and elapsed < 22)
	if coasting:
		# Exercise exactly zero, the cutoff boundary, and endpoint jitter.
		controls.throttle = 0.0 if elapsed < 13.5 else (.01 if elapsed < 14.5 else .005)
		# Centered attitude sticks coast; zero-throttle steering is exercised
		# separately in test_steering.gd, including all six stick directions.
		if not was_coasting: start_passive_reference()
	if elapsed >= 25.5 and elapsed < 25.75: controls.roll = .15
	link.update(aircraft, controls, arm, dt)
	var commands: PackedFloat64Array = link.motors if link.armed and arm else PackedFloat64Array([0,0,0,0])
	aircraft.step(commands, dt)
	if coasting:
		passive.step(PackedFloat64Array([0,0,0,0]), dt)
		for motor in range(4):
			off_command_peak = maxf(off_command_peak, link.motors[motor])
			# Allow the controller's RX/UDP latency, independently checking that
			# its own outputs stop; a game-side mask alone must not pass this.
			if elapsed - coast_started > .12: off_raw_peak = maxf(off_raw_peak, link.raw_motors[motor])
			if elapsed - coast_started > .5: residual_thrust = maxf(residual_thrust, aircraft.thrust[motor])
		passive_error = maxf(passive_error, (aircraft.position-passive.position).length() + (aircraft.velocity-passive.velocity).length() + (aircraft.omega-passive.omega).length())
		passive_attitude_matches = passive_attitude_matches and aircraft.orientation.is_equal_approx(passive.orientation)
		coast_travel = maxf(coast_travel, (aircraft.position-coast_origin).length())
	was_coasting = coasting
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
		print("Off commands/raw: ",off_command_peak," / ",off_raw_peak,"; passive error: ",passive_error,"; residual thrust: ",residual_thrust,"; peak/recovery rates: ",peak_rate," at t=",peak_time," / ",recovery_rate)
		check(not lost_connection, "Continuous controller connection")
		check(not unexpected_disarm, "Throttle cutoff keeps the armed state")
		check(not aircraft.crashed and minimum_altitude > 1, "Flight remains airborne without contact")
		check(coast_intervals == 2 and off_command_peak == 0.0, "Zero throttle with centered sticks cuts commands immediately, including endpoint jitter")
		check(off_raw_peak < .000001, "Betaflight itself stops motors with throttle off and sticks centered")
		check(residual_thrust < .0001, "Rotors coast down without sustained thrust")
		check(passive_error < .00001 and passive_attitude_matches, "Throttle-off motion matches an uncontrolled physical coast")
		check(coast_travel > 1, "Coasting preserves free motion instead of freezing the drone")
		check(peak_rate < 5, "Throttle restoration stays bounded")
		check(powered_roll > .01, "Powered steering resumes after throttle is restored")
		check(recovery_rate < .2, "Rotation settles after powered steering")
		check(not link.armed, "Disarms on request")
		print("Centered-stick coast test failures: ",failures)
		link.close()
		quit(1 if failures else 0)
	return false

func check(ok: bool, label: String) -> void:
	if ok: print("PASS ", label)
	else:
		push_error("FAIL " + label)
		failures += 1
