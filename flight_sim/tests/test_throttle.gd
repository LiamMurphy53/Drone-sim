extends SceneTree
## Real Betaflight regression: throttle chops immediately after attitude inputs,
## plus a commanded turn at zero throttle. No attitude/position resets in flight.
const Aircraft = preload("res://scripts/aircraft.gd")
const Link = preload("res://scripts/betaflight.gd")
var aircraft
var link
var elapsed := 0.0
var failures := 0
var last_second := -1
var unexpected_disarm := false
var lost_connection := false
var peak_rate := 0.0
var settling := [0.0, 0.0, 0.0]
var recovery_rate := 0.0
var low_throttle_roll := 0.0
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
	print("Throttle-transient test: ", aircraft.cfg.name, " at ", Engine.max_fps, " FPS")

func _physics_process(dt: float) -> bool:
	elapsed += dt
	var controls := {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}
	var arm := elapsed > 4 and elapsed < 28
	if elapsed >= 6 and elapsed < 10: controls.throttle = takeoff
	if elapsed >= 10 and elapsed < 28: controls.throttle = cruise
	# Act as a pilot leveling between maneuvers, through RC sticks only. An acro
	# controller holds rates, not attitude; the asymmetric frame needs steering
	# during takeoff. Stop this well before each maneuver and throughout every
	# low-throttle/settling window, so it cannot conceal the instability tested.
	if (elapsed >= 6 and elapsed < 11) or (elapsed >= 15 and elapsed < 16.5) or (elapsed >= 20 and elapsed < 21):
		var up_body: Vector3 = Basis(aircraft.orientation).transposed() * Vector3(0,0,1)
		controls.roll = clampf(-.5 * up_body.x, -.25, .25)
		controls.pitch = clampf(-.5 * up_body.y, -.25, .25)
	if elapsed >= 12 and elapsed < 12.25: controls.roll = .15
	if elapsed >= 12.25 and elapsed < 14.75: controls.throttle = 0.0
	if elapsed >= 17 and elapsed < 17.25: controls.pitch = .15
	if elapsed >= 17.25 and elapsed < 19.75: controls.throttle = .08
	if elapsed >= 22 and elapsed < 24.75: controls.throttle = 0.0
	# Verify genuine low-throttle control, rather than passing by freezing rates.
	if elapsed >= 22.2 and elapsed < 22.45: controls.roll = -.15
	link.update(aircraft, controls, arm, dt)
	var commands: PackedFloat64Array = link.motors if link.armed and arm else PackedFloat64Array([0,0,0,0])
	aircraft.step(commands, dt)
	if elapsed >= 8 and elapsed < 27.9:
		unexpected_disarm = unexpected_disarm or not link.armed
		lost_connection = lost_connection or not link.connected
		peak_rate = maxf(peak_rate, aircraft.omega.length())
		minimum_altitude = minf(minimum_altitude, aircraft.position.z)
	# Require < 0.2 rad/s over a half-second window about two seconds after
	# each transient; the provisional slow motor/PI model is not instantaneous.
	if elapsed >= 14.25 and elapsed < 14.75: settling[0] = maxf(settling[0], aircraft.omega.length())
	if elapsed >= 19.25 and elapsed < 19.75: settling[1] = maxf(settling[1], aircraft.omega.length())
	if elapsed >= 24.25 and elapsed < 24.75: settling[2] = maxf(settling[2], aircraft.omega.length())
	if elapsed >= 26.75 and elapsed < 27.25: recovery_rate = maxf(recovery_rate, aircraft.omega.length())
	if elapsed >= 22.25 and elapsed < 22.45: low_throttle_roll += aircraft.omega.y * dt
	if int(elapsed) != last_second:
		last_second = int(elapsed)
		print("t=",last_second," throttle=",controls.throttle," armed=",link.armed," z=",aircraft.position.z," omega=",aircraft.omega," motors=",link.motors)
	if elapsed > 29:
		print("Peak rate: ",peak_rate,"; settled rates at low throttle: ",settling,"; recovery rate: ", recovery_rate, "; zero-throttle roll: ",low_throttle_roll)
		check(not lost_connection, "Continuous controller connection")
		check(not unexpected_disarm, "No unexpected disarming during throttle chops")
		check(not aircraft.crashed and minimum_altitude > 1, "Maneuvers remain airborne without contact")
		check(peak_rate < 5, "Throttle transients do not cause runaway rotation")
		for i in range(settling.size()): check(settling[i] < .2, "Rotation settles during low-throttle interval %d" % (i+1))
		check(recovery_rate < .2, "Rotation settles after throttle is restored")
		check(low_throttle_roll > .01, "Roll control remains effective at zero throttle")
		check(not link.armed, "Disarms on request after low-throttle flight")
		print("Throttle test failures: ",failures)
		link.close()
		quit(1 if failures else 0)
	return false

func check(ok: bool, label: String) -> void:
	if ok: print("PASS ", label)
	else:
		push_error("FAIL " + label)
		failures += 1
