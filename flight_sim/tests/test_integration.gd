extends SceneTree
const Aircraft = preload("res://scripts/aircraft.gd")
const Link = preload("res://scripts/betaflight.gd")
var aircraft = Aircraft.new()
var link
var elapsed := 0.0
var armed_seen := false
var peak_altitude := 0.0
var roll_response := 0.0
var pitch_response := 0.0
var yaw_response := 0.0
var unexpected_disarm := false
var max_rate := 0.0
var settling_rate := 0.0
var failures := 0
var last_second := -1
var flight_throttle := .28
func _initialize() -> void:
	var arguments := OS.get_cmdline_user_args()
	if not arguments.is_empty(): aircraft = Aircraft.new(true,arguments[0])
	if aircraft.profile_id == "dji_fpv": flight_throttle = .46
	print("Testing aircraft: ",aircraft.cfg.name)
	link = Link.new()
	Engine.max_fps = int(arguments[1]) if arguments.size() > 1 else 60
func _physics_process(dt: float) -> bool:
	elapsed += dt
	var controls := {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}
	var arm := elapsed > 4 and elapsed < 14
	# Maintain altitude margin through the open-loop attitude maneuvers and the
	# final second of motor-off fall; this is not an altitude-hold controller.
	if elapsed > 6 and elapsed < 14: controls.throttle = flight_throttle
	if elapsed > 10 and elapsed < 10.25: controls.roll = 0.15
	if elapsed > 11 and elapsed < 11.25: controls.pitch = 0.15
	if elapsed > 12 and elapsed < 12.25: controls.yaw = 0.2
	link.update(aircraft,controls,arm,dt)
	var commands: PackedFloat64Array = link.motors if link.armed and arm else PackedFloat64Array([0,0,0,0])
	aircraft.step(commands,dt)
	armed_seen = armed_seen or link.armed
	peak_altitude = maxf(peak_altitude,aircraft.position.z)
	# Integrate the signed INITIAL response while the stick is still held.
	# A later oscillatory rebound must never make a reversed axis pass.
	if elapsed > 10.05 and elapsed < 10.25: roll_response += -aircraft.omega.y*dt
	if elapsed > 11.05 and elapsed < 11.25: pitch_response += aircraft.omega.x*dt
	if elapsed > 12.05 and elapsed < 12.25: yaw_response += -aircraft.omega.z*dt
	if elapsed > 7 and elapsed < 13.9:
		unexpected_disarm = unexpected_disarm or not link.armed
		max_rate = maxf(max_rate,aircraft.omega.length())
	if elapsed > 13.5 and elapsed < 13.9:
		settling_rate = maxf(settling_rate,aircraft.omega.length())
	if int(elapsed) != last_second:
		last_second = int(elapsed)
		print("t=",last_second," linked=",link.connected," armed=",link.armed," flags=",link.arming_flags," z=",aircraft.position.z," omega=",aircraft.omega," motors=",link.motors)
	if elapsed > 15:
		print("Initial signed rotations (rad): ",roll_response,", ",pitch_response,", ",yaw_response,"; settling rate: ",settling_rate)
		check(link.connected,"Live motor packets from native Betaflight")
		check(armed_seen,"Betaflight armed through radio channels")
		check(peak_altitude>1,"Aircraft took off under Betaflight control")
		check(roll_response>0.01,"Right roll command initially rotates right")
		check(pitch_response>0.01,"Forward pitch command initially pitches nose down")
		check(yaw_response>0.01,"Right yaw command initially turns right")
		check(not unexpected_disarm,"No runaway or unexpected disarming during flight")
		check(max_rate<5,"Small stick inputs remain controlled")
		check(settling_rate<.2,"Rotation settles after the sticks are centered")
		check(not aircraft.crashed,"Flight ends without a crash")
		check(not link.armed,"Betaflight disarmed on request")
		print("Integration failures: ",failures)
		link.close()
		quit(1 if failures else 0)
	return false
func check(ok: bool, text: String) -> void:
	if ok: print("PASS ",text)
	else:
		push_error("FAIL "+text)
		failures+=1
