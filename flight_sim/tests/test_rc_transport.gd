extends SceneTree
## Inspect the actual wire packet, including the first step after throttle zero.
const Aircraft = preload("res://scripts/aircraft.gd")
const Link = preload("res://scripts/betaflight.gd")
var failures := 0

func _initialize() -> void:
	call_deferred("run")

func receive(peer: PacketPeerUDP) -> PackedByteArray:
	var deadline := Time.get_ticks_usec() + 100000
	while peer.get_available_packet_count() == 0 and Time.get_ticks_usec() < deadline:
		OS.delay_usec(100)
	return peer.get_packet() if peer.get_available_packet_count() else PackedByteArray()

func run() -> void:
	var capture := PacketPeerUDP.new()
	if capture.bind(9004, "127.0.0.1") != OK:
		push_error("Close the simulator before running the RC transport test")
		quit(1)
		return
	var aircraft = Aircraft.new(false, "dji_fpv")
	var link = Link.new()
	check(link.bind_error == OK, "Motor port available")
	var controls := {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}
	for pair in [[0.0,1000],[1.0,2000],[0.5,1500],[0.0,1000],[1.0,2000],[.01,1000]]:
		controls.throttle = pair[0]
		link.update(aircraft, controls, true, .002)
		var packet := receive(capture)
		check(packet.size() == 40 and packet.decode_u16(12) == pair[1], "First packet sends throttle " + str(pair[0]) + " without a ramp")
	# Native braking output must survive even with every pilot stick centered.
	var native_output := PacketPeerUDP.new()
	native_output.set_dest_address("127.0.0.1", 9001)
	var packet := PackedByteArray()
	packet.resize(68)
	packet.encode_u16(0, 4)
	for i in range(4): packet.encode_float(4 + i * 4, 1400.0)
	native_output.put_packet(packet)
	OS.delay_msec(5)
	controls.throttle = 0.0
	link.update(aircraft, controls, true, .002)
	check(is_equal_approx(link.motors[0], .4), "Centered sticks preserve native zero-throttle braking output")
	link.close()
	capture.close()
	native_output.close()
	print("RC transport failures: ", failures)
	quit(1 if failures else 0)

func check(ok: bool, label: String) -> void:
	if ok: print("PASS ", label)
	else:
		push_error("FAIL " + label)
		failures += 1
