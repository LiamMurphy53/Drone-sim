extends RefCounted
## Betaflight 4.5.2 SITL wire format, pinned to the bundled source.
## UDP 9001: uint16 count + 2 padding bytes + 16 float PWM values.
## UDP 9003: 18 little-endian doubles; UDP 9004: double + 16 uint16.
const Aircraft = preload("res://scripts/aircraft.gd")
const MOTOR_CUTOFF_THROTTLE := 0.01
const POWER_RETURN_SECONDS := 0.4
var motors := PackedFloat64Array([0, 0, 0, 0])
var raw_motors := PackedFloat64Array([0, 0, 0, 0])
var coasting := true
var powered_time := 0.0
var output := PacketPeerUDP.new()
var state := PacketPeerUDP.new()
var rc := PacketPeerUDP.new()
var msp := StreamPeerTCP.new()
var buffer := PackedByteArray()
var last_motor_us := 0
var last_status_us := 0
var last_poll_us := 0
var armed := false
var arming_flags := 0
var bind_error := OK
var timestamp := 0.0
var connected := false

func _init() -> void:
	bind_error = output.bind(9001, "127.0.0.1")
	state.set_dest_address("127.0.0.1", 9003)
	rc.set_dest_address("127.0.0.1", 9004)
	msp.connect_to_host("127.0.0.1", 5761)

func close() -> void:
	output.close()
	state.close()
	rc.close()
	msp.disconnect_from_host()

func update(aircraft, controls: Dictionary, arm_request: bool, dt: float) -> void:
	timestamp += dt
	while output.get_available_packet_count() > 0:
		var packet := output.get_packet()
		if packet.size() != 68 or packet.decode_u16(0) != 4:
			continue
		# Betaflight QUADX = RR, FR, RL, FL. Our model = FL, FR, RL, RR.
		var order := [3, 1, 2, 0]
		for i in range(4):
			var value := packet.decode_float(4 + order[i] * 4)
			raw_motors[i] = clampf((value - 1000.0) / 1000.0, 0, 1) if is_finite(value) else 0
		last_motor_us = Time.get_ticks_usec()
	connected = last_motor_us > 0 and Time.get_ticks_usec() - last_motor_us < 250000
	if not connected:
		raw_motors.fill(0)
		armed = false
	var throttle := clampf(float(controls.throttle), 0, 1)
	coasting = throttle <= MOTOR_CUTOFF_THROTTLE
	if coasting or not arm_request:
		powered_time = 0.0
	else:
		powered_time = minf(POWER_RETURN_SECONDS, powered_time + dt)
	if coasting:
		throttle = 0.0
	else:
		# Ramp only the return from motor cutoff. Send the ramp through the FC
		# so its controller sees the applied throttle, rather than masking its
		# outputs and winding up I. Cuts always bypass this ramp immediately.
		throttle *= smoothstep(0.0, POWER_RETURN_SECONDS, powered_time)
	motors = raw_motors.duplicate()
	# Honor a fresh stick cutoff immediately, even if the most recent UDP
	# motor packet predates it. Betaflight MOTOR_STOP + pid_at_min_throttle=OFF
	# also stops its outputs and resets I internally. Rotor coast-down remains
	# in the physical model; do not freeze motion or clear angular momentum.
	if coasting: motors.fill(0)
	var packet := PackedByteArray()
	packet.resize(144)
	var w: Vector3 = aircraft.omega
	var force: Vector3 = aircraft.specific_force
	var basis_flu: Basis = Aircraft.TO_FLU * Basis(aircraft.orientation) * Aircraft.TO_FLU.transposed()
	var q := basis_flu.get_rotation_quaternion()
	var p: Vector3 = aircraft.position
	var v: Vector3 = aircraft.velocity
	# The upstream bridge flips gyro Y/Z, but flips all accelerometer axes.
	# Internal axes are FLU: right roll, nose-down pitch, left yaw positive.
	# BF negates right-stick yaw in rc.c. Preserve that convention: post-bridge
	# rates [-old_y, old_x, old_z], consistent with the attitude quaternion.
	var values := [timestamp, -w.y, -w.x, -w.z, -force.y, -force.x, -force.z,
		q.w, q.x, q.y, q.z, -v.y, -v.x, -v.z, -p.y, -p.x, -p.z,
		101325.0 * pow(maxf(0.1, 1.0 - p.z / 44330.0), 5.255)]
	# 18 doubles: timestamp, gyro3, acceleration3, quaternion4, velocity3, position3, pressure.
	packet.resize(values.size() * 8)
	for i in range(values.size()):
		packet.encode_double(i * 8, values[i])
	state.put_packet(packet)
	var channels := PackedByteArray()
	channels.resize(40)
	channels.encode_double(0, timestamp)
	var inputs := [1500 + 500 * controls.roll, 1500 + 500 * controls.pitch,
		1000 + 1000 * throttle, 1500 + 500 * controls.yaw,
		2000 if arm_request else 1000]
	for i in range(16):
		channels.encode_u16(8 + i * 2, int(inputs[i]) if i < inputs.size() else 1000)
	rc.put_packet(channels)
	poll_status()

func poll_status() -> void:
	msp.poll()
	var now := Time.get_ticks_usec()
	if msp.get_status() == StreamPeerTCP.STATUS_NONE or msp.get_status() == StreamPeerTCP.STATUS_ERROR:
		if now - last_poll_us > 1000000:
			msp.disconnect_from_host()
			msp.connect_to_host("127.0.0.1", 5761)
			last_poll_us = now
		return
	if msp.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		return
	if now - last_poll_us > 100000:
		msp.put_data(PackedByteArray([36, 77, 60, 0, 150, 150])) # MSP_STATUS_EX
		last_poll_us = now
	if msp.get_available_bytes() > 0:
		var result := msp.get_data(msp.get_available_bytes())
		if result[0] == OK:
			buffer.append_array(result[1])
	while buffer.size() >= 6:
		if buffer[0] != 36 or buffer[1] != 77:
			buffer.remove_at(0)
			continue
		var length := int(buffer[3])
		if buffer.size() < length + 6:
			break
		var checksum := 0
		for i in range(3, length + 5):
			checksum ^= buffer[i]
		if checksum == buffer[length + 5] and buffer[2] == 62 and buffer[4] == 150 and length >= 11:
			armed = (buffer.decode_u32(5 + 6) & 1) != 0
			last_status_us = now
			# Fixed STATUS_EX header is 15 bytes, then flight-mode extension count.
			if length >= 21:
				var extra := int(buffer[5 + 15])
				if length >= 21 + extra:
					arming_flags = buffer.decode_u32(5 + 17 + extra)
		buffer = buffer.slice(length + 6)
	if now - last_status_us > 500000:
		armed = false

func arming_reason() -> String:
	if arming_flags & (1 << 9): return "Betaflight is starting. Wait a moment, then arm."
	if arming_flags & (1 << 12): return "Keep the drone still while the gyro calibrates."
	if arming_flags & (1 << 5): return "Betaflight stopped a runaway. Reset the flight."
	if arming_flags & (1 << 7): return "Lower throttle fully, disarm, then arm again."
	if arming_flags & (1 << 25): return "Turn the arm switch OFF, then try again."
	if arming_flags & ((1 << 1) | (1 << 2)): return "Waiting for a valid controller signal."
	if arming_flags != 0: return "Betaflight is not ready to arm. Reset and retry."
	return "Arming…"
