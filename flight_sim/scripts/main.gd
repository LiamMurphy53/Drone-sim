extends Node3D
const Aircraft = preload("res://scripts/aircraft.gd")
const Pilot = preload("res://scripts/pilot.gd")
const Link = preload("res://scripts/betaflight.gd")
const Field = preload("res://scripts/field.gd")
const Catalog = preload("res://scripts/aircraft_catalog.gd")
const DroneVisual = preload("res://scripts/drone_visual.gd")
var aircraft = Aircraft.new()
var pilot = Pilot.new()
var link
var field
var drone := DroneVisual.new()
var camera := Camera3D.new()
var arm_request := false
var paused := false
var focused := true
var chase := false
var camera_tilt := 15.0
var flight_time := 0.0
var next_gate := 0
var laps := 0
var notice := "Lower throttle, then press Space to arm."
var notice_until := 0
var reset_until := 0
var ui_clock := 0.0
var hud: Label
var status_label: Label
var notice_label: Label
var radio_label: Label
var calibration_label: Label
var axis_label: Label
var settings: PanelContainer
var input_select: OptionButton
var capture_button: Button
var telemetry: Label
var sim_rate_label: Label
var aircraft_select: OptionButton
var aircraft_description: Label
var profiles := Catalog.entries()
var rate_steps := 0
var rate_start := 0
var controls := {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}

func _ready() -> void:
	aircraft = Aircraft.new(true,Catalog.saved_id())
	field = Field.new()
	add_child(field)
	build_drone()
	camera.near = 0.025
	camera.far = 1800
	camera.fov = 100
	add_child(camera)
	build_ui()
	update_aircraft_label()
	link = Link.new()
	if aircraft.configuration_error != "":
		message(aircraft.configuration_error)
	if link.bind_error != OK:
		message("Another simulator is using the motor port. Close it and relaunch.")
	Input.joy_connection_changed.connect(_joy_changed)
	call_deferred("rescan_controller")
	rate_start = Time.get_ticks_msec()

func build_drone() -> void:
	if drone.get_parent() == null: add_child(drone)
	drone.build(aircraft)

func select_aircraft(index: int, persist := true) -> void:
	if index < 0 or index >= profiles.size(): return
	arm_request = false
	aircraft = Aircraft.new(true,profiles[index].id)
	reset_flight()
	if link != null: link.motors.fill(0)
	build_drone()
	aircraft_select.select(index)
	update_aircraft_label()
	if persist and Catalog.save_id(aircraft.profile_id) != OK:
		message("Aircraft selected, but the preference could not be saved.")
	elif aircraft.configuration_error != "":
		message(aircraft.configuration_error)
	else:
		message("%s selected. Lower throttle, then arm." % aircraft.cfg.name)

func update_aircraft_label() -> void:
	var entry := Catalog.find(aircraft.profile_id)
	aircraft_description.text = "%.0f g  ·  %s" % [aircraft.mass*1000,entry.description]
	DisplayServer.window_set_title("Flight Lab — " + str(aircraft.cfg.name))

func _physics_process(dt: float) -> void:
	if link == null:
		return
	rate_steps += 1
	var usable: bool = focused and not paused and not aircraft.crashed and Time.get_ticks_msec() > reset_until
	controls = pilot.update(dt, usable)
	var actions: Dictionary = pilot.poll_actions()
	if actions.has("arm"):
		if not actions.arm:
			arm_request = false
		elif usable and not settings.visible and not arm_request:
			toggle_arm()
	if actions.get("reset",false):
		reset_flight()
	if not usable or not pilot.ready_to_fly():
		arm_request = false
	link.update(aircraft, controls, arm_request, dt)
	if not link.connected and arm_request:
		arm_request = false
		message("Betaflight connection lost. Disarmed; wait for reconnection.")
	var commands := PackedFloat64Array([0,0,0,0])
	if usable and arm_request and link.connected and link.armed:
		commands = link.motors
	if not paused and focused:
		var previous: Vector3 = Aircraft.TO_GODOT * aircraft.position
		aircraft.step(commands, dt)
		var current: Vector3 = Aircraft.TO_GODOT * aircraft.position
		if field.hits(previous, current):
			aircraft.crashed = true
		if aircraft.crashed:
			arm_request = false
		if link.armed:
			flight_time += dt
			check_gate(previous,current)

func check_gate(previous: Vector3, current: Vector3) -> void:
	var gate: Vector3 = field.gates[next_gate]
	if (previous.z-gate.z)*(current.z-gate.z) < 0:
		var t := (gate.z-previous.z)/(current.z-previous.z)
		var crossing := previous.lerp(current,t)
		if absf(crossing.x-gate.x)<2.0 and crossing.y>0.3 and crossing.y<3.8:
			next_gate += 1
			message("Gate cleared")
			if next_gate >= field.gates.size():
				next_gate = 0
				laps += 1
				message("Circuit complete")

func _process(dt: float) -> void:
	if link == null:
		return
	drone.transform = aircraft.render_transform()
	drone.visible = chase
	camera.fov = 75 if chase else 100
	if chase:
		var desired: Vector3 = drone.global_position + drone.global_basis * Vector3(0,.65,1.8)
		camera.global_position = camera.global_position.lerp(desired,1-exp(-8*dt))
		camera.look_at(drone.global_position + Vector3(0,0.1,0),Vector3.UP)
	else:
		camera.global_transform = drone.global_transform * Transform3D(Basis(Vector3.RIGHT,deg_to_rad(camera_tilt)),Vector3(0,0.025,-0.07))
	ui_clock += dt
	if ui_clock < 0.05:
		return
	ui_clock = 0
	var state := "DISARMED"
	if not link.connected:
		state = "CONNECTING TO BETAFLIGHT"
	elif aircraft.crashed:
		state = "CRASHED  ·  R TO RESET"
	elif paused:
		state = "PAUSED  ·  ESC TO RESUME"
	elif not focused:
		state = "WINDOW INACTIVE  ·  DISARMED"
	elif link.armed and arm_request:
		state = "ARMED  /  ACRO"
	elif arm_request:
		state = "WAITING TO ARM"
	status_label.text = state
	status_label.modulate = Color("e9af73") if arm_request or aircraft.crashed else Color("c9ded2")
	hud.text = "%04.1f m\nALTITUDE\n\n%04.1f m/s\nSPEED\n\n%02d:%02d\nFLIGHT TIME" % [maxf(0,aircraft.position.z-aircraft.clearance),aircraft.velocity.length(),int(flight_time)/60,int(flight_time)%60]
	telemetry.text = "GATE  %02d / 05     CIRCUITS  %02d\nTHROTTLE  %03d%%    %s\nFL %.0f%%   FR %.0f%%   RL %.0f%%   RR %.0f%%" % [next_gate+1,laps,controls.throttle*100,"CHASE" if chase else "FPV",link.motors[0]*100,link.motors[1]*100,link.motors[2]*100,link.motors[3]*100]
	var now := Time.get_ticks_msec()
	if now-rate_start > 1000:
		if pilot.refresh_device(): arm_request = false
		sim_rate_label.text = "%d FPS   ·   %.0f Hz PHYSICS   ·   BETAFLIGHT 4.5.2" % [Engine.get_frames_per_second(),rate_steps*1000.0/(now-rate_start)]
		rate_steps = 0
		rate_start = now
	if aircraft.crashed:
		notice_label.text = "Impact detected. Press R to return to the launch pad."
	elif now < notice_until:
		notice_label.text = notice
	elif not pilot.ready_to_fly():
		notice_label.text = "Connect and calibrate your Pocket, or select keyboard controls."
	elif arm_request and not link.armed:
		notice_label.text = link.arming_reason()
	elif link.arming_flags != 0 and not arm_request:
		notice_label.text = link.arming_reason()
	elif not arm_request:
		notice_label.text = "Lower throttle · Space to arm · Tab for controller setup"
	else:
		notice_label.text = ""
	radio_label.text = pilot.device_name + ("  /  CALIBRATED" if pilot.calibrated else "")
	calibration_label.text = pilot.calibration_message
	capture_button.text = "Save calibration" if pilot.wizard_step == 5 else ("Capture position" if pilot.wizard_step >= 0 else "Calibrate Pocket")
	var raw: Array = pilot.raw_axes()
	axis_label.text = "ROLL %+.2f   PITCH %+.2f   YAW %+.2f   THR %.2f\n" % [controls.roll,controls.pitch,controls.yaw,controls.throttle]
	for i in range(raw.size()):
		axis_label.text += "%d: %+.2f  " % [i,raw[i]]

func _unhandled_key_input(event: InputEvent) -> void:
	if not event.is_pressed() or event.is_echo():
		return
	match event.physical_keycode:
		KEY_SPACE: toggle_arm()
		KEY_R: reset_flight()
		KEY_C: chase = not chase
		KEY_TAB:
			settings.visible = not settings.visible
			if settings.visible:
				arm_request = false
		KEY_ESCAPE:
			paused = not paused
			arm_request = false
		KEY_F:
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED if DisplayServer.window_get_mode() == DisplayServer.WINDOW_MODE_FULLSCREEN else DisplayServer.WINDOW_MODE_FULLSCREEN)

func toggle_arm() -> void:
	if arm_request:
		arm_request = false
		return
	if aircraft.configuration_error != "":
		message(aircraft.configuration_error)
		return
	if Time.get_ticks_msec() < reset_until:
		message("Resetting the aircraft. Wait a moment before arming.")
		return
	if not link.connected or not pilot.ready_to_fly() or aircraft.crashed or paused or not focused:
		message("Connect Betaflight and finish controller setup before arming.")
		return
	if controls.throttle > 0.03:
		message("Lower throttle fully before arming.")
		return
	arm_request = true
	settings.visible = false

func reset_flight() -> void:
	arm_request = false
	pilot.keyboard_throttle = 0
	aircraft.reset()
	controls = {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0}
	reset_until = Time.get_ticks_msec()+1000
	flight_time = 0
	next_gate = 0
	laps = 0
	message("Reset to launch pad. Lower throttle, then arm.")

func _notification(what: int) -> void:
	if what == NOTIFICATION_APPLICATION_FOCUS_OUT:
		focused = false
		arm_request = false
	elif what == NOTIFICATION_APPLICATION_FOCUS_IN:
		focused = true

func _joy_changed(_device: int, _connected: bool) -> void:
	arm_request = false
	pilot.refresh_device()
	message("Controller changed. Verify calibration before arming.")

func rescan_controller() -> void:
	arm_request = false
	pilot.refresh_device()
	var names: Array[String] = []
	for id in Input.get_connected_joypads(): names.append(Input.get_joy_name(id))
	print("Controller scan: ",names)
	if pilot.device < 0:
		message("No USB radio detected by the simulator. Check the data cable and USB Joystick mode.")
	else:
		message(pilot.device_name + " connected. Calibrate before flying.")

func message(text: String) -> void:
	notice = text
	notice_until = Time.get_ticks_msec()+5000

func label(text: String, size := 16) -> Label:
	var l := Label.new()
	l.text = text
	l.add_theme_font_size_override("font_size",size)
	l.add_theme_color_override("font_color",Color("e8eee6"))
	return l

func panel() -> PanelContainer:
	var p := PanelContainer.new()
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.045,0.082,0.09,0.92)
	style.set_corner_radius_all(12)
	style.set_content_margin_all(20)
	p.add_theme_stylebox_override("panel",style)
	return p

func button(text: String, action: Callable) -> Button:
	var b := Button.new()
	b.text = text
	b.custom_minimum_size.y = 38
	b.focus_mode = Control.FOCUS_NONE
	b.pressed.connect(action)
	return b

func build_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	var root := Control.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	layer.add_child(root)
	var top := panel()
	top.position = Vector2(24,24)
	top.custom_minimum_size = Vector2(500,0)
	root.add_child(top)
	var title := VBoxContainer.new()
	title.add_theme_constant_override("separation",6)
	top.add_child(title)
	title.add_child(label("F L I G H T   L A B   /   O P E N   F I E L D",20))
	var model_row := HBoxContainer.new()
	model_row.add_theme_constant_override("separation",12)
	title.add_child(model_row)
	model_row.add_child(label("DRONE",13))
	aircraft_select = OptionButton.new()
	aircraft_select.custom_minimum_size = Vector2(290,36)
	aircraft_select.focus_mode = Control.FOCUS_NONE
	for index in range(profiles.size()):
		aircraft_select.add_item(profiles[index].name)
		if profiles[index].id == aircraft.profile_id: aircraft_select.select(index)
	aircraft_select.item_selected.connect(select_aircraft)
	model_row.add_child(aircraft_select)
	aircraft_description = label("",12)
	aircraft_description.custom_minimum_size.x = 460
	aircraft_description.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	title.add_child(aircraft_description)
	status_label = label("CONNECTING TO BETAFLIGHT",15)
	title.add_child(status_label)
	var left := panel()
	left.position = Vector2(24,250)
	root.add_child(left)
	hud = label("",22)
	left.add_child(hud)
	var bottom := panel()
	root.add_child(bottom)
	bottom.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_WIDE)
	bottom.offset_left = 24
	bottom.offset_right = -24
	bottom.offset_top = -139
	bottom.offset_bottom = -24
	var rows := VBoxContainer.new()
	rows.add_theme_constant_override("separation",6)
	bottom.add_child(rows)
	notice_label = label("",16)
	notice_label.modulate = Color("f2b97e")
	rows.add_child(notice_label)
	rows.add_child(label("SPACE  arm / disarm      R  reset      C  camera      TAB  setup      ESC  pause      F  fullscreen",14))
	sim_rate_label = label("",12)
	sim_rate_label.modulate = Color("9cb4ae")
	rows.add_child(sim_rate_label)
	var right := panel()
	root.add_child(right)
	right.set_anchors_and_offsets_preset(Control.PRESET_TOP_RIGHT)
	right.offset_left = -414
	right.offset_right = -24
	right.offset_top = 24
	telemetry = label("",16)
	right.add_child(telemetry)
	var crosshair := label("+",22)
	root.add_child(crosshair)
	crosshair.set_anchors_and_offsets_preset(Control.PRESET_CENTER)
	crosshair.modulate = Color(1,1,1,0.65)
	settings = panel()
	root.add_child(settings)
	settings.set_anchors_and_offsets_preset(Control.PRESET_TOP_RIGHT)
	settings.offset_left = -514
	settings.offset_right = -24
	settings.offset_top = 170
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation",12)
	settings.add_child(box)
	box.add_child(label("PILOT SETUP",20))
	box.add_child(label("Pocket: USB data cable → USB Joystick",14))
	radio_label = label("",14)
	box.add_child(radio_label)
	input_select = OptionButton.new()
	input_select.add_item("RadioMaster Pocket / USB radio")
	input_select.add_item("Keyboard practice")
	input_select.focus_mode = Control.FOCUS_NONE
	input_select.item_selected.connect(func(index: int):
		arm_request = false
		pilot.radio_mode = index == 0
		pilot.keyboard_throttle = 0
		pilot.wizard_step = -1
		pilot.learning_action = ""
		pilot.calibration_message = ""
	)
	box.add_child(input_select)
	box.add_child(button("Rescan USB controllers",rescan_controller))
	capture_button = button("Calibrate Pocket",func():
		arm_request = false
		pilot.radio_mode = true
		input_select.select(0)
		pilot.capture()
	)
	box.add_child(capture_button)
	box.add_child(button("Restart calibration",func():
		arm_request = false
		pilot.begin_calibration()
	))
	calibration_label = label("",14)
	calibration_label.custom_minimum_size.x = 440
	calibration_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	box.add_child(calibration_label)
	var switch_row := HBoxContainer.new()
	box.add_child(switch_row)
	switch_row.add_child(button("Bind arm switch",func():
		arm_request = false
		pilot.learn_action("arm")
	))
	switch_row.add_child(button("Bind reset switch",func():
		arm_request = false
		pilot.learn_action("reset")
	))
	axis_label = label("",12)
	axis_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	axis_label.custom_minimum_size.x = 440
	box.add_child(axis_label)
	var tilt_label := label("FPV camera tilt  /  15°",14)
	box.add_child(tilt_label)
	var tilt := HSlider.new()
	tilt.min_value = 0
	tilt.max_value = 45
	tilt.value = camera_tilt
	tilt.focus_mode = Control.FOCUS_NONE
	tilt.value_changed.connect(func(v: float):
		camera_tilt = v
		tilt_label.text = "FPV camera tilt  /  %d°" % v
	)
	box.add_child(tilt)
	box.add_child(label("Keyboard: W/S throttle · arrows pitch/roll · A/D yaw\nAcro mode: centered sticks stop rotation, not motion.\nDirectional drag · rotor dynamics · ground effect",13))
	box.add_child(button("Return to flight",func(): settings.hide()))
