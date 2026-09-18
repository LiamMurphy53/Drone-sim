extends RefCounted
var device := -1
var device_name := "No radio connected"
var radio_mode := true
var calibrated := false
var mapping: Dictionary = {}
var values := {"roll":0.0, "pitch":0.0, "yaw":0.0, "throttle":0.0}
var keyboard_throttle := 0.0
var wizard_step := -1
var baseline: Array[float] = []
var low: Array[float] = []
var high: Array[float] = []
var pending: Dictionary = {}
var calibration_message := ""
const CHANNELS := ["roll", "pitch", "yaw", "throttle"]
const AXIS_COUNT := 8
const PROMPTS := ["Center roll, pitch and yaw. Put throttle fully DOWN. Then capture.",
	"Hold the ROLL stick fully RIGHT. Then capture.",
	"Center roll. Hold the PITCH stick fully FORWARD. Then capture.",
	"Center pitch. Hold the YAW stick fully RIGHT. Then capture.",
	"Center yaw. Hold THROTTLE fully UP. Then capture.",
	"Sweep BOTH sticks through their full range, then center them and lower throttle. Save calibration."]

func _init() -> void:
	refresh_device()

func refresh_device() -> void:
	var old := device
	var old_name := device_name
	var devices := Input.get_connected_joypads()
	device = -1
	for id in devices:
		var name := Input.get_joy_name(id).to_lower()
		if "pocket" in name or "radiomaster" in name or "edgetx" in name or "opentx" in name:
			device = id
			break
	if device < 0 and devices.size() == 1:
		device = devices[0]
	device_name = Input.get_joy_name(device) if device >= 0 else "No radio connected"
	if old != device or old_name != device_name:
		wizard_step = -1
		calibrated = false
		mapping = {}
		action_bindings = {}
		learning_action = ""
		actions_initialized = false
		if FileAccess.file_exists("user://controller.json"):
			var saved = JSON.parse_string(FileAccess.get_file_as_string("user://controller.json"))
			if saved is Dictionary and saved.get("name", "") == device_name:
				mapping = saved.get("mapping", {})
				action_bindings = saved.get("actions", {})
				calibrated = mapping.size() == 4

func raw_axes() -> Array[float]:
	var result: Array[float] = []
	for i in range(AXIS_COUNT):
		result.append(Input.get_joy_axis(device, i) if device >= 0 else 0.0)
	return result

func update(dt: float, enabled: bool) -> Dictionary:
	if wizard_step >= 0:
		var raw := raw_axes()
		for i in range(AXIS_COUNT):
			low[i] = minf(low[i], raw[i])
			high[i] = maxf(high[i], raw[i])
	values = {"roll":0.0, "pitch":0.0, "yaw":0.0, "throttle":0.0}
	if not enabled or wizard_step >= 0:
		return values
	if radio_mode:
		if device < 0 or not calibrated:
			return values
		for key in CHANNELS:
			var m: Dictionary = mapping[key]
			var raw := Input.get_joy_axis(device, int(m.axis))
			if key == "throttle":
				values[key] = clampf((raw - float(m.zero)) / (float(m.full) - float(m.zero)), 0, 1)
			else:
				var delta := raw - float(m.center)
				var extent: float = float(m.high) - float(m.center) if delta >= 0 else float(m.center) - float(m.low)
				var value := clampf(delta / maxf(extent, 0.1) * float(m.direction), -1, 1)
				values[key] = signf(value) * maxf(0, (absf(value) - 0.025) / 0.975)
	else:
		keyboard_throttle = clampf(keyboard_throttle + (key(KEY_W) - key(KEY_S)) * dt * 0.3, 0, 1)
		values = {"roll":(key(KEY_RIGHT) - key(KEY_LEFT)) * 0.35,
			"pitch":(key(KEY_UP) - key(KEY_DOWN)) * 0.35,
			"yaw":(key(KEY_D) - key(KEY_A)) * 0.35, "throttle":keyboard_throttle}
	return values

func key(code: int) -> float:
	return 1.0 if Input.is_physical_key_pressed(code) else 0.0

func ready_to_fly() -> bool:
	return not radio_mode or (device >= 0 and calibrated and wizard_step < 0)

func begin_calibration() -> void:
	if device < 0:
		calibration_message = "Connect the Pocket using its USB DATA port and choose USB Joystick."
		return
	learning_action = ""
	wizard_step = 0
	pending = {}
	baseline = raw_axes()
	low = baseline.duplicate()
	high = baseline.duplicate()
	calibration_message = PROMPTS[0]

func capture() -> void:
	if wizard_step < 0:
		begin_calibration()
		return
	var raw := raw_axes()
	if wizard_step == 0:
		baseline = raw.duplicate()
	elif wizard_step <= 4:
		var channel: String = CHANNELS[wizard_step - 1]
		var axis := -1
		var largest := 0.25
		for i in range(AXIS_COUNT):
			var used := false
			for entry in pending.values():
				if int(entry.axis) == i:
					used = true
			if not used and absf(raw[i] - baseline[i]) > largest:
				axis = i
				largest = absf(raw[i] - baseline[i])
		if axis < 0:
			calibration_message = "No clear movement detected. " + PROMPTS[wizard_step]
			return
		pending[channel] = {"axis":axis, "center":baseline[axis], "zero":baseline[axis],
			"full":raw[axis], "direction":signf(raw[axis] - baseline[axis])}
	else:
		for channel in CHANNELS:
			var m: Dictionary = pending[channel]
			m.low = low[int(m.axis)]
			m.high = high[int(m.axis)]
			if float(m.high) - float(m.low) < 0.5:
				calibration_message = "Sweep both sticks fully; insufficient travel on " + channel
				return
			if channel != "throttle" and (float(m.center) - float(m.low) < 0.2 or float(m.high) - float(m.center) < 0.2):
				calibration_message = "Move " + channel + " fully in BOTH directions before saving."
				return
		mapping = pending.duplicate(true)
		var file := FileAccess.open("user://controller.json", FileAccess.WRITE)
		if file == null:
			calibration_message = "Could not save calibration. Check the app's storage permissions."
			return
		file.store_string(JSON.stringify({"name":device_name, "mapping":mapping, "actions":action_bindings}))
		calibrated = true
		wizard_step = -1
		calibration_message = "Saved. Verify the live stick bars, lower throttle, then arm."
		return
	wizard_step += 1
	calibration_message = PROMPTS[wizard_step]

var action_bindings: Dictionary = {}
var learning_action := ""
var learning_axes: Array[float] = []
var learning_buttons: Array[bool] = []
var action_states: Dictionary = {}
var actions_initialized := false

func save_profile() -> void:
	var file := FileAccess.open("user://controller.json",FileAccess.WRITE)
	if file:
		file.store_string(JSON.stringify({"name":device_name,"mapping":mapping,"actions":action_bindings}))

func learn_action(action: String) -> void:
	if device < 0 or not calibrated:
		calibration_message = "Connect and calibrate your radio first."
		return
	learning_action = action
	learning_axes = raw_axes()
	learning_buttons.clear()
	for i in range(32):
		learning_buttons.append(Input.is_joy_button_pressed(device,i))
	calibration_message = "Now move the desired switch to its ON position for " + action + ". Keep sticks still."

func action_value(binding: Dictionary) -> bool:
	if binding.kind == "button":
		return Input.is_joy_button_pressed(device,int(binding.index)) == bool(binding.active)
	return (Input.get_joy_axis(device,int(binding.index))-float(binding.threshold))*float(binding.direction)>0

func poll_actions() -> Dictionary:
	var events := {}
	if not radio_mode or device < 0 or not calibrated:
		actions_initialized = false
		return events
	if learning_action != "":
		var binding := {}
		for i in range(32):
			var pressed := Input.is_joy_button_pressed(device,i)
			if pressed != learning_buttons[i]:
				binding = {"kind":"button","index":i,"active":pressed}
				break
		if binding.is_empty():
			var raw := raw_axes()
			for i in range(AXIS_COUNT):
				var is_stick := false
				for m in mapping.values():
					if int(m.axis) == i: is_stick = true
				if not is_stick and absf(raw[i]-learning_axes[i])>0.5:
					binding = {"kind":"axis","index":i,"threshold":(raw[i]+learning_axes[i])/2,"direction":signf(raw[i]-learning_axes[i])}
					break
		if not binding.is_empty():
			action_bindings[learning_action] = binding
			save_profile()
			calibration_message = "Bound " + learning_action + ". Return switch to OFF before flying."
			learning_action = ""
			actions_initialized = false
		return events
	for action in action_bindings:
		var active := action_value(action_bindings[action])
		if actions_initialized and active != bool(action_states.get(action,false)):
			events[action] = active
		action_states[action] = active
	actions_initialized = true
	return events
