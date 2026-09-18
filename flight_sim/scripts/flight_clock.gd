extends RefCounted
## Pace the custom plant and native-controller exchange independently of rendering.
## Callbacks may touch only protected data, never the active scene tree.
const STEP_SECONDS := 0.002
const STEP_US := 2000
var mutex := Mutex.new()
var thread := Thread.new()
var running := false
var callback: Callable

func start(step: Callable) -> Error:
	callback = step
	running = true
	var error := thread.start(_run)
	if error != OK: running = false
	return error

func stop() -> void:
	mutex.lock()
	running = false
	mutex.unlock()
	if thread.is_started(): thread.wait_to_finish()
	callback = Callable()

func _run() -> void:
	var deadline := Time.get_ticks_usec()
	while true:
		mutex.lock()
		if not running:
			mutex.unlock()
			break
		callback.call(STEP_SECONDS)
		mutex.unlock()
		deadline += STEP_US
		var now := Time.get_ticks_usec()
		# Do not flood the controller with catch-up sensor packets after a stall.
		if now > deadline + STEP_US: deadline = now + STEP_US
		if deadline > now: OS.delay_usec(deadline - now)
