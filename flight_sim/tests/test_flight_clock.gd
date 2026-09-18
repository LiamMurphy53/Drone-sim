extends SceneTree
const FlightClock = preload("res://scripts/flight_clock.gd")
var clock := FlightClock.new()
var ticks := 0
var failures := 0
var wrong_step := false
var stall_next := false
var stalled_at := 0
var after_stall_gap := 0

func _initialize() -> void:
	call_deferred("run")

func tick(dt: float) -> void:
	ticks += 1
	wrong_step = wrong_step or dt != .002
	if stalled_at > 0 and after_stall_gap == 0:
		after_stall_gap = Time.get_ticks_usec() - stalled_at
	if stall_next:
		stall_next = false
		OS.delay_msec(30)
		stalled_at = Time.get_ticks_usec()

func run() -> void:
	check(clock.start(tick) == OK, "Flight worker starts")
	# Simulate a blocked renderer. A frame-driven loop would execute no ticks.
	OS.delay_msec(120)
	clock.mutex.lock()
	check(ticks >= 30, "Flight updates continue while the main thread is blocked")
	stall_next = true
	clock.mutex.unlock()
	OS.delay_msec(80)
	clock.stop()
	check(not wrong_step, "Plant integration keeps its fixed 2 ms step")
	check(after_stall_gap >= 1000, "A flight-worker stall does not cause a catch-up packet burst")
	var stopped_ticks := ticks
	OS.delay_msec(10)
	check(ticks == stopped_ticks, "Stopping joins the worker before shared state is destroyed")
	print("Flight clock failures: ", failures)
	quit(1 if failures else 0)

func check(ok: bool, label: String) -> void:
	if ok: print("PASS ", label)
	else:
		push_error("FAIL " + label)
		failures += 1
