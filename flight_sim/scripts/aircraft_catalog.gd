extends RefCounted
const DEFAULT_ID := "gopro_drone"

static func entries() -> Array:
	return JSON.parse_string(FileAccess.get_file_as_string("res://config/aircraft_catalog.json"))

static func find(profile_id: String) -> Dictionary:
	for entry in entries():
		if entry.id == profile_id:
			return entry
	return {}

static func saved_id() -> String:
	var selection := ConfigFile.new()
	if selection.load("user://aircraft-selection.cfg") == OK:
		var candidate := str(selection.get_value("aircraft", "id", DEFAULT_ID))
		if not find(candidate).is_empty(): return candidate
	return DEFAULT_ID

static func save_id(profile_id: String) -> Error:
	if find(profile_id).is_empty(): return ERR_INVALID_PARAMETER
	var selection := ConfigFile.new()
	selection.set_value("aircraft", "id", profile_id)
	return selection.save("user://aircraft-selection.cfg")
