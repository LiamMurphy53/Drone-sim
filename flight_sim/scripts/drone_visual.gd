extends Node3D
## Original procedural silhouettes. Appearance is not a collision mesh or CAD model.

func part(shape: Mesh, color: Color, point := Vector3.ZERO) -> MeshInstance3D:
	var node := MeshInstance3D.new()
	node.mesh = shape
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.roughness = .7
	if color.a < 1:
		mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	node.material_override = mat
	node.position = point
	add_child(node)
	return node

func box(size: Vector3, color: Color, point := Vector3.ZERO) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = size
	return part(mesh,color,point)

func ellipsoid(size: Vector3, color: Color, point: Vector3) -> void:
	var mesh := SphereMesh.new()
	mesh.radius = 1
	mesh.height = 2
	var node := part(mesh,color,point)
	node.scale = size/2

func build(aircraft) -> void:
	for child in get_children(): child.free()
	var dji: bool = aircraft.cfg.visual.style == "dji_fpv"
	var size: Vector3 = aircraft.vec(aircraft.cfg.visual.body_size_m)
	if dji:
		ellipsoid(size,Color("343b3b"),Vector3(0,.005,0))
		box(Vector3(.065,.05,.065),Color("1d2526"),Vector3(0,.015,.035))
		ellipsoid(Vector3(.079,.035,.106),Color("69865b"),Vector3(0,.03,-.008))
		box(Vector3(.04,.029,.025),Color("141c1f"),Vector3(0,-.003,-.069))
		ellipsoid(Vector3(.022,.022,.01),Color("556e76"),Vector3(0,-.003,-.084))
	else:
		box(size,Color("242e30"))
		box(Vector3(.034,.035,.024),Color("171e20"),Vector3(0,.033,-.037))
		ellipsoid(Vector3(.018,.018,.006),Color("516b73"),Vector3(-.006,.038,-.051))
	for i in range(4):
		var point: Vector3 = aircraft.TO_GODOT * aircraft.arms[i]
		var arm := box(Vector3(.023 if dji else .012,.016 if dji else .007,point.length()),Color("303b3c"))
		arm.look_at_from_position(point/2,point,Vector3.UP)
		var motor := CylinderMesh.new()
		motor.top_radius = .014
		motor.bottom_radius = .014
		motor.height = .022
		part(motor,Color("22292b"),point-Vector3(0,.012,0))
		var prop := CylinderMesh.new()
		prop.top_radius = aircraft.cfg.propeller_radius_m
		prop.bottom_radius = aircraft.cfg.propeller_radius_m
		prop.height = .002
		prop.radial_segments = 24
		var color := Color("b8c6b0") if dji else Color("db995c")
		if i >= 2: color = Color("97a6a0")
		color.a = .5
		part(prop,color,point)
		var foot: Vector3 = aircraft.TO_GODOT * aircraft.feet[i]
		var leg_top := Vector3(foot.x,point.y-.014,foot.z)
		box(Vector3(.008,leg_top.y-foot.y,.008),Color("303b3c"),(leg_top+foot)/2)
