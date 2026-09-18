extends Node3D
var obstacles: Array[AABB] = []
var gates: Array[Vector3] = []
var materials: Dictionary = {}

func material(color: Color) -> StandardMaterial3D:
	var key := color.to_html()
	if materials.has(key):
		return materials[key]
	var m := StandardMaterial3D.new()
	m.albedo_color = color
	m.roughness = 0.9
	materials[key] = m
	return m

func box(pos: Vector3, size: Vector3, color: Color, solid := false) -> MeshInstance3D:
	var node := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	# Godot clamps very thin primitive dimensions; scale a unit box so surface
	# markings keep their intended sub-centimeter thickness and layer order.
	mesh.size = Vector3.ONE
	node.mesh = mesh
	node.scale = size
	node.material_override = material(color)
	node.position = pos
	add_child(node)
	if solid:
		obstacles.append(AABB(pos - size / 2, size))
	return node

func cylinder(pos: Vector3, radius: float, height: float, color: Color, top := -1.0) -> MeshInstance3D:
	var node := MeshInstance3D.new()
	var mesh := CylinderMesh.new()
	mesh.bottom_radius = radius
	mesh.top_radius = radius if top < 0 else top
	mesh.height = height
	mesh.radial_segments = 10
	node.mesh = mesh
	node.material_override = material(color)
	node.position = pos
	add_child(node)
	return node

func _ready() -> void:
	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var sky_mat := ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color("53788e")
	sky_mat.sky_horizon_color = Color("c8d7d1")
	sky_mat.ground_horizon_color = Color("c8d7d1")
	sky_mat.ground_bottom_color = Color("7c8870")
	sky.sky_material = sky_mat
	e.sky = sky
	e.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	e.ambient_light_color = Color("c2d4df")
	e.ambient_light_energy = 0.45
	e.ambient_light_sky_contribution = 0.0
	e.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	e.fog_enabled = true
	e.fog_light_color = Color("b0c5c3")
	e.fog_density = 0.0018
	env.environment = e
	add_child(env)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-35, -38, 0)
	sun.light_color = Color("fff0d4")
	sun.light_energy = 0.85
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 130
	add_child(sun)
	box(Vector3(0,-0.15,0), Vector3(2500,0.3,2500), Color("6e8060"))
	# Surface markings stay within 3 mm of the physical ground plane, so short
	# landing gear doesn't sink inside the decorative runway or launch pad.
	for i in range(-12,13):
		box(Vector3(i*14,0.0003,-65), Vector3(7,0.0004,300), Color("738465"))
	box(Vector3(0,-0.001,-55), Vector3(8,0.005,170), Color("444e4c"))
	for i in range(18):
		box(Vector3(0,0.00175,20-i*9),Vector3(0.15,0.0005,3),Color("d7d8be"))
	for side in [-1,1]:
		box(Vector3(side*3.7,0.00175,-55),Vector3(0.08,0.0005,166),Color("bcc4b3"))
	cylinder(Vector3(0,0.0015,0),1.4,0.002,Color("e69c5c"))
	box(Vector3(0,0.00275,0),Vector3(1.3,0.0005,0.16),Color("f4eddb"))
	for x in [-0.55,0.55]:
		box(Vector3(x,0.00275,0),Vector3(0.16,0.0005,1.0),Color("f4eddb"))
	var positions := [Vector3(0,0,-22),Vector3(0,0,-55),Vector3(18,0,-80),Vector3(37,0,-55),Vector3(25,0,-22)]
	for i in range(positions.size()):
		var p: Vector3 = positions[i]
		gates.append(p + Vector3(0,2.1,0))
		var color := Color("f1aa63") if i == 0 else Color("d8dfcd")
		box(p+Vector3(-2.2,2,0),Vector3(0.20,4,0.30),color,true)
		box(p+Vector3(2.2,2,0),Vector3(0.20,4,0.30),color,true)
		box(p+Vector3(0,4,0),Vector3(4.6,0.2,0.30),color,true)
		for x in [-2.2,2.2]:
			box(p+Vector3(x,0.35,0),Vector3(0.5,0.7,0.65),Color("384744"),true)
		var label := Label3D.new()
		label.text = "%02d" % (i+1)
		label.font_size = 80
		label.pixel_size = 0.009
		label.position = p+Vector3(0,4.65,0)
		label.modulate = Color("eae9d9")
		add_child(label)
	box(Vector3(-27,3.5,-20),Vector3(14,7,20),Color("4d6262"),true)
	box(Vector3(-27,7.1,-20),Vector3(15,0.4,21),Color("b4bda9"),true)
	box(Vector3(-19.96,2.6,-20),Vector3(0.05,5.2,12),Color("273a3d"))
	box(Vector3(-27,0.02,-3),Vector3(24,0.03,12),Color("a0a48e"))
	# Boundary trees and distant low-poly mountains.
	var rng := RandomNumberGenerator.new()
	rng.seed = 427
	for i in range(100):
		var angle := rng.randf_range(0,TAU)
		var r := rng.randf_range(110,240)
		var p := Vector3(cos(angle)*r,0,sin(angle)*r-50)
		var h := rng.randf_range(6,14)
		cylinder(p+Vector3(0,h*0.22,0),0.35,h*0.44,Color("5b5b48"))
		cylinder(p+Vector3(0,h*0.6,0),h*0.27,h*0.85,Color("3d5f50"),0)
	for i in range(24):
		var angle := i*TAU/24
		var h := rng.randf_range(60,160)
		cylinder(Vector3(cos(angle)*680,h*0.5-10,sin(angle)*680-50),rng.randf_range(120,200),h,Color("71867d"),0)
	cylinder(Vector3(12,3,5),0.06,6,Color("c4cabc"))
	var sock := cylinder(Vector3(12.7,5.6,5),0.3,1.5,Color("e69c5c"),0.15)
	sock.rotation.z = PI/2

func hits(previous: Vector3, current: Vector3) -> bool:
	for obstacle in obstacles:
		var expanded := obstacle.grow(0.15)
		if expanded.has_point(current) or expanded.intersects_segment(previous,current) != null:
			return true
	return false
