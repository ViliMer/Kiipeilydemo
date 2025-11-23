class_name DraggingController
extends Node

var max_distance := 50.0
var dragged_body: PhysicalBone3D = null
var grab_point_local: Vector3
var initial_grab_distance := 10.0

var character: CharacterBody3D
var camera_node: Camera3D

func init(character_ref: CharacterBody3D) -> void:
	character = character_ref
	camera_node = character_ref.camera_node
	pass

func try_grab():
	var mouse_pos := get_viewport().get_mouse_position()
	var ray_origin := camera_node.project_ray_origin(mouse_pos)
	var ray_dir := camera_node.project_ray_normal(mouse_pos)
	var ray_to := ray_origin + ray_dir * max_distance

	var query := PhysicsRayQueryParameters3D.create(ray_origin, ray_to)
	query.collide_with_areas = false
	query.collide_with_bodies = true
	query.collision_mask = 2
	
	var space := character.get_world_3d().direct_space_state
	var result := space.intersect_ray(query)
	
	if not result:
		return

	var body = result["collider"]
	if body is PhysicalBone3D:
		dragged_body = body
		# local strike position so we don't always pull from the center of mass
		grab_point_local = body.to_local(result.position)

		# distance from camera (so the dragged point stays under the cursor)
		initial_grab_distance = camera_node.global_transform.origin.distance_to(result.position)
	else:
		print("Hit collider is not a PhysicalBone3D (it is ", body.get_class(), ")")

func release():
	dragged_body = null

func update_ragdoll(delta: float) -> void:
	
	if not (dragged_body):
		return
	
	var mouse_pos := get_viewport().get_mouse_position()
	var ray_origin := camera_node.project_ray_origin(mouse_pos)
	var ray_dir := camera_node.project_ray_normal(mouse_pos)
	
	var target_distance := initial_grab_distance
	
	# Shoot a ray toward the target to check for obstacles
	var env_ray_to := ray_origin + ray_dir * max_distance
	var query := PhysicsRayQueryParameters3D.create(ray_origin, env_ray_to)
	query.collision_mask = 1  # environment
	query.collide_with_bodies = true
	query.collide_with_areas = false
	query.exclude = [dragged_body]  # ignore the dragged bone itself
	
	var hit := character.get_world_3d().direct_space_state.intersect_ray(query)
	if hit:
		var distance_to_hit := ray_origin.distance_to(hit.position)
		if distance_to_hit < initial_grab_distance:
			# clamp target distance to 0.8 of the obstacle distance
			target_distance = distance_to_hit * 0.8
	
	var target  := ray_origin + ray_dir * target_distance
	
	# ======================================================================
	# NEW: Physically drag the bone using a critically-damped spring force
	# ======================================================================
	
	# ------- Position of the grabbed point on the bone -------
	var grabbed_world_point := dragged_body.to_global(grab_point_local)

	# ------- Linear spring behavior (no angular component) -------
	var stiffness := 50.0         # how strongly it pulls toward the cursor
	var damping := 25.0            # reduces overshoot
	var strength := 1.0           # global multiplier for drag "power"

	# Current velocity of the whole bone
	var current_vel := dragged_body.linear_velocity

	# Desired velocity for the grabbed point
	var displacement := target - grabbed_world_point
	var desired_vel := displacement * stiffness

	# Damping reduces existing velocity
	var correction_vel := (desired_vel - current_vel) * damping * delta * strength

	# Apply correction to the whole bone (no angular effect)
	dragged_body.linear_velocity += correction_vel

	# Optional: clamp for stability
	var max_speed := 25.0
	if dragged_body.linear_velocity.length() > max_speed:
		dragged_body.linear_velocity = dragged_body.linear_velocity.normalized() * max_speed
