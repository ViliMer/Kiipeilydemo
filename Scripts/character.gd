extends CharacterBody3D

@export var move_speed: float = 5.0
@export var jump_force: float = 4.5
@export var mouse_sensitivity: float = 0.002

@export var max_distance := 50.0
@export var drag_strength := 50.0

enum PlayerState { MOVE, RAGDOLL, CLIMB }
var state: PlayerState = PlayerState.MOVE

var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")

@onready var camera_pivot: Node3D = $CameraPivot
@onready var camera_node: Camera3D = $CameraPivot/Camera3D
@onready var anim: AnimationPlayer = $"Imported Character/AnimationPlayer"

@onready var mesh: MeshInstance3D = $"Imported Character/Rig/GeneralSkeleton/Mannequin"
@onready var skeleton: Skeleton3D = %GeneralSkeleton
@onready var bone_sim: PhysicalBoneSimulator3D = $"Imported Character/Rig/GeneralSkeleton/PhysicalBoneSimulator3D"
@onready var collision_shape: CollisionShape3D = $CollisionShape3D

@onready var right_hand_ik: SkeletonIK3D = $"Imported Character/Rig/GeneralSkeleton/Right Hand IK"
@onready var left_hand_ik: SkeletonIK3D = $"Imported Character/Rig/GeneralSkeleton/Left Hand IK"
@onready var right_foot_ik: SkeletonIK3D = $"Imported Character/Rig/GeneralSkeleton/Right Foot IK"
@onready var left_foot_ik: SkeletonIK3D = $"Imported Character/Rig/GeneralSkeleton/Left Foot IK"

@onready var right_hand_target: Node3D = $"Right Hand Target"
@onready var left_hand_target: Node3D = $"Left Hand Target"
@onready var right_foot_target: Node3D = $"Right Foot Target"
@onready var left_foot_target: Node3D = $"Left Foot Target"


var dragged_body: PhysicalBone3D = null
var grab_point_local: Vector3
var joint: PinJoint3D
var anchor: StaticBody3D
var initial_grab_distance := 10.0

var pitch: float = 0.0

var climbing: ClimbingController
var can_climb: bool = false
var wall: Node3D = null
@onready var can_climb_prompt: Label = $"../Can climb prompt"

func _ready() -> void:
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	mesh.visible = true
	
	climbing = ClimbingController.new()
	add_child(climbing)
	climbing.init(self)


func _input(event: InputEvent) -> void:
	# Mouse look
	if event is InputEventMouseMotion:
		rotate_y(-event.relative.x * mouse_sensitivity)

		pitch = clamp(pitch + event.relative.y * mouse_sensitivity/4,
					  deg_to_rad(-50),
					  deg_to_rad(50))
		camera_pivot.rotation.x = pitch
		
	if Input.is_action_just_pressed("Quit"):
		get_tree().quit()
	
	if Input.is_action_just_pressed("Ragdoll") and state == PlayerState.MOVE:
		enable_ragdoll()
	if Input.is_action_just_pressed("Idle") and state == PlayerState.RAGDOLL:
		disable_ragdoll()
	if Input.is_action_just_pressed("Interact") and can_climb:
		print("Start climbing")
		start_climbing()
		can_climb = false
	elif Input.is_action_just_pressed("Interact") and state == PlayerState.CLIMB:
		print("Stop climbing")
		stop_climbing()
	
	if state == PlayerState.RAGDOLL and event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			if event.pressed:
				_try_grab()
			else:
				_release()

func _try_grab():
	var mouse_pos := get_viewport().get_mouse_position()
	var ray_origin := camera_node.project_ray_origin(mouse_pos)
	var ray_dir := camera_node.project_ray_normal(mouse_pos)
	var ray_to := ray_origin + ray_dir * max_distance

	var query := PhysicsRayQueryParameters3D.create(ray_origin, ray_to)
	query.collide_with_areas = false
	query.collide_with_bodies = true
	query.collision_mask = 2
	
	var space := get_world_3d().direct_space_state
	var result := space.intersect_ray(query)
	
	if not result:
		return

	var body = result["collider"]
	if body is PhysicalBone3D:
		# rest of your grab logic...
		dragged_body = body
		grab_point_local = body.to_local(result.position)

		anchor = StaticBody3D.new()
		add_child(anchor)
		anchor.global_transform.origin = result.position
		
		initial_grab_distance = camera_node.global_transform.origin.distance_to(result.position)

		joint = PinJoint3D.new()
		joint.node_a = anchor.get_path()
		joint.node_b = body.get_path()
		anchor.add_child(joint)
	else:
		print("Hit collider is not a RigidBody3D (it is ", body.get_class(), ")")

func _release():
	if joint:
		joint.queue_free()
		joint = null
		if anchor:
			anchor.queue_free()
			anchor = null
	dragged_body = null

func _physics_process(delta: float) -> void:
	match state:
		PlayerState.MOVE:
			_update_movement(delta)
		PlayerState.RAGDOLL:
			_update_ragdoll(delta)
		PlayerState.CLIMB:
			climbing.update_climbing(delta)

func _update_movement(delta: float) -> void:
	var input_dir = Vector3.ZERO

	if Input.is_action_pressed("Forward"):
		input_dir += transform.basis.z
	if Input.is_action_pressed("Back"):
		input_dir -= transform.basis.z
	if Input.is_action_pressed("Left"):
		input_dir += transform.basis.x
	if Input.is_action_pressed("Right"):
		input_dir -= transform.basis.x

	input_dir = input_dir.normalized()

	# Horizontal movement
	velocity.x = input_dir.x * move_speed
	velocity.z = input_dir.z * move_speed

	# Gravity
	if not is_on_floor():
		velocity.y -= gravity * delta

	# Jump
	if Input.is_action_just_pressed("Jump") and is_on_floor():
		velocity.y = jump_force
		anim.play("Jump_Start") # if you have a jump animation

	# Play animations
	if is_on_floor():
		if input_dir.length() > 0.1:
			anim.play("Jog_Fwd")
		else:
			anim.play("Idle")
			
	move_and_slide()

func _update_ragdoll(delta: float) -> void:
	
	if not (dragged_body and anchor):
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
	query.exclude = [dragged_body, anchor]  # ignore the dragged bone itself
	
	var hit := get_world_3d().direct_space_state.intersect_ray(query)
	if hit:
		var distance_to_hit := ray_origin.distance_to(hit.position)
		if distance_to_hit < initial_grab_distance:
			# clamp target distance to 0.8 of the obstacle distance
			target_distance = distance_to_hit * 0.8
	
	var target  := ray_origin + ray_dir * target_distance
	
	#var target := camera_node.project_ray_origin(mouse_pos) + camera_node.project_ray_normal(mouse_pos) * 10.0

	# Move anchor toward target smoothly
	anchor.global_transform.origin = anchor.global_transform.origin.lerp(target, drag_strength * delta)

func enable_ragdoll() -> void:
	# Enter ragdoll state
	state = PlayerState.RAGDOLL
	
	# Release mouse for point-and-click
	Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)

	# Stop player movement completely
	velocity = Vector3.ZERO
	#set_physics_process(false)
	#set_process(false)

	# Disable animation so bones can move freely
	if anim:
		anim.play("A_TPose")
		anim.seek(0.0, true)
		anim.play("hand_pose")
		anim.seek(0.0, true)
		anim.stop()

	# Disable the main capsule collider (to prevent fighting the ragdoll)
	if collision_shape:
		collision_shape.disabled = true

	# Wait 1 frame so animation pose is applied before physics begins
	await get_tree().process_frame
	bone_sim.active = true
	bone_sim.physical_bones_start_simulation()


func disable_ragdoll() -> void:
	# Exit ragdoll state
	state = PlayerState.MOVE
	
	# Re-capture mouse for camera control
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)

	# Stop physical bone simulation
	bone_sim.active = false
	bone_sim.physical_bones_stop_simulation()

	# Wait 1 frame so simulation fully stops before resetting pose
	await get_tree().process_frame

	# Reset animated pose
	if skeleton:
		skeleton.reset_bone_poses()

	# Re-enable capsule and player code
	if collision_shape:
		collision_shape.disabled = false

	#set_physics_process(true)
	set_process(true)

	# Restart idle animation (or any animation you use)
	if anim:
		anim.play("Idle")


func _on_wall_can_climb(new_wall: Node3D) -> void:
	can_climb = true
	can_climb_prompt.visible = true
	wall = new_wall


func _on_wall_cant_climb() -> void:
	can_climb = false
	can_climb_prompt.visible = false
	
func start_climbing():
	state = PlayerState.CLIMB

	var starting_holds = wall.get_starting_holds()
	var start_pos = wall.climb_start_position
	climbing.enter_climb(starting_holds, start_pos)

func stop_climbing():
	state = PlayerState.RAGDOLL
	Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
	climbing.exit_climb()
