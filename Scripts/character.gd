extends CharacterBody3D

@export var move_speed: float = 5.0
@export var jump_force: float = 4.5

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

var pitch: float = 0.0

var climbing: ClimbingController
var can_climb: bool = false
var wall: Node3D = null
@onready var can_climb_prompt: Label = $"../Can climb prompt"

var dragging: DraggingController
var camera_controller: CameraController

func _ready() -> void:
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	mesh.visible = true
	
	climbing = ClimbingController.new()
	add_child(climbing)
	climbing.init(self)
	
	dragging = DraggingController.new()
	add_child(dragging)
	dragging.init(self)
	
	camera_controller = CameraController.new()
	add_child(camera_controller)
	camera_controller.init(self)


func _input(event: InputEvent) -> void:
	if Input.is_action_just_pressed("Quit"):
		get_tree().quit()
	
	if Input.is_action_just_pressed("Ragdoll") and state == PlayerState.MOVE:
		enable_ragdoll()
	if Input.is_action_just_pressed("Idle") and state == PlayerState.RAGDOLL:
		disable_ragdoll()
	if Input.is_action_just_pressed("Interact") and can_climb:
		start_climbing()
		can_climb = false
	elif Input.is_action_just_pressed("Interact") and state == PlayerState.CLIMB:
		stop_climbing()
	
	if (state == PlayerState.RAGDOLL or state == PlayerState.CLIMB) and event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			if event.pressed:
				dragging.try_grab()
			else:
				dragging.release()

func _physics_process(delta: float) -> void:
	match state:
		PlayerState.MOVE:
			_update_movement(delta)
		PlayerState.RAGDOLL:
			dragging.update_ragdoll(delta)
		PlayerState.CLIMB:
			climbing.update_climbing(delta)
			dragging.update_ragdoll(delta)

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

func enable_ragdoll() -> void:
	state = PlayerState.RAGDOLL
	velocity = Vector3.ZERO

	#anim.play("A_TPose")
	#anim.seek(0.0, true)
	anim.play("hand_pose")
	anim.seek(0.0, true)
	anim.stop()

	collision_shape.disabled = true
	bone_sim.active = true
	bone_sim.physical_bones_start_simulation()
	
	camera_controller.enter_free_fly()


func disable_ragdoll() -> void:
	state = PlayerState.MOVE
	bone_sim.active = false
	bone_sim.physical_bones_stop_simulation()
	skeleton.reset_bone_poses()
	collision_shape.disabled = false
	anim.play("Idle")
	rotation.x = 0.0
	rotation.z = 0.0
	camera_controller.exit_free_fly()


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
	await climbing.enter_climb(starting_holds, start_pos)
	camera_controller.enter_free_fly()

func stop_climbing():
	state = PlayerState.RAGDOLL
	climbing.exit_climb()
