extends CharacterBody3D

const PhysicalBoneExtension = preload("res://Scripts/physical_bone_extension.gd")

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
var route: Route = null
@onready var can_climb_prompt: Label = $"../Can climb prompt"

var dragging: DraggingController
var camera_controller: CameraController

var bone_names = {
	"Root":				"Physical Bone Root",
	"Hips":				"Physical Bone Hips",
	"Spine":			"Physical Bone Spine",
	"Chest":			"Physical Bone Chest",
	"UpperChest":		"Physical Bone UpperChest",
	"Neck":				"Physical Bone Neck",
	"LeftShoulder":		"Physical Bone LeftShoulder",
	"LeftUpperArm":		"Physical Bone LeftUpperArm",
	"LeftLowerArm":		"Physical Bone LeftLowerArm",
	"LeftHand":			"Physical Bone LeftHand",
	"RightShoulder":	"Physical Bone RightShoulder",
	"RightUpperArm":	"Physical Bone RightUpperArm",
	"RightLowerArm":	"Physical Bone RightLowerArm",
	"RightHand":		"Physical Bone RightHand",
	"LeftUpperLeg":		"Physical Bone LeftUpperLeg",
	"LeftLowerLeg":		"Physical Bone LeftLowerLeg",
	"LeftFoot":			"Physical Bone LeftFoot",
	"RightUpperLeg":	"Physical Bone RightUpperLeg",
	"RightLowerLeg":	"Physical Bone RightLowerLeg",
	"RightFoot":		"Physical Bone RightFoot",
}
var bone_rotation_offsets = {} #Store offsets that allow transformation between physical bones and bones

func _ready() -> void:
	compute_offsets()
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
	
	var physical_bones = bone_sim.get_children()
	for physical_bone in physical_bones:
		if physical_bone is PhysicalBone3D:
			physical_bone.set_script(PhysicalBoneExtension)
	
	for bone_idx in skeleton.get_bone_count():
		var bone_name = skeleton.get_bone_name(bone_idx)
		if bone_name not in bone_names.keys():
			continue
		var bone_transform = skeleton.get_bone_global_pose(bone_idx)
		var bone_pos = bone_transform.origin
		var bone_rot = bone_transform.basis.get_euler()

		print("Bone:", bone_name, " | Global Position:", bone_pos, " | Global Rotation: ", rad_to_deg(bone_rot.x), ", ", rad_to_deg(bone_rot.y), ", ", rad_to_deg(bone_rot.z))


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
	run_bone_sim(true, ["Hips"])
	
	camera_controller.enter_free_fly()


func disable_ragdoll() -> void:
	state = PlayerState.MOVE
	bone_sim.active = false
	run_bone_sim(false)
	skeleton.reset_bone_poses()
	collision_shape.disabled = false
	anim.play("Idle")
	rotation.x = 0.0
	rotation.z = 0.0
	camera_controller.exit_free_fly()


func _on_wall_can_climb(new_route: Node3D) -> void:
	can_climb = true
	can_climb_prompt.visible = true
	route = new_route


func _on_wall_cant_climb() -> void:
	can_climb = false
	can_climb_prompt.visible = false
	route = null
	
func start_climbing():
	SignalBus.start_route.emit(route)
	state = PlayerState.CLIMB
	await climbing.enter_climb(route)
	camera_controller.enter_free_fly()

func stop_climbing():
	state = PlayerState.RAGDOLL
	climbing.exit_climb()

func compute_offsets():
	
	var skel_world := skeleton.global_transform
	for bone_name in bone_names:
		var bone_idx = skeleton.find_bone(bone_name)
		if bone_idx == -1:
			continue
		
		var physical_bone_name = bone_names[bone_name]
		var phys_bone: PhysicalBone3D = bone_sim.find_child(physical_bone_name)
		if phys_bone == null:
			continue
		
		# --- 1. Capture PHYSICAL BONE REST (world space) ---
		var phys_rest_world: Transform3D = phys_bone.global_transform
		
		# Convert physical rest → skeleton space
		var phys_rest_skel: Transform3D = skel_world.affine_inverse() * phys_rest_world
		
		# --- 2. Skeleton rest (in skeleton space) ---
		var skel_rest_skel: Transform3D = skeleton.get_bone_global_rest(bone_idx)
		
		# --- 3. Compute rest offset ---
		# PhysicalBone orientation = SkeletonBone orientation * offset
		var offset := skel_rest_skel.affine_inverse() * phys_rest_skel
		
		bone_rotation_offsets[bone_name] = offset

func run_bone_sim(run: bool, include: Array[StringName] = []) -> void:
	if not run:
		bone_sim.physical_bones_stop_simulation()
		return
	
	if include.is_empty():
		bone_sim.physical_bones_start_simulation()
		return
	
	bone_sim.physical_bones_start_simulation(include)
