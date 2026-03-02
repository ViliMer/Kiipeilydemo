class_name CameraController
extends Node3D

var initialized := false

var sensitivity := 0.001
var fly_speed := 8.0
var fly_speed_fast := 20.0

var yaw := 0.0
var pitch := 0.0


@onready var camera: Camera3D = $ThirdPersonRig/CameraMount/Camera3D
@onready var third_person_rig: Node3D = $ThirdPersonRig
@onready var third_person_mount: Node3D = $ThirdPersonRig/CameraMount

enum CameraState {THIRD_PERSON, FIRST_PERSON, FREE_FLY, TRANSITION}
var camera_state: CameraState = CameraState.THIRD_PERSON

var player: Node3D

func _ready():
	camera_state = CameraState.THIRD_PERSON
	#camera.global_transform = third_person_mount.global_transform
	initialized = true

func init(p: Node3D) -> void:
	player = p

func _input(event):
	if camera_state == CameraState.TRANSITION:
		return
	
	match camera_state:
		CameraState.THIRD_PERSON:
			_input_third_person(event)

		CameraState.FREE_FLY:
			_input_free_fly(event)

func _input_third_person(event):
	if event is InputEventMouseMotion:
		# Horizontal mouse = player yaw
		player.rotate_y(-event.relative.x * sensitivity)

		# Vertical mouse = camera pitch
		pitch = clamp(
			pitch + event.relative.y * sensitivity,
			deg_to_rad(-50),
			deg_to_rad(50)
		)

func _input_free_fly(event):
	if event is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		yaw -= event.relative.x * sensitivity
		pitch = clamp(
			pitch - event.relative.y * sensitivity,
			deg_to_rad(-89),
			deg_to_rad(89)
		)

	if Input.is_action_just_pressed("Jump"):
		if Input.mouse_mode == Input.MOUSE_MODE_VISIBLE:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)


func _physics_process(delta):
	if not initialized:
		return
	
	match camera_state:
		CameraState.TRANSITION:
			return
		CameraState.THIRD_PERSON:
			_update_third_person()
		CameraState.FREE_FLY:
			_update_free_fly(delta)


func _update_third_person():
	third_person_rig.rotation.x = pitch


func _update_free_fly(delta):
	camera.global_rotation = Vector3(pitch, yaw, 0)
	
	var dir := Vector3.ZERO

	var forward := -camera.global_transform.basis.z
	var right := camera.global_transform.basis.x

	if Input.is_action_pressed("Forward"):
		dir += forward
	if Input.is_action_pressed("Back"):
		dir -= forward
	if Input.is_action_pressed("Left"):
		dir -= right
	if Input.is_action_pressed("Right"):
		dir += right

	if dir != Vector3.ZERO:
		dir = dir.normalized()

	var speed = fly_speed
	if Input.is_action_pressed("Sprint"):
		speed = fly_speed_fast

	camera.global_translate(dir * speed * delta)
	

func enter_free_fly():
	if camera_state == CameraState.FREE_FLY:
		return
	
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)

	camera_state = CameraState.FREE_FLY

	var mount_euler := third_person_mount.global_basis.get_euler()
	yaw = mount_euler.y
	pitch = mount_euler.x

func enter_third_person():
	if camera_state == CameraState.THIRD_PERSON:
		return
	
	camera_state = CameraState.TRANSITION
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	
	var global := camera.global_transform
	_reparent_camera(third_person_mount)
	camera.global_transform = global
	
	third_person_rig.basis = Basis.IDENTITY

	var tween := create_tween()
	tween.set_trans(Tween.TRANS_SINE)
	tween.set_ease(Tween.EASE_IN_OUT)

	tween.tween_property(
		camera,
		"transform",
		Transform3D.IDENTITY,
		0.5
	)

	await tween.finished

	camera_state = CameraState.THIRD_PERSON
	
	var mount_euler := third_person_mount.global_basis.get_euler()
	var rig_euler := third_person_rig.global_basis.get_euler()
	yaw = mount_euler.y
	pitch = rig_euler.x


func _reparent_camera(new_parent: Node3D) -> void:
	if camera.get_parent() == new_parent:
		return

	camera.reparent(new_parent)
