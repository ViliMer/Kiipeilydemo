class_name CameraController
extends Node3D

var sensitivity := 0.002
var fly_speed := 8.0
var fly_speed_fast := 20.0

var yaw := 0.0
var pitch := 0.0

var player: CharacterBody3D
var camera: Camera3D
var camera_pivot = Node3D

var third_person_transform: Transform3D

# state machine for camera
enum CameraState { THIRD_PERSON, FREE_FLY }
var camera_state: CameraState = CameraState.THIRD_PERSON

func init(p: CharacterBody3D):
	player = p
	camera = p.camera_node
	camera_pivot = p.camera_pivot
	
	# sync camera rotation values
	var e = camera.global_transform.basis.get_euler()
	pitch = e.x
	yaw = e.y


func _ready():
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)


func _input(event):
	if not player:
		return
	
	match camera_state:

		CameraState.THIRD_PERSON:
			_handle_third_person_input(event)

		CameraState.FREE_FLY:
			_handle_free_fly_input(event)

func _handle_third_person_input(event):
	if event is InputEventMouseMotion:
		# player rotates horizontally
		player.rotate_y(-event.relative.x * sensitivity)

		# pitch rotates camera_pivot vertically
		pitch = clamp(
			pitch + event.relative.y * sensitivity / 4.0,
			deg_to_rad(-50),
			deg_to_rad(50)
		)
		camera_pivot.global_rotation.x = pitch

func _handle_free_fly_input(event):
	var free_mouse = true
	if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		free_mouse = false
	#var holding_space := Input.is_action_pressed("Jump")

	# Mouse look (disabled while holding space for dragging)
	if event is InputEventMouseMotion and not free_mouse:
		yaw -= event.relative.x * sensitivity
		pitch = clamp(
			pitch - event.relative.y * sensitivity,
			deg_to_rad(-89),
			deg_to_rad(89)
		)
		camera.global_rotation = Vector3(pitch, yaw, 0)

	# Mouse mode toggling
	if Input.is_action_just_pressed("Jump"):
		if Input.mouse_mode == Input.MOUSE_MODE_VISIBLE:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
	#if Input.is_action_just_released("Jump"):
	#	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)

func _physics_process(delta):
	if not player:
		return
	
	match camera_state:
		CameraState.THIRD_PERSON:
			_update_third_person(delta)
		CameraState.FREE_FLY:
			_update_free_fly(delta)


func _update_third_person(_delta):
	#No need to do anything really
	return


func _update_free_fly(delta):
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

	dir = dir.normalized()

	var speed = fly_speed
	if Input.is_action_pressed("Sprint"):
		speed = fly_speed_fast

	camera.global_translate(dir * speed * delta)

func enter_free_fly():
	third_person_transform = camera.global_transform
	camera_state = CameraState.FREE_FLY

	# Start free-fly exactly at the current third-person camera transform
	#camera.global_transform = camera_pivot.global_transform

	# Sync yaw/pitch from current camera transform
	var e = camera.global_transform.basis.get_euler()
	yaw = e.y
	pitch = e.x

	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)


func exit_free_fly():
	camera_state = CameraState.THIRD_PERSON

	camera.global_transform = third_person_transform

	# Sync yaw/pitch to match pivot's rotation
	var e = camera_pivot.global_transform.basis.get_euler()
	yaw = e.y
	pitch = camera_pivot.global_rotation.x

	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
