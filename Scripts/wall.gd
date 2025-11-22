extends Node3D

signal can_climb(route: Node3D)
signal cant_climb()

@onready var lh_start: Node3D = $Hold
@onready var rh_start: Node3D = $Hold2
@onready var lf_start: Node3D = $Hold7
@onready var rf_start: Node3D = $Hold8
@onready var climb_start_position: Node3D = $"Climb Start Position"


func _on_detect_climber_area_body_entered(body: Node3D) -> void:
	if body is CharacterBody3D:
		emit_signal("can_climb", self)


func _on_detect_climber_area_body_exited(body: Node3D) -> void:
	if body is CharacterBody3D:
		emit_signal("cant_climb")

func get_starting_holds():
	return {
		"lh": lh_start,
		"rh": rh_start,
		"lf": lf_start,
		"rf": rf_start,
	}
