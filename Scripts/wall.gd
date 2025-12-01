class_name Route
extends Node3D

signal can_climb(route: Node3D)
signal cant_climb()

@onready var holds: Node3D = $Holds
@onready var lh_start: Hold = $"Holds/Hold 4"
@onready var rh_start: Hold = $"Holds/Hold 3"
@onready var lf_start: Hold = $"Holds/Hold 2"
@onready var rf_start: Hold = $"Holds/Hold 1"
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

func get_holds():
	return holds.get_children()
