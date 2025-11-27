extends Control

#Left hand
@onready var left_hand_strength_edit: LineEdit = $"MarginContainer/GridContainer/Left Hand Container/VBoxContainer/HBoxContainer/Left Hand Strength Edit"
@onready var left_hand_strength_slider: HSlider = $"MarginContainer/GridContainer/Left Hand Container/VBoxContainer/Left Hand Strength Slider"
@onready var release_left_hand: Button = $"MarginContainer/GridContainer/Left Hand Container/Release Left Hand"
@onready var reach_left_hand: Button = $"MarginContainer/GridContainer/Left Hand Container/Reach Left Hand"
@onready var left_hand_options: OptionButton = $"MarginContainer/GridContainer/Left Hand Container/Left Hand Options"

#Right hand
@onready var right_hand_strength_edit: LineEdit = $"MarginContainer/GridContainer/Right Hand Container/VBoxContainer/HBoxContainer/Right Hand Strength Edit"
@onready var right_hand_strength_slider: HSlider = $"MarginContainer/GridContainer/Right Hand Container/VBoxContainer/Right Hand Strength Slider"
@onready var release_right_hand: Button = $"MarginContainer/GridContainer/Right Hand Container/Release Right Hand"
@onready var reach_right_hand: Button = $"MarginContainer/GridContainer/Right Hand Container/Reach Right Hand"
@onready var right_hand_options: OptionButton = $"MarginContainer/GridContainer/Right Hand Container/Right Hand Options"

#Left leg
@onready var left_leg_strength_edit: LineEdit = $"MarginContainer/GridContainer/Left Leg Container/VBoxContainer/HBoxContainer/Left Leg Strength Edit"
@onready var left_leg_strength_slider: HSlider = $"MarginContainer/GridContainer/Left Leg Container/VBoxContainer/Left Leg Strength Slider"
@onready var release_left_leg: Button = $"MarginContainer/GridContainer/Left Leg Container/Release Left Leg"
@onready var reach_left_leg: Button = $"MarginContainer/GridContainer/Left Leg Container/Reach Left Leg"
@onready var left_leg_options: OptionButton = $"MarginContainer/GridContainer/Left Leg Container/Left Leg Options"

#Right leg
@onready var right_leg_strength_edit: LineEdit = $"MarginContainer/GridContainer/Right Leg Container/VBoxContainer/HBoxContainer/Right Leg Strength Edit"
@onready var right_leg_strength_slider: HSlider = $"MarginContainer/GridContainer/Right Leg Container/VBoxContainer/Right Leg Strength Slider"
@onready var release_right_leg: Button = $"MarginContainer/GridContainer/Right Leg Container/Release Right Leg"
@onready var reach_right_leg: Button = $"MarginContainer/GridContainer/Right Leg Container/Reach Right Leg"
@onready var right_leg_options: OptionButton = $"MarginContainer/GridContainer/Right Leg Container/Right Leg Options"

func _ready() -> void:
	_connect_ui()
	
	SignalBus.start_route.connect(_on_start_route)
	
	left_hand_strength_slider.value = 50
	right_hand_strength_slider.value = 50
	left_leg_strength_slider.value = 50
	right_leg_strength_slider.value = 50

func _connect_ui() -> void:
	# LEFT HAND
	left_hand_strength_slider.value_changed.connect(_on_left_hand_slider_changed)
	left_hand_strength_edit.text_submitted.connect(_on_left_hand_edit_submitted)

	release_left_hand.pressed.connect(func():
		SignalBus.release_left_hand.emit()
	)

	reach_left_hand.pressed.connect(func():
		SignalBus.reach_left_hand.emit()
	)

	left_hand_options.item_selected.connect(func(value: int):
		print(value)
		SignalBus.left_hand_target_changed.emit(value)
	)

	# RIGHT HAND
	right_hand_strength_slider.value_changed.connect(_on_right_hand_slider_changed)
	right_hand_strength_edit.text_submitted.connect(_on_right_hand_edit_submitted)

	release_right_hand.pressed.connect(func():
		SignalBus.release_right_hand.emit()
	)

	reach_right_hand.pressed.connect(func():
		SignalBus.reach_right_hand.emit()
	)

	right_hand_options.item_selected.connect(func(value: int):
		SignalBus.right_hand_target_changed.emit(value)
	)

	# LEFT LEG
	left_leg_strength_slider.value_changed.connect(_on_left_leg_slider_changed)
	left_leg_strength_edit.text_submitted.connect(_on_left_leg_edit_submitted)

	release_left_leg.pressed.connect(func():
		SignalBus.release_left_foot.emit()
	)

	reach_left_leg.pressed.connect(func():
		SignalBus.reach_left_foot.emit()
	)

	left_leg_options.item_selected.connect(func(value: int):
		SignalBus.left_foot_target_changed.emit(value)
	)

	# RIGHT LEG
	right_leg_strength_slider.value_changed.connect(_on_right_leg_slider_changed)
	right_leg_strength_edit.text_submitted.connect(_on_right_leg_edit_submitted)

	release_right_leg.pressed.connect(func():
		SignalBus.release_right_foot.emit()
	)

	reach_right_leg.pressed.connect(func():
		SignalBus.reach_right_foot.emit())
	
	right_leg_options.item_selected.connect(func(value: int):
		SignalBus.right_foot_target_changed.emit(value)
	)

func _on_left_hand_slider_changed(value: float) -> void:
	left_hand_strength_edit.text = str(value)
	SignalBus.left_hand_strength_changed.emit(value)

func _on_left_hand_edit_submitted(text: String) -> void:
	var v := float(text)
	left_hand_strength_slider.value = v
	SignalBus.left_hand_strength_changed.emit(v)

func _on_right_hand_slider_changed(value: float) -> void:
	right_hand_strength_edit.text = str(value)
	SignalBus.right_hand_strength_changed.emit(value)

func _on_right_hand_edit_submitted(text: String) -> void:
	var v := float(text)
	right_hand_strength_slider.value = v
	SignalBus.right_hand_strength_changed.emit(v)

func _on_left_leg_slider_changed(value: float) -> void:
	left_leg_strength_edit.text = str(value)
	SignalBus.left_leg_strength_changed.emit(value)

func _on_left_leg_edit_submitted(text: String) -> void:
	var v := float(text)
	left_leg_strength_slider.value = v
	SignalBus.left_leg_strength_changed.emit(v)

func _on_right_leg_slider_changed(value: float) -> void:
	right_leg_strength_edit.text = str(value)
	SignalBus.right_leg_strength_changed.emit(value)

func _on_right_leg_edit_submitted(text: String) -> void:
	var v := float(text)
	right_leg_strength_slider.value = v
	SignalBus.right_leg_strength_changed.emit(v)

func _on_start_route(route: Route) -> void:
	var holds = route.get_holds()
	for hold in holds:
		left_hand_options.add_item(hold.hold_name)
		right_hand_options.add_item(hold.hold_name)
		left_leg_options.add_item(hold.hold_name)
		right_leg_options.add_item(hold.hold_name)
