include "pioneer.inc"

define floorplan model(
	color "gray30"     	# Sombre, sensible, artistic
	boundary 1         	# Most maps will need a bounding box
	gui_nose 0
	gui_grid 0
	gui_move 0
	gui_outline 0
	gripper_return 0
	fiducial_return 0
	laser_return 1
)

window(                   
	size [ 750 750 1]         # Tamanho da janela em pixeis
	rotate [ 0.000 0.000 ]    # Mapa não rodado
	center [ 0.000 0.000 ]  # Centro da Janela no ponto central do ambiente (em metros)
	scale 30                  # zoom da janela
	show_data 1
)