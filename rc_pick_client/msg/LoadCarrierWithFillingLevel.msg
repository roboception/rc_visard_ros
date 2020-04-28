# This represents a detected load carrier with it's filling level

string id

# Required
Box outer_dimensions

# Required
Box inner_dimensions

# Optional. If not provided, the rim thickness is computed from the outer and inner dimensions
Rectangle rim_thickness

geometry_msgs/PoseStamped pose

# True if a detected load carrier is overfilled. Ignored on setting load carrier.
bool overfilled

# Filling level of the whole load carrier
CellFillingLevel overall_filling_level

# Filling level of each load carrier cell
CellFillingLevel[] cells_filling_levels

GridSize filling_level_cell_count
