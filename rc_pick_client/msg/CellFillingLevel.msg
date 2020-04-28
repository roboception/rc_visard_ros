# Dimensions of the 2D cell in meters
Rectangle cell_size

# Position of the cell center in meters (either in camera or external pose frame)
# The z-coordinate is on the level of the load carrier rim
geometry_msgs/Point cell_position

# Filling level of the cell in percent (100% represents a full cell)
RangeValue level_in_percent

# Free space of the cell in meters (how much space is left until the cell is full)
RangeValue level_free_in_meters

# Represents the percentage of valid pixels in this cell. Between 0 and 1 
float64 coverage
