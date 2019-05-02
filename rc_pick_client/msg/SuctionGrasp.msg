# Definition of a suction grasp for the itempick component

string uuid

# Optional
string item_uuid

# The grasp orientation is defined such that:
# - the z axis is the normal of the surface at the grasp position
# - the x axis is directed along the direction of maximum elongation of the surface
# - the y axis is selected to form a right-handed coordinate system
geometry_msgs/PoseStamped pose

# Optional. The quality of the grasp (between 0 and 1)
float64 quality

# Optional. The axis lengths of the biggest ellipse inscribed on the suction surface
float64 max_suction_surface_length
float64 max_suction_surface_width
