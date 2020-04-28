# Definition of a detected item

string uuid

# Contains uuids of grasps on this object (if requested)
string[] grasp_uuids

string type

# Types
string RECTANGLE=RECTANGLE

# if type==RECTANGLE
Rectangle rectangle

# The pose of the item
geometry_msgs/PoseStamped pose
