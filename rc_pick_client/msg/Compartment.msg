# Definition of a compartment (Primitive type without pose_frame and type)
# type is implicit since currently only Box type is supported
# the pose is relative to the pose_frame of the object including the compartment

# Required
# string type

# Supported types
# All primitives are defined to have their bounding boxes centered around (0,0,0)
string BOX=BOX
# string SPHERE=SPHERE
# string CYLINDER=CYLINDER
# string CONE=CONE

# Optional: if not specified, the pose is relative to the pose_frame of the object including the primitive
# string pose_frame

# Optional: identity transformation assumed if not provided
geometry_msgs/Pose pose

# Only one of those required depending on the selected type
Box box
# Sphere sphere
# Cylinder cylinder
# Cone cone
