# Description of a part of a collision object

# Name of the object
# This will be stored in the collision object id
string name

# Path to mesh
# This should be resolvable by resource_retriever, so probably start with package:// or file://
string path

# Offset specific to reference pose
geometry_msgs/Pose offset

# Relevant frames with respect to the mesh
# This is commonly used to store relevant poses for grasping
string[] subframe_names
geometry_msgs/Pose[] subframe_poses
