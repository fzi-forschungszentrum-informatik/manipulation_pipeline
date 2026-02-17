# Parameters describing a single linear motion for approaches or retracts

uint8 MOTION_TYPE_LINEAR = 1
uint8 MOTION_TYPE_PATH = 2
uint8 motion_type 1

# Distance to travel if motion_type is MOTION_TYPE_LINEAR
# If left empty, a default distance of 0.1 along the negative z-axis is assumed.
float64 distance

# Waypoints to follow if motion_type is MOTION_TYPE_PATH
geometry_msgs/Pose[] waypoints

# Cartesian motion limits
CartesianLimits limits
