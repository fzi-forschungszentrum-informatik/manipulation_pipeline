# Command to send to an end-effector
# TODO Add more options

# Select type of command to send
uint8 COMMAND_JOINT = 1
uint8 COMMAND_DWELL = 2
uint8 COMMAND_NONE = 255
uint8 command

# Conventional joint state command
# Only used with command=COMMAND_JOINT. This can be used for all sorts of grippers and other end-effectors that MoveIt natively supports.
sensor_msgs/JointState joint_cmd

# Time in seconds to dwell for
# Only used with command=COMMAND_DWELL.
float64 dwell_duration

# Controller used for execution
# Only relevant for command=COMMAND_JOINT
string controller
