# Execution progress for all provided action interfaces

# The action is in the planning queue
uint8 PROGRESS_PLANNING_QUEUED=1
# Planning has started
uint8 PROGRESS_PLANNING = 2
# Planning is done and execution will start once all prior actions are done
uint8 PROGRESS_EXECUTION_QUEUED = 128
# Execution has started
uint8 PROGRESS_EXECUTING= 129
# Progress as defined in the PROGRESS variables
uint8 progress

# Total number of actions that are executed for this action
# This is only set if progress=PROGRESS_EXECUTING
uint8 execution_action_count
# Index of action that is currently being executed
# This is only set if progress=PROGRESS_EXECUTING
uint8 execution_action_index

# Human-readable progress indicator
string message
