# Authoritative high-level production task state.
# Detailed task phases can be added without coupling them to robot connectivity.

uint8 IDLE=0
uint8 QUEUED=1
uint8 WAITING_FOR_RESOURCES=2
uint8 RUNNING=3
uint8 PAUSED=4
uint8 CANCELING=5
uint8 BLOCKED=6
uint8 SUCCEEDED=7
uint8 FAILED=8

builtin_interfaces/Time stamp
uint8 state
string task_id
string task_type
string phase
float32 progress
uint32 robot_generation
uint64 revision
string error_code
string detail

# Drain/disconnect coordination. New work is rejected as soon as a disconnect
# is requested; disconnect_ready becomes true only after successful homing.
bool accepting_new_tasks
bool at_home
bool disconnect_pending
bool disconnect_ready
bool homing_required
bool homing_active
