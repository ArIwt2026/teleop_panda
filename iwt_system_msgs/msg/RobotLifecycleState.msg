# Authoritative state of the restartable Franka ROS stack.
# Only robot_supervisor publishes this message.

uint8 UNKNOWN=0
uint8 WAITING_FOR_FCI=1
uint8 STARTING=2
uint8 VERIFYING=3
uint8 CONNECTED_NEEDS_RECOVERY=4
uint8 READY=5
uint8 STOPPING=6
uint8 RESTART_BACKOFF=7
uint8 FAILED=8
uint8 OPERATOR_DISCONNECTED=9

uint8 RECOVERY_NONE=0
uint8 RECOVERY_AUTOMATIC_PENDING=1
uint8 RECOVERY_AUTOMATIC_RUNNING=2
uint8 RECOVERY_OPERATOR_REQUIRED=3
uint8 RECOVERY_FAILED=4

builtin_interfaces/Time stamp
uint8 state

bool network_reachable
bool fci_available
bool robot_state_fresh
bool controller_manager_available
bool required_controllers_active
bool recovery_required
bool robot_commands_allowed

# Raw Franka facts and the supervisor's recovery classification. Consumers
# must use these fields instead of independently interpreting robot_mode.
uint8 robot_mode
string robot_mode_name
string[] active_errors
uint8 recovery_state
uint32 recovery_attempt

uint32 restart_generation
uint32 restart_attempt
uint64 revision

string fault_code
string detail
