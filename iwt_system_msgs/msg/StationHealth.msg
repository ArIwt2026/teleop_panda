# Aggregated health for one extensible production station.
uint8 UNKNOWN=0
uint8 INITIALIZING=1
uint8 READY=2
uint8 BUSY=3
uint8 DEGRADED=4
uint8 PAUSED=5
uint8 FAULTED=6
uint8 RECOVERING=7
uint8 OFFLINE=8
uint8 DISABLED=9

builtin_interfaces/Time stamp
string station_id
string display_name
uint8 state
bool available
bool healthy
bool ready
bool accepting_tasks
string active_task_id
string fault_code
string detail
string[] blocking_components
string[] degraded_components
ComponentHealth[] components
uint64 revision
