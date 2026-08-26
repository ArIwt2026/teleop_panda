# System-wide health aggregate. Stations and components are data-driven arrays.
uint8 UNKNOWN=0
uint8 INITIALIZING=1
uint8 READY=2
uint8 DEGRADED=3
uint8 PAUSED=4
uint8 FAULTED=5
uint8 RECOVERING=6
uint8 OPERATOR_DISCONNECTED=7

builtin_interfaces/Time stamp
uint8 state
bool accepting_tasks
bool active_tasks_may_continue
bool recovery_required
string fault_code
string detail
string[] blocking_stations
string[] degraded_stations
StationHealth[] stations
uint64 revision
