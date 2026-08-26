# Generic health lease for any device, driver, service, or software component.
uint8 UNKNOWN=0
uint8 STARTING=1
uint8 READY=2
uint8 BUSY=3
uint8 DEGRADED=4
uint8 PAUSED=5
uint8 FAULTED=6
uint8 RECOVERING=7
uint8 OFFLINE=8
uint8 DISABLED=9

builtin_interfaces/Time stamp
string component_id
string station_id
string component_type
uint8 state
bool available
bool healthy
bool ready
bool enabled
bool required
bool fresh
string[] capabilities
string fault_code
string detail
uint64 revision
