# Patrol execution state constants
uint32 IDLE=0
uint32 RUNNING=1
uint32 PAUSED=2
uint32 COMPLETED=3
uint32 FAILED=4

# Current patrol execution state
uint32 state 0

# Name of the currently active route
string route_name

# Zero-based index of the waypoint currently being navigated to
int32 current_waypoint_index

# Total number of waypoints in the active route
int32 n_waypoints
