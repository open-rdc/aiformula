uint8 TURN_UNKNOWN=0
uint8 TURN_STRAIGHT=1
uint8 TURN_LEFT=2
uint8 TURN_RIGHT=3
uint8 TURN_MERGE=4
uint8 TURN_BRANCH=5
uint8 TURN_U_TURN=6

uint64 id
uint64 from_lanelet_id
uint64 to_lanelet_id
uint8 turn_direction
float64 cost
