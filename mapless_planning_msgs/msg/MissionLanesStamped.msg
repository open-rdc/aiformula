std_msgs/Header header
DrivingCorridor lane_with_goal_point
DrivingCorridor ego_lane
DrivingCorridor[] drivable_lanes_left
DrivingCorridor[] drivable_lanes_right
float32 deadline_target_lane
int16 target_lane

int16 LANE_KEEP=0
int16 LANE_CHANGE_LEFT=-1
int16 LANE_CHANGE_RIGHT=1
int16 TAKE_NEXT_EXIT_LEFT=-2
int16 TAKE_NEXT_EXIT_RIGHT=2
