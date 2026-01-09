std_msgs/Header header

float64 v            # cmd_vel.linear.x [m/s]
float64 w            # cmd_vel.angular.z [rad/s]

float64 caster       # caster angle [rad] (real: caster_orientation, sim: caster_yaw_angle)
float64 motor_cmd    # motor command [rad] (real: odrive input_pos, sim: motor_spin_angle)

float64 delta        # target [rad] (real: caster_data[0], sim: NaN)

uint8 domain         # 0 = REAL, 1 = SIM
int32 run_id         # experiment/run identifier (optional)

