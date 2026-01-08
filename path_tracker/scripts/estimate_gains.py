#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque
import bisect
import sys

class GainEstimator(Node):
    def __init__(self):
        super().__init__('gain_estimator')

        # Parameters
        self.declare_parameter('wheelbase', 0.8)
        self.declare_parameter('min_velocity', 0.1)
        self.L = self.get_parameter('wheelbase').value
        self.min_v = self.get_parameter('min_velocity').value

        # Data buffers
        self.cmds = deque(maxlen=20000)
        self.velocities = deque(maxlen=20000)
        self.casters = deque(maxlen=20000)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/vectornav/velocity_body', self.vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/caster_data', self.caster_callback, 10)

        print("[INFO] Gain Estimator Started. Drive the robot. Press Ctrl+C to calculate.")

    def cmd_callback(self, msg):
        self.cmds.append((self.get_clock().now().nanoseconds, msg))

    def vel_callback(self, msg):
        self.velocities.append((self.get_clock().now().nanoseconds, msg))

    def caster_callback(self, msg):
        self.casters.append((self.get_clock().now().nanoseconds, msg))

    def find_nearest_idx(self, sorted_data, target_t):
        # sorted_data is list of (time, msg)
        # We want to find idx such that sorted_data[idx].time is closest to target_t
        # Use bisect to find insertion point
        times = [x[0] for x in sorted_data]
        idx = bisect.bisect_left(times, target_t)
        
        # Check idx and idx-1
        best_idx = -1
        min_dt = 1e18 # large number
        
        candidates = []
        if idx < len(sorted_data): candidates.append(idx)
        if idx > 0: candidates.append(idx - 1)
        
        for i in candidates:
            dt = abs(sorted_data[i][0] - target_t)
            if dt < min_dt:
                min_dt = dt
                best_idx = i
                
        return best_idx, min_dt

    def calculate(self):
        print("\n[INFO] Calculating gains... (Processing {} cmd points)".format(len(self.cmds)))
        
        if not self.cmds or not self.velocities or not self.casters:
            print("[WARN] Not enough data collected.")
            return

        v_cmd_data = [] 
        v_meas_data = [] 
        
        omega_cmd_data = [] 
        omega_meas_data = [] 

        # Create sorted lists once
        sorted_cmds = sorted(list(self.cmds), key=lambda x: x[0])
        sorted_vels = sorted(list(self.velocities), key=lambda x: x[0])
        sorted_casters = sorted(list(self.casters), key=lambda x: x[0])
        
        # Pre-extract times for bisect (optimization)
        vel_times = [x[0] for x in sorted_vels]
        caster_times = [x[0] for x in sorted_casters]

        MAX_SYNC_DIFF = 0.2 * 1e9 

        for t_cmd, cmd_msg in sorted_cmds:
            # 1. Find Closest Velocity
            idx_v = bisect.bisect_left(vel_times, t_cmd)
            # Compare idx_v and idx_v-1
            best_vel_msg = None
            min_dt_v = 1e18
            
            candidates_v = []
            if idx_v < len(sorted_vels): candidates_v.append(idx_v)
            if idx_v > 0: candidates_v.append(idx_v - 1)
            
            for i in candidates_v:
                dt = abs(vel_times[i] - t_cmd)
                if dt < min_dt_v:
                    min_dt_v = dt
                    best_vel_msg = sorted_vels[i][1]

            # 2. Find Closest Caster
            idx_c = bisect.bisect_left(caster_times, t_cmd)
            best_caster_msg = None
            min_dt_c = 1e18
            
            candidates_c = []
            if idx_c < len(sorted_casters): candidates_c.append(idx_c)
            if idx_c > 0: candidates_c.append(idx_c - 1)
            
            for i in candidates_c:
                dt = abs(caster_times[i] - t_cmd)
                if dt < min_dt_c:
                    min_dt_c = dt
                    best_caster_msg = sorted_casters[i][1]

            if min_dt_v > MAX_SYNC_DIFF or min_dt_c > MAX_SYNC_DIFF:
                continue

            # Data Extraction
            v_cmd = cmd_msg.linear.x
            w_cmd = cmd_msg.angular.z
            v_meas = best_vel_msg.twist.twist.linear.x
            
            if len(best_caster_msg.data) > 0:
                delta_meas = best_caster_msg.data[0]
            else:
                continue

            if abs(v_cmd) < self.min_v or abs(v_meas) < self.min_v:
                continue

            v_cmd_data.append(v_cmd)
            v_meas_data.append(v_meas)

            # Kinematic Model
            w_meas = (v_meas * np.tan(delta_meas)) / self.L
            
            if abs(w_cmd) > 0.05:
                omega_cmd_data.append(w_cmd)
                omega_meas_data.append(w_meas)

        # 1. Velocity Gain
        if len(v_cmd_data) > 10:
            vc = np.array(v_cmd_data)
            vm = np.array(v_meas_data)
            velocity_gain = np.dot(vc, vm) / np.dot(vm, vm)
            
            print(f"--- Velocity Analysis ---")
            print(f"Data Points: {len(vc)}")
            print(f"Calculated Velocity Gain: {velocity_gain:.4f}")
        else:
            print("[WARN] Not enough velocity data points.")
            velocity_gain = 1.0

        # 2. Steering Gain
        if len(omega_cmd_data) > 10:
            wc = np.array(omega_cmd_data)
            wm = np.array(omega_meas_data)
            steer_gain = np.dot(wc, wm) / np.dot(wm, wm)
            
            print(f"--- Steering Analysis ---")
            print(f"Data Points: {len(wc)}")
            print(f"Calculated Steer Gain: {steer_gain:.4f}")
        else:
            print("[WARN] Not enough steering data points.")
            steer_gain = 1.0
            
        print("\n" + "="*30)
        print("RECOMMENDED PARAMETERS:")
        print(f"velocity_gain: {velocity_gain:.4f}")
        print(f"steer_gain:    {steer_gain:.4f}")
        print("="*30 + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = GainEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Run calculation OUTSIDE the spin loop but before destruction
        try:
            node.calculate()
        except Exception as e:
            print(f"Error during calculation: {e}")
            
        node.destroy_node()
        # Check if already shutdown to avoid RCLError
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
