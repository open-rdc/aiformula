#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
import numpy as np
import sys
import threading
import time
import matplotlib.pyplot as plt

class ParameterEstimator(Node):
    def __init__(self):
        super().__init__('parameter_estimator')
        
        # Parameters (User provided)
        self.declare_parameter('mass', 71.5)
        self.declare_parameter('I_zz', 7.72)
        
        self.m = self.get_parameter('mass').value
        self.I = self.get_parameter('I_zz').value
        
        # State variables
        self.cmd_vel = None
        self.t_last_cmd = 0
        
        # Data storage for regression AND validation
        # We store timestamps and raw values for plotting
        self.timestamps = []
        self.v_real = []
        self.w_real = []
        self.v_cmd_hist = []
        self.w_cmd_hist = []

        # Data for regression (filtered for dt)
        self.reg_data_v = [] # [v_next, v_curr, v_cmd, dt]
        self.reg_data_w = [] # [w_next, w_curr, w_cmd, dt]
        
        self.last_odom_time = None
        self.last_v = None
        self.last_w = None

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/vectornav/velocity_body', self.odom_callback, 10)
        
        self.get_logger().info(f"Initialized Estimator. Mass={self.m}, I={self.I}")
        self.get_logger().info("Waiting for data... Play your rosbag now.")

    def cmd_callback(self, msg):
        self.cmd_vel = msg
        self.t_last_cmd = time.time()

    def odom_callback(self, msg):
        # Use ROS timestamp from message for actual sync
        current_time_ros = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall_time = time.time()
        
        v = msg.twist.twist.linear.x
        # Right+ Throughout: Invert IMU (CCW+) to match Right+ internal model
        w = -msg.twist.twist.angular.z 
        
        # Store raw data for plotting
        self.timestamps.append(current_time_ros)
        self.v_real.append(v)
        self.w_real.append(w)
        
        # Align cmd_vel (zero hold)
        if self.cmd_vel and (wall_time - self.t_last_cmd < 0.5):
            curr_v_cmd = self.cmd_vel.linear.x
            # Normalization: Actuator inversion
            curr_w_cmd = -self.cmd_vel.angular.z
            self.v_cmd_hist.append(curr_v_cmd)
            self.w_cmd_hist.append(curr_w_cmd)
            
            # Prepare regression data
            if self.last_odom_time is not None:
                dt = current_time_ros - self.last_odom_time
                if 0.001 < dt < 1.0:
                    self.reg_data_v.append([v, self.last_v, curr_v_cmd, dt])
                    self.reg_data_w.append([w, self.last_w, curr_w_cmd, dt])
        else:
            self.v_cmd_hist.append(0.0)
            self.w_cmd_hist.append(0.0)

        if len(self.timestamps) % 100 == 0:
             self.get_logger().info(f"Collected {len(self.timestamps)} samples...")

        self.last_odom_time = current_time_ros
        self.last_v = v
        self.last_w = w

    def estimate_and_validate(self):
        # Snapshots to avoid race conditions
        ts = list(self.timestamps)
        vr = list(self.v_real)
        wr = list(self.w_real)
        vch = list(self.v_cmd_hist)
        wch = list(self.w_cmd_hist)
        
        # Ensure consistency
        min_len = min(len(ts), len(vr), len(wr), len(vch), len(wch))
        if min_len < 50:
            self.get_logger().warn(f"Not enough data to estimate yet (only {min_len} samples).")
            return

        ts = ts[:min_len]
        vr = vr[:min_len]
        wr = wr[:min_len]
        vch = vch[:min_len]
        wch = wch[:min_len]

        self.get_logger().info(f"Estimating using {len(self.reg_data_v)} regression samples...")
        
        # === Estimate Cv ===
        # Model: v_{k+1} = v_k + (K_v * v_cmd - C_v * v_k) / m * dt
        # Regression Y: (v_{k+1} - v_k)/dt * m
        # Regression X: [v_cmd, v_k]
        # Coeffs: [K_v, -C_v]
        
        Y_v = []
        X_v = []
        for val in self.reg_data_v:
            v_next, v_curr, v_cmd, dt = val
            Y_v.append((v_next - v_curr) / dt * self.m)
            X_v.append([v_cmd, v_curr])
            
        beta_v, _, _, _ = np.linalg.lstsq(X_v, Y_v, rcond=None)
        K_v = beta_v[0]
        C_v = -beta_v[1]

        # === Estimate Cw ===
        Y_w = []
        X_w = []
        for val in self.reg_data_w:
            w_next, w_curr, w_cmd, dt = val
            Y_w.append((w_next - w_curr) / dt * self.I)
            X_w.append([w_cmd, w_curr])
            
        beta_w, _, _, _ = np.linalg.lstsq(X_w, Y_w, rcond=None)
        K_w = beta_w[0]
        C_w = -beta_w[1]

        print(f"\n=== Estimation Results ===")
        print(f"Linear:  Kv={K_v:.4f}, Cv={C_v:.4f}")
        print(f"Angular: Kw={K_w:.4f}, Cw={C_w:.4f}")
        
        # === Validation Simulation ===
        # Simulate model from t=0 using the captured cmd_vel history
        sim_v = [vr[0]]
        sim_w = [wr[0]]
        
        for k in range(min_len - 1):
            dt = ts[k+1] - ts[k]
            if dt > 1.0 or dt <= 0: dt = 0.1 # handle gaps
            
            # Current state
            v_k = sim_v[-1]
            w_k = sim_w[-1]
            
            # Input
            v_cmd = vch[k]
            w_cmd = wch[k]
            
            # Physics Step
            # v_{k+1} = v_k + (K_v * v_cmd - C_v * v_k) / m * dt
            acc_v = (K_v * v_cmd - C_v * v_k) / self.m
            v_next = v_k + acc_v * dt
            
            acc_w = (K_w * w_cmd - C_w * w_k) / self.I
            w_next = w_k + acc_w * dt
            
            sim_v.append(v_next)
            sim_w.append(w_next)

        # === Plotting ===
        fig, axs = plt.subplots(2, 1, figsize=(10, 8))
        
        # Linear Velocity
        t_axis = [t - ts[0] for t in ts]
        axs[0].plot(t_axis, vr, label='Real Odom', color='blue', alpha=0.7)
        axs[0].plot(t_axis, sim_v, label='Simulated Model', color='orange', linestyle='--')
        axs[0].plot(t_axis, vch, label='Command', color='gray', alpha=0.3)
        axs[0].set_title(f"Linear Velocity (Cv={C_v:.3f})")
        axs[0].set_ylabel("v [m/s]")
        axs[0].legend()
        axs[0].grid(True)
        
        # Angular Velocity
        axs[1].plot(t_axis, wr, label='Real Odom', color='blue', alpha=0.7)
        axs[1].plot(t_axis, sim_w, label='Simulated Model', color='orange', linestyle='--')
        axs[1].plot(t_axis, wch, label='Command', color='gray', alpha=0.3)
        axs[1].set_title(f"Angular Velocity (Cw={C_w:.3f})")
        axs[1].set_ylabel("w [rad/s]")
        axs[1].set_xlabel("Time [s]")
        axs[1].legend()
        axs[1].grid(True)
        
        plt.tight_layout()
        print("Displaying plot... close window to continue.")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ParameterEstimator()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    try:
        input("Press Enter to START ESTIMATION & PLOTTING (ensure data is collected)...\n")
        node.estimate_and_validate()
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
