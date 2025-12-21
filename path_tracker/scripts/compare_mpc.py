#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
import matplotlib.pyplot as plt
from collections import deque
import threading
import time

class MPCChecker(Node):
    def __init__(self):
        super().__init__('mpc_checker')
        self.bag_vel = deque(maxlen=5000)
        self.bag_ang = deque(maxlen=5000)
        self.mpc_vel = deque(maxlen=5000)
        self.mpc_ang = deque(maxlen=5000)
        self.real_vel = deque(maxlen=5000)
        self.real_ang = deque(maxlen=5000)
        self.times_bag = deque(maxlen=5000)
        self.times_mpc = deque(maxlen=5000)
        self.times_real = deque(maxlen=5000)

        self.start_time = None

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Twist, '/cmd_vel', self.bag_callback, qos_profile)
        self.create_subscription(Twist, '/cmd_vel_mpc', self.mpc_callback, qos_profile)
        self.create_subscription(TwistWithCovarianceStamped, '/vectornav/velocity_body', self.real_callback, qos_profile)

        print("Subscriptions started. Listening for /cmd_vel (Bag), /cmd_vel_mpc (MPC), and /vectornav/velocity_body (Real)...")

    def sync_time(self):
        if self.start_time is None:
            self.start_time = time.time()
            print("First message received, starting recording...")

    def bag_callback(self, msg):
        self.sync_time()
        self.times_bag.append(time.time() - self.start_time)
        self.bag_vel.append(msg.linear.x)
        self.bag_ang.append(msg.angular.z)

    def mpc_callback(self, msg):
        self.sync_time()
        self.times_mpc.append(time.time() - self.start_time)
        self.mpc_vel.append(msg.linear.x)
        self.mpc_ang.append(msg.angular.z)

    def real_callback(self, msg):
        self.sync_time()
        self.times_real.append(time.time() - self.start_time)
        self.real_vel.append(msg.twist.twist.linear.x)
        # Right+ Throughout: Invert IMU (CCW+) to match Right+ plot labels
        self.real_ang.append(-msg.twist.twist.angular.z)

def plot_thread(checker):
    plt.ion()
    fig, ax = plt.subplots(2, 1, figsize=(10, 8))
    
    while rclpy.ok():
        # Snapshots to avoid race conditions
        t_bag = list(checker.times_bag)
        v_bag = list(checker.bag_vel)
        w_bag = list(checker.bag_ang)
        
        t_mpc = list(checker.times_mpc)
        v_mpc = list(checker.mpc_vel)
        w_mpc = list(checker.mpc_ang)
        
        t_real = list(checker.times_real)
        v_real = list(checker.real_vel)
        w_real = list(checker.real_ang)

        if len(t_bag) > 0 or len(t_mpc) > 0:
            ax[0].cla()
            ax[0].plot(t_bag, v_bag, label='Bag Cmd (linear.x)', alpha=0.5)
            ax[0].plot(t_mpc, v_mpc, label='MPC Cmd (linear.x)', alpha=0.9)
            ax[0].plot(t_real, v_real, label='Real Vel', color='black', linewidth=1.5, alpha=0.6)
            ax[0].set_ylabel('Velocity [m/s]')
            ax[0].legend()
            ax[0].grid(True)
            ax[0].set_title('Velocity Comparison')

            ax[1].cla()
            # Right+ Throughout: Plot raw Right+ values to match raw IMU
            ax[1].plot(t_bag, w_bag, label='Bag Cmd (Right+)', alpha=0.5)
            ax[1].plot(t_mpc, w_mpc, label='MPC Cmd (Right+)', alpha=0.9)
            ax[1].plot(t_real, w_real, label='Real Angular (Right+)', color='black', linewidth=1.5, alpha=0.6)
            ax[1].set_ylabel('Angular Vel [rad/s]')
            ax[1].set_xlabel('Time [s]')
            ax[1].legend()
            ax[1].grid(True)
            
            plt.draw()
            plt.pause(0.01)
        time.sleep(0.1)

def main():
    rclpy.init()
    node = MPCChecker()
    
    t = threading.Thread(target=plot_thread, args=(node,))
    t.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
