import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
import time
import subprocess

class Diagnostics(Node):
    def __init__(self):
        super().__init__('diag_node')
        self.counts = {}
        self.topics = [
            ('/cmd_vel_bag', Twist),
            ('/cmd_vel_mpc', Twist),
            ('/vectornav/velocity_body', TwistWithCovarianceStamped),
            ('/e2e_planner/path', Path),
            ('/current_pose', PoseStamped)
        ]
        
        for topic, msg_type in self.topics:
            self.counts[topic] = 0
            self.create_subscription(msg_type, topic, self.make_callback(topic), 10)

    def make_callback(self, topic):
        def cb(msg):
            self.counts[topic] += 1
        return cb

def main():
    rclpy.init()
    node = Diagnostics()
    print("Checking topics for 5 seconds...")
    start = time.time()
    while time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("\nTopic Counts and Info:")
    for topic, count in node.counts.items():
        # Get publisher count via CLI
        try:
            res = subprocess.check_output(['ros2', 'topic', 'info', topic], encoding='utf-8')
            pub_info = [line for line in res.split('\n') if 'Publisher count' in line]
            pub_count = pub_info[0].split(':')[-1].strip() if pub_info else "Unknown"
        except:
            pub_count = "Error"
            
        print(f"  {topic}: {count} msgs (Pub count: {pub_count})")
    
    rclpy.shutdown()

def main():
    rclpy.init()
    node = Diagnostics()
    print("Checking topics for 5 seconds...")
    start = time.time()
    while time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("\nTopic Counts:")
    for topic, count in node.counts.items():
        print(f"  {topic}: {count} msgs")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
