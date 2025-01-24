# Intelligent Robotics
# Assignment Number 2 
# Topic: IRN

# Name: Johanna Vaske (Chetan)
# Matrikelnummer: 1067512


import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SpawnEntity
import math
from collections import deque

from ament_index_python.packages import get_package_share_directory


class InRowNavigation(Node):
    def __init__(self):
        super().__init__("in_row_navigation")
        self.get_logger().info("Node initialized")

        self.spawn_robot_client = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.spawn_robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("set entity service not available, waiting again...")

        TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
        model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
        urdf_path = os.path.join(
            get_package_share_directory("turtlebot3_gazebo"),
            "models",
            model_folder,
            "model.sdf",
        )
        with open(urdf_path) as f:
            urdf_string = f.read()
        pose = Pose(
            position=Point(x=-4.0, y=-0.7), orientation=Quaternion(z=-0.3, w=2.62)
        )
        # self.spawn_entity_request = SpawnEntity.Request(name="burger", xml=urdf_string, robot_namespace=f"epoch_{epoch}",initial_pose=pose, reference_frame="world")
        self.spawn_entity_request = SpawnEntity.Request(
            name="burger", xml=urdf_string, initial_pose=pose, reference_frame="world"
        )

        future = self.spawn_robot_client.call_async(
                self.spawn_entity_request
            )
        rclpy.spin_until_future_complete(self, future)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        
        # Subscription for LaserScan data
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.read_scan, qos_profile=qos_policy)
        self.scan_sub_2 = self.create_subscription(LaserScan, "/scan2", self.read_scan_2, qos_profile=qos_policy)

        self.control_timer = self.create_timer(0.5,self.control_robot)
        self.left_queue = deque(maxlen=5)
        self.left_2_queue = deque(maxlen=5)
        self.right_queue = deque(maxlen=5)
        self.right_2_queue = deque(maxlen=5)

        # PID parameters
        self.Kp = 0.3  # Proportional gain
        self.Ki = 0.0 # Integral gain 
        self.Kd = 0.15  # Derivative gain 
        self.Tp = 0.2 # Base forward speed (m/s)
        self.integral = 0.0
        self.lastError = 0.0

    def read_scan(self, msg: LaserScan):

        if msg.ranges[0] > msg.range_max: msg.ranges[0] = msg.range_max
        if msg.ranges[1] > msg.range_max: msg.ranges[1] = msg.range_max
        self.left_queue.append(msg.ranges[0])
        self.right_queue.append(msg.ranges[1])
        

    def read_scan_2(self, msg: LaserScan):
        if msg.ranges[0] > msg.range_max: msg.ranges[0] = msg.range_max
        if msg.ranges[1] > msg.range_max: msg.ranges[1] = msg.range_max

        self.left_2_queue.append(msg.ranges[0]) 
        self.right_2_queue.append(msg.ranges[1])
        


    def stop_robot(self):
        msg_pub = Twist()
        msg_pub.linear.x = 0.0
        msg_pub.angular.z = 0.0
        self.cmd_vel_pub.publish(msg_pub)

    def control_robot(self):
        if not self.left_queue or not self.left_2_queue or not self.right_2_queue or not self.right_queue:
            return
        
        left_avg = sum(self.left_queue) / len(self.left_queue)
        left_2_avg = sum(self.left_2_queue) / len(self.left_2_queue)
        right_avg = sum(self.right_queue) / len(self.right_queue)
        right_2_avg = sum(self.right_2_queue) / len(self.right_2_queue)
        
        factor_left = left_avg / left_2_avg
        factor_right = right_avg / right_2_avg

        direction = (left_avg - right_avg) / abs(left_avg - right_avg)

        error = left_avg * factor_left - right_avg * factor_right
        if error > 4: error = 4
        print("error", error)
        print("right_avg", right_avg)
        print("right_2_avg",right_2_avg)
        print("left_avg", left_avg)
        print("left_2_avg", left_2_avg)
        msg_pub = Twist()

        # PID control: calculate correction based on alignment error
        self.integral += error
        derivative = abs(error) - abs(self.lastError)
        print("derv: ", derivative) 
        if derivative > 1.0: derivative = 1.0
        if derivative < -1.0: derivative = -1.0
        turn = (self.Kp * abs(error)) + (self.Ki * self.integral) + (self.Kd * derivative)
        turn = turn * direction
        self.lastError = error

        # Set linear speed and adjust angular speed using PID
        msg_pub.linear.x = self.Tp
        msg_pub.angular.z = turn
        self.cmd_vel_pub.publish(msg_pub)

def main(args=None):
    rclpy.init(args=args)
    node = InRowNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

