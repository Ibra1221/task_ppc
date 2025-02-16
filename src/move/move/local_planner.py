import rclpy
from rclpy.node import Node 
from custom_interface.srv import Plan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from custom_interface.action import Navigate
from custom_interface.msg import MissionMsg
from rclpy.action import ActionServer
import math
from std_msgs.msg import String
class LocalPlanner(Node):
    def __init__(self):
        super().__init__("local_planner")
        self.odom_sub = self.create_subscription(Odometry, "/odom",self.odom_handler, 10)
        self.server = ActionServer(self, Navigate,"/navigate", self.move_callback)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel",10)
        self.result = Navigate.Result()
        self.result.status = False
        self.robot_pose = None
        self.velocity = Twist()  
        self.state_pub = self.create_publisher(String,"/state",10)
        self.state = "navigate"
        self.msg = String()
    def move_callback(self,goal_handle):
        self.get_logger().info("Within Local Planner")
        if self.robot_pose is None:
            self.get_logger().error("Odometry data not received yet!")
            goal_handle.abort()
            return self.result
        self.goal = goal_handle.request.path.poses[1].pose.position
        self.current = self.robot_pose.position
        goal_x = self.goal.x
        goal_y = self.goal.y
        current_x = self.current.x
        current_y = self.current.y
        if goal_x == current_x:
            gradient = float('inf')  
        else:
            gradient = (goal_y - current_y) / (goal_x - current_x)
        while(gradient > 1.25 or gradient < 0.75):
            self.velocity.angular.z = 1.0
            self.velocity.linear.x=0
            self.velocity.linear.y=0
            self.vel_pub.publish(self.velocity)
            self.current = self.robot_pose.position
            current_x = self.current.x
            current_y = self.current.y
            if goal_x == current_x:
                gradient = float('inf')  
            else:
                gradient = (goal_y - current_y) / (goal_x - current_x)
        self.distance = math.sqrt(pow(goal_y-current_y,2) + pow(goal_x-current_x,2))
        while(self.distance>0.1):
            self.velocity.angular.z = 0
            self.velocity.linear.x=1.0
            self.vel_pub.publish(self.velocity)
            self.current = self.robot_pose.position
            current_x = self.current.x
            current_y = self.current.y
            self.distance = math.sqrt(pow(goal_y-current_y,2) + pow(goal_x-current_x,2))
        self.velocity.linear.x=0
        self.vel_pub.publish(self.velocity)
        self.result.status = True
        self.state = "idle"
        self.msg = self.state
        self.state_pub.publish(self.msg)
        goal_handle.succeed()
        return self.result
    def odom_handler(self, msg:Odometry):
        self.robot_pose = msg.pose.pose
def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__=="__main__":
    main()