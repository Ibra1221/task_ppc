import rclpy
from rclpy.node import Node 
from custom_interface.srv import Plan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
class GlobalPlanner(Node):
    def __init__(self):
        super().__init__("global_planner")
        self.robot_pose = None
        self.service = self.create_service(Plan, "/create_plan", self.global_planner)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_handler, 10)
        self.path_pub = self.create_publisher(Path, "/plan", 10)
    def global_planner(self, request, response):
        self.get_logger().info(f"Global Planner received goal: {request.goal}")
        
        if self.robot_pose is None:
            self.get_logger().error("Odometry data not received yet!")
            return response  

        self.get_logger().info("In Global Planner, computing path...")

        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = self.robot_pose

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose = request.goal  

        response.path.header.stamp = self.get_clock().now().to_msg()
        response.path.header.frame_id = "map"
        response.path.poses.append(current_pose)
        response.path.poses.append(goal_pose)

        if response.path.poses:
            self.get_logger().info("Path successfully generated, publishing now!")
            self.path_pub.publish(response.path)
        else:
            self.get_logger().error("Failed to generate path!")

        return response

    def odom_handler(self, msg: Odometry):
        self.robot_pose = msg.pose.pose  

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
