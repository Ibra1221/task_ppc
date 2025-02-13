import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interface.srv import Plan
from custom_interface.msg import MissionMsg
from nav_msgs.msg import Path
class BehaviorNode(Node):
    def __init__(self):
        super().__init__("behavior_node")
        self.sub = self.create_subscription(MissionMsg, "/mission", self.mission_handler,10)
        self.state_pub = self.create_publisher(String, "/state",10)
        self.state = String()
        self.state.data = "idle"
        self.state_pub.publish(self.state)
        self.client = self.create_client(Plan,"/create_plan")
        self.planner_pub = self.create_publisher(Path,"/global_plan",10)
        self.get_logger().info("Behavior Node Active.")
    def mission_handler(self, msg:MissionMsg):
        mission = msg.name.data
        if (mission == "GoTo"):
            self.state.data = "create_path"
            self.state_pub.publish(self.state)
            response = self.sender(msg.target)
            self.planner_pub.publish(response)
            self.state.data = "navigate"
            self.state_pub.publish(self.state)

        else:
            self.state.data = "idle"
        self.state_pub.publish(self.state)
    def sender(self,target):
        while not self.client.wait_for_service(1.0):
            self.get_logger().info("Service Offline...")
        self.Request = Plan.Request()
        self.Request.goal = target
        self.future = self.client.call_async(self.Request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__=="__main__":
    main()

