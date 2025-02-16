import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interface.srv import Plan
from custom_interface.msg import MissionMsg
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from custom_interface.action import Navigate
class BehaviorNode(Node):
    def __init__(self):
        super().__init__("behavior_node")
        self.sub = self.create_subscription(MissionMsg, "/mission", self.mission_handler, 10)
        self.state_pub = self.create_publisher(String, "/state", 10)
        self.client = self.create_client(Plan, "/create_plan")
        self.planner_pub = self.create_publisher(Path, "/global_plan", 10)
        self.action_client = ActionClient(self, Navigate, "/navigate")
        self.state = String()
        self.state.data = "idle"
        self.state_pub.publish(self.state)
        self.get_logger().info("Behavior Node Active.")
    def mission_handler(self, msg: MissionMsg):
        mission = msg.name.data
        if mission == "GoTo":
            self.state.data = "create_path"
            self.state_pub.publish(self.state)
            self.get_logger().info("Sending request to planner")
            self.sender(msg.target) 
            self.state.data = "navigate"
            self.state_pub.publish(self.state)
        else:
            self.state.data = "idle"
            self.state_pub.publish(self.state)
    def sender(self, target):
        if not self.client.wait_for_service(4.0):
            self.get_logger().error("Service Offline...")
            return
        request = Plan.Request()
        request.goal = target
        self.get_logger().info("Sending Request")
        future = self.client.call_async(request)
        future = self.client.call_async(request)
        if future is None:
            self.get_logger().error("Failed to call service asynchronously.")
        else:
            self.get_logger().info("Request successfully sent, awaiting response.")
        future.add_done_callback(self.plan_response_callback)

        self.get_logger().info("Request sent!")
    
        self.get_logger().info(f"Future state before callback: {future.done()}")  
        future.add_done_callback(self.plan_response_callback) 
    def plan_response_callback(self, future):
        self.get_logger().info("Inside plan_response_callback")  

        try:
            self.response = future.result()
            
            if self.response:
                self.get_logger().info(f"Received response: {self.response}")
                
                if self.response.path.poses:  
                    self.get_logger().info("Publishing path to /global_plan")
                    self.planner_pub.publish(self.response.path)
                else:
                    self.get_logger().error("Received an empty path from planner.")
            else:
                self.get_logger().error("Received an empty response from planner.")
        
        except Exception as e:
            self.get_logger().error(f"Error in plan_response_callback: {str(e)}")
        self.send_goal()


    def send_goal(self):
        self.goal = Navigate.Goal()
        self.goal.path = self.response.path
        self.action_client.wait_for_server()
        future =self.action_client.send_goal_async(self.goal, self.feedback_callback)
        future.add_done_callback(self.future_handle)
    def future_handle(self,future):
        self.goal_handle = future.result()
        if not self.goal_handle:
            self.get_logger().info("Goal Rejected.")
            return
        else:
            self.get_logger().info("Goal Accepted.")
        self.get_result_future = self.goal_handle.get_result_async()   
        self.get_result_future.add_done_callback(self.result_callback)     
    def result_callback(self,result_future):
        result = result_future.result().status
        self.get_logger().info(f"The received status is : {result}")
    def feedback_callback(self, goal_handle):
        feedback = goal_handle.feedback
        print(f"The remaining distance is {feedback.distance}")

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


