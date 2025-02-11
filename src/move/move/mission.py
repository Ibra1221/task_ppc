import rclpy
from custom_interface.srv import Mission
from rclpy.node import Node
from std_msgs.msg import String
class MissionService(Node):
    def __init__(self):    
        super().__init__("mission_service")
        self.service = self.create_service(Mission,
        "start_mission", 
        callback= self.service_handler)
        self.get_logger().info("Service Started")
        self.accepted = ["Stop", "GoTo"]
        self.pub = self.create_publisher(String,"/mission", 10)
    def service_handler(self, Request, Response):
        msg = String()
        self.get_logger().info(f"Received Request: \nname : {Request.name.data}")
        Response.status = False
        if str(Request.name.data) in self.accepted:
            self.get_logger().info(f" {Request.name.data} is accepted")
            Response.status = True
            msg.data = Request.name.data
            self.pub.publish(msg)
        return Response
def main(args=None):
    rclpy.init(args=args)
    node = MissionService()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
        