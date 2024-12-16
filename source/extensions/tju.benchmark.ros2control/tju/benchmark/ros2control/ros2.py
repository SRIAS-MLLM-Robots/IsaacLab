import rclpy
from rclpy.node import Node
from std_msgs.msg import String
    
# Define a ROS2 node
class IsaacSimNode(Node):
    def __init__(self):
        super().__init__('echo_node')
        self.subscription = self.create_subscription(
            String,
            'ros2_control/command_topic',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'ros2_control/observation_topic', 10)
        self.simulation_running = False
        self.simulation_time = 0.0

    def listener_callback(self, msg):
        if self.simulation_running:
            # 仅在仿真运行时处理消息
            self.get_logger().info(f'Received: "{msg.data}" at simulation time {self.simulation_time:.2f}s')


    def publish_state(self):
        # 发布回复
        # 构造回复消息，附带仿真时间
        response = String()
        response.data = f'Echo: simulation time {self.simulation_time:.2f}s'
        self.publisher.publish(response)
        self.get_logger().info(f'Published: "{response.data}"')

    def update_simulation_state(self, running, current_time):
        self.simulation_running = running
        self.simulation_time = current_time
