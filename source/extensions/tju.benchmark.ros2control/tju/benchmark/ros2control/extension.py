import omni.ext
import omni.timeline
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .ros2 import IsaacSimNode

class Ros2ControlExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[my_ros_extension] Startup")

        # 初始化 rclpy
        rclpy.init()

        # 创建 ROS2 节点
        self.node = IsaacSimNode()


        app = omni.kit.app.get_app()

        self.event_stream = app.get_update_event_stream()
        
        self.timeline = omni.timeline.get_timeline_interface()

        # 订阅PhysicsStepEvent，而不是UpdateEvent
        self._physics_sub = self.event_stream.create_subscription_to_pop(
            self.on_update,
        )


    def on_update(self, dt):
        # 获取仿真是否在运行
        playing = self.timeline.is_playing()

        # 获取当前帧数和仿真时间
        current_time = self.timeline.get_current_time()


        # 更新 ROS2 节点的仿真状态和时间
        self.node.update_simulation_state(playing, current_time)
        if playing:
            # 如果仿真在运行，处理消息
            self.node.publish_state()
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def on_shutdown(self):
        print("[my_ros_extension] Shutdown")
        if rclpy.ok():
            rclpy.shutdown()
        # 移除回调
        if hasattr(self, '_handler'):
            self.event_stream.unsubscribe(self._physics_sub)

