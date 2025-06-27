import rclpy
import rclpy.node
import rclpy.qos
from rosgraph_msgs.msg import Clock
from rosbag2_interfaces.srv import SplitBagfile
from rosbag2_interfaces.msg import ReadSplitEvent
from asyncio import Future
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header


def call_split_bagfile():
    rclpy.init()
    node = rclpy.create_node("call_split_bagfile")
    cli = node.create_client(SplitBagfile, "split_bagfile")
    try:
        if not cli.wait_for_service(5.0):
            node.get_logger().error("Service not available, exiting...")
        else:
            req = SplitBagfile.Request()
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            node.get_logger().info("Bagfile split")
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


def wait_for_read_split():
    rclpy.init()
    node = rclpy.create_node("wait_for_read_split")
    future = Future()
    node.create_subscription(
        ReadSplitEvent,
        "events/read_split",
        lambda _: node.get_logger().info("Received split event, exiting...")
        and future.set_result(None),
        rclpy.qos.qos_profile_services_default,
    )
    try:
        rclpy.spin_until_future_complete(node, future)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


class WaitForBagEnd(rclpy.node.Node):
    def __init__(self):
        super().__init__("wait_for_bag_end")
        self.declare_parameter("timeout", 1.0)
        self.timeout = self.get_parameter("timeout").get_parameter_value().double_value
        self.timed_out = False
        self.future = Future()
        self.create_subscription(
            Clock, "/clock", self.clock_callback, rclpy.qos.qos_profile_sensor_data
        )
        self.timer = None
        self.get_logger().info(
            f"Waiting for bag to start, timeout set to {self.timeout:.4f}s"
        )

    def clock_callback(self, _):
        self.timed_out = False
        if self.timer is None:
            self.timer = self.create_timer(self.timeout, self.timer_callback)
            self.get_logger().info("Timer started")

    def timer_callback(self):
        if self.timed_out:
            self.get_logger().info(
                f"Clock timed out after {self.timeout:.4f}s, exiting..."
            )
            self.future.set_result(None)
        else:
            self.timed_out = True


def wait_for_bag_end():
    rclpy.init()
    node = WaitForBagEnd()
    try:
        rclpy.spin_until_future_complete(node, node.future)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


def dummy_publisher():
    rclpy.init()
    node = rclpy.create_node("dummy_publisher")
    _ = node.create_publisher(Header, "dummy", rclpy.qos.qos_profile_services_default)
    node.get_logger().info("Pretending to publish messages")
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


def reset_kalman_origin():
    def callback(msg, logger, pub, template):
        logger.info(f"Received origin reset request from: {msg.frame_id}")
        template.header.stamp = msg.stamp
        pub.publish(template)

    rclpy.init()
    node = rclpy.create_node("reset_kalman_origin")
    node.declare_parameter("frame_odom", "odom")
    pub = node.create_publisher(
        PoseWithCovarianceStamped, "set_pose", rclpy.qos.qos_profile_services_default
    )
    msg_out = PoseWithCovarianceStamped()
    msg_out.header.frame_id = (
        node.get_parameter("frame_odom").get_parameter_value().string_value
    )
    msg_out.pose.pose.orientation.w = 1.0
    node.create_subscription(
        Header,
        "reset_origin",
        lambda msg_in: callback(msg_in, node.get_logger(), pub, msg_out),
        rclpy.qos.qos_profile_services_default,
    )
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
