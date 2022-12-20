import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'line_marker_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        marker1 = Marker()
        marker1.header.frame_id = "start-line-frame"
        marker1.id=0
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD
        marker1.scale.x = 0.2
        marker1.scale.y = 0.2
        marker1.scale.z = 0.2
        marker1.color.a = 1.0
        marker1.color.r = 1.0
        marker1.color.g = 0.0
        marker1.color.b = 0.0
        marker1.frame_locked = True
        marker1.pose.orientation.w = 1.0
        marker1.pose.position.x = 0.0
        marker1.pose.position.y = 0.0
        marker1.pose.position.z = 0.0
        
        marker2 = Marker()
        marker2.header.frame_id = "end-line_frame"
        marker2.id=1
        marker2.type = marker2.SPHERE
        marker2.action = marker2.ADD
        marker2.scale.x = 0.2
        marker2.scale.y = 0.2
        marker2.scale.z = 0.2
        marker2.color.a = 1.0
        marker2.color.r = 0.0
        marker2.color.g = 1.0
        marker2.color.b = 0.0
        marker2.frame_locked = True
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 0.0
        marker2.pose.position.y = 0.0 
        marker2.pose.position.z = 0.0 
        
        msg = MarkerArray()
        msg.markers.append(marker1)
        msg.markers.append(marker2)
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
