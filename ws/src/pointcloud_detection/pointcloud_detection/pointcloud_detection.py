# BUILD process, sourcing in every new terminal:
# colcon build
# . install/setup.bash 
# TEST in docker dev in three terminals and source all of them
#1 ros2 run pointcloud_detection pointcloud_detection
#2 ros2 topic pub /pointcloud sensor_msgs/msg/PointCloud2 "{header: {frame_id: 'map'}, height: 1, width: 1, fields: [], is_bigendian: false, point_step: 1, row_step: 1, data: [0], is_dense: false}"
#3 ros2 topic echo /detected_objects 


import rclpy
from rclpy.node import Node
from time import sleep
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PointcloudDetectionDummy(Node):
    def __init__(self):
        super().__init__('pointcloud_detection_dummy')
        self.get_logger().info("Starting Pointcloud Detection Node ...")
        
        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('test', 2.0),
                ('subscribed_pointcloud_name', 'pointcloud'),
                ('published_detections_topic', 'detected_objects')
            ]
        )

        # Create subscription for pointcloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("subscribed_pointcloud_name").value,
            self.pointcloud_callback,
            10)
        
        # Create publisher for detected objects (using Marker for visualization)
        self.publisher = self.create_publisher(
            Marker,
            self.get_parameter("published_detections_topic").value,
            10)

        self.frame_count = 0
        
    def pointcloud_callback(self, msg):
        """Process incoming pointcloud and publish detected objects."""
        self.frame_count += 1
        self.get_logger().info(f"Received pointcloud frame {self.frame_count}")
        
        # Simulate some processing time
        sleep(1.0)
        
        # Create a marker for visualization
        marker = Marker()
        marker.header = msg.header
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        
        # Publish detection
        self.publisher.publish(marker)
        self.get_logger().info(f"Published detection marker for frame {self.frame_count}")

def main(args=None):
    rclpy.init(args=args)
    node = PointcloudDetectionDummy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
