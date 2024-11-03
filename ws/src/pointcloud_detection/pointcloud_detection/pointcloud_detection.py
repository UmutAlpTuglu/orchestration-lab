# colcon build --symlink-install
# source install/setup.bash
# ros2 pkg list | grep pointcloud_detection
# ros2 run pointcloud_detection pointcloud_detection


import rclpy
from rclpy.node import Node
from time import sleep

from sensor_msgs.msg import PointCloud2

class PointcloudDetectionDummy(Node):
    def __init__(self):
        super().__init__('object_detection_dummy')
        self.get_logger().info("Starting Object Detection Node ...")

        # Test Parameter Configuration
        self.declare_parameter("test", 2.0)
        test_config_value = self.get_parameter("test").value
        self.get_logger().info(f'Test Config Value: {test_config_value}')

        # Subscription for pointcloud topic
        self.declare_parameter("subscribed_pointcloud_name", 'pointcloud')
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("subscribed_pointcloud_name").value,  
            self.callback,
            10)
        
        # Publisher for object list
        self.declare_parameter("published_object_list_name", 'object_list')
    
    def callback(self, msg):
        self.get_logger().info("Processing pointclouds and publishing object list...")
        sleep(3)

def main(args=None):
    rclpy.init(args=args)
    object_list_publisher = PointcloudDetectionDummy()
    rclpy.spin(object_list_publisher)
    object_list_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
