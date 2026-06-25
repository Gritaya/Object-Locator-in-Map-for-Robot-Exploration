import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# For TF2 transforms
import tf2_ros
import tf2_geometry_msgs


    # Make sure to finish camera calibration
    # sudo apt install ros-humble-aruco-ros
    # pip install ultralytics --> import to node
    
    # logic:
    # subscribes two topics from aruco_ros (/marker_array, /result)
    # image callback from /result to YOLO (model.predict(image)), when arcu_ros sees a marker
    # store latest poses{marker_id_5: pose_stamped,...}
    # check if aruco is in YOLO frame
    # find way to add object type into dictionaries??
    # have poses in 'camera_link' --> use 'tf2' to transform to 'map'(SLAM, acml)
    # publish final 'map' to 'visualization_msgs/Marker' with 'text' obj type
    
class ObjectLocatorNode(Node):

    def __init__(self):
        super().__init__('text_test')

        # --- Parameters ---
        # This is CRITICAL. Measure your ArUco marker side length in meters.
        self.declare_parameter('marker_size', 0.05)  # Default: 5cm
        self.marker_size_m = self.get_parameter('marker_size').value
        self.get_logger().info(f"Using marker size: {self.marker_size_m} meters")

        # --- Publisher ---
        self.viz_marker_pub = self.create_publisher(
            MarkerArray,
            '/localized_objects',
            10
        )
        
        self.get_logger().info("Creating timer")
        
        # self.timer = self.create_timer(1.0, self.publish_test_marker)

        self.get_logger().info("Object locator node started. Publishing test marker...")

    def publish_test_marker(self):
        """
        Publishes a single, permanent text marker to the map.
        """
        output_viz_markers = MarkerArray()
        
        viz_marker = Marker()
        viz_marker.header.frame_id = 'map'
        viz_marker.header.stamp = self.get_clock().now().to_msg()
        viz_marker.ns = "test_marker"
        viz_marker.id = 0
        viz_marker.type = Marker.TEXT_VIEW_FACING
        viz_marker.action = Marker.ADD
        
        # Set a test pose
        viz_marker.pose.position.x = 1.0
        viz_marker.pose.position.y = 1.0
        viz_marker.pose.position.z = 0.5  # 0.5m above the map floor
        viz_marker.pose.orientation.x = 0.0
        viz_marker.pose.orientation.y = 0.0
        viz_marker.pose.orientation.z = 0.0
        viz_marker.pose.orientation.w = 1.0
        
        viz_marker.text = "ID_i"
        
        viz_marker.scale.z = 0.2  # Text height in meters
        viz_marker.color.a = 1.0
        viz_marker.color.r = 1.0
        viz_marker.color.g = 1.0
        viz_marker.color.b = 0.0  # Yellow, for visibility
        
        # Make the marker last forever (Duration(seconds=0))
        viz_marker.lifetime = Duration(seconds=0).to_msg()
        
        output_viz_markers.markers.append(viz_marker)

        # --- Publish the test marker ---
        
        self.viz_marker_pub.publish(output_viz_markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
   
        

    
# map -> base_link transform (slam_toolbox and amcl)
# aruco subscribes cam's /image_raw, /camera_info
