import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
import rclpy.time

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from scipy.spatial.transform import Rotation

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
    
class YOLONode(Node):

    def __init__(self):
        super().__init__('yolo_test')

        # --- Parameters ---
        # This is CRITICAL. Measure your ArUco marker side length in meters.
        self.declare_parameter('marker_size', 0.05)  # Default: 5cm
        self.marker_size_m = self.get_parameter('marker_size').value
        self.get_logger().info(f"Using marker size: {self.marker_size_m} meters")

        # --- YOLO & ArUco Setup ---
        self.get_logger().info("Loading YOLOv8n model...")
        self.yolo_model = YOLO('yolov8n.pt')  # 'n' is the fastest model
        self.get_logger().info("YOLO model loaded.")

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Camera Info & Image Processing ---
        self.cam_info = None
        self.intrinsic_matrix = None
        self.dist_coeffs = None

        # This QoS profile is "latched" - it gets the last published message
        # This is perfect for the /camera_info topic
        cam_info_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE #TRANSIENT_LOCAL
        )

        # We MUST have camera info before we can process images
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',  # Make sure this matches your camera topic
            self.cam_info_callback,
            cam_info_qos
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Make sure this matches your camera topic
            self.image_callback,
            10  # QoS depth
        )

        self.get_logger().info("Object locator node started. Waiting for CameraInfo...")

    def cam_info_callback(self, msg):
        """
        Callback to receive and store camera calibration data.
        """
        if self.cam_info is None:
            self.cam_info = msg
            self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("CameraInfo received successfully!")
            # Once we have it, we don't need to subscribe anymore
            self.destroy_subscription(self.cam_info_sub)

    def image_callback(self, msg):
        """
        Main callback. Runs ArUco and YOLO, matches, transforms, and publishes.
        """
        # --- Guard Clause ---
        # Wait until we have calibration data
        if self.cam_info is None:
            self.get_logger().warn(
                "No CameraInfo received yet. Skipping image processing. "
                "Make sure your camera is calibrated and publishing.",
                throttle_duration_sec=5
            )
            return

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            output_viz_markers = MarkerArray()
                        
            # --- Part 3: YOLO Detection ---
            yolo_results = self.yolo_model.predict(cv_image, verbose=False)

            for box in yolo_results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0].cpu().numpy())
                class_name = self.yolo_model.names[cls_id]
                
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'Main callback error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
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
