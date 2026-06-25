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
    
class ObjectLocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')

        # --- Parameters ---
        # This is CRITICAL. Measure your ArUco marker side length in meters.
        self.declare_parameter('marker_size', 0.05)  # Default: 5cm
        self.marker_size_m = self.get_parameter('marker_size').value
        self.get_logger().info(f"Using marker size: {self.marker_size_m} meters")

        # --- YOLO & ArUco Setup ---
        self.get_logger().info("Loading YOLOv8n model...")
        self.yolo_model = YOLO('yolov8n.pt')  # 'n' is the fastest model
        self.get_logger().info("YOLO model loaded.")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.cv_bridge = CvBridge()

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
            durability=DurabilityPolicy.VOLATILE # TRANSIENT_LOCAL
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
        
        # --- Publisher ---
        self.viz_marker_pub = self.create_publisher(
            MarkerArray,
            '/localized_objects',
            10
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
            
            # --- Part 1: ArUco Detection & Pose Estimation ---
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Suggestion from forums -- my version is 4.12.0
            if version.parse(cv2.__version__) >= version.parse("4.7.0"):
            	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            	detectorParams = cv2.aruco.DetectorParameters()
            	detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
            	corners, ids, rejected = detector.detectMarkers(gray)
        	
            else:
            	corners, ids, rejected = cv2.aruco.detectMarkers(
		    gray,
		    self.aruco_dict,
		    parameters=self.aruco_params
		)
		
            marker_data = {}  # Stores {id: {'map_pose': Pose, 'center': (cx, cy)}}

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size_m, self.intrinsic_matrix, self.dist_coeffs
                )

                for i, marker_id in enumerate(ids.flatten()):
                    # Get 2D center point for matching with YOLO
                    marker_corners = corners[i][0]
                    cx = int(np.mean(marker_corners[:, 0]))
                    cy = int(np.mean(marker_corners[:, 1]))

                    # Convert rvec/tvec to a PoseStamped message
                    pose_stamped_cam = self.rvec_tvec_to_pose_stamped(
                        tvecs[i], rvecs[i], msg.header
                    )

                    # --- Part 2: Transform Pose to Map Frame ---
                    try:
                        target_frame = 'map'
                        source_frame = msg.header.frame_id  # e.g., 'camera_link'
                        
                        transform = self.tf_buffer.lookup_transform(
                            target_frame, source_frame, rclpy.time.Time()
                        )
                        
                        pose_stamped_map = tf2_geometry_msgs.do_transform_pose(
                            pose_stamped_cam, transform
                        )
                        
                        marker_data[marker_id] = {
                            'map_pose': pose_stamped_map.pose,
                            'center': (cx, cy)
                        }

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                            tf2_ros.ExtrapolationException) as e:
                        self.get_logger().warn(f"TF transform error: {e}", throttle_duration_sec=5)
            
            # --- Part 3: YOLO Detection ---
            yolo_results = self.yolo_model.predict(cv_image, verbose=False)

            for box in yolo_results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0].cpu().numpy())
                class_name = self.yolo_model.names[cls_id]

                # --- Part 4: Match YOLO box with ArUco marker ---
                matched_id = None
                for marker_id, data in marker_data.items():
                    cx, cy = data['center']
                    # Check if the marker's center is inside the YOLO box
                    if x1 < cx < x2 and y1 < cy < y2:
                        matched_id = marker_id
                        break
                
                # --- Part 5: Create Visualization Marker ---
                if matched_id is not None:
                    self.get_logger().info(f"Found '{class_name}' at ArUco marker {matched_id}")

                    viz_marker = Marker()
                    viz_marker.header.frame_id = 'map'
                    viz_marker.header.stamp = self.get_clock().now().to_msg()
                    viz_marker.ns = "yolo_aruco_objects"
                    viz_marker.id = int(matched_id)  # Use marker ID as RViz ID
                    viz_marker.type = Marker.TEXT_VIEW_FACING
                    viz_marker.action = Marker.ADD
                    
                    # Set pose from our transformed data
                    viz_marker.pose = marker_data[matched_id]['map_pose']
                    # Offset the text to be slightly above the marker
                    viz_marker.pose.position.z += 0.1  
                    
                    viz_marker.text = class_name
                    
                    viz_marker.scale.z = 0.1  # Text height in meters
                    viz_marker.color.a = 1.0
                    viz_marker.color.r = 1.0
                    viz_marker.color.g = 1.0
                    viz_marker.color.b = 1.0
                    
                    # Make the marker last for 2 seconds
                    viz_marker.lifetime = Duration(seconds=2).to_msg()
                    
                    output_viz_markers.markers.append(viz_marker)

                    # Remove this marker so it's not matched twice
                    del marker_data[matched_id]
            
            # --- Part 6: Publish all found markers ---
            if output_viz_markers.markers:
                self.viz_marker_pub.publish(output_viz_markers)

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'Main callback error: {e}')

    def rvec_tvec_to_pose_stamped(self, tvec, rvec, header):
        """
        Converts OpenCV rvec and tvec to a geometry_msgs/PoseStamped.
        """
        # Convert rotation vector to quaternion
        r = Rotation.from_rotvec(rvec.flatten())
        quat = r.as_quat()  # Returns (x, y, z, w)
        
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        
        # tvec is [[x], [y], [z]] or [x,y,z], flatten to be safe
        tvec_flat = tvec.flatten()
        pose_stamped.pose.position.x = tvec_flat[0]
        pose_stamped.pose.position.y = tvec_flat[1]
        pose_stamped.pose.position.z = tvec_flat[2]
        
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]
        
        return pose_stamped


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
