import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
import rclpy.time
from packaging import version

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from scipy.spatial.transform import Rotation

# TF2
import tf2_ros
import tf2_geometry_msgs


class ObjectLocatorNode(Node):

    def __init__(self):
        super().__init__('aruco_test')

        # --- Parameters ---
        self.marker_size_m = 0.05  # CHANGE THIS to your marker size
        self.get_logger().info(f"Using marker size: {self.marker_size_m} meters")
        
        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # CameraInfo storage
        self.cam_info = None
        self.intrinsic_matrix = None
        self.dist_coeffs = None

        # TF2 Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- YOLO & ArUco Setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.cv_bridge = CvBridge()

        # Latched QoS
        cam_info_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE  # TRANSIENT_LOCAL
        )

        # Camera Info subscription
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.cam_info_callback,
            cam_info_qos
        )

        # Image subscription
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # --- Publisher ---
        self.viz_marker_pub = self.create_publisher(
            MarkerArray,
            '/localized_objects',
            10
        )
        
        self.get_logger().info("Creating timer")
        
        # self.timer = self.create_timer(1.0, self.publish_marker_for_id)
        self.get_logger().info("Object locator node started. Waiting for CameraInfo...")
        
    def publish_marker_for_id(self, marker_id, pose_map):
        """
        Publishes a text marker at the detected marker's map-frame pose.
        """
        marker_array = MarkerArray()
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "aruco_ids"
        marker.id = int(marker_id)  # Use marker ID so they don’t overwrite each other
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position from pose_map
        marker.pose = pose_map
        self.get_logger().info(f"pose_map: {pose_map}")SSSS

        # Slightly above ground
        marker.pose.position.z += 0.2

        marker.text = f"ID{marker_id}"
        self.get_logger().info(f"marker_id: {marker_id}")

        marker.scale.z = 0.25     # Text height
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.lifetime = Duration(seconds=0).to_msg()

        marker_array.markers.append(marker)
        self.viz_marker_pub.publish(marker_array)


    def cam_info_callback(self, msg):
        """
        Callback to receive and store camera calibration data.
        """
        if self.cam_info is None:
            self.cam_info = msg
            self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("CameraInfo received successfully!")
            self.destroy_subscription(self.cam_info_sub)
            self.get_logger().info(f"cam_info size: {self.cam_info.width, self.cam_info.height}")

    def image_callback(self, msg):
        """
        Main callback. Runs ArUco and YOLO, matches, transforms, and publishes.
        """
        if self.cam_info is None:
            self.get_logger().warn(
                "No CameraInfo received yet. Skipping image processing.",
                throttle_duration_sec=5
            )
            return

        try:
            self.get_logger().info(f"msg size: {msg.width, msg.height}")
            self.get_logger().info("In try..")
            if msg.encoding.lower() == 'nv21':
                self.get_logger().info("nv21..")
                self.get_logger().info("Handling NV21 image")

                # Image dimensions
                h = msg.height
                w = msg.width

                # NV21 = Y plane + interleaved VU plane
                nv21 = np.frombuffer(msg.data, dtype=np.uint8)
                nv21 = nv21.reshape((h * 3 // 2, w))

                # Convert NV21 → BGR
                cv_image = cv2.cvtColor(nv21, cv2.COLOR_YUV2BGR_NV21)

            else:
                # Normal encodings (rgb8, bgr8)
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
                # cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

            # --- ArUco Detection ---
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            if version.parse(cv2.__version__) >= version.parse("4.7.0"):
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
                detectorParams = cv2.aruco.DetectorParameters()
                detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
                corners, ids, rejected = detector.detectMarkers(gray)
                self.get_logger().info(f"[Aruco ≥4.7] corners={len(corners)}, ids={ids}")
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray,
                    self.aruco_dict,
                    parameters=self.aruco_params
                )
                self.get_logger().info(f"[Aruco <4.7] corners={len(corners)}, ids={ids}")

            marker_data = {}
            if ids is None or len(corners) == 0:
                return

            marker_data = {}

        # ===============================
        # Pose estimation
        # ===============================
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]

            # --- Get 2D center ---
                cx = int(np.mean(marker_corners[:, 0]))
                cy = int(np.mean(marker_corners[:, 1]))

            # --- Fix for missing estimatePoseSingleMarkers ---
                try:
                # Try the built-in pose estimator
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i],
                        self.marker_size_m,
                        self.intrinsic_matrix,
                        self.dist_coeffs
                    )
                    rvec = rvecs[0]
                    tvec = tvecs[0]
                    if abs(tvec[2][0]) < 1e-6:
                        self.get_logger().warn("Invalid pose: Z ≈ 0, skipping marker")
                        continue
                    self.get_logger().info(f"tvec raw: {tvec.flatten()}")
                    
                except AttributeError:
                # =========================================
                # OpenCV didn’t include estimatePose...
                # Use solvePnP manually
                # =========================================
                    obj_points = np.array([
                        [-0.5,  0.5, 0],
                        [ 0.5,  0.5, 0],
                        [ 0.5, -0.5, 0],
                        [-0.5, -0.5, 0]
                    ]) * self.marker_size_m

                    retval, rvec, tvec = cv2.solvePnP(
                        obj_points,
                        marker_corners,
                        self.intrinsic_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_ITERATIVE
                        # flags=cv2.SOLVEPNP_SQPNP
                        # flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    

            # Build PoseStamped relative to camera
                
                pose_cam = self.rvec_tvec_to_pose_stamped(
                    tvec, rvec, msg.header
                )
                pose_cam.header.frame_id = "camera_optical_frame"
                # distance
                self.get_logger().info(f"Z={tvec[2][0]:.3f} m | rnorm={np.linalg.norm(rvec):.3f}")
                self.get_logger().info(f"Image frame: {msg.header.frame_id}")
                self.get_logger().info(f"Pose frame: {pose_cam.header.frame_id}")

            # ===============================
            # Transform to map frame
            # ===============================
                try:
                    
                    transform = self.tf_buffer.lookup_transform(
                        "map", "camera_optical_frame", rclpy.time.Time()
                        #"map", msg.header.frame_id, rclpy.time.Time()
                    )
                    self.get_logger().info(
                        f"Optical_frame in map: "
                        f"x={transform.transform.translation.x}, "
                        f"y={transform.transform.translation.y}, "
                        f"z={transform.transform.translation.z}"
                    )
                    pose_map = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, transform)

                    marker_data[marker_id] = {
                        "map_pose": pose_map.pose,
                        "center": (cx, cy)
                    }
                    self.publish_marker_for_id(marker_id, pose_map.pose)

                except Exception as e:
                    self.get_logger().warn(f"TF transform error: {e}")

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

        
    def rvec_tvec_to_pose_stamped(self, tvec, rvec, header):
        """
        Converts OpenCV rvec and tvec to a geometry_msgs/PoseStamped.
        """
        r = Rotation.from_rotvec(rvec.flatten())
        quat = r.as_quat()

        pose_stamped = PoseStamped()
        pose_stamped.header = header

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

