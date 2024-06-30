import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from rcl_interfaces.msg import ParameterDescriptor
import threading
import numpy as np
import math
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from threading import Event
from tutorial_interfaces.srv import PoseEstimation
from tutorial_interfaces.msg import PoseErr

class PoseEstimationService(Node):

    def __init__(self):
        super().__init__('pose_estimation_service')
        self.get_namespace()
        self.declare_parameter('dock.oak_offs', 0.23, ParameterDescriptor(description='Offset between camera and robot in m'))
        self.declare_parameter('dock.marker_length', 0.02, ParameterDescriptor(description='Side length of the Marker in m'))
        self.OAK_OFFS = self.get_parameter('dock.oak_offs').get_parameter_value().double_value
        self.MARKER_LENGTH = self.get_parameter('dock.marker_length').get_parameter_value().double_value
        callback_group1_ = MutuallyExclusiveCallbackGroup()
        self.image_subscriber_ = self.create_subscription(Image, "oakd/rgb/preview/image_raw", self.image_callback, 10, callback_group=callback_group1_)
        self.calibration_subscriber_ = self.create_subscription(CameraInfo, "oakd/rgb/preview/camera_info", self.calib_callback, 1)
        callback_groupserver_ = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(PoseEstimation, "pose_estimation", self.pose_estimation, callback_group=callback_groupserver_)
        self.got_image = Event()
        self._image_lock = threading.Lock()
        self.get_logger().info('pose estimation node started')

    def image_callback(self, msg):
        with self._image_lock:
            self._image = msg
        self.got_image.set()

    def calib_callback(self, msg):
        self.calib_data = msg
        self.get_logger().info('Unsubscribing from Calibration')
        self.mtx = np.array(self.calib_data.k).reshape((3, 3))
        self.dist = np.array(self.calib_data.d)
        self.destroy_subscription(self.calibration_subscriber_)

    def rot_mat2euler_angles(self, R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def pose_estimation(self, request, response):
        self.got_image.clear()
        self.get_logger().info('Waiting for image')
        self.got_image.wait()  # Wait for an image to be available
        self.get_logger().info('Image received')

        pose_err = PoseErr()
        bridge = CvBridge()
        pose_err.angular_error = float('inf')
        pose_err.x_err = float('inf')
        pose_err.y_err = float('inf')

        with self._image_lock:
            image_msg = self._image

        try:
            self.cvImg = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')  # Assuming the image is in color (bgr8)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
            response.got_pose = False
            return response

        objPoints = np.zeros((4, 1, 3), dtype=np.float32)
        objPoints[0, 0] = np.array([-self.MARKER_LENGTH / 2, self.MARKER_LENGTH / 2, 0])
        objPoints[1, 0] = np.array([self.MARKER_LENGTH / 2, self.MARKER_LENGTH / 2, 0])
        objPoints[2, 0] = np.array([self.MARKER_LENGTH / 2, -self.MARKER_LENGTH / 2, 0])
        objPoints[3, 0] = np.array([-self.MARKER_LENGTH / 2, -self.MARKER_LENGTH / 2, 0])

        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_1000)
        params = cv.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX

        #detector = cv.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = cv.aruco.detectMarkers(self.cvImg,aruco_dict, parameters=params)

        if ids is not None:
            cv.aruco.drawDetectedMarkers(self.cvImg, corners, ids)
            if request.id in ids:
                for i in range(len(ids)):
                    if ids[i] == request.id:
                        _, rvecs, tvecs = cv.solvePnP(objPoints, corners[i], self.mtx, self.dist)
                        cv.drawFrameAxes(self.cvImg, self.mtx, self.dist, rvecs, tvecs, 0.05)
            else:
                response.got_pose = False
                return response

            tvecs_mat = np.array([tvecs[0], tvecs[1], tvecs[2]])
            cam_R_aruco, _ = cv.Rodrigues(rvecs)
            aruco_tvec_cam = np.matmul(cam_R_aruco, tvecs_mat)
            rot_vec = self.rot_mat2euler_angles(cam_R_aruco.T)
            angle_error = rot_vec[1]
            y_error = -aruco_tvec_cam[0]
            x_error = aruco_tvec_cam[2] - self.OAK_OFFS

            pose_err.x_err = float(x_error)
            pose_err.y_err = float(y_error)
            pose_err.angular_error = float(angle_error)

            response.pose_err = pose_err
            response.got_pose = True
            return response
        else:
            response.got_pose = False
            return response

def main(args=None):
    rclpy.init(args=args)
    pose_estimation = PoseEstimationService()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(pose_estimation)
        executor.spin()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

