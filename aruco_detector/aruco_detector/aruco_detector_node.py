from __future__ import annotations

import rclpy
import os
import cv2
import yaml
import numpy as np
import quaternion as qu
from rclpy_wrapper.node import Node2
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from custom_srvs.srv import CalibrateCameraTransform, GetPose
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform, PoseStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster
from cv_bridge import CvBridge
from cv2 import aruco

class ArucoDetectorNode(Node2):

    def __init__(self):
        
        super().__init__('aruco_detector')

        self.declare_parameter('calibration_file', os.path.join(get_package_share_directory('aruco_detector'), 'config/calibration.yaml'))
        self.declare_parameter('transform_file', os.path.join(get_package_share_directory('aruco_detector'), 'config/transform.yaml'))
        self.declare_parameter('marker_id', int(0))
        self.declare_parameter('marker_size', float(0.01))
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('rate', float(1))
        
        # calibration parameters
        self._camera_calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value #'/home/srl-orin/calibration.pkl'
        self._camera_transform_file = self.get_parameter('transform_file').get_parameter_value().string_value
        self._camera_calibration_matrix = None
        self._camera_calibration_distcoeff = None
        self._camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        # aruco marker parameters
        self._aruco_marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self._aruco_marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self._aruco_parameters = aruco.DetectorParameters_create()
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)

        # variable for storing camera images
        self._image_raw: np.ndarray = None
        
        # subscriptions
        self.create_subscription(PoseStamped, '/base_pose', self.base_pose_sub_callback, 1)
        self.create_subscription(Image, '/my_camera/pylon_ros2_camera_node/image_raw', self.image_sub_callback, 1)
        # publishers
        self._image_markers_pub = self.create_publisher(Image, self.get_name() + '/image_markers', 1)
        self._detected_pub = self.create_publisher(Bool, self.get_name() + '/detected', 1)
        self._pose_pub = self.create_publisher(PoseStamped, self.get_name() + '/marker_pose', 1)
        # service
        self.create_service(CalibrateCameraTransform, self.get_name() + '/calibrate_camera_transform', self.calibrate_camera_transform_callback)
        # clients
        self._get_pose_cli = self.create_client(GetPose, 'get_marker_pose', callback_group = ReentrantCallbackGroup())
        # timer
        self.create_timer(self.get_parameter('rate').get_parameter_value().double_value, self.pub_timer_callback)
        # rate
        self._rate = self.create_rate(10, self.get_clock())
        # time out for client
        self._timeout_sec = 10
        
        # opencv bridge
        self._cv_bridge = CvBridge()

        # tf broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # transformations
        self._tf_map_to_base = Transform()
        self._tf_base_to_camera = Transform()
        self._tf_camera_to_marker = Transform()

        # load camera calibration file     
        try:  
            with open(self._camera_calibration_file, "r") as stream:
                data = yaml.safe_load(stream)
            self._camera_calibration_matrix = np.array(data['camera_matrix']['data']).reshape((3,3))
            self._camera_calibration_distcoeff = np.array(data['distortion_coefficients']['data'])
        except Exception as exc:
            self.get_logger().warn("Failed loading camera calibration file '" + self._camera_calibration_file + ": '" + str(exc))
        else:
            self.get_logger().info("Loaded camera calibration file '" + self._camera_calibration_file + "'")

        # load camera transform file       
        try:  
            with open(self._camera_transform_file, "r") as stream:  
                data = yaml.safe_load(stream)
            self._tf_base_to_camera.translation.x = float(data['transform']['translation']['x'])
            self._tf_base_to_camera.translation.y = float(data['transform']['translation']['y'])
            self._tf_base_to_camera.translation.z = float(data['transform']['translation']['z'])
            self._tf_base_to_camera.rotation.w = float(data['transform']['rotation']['w'])
            self._tf_base_to_camera.rotation.x = float(data['transform']['rotation']['x'])
            self._tf_base_to_camera.rotation.y = float(data['transform']['rotation']['y'])
            self._tf_base_to_camera.rotation.z = float(data['transform']['rotation']['z'])
        except Exception as exc:
            self.get_logger().warn("Failed loading camera transform file '" + self._camera_transform_file + ": '" + str(exc) + ', proceeding with default transform')
        else:
            self.get_logger().info("Loaded camera transform file '" + self._camera_transform_file + "'")
        
    def image_sub_callback(self, msg: Image):
        
        # convert to open cv image
        self._image_raw = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def base_pose_sub_callback(self, msg: PoseStamped):

        self._tf_map_to_base.translation.x = msg.pose.position.x
        self._tf_map_to_base.translation.y = msg.pose.position.y
        self._tf_map_to_base.translation.z = msg.pose.position.z
        self._tf_map_to_base.rotation.w = msg.pose.orientation.w
        self._tf_map_to_base.rotation.x = msg.pose.orientation.x
        self._tf_map_to_base.rotation.y = msg.pose.orientation.y
        self._tf_map_to_base.rotation.z = msg.pose.orientation.z

        transform = TransformStamped()
        transform.transform = self._tf_map_to_base
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = self._camera_name + '_base'

        self._tf_broadcaster.sendTransform(transform)

    def pub_timer_callback(self):

        detected = False

        if self._camera_calibration_distcoeff is not None and self._camera_calibration_matrix is not None and self._image_raw is not None:
            # grayscale image
            if self._image_raw.ndim > 2:
                gray = cv2.cvtColor(self._image_raw, cv2.COLOR_BGR2GRAY)
            else:
                gray = self._image_raw

            # detect markers/ estimate poses
            corners, ids, _ = aruco.detectMarkers(gray, self._aruco_dict, parameters=self._aruco_parameters)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self._aruco_marker_size, self._camera_calibration_matrix, self._camera_calibration_distcoeff)
            
            # draw markers
            img = aruco.drawDetectedMarkers(self._image_raw, corners, borderColor=(255, 255, 255))

            if ids is not None:
                ids = ids.ravel().tolist()

                if self._aruco_marker_id in ids:
                    rvec = rvecs[ids.index(self._aruco_marker_id)][0]
                    tvec = tvecs[ids.index(self._aruco_marker_id)][0]

                    self._tf_camera_to_marker.translation = self.tvec2translation(tvec)
                    self._tf_camera_to_marker.rotation = self.rvec2quaterion(rvec)
                    detected = True

                    img = aruco.drawAxis(img, self._camera_calibration_matrix, self._camera_calibration_distcoeff, 
                                         rvec, tvec, 1.5*self._aruco_marker_size)

            # publish image with aruco markers
            self._image_markers_pub.publish(self._cv_bridge.cv2_to_imgmsg(img))
            
        # publish detected message
        msg = Bool()
        msg.data = bool(detected)
        self._detected_pub.publish(msg)

        # broadcast transformation from base to camera
        transform = TransformStamped()
        transform.transform = self._tf_base_to_camera
        transform.header.frame_id = self._camera_name + '_base'
        transform.child_frame_id = self._camera_name + '_camera'
        transform.header.stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(transform)

        # broadcast transformation from camera to marker
        transform = TransformStamped()
        transform.transform = self._tf_camera_to_marker
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._camera_name + '_camera'
        transform.child_frame_id = self._camera_name + '_marker'
        self._tf_broadcaster.sendTransform(transform)

        # calculate transform from map to marker
        tf_map_to_camera = self.relative_transform(self.inverse_transform(self._tf_map_to_base), self._tf_base_to_camera)
        tf_map_to_marker = self.relative_transform(self.inverse_transform(tf_map_to_camera), self._tf_camera_to_marker)
        
        # publish pose from base to marker
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf_map_to_marker.translation.x
        pose.pose.position.y = tf_map_to_marker.translation.y
        pose.pose.position.z = tf_map_to_marker.translation.z
        pose.pose.orientation.w = tf_map_to_marker.rotation.w
        pose.pose.orientation.x = tf_map_to_marker.rotation.x
        pose.pose.orientation.y = tf_map_to_marker.rotation.y
        pose.pose.orientation.z = tf_map_to_marker.rotation.z
        self._pose_pub.publish(pose)

    def calibrate_camera_transform_callback(self, request: CalibrateCameraTransform.Request, response: CalibrateCameraTransform.Response) -> CalibrateCameraTransform.Response:

        state = 0

        while True:
            self._rate.sleep()

            # initialize
            if state == 0:
                response.success = False
                start_time = self.get_clock().now().seconds_nanoseconds()[0]
                state = 1

            # wait for service to become ready
            elif state == 1:
                if self._get_pose_cli.service_is_ready():
                    start_time = self.get_clock().now().seconds_nanoseconds()[0]
                    future = self._get_pose_cli.call_async(GetPose.Request())
                    response.success = False
                    state = 2
                else:
                    state = 1
            
            # wait for/get results
            elif state == 2:
                if future.done():
                    future_result: GetPose.Response = future.result()  
                    if future_result.succeeded:
                        # get transform            
                        tf_map_to_marker = Transform()
                        tf_map_to_marker.translation.x = future_result.pose.pose.position.x
                        tf_map_to_marker.translation.y = future_result.pose.pose.position.y
                        tf_map_to_marker.translation.z = future_result.pose.pose.position.z
                        tf_map_to_marker.rotation.w = future_result.pose.pose.orientation.w
                        tf_map_to_marker.rotation.x = future_result.pose.pose.orientation.x
                        tf_map_to_marker.rotation.y = future_result.pose.pose.orientation.y
                        tf_map_to_marker.rotation.z = future_result.pose.pose.orientation.z
                        # calculate transformation from base to camera
                        tf_base_to_marker = self.relative_transform(self._tf_map_to_base, tf_map_to_marker)
                        tf_base_to_camera = self.relative_transform(self._tf_camera_to_marker, tf_base_to_marker)
                        # save transformation in provided config file        
                        try:  
                            with open(self._camera_transform_file, "r") as stream:
                                data = yaml.safe_load(stream)   
                                data['transform'] = {} 
                                data['transform']['translation'] = {}
                                data['transform']['rotation'] = {}
                                data['transform']['translation']['x'] = self._tf_base_to_camera.translation.x
                                data['transform']['translation']['y'] = self._tf_base_to_camera.translation.x
                                data['transform']['translation']['z'] = self._tf_base_to_camera.translation.x
                                data['transform']['rotation']['w'] = self._tf_base_to_camera.rotation.w
                                data['transform']['rotation']['x'] = self._tf_base_to_camera.rotation.x
                                data['transform']['rotation']['y'] = self._tf_base_to_camera.rotation.y
                                data['transform']['rotation']['z'] = self._tf_base_to_camera.rotation.z
                            with open(self._camera_transform_file, "w") as stream:
                                yaml.dump(data, stream)
                        except Exception as exc:
                            self.get_logger().error('Calibrate camera transform: ' + str(exc))
                        else:
                            self.get_logger().info('Calibrate camera transform: Succeeded')
                            self._tf_base_to_camera = tf_base_to_camera
                            response.success = True
                    else:
                        self.get_logger().error('Calibrate camera transform: Could not get marker pose')
                    state = 99
                else:
                    state = 2
            
            # exit 
            elif state == 99:
                break

            if (self.get_clock().now().seconds_nanoseconds()[0] - start_time) >= self._timeout_sec:
                self.get_logger().error('Calibrate camera transform: Timed out')
                state = 99   

        return response

    def rvec2quaterion(self, rvec: np.ndarray) -> Quaternion:

        # calculate rotation matrix
        R = np.zeros((3,3))
        R, _ = cv2.Rodrigues(rvec)

        # calculate quaternion array
        q = qu.as_float_array(qu.from_rotation_matrix(R))

        # create quaternion object
        quaternion = Quaternion()
        quaternion.w = q[0]
        quaternion.x = q[1]
        quaternion.y = q[2]
        quaternion.z = q[3]

        return quaternion
    
    def tvec2translation(self, tvec: np.ndarray) -> Vector3:

        # create translation object
        translation = Vector3()
        translation.x = tvec[0]
        translation.y = tvec[1]
        translation.z = tvec[2]

        return translation
    
    def relative_transform(self, transform0: Transform, transform1: Transform) -> Transform:
        # calculate the relative transform 0->1 of two transforms 0 and 1 in the same coordinate frame

        # convert to transformation matrix
        T0 = self.transform2mat(transform0)
        T1 = self.transform2mat(transform1)
        
        # invert T1
        T0[:3,:3] = T0[:3,:3].T
        T0[:3,3] = -T0[:3,:3] @ T0[:3,3]

        # calculate relative transform
        T = T0 @ T1
        
        # convert to Transform object
        transform = self.mat2transform(T)

        return transform
    
    def inverse_transform(self, transform: Transform):
        # calculate inverse transform

        T = self.transform2mat(transform)
        
        # invert T1
        T[:3,:3] = T[:3,:3].T
        T[:3,3] = -T[:3,:3] @ T[:3,3]

        inverse = self.mat2transform(T)

        return inverse

    def transform2mat(self, transform: Transform) -> np.ndarray:

        T = np.eye(4)
        T[:3,:3] = qu.as_rotation_matrix(qu.from_float_array([transform.rotation.w, 
                                                              transform.rotation.x, 
                                                              transform.rotation.y,
                                                              transform.rotation.z]))
        T[:3,3]  = np.array([transform.translation.x,
                             transform.translation.y,
                             transform.translation.z])
        
        return T
    
    def mat2transform(self, T: np.ndarray) -> Transform:

        q = qu.as_float_array(qu.from_rotation_matrix(T[:3,:3]))

        transform = Transform()
        transform.translation.x = T[0,3]
        transform.translation.y = T[1,3]
        transform.translation.z = T[2,3]
        transform.rotation.w = q[0]
        transform.rotation.x = q[1]
        transform.rotation.y = q[2]
        transform.rotation.z = q[3]

        return transform
    
def main(args = None):

    rclpy.init(args = args)
    try:
        node = ArucoDetectorNode()
        executor = MultiThreadedExecutor()
        try:
            rclpy.spin(node, executor = executor)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()