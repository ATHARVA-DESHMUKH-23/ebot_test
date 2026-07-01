#!/usr/bin/python3
# -*- coding: utf-8 -*-

from matplotlib.pyplot import gray

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from rclpy.time import Time
from builtin_interfaces.msg import Time as BuiltinTime


##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.


    ############ ADD YOUR CODE HERE ############
    # Convert to NumPy array
    pts = np.array(coordinates, dtype=np.float32)

    # Calculate polygon area
    area = cv2.contourArea(pts)

    # Calculate side lengths
    width1 = np.linalg.norm(pts[0] - pts[1])
    width2 = np.linalg.norm(pts[2] - pts[3])
    height1 = np.linalg.norm(pts[1] - pts[2])
    height2 = np.linalg.norm(pts[3] - pts[0])

    # Average width and height
    width = (width1 + width2 + height1 + height2) / 4.0
    ############################################

    return area, width


def detect_aruco(image):
   
    aruco_area_threshold = 200

    
    cam_mat = np.array([
        [223.40954070056824, 0.0, 212.0],
        [0.0, 223.40954070056824, 120.0],
        [0.0, 0.0, 1.0]
    ])

    
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    
    size_of_aruco_m = 0.13

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    ############ ADD YOUR CODE HERE ############


    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list = [], [], [], []

    filtered_ids = []
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i, corner in enumerate(corners):
            pts = corner[0]
            (area, width) = calculate_rectangle_area(pts)
            print(f"Marker area: {area}")
            if area < aruco_area_threshold:  # threshold
                continue
            filtered_ids.append(ids[i])

            cX = int(np.mean(pts[:, 0]))
            cY = int(np.mean(pts[:, 1]))
            center_aruco_list.append((cX, cY))
            width_aruco_list.append(width)

            # Pose estimation
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, size_of_aruco_m, cam_mat, dist_mat)
            # print("tvec shape:", tvec)
            # print("rvec shape:", rvec)
            distance = np.linalg.norm(tvec[0][0])
            distance_from_rgb_list.append(distance)
            angle_aruco_list.append(rvec[0][0][2]) 
            cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.1)
    
    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, filtered_ids


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        # self.color_cam_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.colorimagecb, 10)
        # self.depth_cam_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)
        self.color_cam_sub = self.create_subscription(Image, '/camera_head/image', self.colorimagecb, 10)#'/camera/image_raw'
        self.depth_cam_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depthimagecb, 10)

        self.request_sub = self.create_subscription(String, '/ur5/request', self.ur5_request_cb, 10)
        ############ Constructor VARIABLES/OBJECTS ############

        self.detect_fertilizer = True      # always ON
        self.detect_cart = False           # OFF until request arrives

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())

    def ur5_request_cb(self, msg: String):
        # Accept only the exact request
        if msg.data == "REQUEST_FERTILIZER":
            self.get_logger().info("Received REQUEST_FERTILIZER -> enabling detection")
            self.detect_cart = True
        else:
            # If other messages exist, ignore or handle accordingly
            pass

    def depthimagecb(self, data):
        self.get_logger().info("DEPTH CALLBACK HIT")
        
        ############ ADD YOUR CODE HERE ############
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
            
        except CvBridgeError as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")
        

        ############################################


    def colorimagecb(self, data):
        # self.get_logger().info("COLOR CALLBACK HIT")
        
        ############ ADD YOUR CODE HERE ############
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            # If image is mirrored, uncomment:
            # cv_image = cv2.flip(cv_image, 1)
            self.cv_image = cv_image        
        except CvBridgeError as e:
            self.get_logger().error(f"Color image conversion failed: {e}")

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 424
        sizeCamY = 240

        centerCamX = 212.0
        centerCamY = 120.0

        focalX = 223.40954070056824
        focalY = 223.40954070056824

        self.get_logger().info("TIMER RUNNING")
            

        ############ ADD YOUR CODE HERE ############
        if self.cv_image is None:
            self.get_logger().warn("No image received yet")
            return
        else:
            self.get_logger().info("Image received")

        centers, distances, angles, widths, ids = detect_aruco(self.cv_image)
        self.get_logger().info(f"Detected IDs: {ids}")
        if ids is None:
            return

        for i, marker_id in enumerate(ids):
            self.get_logger().info(f"Processing marker: {marker_id}")
            # ---- DETECTION GATING ----
            if marker_id == 3:
                # Fertilizer → always allowed
                pass  

            elif marker_id == 6:
                # Cart → only detect after request
                if not self.detect_cart:
                    continue  # skip cart until request

            else:
                self.get_logger().warn(f"Ignoring marker {marker_id}")
                # Any other markers: ignore or handle similarly
                continue
            cX, cY = centers[i]
            distance = distances[i]
            angle = angles[i]

            angle = (0.788 * angle) - ((angle ** 2) / 3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)
            r = R.from_euler('zyx', [angle, 0, np.deg2rad(315)])
            quat = r.as_quat()  # x, y, z, w
        
            x = distance * (sizeCamX - cX - centerCamX) / focalX
            y = distance * (sizeCamY - cY - centerCamY) / focalY
            z = distance

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 
            cv2.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)

            #naming according to Aruco Ids
            if(marker_id==3):
                names = "aruco_3"
            elif(marker_id==6):
                names = "aruco_"+str(marker_id)
            else: names = "aruco_"+str(marker_id)

            self.get_logger().info(f"Publishing TF: {names}")
        
            t=TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id ="camera_head_link"
            t.child_frame_id = f"{names}"
            
            if(marker_id==3):
                t.transform.translation.x = z
                t.transform.translation.y = x
                t.transform.translation.z = y
            else:
                t.transform.translation.x = z
                t.transform.translation.y = x
                t.transform.translation.z = y

            if marker_id == 6:
                # Multiply by a 180° rotation around Z
                flip_r = R.from_euler('x', np.pi)
                r_flipped = flip_r * r
                quat = r_flipped.as_quat()

            t.transform.rotation.x = quat[2]
            t.transform.rotation.y = quat[0]
            t.transform.rotation.z = quat[1]
            t.transform.rotation.w = quat[3]
            self.br.sendTransform(t)
            # rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().info(f"TF sent: {t.child_frame_id}")
                        
            # target = 'odom'
            # source = f'1039_{names}'

            # try:
            #     if not self.tf_buffer.can_transform(
            #         target,
            #         source,
            #         Time(seconds=0),
            #         timeout=rclpy.duration.Duration(seconds=0.5)
            #     ):
            #         self.get_logger().warn(f"TF not ready: {target} <- {source}")
            #         return

            #     trans = self.tf_buffer.lookup_transform(
            #         target,
            #         source,
            #         Time(seconds=0),
            #         timeout=rclpy.duration.Duration(seconds=0.5)
            #     )

            #     self.get_logger().info(f"Lookup SUCCESS for {names}")

            # except Exception as e:
            #     self.get_logger().error(f"TF lookup failed: {e}")

                
        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1C)
        #               Also, auto eval script will be judging angular difference as well. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)
        cv2.imshow("Aruco Detection", self.cv_image)
        cv2.waitKey(1)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()