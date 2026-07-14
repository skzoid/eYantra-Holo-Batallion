#!/usr/bin/env python3

"""

This Python file runs a ROS 2 node named localization_node which publishes the position of crates and a holonomic drive robot.
This node subscribes to the following topics:
 SUBSCRIPTIONS
 /camera/image_raw
 /camera/camera_info
 /crates_pose
 /bot_pose
"""
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from hb_interfaces.msg import Pose2D, Poses2D
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import pickle
from collections import defaultdict

target_ids = [12,30,21,16,13,22,11,14,17]
yaw_data = defaultdict(list)




class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.05  # Set marker size in meters
        self.bots_marker_length = 0.05    # Set bot marker size in meters
        self.aruco_dict_name = 'DICT_4X4_50'  # Choose ArUco dictionary
        self.arena_len = 2438.4 
        self.all_corners_detected = False
        self.running_avgs = {12 : 0.0 , 30 : 0.0 , 21 : 0.0 , 16 : 0.0 , 13 : 0.0 , 22 : 0.0 , 11 : 0.0 , 14 : 0.0 , 17 : 0.0}
        self.counts = {12 : 0 , 30 : 0 , 21 : 0 , 16 : 0 , 13 : 0 , 22 : 0 , 11 : 0 , 14 : 0 , 17 : 0}
        
        
        # ---------- TOPICS ----------

        #self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_sub = self.create_timer(0.01 , self.image_callback)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 30)
        
        
        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = np.array([
                                            [1043.49831,    0.     ,  970.66775],
                                            [ 0.     , 1048.43336,  544.79546],
                                            [0.     ,    0.     ,    1.     ]
                                       ])
        # load camera intrinsics (3x3 matrix)
        self.dist_coeffs = np.array([-0.020211, -0.027766, 0.002047, -0.000568, 0.000000])   # load distortion coefficients (1x5 array)
        
        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = []  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = []  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
        
        # ---------- ARUCO SETUP ----------
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        # self.aruco_params.adaptiveThreshWinSizeMin = 3
        # self.aruco_params.adaptiveThreshWinSizeMax = 25
        # self.aruco_params.adaptiveThreshWinSizeStep = 4
        # self.aruco_params.adaptiveThreshConstant = 5

        # self.aruco_params.minMarkerPerimeterRate = 0.015
        # self.aruco_params.maxMarkerPerimeterRate = 4.0
        # self.aruco_params.polygonalApproxAccuracyRate = 0.04

        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # self.aruco_params.cornerRefinementWinSize = 5
        # self.aruco_params.cornerRefinementMaxIterations = 50
        # self.aruco_params.cornerRefinementMinAccuracy = 0.01
        # self.aruco_params.minOtsuStdDev = 2.5
        # self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.2
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict,self.aruco_params)
        self.markers_detected = False
        self.img = None
        
        self.get_logger().info('PoseDetector initialized')
        self.create_subscription(CameraInfo , 'camera/camera_info' , self.cam_callback , 10)

        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


    def pixel_to_world(self, pixel_x, pixel_y):
        """
        - Calculate the H_matrix using: use cv2.findHomography
        - Convert the pixel coordinates into real world coordinates using: cv2.perspectiveTransform(src_pts, self.H_matrix)
        """
        # Implement pixel to world coordinate conversion
        # Step 1: Ensure H_matrix is computed
        # Step 2: Create pixel point in correct format for cv2.perspectiveTransform
        # Step 3: Apply transformation and return world coordinates
        self.pixel_matrix.append([pixel_x,pixel_y])
        src_pts = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        self.H_matrix = self.H_matrix.astype(np.float32)
        world_pts= cv2.perspectiveTransform(np.array(src_pts),self.H_matrix)
        world_x = world_pts[0,0,0]
        world_y = world_pts[0,0,1]
        self.world_matrix.append([world_x,world_y])


        return world_x,world_y

    def image_callback(self):
        """
        Callback function for the image subscriber.
        Main Steps:
        1) Convert ROS Image -> cv image using CvBridge
        2) Undistort the image using camera intrinsics
        3) Detect all the markers in the world (cv2.aruco.drawDetectedMarkers)
        4) Derive the Pixel Matrix and the World Matrix using Corner Markers
        5) Compute the Homography Matrix (cv2.findHomography)
        5) Convert center pixel of crates marker and bot markers to world coordinates
        6) Using OpenCV calculate the yaw angle of each marker (cv2.aruco.estimatePoseSingleMarkers)
        7) Convert the yaw angle as per the new coordinate system
        8) Publish the bot pose and crate poses using the given custom message type
        """

        # Step 1: Convert ROS Image -> cv image using CvBridge
        # Use self.bridge.imgmsg_to_cv2() to convert ROS image to OpenCV format
        ret , cv_image = self.cap.read()
        if ret is False:
            return
        if(self.markers_detected is True):
            undistorted_image = cv2.undistort(cv_image , cameraMatrix = self.camera_matrix , distCoeffs = self.dist_coeffs)
        else:
            undistorted_image = cv_image
        # Step 2: Undistort the image using camera intrinsics
        # Use cv2.undistort() with camera_matrix and dist_coeffs
        # Convert to grayscale for marker detection
        #undistorted_image = cv2.undistort(cv_image,self.camera_matrix,self.dist_coeffs)

        grayscale_image =cv2.cvtColor(undistorted_image,cv2.COLOR_BGR2GRAY)
        #grayscale_image = cv2.GaussianBlur(grayscale_image, (3, 3), 0)

        # Step 3: Detect all the markers in the world
        # Use self.detector.detectMarkers() to find ArUco markers
        # Use cv2.aruco.drawDetectedMarkers() to visualize detected markers
        
        corners, ids, _ = self.detector.detectMarkers(grayscale_image)
        corners = np.array(corners)

        cv2.aruco.drawDetectedMarkers(undistorted_image, corners, ids )
        # Step 4: Derive the Pixel Matrix and the World Matrix using Corner Markers
        # Identify corner markers (IDs 1, 3, 5, 7)
        # Extract their pixel coordinates and map to known world coordinates
        corner_corners = []
        world_end_matrix =[]

        for i in range(len(corners)):
            
            if ids[i][0] in [1,3,5,7]:
                
                
                if ids[i][0]==1:
                    corner_corners.append([corners[i][0][0][0],corners[i][0][0][1]])
                    world_end_matrix.append([self.arena_len/2.0,self.arena_len/2.0])
                    
                if ids[i][0]==3:
                    corner_corners.append([corners[i][0][1][0],corners[i][0][1][1]])
                    world_end_matrix.append([self.arena_len/2.0,-self.arena_len/2.0])
                if ids[i][0]==5:
                    corner_corners.append([corners[i][0][3][0],corners[i][0][3][1]])
                    world_end_matrix.append([-self.arena_len/2.0,self.arena_len/2.0])
                if ids[i][0]==7:
                    corner_corners.append([corners[i][0][2][0],corners[i][0][2][1]])
                    world_end_matrix.append([-self.arena_len/2.0,-self.arena_len/2.0])
        
        #corner_corners = [corners[i] for i in range(len(corners)) if ids[i] in [1,3,5,7]]
        corner_corners = np.array(corner_corners)
        if(len(world_end_matrix) != 4 and (self.markers_detected is False)):
            #print("All corners not detected!")
            image = cv2.resize(undistorted_image , (900,500))
            cv2.imshow('Detected Markers' , image)
            cv2.waitKey(1)
            return
        elif(self.markers_detected is False):
            self.markers_detected = True
            world_end_matrix = np.array(world_end_matrix).reshape(4,2)
        #print(pixel_end_matrix , world_end_matrix)
        #print(corners)



        # Step 5: Compute the Homography Matrix
        # Use cv2.findHomography() with pixel and world points
            self.H_matrix,_ = cv2.findHomography(corner_corners,world_end_matrix)
           # print('Corners Detected Successfully')
            image = cv2.resize(undistorted_image , (900,500))
            cv2.imshow('Detected Markers' , image)
            cv2.waitKey(1)
            return

        

        # Step 6: Convert center pixel of markers to world coordinates
        # For each detected marker (excluding corner markers):
        #       - Calculate center pixel coordinate
        #       - Use pixel_to_world() to convert to world coordinates
        else:
            for i in range(len(corners)):
                if ids[i][0] not in [1,3,5,7]:
                    marker = corners[i]
                    marker = np.mean(marker ,axis =0)

                    self.pixel_to_world(marker[0][0],marker[0][1])
 


            # Step 7: Calculate yaw angle of each marker
            # Use cv2.aruco.estimatePoseSingleMarkers() or any other method to get rotation vectors
            # If you are going ahead with it, convert rotation vector to rotation matrix using cv2.Rodrigues()
            # Extract yaw angle from rotation matrix
            rvecs, tvecs, _ = self.custom_estimatePoseSingleMarkers(corners , ids , self.camera_matrix , self.dist_coeffs)
            crate_poses = {}
            bot_pose = {}  
            
            for i in range(len(corners)):

                R, _ = cv2.Rodrigues(rvecs[i])
                yaw = math.atan2(R[1, 0], R[0, 0])
                marker = corners[i]
                    # marker = marker.reshape(4,2)
                marker = np.mean(marker ,axis =1)

                w_x , w_y = self.pixel_to_world(marker[0][0] , marker[0][1])
                scale = 1 - 50.0/2438.4
                bot_scale = 1 - 90/2438.4
                if(ids[i][0] in [0,2,4]):
                    bot_pose[ids[i][0]] = [ w_x * bot_scale  , w_y * bot_scale  , yaw]

                # elif(ids[i][0] in [1,3,5,7]):
                #     crate_poses[ids[i][0]] = [ w_x  , w_y  , yaw ]
                elif(ids[i][0] in [12,14,30,13,16,21,11,17,22]):
                    crate_poses[ids[i][0]] = [ w_x * scale , w_y * scale  , yaw ]

                
            #print(corners[1].shape)  


            

    



        # Step 8: Separate and publish poses
        # Create separate dictionaries for bot_poses and crate_poses
        # Call publish_crate_poses() and publish_bot_poses()
        self.publish_crate_poses(crate_poses)
        self.publish_bot_poses(bot_pose)

        # Display the image with detected markers
        undistorted_image = cv2.resize(undistorted_image , (700,900))

        cv2.imshow('Detected Markers', undistorted_image)
        cv2.waitKey(1)
            



    def custom_estimatePoseSingleMarkers(self , corners ,ids, cameraMatrix, distCoeffs , crate_markerLength = 45.0 , bot_markerLength = 50.0):
        """
        Replacement for cv2.aruco.estimatePoseSingleMarkers() using solvePnP.
        
        Args:
            corners: list of detected marker corners, each shape (1,4,2)
            markerLength: side length of the marker (same units as camera)
            cameraMatrix: camera intrinsic matrix
            distCoeffs: distortion coefficients
        
        Returns:
            rvecs: np.array of rotation vectors, shape (N,1,3)
            tvecs: np.array of translation vectors, shape (N,1,3)
            _ : placeholder, returns None for compatibility with original signature
        """
        rvecs = []
        tvecs = []

        # Define marker 3D coordinates in marker-local frame
        crate_half_len = crate_markerLength / 2.0
        crate_obj_points = np.array([
            [-crate_half_len,  crate_half_len, 0.0],  # top-left
            [ crate_half_len,  crate_half_len, 0.0],  # top-right
            [ crate_half_len, -crate_half_len, 0.0],  # bottom-right
            [-crate_half_len, -crate_half_len, 0.0],  # bottom-left
        ], dtype=np.float32)
        
        bot_half_len = bot_markerLength / 2.0
        bot_obj_points = np.array([
            [-bot_half_len,  bot_half_len, 0.0],  # top-left
            [ bot_half_len,  bot_half_len, 0.0],  # top-right
            [ bot_half_len, -bot_half_len, 0.0],  # bottom-right
            [-bot_half_len, -bot_half_len, 0.0],  # bottom-left
        ], dtype=np.float32)

        for i in range(len(corners)):
            # corner shape: (1,4,2) -> reshape to (4,2)
            img_points = corners[i].reshape(4, 2).astype(np.float32)

            # Solve PnP
            if ids[i][0] in [0,2,4]:
                success, rvec, tvec = cv2.solvePnP(
                    bot_obj_points,
                    img_points,
                    cameraMatrix,
                    distCoeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )


                if success:
                    cv2.solvePnPRefineLM(
                            bot_obj_points,
                            img_points,
                            cameraMatrix,
                            distCoeffs,
                            rvec,
                            tvec
                        )

                if not success:
                    rvec = np.zeros((3,1), dtype=np.float32)
                    tvec = np.zeros((3,1), dtype=np.float32)
            else:
                success, rvec, tvec = cv2.solvePnP(
                    crate_obj_points,
                    img_points,
                    cameraMatrix,
                    distCoeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )


                if success:
                    cv2.solvePnPRefineLM(
                            crate_obj_points,
                            img_points,
                            cameraMatrix,
                            distCoeffs,
                            rvec,
                            tvec
                        )

                if not success:
                    rvec = np.zeros((3,1), dtype=np.float32)
                    tvec = np.zeros((3,1), dtype=np.float32)


            rvecs.append(rvec)
            tvecs.append(tvec)

        # Convert to same shape as original OpenCV function
        rvecs = np.array(rvecs, dtype=np.float32).reshape(-1, 1, 3)
        tvecs = np.array(tvecs, dtype=np.float32).reshape(-1, 1, 3)

        return rvecs, tvecs, None

    def publish_crate_poses(self, poses):
        """
        - Convert python pose dictionary -> message (Poses2D)
        - self.crate_poses_pub.publish(msg)
        """
        global yaw_data
        global target_ids
        # Create Poses2D message
        # For each pose in poses list:
        #       - Create Pose2D message
        #       - Set id, x, y, w fields
        #       - Append to poses message
        # Publish the message
        ids = list(poses.keys())
        poses_list = list(poses.values())
        message = Poses2D()
        for id , pose in zip(ids , poses_list):
            pose_object = Pose2D()
            pose_object.id = int(id)
            pose_object.x = self.arena_len/2 - float(pose[1])
            pose_object.y = self.arena_len/2 - float(pose[0])
            
            
            yaw = float(pose[2]) * (180 / np.pi)
            if(yaw < 0):
                yaw += 360
            
            if id in target_ids:
                self.counts[id] += 1
                self.running_avgs[id] = (self.running_avgs[id] * (self.counts[id] - 1) + yaw )/self.counts[id]
                #pose_object.w = float(self.running_avgs[id])
                pose_object.w = float(yaw)
            else:
                pose_object.w = float(yaw)
            message.poses.append(pose_object) 

            if id in target_ids:
                yaw_data[id].append(pose_object.w)
        with open("yaw_data.pkl","wb") as f:
            pickle.dump(dict(yaw_data),f)


            
        
        self.crate_poses_pub.publish(message)
        #print(message)


    def publish_bot_poses(self, poses):
        """
        - Convert python pose dictionary -> message (Poses2D)
        - self.bot_poses_pub.publish(msg)
        """
        # Create Poses2D message
        # For each pose in poses list:
        #       - Create Pose2D message
        #       - Set id, x, y, w fields
        #       - Append to poses message
        # Publish the message
        ids = list(poses.keys())
        poses_list = list(poses.values())
        message = Poses2D()
        for id , pose in zip(ids , poses_list):
            pose_object = Pose2D()
            pose_object.id = int(id)
            pose_object.x = self.arena_len/2 - float(pose[1])
            pose_object.y = self.arena_len/2 - float(pose[0])
            pose_object.w = float(pose[2]) * 180 / np.pi
            if(pose_object.w < 0):
                pose_object.w += 360
            
            #print(pose_object.id * 10 * "-",pose_object.w)
            # if id ==4:
            #     print(pose_object.w)


            message.poses.append(pose_object)
        
        self.bot_poses_pub.publish(message)
        #print(message)

    def cam_callback(self , ans):
        pass
        

def main(args=None):
    rclpy.init(args=args)
    pose_detector = PoseDetector()
    try:
        rclpy.spin(pose_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pose_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


