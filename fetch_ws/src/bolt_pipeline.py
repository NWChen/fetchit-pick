from __future__ import print_function

# for pipeline
import cv2
import image_geometry as img_geo
import matplotlib.pyplot as plt
import numpy as np
import os
import pickle
import rosbag
import rospy

# for main
import argparse
import time
import tf
from fetch_api import Arm

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from moveit_python import PlanningSceneInterface
from sensor_msgs.msg import Image, CameraInfo


class BoltPipeline(object):

    '''
    type(image_raw) = sensor_msgs.msg.Image
    type(depth_raw) = sensor_msgs.msg.Image
    '''
    def __init__(self, image_raw, depth_raw):
        bridge, encoding = CvBridge(), 'passthrough'
        self.rgb_img = bridge.imgmsg_to_cv2(image_raw, desired_encoding=encoding)
        self.depth_img = bridge.imgmsg_to_cv2(depth_raw, desired_encoding=encoding)

        # make sure rgb_img has expected dim
        if self.rgb_img.shape != (480, 640, 3):
            print('WARNING: rgb_img has shape (%d, %d, %d)' % self.rgb_img.shape)

        # get a benchmark RGB value for the green bin
        self.real_green, green_file = None, 'green.pkl'
        if green_file not in os.listdir('.'):
            with open(green_file, 'w') as f:
                self.real_green = np.random.normal(loc=64.2, scale=45.3, size=(100,))
                pickle.dump(self.real_green, f)
        else:
            with open(green_file, 'r') as f:
                self.real_green = pickle.load(f)
   
    '''
    some cv helpers
    '''
    def _bounding_box(self, contour):
        rect = cv2.minAreaRect(contour)
        box = np.int0(cv2.boxPoints(rect))
        x, y = np.mean(box[:,0]), np.mean(box[:,1])
        _, _, angle = cv2.fitEllipse(contour)
        return int(x), int(y), angle, box

    def _score(self, x, y, angle):
        dist_from_normal = abs((90.0 - angle)/180.0)
        return y/480.0 + (1.0 - dist_from_normal)

    '''
    gets contour of green bin
    '''
    def bin_contour(self, STD_WEIGHT=2.5, CONTRAST_WEIGHT=1.3, ARC_WEIGHT=0.1):
        # color filtering constants for bin
        MEAN_GREEN = np.mean(self.real_green, axis=0, dtype=int)
        STDEV_GREEN = STD_WEIGHT * np.std(self.real_green, axis=0).astype(int)
        
        # processed bin images
        hc_img = np.clip(self.rgb_img * CONTRAST_WEIGHT, 0, 255).astype(np.uint8) # increase contrast
        bin_mask = cv2.inRange(hc_img, MEAN_GREEN - STDEV_GREEN, MEAN_GREEN + STDEV_GREEN)
        bin_img = cv2.bitwise_and(hc_img, hc_img, mask=bin_mask) # filtered image
       
        # bin outer contour
        gray_img = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        _, contours, hierarchy = cv2.findContours(gray_img, 1, 2)
        max_contour = contours[np.argmax([cv2.contourArea(c) for c in contours])]

        # bin convexhull contour
        EPSILON = 0.1 * cv2.arcLength(max_contour, True)
        hull = cv2.convexHull(cv2.approxPolyDP(max_contour, EPSILON, True))
        return hull, bin_img

    '''
    defines a region of interest in the green bin
    larger ROI_THRESHOLD = bigger / less discriminatory ROI filter
    HEIGHT_RANGE, WIDTH_RANGE correspond to estimated image-dimensions of green bin
    '''
    def bin_roi(self, hull, bin_img, HEIGHT_RANGE=0.3, WIDTH_RANGE=1.8, ROI_THRESHOLD=[50, 25, 50]):
        # ROI dimensions
        HEIGHT_RANGE = int(HEIGHT_RANGE * (np.max(hull[:,:,1]) - np.min(hull[:,:,1])))
        WIDTH_RANGE = int(WIDTH_RANGE * HEIGHT_RANGE)

        # find ROI
        cx, cy = np.mean(hull, axis=0)[0].astype(int) # (cx, cy) define the centroid of the ROI
        roi_origin = (cy-HEIGHT_RANGE, cx-WIDTH_RANGE)
        roi = bin_img[cy-HEIGHT_RANGE:cy+HEIGHT_RANGE,\
                cx-WIDTH_RANGE:cx+WIDTH_RANGE, :]

        # polarize
        roi[roi < np.array(ROI_THRESHOLD)] = 0
        return roi, roi_origin
    
    '''
    BOLT_AREA_WEIGHT defines a tolerance for size of a bolt in image
    '''
    def bolt_contours(self, roi , BOLT_AREA_WEIGHT=0.6):
        # more filtering
        gray_img = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, contours, hierarchy = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        areas = np.array([cv2.contourArea(c) for c in contours])
        mean_area = np.mean(areas)
        part_area = BOLT_AREA_WEIGHT * mean_area
        MIN_AREA, MAX_AREA = mean_area - part_area, mean_area + 2*part_area
        entities, metadata = [], []

        # get bolt candidates
        for i in np.where((areas > MIN_AREA) & (areas < MAX_AREA))[0]:
            x, y, angle, box = self._bounding_box(contours[i])
            entities.append(box)
            metadata.append((x, y, angle))

        return metadata

    '''
    returns (x, y, z) position (in camera frame) of best bolt candidate
    '''
    def get_bolt_position(self, metadata, roi_origin):
        K_TABLE_HEIGHT = np.inf # unknown?
        i_best = np.argmax([self._score(x, y, angle) for (x, y, angle) in metadata])
        x = int(metadata[i_best][1] + roi_origin[0])
        y = int(metadata[i_best][0] + roi_origin[1])
        z = min(self.depth_img[x, y], K_TABLE_HEIGHT)
        angle = metadata[i_best][2]
        return (x, y, z, angle)

    '''
    runs whole pipeline:
    gives you back (x, y, z, theta wrt horizontal) of best bolt candidate in image frame
    '''
    def get_bolt(self):
        hull, img = self.bin_contour()
        roi, roi_origin = self.bin_roi(hull, img)
        metadata = self.bolt_contours(roi)
        return self.get_bolt_position(metadata, roi_origin)

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Run bolt perception & grasping pipeline.')
    parser.add_argument('--bag', help='rosbag file (.bag), if playing back a recording')
    args = parser.parse_args()

    # setup
    rospy.init_node('grasp_bolts')
    while not rospy.Time.now():
        pass
    time.sleep(2) # let TF buffer fill
    tfl = tf.TransformListener()
    rgb_img, depth_img = None, None
    topics = ['/head_camera/rgb/image_raw', '/head_camera/depth_registered/image'] #, '/head_camera/depth_registered/points']

    # process bagfile, if provided
    # start kludge
    rgb_img, depth_img = None, None

    def rgb_callback(img):
        global rgb_img
        rgb_img = img

    def depth_callback(img):
        global depth_img
        depth_img = img

    # end kludge

    if args.bag:
        print('Reading rosbag.')
        for topic, msg, _ in rosbag.Bag(args.bag, 'r').read_messages(topics=topics):
            if topic==topics[0]:
                rgb_img = msg
            elif topic==topics[1]:
                depth_img = msg
            #else:
            #    cloud = ros_to_pcl(msg)
    else:
        print('Reading live topics.')
        rgb_sub = rospy.Subscriber(topics[0], Image, rgb_callback)
        depth_sub = rospy.Subscriber(topics[1], Image, depth_callback)

    # setup pipeline & camera
    print('Setting up arm and camera.')
    print('If this is taking a long time, make sure you\'re playing the rosbag: rosbag play -l <bagfile>')
    arm = Arm()
    pipeline = BoltPipeline(rgb_img, depth_img)
    camera = img_geo.PinholeCameraModel()
    camera.fromCameraInfo(rospy.wait_for_message('/head_camera/depth/camera_info', CameraInfo))
     
    # move the arm
    print('Sending goal to arm.')
    x, y, z = pipeline.get_bolt()
    src_frame = '/base_link'
    dst_frame = '/head_camera_rgb_optical_frame'
    pose = PoseStamped()
    pose.header.frame_id = dst_frame
    #pose.header.stamp = tfl.getLatestCommonTime(src_frame, dst_frame) #rospy.Time.now()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z if z<=1000.0 else z/1000.0 # max depth range 1 meter

    # TODO include gripper rotation
    pose.pose.orientation.w = 1.0
    tfl.waitForTransform(camera.tfFrame(), src_frame, tfl.getLatestCommonTime(src_frame, dst_frame), rospy.Duration(5.0))
    pose = tfl.transformPose(src_frame, pose)

    # UNTESTED
    yaw = 0 # TODO change based on bolt pose
    q = tf.transformations.quaternion_from_euler(np.radians(180), 0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    # add objects to planning scene
    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    xt, yt, zt = 1, 0, 0.5
    planning_scene.addBox('table', 2, 1, 0.5, xt, yt, zt)

    # assumes torso is already up (height=0.4)!
    arm.move_to_pose(pose, replan=True, execution_timeout=15.0, num_planning_attempts=5, replan_attempts=5)
    print('Done.')
