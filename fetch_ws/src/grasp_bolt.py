import argparse
import time
import tf
import rospy
from bolt_pipeline import BoltPipeline
from fetch_api import Arm, Head, Torso

from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PoseStamped
from moveit_python import PlanningSceneInterface
from sensor_msgs.msg import Image, CameraInfo

class BoltController(object):

    def __init__(self):
        rospy.init_node(self.__class__.__name__)
        while not rospy.Time.now():
            pass

        time.sleep(2) # let TF buffer fill TODO shorten this delay
        topics = ['/head_camera/rgb/image_raw', '/head_camera/depth_registered/image']
        self.tfl = tf.TransformListener()

        # image processing objects, updated over time
        self.rgb_image, self.depth_image, self.pipeline = None, None, None # these will be initialized later
        self.rgb_sub = rospy.Subscriber(topics[0], Image, self._update_rgb)
        self.depth_sub = rospy.Subscriber(topics[1], Image, self._update_depth)
        time.sleep(2) # let image buffer fill TODO shorten this delay
        self.pipeline = BoltPipeline(self.rgb_image, self.depth_image)

        # setup image frame -> camera frame transformer
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(rospy.wait_for_message('/head_camera/rgb/camera_info', CameraInfo))

    def _update_rgb(self, image):
        self.rgb_image = image

    def _update_depth(self, image):
        self.depth_image = image

    def get_goal_pose(self):
        x, y, z = self.pipeline.get_bolt()
        src_frame = '/base_link'
        dst_frame = self.camera.tfFrame()
        if 'head_camera_rgb_optical_frame' not in dst_frame:
            print('WARNING: destination frame is [%s]')

        # pose in camera frame
        pose = PoseStamped()
        pose.header.frame_id = dst_frame
        pose.header.stamp = rospy.Time.now() # TODO check if this is the correct timestamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z if z<=2000.0 else z/1000.0 # max depth range 2 meter TODO make non-magic #
        pose.pose.orientation.w = 1.0

        # transform target point to base frame
        latest_common_time = tfl.getLatestCommonTime(src_frame, dst_frame)
        self.tfl.waitForTransform(dst_frame, src_frame, rospy.Duration(5.0))
        target = self.tfl.transformPose(src_frame, pose)

        # adjust end-effector for top-down grasp
        yaw = np.radians(90) # TODO update based on angle returned from pipeline
        quat = tf.transformations.quaternion_from_euler(np.radians(180), np.radians(90), yaw)
        target.pose.orientation.x = quat[0]
        target.pose.orientation.y = quat[1]
        target.pose.orientation.z = quat[2]
        target.pose.orientation.w = quat[3]

        return target


if __name__=='__main__':

    print('Setting up Fetch...')
    arm, head, torso = Arm(), Head(), Torso()
    head.pan_tilt(0, 0.6)
    torso.set_height(0.4)
    print('Finished setting up Fetch.')

    controller = BoltController() 
    print('Successfully started bolt-grasp controller.')

    target = controller.get_goal_pose()
    print('Target pose:', target)

    # TODO pass these in more gracefully
    print('MAKE SURE ARM ENV IS CLEAR, THEN PRESS ANY KEY TO CONTINUE.')
    _ = input()
    for i in range(5):
        print('executing in %d', i)
        time.sleep(1)

    arm.move_to_pose(target, replan=True, execution_timeout=15.0, num_planning_attempts=5, replan_attempts=5)
    print('Done.')
