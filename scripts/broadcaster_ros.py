#!/usr/bin/env python
import tensorflow as tf
from threading import Lock

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import rospkg
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse

from tfpose_ros.estimator import TfPoseEstimator
from tfpose_ros.msg import Persons, Person, BodyPartElm
from tfpose_ros.networks import model_wh


class PoseEstimator(object):
    def __init__(self):
        # parameters
        resolution = rospy.get_param('~resolution', '432x368')
        self.__resize_out_ratio = float(rospy.get_param('~resize_out_ratio', '4.0'))
        self.__hz = rospy.get_param('~hertz', 5)
        self.__tf_lock = Lock()
        self.__prev_time = rospy.Time.now()

        self.__graph_path = None
        try:
            self.__target_size = model_wh(resolution)
            ros_pack = rospkg.RosPack()
            package_path = ros_pack.get_path('tfpose_ros')
            self.__graph_path = rospy.get_param('~model', package_path + '/models/graph/cmu/graph_opt.pb')
        except Exception as e:
            rospy.logerr('invalid model: %s, e=%s' % (self.__graph_path, e))
            exit()

        self.__tf_config = tf.ConfigProto()
        self.__tf_config.gpu_options.allow_growth = True
        self.__tf_config.gpu_options.visible_device_list = "0"
        self.__pose_estimator = None

        self.__is_active = False

        rospy.Service('~enable', SetBool, self.__set_enable)

        self.__pub_pose = rospy.Publisher('~pose', Persons, queue_size=1)

        self.__cv_bridge = CvBridge()
        color_sub = message_filters.Subscriber("~color", Image)
        depth_sub = message_filters.Subscriber("~depth", Image)
        sub = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.5)
        sub.registerCallback(self.__callback_image)

    def __set_enable(self, msg):
        if self.__is_active and not msg.data:
            self.__pose_estimator = None
        elif not self.__is_active and msg.data:
            self.__pose_estimator = TfPoseEstimator(
                self.__graph_path,
                target_size=self.__target_size,
                tf_config=self.__tf_config)
        self.__is_active = msg.data
        message = '{} pose estimator.'.format('Enabled' if msg.data else 'Disabled')
        return SetBoolResponse(success=True, message=message)

    def __humans_to_msg(self, humans, depth_image):
        persons = Persons()

        for human in humans:
            person = Person()

            for k in human.body_parts:
                body_part = human.body_parts[k]

                body_part_msg = BodyPartElm()
                body_part_msg.part_id = body_part.part_idx
                body_part_msg.x = body_part.x
                body_part_msg.y = body_part.y
                body_part_msg.z = depth_image[int(body_part.y), int(body_part.x)]
                body_part_msg.confidence = body_part.score
                person.body_part.append(body_part_msg)
            persons.persons.append(person)

        return persons

    def __callback_image(self, color_msg, depth_msg):
        if not self.__is_active:
            return
        if rospy.Duration(1. / self.__hz) > rospy.Time.now() - self.__prev_time:
            return

        try:
            color_image = self.__cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.__cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr('Converting Image Error. ' + str(e))
            return

        acquired = self.__tf_lock.acquire(False)
        if not acquired:
            return

        self.__prev_time = rospy.Time.now()

        try:
            humans = self.__pose_estimator.inference(
                color_image, resize_to_default=True, upsample_size=self.__resize_out_ratio)
        finally:
            self.__tf_lock.release()

        msg = self.__humans_to_msg(humans, depth_image)
        msg.image_w = color_msg.width
        msg.image_h = color_msg.height
        msg.header = color_msg.header

        self.__pub_pose.publish(msg)


if __name__ == '__main__':
    rospy.init_node('tf_pose_estimator')
    node = PoseEstimator()
    rospy.spin()
