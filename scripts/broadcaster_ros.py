#!/usr/bin/env python

import tensorflow as tf
from threading import Lock
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
from tfpose_ros.msg import Persons, Person, BodyPartElm

from tfpose_ros.estimator import TfPoseEstimator
from tfpose_ros.networks import model_wh, get_graph_path


class PoseEstimator(object):
    def __init__(self):
        rospy.loginfo('initialization+')
        rospy.init_node('TfPoseEstimatorROS', anonymous=True, log_level=rospy.INFO)

        # parameters
        resolution = rospy.get_param('~resolution', '432x368')
        self.resize_out_ratio = float(rospy.get_param('~resize_out_ratio', '4.0'))
        self.hz = rospy.get_param('~hertz', 5)
        self.tf_lock = Lock()
        self.prev_time = rospy.Time.now()

        try:
            w, h = model_wh(resolution)
            ros_pack = rospkg.RosPack()
            package_path = ros_pack.get_path('tfpose_ros')
            graph_path = rospy.get_param('~model', package_path + '/models/graph/cmu/graph_opt.pb')
        except Exception as e:
            rospy.logerr('invalid model: %s, e=%s' % (graph_path, e))
            exit()

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.gpu_options.visible_device_list = "0"
        self.pose_estimator = TfPoseEstimator(graph_path, target_size=(w, h), tf_config=config)
        self.cv_bridge = CvBridge()

        color_sub = message_filters.Subscriber("~color", Image)
        depth_sub = message_filters.Subscriber("~depth", Image)
        sub = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.5)
        sub.registerCallback(self.callback_image)
        self.pub_pose = rospy.Publisher('~pose', Persons, queue_size=1)
        rospy.loginfo('start+')

    def humans_to_msg(self, humans, depth_image):
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

    def callback_image(self, color_msg, depth_msg):
        if rospy.Duration(1. / self.hz) > rospy.Time.now() - self.prev_time:
            return
        # et = time.time()
        try:
            color_image = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr('[tf-pose-estimation] Converting Image Error. ' + str(e))
            return

        acquired = self.tf_lock.acquire(False)
        if not acquired:
            return

        self.prev_time = rospy.Time.now()

        try:
            humans = self.pose_estimator.inference(
                color_image, resize_to_default=True, upsample_size=self.resize_out_ratio)
        finally:
            self.tf_lock.release()

        msg = self.humans_to_msg(humans, depth_image)
        msg.image_w = color_msg.width
        msg.image_h = color_msg.height
        msg.header = color_msg.header

        self.pub_pose.publish(msg)


if __name__ == '__main__':
    node = PoseEstimator()
    rospy.spin()
    rospy.loginfo('finished')
