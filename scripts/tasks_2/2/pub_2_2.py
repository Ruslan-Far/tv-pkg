#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Final

ROS_NODE_NAME: Final[str] = "publisher"
# ROS_IMAGE_TOPIC_LISTEN: Final[str] = "/pylon_camera_node/image_raw"
ROS_IMAGE_TOPIC_LISTEN: Final[str] = "/dvs/image_raw"
ROS_IMAGE_TOPIC_TALK: Final[str] = "image_resized"

def image_callback(msg: Image, cv_bridge: CvBridge, pub: rospy.Publisher) -> None:
	image = cv_bridge.imgmsg_to_cv2(msg)
	image = cv2.resize(image, (128, 128))
	image = cv_bridge.cv2_to_imgmsg(image, msg.encoding)
	pub.publish(image)


def main() -> None:
	rospy.init_node(ROS_NODE_NAME)

	sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC_LISTEN, Image, timeout = 3.0)

	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")

	publisher = rospy.Publisher(ROS_IMAGE_TOPIC_TALK, Image, queue_size = 10)

	cv_bridge: CvBridge = CvBridge()

	rospy.loginfo(f"Publishing to '{rospy.resolve_name(ROS_IMAGE_TOPIC_TALK)}'")

	rospy.Subscriber(ROS_IMAGE_TOPIC_LISTEN, Image, lambda msg: image_callback(msg, cv_bridge, publisher), queue_size = None)

	rospy.spin()


if __name__ == '__main__':
	main()
