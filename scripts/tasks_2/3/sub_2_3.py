#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from typing import Final

ROS_NODE_NAME: Final[str] = "subscriber"
# ROS_IMAGE_TOPIC: Final[str] = "/pylon_camera_node/image_raw"
ROS_IMAGE_TOPIC: Final[str] = "/dvs/image_raw"

def image_callback(msg: Image, cv_bridge: CvBridge, height_start: int, width_start: int, height: int, width: int) -> None:
	img_gray = cv_bridge.imgmsg_to_cv2(msg)
	img_rgb = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2RGB)

	cropped_img_rgb = img_rgb[height_start : height_start + height, width_start : width_start + width]

	img_hsv = cv2.cvtColor(cropped_img_rgb, cv2.COLOR_RGB2HSV)
	h, s, v = cv2.split(img_hsv)
	
	cv2.imshow("hue", h)
	cv2.imshow("saturation", s)
	cv2.imshow("value", v)
	cv2.waitKey(1)


def main() -> None:
	rospy.init_node(ROS_NODE_NAME)

	sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout = 3.0)

	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")

	cv_bridge: CvBridge = CvBridge()

	height = 128
	width = 128
	height_start = int(sample.height / 2) - int(height / 2)
	width_start = int(sample.width / 2) - int(width / 2)

	rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge, height_start, width_start, height, width), queue_size = None)

	rospy.spin()


if __name__ == '__main__':
	main()
