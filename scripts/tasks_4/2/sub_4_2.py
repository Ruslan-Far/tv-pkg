#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Final
import numpy as np
import random

ROS_NODE_NAME: Final[str] = "subscriber"
ROS_IMAGE_TOPIC: Final[str] = "/pylon_camera_node/image_raw"
HEIGHT: Final[int] = 300
WIDTH: Final[int] = 300
WINDOW_ORIG: Final[str] = "original"
WINDOW_CHANGED: Final[str] = "changed"
random.seed(777)


def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
	img_bgr = cv_bridge.imgmsg_to_cv2(msg)
	img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	img_rgb = cv2.resize(img_rgb, (WIDTH, HEIGHT))

	# perspective projection
	pts_src = np.float32([[0, 0], [WIDTH - 1, 0], [0, HEIGHT - 1], [WIDTH - 1, HEIGHT - 1]])
	koef_width_1 = random.random() * (0.1 - 0.01) + 0.01
	koef_width_2 = random.random() * (1 - 0.9) + 0.9
	koef_width_3 = random.random() * (0.3 - 0.01) + 0.01
	koef_width_4 = random.random() * (1 - 0.8) + 0.8
	koef_height_1 = random.random() * (0.4 - 0.01) + 0.01
	koef_height_2 = random.random() * (0.2 - 0.01) + 0.01
	koef_height_3 = random.random() * (1 - 0.7) + 0.7
	koef_height_4 = random.random() * (1 - 0.95) + 0.95
	pts_dst = np.float32([[WIDTH * koef_width_1, HEIGHT * koef_height_1], [WIDTH * koef_width_2, HEIGHT * koef_height_2],
					   		[WIDTH * koef_width_3, HEIGHT * koef_height_3], [WIDTH * koef_width_4, HEIGHT * koef_height_4]])
	perspective_matrix = cv2.getPerspectiveTransform(pts_src, pts_dst)
	img_changed = cv2.warpPerspective(img_rgb, perspective_matrix, (WIDTH, HEIGHT))

	# translation
	translation_matrix = np.float32([[1, 0, random.randint(-WIDTH / 5, WIDTH / 5)], [0, 1, random.randint(-HEIGHT / 5, HEIGHT / 5)]])
	img_changed = cv2.warpAffine(img_changed, translation_matrix, dsize=(WIDTH, HEIGHT))

	# rotation and scale
	rotation_matrix = cv2.getRotationMatrix2D(center=(WIDTH / 2, HEIGHT / 2), angle=random.randint(-90, 90),
										   												scale=random.random() * (1.5 - 0.5) + 0.5)
	img_changed = cv2.warpAffine(img_changed, rotation_matrix, dsize=(WIDTH, HEIGHT))

	# reflection
	img_changed = cv2.flip(img_changed, flipCode=random.randint(-1, 1))

	# shear
	shear_matrix = np.float32 ([[1, random.random() * (0.3 + 0.3) - 0.3, 0],
								[random.random() * (0.3 + 0.3) - 0.3, 1, 0]])
	img_changed = cv2.warpAffine(img_changed, shear_matrix, dsize=(WIDTH, HEIGHT))

	cv2.imshow(WINDOW_ORIG, img_rgb)
	cv2.imshow(WINDOW_CHANGED, img_changed)
	cv2.waitKey(1)


def main() -> None:
	rospy.init_node(ROS_NODE_NAME)

	sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout = 3.0)

	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")

	cv_bridge: CvBridge = CvBridge()

	rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge), queue_size = None)

	rospy.spin()


if __name__ == '__main__':
	main()
