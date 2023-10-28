#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from typing import Final
import numpy as np

ROS_NODE_NAME: Final[str] = "subscriber"
# ROS_IMAGE_TOPIC: Final[str] = "/pylon_camera_node/image_raw"
ROS_IMAGE_TOPIC: Final[str] = "/dvs/image_raw"
HEIGHT: Final[int] = 250
WIDTH: Final[int] = HEIGHT

def image_callback(msg: Image, cv_bridge: CvBridge,
				   center_height: int, center_width: int, cutoff_win_size: int) -> None:
	low_img = cv_bridge.imgmsg_to_cv2(msg)
	low_img = cv2.resize(low_img, (HEIGHT, WIDTH))

	low_img_dft = np.fft.fft2(low_img)
	low_img_dft = np.fft.fftshift(low_img_dft)
	high_img_dft = low_img_dft.copy()

	low_img_dft[(center_height - cutoff_win_size) : (center_height + cutoff_win_size),
             (center_width - cutoff_win_size) : (center_width + cutoff_win_size)] = 0
	
	high_img_dft[:, 0 : (center_width - cutoff_win_size)] = 0
	high_img_dft[:, (center_width + cutoff_win_size) : WIDTH] = 0
	high_img_dft[0 : (center_height - cutoff_win_size), :] = 0
	high_img_dft[(center_height + cutoff_win_size) : HEIGHT, :] = 0
	
	low_img = np.abs(np.fft.ifft2(np.fft.ifftshift(low_img_dft))).astype(np.uint8)
	high_img = np.abs(np.fft.ifft2(np.fft.ifftshift(high_img_dft))).astype(np.uint8)

	cv2.imshow("cut_LOW_freq", low_img)
	cv2.imshow("cut_HIGH_freq", high_img)
	
	cv2.waitKey(1)


def main() -> None:
	rospy.init_node(ROS_NODE_NAME)

	sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout = 3.0)

	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")

	cv_bridge: CvBridge = CvBridge()

	(center_height, center_width) = (HEIGHT // 2, WIDTH // 2)

	cutoff_win_size = HEIGHT // 8

	rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge,
											center_height, center_width, cutoff_win_size), queue_size = None)

	rospy.spin()


if __name__ == '__main__':
	main()
