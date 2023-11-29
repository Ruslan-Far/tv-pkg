#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Final

ROS_NODE_NAME: Final[str] = "subscriber"
ROS_IMAGE_TOPIC: Final[str] = "/pylon_camera_node/image_raw"
HEIGHT: Final[int] = 300
WIDTH: Final[int] = 300
WINDOW_ORIG: Final[str] = "original"
WINDOW_IMPROVED_CANNY: Final[str] = "improved_canny"





def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
	img_bgr = cv_bridge.imgmsg_to_cv2(msg)
	img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	img_rgb = cv2.resize(img_rgb, (HEIGHT, WIDTH))
	img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
	img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

	# Попробовал разные фильтры и везде результат был хуже

	# kernel_size = (3, 3)
	# img_gaussian_blur_filter = cv2.GaussianBlur(img_gray, kernel_size, 0)
	# img_median_blur_filter = cv2.medianBlur(img_gray, kernel_size[0])
	# img_box_filter = cv2.boxFilter(src = img_gray, ddepth = 0, ksize = kernel_size)

	# Попробовал применить различные способы обнаружения контуров сначала без увеличения контрастности

	# img_canny = cv2.Canny(img_gray, 120, 300)
	# img_sobel = cv2.Sobel(img_gray, -1, dx=1, dy=1, ksize=5, scale=3)
	# _, img_bin_thresh = cv2.threshold(img_gray, 50, 255, cv2.THRESH_BINARY)
	# img_bin_thresh_canny = cv2.Canny(img_bin_thresh, 120, 300)
	# contours, _ = cv2.findContours(img_bin_thresh_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	# img_bin_thresh_canny_rgb = img_rgb.copy()
	# cv2.drawContours(img_bin_thresh_canny_rgb, contours, -1, (0,255,0), 1)

	# Увеличил контрастность

	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
	img_adaptive_equalized = clahe.apply(img_gray)

	# Попробовал применить различные способы обнаружения контуров теперь с увеличением контрастности, и результат стал намного лучше
	# Дальнейшую работу буду проводить с Canny

	img_improved_canny = cv2.Canny(img_adaptive_equalized, 120, 300)
	# img_improved_sobel = cv2.Sobel(img_adaptive_equalized, -1, dx=1, dy=1, ksize=5, scale=3)
	# _, img_bin_thresh = cv2.threshold(img_adaptive_equalized, 50, 255, cv2.THRESH_BINARY)
	# img_bin_thresh_canny = cv2.Canny(img_bin_thresh, 120, 300)
	# contours, _ = cv2.findContours(img_bin_thresh_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	# img_improved_bin_thresh_canny = img_rgb.copy()
	# cv2.drawContours(img_improved_bin_thresh_canny, contours, -1, (0,255,0), 1)

	# Если улучшить контраст отфильтрованного изображения, то результат все равно будет хуже, чем без фильтрации

	# img_adaptive_equalized = clahe.apply(img_box_filter)
	# img_filtered_improved_canny = cv2.Canny(img_adaptive_equalized, 120, 300)

	cv2.imshow(WINDOW_ORIG, img_rgb)
	cv2.imshow(WINDOW_IMPROVED_CANNY, img_improved_canny)

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
