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
WINDOW_CANNY: Final[str] = "canny"
WINDOW_BIN: Final[str] = "binary"
WINDOW_CONT_POLY: Final[str] = "cont_poly"
TRACK_H_MIN: Final[str] = "h_min"
TRACK_H_MAX: Final[str] = "h_max"
TRACK_S_MIN: Final[str] = "s_min"
TRACK_S_MAX: Final[str] = "s_max"
TRACK_V_MIN: Final[str] = "v_min"
TRACK_V_MAX: Final[str] = "v_max"
TRACK_THRESH_CANNY: Final[str] = "thresh_canny"

# h_min = 82
# h_max = 84
# s_min = 249
# s_max = 255
# v_min = 137
# v_max = 161
h_min = 75
h_max = 84
s_min = 239
s_max = 255
v_min = 142
v_max = 161
thresh_canny = 42


def nothing(arg):
	pass


def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
	img_bgr = cv_bridge.imgmsg_to_cv2(msg)
	img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	img_rgb = cv2.resize(img_rgb, (HEIGHT, WIDTH))
	# img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
	img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
	img_cont_poly = img_rgb.copy()

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

	# clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
	# img_adaptive_equalized = clahe.apply(img_gray)

	# Дальнейшую работу буду проводить с Canny

	h_min = cv2.getTrackbarPos(TRACK_H_MIN, WINDOW_BIN)
	h_max = cv2.getTrackbarPos(TRACK_H_MAX, WINDOW_BIN)
	s_min = cv2.getTrackbarPos(TRACK_S_MIN, WINDOW_BIN)
	s_max = cv2.getTrackbarPos(TRACK_S_MAX, WINDOW_BIN)
	v_min = cv2.getTrackbarPos(TRACK_V_MIN, WINDOW_BIN)
	v_max = cv2.getTrackbarPos(TRACK_V_MAX, WINDOW_BIN)
	thresh_canny = cv2.getTrackbarPos(TRACK_THRESH_CANNY, WINDOW_CANNY)

	img_hsv_thresh = cv2.inRange(img_hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
	img_canny = cv2.Canny(img_hsv_thresh, thresh_canny, 2 * thresh_canny)

	# Попробовал применить различные способы обнаружения контуров теперь с увеличением контрастности, и результат стал намного лучше

	# img_improved_canny = cv2.Canny(img_adaptive_equalized, thresh_canny, 2 * thresh_canny)
	# img_improved_sobel = cv2.Sobel(img_adaptive_equalized, -1, dx=1, dy=1, ksize=5, scale=3)
	# _, img_bin_thresh = cv2.threshold(img_adaptive_equalized, 50, 255, cv2.THRESH_BINARY)
	# img_bin_thresh_canny = cv2.Canny(img_bin_thresh, 120, 300)
	# contours, _ = cv2.findContours(img_bin_thresh_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	# img_improved_bin_thresh_canny = img_rgb.copy()
	# cv2.drawContours(img_improved_bin_thresh_canny, contours, -1, (0,255,0), 1)

	# Если улучшить контраст отфильтрованного изображения, то результат все равно будет хуже, чем без фильтрации

	# img_adaptive_equalized = clahe.apply(img_box_filter)
	# img_filtered_improved_canny = cv2.Canny(img_adaptive_equalized, 120, 300)

	contours, _ = cv2.findContours(img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	len_contours = len(contours)
	if len_contours > 0:
		eps_max = -1
		i = 0
		while i < len_contours:
			eps = 0.04 * cv2.arcLength(contours[i], closed=True)
			if eps > eps_max:
				eps_max = eps
				target_contour = contours[i]
			i += 1
		approx_poly = cv2.approxPolyDP(target_contour, epsilon=eps_max, closed=True)
		cv2.drawContours(img_cont_poly, [target_contour], 0, (0, 0, 255), 1)
		cv2.drawContours(img_cont_poly, [approx_poly], 0, (255, 0, 0), 1)

	cv2.imshow(WINDOW_ORIG, img_rgb)
	cv2.imshow(WINDOW_CANNY, img_canny)
	cv2.imshow(WINDOW_BIN, img_hsv_thresh)
	cv2.imshow(WINDOW_CONT_POLY, img_cont_poly)

	cv2.waitKey(0)


def main() -> None:
	rospy.init_node(ROS_NODE_NAME)

	cv2.namedWindow(WINDOW_BIN)
	cv2.namedWindow(WINDOW_CANNY)

	cv2.createTrackbar(TRACK_H_MIN, WINDOW_BIN, 0, 179, nothing)
	cv2.createTrackbar(TRACK_H_MAX, WINDOW_BIN, 0, 179, nothing)
	cv2.createTrackbar(TRACK_S_MIN, WINDOW_BIN, 0, 255, nothing)
	cv2.createTrackbar(TRACK_S_MAX, WINDOW_BIN, 0, 255, nothing)
	cv2.createTrackbar(TRACK_V_MIN, WINDOW_BIN, 0, 255, nothing)
	cv2.createTrackbar(TRACK_V_MAX, WINDOW_BIN, 0, 255, nothing)
	cv2.createTrackbar(TRACK_THRESH_CANNY, WINDOW_CANNY, 0, 255, nothing)

	cv2.setTrackbarPos(TRACK_H_MIN, WINDOW_BIN, h_min)
	cv2.setTrackbarPos(TRACK_H_MAX, WINDOW_BIN, h_max)
	cv2.setTrackbarPos(TRACK_S_MIN, WINDOW_BIN, s_min)
	cv2.setTrackbarPos(TRACK_S_MAX, WINDOW_BIN, s_max)
	cv2.setTrackbarPos(TRACK_V_MIN, WINDOW_BIN, v_min)
	cv2.setTrackbarPos(TRACK_V_MAX, WINDOW_BIN, v_max)
	cv2.setTrackbarPos(TRACK_THRESH_CANNY, WINDOW_CANNY, thresh_canny)

	sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout = 3.0)

	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")

	cv_bridge: CvBridge = CvBridge()

	rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge), queue_size = None)

	rospy.spin()


if __name__ == '__main__':
	main()
