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

flag_trackbar = True

# Можно использовать данный способ, если под рукой есть камера
# h_min = 999
# h_max = -1
# s_min = 999
# s_max = -1
# v_min = 999
# v_max = -1

h_min = 0
h_max = 179
s_min = 0
s_max = 50
v_min = 190
v_max = 255

def change_h_min():
	# global h_min
	# h_min = arg
	print(cv2.getTrackbarPos("h_min", "binary"))
	# print(type(arg))
	print("jjjjj")
	pass

def find_h_min_max(h):
	global h_min
	global h_max
	tmp = min(h)
	if tmp < h_min:
		h_min = tmp
	tmp = max(h)
	if tmp > h_max:
		h_max = tmp

def find_s_min_max(s):
	global s_min
	global s_max
	tmp = min(s)
	if tmp < s_min:
		s_min = tmp
	tmp = max(s)
	if tmp > s_max:
		s_max = tmp

def find_v_min_max(v):
	global v_min
	global v_max
	tmp = min(v)
	if tmp < v_min:
		v_min = tmp
	tmp = max(v)
	if tmp > v_max:
		v_max = tmp

def print_h_s_v_min_max():
	print("----------------------------------")
	print("h_min =", h_min)
	print("h_max =", h_max)
	print("s_min =", s_min)
	print("s_max =", s_max)
	print("v_min =", v_min)
	print("v_max =", v_max)
	print("++++++++++++++++++++++++++++++++++")

def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
	global flag_trackbar
	global h_min
	img_bgr = cv_bridge.imgmsg_to_cv2(msg)
	img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	img_rgb = cv2.resize(img_rgb, (HEIGHT, WIDTH))

	img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

	# Можно использовать данный способ, если под рукой есть камера
	# h, s, v = cv2.split(img_hsv)
	# h = h.reshape(HEIGHT * WIDTH)
	# s = s.reshape(HEIGHT * WIDTH)
	# v = v.reshape(HEIGHT * WIDTH)
	# find_h_min_max(h)
	# find_s_min_max(s)
	# find_v_min_max(v)
	# print_h_s_v_min_max()
	
	# if flag_trackbar:
	cv2.createTrackbar("h_min", "binary", h_min, 179, change_h_min)
	# print("-----------------------------------------")
	flag_trackbar = not flag_trackbar
	# current_h_min= cv2.getTrackbarPos("h_min", "binary")
	# print(current_h_min)
	# h_min = current_h_min

	img_hsv_thresh = cv2.inRange(img_hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))

	cv2.imshow("original", img_rgb)
	cv2.imshow("binary", img_hsv_thresh)
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
