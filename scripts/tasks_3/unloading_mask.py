#!/usr/bin/env python
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import sys
import rospy, cv2, numpy, roslib
from cv_bridge import CvBridge
import time

class UnloadingMask:

	MIN_NUM_PIXELS = 3

	# UNLOADING_POSITION_X = -0.74
	# UNLOADING_POSITION_Y = -10

	LAM_X_ANGLE = 9
	LAM_X_ANGLE_2 = 5
	LAM_X = 11
	LAM_X_2 = 16
	LAM_Y = 10
	LAM_Y_2 = 5

	FACTOR_X_ANGLE = 0
	# 0.4 0.8 0.85 0.9  0.92
	# 2   3   4    5    6

	FACTOR_Y_ANGLE = 0

	FACTOR_X = 0
	# 10  8  7  6.5  6
	# 2   3  4  5    6

	FACTOR_Y = 0
	# 5  3.3  3  2.9  2.8
	# 2  3    4  5    6

	def nothing(arg):
		pass

	def __init__(self):
		# self.startPositionX = 0
		# self.startPositionY = 0

		# self.amclPoseSub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amclPoseCallback, queue_size = 1)
		
		# self.imageRawSub = rospy.Subscriber("/forward_rgb_camera/rgb/image_raw", Image, self.imageCallback)
		self.imageRawSub = rospy.Subscriber("/usb_cam/image_raw", Image, self.imageCallback)

		# self.imageRawPub = rospy.Publisher("/my_forward_rgb_camera/rgb/image_raw", Image, queue_size=1)

		self.WINDOW_ORIG = "forward.original"
		self.WINDOW_BIN = "forward.binary"
		self.WINDOW_CANNY = "canny"
		self.WINDOW_CONT_POLY = "cont_poly"
		self.WINDOW_EROSION = "erosion"
		self.WINDOW_DILATION = "dilation"

		cv2.namedWindow(self.WINDOW_ORIG)
		self.bridge = CvBridge()

		self.TRACK_POS_Y = "Y"
		self.TRACK_LAM_X_ANGLE = "LAM_X_ANGLE"
		self.TRACK_LAM_X_ANGLE_2 = "LAM_X_ANGLE_2"
		self.TRACK_LAM_X = "LAM_X"
		self.TRACK_LAM_X_2 = "LAM_X_2"
		self.TRACK_LAM_Y = "LAM_Y"
		self.TRACK_LAM_Y_2 = "LAM_Y_2"

		cv2.createTrackbar(self.TRACK_POS_Y, self.WINDOW_ORIG, 0, 15, self.nothing)
		cv2.createTrackbar(self.TRACK_LAM_X_ANGLE, self.WINDOW_ORIG, 1, 50, self.nothing)
		cv2.createTrackbar(self.TRACK_LAM_X_ANGLE_2, self.WINDOW_ORIG, 1, 50, self.nothing)
		cv2.createTrackbar(self.TRACK_LAM_X, self.WINDOW_ORIG, 1, 50, self.nothing)
		cv2.createTrackbar(self.TRACK_LAM_X_2, self.WINDOW_ORIG, 1, 50, self.nothing)
		cv2.createTrackbar(self.TRACK_LAM_Y, self.WINDOW_ORIG, 1, 50, self.nothing)
		cv2.createTrackbar(self.TRACK_LAM_Y_2, self.WINDOW_ORIG, 1, 50, self.nothing)

		cv2.setTrackbarPos(self.TRACK_POS_Y, self.WINDOW_ORIG, 0)
		cv2.setTrackbarPos(self.TRACK_LAM_X_ANGLE, self.WINDOW_ORIG, self.LAM_X_ANGLE)
		cv2.setTrackbarPos(self.TRACK_LAM_X_ANGLE_2, self.WINDOW_ORIG, self.LAM_X_ANGLE_2)
		cv2.setTrackbarPos(self.TRACK_LAM_X, self.WINDOW_ORIG, self.LAM_X)
		cv2.setTrackbarPos(self.TRACK_LAM_X_2, self.WINDOW_ORIG, self.LAM_X_2)
		cv2.setTrackbarPos(self.TRACK_LAM_Y, self.WINDOW_ORIG, self.LAM_Y)
		cv2.setTrackbarPos(self.TRACK_LAM_Y_2, self.WINDOW_ORIG, self.LAM_Y_2)


	# def amclPoseCallback(self, PoseWithCovarianceStamped):
		# currentPositionY = abs(PoseWithCovarianceStamped.pose.pose.position.y - self.UNLOADING_POSITION_Y)
		# self.FACTOR_X_ANGLE = self.LAM_X_ANGLE / 10 - math.exp(-self.LAM_X_ANGLE_2 / 10 * currentPositionY)
		# self.FACTOR_X = self.LAM_X / math.exp(currentPositionY / self.LAM_X_2)
		# self.FACTOR_Y_ANGLE = 1
		# self.FACTOR_Y = self.LAM_Y / math.exp(currentPositionY / self.LAM_Y_2)


	def findLeftUpperPoint(self, approx_poly):
		min_x = min(approx_poly[:, 0, 0])
		min_y = min(approx_poly[:, 0, 1])
		return [min_x, min_y]
	

	def findRightLowerPoint(self, approx_poly):
		max_x = max(approx_poly[:, 0, 0])
		max_y = max(approx_poly[:, 0, 1])
		return [max_x, max_y]
	

	def findUnloadingMask(self, img_rgb, mask):
		img_cont_poly = img_rgb.copy()
		thresh_canny = 42
		h, w, d = img_rgb.shape
		M = cv2.moments(mask)

		if M['m00'] > self.MIN_NUM_PIXELS:
			img_canny = cv2.Canny(mask, thresh_canny, 2 * thresh_canny)
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
				lu = self.findLeftUpperPoint(approx_poly)
				rl = self.findRightLowerPoint(approx_poly)
				dx = rl[0] - lu[0]
				dy = rl[1] - lu[1]
				
				rl[1] = int(rl[1] * self.FACTOR_Y_ANGLE)
				lu[0] = int(lu[0] * self.FACTOR_X_ANGLE)
				dy = int(dy * self.FACTOR_Y)
				dx = int(dx * self.FACTOR_X)
				img_rgb[rl[1]:rl[1]+dy, lu[0]:lu[0]+dx] = 0

				cv2.drawContours(img_cont_poly, [approx_poly], 0, (255, 0, 0), 1)
			cv2.imshow(self.WINDOW_CONT_POLY, img_cont_poly)
		return img_rgb


	def processOrange(self):
		pass


	def processBlue(self):
		pass


	def imageCallback(self, msg):
		currentPositionY = cv2.getTrackbarPos(self.TRACK_POS_Y, self.WINDOW_ORIG)
		self.LAM_X_ANGLE = cv2.getTrackbarPos(self.TRACK_LAM_X_ANGLE, self.WINDOW_ORIG)
		self.LAM_X_ANGLE_2 = cv2.getTrackbarPos(self.TRACK_LAM_X_ANGLE_2, self.WINDOW_ORIG)
		self.LAM_X = cv2.getTrackbarPos(self.TRACK_LAM_X, self.WINDOW_ORIG)
		self.LAM_X_2 = cv2.getTrackbarPos(self.TRACK_LAM_X_2, self.WINDOW_ORIG)
		self.LAM_Y = cv2.getTrackbarPos(self.TRACK_LAM_Y, self.WINDOW_ORIG)
		self.LAM_Y_2 = cv2.getTrackbarPos(self.TRACK_LAM_Y_2, self.WINDOW_ORIG)

		self.FACTOR_X_ANGLE = self.LAM_X_ANGLE / 10 - math.exp(-self.LAM_X_ANGLE_2 / 10 * currentPositionY)
		self.FACTOR_X = self.LAM_X / math.exp(currentPositionY / self.LAM_X_2)
		self.FACTOR_Y_ANGLE = 1
		self.FACTOR_Y = self.LAM_Y / math.exp(currentPositionY / self.LAM_Y_2)

		img_rgb = self.bridge.imgmsg_to_cv2(msg)
		img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
		img_rgb = cv2.resize(img_rgb, (1280, 720))
		img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

		# orange
		# h_min = 70
		# h_max = 109
		# s_min = 69
		# s_max = 152
		# v_min = 131
		# v_max = 158
		# mask = cv2.inRange(img_hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))

		# img_rgb = self.findUnloadingMask(img_rgb, mask)
		# -----------------------------------------------------

		# blue
		h_min = 14
		h_max = 26
		s_min = 110
		s_max = 125
		v_min = 121
		v_max = 134
		mask = cv2.inRange(img_hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
		kernel = numpy.ones((3,3), numpy.uint8)
		J_erosion = cv2.erode(mask, kernel, iterations = 1)
		cv2.imshow(self.WINDOW_EROSION, J_erosion)
		kernel = numpy.ones((3,3), numpy.uint8)
		J_dilation = cv2.dilate(J_erosion, kernel, iterations = 7)
		cv2.imshow(self.WINDOW_DILATION, J_dilation)

		# img_rgb = self.findUnloadingMask(img_rgb, mask)
		img_rgb = self.findUnloadingMask(img_rgb, J_dilation)
		# -----------------------------------------------------

		# self.imageRawPub.publish(self.bridge.cv2_to_imgmsg(img_rgb))

		cv2.imshow(self.WINDOW_ORIG, img_rgb)
		cv2.imshow(self.WINDOW_BIN, mask)
		cv2.waitKey(3)
	

	def start(self):
		rospy.init_node("unloading_mask", anonymous = False)
		rospy.spin()
			

if __name__ == '__main__':
	try:
		unloadingMask = UnloadingMask()
		unloadingMask.start()
	except rospy.ROSInterruptException:
		pass



# orange
		# h_min = 63
		# h_max = 105
		# s_min = 128
		# s_max = 255
		# v_min = 76
		# v_max = 188
	
	# orange 2
		# h_min = 93
		# h_max = 108
		# s_min = 115
		# s_max = 147
		# v_min = 128
		# v_max = 144
	
	# orange 3
		# h_min = 70
		# h_max = 109
		# s_min = 69
		# s_max = 152
		# v_min = 131
		# v_max = 158
	
# blue
		# h_min = 15
		# h_max = 22
		# s_min = 127
		# s_max = 139
		# v_min = 113
		# v_max = 129
	
	# blue 2
		# h_min = 0
		# h_max = 26
		# s_min = 102
		# s_max = 126
		# v_min = 132
		# v_max = 151
	
	# blue 3
		# h_min = 14
		# h_max = 26
		# s_min = 110
		# s_max = 125
		# v_min = 121
		# v_max = 134