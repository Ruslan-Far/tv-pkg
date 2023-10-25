#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2

import rospy
import rospkg

from cv_bridge import CvBridge

from sensor_msgs.msg import Image

from typing import Final

ROS_NODE_NAME: Final[str] = "subscriber_py"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")

ROS_IMAGE_TOPIC: Final[str] = "image_resized"

def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
  image = cv_bridge.imgmsg_to_cv2(msg)
  window_name = 'window_name'

  image = cv2.resize(image, (128, 128))

  cv2.imshow(window_name, image)
  cv2.waitKey(1)


def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

  rospy.loginfo(f"ROS package: {ROS_PACKAGE_PATH}")
  rospy.loginfo(f"OpenCV version: {cv2.__version__}")

  sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout=3.0)
  
  if sample is not None:
    rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")
  
  cv_bridge: CvBridge = CvBridge()

  rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge), queue_size=None)

  rospy.spin()


if __name__ == '__main__':
  main()