#!/usr/bin/env python3
# encoding: utf-8

import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from typing import Final

ROS_NODE_NAME: Final[str] = "publisher"

ROS_PARAM_PUB_RATE: Final[int] = 30
ROS_IMAGE_TOPIC_2: Final[str] = "/pylon_camera_node/image_raw"
ROS_IMAGE_TOPIC: Final[str] = "image_resized"

def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
    #  image = cv_bridge.imgmsg_to_cv2(msg)
     pub = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)
     pub.publish(msg)
     
def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

#   pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)

#   publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)

#   rospy.loginfo(f"Publishing to '{rospy.resolve_name(ROS_IMAGE_TOPIC)}' at {pub_frequency} Hz ...")

#   height = 128
#   width = 128

#   rate = rospy.Rate(pub_frequency)

#   while not rospy.is_shutdown():
#     publisher.publish(Image(height=height, width=width, encoding='mono8', data=generate_image(height, width)))
#     rate.sleep()
    
  cv_bridge: CvBridge = CvBridge()

  rospy.Subscriber(ROS_IMAGE_TOPIC_2, Image, lambda msg: image_callback(msg, cv_bridge), queue_size=None)

  rospy.spin()


if __name__ == '__main__':
    main()
