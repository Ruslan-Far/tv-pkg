#!/usr/bin/env python3
# encoding: utf-8

import numpy as np

import rospy
from sensor_msgs.msg import Image

from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "publisher"

ROS_PARAM_PUB_RATE: Final[int] = 30
ROS_IMAGE_TOPIC: Final[str] = "image"

def generate_image(height, width):
	img_mono8 = np.random.randint(0, 256, (height * width))
	return img_mono8.tolist()


def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

  pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)

  # Q: Почему здесь не нужно писать rospy.resolve_name(ROS_IMAGE_TOPIC)?
#   publisher = rospy.Publisher(rospy.resolve_name(ROS_IMAGE_TOPIC), Image, queue_size=10)
  publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)

  # Обратите внимание: топик "image" может переименоваться при запуске ROS-узла.
  # rosrun project_template publisher.py image:=image_raw
  # Более подробно об этом можно узнать по ссылке: http://wiki.ros.org/Names
  rospy.loginfo(f"Publishing to '{rospy.resolve_name(ROS_IMAGE_TOPIC)}' at {pub_frequency} Hz ...")

  height = 240
  width = 320

  rate = rospy.Rate(pub_frequency)

  while not rospy.is_shutdown():
    # Задание 1: сгенерируйте случайное изображение.
    # Разрешение: 320 x 240 (ширина x высота).
    # Формат пикселей: монохром, 8-бит.
    # Создайте функцию для генерации изображения "generate_image(width = 320, height = 240)".
    # publisher.publish(Image(width=0, height=0, encoding='mono8'))
    publisher.publish(Image(height=height, width=width, encoding='mono8', data=generate_image(height, width)))
    
    rate.sleep()


if __name__ == '__main__':
    main()
