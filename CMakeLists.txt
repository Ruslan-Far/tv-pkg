cmake_minimum_required(VERSION 3.10)
project(tv_pkg VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED 
  rospy
  roscpp
  roslib
  cv_bridge
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/tasks_1/1/pub_1_1.py
  scripts/tasks_1/2/sub_1_2.py
  scripts/tasks_1/3/sub_1_3.py
  
  scripts/tasks_2/1/sub_2_1.py
  scripts/tasks_2/2/pub_2_2.py
  scripts/tasks_2/2/sub_2_2.py
  scripts/tasks_2/3/sub_2_3.py
  scripts/tasks_2/4/sub_2_4.py

  scripts/tasks_3/sub_3.py

  scripts/tasks_4/1/sub_4_1.py
  scripts/tasks_4/2/sub_4_2.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
