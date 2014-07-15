cmake_minimum_required(VERSION 2.8.3)
project(hiwr_opencv_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  camera_info_manager
  driver_base
  message_generation
  nodelet
  cv_bridge

)

find_package(OpenCV)


catkin_package(
  LIBRARIES hiwr_opencv_detector_nodelet
  CATKIN_DEPENDS roscpp rospy std_msgs nodelet
  DEPENDS opencv nodelet std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

install(
	FILES 
        lbpcascade_frontalface.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++0x -pthread -pg")

add_library(hiwr_opencv_detector_nodelet src/hiwr_opencv_detector.cpp)
target_link_libraries(hiwr_opencv_detector_nodelet ${catkin_LIBRARIES})
target_link_libraries(hiwr_opencv_detector_nodelet ${OpenCV_LIBRARIES})


