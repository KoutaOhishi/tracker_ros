cmake_minimum_required(VERSION 2.8.3)
project(tracker_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  darknet_dnn
  tf
  pcl_conversions
  pcl_ros
)

find_package(OpenCV REQUIRED)



## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

catkin_package(
)

message(STATUS ${OpenCV_INCLUDE_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  #${OpenCV_INCLUDE_DIRS}
)


add_executable(kcf_tracker src/kcf_tracker.cpp)
target_link_libraries(kcf_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(goturn_tracker src/goturn_tracker.cpp)
target_link_libraries(goturn_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(tf_broadcaster_kcf src/tf_broadcaster_kcf.cpp)
target_link_libraries(tf_broadcaster_kcf ${catkin_LIBRARIES})

add_executable(tf_broadcaster_goturn src/tf_broadcaster_goturn.cpp)
target_link_libraries(tf_broadcaster_goturn ${catkin_LIBRARIES})

