#要求的最低的cmake版本
cmake_minimum_required(VERSION 3.0.2)
#工程名字，输出的执行文件也会同名
project(pathing_get)
#包含的头文件名
include_directories("/usr/include/eigen3")

#set(OpenCV_DIR /usr/include/opencv4)
#在计算机中寻找符合要求的第三方库 find_package（库名字 必须）
#find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #airsim_ros_pkgs
  nav_msgs
  sensor_msgs
  message_filters
  mavros_msgs 
 # nlink_parse
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracker_8
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

#包含的头文件
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#需要构建的可执行文件
add_executable(pathing_get
  src/ros_node.cpp
  src/controller.cpp
  src/get_path.cpp
)


#添加运行依赖 
add_dependencies(pathing_get
 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#连接第三方库
target_link_libraries(pathing_get
  ${catkin_LIBRARIES} ${OpenCV_LIBS} 
)



