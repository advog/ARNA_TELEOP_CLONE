cmake_minimum_required(VERSION 3.0.2)
project(arna_teleop)

add_compile_options(-std=c++11)

set(kortex_driver_DIR "~/ros/kortex_ws/devel/share/kortex_driver/cmake")

find_package(catkin REQUIRED COMPONENTS
  kortex_driver
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

catkin_package(CATKIN_DEPENDS
  kortex_driver
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(arna_teleop_fwd_node src/arna_teleop_fwd_node.cpp)
add_dependencies(arna_teleop_fwd_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(arna_teleop_fwd_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

