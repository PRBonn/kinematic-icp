# MIT License
#
# Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
cmake_minimum_required(VERSION 3.16...3.26)
project(kinematic_icp VERSION 0.1.1 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/kinematic_icp ${CMAKE_CURRENT_BINARY_DIR}/kinematic_icp)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
    geometry_msgs
    nav_msgs
    rclcpp
    rclcpp_components
    rcutils
    rosbag2_cpp
    rosbag2_storage
    sensor_msgs
    laser_geometry
    std_msgs
    std_srvs
    tf2_ros
    visualization_msgs)

add_library(
  kinematic_icp_ros SHARED
  src/kinematic_icp_ros/server/LidarOdometryServer.cpp src/kinematic_icp_ros/utils/RosUtils.cpp
  src/kinematic_icp_ros/utils/RosbagUtils.cpp src/kinematic_icp_ros/utils/TimeStampHandler.cpp
  src/kinematic_icp_ros/nodes/online_node.cpp) # Adding it here for composition
target_compile_features(kinematic_icp_ros PUBLIC cxx_std_17)
target_include_directories(kinematic_icp_ros PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kinematic_icp_ros kinematic_icp_pipeline "${cpp_typesupport_target}")
ament_target_dependencies(kinematic_icp_ros ${dependencies})

add_executable(kinematic_icp_offline_node src/kinematic_icp_ros/nodes/offline_node.cpp)
target_link_libraries(kinematic_icp_offline_node PUBLIC kinematic_icp_ros)

rclcpp_components_register_node(kinematic_icp_ros PLUGIN "kinematic_icp_ros::OnlineNode" EXECUTABLE
                                kinematic_icp_online_node)

install(TARGETS kinematic_icp_ros kinematic_icp_online_node kinematic_icp_offline_node LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
