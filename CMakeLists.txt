cmake_minimum_required(VERSION 3.0.2)
project(get_gps_from_map)


add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_generation
    cv_bridge
    )

find_package(OpenCV REQUIRED)


add_service_files(
    FILES
    get_gps_from_map_srv.srv
    )


# add_message_files(
#   FILES
# )


generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    )


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
    )



include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )


add_executable(get_gps_from_map_client src/get_gps_from_map_client.cpp)
target_link_libraries(get_gps_from_map_client ${catkin_LIBRARIES})
add_dependencies(get_gps_from_map_client get_gps_from_map_generate_messages_cpp)


catkin_install_python(PROGRAMS scripts/get_gps_from_map_server.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
