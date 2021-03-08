cmake_minimum_required(VERSION 3.0.2)
project(hdmap)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    sensor_msgs
    pcl_ros
    pcl_conversions
    lanelet2_core
    lanelet2_io
    lanelet2_maps
    lanelet2_projection
    lanelet2_routing
    lanelet2_traffic_rules
    lanelet2_validation
    lanelet2_examples
    message_generation
    std_msgs
)

add_message_files(
    FILES
        MapBin.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        std_msgs
)



catkin_package(
    # INCLUDE_DIRS include
#  LIBRARIES hdmap_lidar_camera_fusion
    CATKIN_DEPENDS  cv_bridge 
                    roscpp 
                    sensor_msgs 
                    pcl_ros
                    pcl_conversions
                    lanelet2_core
                    lanelet2_io
                    lanelet2_maps
                    lanelet2_projection
                    lanelet2_routing
                    lanelet2_traffic_rules
                    lanelet2_validation
                    lanelet2_examples
                    hdmap_msgs
                    message_runtime
                    nav_msgs
                    image_transport
                    tf
                    rosconsole
    DEPENDS Boost
)



include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)


# hdmap_lanelet
add_executable(hdmap_lanelet    nodes/lanelet2_extension/extension.h
                                nodes/lanelet2_extension/extension.cpp
                                nodes/hdmap_lanelet/image_preprocessor.h
                                nodes/hdmap_lanelet/image_preprocessor.cpp
                                # nodes/hdmap_lanelet/points_preprocessor.h
                                # nodes/hdmap_lanelet/points_preprocessor.cpp
                                # nodes/hdmap_lanelet/maps.h
                                # nodes/hdmap_lanelet/maps.cpp
                                nodes/hdmap_lanelet/hdmap_lanelet.h
                                nodes/hdmap_lanelet/hdmap_lanelet.cpp
                                nodes/hdmap_lanelet/hdmap_lanelet_node.cpp
                                )

add_dependencies(hdmap_lanelet  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                                ${catkin_EXPORTED_TARGETS})

target_link_libraries(hdmap_lanelet     ${catkin_LIBRARIES}
                                        ${OpenCV_LIBRARIES}
                                        ${Boost_LIBRARIES}
                                        jsoncpp
                                        yaml-cpp
                                        )


# gen_trajectory
add_executable(gen_trajectory   nodes/lanelet2_extension/extension.h
                                nodes/lanelet2_extension/extension.cpp
                                nodes/gen_trajectory/gen_trajectory.cpp
                                )
add_dependencies(gen_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                                ${catkin_EXPORTED_TARGETS})
target_link_libraries(gen_trajectory    ${catkin_LIBRARIES}
                                        ${Boost_LIBRARIES}
                                        jsoncpp)


# gen_costmap
add_executable(gen_costmap   nodes/gen_costmap/gen_costmap.cpp)
add_dependencies(gen_costmap        ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                                    ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(gen_costmap   ${catkin_LIBRARIES}
                                    ${Boost_LIBRARIES}
                                    jsoncpp)

# directly pub autoware calibration file
add_executable(calibration_pub      nodes/calibration_pub/calibration_publisher.cpp)
add_dependencies(calibration_pub    ${${PROJECT_NAME}_EXPORTED_TARGETS}
					                ${catkin_EXPORTED_TARGETS}
                                    ${OpenCV_LIBRARIES}) 
target_link_libraries(calibration_pub   ${catkin_LIBRARIES})
