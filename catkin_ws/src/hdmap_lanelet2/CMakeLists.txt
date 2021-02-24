cmake_minimum_required(VERSION 3.0.2)
project(hdmap_lanelet2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    sensor_msgs
    pcl_ros
    pcl_conversions
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hdmap_lidar_camera_fusion
    CATKIN_DEPENDS  cv_bridge 
                    roscpp 
                    sensor_msgs 
                    pcl_ros
                    pcl_conversions
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_LIBRARIES}
    ${PCL_INCLUDE_DIRS}
)

add_executable(aerial_viewer_points nodes/aerial_viewer_points/image_preprocessor.h
                                    nodes/aerial_viewer_points/image_preprocessor.cpp
                                    # nodes/aerial_viewer_points/points_preprocessor.h
                                    # nodes/aerial_viewer_points/points_preprocessor.cpp
                                    nodes/aerial_viewer_points/aerial_viewer_points.h
                                    nodes/aerial_viewer_points/aerial_viewer_points.cpp
                                    nodes/aerial_viewer_points/aerial_viewer_points_node.cpp
                                    )
add_dependencies(aerial_viewer_points   ${catkin_EXPORTED_TARGETS}
                                        ) 
target_link_libraries(aerial_viewer_points  ${catkin_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            )

# add_executable(points_coloring  nodes/points_coloring/points_coloring.h
#                                 nodes/points_coloring/points_coloring_node.cpp
#                                 nodes/points_coloring/points_coloring.cpp
#                                 )
# add_dependencies(points_coloring    ${catkin_EXPORTED_TARGETS}
#                                     ) 
# target_link_libraries(points_coloring   ${catkin_LIBRARIES}
#                                         ${OpenCV_LIBRARIES}
#                                         )
