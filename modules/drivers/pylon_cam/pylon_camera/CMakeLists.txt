cmake_minimum_required(VERSION 2.8.3)
project(pylon_camera)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -fopenmp")
#set(CMAKE_CXX_FLAGS "-g -Wall -Wno-unknown-pragmas -Wno-delete-non-virtual-dtor -Wno-unused-variable")
set(
    CATKIN_COMPONENTS
     actionlib
     camera_control_msgs
     camera_info_manager
     cv_bridge
     image_geometry
     image_transport
     roscpp
     roslaunch
     sensor_msgs
     #std_srvs
)

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
find_package(
    catkin REQUIRED
    COMPONENTS
     ${CATKIN_COMPONENTS}
)

catkin_package(
    INCLUDE_DIRS
     include
    LIBRARIES
     ${PROJECT_NAME}
    CATKIN_DEPENDS
     ${CATKIN_COMPONENTS}
)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${Pylon_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

# Add the pylon camera library
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/pylon_camera.cpp
        src/${PROJECT_NAME}/binary_exposure_search.cpp
        src/${PROJECT_NAME}/encoding_conversions.cpp
        nodes/pylon_camera_node.cpp
        src/${PROJECT_NAME}/pylon_camera_parameter.cpp)
target_link_libraries(${PROJECT_NAME} ${Pylon_LIBRARIES} ${catkin_LIBRARIES})

## Add the pylon camera nodelet
add_library(${PROJECT_NAME}_nodelet nodes/pylon_camera_nodelet.cpp nodes/pylon_camera_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME} ${catkin_LIBRARIES})

# Add pylon_camera_node
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node nodes/pylon_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Add pylon_camera_node
add_executable(${PROJECT_NAME}_write_device_user_id_to_camera
        src/${PROJECT_NAME}/write_device_user_id_to_camera.cpp)
target_link_libraries(${PROJECT_NAME}_write_device_user_id_to_camera ${PROJECT_NAME})

catkin_python_setup()

install(
    DIRECTORY
     launch/
    DESTINATION
     ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

install(
    DIRECTORY
     config/
    DESTINATION
     ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

install(
    PROGRAMS
     scripts/file_sequencer.py
     scripts/grab_and_save_image_action_server.py
     scripts/result_bag_to_action.py
     scripts/sequence_to_file.py
     scripts/toggle_camera
    DESTINATION
     ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    TARGETS
     ${PROJECT_NAME}
     ${PROJECT_NAME}_node
     ${PROJECT_NAME}_nodelet
     ${PROJECT_NAME}_write_device_user_id_to_camera
    LIBRARY DESTINATION
     ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION
     ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    DIRECTORY
     include/${PROJECT_NAME}/
    DESTINATION
     ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN
     "*.h"
    PATTERN "internal" EXCLUDE
)

