cmake_minimum_required(VERSION 3.15)
project(roscomp
    VERSION 1.0.0
    DESCRIPTION "ros compatibility project"
    LANGUAGES CXX
)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})

if (NOT DEFINED ROS_VERSION)
    MESSAGE(FATAL_ERROR " Could not find ROS.")
endif()

if (ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosbag
        rosbag_storage
        # action
        actionlib
        # composition
        nodelet
    )
    get_directory_property(HAS_PARENT PARENT_DIRECTORY)
    if(HAS_PARENT)
        catkin_package()
    endif()

    # ros1 specific msgs
    find_package(actionlib_msgs QUIET)
    find_package(gps_common QUIET)

    if(NOT HAS_PARENT)
        add_subdirectory(nav2_msgs)
    endif()

elseif (ROS_VERSION EQUAL 2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rosbag2_cpp REQUIRED)

    # action
    find_package(example_interfaces)
    find_package(nav2_msgs)
    find_package(ament_index_cpp)

    # composition
    find_package(rclcpp_components REQUIRED)
    find_package(rcutils REQUIRED)
    find_package(pcl_conversions REQUIRED)

    # ros2 specific msgs
    find_package(action_msgs QUIET)
    find_package(gps_msgs QUIET)
    find_package(nav2_msgs QUIET)
endif()

# msgs
find_package(ackermann_msgs QUIET)
find_package(actuation_msgs QUIET)
find_package(adx_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(std_msgs QUIET)
find_package(std_srvs QUIET)
find_package(visualization_msgs QUIET)
find_package(wiimote_msgs QUIET)

# tf2
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)


add_library(${PROJECT_NAME} STATIC
    src/${PROJECT_NAME}/ros${ROS_VERSION}/executors.cpp
    src/${PROJECT_NAME}/ros${ROS_VERSION}/roscomp.cpp
    src/${PROJECT_NAME}/ros${ROS_VERSION}/time.cpp
    src/${PROJECT_NAME}/ros${ROS_VERSION}/tf.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PUBLIC
    -DTKROS_VERSION=${ROS_VERSION}
    -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# rosbag
add_library(roscompbag STATIC
    src/${PROJECT_NAME}/ros${ROS_VERSION}/bag.cpp
)
add_library(${PROJECT_NAME}::bag ALIAS roscompbag)
target_include_directories(roscompbag PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roscompbag PRIVATE
    roscomp
)

#ackermann_msgs
if(ackermann_msgs_FOUND)
    add_library(roscompackermann_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompackermann_msgs ALIAS roscompackermann_msgs)
    target_include_directories(roscompackermann_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompackermann_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompackermann_msgs)
endif()

#action_msgs
if(actionlib_msgs_FOUND OR action_msgs_FOUND)
    add_library(roscompaction_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompaction_msgs ALIAS roscompaction_msgs)
    target_include_directories(roscompaction_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompaction_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompaction_msgs)
endif()

#actuation_msgs
if(actuation_msgs_FOUND)
    add_library(roscompactuation_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompactuation_msgs ALIAS roscompactuation_msgs)
    target_include_directories(roscompactuation_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompactuation_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompactuation_msgs)
endif()

#adx_msgs
if(adx_msgs_FOUND)
    add_library(roscompadx_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompadx_msgs ALIAS roscompadx_msgs)
    target_include_directories(roscompadx_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompadx_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompadx_msgs)
endif()

# geometry_msgs
if(geometry_msgs_FOUND)
    add_library(roscompgeometry_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompgeometry_msgs ALIAS roscompgeometry_msgs)
    target_include_directories(roscompgeometry_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompgeometry_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompgeometry_msgs)
endif()

# gps_msgs
if(gps_common_FOUND OR gps_msgs_FOUND)
    add_library(roscompgps_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompgps_msgs ALIAS roscompgps_msgs)
    target_include_directories(roscompgps_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompgps_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompgps_msgs)
endif()

# nav_msgs
if(nav_msgs_FOUND)
    add_library(roscompnav_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompnav_msgs ALIAS roscompnav_msgs)
    target_include_directories(roscompnav_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompnav_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompnav_msgs)
endif()

# nav2_msgs
if(nav2_msgs_FOUND OR ROS_VERSION EQUAL 1)
    add_library(roscompnav2_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompnav2_msgs ALIAS roscompnav2_msgs)
    target_include_directories(roscompnav2_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompnav2_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompnav2_msgs)
endif()

# sensor_msgs
if(sensor_msgs_FOUND)
    add_library(roscompsensor_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompsensor_msgs ALIAS roscompsensor_msgs)
    target_include_directories(roscompsensor_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompsensor_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompsensor_msgs)
endif()

# std_msgs
if(std_msgs_FOUND)
    add_library(roscompstd_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompstd_msgs ALIAS roscompstd_msgs)
    target_include_directories(roscompstd_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompstd_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompstd_msgs)
endif()

# std_srvs
if(std_srvs_FOUND)
    add_library(roscompstd_srvs INTERFACE)
    add_library(${PROJECT_NAME}::roscompstd_srvs ALIAS roscompstd_srvs)
    target_include_directories(roscompstd_srvs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompstd_srvs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompstd_srvs)
endif()

# visualization_msgs
if(visualization_msgs_FOUND)
    add_library(roscompvisualization_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompvisualization_msgs ALIAS roscompvisualization_msgs)
    target_include_directories(roscompvisualization_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompvisualization_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompvisualization_msgs)
endif()

# wiimote_msgs
if(wiimote_msgs_FOUND)
    add_library(roscompwiimote_msgs INTERFACE)
    add_library(${PROJECT_NAME}::roscompwiimote_msgs ALIAS roscompwiimote_msgs)
    target_include_directories(roscompwiimote_msgs INTERFACE
        $<INSTALL_INTERFACE:include>
    )
    target_compile_options(roscompwiimote_msgs INTERFACE
        -DTKROS_VERSION=${ROS_VERSION}
        -DTKROS_DISTRO=TKROS_DISTRO_${ROS_DISTRO})
    list(APPEND OPTIONAL_LIBS roscompwiimote_msgs)
endif()

if(ROS_VERSION EQUAL 1)
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(${PROJECT_NAME} PUBLIC
        ${catkin_LIBRARIES}
    )
    add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
    )

    target_include_directories(roscompbag PUBLIC
        ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(roscompbag PUBLIC
        ${catkin_LIBRARIES}
    )
    add_dependencies(roscompbag
        ${catkin_EXPORTED_TARGETS}
    )

    if(ackermann_msgs_FOUND)
        target_include_directories(roscompackermann_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${ackermann_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompackermann_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${ackermann_msgs_LIBRARIES}
        )
        add_dependencies(roscompackermann_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(actionlib_msgs_FOUND)
        target_include_directories(roscompaction_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${actionlib_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompaction_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${actionlib_msgs_LIBRARIES}
        )
        add_dependencies(roscompaction_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(actuation_msgs_FOUND)
        target_include_directories(roscompactuation_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${actuation_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompactuation_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${actuation_msgs_LIBRARIES}
        )
        add_dependencies(roscompactuation_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(adx_msgs_FOUND)
        target_include_directories(roscompadx_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${adx_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompadx_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${adx_msgs_LIBRARIES}
        )
        add_dependencies(roscompadx_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(geometry_msgs_FOUND)
        target_include_directories(roscompgeometry_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${geometry_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompgeometry_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${geometry_msgs_LIBRARIES}
        )
        add_dependencies(roscompgeometry_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(nav_msgs_FOUND)
        target_include_directories(roscompnav_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${nav_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompnav_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${nav_msgs_LIBRARIES}
        )
        add_dependencies(roscompnav_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(sensor_msgs_FOUND)
        target_include_directories(roscompsensor_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${sensor_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompsensor_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${sensor_msgs_LIBRARIES}
        )
        add_dependencies(roscompsensor_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(std_msgs_FOUND)
        target_include_directories(roscompstd_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${std_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompstd_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${std_msgs_LIBRARIES}
        )
        add_dependencies(roscompstd_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(std_srvs_FOUND)
        target_include_directories(roscompstd_srvs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${std_srvs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompstd_srvs INTERFACE
            ${catkin_LIBRARIES}
            ${std_srvs_LIBRARIES}
        )
        add_dependencies(roscompstd_srvs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(visualization_msgs_FOUND)
        target_include_directories(roscompvisualization_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${visualization_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompvisualization_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${visualization_msgs_LIBRARIES}
        )
        add_dependencies(roscompvisualization_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()

    if(wiimote_msgs_FOUND)
        target_include_directories(roscompwiimote_msgs INTERFACE
            ${catkin_INCLUDE_DIRS}
            ${wiimote_msgs_INCLUDE_DIRS}
        )
        target_link_libraries(roscompwiimote_msgs INTERFACE
            ${catkin_LIBRARIES}
            ${wiimote_msgs_LIBRARIES}
        )
        add_dependencies(roscompwiimote_msgs
            ${catkin_EXPORTED_TARGETS}
        )
    endif()
elseif(ROS_VERSION EQUAL 2)
    ament_target_dependencies(${PROJECT_NAME} PUBLIC
        rclcpp
        rclcpp_components
        rcutils
        tf2
        tf2_ros
    )
    ament_target_dependencies(roscompbag PUBLIC
        rosbag2_cpp
    )


    if(ackermann_msgs_FOUND)
        ament_target_dependencies(roscompackermann_msgs INTERFACE
            ackermann_msgs
        )
    endif()
    if(actuation_msgs_FOUND)
        ament_target_dependencies(roscompactuation_msgs INTERFACE
            actuation_msgs
        )
    endif()
    if(adx_msgs_FOUND)
        ament_target_dependencies(roscompadx_msgs INTERFACE
            adx_msgs
        )
    endif()
    if(geometry_msgs_FOUND)
        ament_target_dependencies(roscompgeometry_msgs INTERFACE
            geometry_msgs
        )
    endif()
    if(nav_msgs_FOUND)
        ament_target_dependencies(roscompnav_msgs INTERFACE
            nav_msgs
        )
    endif()
    if(sensor_msgs_FOUND)
        ament_target_dependencies(roscompsensor_msgs INTERFACE
            sensor_msgs
        )
    endif()
    if(std_msgs_FOUND)
        ament_target_dependencies(roscompstd_msgs INTERFACE
            std_msgs
        )
    endif()
    if(std_srvs_FOUND)
        ament_target_dependencies(roscompstd_srvs INTERFACE
            std_srvs
        )
    endif()
    if(visualization_msgs_FOUND)
        ament_target_dependencies(roscompvisualization_msgs INTERFACE
            visualization_msgs
        )
    endif()
    if(wiimote_msgs_FOUND)
        ament_target_dependencies(roscompwiimote_msgs INTERFACE
            wiimote_msgs
        )
    endif()

endif()

install(TARGETS
        ${PROJECT_NAME}
        roscompbag
        ${OPTIONAL_LIBS}
    DESTINATION lib
    EXPORT ${PROJECT_NAME}Targets
)

install(DIRECTORY
    include/${PROJECT_NAME}
    DESTINATION include/
)

install(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

include(CMakePackageConfigHelpers)
configure_package_config_file(Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

export(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE ${PROJECT_NAME}::
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

if(ROS_VERSION EQUAL 2)
    #ament_package()
endif()
