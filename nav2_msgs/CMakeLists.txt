cmake_minimum_required(VERSION 3.5)
project(nav2_msgs VERSION 1.0
                  DESCRIPTION "nav2_msgs project"
                  LANGUAGES CXX
)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROS_VERSION $ENV{ROS_VERSION})

if (NOT DEFINED ROS_VERSION)
    MESSAGE(FATAL_ERROR " Could not find ROS.")
endif()

add_action_files(
  DIRECTORY
    action/
  FILES
    BackUp.action
    ComputePathThroughPoses.action
    ComputePathToPose.action
    DummyRecovery.action
    FollowPath.action
    FollowWaypoints.action
    NavigateThroughPoses.action
    NavigateToPose.action
    Spin.action
    Wait.action
)

add_message_files(
  DIRECTORY
    msg/
  FILES
    BehaviorTreeLog.msg
    BehaviorTreeStatusChange.msg
    Costmap.msg
    CostmapFilterInfo.msg
    CostmapMetaData.msg
    Particle.msg
    ParticleCloud.msg
    SpeedLimit.msg
    VoxelGrid.msg
)

add_service_files(
  DIRECTORY
    srv/
  FILES
    ClearCostmapAroundRobot.srv
    ClearCostmapExceptRegion.srv
    ClearEntireCostmap.srv
    GetCostmap.srv
    LoadMap.srv
    ManageLifecycleNodes.srv
    SaveMap.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    nav_msgs
    std_msgs
)
