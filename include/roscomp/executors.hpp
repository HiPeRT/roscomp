#pragma once

#include <vector>

#if TKROS_VERSION == 1
#include <ros/callback_queue.h>

#elif TKROS_VERSION == 2
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

#include "roscomp/roscomp.hpp"

namespace roscomp {

#if TKROS_VERSION == 1

class Node; // forward declaration

class CallbackGroup
{
  public:
    using SharedPtr = std::shared_ptr<CallbackGroup>;

    CallbackGroup();

    std::shared_ptr<::ros::CallbackQueue> mCallbackQueue;
};

#elif TKROS_VERSION == 2
typedef rclcpp::CallbackGroupType CallbackGroupType;
typedef rclcpp::CallbackGroup CallbackGroup;
#endif

namespace executors {

#if TKROS_VERSION == 1
class MultiThreadedExecutor
{
  public:
    MultiThreadedExecutor();

    void add_node(std::shared_ptr<Node> aNode);

    void spin();

  private:
    std::vector<std::shared_ptr<Node>> mNodes;

    std::vector<::ros::AsyncSpinner> mSpinners;
};

class SingleThreadedExecutor
{};

#elif TKROS_VERSION == 2
typedef rclcpp::executors::MultiThreadedExecutor MultiThreadedExecutor;
typedef rclcpp::executors::SingleThreadedExecutor SingleThreadedExecutor;
#endif

} // namespace executors
} // namespace roscomp
