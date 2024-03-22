#pragma once

#include <vector>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "roscomp/msgs/geometry_msgs.hpp"
#include "roscomp/roscomp.hpp"
#include "roscomp/time.hpp"

#if TKROS_VERSINON == 2
#include <tf2/time.h>
#endif

namespace roscomp {
namespace tf {

#if TKROS_VERSION == 1
class TransformBroadcaster
{
  public:
    TransformBroadcaster(Node& node);
    void sendTransform(const geometry_msgs::TransformStamped& transform);
    void sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms);

  private:
    tf2_ros::TransformBroadcaster br;
};

class StaticTransformBroadcaster
{
  public:
    StaticTransformBroadcaster(Node& node);
    void sendTransform(const geometry_msgs::TransformStamped& transform);
    void sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms);

  private:
    tf2_ros::StaticTransformBroadcaster br;
};

#elif TKROS_VERSION == 2
typedef tf2_ros::TransformBroadcaster TransformBroadcaster;
typedef tf2_ros::StaticTransformBroadcaster StaticTransformBroadcaster;
#endif
typedef tf2_ros::Buffer Buffer;
typedef tf2_ros::TransformListener TransformListener;
typedef tf2::LookupException LookupException;
typedef tf2::ConnectivityException ConnectivityException;
typedef tf2::ExtrapolationException ExtrapolationException;
typedef tf2::TimeoutException TimeoutException;
typedef tf2::TransformException TransformException;

#if TKROS_VERSION == 1
static const ::ros::Time TimePointZero = ::ros::Time(0);
#elif TKROS_VERSION == 2
static const tf2::TimePoint TimePointZero = tf2::TimePointZero;
#endif
} // namespace tf

#if TKROS_VERSION == 1
::ros::Duration toTf2(const ::ros::Duration& aDuration);
#elif TKROS_VERSION == 2
::tf2::Duration toTf2(const ::rclcpp::Duration& aDuration);
#endif

} // namespace roscomp