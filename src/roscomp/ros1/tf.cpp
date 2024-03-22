#include "roscomp/tf.hpp"

namespace roscomp {
namespace tf {

TransformBroadcaster::TransformBroadcaster(Node& node)
{
    (void)node;
}

void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped& transform)
{
    br.sendTransform(transform);
}

void TransformBroadcaster::sendTransform(
  const std::vector<geometry_msgs::TransformStamped>& transforms)
{
    br.sendTransform(transforms);
}

StaticTransformBroadcaster::StaticTransformBroadcaster(Node& node)
{
    (void)node;
}

void StaticTransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped& transform)
{
    br.sendTransform(transform);
}

void StaticTransformBroadcaster::sendTransform(
  const std::vector<geometry_msgs::TransformStamped>& transforms)
{
    br.sendTransform(transforms);
}

} // namespace tf

::ros::Duration toTf2(const ::ros::Duration& aDuration)
{
    return aDuration;
}

} // namespace roscomp