#ifndef ROSCOMP_MSGS_ADX_MSGS_HPP
#define ROSCOMP_MSGS_ADX_MSGS_HPP

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#include <adx_msgs/ArbitratedAckermannDrive.h>
#include <adx_msgs/Obstacle.h>
#include <adx_msgs/Plan.h>
#include <adx_msgs/PlanPoint.h>
#include <adx_msgs/Track.h>
#include <adx_msgs/TrackArray.h>

#elif TKROS_VERSION == 2
#include <adx_msgs/msg/arbitrated_ackermann_drive.hpp>
#include <adx_msgs/msg/obstacle.hpp>
#include <adx_msgs/msg/plan.hpp>
#include <adx_msgs/msg/plan_point.hpp>
#include <adx_msgs/msg/track.hpp>
#include <adx_msgs/msg/track_array.hpp>

#endif

namespace roscomp {
namespace adx_msgs {

#if TKROS_VERSION == 1
typedef ::adx_msgs::ArbitratedAckermannDrive ArbitratedAckermannDrive;
typedef ::adx_msgs::Obstacle Obstacle;
typedef ::adx_msgs::Plan Plan;
typedef ::adx_msgs::PlanPoint PlanPoint;
typedef ::adx_msgs::Track Track;
typedef ::adx_msgs::TrackArray TrackArray;

#elif TKROS_VERSION == 2
typedef ::adx_msgs::msg::ArbitratedAckermannDrive ArbitratedAckermannDrive;
typedef ::adx_msgs::msg::Obstacle Obstacle;
typedef ::adx_msgs::msg::Plan Plan;
typedef ::adx_msgs::msg::PlanPoint PlanPoint;
typedef ::adx_msgs::msg::Track Track;
typedef ::adx_msgs::msg::TrackArray TrackArray;

#endif

} // namespace adx_msgs
} // namespace roscomp

#endif // ROSCOMP_MSGS_ADX_MSGS_HPP
