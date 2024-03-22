#pragma once

#include "roscomp/roscomp.hpp"

#if TKROS_VERSION == 1
#include <ros/time.h>
#elif TKROS_VERSION == 2
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#endif

namespace roscomp {
namespace time {

#if TKROS_VERSION == 1
typedef ::ros::Time Time;
typedef ::ros::Duration Duration;

#elif TKROS_VERSION == 2
typedef rclcpp::Time Time;
typedef rclcpp::Duration Duration;

#endif

double to_sec(const Time time);
double to_sec(const Duration time);
uint64_t to_nanosec(const Time time);
uint64_t to_nanosec(const Duration time);
Time from_sec(const double sec);
Time time_now(Node::SharedPtr node);

} // namespace time
} // namespace roscomp
