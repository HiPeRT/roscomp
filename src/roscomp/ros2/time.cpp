#include "roscomp/time.hpp"

namespace roscomp {
namespace time {

double to_sec(const Duration time)
{
    return time.seconds();
}

double to_sec(const Time time)
{
    return rclcpp::Time(time).seconds();
}

uint64_t to_nanosec(const Duration time)
{
    return static_cast<uint64_t>(time.nanoseconds());
}

uint64_t to_nanosec(const Time time)
{
    return static_cast<uint64_t>(time.nanoseconds());
}

Time from_sec(const double sec)
{
    return rclcpp::Time((rcl_time_point_value_t)(sec * 1e9));
}

Time time_now(Node::SharedPtr node)
{
    return node->get_clock()->now();
}

} // namespace time
} // namespace roscomp