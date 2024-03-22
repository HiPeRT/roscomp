#include "roscomp/time.hpp"

namespace roscomp {
namespace time {

double to_sec(const Duration time)
{
    return time.toSec();
}

double to_sec(const Time time)
{
    return time.toSec();
}

uint64_t to_nanosec(const Duration time)
{
    return static_cast<uint64_t>(time.toNSec());
}

uint64_t to_nanosec(const Time time)
{
    return static_cast<uint64_t>(time.toNSec());
}

Time from_sec(const double sec)
{
    return ::ros::Time().fromSec(sec);
}

Time time_now(Node::SharedPtr node)
{
    (void)node;
    return ::ros::Time::now();
}

} // namespace time
} // namespace roscomp