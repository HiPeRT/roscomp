#include <chrono>

#include "roscomp/tf.hpp"

namespace roscomp {

::tf2::Duration toTf2(const ::rclcpp::Duration& aDuration)
{
    return aDuration.to_chrono<std::chrono::nanoseconds>();
}

} // namespace roscomp
