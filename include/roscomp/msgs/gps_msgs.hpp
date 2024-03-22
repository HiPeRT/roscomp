#ifndef ROSCOMP_MSGS_GPS_MSGS_HPP
#define ROSCOMP_MSGS_GPS_MSGS_HPP

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#include <gps_common/GPSFix.h>
#elif TKROS_VERSION == 2
#include <gps_msgs/msg/gps_fix.hpp>
#endif

namespace roscomp {
namespace gps_msgs {

#if TKROS_VERSION == 1
typedef ::gps_common::GPSFix GPSFix;
#elif TKROS_VERSION == 2
typedef ::gps_msgs::msg::GPSFix GPSFix;
#endif

} // namespace gps_msgs
} // namespace roscomp

#endif // ROSCOMP_MSGS_GPS_MSGS_HPP
