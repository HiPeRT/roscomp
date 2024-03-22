#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#elif TKROS_VERSION == 2
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#endif

namespace roscomp {
namespace ackermann_msgs {

#if TKROS_VERSION == 1

typedef ::ackermann_msgs::AckermannDrive AckermannDrive;
typedef ::ackermann_msgs::AckermannDriveStamped AckermannDriveStamped;

#elif TKROS_VERSION == 2

typedef ::ackermann_msgs::msg::AckermannDrive AckermannDrive;
typedef ::ackermann_msgs::msg::AckermannDriveStamped AckermannDriveStamped;

#endif

} // namespace ackermann_msgs
} // namespace roscomp
