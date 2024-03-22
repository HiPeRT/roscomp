#ifndef ROSCOMP_MSGS_ACTUATION_MSGS_HPP
#define ROSCOMP_MSGS_ACTUATION_MSGS_HPP

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1

#include <actuation_msgs/Actuation.h>
#include <actuation_msgs/ActuationStamped.h>
#include <actuation_msgs/Brake.h>
#include <actuation_msgs/BrakeStamped.h>
#include <actuation_msgs/Gear.h>
#include <actuation_msgs/GearStamped.h>
#include <actuation_msgs/Steering.h>
#include <actuation_msgs/SteeringStamped.h>
#include <actuation_msgs/Throttle.h>
#include <actuation_msgs/ThrottleStamped.h>

#elif TKROS_VERSION == 2

#include <actuation_msgs/msg/actuation.hpp>
#include <actuation_msgs/msg/actuation_stamped.hpp>
#include <actuation_msgs/msg/brake.hpp>
#include <actuation_msgs/msg/brake_stamped.hpp>
#include <actuation_msgs/msg/gear.hpp>
#include <actuation_msgs/msg/gear_stamped.hpp>
#include <actuation_msgs/msg/steering.hpp>
#include <actuation_msgs/msg/steering_stamped.hpp>
#include <actuation_msgs/msg/throttle.hpp>
#include <actuation_msgs/msg/throttle_stamped.hpp>

#endif

namespace roscomp {
namespace actuation_msgs {

#if TKROS_VERSION == 1

typedef ::actuation_msgs::Actuation Actuation;
typedef ::actuation_msgs::ActuationStamped ActuationStamped;
typedef ::actuation_msgs::Brake Brake;
typedef ::actuation_msgs::BrakeStamped BrakeStamped;
typedef ::actuation_msgs::Gear Gear;
typedef ::actuation_msgs::GearStamped GearStamped;
typedef ::actuation_msgs::Steering Steering;
typedef ::actuation_msgs::SteeringStamped SteeringStamped;
typedef ::actuation_msgs::Throttle Throttle;
typedef ::actuation_msgs::ThrottleStamped ThrottleStamped;

#elif TKROS_VERSION == 2

typedef ::actuation_msgs::msg::Actuation Actuation;
typedef ::actuation_msgs::msg::ActuationStamped ActuationStamped;
typedef ::actuation_msgs::msg::Brake Brake;
typedef ::actuation_msgs::msg::BrakeStamped BrakeStamped;
typedef ::actuation_msgs::msg::Gear Gear;
typedef ::actuation_msgs::msg::GearStamped GearStamped;
typedef ::actuation_msgs::msg::Steering Steering;
typedef ::actuation_msgs::msg::SteeringStamped SteeringStamped;
typedef ::actuation_msgs::msg::Throttle Throttle;
typedef ::actuation_msgs::msg::ThrottleStamped ThrottleStamped;

#endif

} // namespace actuation_msgs
} // namespace roscomp

#endif // ROSCOMP_MSGS_ACTUATION_MSGS_HPP