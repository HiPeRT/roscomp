#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#include <tf2_msgs/TF2Error.h>
#include <tf2_msgs/TFMessage.h>

#elif TKROS_VERSION == 2
#include <tf2_msgs/msg/tf2_error.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#endif

namespace roscomp {
namespace tf2_msgs {

#if TKROS_VERSION == 1

typedef ::tf2_msgs::TF2Error TF2Error;
typedef ::tf2_msgs::TFMessage TFMessage;

#elif TKROS_VERSION == 2

typedef ::tf2_msgs::msg::TF2Error TF2Error;
typedef ::tf2_msgs::msg::TFMessage TFMessage;

#endif
} // namespace tf2_msgs
} // namespace roscomp