#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#include <wiimote_msgs/IrSourceInfo.h>
#include <wiimote_msgs/State.h>
#include <wiimote_msgs/TimedSwitch.h>

#elif TKROS_VERSION == 2
#include <wiimote_msgs/msgs/ir_source_info.hpp>
#include <wiimote_msgs/msgs/state.hpp>
#include <wiimote_msgs/msgs/timed_switch.hpp>

#endif

namespace roscomp::wiimote_msgs {

#if TKROS_VERSION == 1
// These are most likely wrong
typedef ::wiimote_msgs::IrSourceInfo IrSourceInfo;
typedef ::wiimote_msgs::State State;
typedef ::wiimote_msgs::TimedSwitch TimedSwitch;

#elif TKROS_VERSION == 2
typedef ::wiimote_msgs::msgs::IrSourceInfo IrSourceInfo;
typedef ::wiimote_msgs::msgs::State State;
typedef ::wiimote_msgs::msgs::TimedSwitch TimedSwitch;

#endif

} // namespace roscomp::wiimote_msgs

#endif // ROSCOMP_MSGS_wiimote_msgs_HPP