#ifndef ROSCOMP_MSGS_STD_SRVS_HPP
#define ROSCOMP_MSGS_STD_SRVS_HPP

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#elif TKROS_VERSION == 2
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#endif

namespace roscomp {
namespace std_srvs {

#if TKROS_VERSION == 1
typedef ::std_srvs::Empty Empty;
typedef ::std_srvs::SetBool SetBool;
typedef ::std_srvs::Trigger Trigger;

#elif TKROS_VERSION == 2
typedef ::std_srvs::srv::Empty Empty;
typedef ::std_srvs::srv::SetBool SetBool;
typedef ::std_srvs::srv::Trigger Trigger;

#endif

} // namespace std_srvs
} // namespace roscomp

#endif // ROSCOMP_MSGS_STD_SRVS_HPP