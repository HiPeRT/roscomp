#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>

#include "roscomp/time.hpp"

#elif TKROS_VERSION == 2
#include <action_msgs/msg/goal_status.hpp>

#endif

namespace roscomp {
namespace action_msgs {

#if TKROS_VERSION == 1

struct GoalStatus
{
    ::actionlib_msgs::GoalID goal_info;
    int8_t status;

    // constant declarations
    static constexpr int8_t STATUS_UNKNOWN = 0;
    static constexpr int8_t STATUS_ACCEPTED = 1;
    static constexpr int8_t STATUS_EXECUTING = 2;
    static constexpr int8_t STATUS_CANCELING = 3;
    static constexpr int8_t STATUS_SUCCEEDED = 4;
    static constexpr int8_t STATUS_CANCELED = 5;
    static constexpr int8_t STATUS_ABORTED = 6;
};

struct GoalInfo
{
    tk::ros::time::Time stamp;
    std::array<uint8_t, 16> goal_id;
};

struct CancelGoal_Response
{
    using SharedPtr = std::shared_ptr<CancelGoal_Response>;

    int8_t return_code;
    std::vector<GoalInfo> goals_canceling;
};

struct CancelGoal
{
    // using Request = CancelGoal_Request;
    using Response = CancelGoal_Response;
};

#elif TKROS_VERSION == 2

typedef ::action_msgs::msg::GoalStatus GoalStatus;

#endif

} // namespace action_msgs
} // namespace roscomp
