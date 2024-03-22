#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1

#include "nav2_msgs/BackUpAction.h"
#include "nav2_msgs/ComputePathThroughPosesAction.h"
#include "nav2_msgs/ComputePathToPoseAction.h"
#include "nav2_msgs/DummyRecoveryAction.h"
#include "nav2_msgs/FollowPathAction.h"
#include "nav2_msgs/FollowWaypointsAction.h"
#include "nav2_msgs/NavigateThroughPosesAction.h"
#include "nav2_msgs/NavigateToPoseAction.h"
#include "nav2_msgs/SpinAction.h"
#include "nav2_msgs/WaitAction.h"

#elif TKROS_VERSION == 2

#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/dummy_recovery.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/wait.hpp>

#endif

namespace tk {
namespace ros {
namespace nav2_msgs {

#if TKROS_VERSION == 1

struct BackUp
{
    using ActionSpec = ::nav2_msgs::BackUpAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::BackUpGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::BackUpResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::BackUpFeedback;
};

struct ComputePathThroughPoses
{
    using ActionSpec = ::nav2_msgs::ComputePathThroughPosesAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::ComputePathThroughPosesGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::ComputePathThroughPosesResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::ComputePathThroughPosesFeedback;
};

struct ComputePathToPose
{
    using ActionSpec = ::nav2_msgs::ComputePathToPoseAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::ComputePathToPoseGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::ComputePathToPoseResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::ComputePathToPoseFeedback;
};

struct DummyRecovery
{
    using ActionSpec = ::nav2_msgs::DummyRecoveryAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::DummyRecoveryGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::DummyRecoveryResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::DummyRecoveryFeedback;
};

struct FollowPath
{
    using ActionSpec = ::nav2_msgs::FollowPathAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::FollowPathGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::FollowPathResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::FollowPathFeedback;
};

struct FollowWaypoints
{
    using ActionSpec = ::nav2_msgs::FollowWaypointsAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::FollowWaypointsGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::FollowWaypointsResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::FollowWaypointsFeedback;
};

struct NavigateThroughPoses
{
    using ActionSpec = ::nav2_msgs::NavigateThroughPosesAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::NavigateThroughPosesGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::NavigateThroughPosesResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::NavigateThroughPosesFeedback;
};

struct NavigateToPose
{
    using ActionSpec = ::nav2_msgs::NavigateToPoseAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::NavigateToPoseGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::NavigateToPoseResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::NavigateToPoseFeedback;
};

struct Spin
{
    using ActionSpec = ::nav2_msgs::SpinAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::SpinGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::SpinResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::SpinFeedback;
};

struct Wait
{
    using ActionSpec = ::nav2_msgs::WaitAction;
    /// The goal message defined in the action definition.
    using Goal = ::nav2_msgs::WaitGoal;
    /// The result message defined in the action definition.
    using Result = ::nav2_msgs::WaitResult;
    /// The feedback message defined in the action definition.
    using Feedback = ::nav2_msgs::WaitFeedback;
};

#elif TKROS_VERSION == 2

typedef ::nav2_msgs::action::BackUp BackUp;
typedef ::nav2_msgs::action::ComputePathThroughPoses ComputePathThroughPoses;
typedef ::nav2_msgs::action::ComputePathToPose ComputePathToPose;
typedef ::nav2_msgs::action::DummyRecovery DummyRecovery;
typedef ::nav2_msgs::action::FollowPath FollowPath;
typedef ::nav2_msgs::action::FollowWaypoints FollowWaypoints;
typedef ::nav2_msgs::action::NavigateThroughPoses NavigateThroughPoses;
typedef ::nav2_msgs::action::NavigateToPose NavigateToPose;
typedef ::nav2_msgs::action::Spin Spin;
typedef ::nav2_msgs::action::Wait Wait;

#endif

} // namespace nav2_msgs
} // namespace ros
} // namespace tk