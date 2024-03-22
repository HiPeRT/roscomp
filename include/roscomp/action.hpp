#pragma once

#if TKROS_VERSION == 1
#include <actionlib/client/action_client.h>
#include <actionlib/server/action_server.h>

#include "roscomp/msgs/std_msgs.hpp"
#elif TKROS_VERSION == 2
#include <rclcpp_action/rclcpp_action.hpp>

#endif

#include "roscomp/msgs/action_msgs.hpp"
#include "roscomp/roscomp.hpp"

namespace roscomp {
namespace action {

#if TKROS_VERSION == 1

namespace internal {
template<class T>
std::shared_ptr<T> as_std_shared_ptr(boost::shared_ptr<T> bp)
{
    if (!bp)
        return nullptr;
    // a std shared pointer to boost shared ptr.  Yes.
    auto pq = std::make_shared<boost::shared_ptr<T>>(std::move(bp));
    // aliasing ctor.  Hide the double shared ptr.  Sneaky.
    return std::shared_ptr<T>(pq, pq.get()->get());
}
} // namespace internal

#define UUID_SIZE 16
using GoalUUID = std::array<uint8_t, UUID_SIZE>;
using GoalStatus = roscomp::action_msgs::GoalStatus;
using GoalInfo = roscomp::action_msgs::GoalInfo;

enum class ResultCode : int8_t
{
    UNKNOWN = roscomp::action_msgs::GoalStatus::STATUS_UNKNOWN,
    SUCCEEDED = roscomp::action_msgs::GoalStatus::STATUS_SUCCEEDED,
    CANCELED = roscomp::action_msgs::GoalStatus::STATUS_CANCELED,
    ABORTED = roscomp::action_msgs::GoalStatus::STATUS_ABORTED
};

/// A response returned by an action server callback when a goal is requested.
enum class GoalResponse : int8_t
{
    /// The goal is rejected and will not be executed.
    REJECT = 1,
    /// The server accepts the goal, and is going to begin execution immediately.
    ACCEPT_AND_EXECUTE = 2,
    /// The server accepts the goal, and is going to execute it later.
    ACCEPT_AND_DEFER = 3,
};

/// A response returned by an action server callback when a goal has been asked to be canceled.
enum class CancelResponse : int8_t
{
    /// The server will not try to cancel the goal.
    REJECT = 1,
    /// The server has agreed to try to cancel the goal.
    ACCEPT = 2,
};

template<typename ActionT>
class ClientGoalHandle
{
  public:
    using SharedPtr = std::shared_ptr<ClientGoalHandle>;

    // A wrapper that defines the result of an action
    typedef struct WrappedResult
    {
        /// The unique identifier of the goal
        GoalUUID goal_id;
        /// A status to indicate if the goal was canceled, aborted, or suceeded
        ResultCode code;
        /// User defined fields sent back with an action
        std::shared_ptr<const typename ActionT::Result> result;
    } WrappedResult;

    ClientGoalHandle(actionlib::ClientGoalHandle<typename ActionT::ActionSpec> aRosClientGoalHandle)
    {
        mRosClientGoalHandle =
          std::make_shared<actionlib::ClientGoalHandle<typename ActionT::ActionSpec>>();
        *mRosClientGoalHandle = aRosClientGoalHandle;
    }

    int8_t get_status() { return mGoalStatus.status; }

  private:
    action_msgs::GoalStatus mGoalStatus;
    std::shared_ptr<actionlib::ClientGoalHandle<typename ActionT::ActionSpec>> mRosClientGoalHandle;
};

template<typename ActionT>
class Client
{
  public:
    using SharedPtr = std::shared_ptr<Client>;

    using GoalHandle = ClientGoalHandle<ActionT>;
    using CancelServiceResponse = typename action_msgs::CancelGoal::Response;

    using GoalPromise = std::promise<typename GoalHandle::SharedPtr>;
    using CancelPromise = std::promise<typename CancelServiceResponse::SharedPtr>;

    using SharedGoalPromise = std::shared_ptr<GoalPromise>;
    using SharedCancelPromise = std::shared_ptr<CancelPromise>;

    using SharedGoalFuture = std::shared_future<typename GoalHandle::SharedPtr>;
    using SharedCancelFuture = std::shared_future<typename CancelServiceResponse::SharedPtr>;

    struct SendGoalOptions
    {
        SendGoalOptions()
          : goal_response_callback(nullptr)
          , feedback_callback(nullptr)
          , result_callback(nullptr)
        {}

        /// Function called when the goal is accepted or rejected.
        /**
         * Takes a single argument that is a goal handle shared pointer.
         * If the goal is accepted, then the pointer points to a valid goal handle.
         * If the goal is rejected, then pointer has the value `nullptr`.
         */
        std::function<void(typename ClientGoalHandle<ActionT>::SharedPtr)> goal_response_callback;

        /// Function called whenever feedback is received for the goal.
        std::function<void(const typename ClientGoalHandle<ActionT>::SharedPtr,
                           const std::shared_ptr<const typename ActionT::Feedback>)>
          feedback_callback;

        /// Function called when the result for the goal is received.
        std::function<void(const typename ClientGoalHandle<ActionT>::WrappedResult&)>
          result_callback;
    };

    Client(Node::SharedPtr aNode, std::string aName)
    {
        // mCallbackQueue = std::make_shared<::ros::CallbackQueue>();
        mRosClient = std::make_shared<actionlib::ActionClient<typename ActionT::ActionSpec>>(
          *aNode->nh_, aName);

        mCancelSub = aNode->create_subscription<std_msgs::Int8>(
          aName + "/cancel_response",
          1,
          std::bind(&Client<ActionT>::cancelResponseCallback, this, std::placeholders::_1));
    }

    bool wait_for_action_server(std::chrono::duration<int64_t, std::milli> timeout =
                                  std::chrono::duration<int64_t, std::milli>(-1))
    {
        std::atomic<bool> serverStarted = false, waiterStopped = false;

        std::thread waiter = std::thread([&]() -> void {
            serverStarted.store(mRosClient->waitForActionServerToStart(::ros::Duration(
              static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count()) /
              1e9)));
            waiterStopped.store(true);
        });

        while (!waiterStopped.load())
            ::ros::spinOnce();

        waiter.join();
        return serverStarted.load();
    }

    std::shared_future<typename GoalHandle::SharedPtr> async_send_goal(
      const typename ActionT::Goal& goal,
      const SendGoalOptions& options = SendGoalOptions())
    {
        mSendGoalOptions = options;

        SharedGoalPromise goal_promise = std::make_shared<GoalPromise>();

        SharedGoalFuture future(goal_promise->get_future());

        mPendingGoalRequest = std::make_pair(goal_promise, future);

        mRosGoalHandle = mRosClient->sendGoal(
          goal,
          std::bind(&Client<ActionT>::transitionCallback, this, std::placeholders::_1),
          std::bind(&Client<ActionT>::feedbackCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        return future;
    }

    std::shared_future<typename CancelServiceResponse::SharedPtr> async_cancel_goal(
      typename ClientGoalHandle<ActionT>::SharedPtr aGoalHandle)
    {
        (void)aGoalHandle;
        SharedCancelPromise cancel_promise = std::make_shared<CancelPromise>();

        SharedCancelFuture future(cancel_promise->get_future());

        mPendingCancelRequest = std::make_pair(cancel_promise, future);

        std::thread(&actionlib::ActionClient<typename ActionT::ActionSpec>::cancelAllGoals,
                    mRosClient.get())
          .detach();

        return future;
    }

    /// Asynchronously request all goals at or before a specified time be canceled.
    /**
     * \param[in] aStamp The timestamp for the cancel goal request.
     * \param[in] cancel_callback Optional callback that is called when the response is received.
     *   The callback takes one parameter: a shared pointer to the CancelServiceResponse message.
     * \return A future to a CancelServiceResponse message that is set when the request has been
     * acknowledged by an action server.
     * See
     * <a href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
     * action_msgs/CancelGoal.srv</a>.
     */
    std::shared_future<typename CancelServiceResponse::SharedPtr> async_cancel_goals_before(
      const ::ros::Time& aStamp,
      std::function<void(typename CancelServiceResponse::SharedPtr)> cancel_callback = nullptr)
    {
        SharedCancelPromise cancel_promise = std::make_shared<CancelPromise>();

        SharedCancelFuture future(cancel_promise->get_future());

        mPendingCancelRequest = std::make_pair(cancel_promise, future);

        mRosClient->cancelGoalsAtAndBeforeTime(aStamp);

        if (cancel_callback != nullptr) {
            cancel_callback(std::make_shared<CancelServiceResponse>());
            tkASSERT(false);
        }

        return future;
    }

  private:
    std::shared_ptr<::ros::CallbackQueue> mCallbackQueue;
    std::shared_ptr<actionlib::ActionClient<typename ActionT::ActionSpec>> mRosClient;
    actionlib::ClientGoalHandle<typename ActionT::ActionSpec> mRosGoalHandle;

    SendGoalOptions mSendGoalOptions;
    // actionlib::CommState mLastCommState;

    std::tuple<SharedGoalPromise, SharedGoalFuture> mPendingGoalRequest;
    std::tuple<SharedCancelPromise, SharedCancelFuture> mPendingCancelRequest;
    roscomp::Subscription<std_msgs::Int8>::SharedPtr mCancelSub;

    // template<typename Type>
    // void do_release(typename boost::shared_ptr<Type> const&, Type*)
    // {}

    void transitionCallback(actionlib::ClientGoalHandle<typename ActionT::ActionSpec> aGoalHandle)
    {
        actionlib::CommState current_comm_state = aGoalHandle.getCommState();

        auto goal_promise = std::get<0>(mPendingGoalRequest);
        auto future = std::get<1>(mPendingGoalRequest);

        typename ClientGoalHandle<ActionT>::SharedPtr goal_handle = nullptr;

        if (current_comm_state == actionlib::CommState::ACTIVE) {
            // accepted
            goal_handle = std::make_shared<ClientGoalHandle<ActionT>>(aGoalHandle);
        }

        else if (current_comm_state == actionlib::CommState::DONE) {
            actionlib::TerminalState current_terminal_state = aGoalHandle.getTerminalState();

            if (current_terminal_state == actionlib::TerminalState::REJECTED) {
                // rejected
                goal_handle = std::make_shared<ClientGoalHandle<ActionT>>(aGoalHandle);
            }

            if (mSendGoalOptions.result_callback != nullptr) {

                if (current_terminal_state == actionlib::TerminalState::PREEMPTED ||
                    current_terminal_state == actionlib::TerminalState::ABORTED ||
                    current_terminal_state == actionlib::TerminalState::SUCCEEDED ||
                    current_terminal_state == actionlib::TerminalState::LOST) {

                    typename ClientGoalHandle<ActionT>::WrappedResult wrapped_result;
                    wrapped_result.result = nullptr;

                    boost::shared_ptr<const typename ActionT::Result> rosResult =
                      aGoalHandle.getResult();

                    wrapped_result.goal_id = {}; // I have found no way to get it out of ROS

                    if (current_terminal_state == actionlib::TerminalState::LOST)
                        wrapped_result.code = ResultCode::UNKNOWN;
                    if (current_terminal_state == actionlib::TerminalState::SUCCEEDED)
                        wrapped_result.code = ResultCode::SUCCEEDED;
                    if (current_terminal_state == actionlib::TerminalState::ABORTED)
                        wrapped_result.code = ResultCode::ABORTED;
                    if (current_terminal_state == actionlib::TerminalState::PREEMPTED)
                        wrapped_result.code = ResultCode::CANCELED;

                    if (rosResult != nullptr) {
                        wrapped_result.result =
                          internal::as_std_shared_ptr<const typename ActionT::Result>(rosResult);
                    }
                    mSendGoalOptions.result_callback(wrapped_result);
                }
            }
        }

        if (goal_handle != nullptr) {
            if (mSendGoalOptions.goal_response_callback != nullptr) {
                mSendGoalOptions.goal_response_callback(goal_handle);
            }
            goal_promise->set_value(goal_handle);
        }
    }

    void feedbackCallback(actionlib::ClientGoalHandle<typename ActionT::ActionSpec> aGoalHandle,
                          const boost::shared_ptr<const typename ActionT::Feedback>& aFeedback)
    {
        std::shared_ptr<ClientGoalHandle<ActionT>> goal_handle =
          std::make_shared<ClientGoalHandle<ActionT>>(aGoalHandle);

        if (mSendGoalOptions.feedback_callback != nullptr)
            mSendGoalOptions.feedback_callback(
              goal_handle,
              internal::as_std_shared_ptr<const typename ActionT::Feedback>(aFeedback));
    }

    void cancelResponseCallback(CallbackPtr<std_msgs::Int8> aCancelResponse)
    {
        auto cancel_promise = std::get<0>(mPendingCancelRequest);
        auto future = std::get<1>(mPendingCancelRequest);

        std::shared_ptr<CancelServiceResponse> response = std::make_shared<CancelServiceResponse>();
        response->return_code = aCancelResponse->data;
        if (cancel_promise != nullptr) {
            cancel_promise->set_value(response);
        }
    }
};

template<typename ActionT>
class ServerGoalHandle
{
  public:
    ServerGoalHandle(const actionlib::ServerGoalHandle<typename ActionT::ActionSpec>& aRosGoal)
      : mRosServerGoalHandle(aRosGoal)
    {
        mRosServerGoalHandle.setAccepted("Accepted");
    }

    void abort(std::shared_ptr<typename ActionT::Result> aResultMsg)
    {
        mRosServerGoalHandle.setAborted(*aResultMsg);
    }

    void canceled(std::shared_ptr<typename ActionT::Result> aResultMsg)
    {
        mRosServerGoalHandle.setCanceled(*aResultMsg);
    }

    void succeed(std::shared_ptr<typename ActionT::Result> aResultMsg)
    {
        mRosServerGoalHandle.setSucceeded(*aResultMsg);
    }

    const std::shared_ptr<const typename ActionT::Goal> get_goal()
    {
        return internal::as_std_shared_ptr<const typename ActionT::Goal>(
          mRosServerGoalHandle.getGoal());
    }

    actionlib_msgs::GoalID get_goal_id() const { return mRosServerGoalHandle.getGoalID(); }

    bool is_active()
    {
        return (
          mRosServerGoalHandle.getGoalStatus().status == ::actionlib_msgs::GoalStatus::ACTIVE ||
          mRosServerGoalHandle.getGoalStatus().status == ::actionlib_msgs::GoalStatus::PREEMPTING);
    }

    bool is_canceling()
    {
        return (mRosServerGoalHandle.getGoalStatus().status ==
                ::actionlib_msgs::GoalStatus::PREEMPTING);
    }

    void publish_feedback(std::shared_ptr<typename ActionT::Feedback> aFeedbackMsg)
    {
        mRosServerGoalHandle.publishFeedback(*aFeedbackMsg);
    }

  private:
    ::actionlib::ServerGoalHandle<typename ActionT::ActionSpec> mRosServerGoalHandle;
};

template<typename ActionT>
class Server
{
  public:
    using SharedPtr = std::shared_ptr<Server>;
    // using CancelResponse = typename CancelGoalService::Response;

    Server(
      Node::SharedPtr aNode,
      std::string aName,
      std::function<GoalResponse(const GoalUUID& uuid,
                                 std::shared_ptr<const typename ActionT::Goal>)> aGoalCallback,
      std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)> aCancelCallback,
      std::function<void(const std::shared_ptr<ServerGoalHandle<ActionT>> goal_handle)>
        aAcceptedCallback)
      : mGoalCallback(aGoalCallback)
      , mCancelCallback(aCancelCallback)
      , mAcceptedCallback(aAcceptedCallback)
    {
        mServer = std::make_shared<::actionlib::ActionServer<typename ActionT::ActionSpec>>(
          *aNode->nh_,
          aName,
          std::bind(&Server<ActionT>::rosGoalCb, this, std::placeholders::_1),
          std::bind(&Server<ActionT>::rosCancelCb, this, std::placeholders::_1),
          false);
        mCancelResponsePub = aNode->create_publisher<std_msgs::Int8>(aName + "/cancel_response", 1);

        mServer->start();
    }

    // std::shared_future<typename CancelResponse::SharedPtr>;

  private:
    std::shared_ptr<actionlib::ActionServer<typename ActionT::ActionSpec>> mServer;

    std::function<GoalResponse(const GoalUUID& uuid, std::shared_ptr<const typename ActionT::Goal>)>
      mGoalCallback;
    std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)> mCancelCallback;
    std::function<void(const std::shared_ptr<ServerGoalHandle<ActionT>>)> mAcceptedCallback;

    std::shared_ptr<ServerGoalHandle<ActionT>> mCurrentGoalHandle;

    Publisher<std_msgs::Int8>::SharedPtr mCancelResponsePub;

    void rosGoalCb(const actionlib::ServerGoalHandle<typename ActionT::ActionSpec>& ros_goal)
    {
        std::string ros_id = ros_goal.getGoalID().id;
        GoalUUID uuid;
        for (unsigned int i = 0; i < std::min(ros_id.size(), static_cast<std::size_t>(UUID_SIZE));
             ++i) {
            uuid[i] = ros_id[i];
        }

        const std::shared_ptr<typename ActionT::Goal> goal =
          std::make_shared<typename ActionT::Goal>();
        *goal = *ros_goal.getGoal();

        GoalResponse status;
        status = mGoalCallback(uuid, goal);

        if (status == GoalResponse::ACCEPT_AND_EXECUTE ||
            status == GoalResponse::ACCEPT_AND_DEFER) {
            mCurrentGoalHandle = std::make_shared<ServerGoalHandle<ActionT>>(ros_goal);
            mAcceptedCallback(mCurrentGoalHandle);
        }
    }

    void rosCancelCb(const actionlib::ServerGoalHandle<typename ActionT::ActionSpec>& ros_goal)
    {
        if (mCurrentGoalHandle->get_goal_id() != ros_goal.getGoalID())
            return;

        std::string ros_id = ros_goal.getGoalID().id;
        GoalUUID uuid;
        for (unsigned int i = 0; i < std::min(ros_id.size(), static_cast<std::size_t>(UUID_SIZE));
             ++i) {
            uuid[i] = ros_id[i];
        }

        roscomp::std_msgs::Int8 status;
        status.data = static_cast<int8_t>(mCancelCallback(mCurrentGoalHandle));
        mCancelResponsePub->publish(status);
    }
};

template<typename ActionT, typename... Args>
typename Client<ActionT>::SharedPtr create_client(Node::SharedPtr aNode, std::string aName)
{
    return std::make_shared<Client<ActionT>>(aNode, aName);
}

template<typename ActionT>
typename Server<ActionT>::SharedPtr create_server(
  Node::SharedPtr aNode,
  std::string aName,
  std::function<GoalResponse(const GoalUUID& uuid, std::shared_ptr<const typename ActionT::Goal>)>
    aGoalCallback,
  std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)> aCancelCallback,
  std::function<void(const std::shared_ptr<ServerGoalHandle<ActionT>> goal_handle)>
    aAcceptedCallback)
{
    return std::make_shared<Server<ActionT>>(
      aNode, aName, aGoalCallback, aCancelCallback, aAcceptedCallback);
}

#elif TKROS_VERSION == 2
// client
template<typename ActionT>
using ClientGoalHandle = ::rclcpp_action::ClientGoalHandle<ActionT>;

template<typename ActionT>
using Client = ::rclcpp_action::Client<ActionT>;

typedef ::rclcpp_action::GoalResponse GoalResponse;
typedef ::rclcpp_action::CancelResponse CancelResponse;
typedef ::rclcpp_action::ResultCode ResultCode;

template<typename ActionT, typename... Args>
typename Client<ActionT>::SharedPtr create_client(Args&&... args)
{
    return ::rclcpp_action::create_client<ActionT>(std::forward<Args>(args)...);
}

// server
template<typename ActionT>
using ServerGoalHandle = ::rclcpp_action::ServerGoalHandle<ActionT>;

typedef ::rclcpp_action::GoalUUID GoalUUID;

template<typename ActionT>
using Server = ::rclcpp_action::Server<ActionT>;

template<typename ActionT, typename... Args>
typename Server<ActionT>::SharedPtr create_server(Args&&... args)
{
    return ::rclcpp_action::create_server<ActionT>(std::forward<Args>(args)...);
}

#endif
} // namespace action
} // namespace roscomp
