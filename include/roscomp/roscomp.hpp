#pragma once

#if TKROS_VERSION == 1
#include <atomic>
#include <chrono>
#include <functional>
#include <future>

#include <ros/ros.h>

#elif TKROS_VERSION == 2
#include <rclcpp/rclcpp.hpp>

#endif

#include "roscomp/executors.hpp"

namespace roscomp {

#if TKROS_VERSION == 1

enum class FutureReturnCode
{
    SUCCESS,
    INTERRUPTED,
    TIMEOUT
};

static constexpr int DEFAULT_CACHE_TIME = 10;

class Node;

template<typename ResponseT>
FutureReturnCode spin_until_future_complete(std::shared_ptr<Node> node,
                                            const std::shared_future<ResponseT> future,
                                            std::chrono::duration<int64_t, std::milli> timeout =
                                              std::chrono::duration<int64_t, std::milli>(-1));

class TimerBase
{
  public:
    using SharedPtr = std::shared_ptr<TimerBase>;

    TimerBase(std::shared_ptr<::ros::NodeHandle> aNode,
              std::chrono::nanoseconds aNs,
              std::function<void()> aCallback)
    {
        mTimer = std::make_shared<::ros::Timer>(
          aNode->createTimer(::ros::Duration(aNs.count() / 1e9),
                             [aCallback](const ::ros::TimerEvent&) { aCallback(); }));
    }

    ~TimerBase() {}

  private:
    std::shared_ptr<::ros::Timer> mTimer;
};

class SubscriptionBase
{
    friend class Node;
    virtual size_t getNumPublishers() = 0;
};

template<typename MessageT>
class Subscription : public SubscriptionBase
{
    friend class Node;

  public:
    using SharedPtr = std::shared_ptr<Subscription>;

    Subscription(::ros::Subscriber sub)
      : sub_(std::make_shared<::ros::Subscriber>(sub))
    {}

    ~Subscription() { sub_.reset(); }

  private:
    std::shared_ptr<::ros::Subscriber> sub_;

    virtual size_t getNumPublishers() override
    {
        return static_cast<size_t>(sub_->getNumPublishers());
    }
};

class PublisherBase
{
    friend class Node;
    virtual size_t getNumSubscribers() = 0;
};

template<typename MessageT>
class Publisher : public PublisherBase
{
    friend class Node;

  public:
    using SharedPtr = std::shared_ptr<Publisher>;

    Publisher(::ros::Publisher pub)
      : pub_(std::make_shared<::ros::Publisher>(pub))
    {}

    ~Publisher() { pub_.reset(); }

    void publish(const typename MessageT::Ptr& msg) { pub_->publish(msg); }
    void publish(const MessageT& msg) { pub_->publish(msg); }

  private:
    std::shared_ptr<::ros::Publisher> pub_;

    size_t getNumSubscribers() override { return static_cast<size_t>(pub_->getNumSubscribers()); }
};

template<typename ServiceT>
class Service
{
  public:
    using SharedPtr = std::shared_ptr<Service>;

    Service(std::shared_ptr<::ros::NodeHandle>& nh,
            std::string topic_name,
            std::function<bool(const std::shared_ptr<typename ServiceT::Request>,
                               std::shared_ptr<typename ServiceT::Response>)> callbackFunction)
      : callback_(callbackFunction)
    {
        service_ = std::make_shared<::ros::ServiceServer>(
          nh->advertiseService(topic_name, &Service::serviceCallback, this));
    }

    ~Service() { service_.reset(); }

  private:
    std::shared_ptr<::ros::ServiceServer> service_;
    std::function<bool(const std::shared_ptr<typename ServiceT::Request>,
                       std::shared_ptr<typename ServiceT::Response>)>
      callback_;

    bool serviceCallback(typename ServiceT::Request& aRequest,
                         typename ServiceT::Response& aResponse)
    {
        const std::shared_ptr<typename ServiceT::Request> request_out =
          std::make_shared<typename ServiceT::Request>(aRequest);
        std::shared_ptr<typename ServiceT::Response> response_out =
          std::make_shared<typename ServiceT::Response>();

        bool status = callback_(request_out, response_out);

        aResponse = *response_out;
        return status;
    }
};

template<typename ServiceT>
class Client
{
  public:
    using SharedPtr = std::shared_ptr<Client>;

    using SharedRequest = std::shared_ptr<typename ServiceT::Request>;
    using SharedResponse = std::shared_ptr<typename ServiceT::Response>;

    using Promise = std::promise<SharedResponse>;
    using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;

    using SharedPromise = std::shared_ptr<Promise>;
    using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;

    using SharedFuture = std::shared_future<SharedResponse>;
    using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;

    Client(::ros::ServiceClient client, std::string& topic_name)
      : client_(std::make_shared<::ros::ServiceClient>(client))
      , topic_name_(topic_name)
      , running_(false)
      , running_callback_(false)
    {}

    ~Client()
    {
        call_thread_.join();
        running_callback_.store(false);
        callback_thread_.join();
        client_.reset();
    }

    bool wait_for_service(std::chrono::duration<int64_t, std::milli> timeout =
                            std::chrono::duration<int64_t, std::milli>(-1))
    {
        return wait_for_service_nanoseconds(
          std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
    }

    SharedFuture async_send_request(std::shared_ptr<typename ServiceT::Request> request)
    {
        if (running_) {
            call_thread_.join();
            running_ = false;
        }

        srv_msg_.request = *request;

        SharedPromise call_promise = std::make_shared<Promise>();

        SharedFuture future(call_promise->get_future());

        pending_request_ = std::make_tuple(call_promise, future);

        call_thread_ = std::thread(&Client::send_request, this);
        // send_request();

        return future;
    }

    SharedFuture async_send_request(std::shared_ptr<typename ServiceT::Request> request,
                                    std::function<void(SharedFuture)> callback)
    {
        if (running_callback_.load()) {
            running_callback_.store(false);
        }
        if (callback_thread_.joinable()) {
            callback_thread_.join();
        }
        running_callback_.store(true);
        SharedFuture future = async_send_request(request);
        callback_thread_ = std::thread(&Client<ServiceT>::private_spin, this, future, callback);
        return future;
    }

  private:
    std::shared_ptr<::ros::ServiceClient> client_;
    std::string topic_name_;

    ServiceT srv_msg_;
    std::tuple<SharedPromise, SharedFuture> pending_request_;

    std::atomic<bool> running_;
    std::thread call_thread_;
    std::atomic<bool> running_callback_;
    std::thread callback_thread_;

    bool wait_for_service_nanoseconds(std::chrono::nanoseconds timeout)
    {
        static uint64_t sTmp;
        sTmp = timeout.count();
        return ::ros::service::waitForService(
          topic_name_,
          ::ros::Duration(static_cast<int32_t>(sTmp / static_cast<int64_t>(1e9)),
                          static_cast<int32_t>(sTmp) % static_cast<int32_t>(1e9)));
    }

    void send_request()
    {
        std::shared_ptr<typename ServiceT::Response> response_ =
          std::make_shared<typename ServiceT::Response>();
        running_ = true;

        client_->call(srv_msg_);

        auto call_promise = std::get<0>(pending_request_);
        auto future = std::get<1>(pending_request_);

        *response_ = srv_msg_.response;

        call_promise->set_value(response_);
    }

    void private_spin(SharedFuture future, std::function<void(SharedFuture)> callback)
    {
        while (spin_until_future_complete(std::make_shared<Node>(), future) !=
               FutureReturnCode::SUCCESS) {
            if (!running_callback_.load()) {
                running_callback_.store(false);
                return;
            }
        }
        callback(future);
        running_callback_.store(false);
    }
};

class NodeOptions
{};

class CallbackGroup; // forward declaration
class SubscriptionOptions
{
  public:
    SubscriptionOptions();

    std::shared_ptr<CallbackGroup> callback_group;
};
enum class CallbackGroupType
{
    MutuallyExclusive,
    Reentrant
};

class Node : public std::enable_shared_from_this<Node>
{
  public:
    using SharedPtr = std::shared_ptr<Node>;
    Node();
    Node(const ::ros::NodeHandle& nh);
    Node(const std::string& string, const NodeOptions& aNodeOptions = NodeOptions());

    ~Node();

    ::ros::Duration get_clock() { return ::ros::Duration(DEFAULT_CACHE_TIME); }

    template<typename MessageT> //, typename... Args>
    typename Subscription<MessageT>::SharedPtr create_subscription(
      const std::string& aTopic,
      const uint32_t& aDepth,
      std::function<void(const boost::shared_ptr<const MessageT>&)> aCallback,
      const SubscriptionOptions& aSubOpt = SubscriptionOptions())
    {
        (void)aSubOpt;

        typename Subscription<MessageT>::SharedPtr sub = std::make_shared<Subscription<MessageT>>(
          nh_->subscribe<MessageT>(aTopic, aDepth, aCallback));

        mSubs.push_back(std::make_pair(sub, aTopic));

        return sub;
    }

    template<typename MessageT, typename... Args>
    typename Publisher<MessageT>::SharedPtr create_publisher(const std::string& aTopic,
                                                             Args&&... args)
    {
        typename Publisher<MessageT>::SharedPtr pub = std::make_shared<Publisher<MessageT>>(
          nh_->advertise<MessageT>(aTopic, std::forward<Args>(args)...));

        mPubs.push_back(std::make_pair(pub, aTopic));

        return pub;
    }

    template<typename ServiceT>
    typename Service<ServiceT>::SharedPtr create_service(
      std::string topic_name,
      std::function<bool(const std::shared_ptr<typename ServiceT::Request>,
                         std::shared_ptr<typename ServiceT::Response>)> callback_function)
    {
        return std::make_shared<Service<ServiceT>>(nh_, topic_name, callback_function);
    }

    template<typename ServiceT>
    typename Client<ServiceT>::SharedPtr create_client(std::string topic_name)
    {
        return std::make_shared<Client<ServiceT>>(nh_->serviceClient<ServiceT>(topic_name),
                                                  topic_name);
    }

    template<typename T>
    void param(std::string name, T& out, T def)
    {
        nh_->param<T>(name, out, def);
    }

    std::shared_ptr<CallbackGroup> create_callback_group(const CallbackGroupType& aCbkGrpType);

    template<typename DurationRepT = int64_t, typename DurationT = std::milli>
    std::shared_ptr<TimerBase> create_wall_timer(
      std::chrono::duration<DurationRepT, DurationT> aPeriod,
      std::function<void()> aCallback)
    {
        return std::make_shared<TimerBase>(
          nh_, std::chrono::duration_cast<std::chrono::nanoseconds>(aPeriod), aCallback);
    }

    size_t count_publishers(const std::string& aTopicName) const;
    size_t count_subscribers(const std::string& aTopicName) const;

    std::vector<std::shared_ptr<CallbackGroup>> mCallbackGroups;

    std::shared_ptr<::ros::NodeHandle> nh_;

  private:
    std::vector<std::pair<std::shared_ptr<SubscriptionBase>, std::string>> mSubs;
    std::vector<std::pair<std::shared_ptr<PublisherBase>, std::string>> mPubs;
};

#elif TKROS_VERSION == 2
typedef rclcpp::FutureReturnCode FutureReturnCode;

typedef rclcpp::Node Node;
typedef rclcpp::NodeOptions NodeOptions;
typedef rclcpp::SubscriptionOptions SubscriptionOptions;

template<typename MessageT>
using Subscription = rclcpp::Subscription<MessageT>;
// typedef rclcpp::Subscription Subscription;

template<typename MessageT>
using Publisher = rclcpp::Publisher<MessageT>;
// typedef rclcpp::Publisher Publisher;

template<typename MessageT>
using Client = rclcpp::Client<MessageT>;

template<typename MessageT>
using Service = rclcpp::Service<MessageT>;

#endif

#ifdef TKROS_VERSION

enum class init_options
{
    NoSigIntHandler = 1
};

#if TKROS_VERSION == 1
template<typename T>
void parameter(Node::SharedPtr node, std::string name, T& out, T def)
{
    node->param<T>(name, out, def);
}
template<typename T>
void parameter(Node* node, std::string name, T& out, T def)
{
    node->param<T>(name, out, def);
}
#elif TKROS_VERSION == 2
template<typename T>
void parameter(Node::SharedPtr node, std::string name, T& out, T def)
{
    node->declare_parameter<T>(name, def);
    node->get_parameter<T>(name, out);
}
template<typename T>
void parameter(Node* node, std::string name, T& out, T def)
{
    node->declare_parameter<T>(name, def);
    node->get_parameter<T>(name, out);
}
#endif

void init(int argc, char** argv, std::string node_name);
void init(int argc, char** argv, std::string node_name, init_options options);
void shutdown();
bool ok();

void spin(Node::SharedPtr node);
void spin_some(Node::SharedPtr node);

#if TKROS_VERSION == 1
template<typename ResponseT>
FutureReturnCode spin_until_future_complete(Node::SharedPtr node,
                                            const std::shared_future<ResponseT> future,
                                            std::chrono::duration<int64_t, std::milli> timeout)
{
    (void)node;

    std::future_status status = future.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
        return FutureReturnCode::SUCCESS;
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::nanoseconds timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
    if (timeout_ns > std::chrono::nanoseconds::zero()) {
        end_time += timeout_ns;
    }
    std::chrono::nanoseconds timeout_left = timeout_ns;

    while (::ros::ok()) {
        // Do one item of work.
        ::ros::spinOnce();

        // Check if the future is set, return SUCCESS if it is.
        status = future.wait_for(std::chrono::seconds(0));
        if (status == std::future_status::ready) {
            return FutureReturnCode::SUCCESS;
        }
        // If the original timeout is < 0, then this is blocking, never TIMEOUT.
        if (timeout_ns < std::chrono::nanoseconds::zero()) {
            continue;
        }
        // Otherwise check if we still have time to wait, return TIMEOUT if not.
        auto now = std::chrono::steady_clock::now();
        if (now >= end_time) {
            return FutureReturnCode::TIMEOUT;
        }
        // Subtract the elapsed time from the original timeout.
        timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
    }

    // The future did not complete before ok() returned false, return INTERRUPTED.
    return FutureReturnCode::INTERRUPTED;
}
#elif TKROS_VERSION == 2

// perfect forwarding
template<typename... Args>
auto spin_until_future_complete(Args&&... args)
  -> decltype(::rclcpp::spin_until_future_complete(std::forward<Args>(args)...))
{
    return ::rclcpp::spin_until_future_complete(std::forward<Args>(args)...);
}
#endif

Node::SharedPtr create_node(std::string node_name);

std::string get_name(Node::SharedPtr aNode);

void DEBUG(const char* format, ...);
void INFO(const char* format, ...);
void WARN(const char* format, ...);
void ERROR(const char* format, ...);

#endif

} // namespace roscomp
