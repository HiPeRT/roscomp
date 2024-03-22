#include <cassert>

#include "roscomp/roscomp.hpp"

namespace roscomp {

// Node class
Node::Node()
  : nh_(std::make_shared<::ros::NodeHandle>("~"))
{}

Node::Node(const ::ros::NodeHandle& nh)
  : nh_(std::make_shared<::ros::NodeHandle>(nh))
{}

Node::Node(const std::string& string, const NodeOptions& aNodeOptions)
  : nh_(std::make_shared<::ros::NodeHandle>(string))
{
    (void)aNodeOptions;
}

Node::~Node()
{
    nh_.reset();
}

size_t Node::count_publishers(const std::string& aTopicName) const
{
    for (auto sub_tuple : mSubs) {
        if (sub_tuple.second == aTopicName) {
            return sub_tuple.first->getNumPublishers();
        }
    }
    return 0;
}

size_t Node::count_subscribers(const std::string& aTopicName) const
{
    for (auto pub_tuple : mPubs) {
        if (pub_tuple.second == aTopicName) {
            return pub_tuple.first->getNumSubscribers();
        }
    }
    return 0;
}

Node::SharedPtr create_node(std::string node_name)
{
    (void)node_name;
    return std::make_shared<Node>();
}

std::shared_ptr<CallbackGroup> Node::create_callback_group(const CallbackGroupType& aCbkGrpType)
{
    assert(aCbkGrpType == CallbackGroupType::MutuallyExclusive);
    std::shared_ptr<CallbackGroup> cbkq = std::make_shared<CallbackGroup>();
    mCallbackGroups.push_back(cbkq);
    return mCallbackGroups.back();
}

void init(int argc, char** argv, std::string node_name)
{
    ::ros::init(argc, argv, node_name);
}

void init(int argc, char** argv, std::string node_name, init_options options)
{
    ::ros::init(argc, argv, node_name, static_cast<::ros::InitOption>(options));
}

bool ok()
{
    return ::ros::ok();
}

void shutdown()
{
    ::ros::shutdown();
}

void spin(Node node)
{
    (void)node;
    ::ros::spin();
}

void spin(Node::SharedPtr node)
{
    (void)node;
    ::ros::spin();
}

void spin_some(Node node)
{
    (void)node;
    ::ros::spinOnce();
}

void spin_some(Node::SharedPtr node)
{
    (void)node;
    ::ros::spinOnce();
}

std::string get_name(Node::SharedPtr aNode)
{
    (void)aNode;
    return ::ros::this_node::getName();
}

void DEBUG(const char* format, ...)
{
    va_list args;
    ROS_DEBUG(format, args);
}

void INFO(const char* format, ...)
{
    va_list args;
    ROS_INFO(format, args);
}

void WARN(const char* format, ...)
{
    va_list args;
    ROS_WARN(format, args);
}

void ERROR(const char* format, ...)
{
    va_list args;
    ROS_ERROR(format, args);
}

} // namespace roscomp
