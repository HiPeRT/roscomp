#pragma once

#include "roscomp/roscomp.hpp"

#if TKROS_VERSION == 1
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#define TK_COMPOSITION_EXPORT(NodeClass) PLUGINLIB_EXPORT_CLASS(NodeClass, nodelet::Nodelet)

using CompositionBase = nodelet::Nodelet;

#elif TKROS_VERSION == 2
#include <rclcpp_components/register_node_macro.hpp>
#include "roscomp/visibility_control.h"

#define TK_COMPOSITION_EXPORT(NodeClass) RCLCPP_COMPONENTS_REGISTER_NODE(NodeClass)

using CompositionBase = rclcpp::Node;

#endif

namespace roscomp {

#if TKROS_VERSION == 1

class CompositionClass : public CompositionBase
{
  public:
    explicit CompositionClass()
      : CompositionClass(NodeOptions())
    {}
    explicit CompositionClass(const NodeOptions& options)
      : CompositionBase()
    {
        (void)options;
    }

    void init()
    {
        nodelet::V_string my_argv;
        CompositionBase::init(
          ::ros::this_node::getName(), ::ros::names::getRemappings(), my_argv, nullptr, nullptr);
    }

    virtual void onInit() final
    {
        mNode = std::make_shared<Node>(getMTPrivateNodeHandle());
        onCompositionInit();
    }

    virtual void onCompositionInit() = 0;

    Node& getNode() { return *mNode; }
    std::shared_ptr<Node> getNodePtr() { return mNode; }

  private:
    Node::SharedPtr mNode;
};

#elif TKROS_VERSION == 2

class CompositionClass : public CompositionBase
{
  public:
    COMPOSITION_PUBLIC
    explicit CompositionClass()
      : CompositionClass(NodeOptions())
    {}
    COMPOSITION_PUBLIC
    explicit CompositionClass(const NodeOptions& options)
      : CompositionBase("CompositionNode", options)
    {}

    void init() { onInit(); }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        onInit();
        return getNodePtr()->get_node_base_interface();
    }

    void onInit()
    {
        if (!mInitialized) {
            mInitialized = true;
            onCompositionInit();
        }
    }

    virtual void onCompositionInit() = 0;

    Node& getNode() { return *this; }
    Node::SharedPtr getNodePtr() { return shared_from_this(); }

  private:
    bool mInitialized = false;
};
#endif

} // namespace roscomp
