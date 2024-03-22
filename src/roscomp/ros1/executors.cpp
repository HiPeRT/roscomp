#include "roscomp/roscomp.hpp"

namespace roscomp {

CallbackGroup::CallbackGroup()
{
    mCallbackQueue = std::make_shared<::ros::CallbackQueue>();
}

SubscriptionOptions::SubscriptionOptions() {}

namespace executors {

MultiThreadedExecutor::MultiThreadedExecutor() {}

void MultiThreadedExecutor::add_node(std::shared_ptr<Node> aNode)
{
    mNodes.push_back(aNode);

    ::ros::AsyncSpinner spinner(aNode->mCallbackGroups.size());
    mSpinners.push_back(spinner);
}

void MultiThreadedExecutor::spin()
{
    for (auto& spinner : mSpinners) {
        spinner.start();
    }

    ::ros::waitForShutdown();
}

} // namespace executors
} // namespace roscomp
