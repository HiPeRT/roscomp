#include "roscomp/roscomp.hpp"

namespace roscomp {

Node::SharedPtr create_node(std::string node_name)
{
    return std::make_shared<rclcpp::Node>(node_name);
}

void init(int argc, char** argv, std::string node_name)
{
    (void)node_name;
    rclcpp::init(argc, argv);
}

void init(int argc, char** argv, std::string node_name, init_options options)
{
    (void)node_name;
    if (options == init_options::NoSigIntHandler) {
        rclcpp::InitOptions ros2_init_options = rclcpp::InitOptions();
        ros2_init_options.shutdown_on_sigint = false;
        rclcpp::init(argc, argv, ros2_init_options);
    } else
        exit(-1);
}

bool ok()
{
    return rclcpp::ok();
}

void shutdown()
{
    rclcpp::shutdown();
}

void spin(Node::SharedPtr node)
{
    rclcpp::spin(node);
}

void spin_some(Node::SharedPtr node)
{
    rclcpp::spin_some(node);
}

std::string get_name(Node::SharedPtr aNode)
{
    return aNode->get_name();
}

void DEBUG(const char* format, ...)
{
    va_list args;
    RCLCPP_DEBUG(rclcpp::get_logger("global_info"), format, args);
}

void INFO(const char* format, ...)
{
    va_list args;
    RCLCPP_INFO(rclcpp::get_logger("global_info"), format, args);
}

void WARN(const char* format, ...)
{
    va_list args;
    RCLCPP_WARN(rclcpp::get_logger("global_warn"), format, args);
}

void ERROR(const char* format, ...)
{
    va_list args;
    RCLCPP_ERROR(rclcpp::get_logger("global_error"), format, args);
}

} // namespace roscomp
