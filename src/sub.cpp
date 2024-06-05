#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "dxl/dxl.hpp"
#include <memory>
#include <functional>

using namespace std;
using namespace std::placeholders;
void error_callback(rclcpp::Node::SharedPtr node, Dxl& mx, const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Received message: %lf,%lf", msg->x,msg->y);
    mx.setVelocity((int)msg->x, (int)msg->y);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    Dxl mx;
    auto node = rclcpp::Node::make_shared("dxl_subscriber");
    if (!mx.open()) {
        RCLCPP_ERROR(node->get_logger(), "Dynamixel open error");
        return -1;
    }
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    function<void(const geometry_msgs::msg::Vector3::SharedPtr msg)> fn;
    fn = bind(error_callback, node, mx, _1);
    auto error_subscriber = node->create_subscription<geometry_msgs::msg::Vector3>(
        "error_topic", qos_profile, fn);

    rclcpp::spin(node);

    mx.close();
    rclcpp::shutdown();
    return 0;
}