#include "rclcpp/rclcpp.hpp" // ROS2의 C++ 클라이언트 라이브러리
#include "geometry_msgs/msg/vector3.hpp" // 기하학적 벡터 메시지를 포함
#include "dxl/dxl.hpp" // dxl 라이브러리를 포함
#include <memory> // 스마트 포인터를 사용하기 위한 헤더 파일
#include <functional> // 함수 객체를 사용하기 위한 헤더 파일

using namespace std;
using namespace std::placeholders;

//메시지를 수신할 때 호출되는 콜백 함수입니다.
void error_callback(rclcpp::Node::SharedPtr node, Dxl& mx, const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Received message: %lf,%lf", msg->x,msg->y); // 수신된 메시지를 로깅합니다.
    mx.setVelocity((int)msg->x, (int)msg->y); // 수신된 메시지의 값을 사용하여 Dxl 모터의 속도를 설정합니다.
}

// 프로그램의 메인 함수
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); //  ROS2 노드를 초기화
    Dxl mx; // dxl객체 생성
    auto node = rclcpp::Node::make_shared("dxl_subscriber"); // "dxl_subscriber"라는 이름의 노드를 생성
    if (!mx.open()) {
        RCLCPP_ERROR(node->get_logger(), "Dynamixel open error");
        return -1;
    }
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // Qos를 설정
    function<void(const geometry_msgs::msg::Vector3::SharedPtr msg)> fn; // 콜백 함수를 위한 std::function 객체를 선언
    fn = bind(error_callback, node, mx, _1); // 콜백 함수를 바인딩
    auto error_subscriber = node->create_subscription<geometry_msgs::msg::Vector3>(
        "error_topic", qos_profile, fn); // "error_topic" 토픽에 서브스크립션을 생성

    rclcpp::spin(node);

    mx.close();
    rclcpp::shutdown();
    return 0;
}
