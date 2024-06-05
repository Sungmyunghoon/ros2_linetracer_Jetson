#include "rclcpp/rclcpp.hpp" //ROS2의 C++ 클라이언트 라이브러리
#include "geometry_msgs/msg/vector3.hpp" //3D 벡터 메시지
#include "dxl/dxl.hpp" //dxl의 기능을 사용할 헤더파일
#include <memory>
#include <chrono>
#include "sensor_msgs/msg/compressed_image.hpp" //압축 이미지 메시지를 사용하기 위한 헤더 파일
#include "cv_bridge/cv_bridge.h" //OpenCV와 ROS 이미지 메시지 간의 변환을 도와주는 해더파일
#include "opencv2/opencv.hpp" //OpenCV 라이브러리를 포함
using namespace std;
using namespace cv;

//카메라의 영상 소스를 GStreamer을 사용
string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); //ROS2 노드 초기화를 위한 명령어
    
    //"camera_publisher"와 "cam_pub"라는 이름의 ROS2 노드를 생성
    auto node = rclcpp::Node::make_shared("camera_publisher");
    auto cam_node = std::make_shared<rclcpp::Node>("cam_pub");

    // QoS설정을 위해 최신 10개의 메시지를 유지
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    //"error_topic"과 "image/compressed"라는 두 개의 토픽에 퍼블리셔를 생성
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("error_topic", qos_profile );
    auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);

    //벡터 메시지 객체를 생성하고 초기화
    geometry_msgs::msg::Vector3 vel;
    vel.x = 10;
    vel.y = 10;
    vel.z = 0;

    //루프 주기를 30Hz로 설정
    rclcpp::WallRate loop_rate(30.0);

    //메시지 헤더 객체와 압축 이미지 메시지 공유 포인터를 선언
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;

    //카메라를 열기
    VideoCapture cap(/*"/home/jetson/ros2_ws/src/linetracer/src/5_lt_cw_100rpm_out.mp4"*/ src, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Camera error");
        return -1;
    }

    //프레임을 카메라에 전송
    Mat frame;
    while (rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Captured frame is empty");
            continue;
        }

        //OpenCV 이미지를 ROS 압축 이미지 메시지로 변환
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        
        //변환된 압축 이미지 메시지를 퍼블리시
        publisher->publish(*msg);

        //루프 주기를 유지
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
