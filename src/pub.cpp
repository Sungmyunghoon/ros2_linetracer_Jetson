#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "dxl/dxl.hpp"
#include <memory>
#include <chrono>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("camera_publisher");
    auto cam_node = std::make_shared<rclcpp::Node>("cam_pub");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("error_topic", qos_profile );
    auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);
    geometry_msgs::msg::Vector3 vel;
    vel.x = 10;
    vel.y = 10;
    vel.z = 0;

    rclcpp::WallRate loop_rate(30.0);

    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    VideoCapture cap(/*"/home/jetson/ros2_ws/src/linetracer/src/5_lt_cw_100rpm_out.mp4"*/ src, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Camera error");
        return -1;
    }

    Mat frame;
    while (rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Captured frame is empty");
            continue;
        }

        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        
        publisher->publish(*msg);

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}