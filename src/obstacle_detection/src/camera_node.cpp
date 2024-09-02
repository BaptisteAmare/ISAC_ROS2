#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ament_index_cpp/get_package_share_directory.hpp"  

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("obstacle_detection");
        std::string video_path = package_share_directory + "/ressources/dog_video.mp4";
        
        video_capture_.open(video_path);
        if (!video_capture_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", video_path.c_str());
            rclcpp::shutdown();
        }
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),  // Réduit la fréquence à 1 Hz
            std::bind(&CameraNode::publish_frame, this));
    }

private:
    void publish_frame()
    {
        cv::Mat frame;
        if (!video_capture_.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "End of video file, rewinding.");
            video_capture_.set(cv::CAP_PROP_POS_FRAMES, 0); 
            return; 
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_capture_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
