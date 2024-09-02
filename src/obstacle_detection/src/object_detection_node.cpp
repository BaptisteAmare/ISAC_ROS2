#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ament_index_cpp/get_package_share_directory.hpp"  // Pour obtenir le chemin du package

class ObjectDetectionNode : public rclcpp::Node
{
public:
    ObjectDetectionNode() : Node("object_detection_node")
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("obstacle_detection");
        std::string yolo_cfg_path = package_share_directory + "/ressources/yolov3-tiny.cfg";
        std::string yolo_weights_path = package_share_directory + "/ressources/yolov3-tiny.weights";

        // Load YOLO model
        try {
            net_ = cv::dnn::readNetFromDarknet(yolo_cfg_path, yolo_weights_path);
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            output_layer_names_ = net_.getUnconnectedOutLayersNames();

            RCLCPP_INFO(this->get_logger(), "YOLO model loaded successfully.");
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading YOLO model: %s", e.what());
            rclcpp::shutdown();
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&ObjectDetectionNode::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("object_detection/coordinates", 10);

        // Set up a timer to control inference frequency
        inference_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // Adjust the frequency (e.g., 500ms for 2Hz)
            std::bind(&ObjectDetectionNode::perform_inference, this));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Store the latest image
        last_image_ = msg;
    }

    void perform_inference()
    {
        if (!last_image_) {
            return;  // No image received yet
        }

        // Perform inference on the latest image
        detect_objects(last_image_);
    }

    void detect_objects(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
        net_.setInput(blob);

        std::vector<cv::Mat> outs;
        net_.forward(outs, output_layer_names_);

        std_msgs::msg::Float32MultiArray coordinates_msg;
        extract_coordinates(outs, coordinates_msg, frame);
        
        if (coordinates_msg.data.empty()) {
            RCLCPP_INFO(this->get_logger(), "No objects detected.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Detected %zu objects.", coordinates_msg.data.size() / 2);
        }

        publisher_->publish(coordinates_msg);
    }

    void extract_coordinates(const std::vector<cv::Mat>& outs, std_msgs::msg::Float32MultiArray& coordinates_msg, const cv::Mat& frame)
    {
        for (const auto& out : outs)
        {
            for (int i = 0; i < out.rows; ++i)
            {
                const int probability_index = 5;
                const int probability_size = out.cols - probability_index;
                float* data = (float*)out.data;
                for (int j = 0; j < probability_size; ++j)
                {
                    if (data[probability_index + j] > 0.3) // Assuming a detection threshold of 0.3
                    {
                        float center_x = data[0] * frame.cols;
                        float center_y = data[1] * frame.rows;
                        coordinates_msg.data.push_back(center_x);
                        coordinates_msg.data.push_back(center_y);
                    }
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr inference_timer_;
    cv::dnn::Net net_;
    std::vector<std::string> output_layer_names_;

    sensor_msgs::msg::Image::SharedPtr last_image_;  // Store the last received image
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
