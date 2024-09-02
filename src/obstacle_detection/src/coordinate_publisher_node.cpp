#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class CoordinatePublisherNode : public rclcpp::Node
{
public:
    CoordinatePublisherNode() : Node("coordinate_publisher_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("object_detection/coordinates", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CoordinatePublisherNode::publish_coordinates, this));
    }

private:
    void publish_coordinates()
    {
        auto message = std_msgs::msg::Float32MultiArray();
        // Example coordinates (x1, y1, x2, y2, ...)
        message.data = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinatePublisherNode>());
    rclcpp::shutdown();
    return 0;
}
