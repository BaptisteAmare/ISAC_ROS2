#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

class GPSTrackerNode : public rclcpp::Node
{
public:
    GPSTrackerNode() : Node("gps_tracker_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("gps/current_position", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GPSTrackerNode::publish_position, this));
    }

private:
    void publish_position()
    {
        auto message = geometry_msgs::msg::Point();
        // Simulation de la position GPS (coordonnées fictives pour l'exemple)
        message.x = 48.858844; // Latitude
        message.y = 2.294351;  // Longitude
        message.z = 0.0;       // Altitude
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing current GPS position: [%f, %f, %f]", message.x, message.y, message.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
