 #include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"

class MonitoringDisplayNode : public rclcpp::Node
{
public:
    MonitoringDisplayNode() : Node("monitoring_display_node")
    {
        distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "navigation/distance_to_waypoint", 10, std::bind(&MonitoringDisplayNode::distance_callback, this, std::placeholders::_1));

        direction_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "navigation/navigation_direction", 10, std::bind(&MonitoringDisplayNode::direction_callback, this, std::placeholders::_1));

        remaining_waypoints_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "navigation/remaining_waypoints", 10, std::bind(&MonitoringDisplayNode::remaining_waypoints_callback, this, std::placeholders::_1));
    }

private:
    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f meters", msg->data);
    }

    void direction_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Direction to waypoint: %f degrees", msg->data);
    }

    void remaining_waypoints_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Remaining waypoints: %d", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr direction_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr remaining_waypoints_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitoringDisplayNode>());
    rclcpp::shutdown();
    return 0;
}