#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

class NavigationCalculatorNode : public rclcpp::Node
{
public:
    NavigationCalculatorNode() : Node("navigation_calculator_node")
    {
        current_position_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "gps/current_position", 10, std::bind(&NavigationCalculatorNode::calculate_navigation, this, std::placeholders::_1));
        
        current_waypoint_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "navigation/current_waypoint", 10, std::bind(&NavigationCalculatorNode::update_current_waypoint, this, std::placeholders::_1));

        distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("navigation/distance_to_waypoint", 10);
        direction_publisher_ = this->create_publisher<std_msgs::msg::Float32>("navigation/navigation_direction", 10);
    }

private:
    void update_current_waypoint(const geometry_msgs::msg::Point::SharedPtr waypoint_msg)
    {
        current_waypoint_ = *waypoint_msg;
    }

    void calculate_navigation(const geometry_msgs::msg::Point::SharedPtr position_msg)
    {
        if (!current_waypoint_) {
            RCLCPP_WARN(this->get_logger(), "No current waypoint set.");
            return;
        }

        double distance = calculate_distance(position_msg->x, position_msg->y, current_waypoint_->x, current_waypoint_->y);
        double direction = calculate_direction(position_msg->x, position_msg->y, current_waypoint_->x, current_waypoint_->y);

        auto distance_msg = std_msgs::msg::Float32();
        distance_msg.data = static_cast<float>(distance);
        distance_publisher_->publish(distance_msg);

        auto direction_msg = std_msgs::msg::Float32();
        direction_msg.data = static_cast<float>(direction);
        direction_publisher_->publish(direction_msg);

        RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f, Direction: %f", distance, direction);
    }

    double calculate_distance(double lat1, double lon1, double lat2, double lon2)
    {
        // Utilisation de la formule de Haversine pour calculer la distance entre deux points GPS
        double R = 6371000; // Rayon de la Terre en mètres
        double phi1 = lat1 * M_PI / 180.0;
        double phi2 = lat2 * M_PI / 180.0;
        double delta_phi = (lat2 - lat1) * M_PI / 180.0;
        double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

        double a = std::sin(delta_phi / 2) * std::sin(delta_phi / 2) +
                   std::cos(phi1) * std::cos(phi2) *
                   std::sin(delta_lambda / 2) * std::sin(delta_lambda / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        return R * c; // Distance en mètres
    }

    double calculate_direction(double lat1, double lon1, double lat2, double lon2)
    {
        double y = std::sin(lon2 - lon1) * std::cos(lat2);
        double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(lon2 - lon1);
        return std::atan2(y, x) * 180.0 / M_PI; // Direction en degrés
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr current_position_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr current_waypoint_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr direction_publisher_;
    std::optional<geometry_msgs::msg::Point> current_waypoint_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationCalculatorNode>());
    rclcpp::shutdown();
    return 0;
}
