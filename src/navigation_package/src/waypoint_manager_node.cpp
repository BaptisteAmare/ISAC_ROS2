#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int32.hpp"
#include "navigation_package/srv/add_waypoint.hpp"  
#include "navigation_package/srv/remove_waypoint.hpp"

class WaypointManagerNode : public rclcpp::Node
{
public:
    WaypointManagerNode() : Node("waypoint_manager_node")
    {
        add_waypoint_service_ = this->create_service<navigation_package::srv::AddWaypoint>(
            "add_waypoint", std::bind(&WaypointManagerNode::add_waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));

        remove_waypoint_by_index_service_ = this->create_service<navigation_package::srv::RemoveWaypoint>(
            "remove_waypoint_by_index", std::bind(&WaypointManagerNode::remove_waypoint_by_index_callback, this, std::placeholders::_1, std::placeholders::_2));

        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("navigation/current_waypoint", 10);
        waypoints_remaining_publisher_ = this->create_publisher<std_msgs::msg::Int32>("navigation/remaining_waypoints", 10);

        // Initialisation de la liste des waypoints avec un point correctement construit
        geometry_msgs::msg::Point waypoint;
        waypoint.x = 48.858844; // Latitude
        waypoint.y = 2.294351;  // Longitude
        waypoint.z = 0.0;       // Altitude
        waypoints_.push_back(waypoint); // Ajouter le waypoint à la liste

        publish_waypoints_info();
    }

private:
    void add_waypoint_callback(const std::shared_ptr<navigation_package::srv::AddWaypoint::Request> request,
                               std::shared_ptr<navigation_package::srv::AddWaypoint::Response> response)
    {
        // Ajouter le waypoint fourni dans la requête
        geometry_msgs::msg::Point new_waypoint = request->point;
        waypoints_.push_back(new_waypoint);

        // Index du nouveau waypoint (basé sur sa position dans le vecteur)
        int32_t waypoint_index = waypoints_.size() - 1;

        response->success = true;
        response->index = waypoint_index;
        response->message = "Waypoint ajouté avec succès";
        RCLCPP_INFO(this->get_logger(), "Waypoint ajouté : [%f, %f, %f] avec l'index %d", new_waypoint.x, new_waypoint.y, new_waypoint.z, waypoint_index);

        publish_waypoints_info();
    }

    void remove_waypoint_by_index_callback(const std::shared_ptr<navigation_package::srv::RemoveWaypoint::Request> request,
                                       std::shared_ptr<navigation_package::srv::RemoveWaypoint::Response> response)
    {
        if (request->index >= 0 && request->index < static_cast<int>(waypoints_.size())) {
            waypoints_.erase(waypoints_.begin() + request->index);
            response->success = true;
            response->message = "Waypoint supprimé avec succès";
            RCLCPP_INFO(this->get_logger(), "Waypoint à l'index %d supprimé", request->index);
        } else {
            response->success = false;
            response->message = "Index invalide";
            RCLCPP_WARN(this->get_logger(), "Échec de la suppression : index %d invalide", request->index);
        }

        publish_waypoints_info();
    }

    void publish_waypoints_info()
    {
        if (!waypoints_.empty()) {
            waypoint_publisher_->publish(waypoints_.front());
        }
        auto remaining_msg = std_msgs::msg::Int32();
        remaining_msg.data = waypoints_.size();
        waypoints_remaining_publisher_->publish(remaining_msg);
    }

    rclcpp::Service<navigation_package::srv::AddWaypoint>::SharedPtr add_waypoint_service_;
    rclcpp::Service<navigation_package::srv::RemoveWaypoint>::SharedPtr remove_waypoint_by_index_service_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoints_remaining_publisher_;
    std::vector<geometry_msgs::msg::Point> waypoints_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointManagerNode>());
    rclcpp::shutdown();
    return 0;
}