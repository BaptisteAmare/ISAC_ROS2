#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "navigation_package/msg/waypoint_info.hpp"
#include "navigation_package/srv/add_waypoint.hpp"
#include "navigation_package/srv/remove_waypoint.hpp"
#include "nlohmann/json.hpp"
#include "navigation_package/srv/get_all_waypoints.hpp"

struct WaypointInfo
{
    int32_t index;                        // Waypoint index
    geometry_msgs::msg::Point coordinates; // Waypoint coordinates
    float distance_to_next_waypoint;       // Distance to next waypoint
    float direction_to_next_waypoint;      // Direction to next waypoint
    std::string status;                    // Waypoint status
};

class WaypointManagerNode : public rclcpp::Node
{
public:
    WaypointManagerNode() : Node("waypoint_manager_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing WaypointManagerNode...");

        add_waypoint_service_ = this->create_service<navigation_package::srv::AddWaypoint>(
            "add_waypoint", std::bind(&WaypointManagerNode::add_waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));
        remove_waypoint_by_index_service_ = this->create_service<navigation_package::srv::RemoveWaypoint>(
            "remove_waypoint_by_index", std::bind(&WaypointManagerNode::remove_waypoint_by_index_callback, this, std::placeholders::_1, std::placeholders::_2));
        get_all_waypoints_service_ = this->create_service<navigation_package::srv::GetAllWaypoints>(
            "get_all_waypoints", std::bind(&WaypointManagerNode::get_all_waypoints_callback, this, std::placeholders::_1, std::placeholders::_2));

        waypoint_info_publisher_ = this->create_publisher<navigation_package::msg::WaypointInfo>("navigation/waypoint_info", 10);
        waypoints_remaining_publisher_ = this->create_publisher<std_msgs::msg::Int32>("navigation/remaining_waypoints", 10);
        current_waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("navigation/current_waypoint", 10);

        distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "navigation/distance_to_waypoint", 10, std::bind(&WaypointManagerNode::update_distance, this, std::placeholders::_1));
        
        direction_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "navigation/navigation_direction", 10, std::bind(&WaypointManagerNode::update_direction, this, std::placeholders::_1));

        //add_sample_waypoint();
        RCLCPP_INFO(this->get_logger(), "WaypointManagerNode initialized successfully.");
    }

private:
    void add_waypoint_callback(const std::shared_ptr<navigation_package::srv::AddWaypoint::Request> request,
                           std::shared_ptr<navigation_package::srv::AddWaypoint::Response> response)
    {
        WaypointInfo new_waypoint;
        new_waypoint.index = active_waypoints_.size();
        new_waypoint.coordinates = request->point;
        new_waypoint.status = "Waiting";

        // If this is not the first waypoint, calculate the distance and direction to the previous waypoint
        if (!active_waypoints_.empty()) {
            WaypointInfo& previous_waypoint = active_waypoints_.back();
            new_waypoint.distance_to_next_waypoint = calculate_distance(previous_waypoint.coordinates.x, previous_waypoint.coordinates.y,
                                                                            new_waypoint.coordinates.x, new_waypoint.coordinates.y);
            new_waypoint.direction_to_next_waypoint = calculate_direction(previous_waypoint.coordinates.x, previous_waypoint.coordinates.y,
                                                                            new_waypoint.coordinates.x, new_waypoint.coordinates.y);
        } else {
            // For the first waypoint, no distance or direction to calculate, set arbitrarily to 10.
            new_waypoint.distance_to_next_waypoint = 10.0;
            new_waypoint.direction_to_next_waypoint = 10.0;
        }

        active_waypoints_.push_back(new_waypoint);

        response->success = true;
        response->index = new_waypoint.index;
        response->message = "Waypoint added";
        RCLCPP_INFO(this->get_logger(), "Waypoint added : [%f, %f, %f] with index %d", 
                    new_waypoint.coordinates.x, new_waypoint.coordinates.y, 
                    new_waypoint.coordinates.z, new_waypoint.index);

        update_waypoint_status();
        // Publish waypoint info after addition
        publish_waypoints_info();
    }

    void remove_waypoint_by_index_callback(const std::shared_ptr<navigation_package::srv::RemoveWaypoint::Request> request,
                                           std::shared_ptr<navigation_package::srv::RemoveWaypoint::Response> response)
    {
        if (request->index >= 0 && request->index < static_cast<int>(active_waypoints_.size())) {
            active_waypoints_.erase(active_waypoints_.begin() + request->index);
            response->success = true;
            response->message = "Waypoint Deleted";
        } else {
            response->success = false;
            response->message = "Invalid Index";
        }

        publish_waypoints_info();
    }

    void get_all_waypoints_callback(const std::shared_ptr<navigation_package::srv::GetAllWaypoints::Request> request,
                                std::shared_ptr<navigation_package::srv::GetAllWaypoints::Response> response)
    {
        nlohmann::json json_waypoints = nlohmann::json::array();

        for (const auto& waypoint : active_waypoints_) {
            nlohmann::json json_waypoint;
            json_waypoint["index"] = waypoint.index;
            json_waypoint["coordinates"] = {{"x", waypoint.coordinates.x}, {"y", waypoint.coordinates.y}, {"z", waypoint.coordinates.z}};
            json_waypoint["distance_to_next_waypoint"] = waypoint.distance_to_next_waypoint;
            json_waypoint["direction_to_next_waypoint"] = waypoint.direction_to_next_waypoint;
            json_waypoint["status"] = waypoint.status;

            json_waypoints.push_back(json_waypoint);
        }

        response->json_waypoints = json_waypoints.dump();
    }

    void update_distance(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!active_waypoints_.empty()) {
            active_waypoints_.front().distance_to_next_waypoint = msg->data;
            update_waypoint_status();
            publish_waypoints_info();
        }
    }

    void update_direction(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!active_waypoints_.empty()) {
            active_waypoints_.front().direction_to_next_waypoint = msg->data;
            publish_waypoints_info();
        }
    }

    double calculate_distance(double lat1, double lon1, double lat2, double lon2)
    {
        // Haversine formula to calculate the distance between two GPS points
        double R = 6371000; // Earth radius in meters
        double phi1 = lat1 * M_PI / 180.0;
        double phi2 = lat2 * M_PI / 180.0;
        double delta_phi = (lat2 - lat1) * M_PI / 180.0;
        double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

        double a = std::sin(delta_phi / 2) * std::sin(delta_phi / 2) +
                std::cos(phi1) * std::cos(phi2) *
                std::sin(delta_lambda / 2) * std::sin(delta_lambda / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        return R * c; // Distance in meters
    }

    double calculate_direction(double lat1, double lon1, double lat2, double lon2)
    {
        double y = std::sin(lon2 - lon1) * std::cos(lat2);
        double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(lon2 - lon1);
        return std::atan2(y, x) * 180.0 / M_PI; // Direction in degrees
    }

    void update_waypoint_status()
    {
        bool waypoint_in_progress = false;

        // Loop through all waypoints to update their status
        for (auto& waypoint : active_waypoints_) {
            // If the waypoint is already done, do not change its status
            if (waypoint.status == "Done") {
                continue;
            }

            // Check distance to see if the waypoint should be marked as done
            if (waypoint.distance_to_next_waypoint < 1.0) {
                waypoint.status = "Done";
                RCLCPP_INFO(this->get_logger(), "Waypoint %d Done.", waypoint.index);
            }
            // If no waypoint is in progress, set the first unfinished one to "Target"
            else if (!waypoint_in_progress) {
                waypoint.status = "Target";
                publish_current_waypoint(waypoint.coordinates);
                waypoint_in_progress = true;  // A waypoint is now in progress
            }
            // All other waypoints remain in waiting status
            else {
                waypoint.status = "Waiting";
            }
        }
    }

    void publish_current_waypoint(const geometry_msgs::msg::Point& current_waypoint)
    {
        current_waypoint_publisher_->publish(current_waypoint);
        RCLCPP_INFO(this->get_logger(), "Published current waypoint: [%f, %f, %f]", current_waypoint.x, current_waypoint.y, current_waypoint.z);
    }

    void publish_waypoints_info()
    {
        for (const auto& waypoint : active_waypoints_) {
            auto waypoint_info_msg = navigation_package::msg::WaypointInfo();
            waypoint_info_msg.index = waypoint.index;
            waypoint_info_msg.coordinates = waypoint.coordinates;

            if (waypoint.distance_to_next_waypoint < 0) {
                RCLCPP_WARN(this->get_logger(), "Negative distance %d", waypoint.index);
                waypoint_info_msg.distance_to_next_waypoint = 0.0;
            } else {
                waypoint_info_msg.distance_to_next_waypoint = waypoint.distance_to_next_waypoint;
            }

            if (std::isnan(waypoint.direction_to_next_waypoint)) {
                RCLCPP_WARN(this->get_logger(), "Invalid direction %d", waypoint.index);
                waypoint_info_msg.direction_to_next_waypoint = 0.0;
            } else {
                waypoint_info_msg.direction_to_next_waypoint = waypoint.direction_to_next_waypoint;
            }

            waypoint_info_msg.status = waypoint.status;

            waypoint_info_publisher_->publish(waypoint_info_msg);
        }

        auto remaining_msg = std_msgs::msg::Int32();
        remaining_msg.data = active_waypoints_.size();
        waypoints_remaining_publisher_->publish(remaining_msg);
    }

    void add_sample_waypoint()
    {
        geometry_msgs::msg::Point waypoint;
        waypoint.x = 48.858844;
        waypoint.y = 2.294351;
        waypoint.z = 0.0;

        WaypointInfo sample_waypoint;
        sample_waypoint.index = 0;
        sample_waypoint.coordinates = waypoint;
        sample_waypoint.status = "Target";

        active_waypoints_.push_back(sample_waypoint);
        publish_waypoints_info();
        update_waypoint_status();
    }

    rclcpp::Service<navigation_package::srv::AddWaypoint>::SharedPtr add_waypoint_service_;
    rclcpp::Service<navigation_package::srv::RemoveWaypoint>::SharedPtr remove_waypoint_by_index_service_;
    rclcpp::Service<navigation_package::srv::GetAllWaypoints>::SharedPtr get_all_waypoints_service_;
    rclcpp::Publisher<navigation_package::msg::WaypointInfo>::SharedPtr waypoint_info_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoints_remaining_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr current_waypoint_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr direction_subscriber_;

    std::vector<WaypointInfo> active_waypoints_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointManagerNode>());
    rclcpp::shutdown();
    return 0;
}
