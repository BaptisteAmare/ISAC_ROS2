#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
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
        mission_created_publisher_ = this->create_publisher<std_msgs::msg::String>("mission_created", 10);
        mission_completed_publisher_ = this->create_publisher<std_msgs::msg::String>("mission_completed", 10);
        waypoint_added_publisher_ = this->create_publisher<std_msgs::msg::String>("waypoint_added", 10);
        waypoint_removed_publisher_ = this->create_publisher<std_msgs::msg::String>("waypoint_removed", 10);

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

        // Vérifie si c'est le premier waypoint ajouté (aucun waypoint présent)
        bool is_first_waypoint = active_waypoints_.empty();

        if (!active_waypoints_.empty()) {
            WaypointInfo& previous_waypoint = active_waypoints_.back();
            new_waypoint.distance_to_next_waypoint = calculate_distance(
                previous_waypoint.coordinates.x, previous_waypoint.coordinates.y,
                new_waypoint.coordinates.x, new_waypoint.coordinates.y);
            new_waypoint.direction_to_next_waypoint = calculate_direction(
                previous_waypoint.coordinates.x, previous_waypoint.coordinates.y,
                new_waypoint.coordinates.x, new_waypoint.coordinates.y);
        } else {
            new_waypoint.distance_to_next_waypoint = 10.0;
            new_waypoint.direction_to_next_waypoint = 10.0;
        }

        active_waypoints_.push_back(new_waypoint);

        // Création du message JSON pour le waypoint ajouté
        nlohmann::json json_msg;
        json_msg["index"] = new_waypoint.index;
        json_msg["coordinates"]["x"] = new_waypoint.coordinates.x;
        json_msg["coordinates"]["y"] = new_waypoint.coordinates.y;
        json_msg["coordinates"]["z"] = new_waypoint.coordinates.z;
        json_msg["distance_to_next_waypoint"] = new_waypoint.distance_to_next_waypoint;
        json_msg["direction_to_next_waypoint"] = new_waypoint.direction_to_next_waypoint;
        json_msg["status"] = new_waypoint.status;

        // Publier le message JSON sur le topic "waypoint_added"
        std_msgs::msg::String json_msg_str;
        json_msg_str.data = json_msg.dump();
        waypoint_added_publisher_->publish(json_msg_str);
        RCLCPP_INFO(this->get_logger(), "Waypoint added event published");

        // Si c'est le premier waypoint, envoyer MissionCreatedEvent
        if (is_first_waypoint) {
            nlohmann::json mission_created_msg;
            mission_created_msg["mission_id"] = "<unique_mission_id>";  // Remplacer par l'ID de la mission réelle
            mission_created_msg["mission_name"] = "Test Mission";
            mission_created_msg["created_at"] = "<timestamp>";  // Générer la date actuelle
            mission_created_msg["first_waypoint"] = json_msg;

            std_msgs::msg::String mission_created_str;
            mission_created_str.data = mission_created_msg.dump();
            mission_created_publisher_->publish(mission_created_str);
            RCLCPP_INFO(this->get_logger(), "Mission created event published");
        }

        // Réponse du service
        response->success = true;
        response->index = new_waypoint.index;
        response->message = "Waypoint added";
        RCLCPP_INFO(this->get_logger(), "Waypoint added : [%f, %f, %f] with index %d", 
                    new_waypoint.coordinates.x, new_waypoint.coordinates.y, 
                    new_waypoint.coordinates.z, new_waypoint.index);

        update_waypoint_status();
        publish_waypoints_info();
    }


    void remove_waypoint_by_index_callback(const std::shared_ptr<navigation_package::srv::RemoveWaypoint::Request> request,
                                       std::shared_ptr<navigation_package::srv::RemoveWaypoint::Response> response)
    {
        if (request->index >= 0 && request->index < static_cast<int>(active_waypoints_.size())) {
            active_waypoints_.erase(active_waypoints_.begin() + request->index);
            
            // Création du message JSON pour suppression
            nlohmann::json json_msg;
            json_msg["index"] = request->index;
            json_msg["status"] = "Deleted";
            
            // Publier le message JSON sur le topic "waypoint_json"
            std_msgs::msg::String json_msg_str;
            json_msg_str.data = json_msg.dump();
            waypoint_removed_publisher_->publish(json_msg_str);
            RCLCPP_INFO(this->get_logger(), "Waypoint removed event published");

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
        bool mission_completed = true;

        // Boucle sur tous les waypoints pour mettre à jour leur statut
        for (auto& waypoint : active_waypoints_) {
            // Si le waypoint est déjà fait, ne changez pas son statut
            if (waypoint.status == "Done") {
                continue;
            }

            // Vérifie la distance pour voir si le waypoint doit être marqué comme fait
            if (waypoint.distance_to_next_waypoint < 5.0) {
                waypoint.status = "Done";
                RCLCPP_INFO(this->get_logger(), "Waypoint %d Done.", waypoint.index);
            }
            // Si aucun waypoint n'est en cours, définis le premier non terminé à "Target"
            else if (!waypoint_in_progress) {
                waypoint.status = "Target";
                publish_current_waypoint(waypoint.coordinates);
                waypoint_in_progress = true;  // Un waypoint est maintenant en cours
            }
            // Tous les autres waypoints restent en attente
            else {
                waypoint.status = "Waiting";
            }

            // Si au moins un waypoint n'est pas terminé, la mission n'est pas terminée
            if (waypoint.status != "Done") {
                mission_completed = false;
            }
        }

        // Si tous les waypoints sont terminés, publie l'événement "MissionCompleted" et retire les waypoints
        if (mission_completed) {
            nlohmann::json mission_completed_msg;
            mission_completed_msg["mission_id"] = "<unique_mission_id>";  // Remplacer par l'ID de la mission réelle
            mission_completed_msg["completed_at"] = "<timestamp>";  // Générer la date actuelle

            std_msgs::msg::String mission_completed_str;
            mission_completed_str.data = mission_completed_msg.dump();
            mission_completed_publisher_->publish(mission_completed_str);
            RCLCPP_INFO(this->get_logger(), "Mission completed event published");

            // Retirer tous les waypoints une fois la mission complétée
            active_waypoints_.clear();
            RCLCPP_INFO(this->get_logger(), "All waypoints cleared after mission completion.");
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

    rclcpp::Service<navigation_package::srv::AddWaypoint>::SharedPtr add_waypoint_service_;
    rclcpp::Service<navigation_package::srv::RemoveWaypoint>::SharedPtr remove_waypoint_by_index_service_;
    rclcpp::Service<navigation_package::srv::GetAllWaypoints>::SharedPtr get_all_waypoints_service_;
    rclcpp::Publisher<navigation_package::msg::WaypointInfo>::SharedPtr waypoint_info_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoints_remaining_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr current_waypoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_created_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_completed_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_added_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_removed_publisher_;

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
