#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"

class CondensedMissionInfoNode : public rclcpp::Node
{
public:
    CondensedMissionInfoNode() : Node("condensed_mission_info_node")
    {
        // Souscriptions
        gps_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/gps/current_position", 10, std::bind(&CondensedMissionInfoNode::gps_callback, this, std::placeholders::_1));
        mission_id_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/navigation/mission_id", 10, std::bind(&CondensedMissionInfoNode::mission_id_callback, this, std::placeholders::_1));

        // Publication
        condensed_info_publisher_ = this->create_publisher<std_msgs::msg::String>("/navigation/condensed_mission_info", 10);

        // Timer pour publication régulière
        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&CondensedMissionInfoNode::publish_condensed_info, this));
    }

private:
    void gps_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    void mission_id_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_mission_id_ = msg->data;
    }

    void publish_condensed_info()
    {
        // Vérifie si les données nécessaires sont disponibles
        if (!current_position_ || current_mission_id_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete data, not publishing condensed info.");
            return;
        }

        // Construire le message JSON
        nlohmann::json condensed_info;
        condensed_info["mission_id"] = current_mission_id_;
        condensed_info["coordinates"]["x"] = current_position_->x;
        condensed_info["coordinates"]["y"] = current_position_->y;
        condensed_info["coordinates"]["z"] = current_position_->z;

        // Ajouter un champ timestamp
        condensed_info["timestamp"] = get_current_time_iso8601();

        auto msg = std_msgs::msg::String();
        msg.data = condensed_info.dump();

        condensed_info_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published condensed info: %s", msg.data.c_str());
    }

    // Souscriptions
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr gps_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_id_subscription_;

    // Publication
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr condensed_info_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Données actuelles
    std::optional<geometry_msgs::msg::Point> current_position_;
    std::string current_mission_id_;

    std::string get_current_time_iso8601()
    {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&now_time), "%Y-%m-%dT%H:%M:%SZ");
        return ss.str();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CondensedMissionInfoNode>());
    rclcpp::shutdown();
    return 0;
}
