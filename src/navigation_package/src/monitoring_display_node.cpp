#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include "nlohmann/json.hpp"  // Correct include for nlohmann::json

typedef websocketpp::client<websocketpp::config::asio_client> websocket_client;

class MonitoringDisplayNode : public rclcpp::Node
{
public:
    MonitoringDisplayNode() : Node("monitoring_display_node")
    {
        // Initialize subscriptions
        distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "navigation/distance_to_waypoint", 10, std::bind(&MonitoringDisplayNode::distance_callback, this, std::placeholders::_1));

        direction_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "navigation/navigation_direction", 10, std::bind(&MonitoringDisplayNode::direction_callback, this, std::placeholders::_1));

        remaining_waypoints_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "navigation/remaining_waypoints", 10, std::bind(&MonitoringDisplayNode::remaining_waypoints_callback, this, std::placeholders::_1));

        // Setup WebSocket client
        ws_client_.init_asio();
        ws_client_.set_open_handler([this](websocketpp::connection_hdl hdl) {
            ws_hdl_ = hdl;
            RCLCPP_INFO(this->get_logger(), "WebSocket connection opened.");
        });

        ws_client_.set_fail_handler([this](websocketpp::connection_hdl) {
            RCLCPP_ERROR(this->get_logger(), "WebSocket connection failed.");
        });

        ws_client_.set_close_handler([this](websocketpp::connection_hdl) {
            RCLCPP_INFO(this->get_logger(), "WebSocket connection closed.");
        });

        websocketpp::lib::error_code ec;
        websocket_client::connection_ptr con = ws_client_.get_connection("ws://localhost:9002", ec);
        if (ec)
        {
            RCLCPP_ERROR(this->get_logger(), "WebSocket connection initialization failed: %s", ec.message().c_str());
            return;
        }
        ws_client_.connect(con);
        ws_thread_ = std::thread([this]() { ws_client_.run(); });
    }

    ~MonitoringDisplayNode()
    {
        ws_client_.stop_perpetual();
        if (ws_thread_.joinable())
        {
            ws_thread_.join();
        }
    }

private:
    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f meters", msg->data);
        send_to_websocket("distance", msg->data);
    }

    void direction_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Direction to waypoint: %f degrees", msg->data);
        send_to_websocket("direction", msg->data);
    }

    void remaining_waypoints_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Remaining waypoints: %d", msg->data);
        send_to_websocket("remaining_waypoints", static_cast<float>(msg->data));
    }

    void send_to_websocket(const std::string &key, float value)
    {
        nlohmann::json data;
        data[key] = value;
        std::string message = data.dump();  // Convert JSON object to string

        websocketpp::lib::error_code ec;
        ws_client_.send(ws_hdl_, message, websocketpp::frame::opcode::text, ec);
        if (ec)
        {
            RCLCPP_ERROR(this->get_logger(), "WebSocket send failed: %s", ec.message().c_str());
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr direction_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr remaining_waypoints_subscriber_;

    websocket_client ws_client_;
    websocketpp::connection_hdl ws_hdl_;
    std::thread ws_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitoringDisplayNode>());
    rclcpp::shutdown();
    return 0;
}
