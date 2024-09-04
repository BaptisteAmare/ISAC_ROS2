#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "navigation_package/srv/get_all_waypoints.hpp"
#include "nlohmann/json.hpp"
#include <chrono> 

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

using namespace std::chrono_literals;

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

        current_position_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "gps/current_position", 10, std::bind(&MonitoringDisplayNode::current_position_callback, this, std::placeholders::_1));

        current_waypoint_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "navigation/current_waypoint", 10, std::bind(&MonitoringDisplayNode::current_waypoint_callback, this, std::placeholders::_1));

        // Setup WebSocket client
        setup_websocket_client();

        // Service to get all waypoints
        get_all_waypoints_client_ = this->create_client<navigation_package::srv::GetAllWaypoints>("get_all_waypoints");

        // Timer to periodically send consolidated data as a single JSON
        consolidated_data_timer_ = this->create_wall_timer(10s, std::bind(&MonitoringDisplayNode::send_consolidated_data, this));

        // Timer to periodically fetch and send all waypoints
        waypoints_data_timer_ = this->create_wall_timer(15s, std::bind(&MonitoringDisplayNode::send_all_waypoints, this));
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
    void setup_websocket_client()
    {
        if (ws_initialized_)
            return;

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

        ws_initialized_ = true;  // Flag to prevent multiple initializations
    }

    // Callbacks to update local data
    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        distance_to_waypoint_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f meters", distance_to_waypoint_);
    }

    void direction_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        direction_to_waypoint_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Direction to waypoint: %f degrees", direction_to_waypoint_);
    }

    void remaining_waypoints_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        remaining_waypoints_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Remaining waypoints: %d", remaining_waypoints_);
    }

    void current_position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        current_position_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current GPS position: [%f, %f, %f]", current_position_.x, current_position_.y, current_position_.z);
    }

    void current_waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        current_waypoint_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current Waypoint: [%f, %f, %f]", current_waypoint_.x, current_waypoint_.y, current_waypoint_.z);
    }

    // Function to send consolidated data as a single JSON
    void send_consolidated_data()
    {
        nlohmann::json consolidated_data;

        consolidated_data["distance_to_waypoint"] = distance_to_waypoint_;
        consolidated_data["direction_to_waypoint"] = direction_to_waypoint_;
        consolidated_data["remaining_waypoints"] = remaining_waypoints_;
        consolidated_data["current_position"] = {{"x", current_position_.x}, {"y", current_position_.y}, {"z", current_position_.z}};
        consolidated_data["current_waypoint"] = {{"x", current_waypoint_.x}, {"y", current_waypoint_.y}, {"z", current_waypoint_.z}};

        // Send the consolidated JSON data
        send_to_websocket_json(consolidated_data);
    }

    // Function to fetch and send all waypoints via WebSocket
    void send_all_waypoints()
    {
        auto request = std::make_shared<navigation_package::srv::GetAllWaypoints::Request>();
        if (get_all_waypoints_client_->wait_for_service(5s))
        {
            auto future = get_all_waypoints_client_->async_send_request(request);
            future.wait();
            if (future.valid())
            {
                auto response = future.get();
                nlohmann::json waypoints_json = nlohmann::json::parse(response->json_waypoints);
                send_to_websocket_json(waypoints_json);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get waypoints from service.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Waypoint service not available.");
        }
    }

    // Function to send a JSON message over WebSocket
    void send_to_websocket_json(const nlohmann::json &data)
    {
        if (!ws_initialized_) {
            RCLCPP_ERROR(this->get_logger(), "WebSocket is not initialized.");
            return;
        }

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
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr current_position_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr current_waypoint_subscriber_;

    websocket_client ws_client_;
    websocketpp::connection_hdl ws_hdl_;
    std::thread ws_thread_;

    // Flag to prevent multiple WebSocket connections
    bool ws_initialized_ = false;

    // Client to call GetAllWaypoints service
    rclcpp::Client<navigation_package::srv::GetAllWaypoints>::SharedPtr get_all_waypoints_client_;

    // Timer to send consolidated data periodically
    rclcpp::TimerBase::SharedPtr consolidated_data_timer_;

    // Timer to send all waypoints periodically
    rclcpp::TimerBase::SharedPtr waypoints_data_timer_;

    // Local storage for data from the callbacks
    float distance_to_waypoint_ = 0.0;
    float direction_to_waypoint_ = 0.0;
    int remaining_waypoints_ = 0;
    geometry_msgs::msg::Point current_position_;
    geometry_msgs::msg::Point current_waypoint_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonitoringDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
