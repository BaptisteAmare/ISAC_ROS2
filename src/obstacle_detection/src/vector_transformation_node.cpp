#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class VectorTransformationNode : public rclcpp::Node
{
public:
    VectorTransformationNode() : Node("vector_transformation_node")
    {
        // Subscriber to the coordinates topic
        coordinates_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "object_detection/coordinates", 10,
            std::bind(&VectorTransformationNode::transform_coordinates, this, std::placeholders::_1));

        // Publisher for the 2D vector topic
        vectors_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("object_detection/vectors", 10);
    }

private:
    void transform_coordinates(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Assuming msg->data contains [x1, y1, x2, y2, ..., xn, yn]
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            auto vector_msg = geometry_msgs::msg::Vector3();
            vector_msg.x = msg->data[i];
            vector_msg.y = msg->data[i + 1];
            vector_msg.z = 0.0; // Assuming 2D, so z is set to 0

            vectors_publisher_->publish(vector_msg);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr coordinates_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vectors_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorTransformationNode>());
    rclcpp::shutdown();
    return 0;
}
