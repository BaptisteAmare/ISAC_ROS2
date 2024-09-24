import rclpy
from rclpy.node import Node
from azure.iot.device import IoTHubDeviceClient, Message
from std_msgs.msg import String

# Connection string to iotHub
CONNECTION_STRING = "HostName=sargasolutions-iothub.azure-devices.net;DeviceId=Device1;SharedAccessKey=b9b4ivKzI8OaokB8KtqNggg4nl73QJRqzAIoTKEKpOw="

class IoTNode(Node):
    def __init__(self):
        super().__init__('iot_node')
        # Create an iotHub client
        self.client = IoTHubDeviceClient.create_from_connection_string(CONNECTION_STRING)
        
        # subscribe to topic where waypoint information is sent 
        self.subscription = self.create_subscription(
            String,
            'waypoint_json',  # ros waypoint added topic
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            # Create and send json message to the iotHub
            message = Message(msg.data)
            self.client.send_message(message)
            self.get_logger().info(f"Json message sent to IoT Hub : {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error when sending a message : {str(e)}")

    def destroy(self):
        self.client.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = IoTNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
