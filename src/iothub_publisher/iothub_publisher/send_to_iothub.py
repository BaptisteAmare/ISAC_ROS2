import rclpy
from rclpy.node import Node
from azure.iot.device import IoTHubDeviceClient, Message
from std_msgs.msg import String
from iothub_publisher.events.mission_created_event import MissionCreatedEvent
from iothub_publisher.events.mission_completed_event import MissionCompletedEvent
from iothub_publisher.events.waypoint_added_event import WaypointAddedEvent
from iothub_publisher.events.waypoint_removed_event import WaypointRemovedEvent
from iothub_publisher.cloud_event import CloudEvent
from datetime import datetime
import json



CONNECTION_STRING = "HostName=sargasolutions-iothub.azure-devices.net;DeviceId=Device1;SharedAccessKey=b9b4ivKzI8OaokB8KtqNggg4nl73QJRqzAIoTKEKpOw="

class IoTNode(Node):
    def __init__(self):
        super().__init__('iot_node')
        # Create an IoTHub client
        self.client = IoTHubDeviceClient.create_from_connection_string(CONNECTION_STRING)
        
        # Subscribe to topics for each event
        self.create_subscription(String, 'mission_created', self.mission_created_callback, 10)
        self.create_subscription(String, 'mission_completed', self.mission_completed_callback, 10)
        self.create_subscription(String, 'waypoint_added', self.waypoint_added_callback, 10)
        self.create_subscription(String, 'waypoint_removed', self.waypoint_removed_callback, 10)

    # Callback for mission_created topic
    def mission_created_callback(self, msg):
        try:
            # Parse the incoming JSON data (msg.data is already a string)
            data = json.loads(msg.data)
            
            # Create a MissionCreatedEvent object using the dictionary data
            mission_event = MissionCreatedEvent(
                mission_id=data["mission_id"],
                mission_name=data["mission_name"],
                created_at=datetime.strptime(data["created_at"], "%Y-%m-%dT%H:%M:%SZ"),
                first_waypoint=data["first_waypoint"]
            )

            # Convert the object to a dictionary manually (if needed)
            message_dict = {
                "mission_id": mission_event.mission_id,
                "mission_name": mission_event.mission_name,
                "created_at": mission_event.created_at.isoformat(),
                "first_waypoint": mission_event.first_waypoint
            }
            cloud_event = CloudEvent(event_type="mission_created", source="/ros2/iot_node", data=message_dict)

            # Send the message as JSON
            message = Message(json.dumps(cloud_event.to_dict()))
            self.client.send_message(message)
            self.get_logger().info(f"MissionCreatedEvent sent to IoT Hub: {message_dict}")
        except KeyError as e:
            self.get_logger().error(f"Error when sending a message: missing key {e}")
        except Exception as e:
            self.get_logger().error(f"Error when sending a message: {str(e)}")

    # Callback for mission_completed topic
    def mission_completed_callback(self, msg):
        try:
            # Parse the incoming JSON data
            data = json.loads(msg.data)
            
            # Create a MissionCompletedEvent object from the topic data
            mission_event = MissionCompletedEvent(
                mission_id=data["mission_id"],
                completed_at=datetime.strptime(data["completed_at"], "%Y-%m-%dT%H:%M:%SZ"),
            )

            # Convert the object to a dictionary manually (if needed)
            message_dict = {
                "mission_id": mission_event.mission_id,
                "completed_at": mission_event.completed_at.isoformat(),
            }
            cloud_event = CloudEvent(event_type="mission_completed", source="/ros2/iot_node", data=message_dict)

            # Send the message as JSON
            message = Message(json.dumps(cloud_event.to_dict()))
            self.client.send_message(message)
            self.get_logger().info(f"MissionCompletedEvent sent to IoT Hub: {message_dict}")
        except Exception as e:
            self.get_logger().error(f"Error when sending a message: {str(e)}")

    # Callback for waypoint_added topic
    def waypoint_added_callback(self, msg):
        try:
            # Parse the incoming JSON data
            data = json.loads(msg.data)
            
            # Create a WaypointAddedEvent object from the topic data
            waypoint_event = WaypointAddedEvent(
                mission_id=data["mission_id"],
                added_at=datetime.strptime(data["added_at"], "%Y-%m-%dT%H:%M:%SZ"),
                waypoint=data["waypoint_info"],
            )

            # Convert the object to a dictionary manually (if needed)
            message_dict = {
                "mission_id": waypoint_event.mission_id,
                "added_at": waypoint_event.added_at.isoformat(),
                "waypoint_info":waypoint_event.waypoint,
            }

            cloud_event = CloudEvent(event_type="waypoint_added", source="/ros2/iot_node", data=message_dict)

            # Send the message as JSON
            message = Message(json.dumps(cloud_event.to_dict()))
            self.client.send_message(message)
            self.get_logger().info(f"WaypointAddedEvent sent to IoT Hub: {message_dict}")
        except Exception as e:
            self.get_logger().error(f"Error when sending a message: {str(e)}")

    # Callback for waypoint_removed topic
    def waypoint_removed_callback(self, msg):
        try:
            # Parse the incoming JSON data
            data = json.loads(msg.data)
            
            # Create a WaypointRemovedEvent object from the topic data
            waypoint_event = WaypointRemovedEvent(
                mission_id=data["mission_id"],
                waypoint_id=data["waypoint_id"],
                removed_at=data["removed_at"],
            )
             # Convert the object to a dictionary manually (if needed)
            message_dict = {
                "mission_id": waypoint_event.mission_id,
                "waypoint_id": waypoint_event.waypoint_id,
                "removed_at":waypoint_event.removed_at,
            }
            cloud_event = CloudEvent(event_type="waypoint_removed", source="/ros2/iot_node", data=message_dict)

            # Send the message as JSON
            message = Message(json.dumps(cloud_event.to_dict()))
            self.client.send_message(message)
            self.get_logger().info(f"WaypointRemovedEvent sent to IoT Hub: {message_dict}")
        except Exception as e:
            self.get_logger().error(f"Error when sending a message: {str(e)}")

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
