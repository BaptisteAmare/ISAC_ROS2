import rclpy
from rclpy.node import Node
from azure.iot.device import IoTHubDeviceClient, Message
from std_msgs.msg import String
from geometry_msgs.msg import Point
from iothub_publisher.events.mission_created_event import MissionCreatedEvent
from iothub_publisher.events.mission_completed_event import MissionCompletedEvent
from iothub_publisher.events.waypoint_added_event import WaypointAddedEvent
from iothub_publisher.events.waypoint_status_edited_event import WaypointStatusEditedEvent
from iothub_publisher.events.waypoint_removed_event import WaypointRemovedEvent
from iothub_publisher.events.position_update_event import PositionUpdateEvent
from iothub_publisher.cloud_event import CloudEvent
from datetime import datetime
import json
import time
import uuid
import random



CONNECTION_STRING = "HostName=sargasolutions-iothub.azure-devices.net;DeviceId=Device1;SharedAccessKey=b9b4ivKzI8OaokB8KtqNggg4nl73QJRqzAIoTKEKpOw="

class IoTNode(Node):
    def __init__(self):
        super().__init__('iot_node')
        # Create an IoTHub client
        self.client = IoTHubDeviceClient.create_from_connection_string(CONNECTION_STRING)
        
        # Last processed time for position updates
        self.last_position_update_time = time.time()

        # Subscribe to topics for each event
        self.create_subscription(String, 'mission_created', self.mission_created_callback, 10)
        self.create_subscription(String, 'mission_completed', self.mission_completed_callback, 10)
        self.create_subscription(String, 'waypoint_added', self.waypoint_added_callback, 10)
        self.create_subscription(String, 'waypoint_status_edited', self.waypoint_status_edited_callback, 10)
        self.create_subscription(String, 'waypoint_removed', self.waypoint_removed_callback, 10)
        self.create_subscription(String, '/navigation/condensed_mission_info', self.position_updated_callback, 10)

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

    # Callback for waypoint_status_edited topic
    def waypoint_status_edited_callback(self, msg):
        try:
            # Parse the incoming JSON data
            data = json.loads(msg.data)
            
            # Create a WaypointAddedEvent object from the topic data
            waypoint_event = WaypointStatusEditedEvent(
                mission_id=data["mission_id"],
                edited_at=datetime.strptime(data["edited_at"], "%Y-%m-%dT%H:%M:%SZ"),
                waypoint_id=data["waypoint_id"],
                new_waypoint_info=data["new_waypoint_info"],
            )

            # Convert the object to a dictionary manually (if needed)
            message_dict = {
                "mission_id": waypoint_event.mission_id,
                "edited_at": waypoint_event.edited_at.isoformat(),
                "waypoint_id": waypoint_event.waypoint_id,
                "new_waypoint_info": waypoint_event.new_waypoint_info,
            }

            cloud_event = CloudEvent(event_type="waypoint_status_edited", source="/ros2/iot_node", data=message_dict)

            # Send the message as JSON
            message = Message(json.dumps(cloud_event.to_dict()))
            self.client.send_message(message)
            self.get_logger().info(f"WaypointStatusEditedEvent sent to IoT Hub: {message_dict}")
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

    # Callback for condensed mission info topic
    def position_updated_callback(self, msg):
        try:
            # Check if 20 seconds have passed since the last update
            current_time = time.time()
            if current_time - self.last_position_update_time < 20:
                return  # Skip processing if less than 20 seconds

            self.last_position_update_time = current_time

            # Parse the incoming JSON data
            data = json.loads(msg.data)
            mission_id = data.get("mission_id", "No mission")
            coordinates = data.get("coordinates", {})
            timestamp = data.get("timestamp", datetime.utcnow().isoformat() + "Z")

            x = coordinates.get("x", 0.0)
            y = coordinates.get("y", 0.0)
            z = coordinates.get("z", 0.0)

            # Generate additional fields
            heading = random.uniform(0, 360)  # Random heading
            speed = random.uniform(0, 120)   # Random speed

            # Build the message dictionary
            message_dict = {
                "mission_id": mission_id,
                "date": timestamp,
                "vehicule_info": {
                    "heading": heading,
                    "speed": speed,
                    "position": {
                        "x": x,
                        "y": y,
                        "z": z
                    }
                }
            }

            # Convert the dictionary to a CloudEvent
            cloud_event = CloudEvent(event_type="gps_position_update", source="/ros2/iot_node", data=message_dict)

            # Send the message as JSON
            message = Message(json.dumps(cloud_event.to_dict()))
            self.client.send_message(message)
            self.get_logger().info(f"Condensed mission info sent to IoT Hub: {message_dict}")
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
