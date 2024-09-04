# Navigation Package - ROS 2 Humble

## Description

**`navigation_package`** is a ROS 2 package developed to manage autonomous navigation using waypoints. The system allows you to create, delete, and manage waypoints, calculate routes and distances, and integrate this data into a real-time monitoring interface via a WebSocket server.

The main components include a waypoint manager, a navigation calculator, a GPS simulator, and an interface for monitoring navigation data.

---

## Main Features

- Adding and removing waypoints.
- Calculating the distance and direction to waypoints.
- Managing waypoint status (`Waiting`, `Target`, `Done`).
- Publishing navigation information on ROS 2 topics.
- Integration with a WebSocket server to send real-time data to a monitoring interface.

---

## Package Structure

The package consists of several nodes:

1. **`waypoint_manager_node`**: Manages waypoints. Allows adding, deleting, and managing the list of waypoints. Publishes active waypoints, their distances, and their directions.
2. **`navigation_calculator_node`**: Calculates distances and directions between current GPS positions and waypoints.
3. **`gps_tracker_node`**: Simulates a GPS data stream by publishing positions on a ROS topic.
4. **`monitoring_display_node`**: Subscribes to navigation and position topics and sends the data via WebSocket for real-time monitoring.

---

## Prerequisites

Ensure the following components are installed before starting:

- **ROS 2 Humble**
- **WebSocket++** (included through the package dependencies)
- **nlohmann_json** (JSON library for C++)

### Installing Dependencies

To install the necessary libraries, use the following commands:

```bash
sudo apt update
sudo apt install ros-humble-rclcpp ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-std-srvs ros-humble-nlohmann-json3-dev
```

---

## Installation

1. Clone this package into your ROS 2 workspace:

```bash
git clone https://github.com/your-repo.git
```

2. Build the package:

```bash
colcon build
```

3. Source your ROS 2 environment:

```bash
source install/setup.bash
```

---

## Usage

### 1. Start the Navigation Package

Launch the various ROS 2 nodes to start the navigation and related services.

```bash
# Launch the waypoint manager
ros2 run navigation_package waypoint_manager_node

# Launch the navigation calculator
ros2 run navigation_package navigation_calculator_node

# Launch the GPS simulator
ros2 run navigation_package gps_tracker_node

# Launch the monitoring interface
ros2 run navigation_package monitoring_display_node
```

### 2. Interact with the System

You can interact with the system via ROS 2 services and topic publications.

#### Add a Waypoint

Use the `add_waypoint` service to add a waypoint with specific coordinates.

```bash
ros2 service call /add_waypoint navigation_package/srv/AddWaypoint "{point: {x: 44.0, y: 7.0, z: 0.0}}"
```

#### Remove a Waypoint

Use the `remove_waypoint_by_index` service to remove a waypoint by specifying its index.

```bash
ros2 service call /remove_waypoint_by_index navigation_package/srv/RemoveWaypoint "{index: 0}"
```

#### Retrieve All Waypoints

Use the `get_all_waypoints` service to get the current list of waypoints in JSON format.

```bash
ros2 service call /get_all_waypoints navigation_package/srv/GetAllWaypoints
```

---

## Topics

Here is a list of the main topics used in this package:

- **`/gps/current_position`** (type: `geometry_msgs/msg/Point`): Simulated current GPS position.
- **`/navigation/current_waypoint`** (type: `geometry_msgs/msg/Point`): Coordinates of the active waypoint.
- **`/navigation/distance_to_waypoint`** (type: `std_msgs/msg/Float32`): Distance to the active waypoint.
- **`/navigation/navigation_direction`** (type: `std_msgs/msg/Float32`): Direction to the active waypoint.
- **`/navigation/remaining_waypoints`** (type: `std_msgs/msg/Int32`): Number of remaining waypoints.

---

## Services

The package provides several services to interact with waypoints:

- **`/add_waypoint`**: Add a new waypoint.
- **`/remove_waypoint_by_index`**: Remove a waypoint by its index.
- **`/get_all_waypoints`**: Retrieve the list of all waypoints.

---