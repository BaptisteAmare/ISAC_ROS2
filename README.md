# ISAC ROS2 Humble Project

## Description

This project is based on ROS2 Humble and includes several packages.

## Prerequisites

Before you begin, make sure you have the following installed:

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [colcon](https://colcon.readthedocs.io/en/released/)

## Installation

Clone the repository to your local machine:

```bash
git clone <repository_url>
cd <repository_directory>
```

## Building the Project

To build the entire project, run the following command from the root of the repository:

```bash
colcon build
```

This will build all the packages in the repository.

### Building a Specific Package

If you want to build only a specific package, like `navigation_package`, use:

```bash
colcon build --packages-select navigation_package
```

## Running the Project

After building, you can run the ROS2 nodes using the `ros2 run` command. For example, to run a node from the `navigation_package`, use:

```bash
source install/setup.bash
ros2 run navigation_package <node_name>
```

Replace `<node_name>` with the name of the node you want to run.

## Contributing

Contributions are welcome! To contribute:

1. Fork this repository.
2. Create a new branch (`git checkout -b feature/your-feature`).
3. Commit your changes (`git commit -am 'Add new feature'`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a Pull Request.


## Authors

- **Baptiste AMARE** - *Main Developer*
