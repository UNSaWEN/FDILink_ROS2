# FDILink_ROS2

A restructured driver for FDILink based on the official implementation, optimized for ROS2 Humble.

## Overview

This project provides an improved version of the official FDILink driver, designed to address performance issues and enhance reliability. The code structure and build process remain consistent with the original implementation, ensuring ease of integration.

## Features

- **Based on Official Driver**: Built upon the original FDILink driver code.
- **Consistent Build Process**: Same compilation and usage workflow as the original driver.
- **ROS2 Humble Support**: Tested and verified on ROS2 Humble.
- **IO Optimization**: Improved IO reading mechanism to avoid congestion and incomplete data retrieval.
- **TODO**: Completion of data types and internal message structures.

## Installation

### Prerequisites

- ROS2 Humble (https://docs.ros.org/en/humble/Installation.html)
- CMake 3.10 or higher
- Python 3.10 or higher
- pyserial

### Building from Source

1. Clone the repository:
   ```bash
   git clone https://github.com/UNSaWEN/FDILink_ROS2.git
   ```

2. Navigate to the workspace:
   ```bash
   cd FDILink_ROS2
   ```

3. Build the package:
   ```bash
   colcon build --packages-select fdilink_ahrs
   ```

4. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage

### Running the Driver

1. Launch the driver node:
   ```bash
   ros2 launch fdilink_ahrs ahrs_driver.launch.py
   ```

2. Verify the connection:
   ```bash
   ros2 topic echo /gps/fix
   ```

## Improvements

- **IO Reading Mechanism**: The original driver used a serial reading approach, which caused IO congestion and incomplete data retrieval. This implementation introduces a parallel reading mechanism to optimize performance.
- **Data Type and Message Structures**: The official driver lacks complete definitions for data types and internal messages. These will be addressed in future updates (TODO).

## TODO List

- [ ] Complete data type definitions
- [ ] Implement missing message structures
- [ ] Add comprehensive error handling
- [ ] Improve documentation

## Contributing

Contributions are welcome! Please submit issues or pull requests with improvements.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
