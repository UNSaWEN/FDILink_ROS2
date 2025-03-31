# FDILink_ROS2

基于官方实现的 FDILink 驱动重构版本，针对 ROS2 Humble 进行优化。

## 概述

该项目提供了一个改进版的 FDILink 驱动，旨在解决性能问题并提高可靠性。代码结构和构建流程与原实现保持一致，确保集成的便捷性。

## 功能

- **基于官方驱动**：基于官方 FDILink 驱动代码重构。
- **一致的构建流程**：与原驱动相同的编译和使用流程。
- **ROS2 Humble 支持**：在 ROS2 Humble 上测试并验证。
- **IO 优化**：改进 IO 读取机制，避免串行读取导致的 IO 拥塞和数据读取不全。
- **TODO**：完成数据类型和内部消息结构的实现。

## 安装

### 依赖项

- ROS2 Humble (https://docs.ros.org/en/humble/Installation.html)
- CMake 3.10 或更高版本
- Python 3.10 或更高版本
- pyserial

### 从源代码构建

1. 克隆仓库：
   ```bash
   git clone https://github.com/UNSaWEN/FDILink_ROS2.git
   ```

2. 进入工作区：
   ```bash
   cd FDILink_ROS2
   ```

3. 构建包：
   ```bash
   colcon build --packages-select fdilink_ahrs
   ```

4. 源化设置文件：
   ```bash
   source install/setup.bash
   ```

## 使用方法

### 启动驱动

1. 启动驱动节点：
   ```bash
   ros2 launch fdilink_ahrs ahrs_driver.py
   ```

2. 验证连接：
   ```bash
   ros2 topic echo /gps/fix
   ```

## 改进点

- **IO 读取机制**：原驱动采用串行读取方式，导致 IO 拥塞和数据读取不全。本实现引入了并行读取机制以优化性能。
- **数据类型和消息结构**：官方驱动缺少数据类型和内部消息结构的完整定义。这些将在后续更新中完成（TODO）。

## 待完成工作

- [ ] 完成数据类型定义
- [ ] 实现缺失的消息结构
- [ ] 添加全面的错误处理
- [ ] 改进文档

## 贡献

欢迎贡献！请提交问题或拉取请求以提出改进建议。

## 许可证

本项目采用 MIT 许可证 - 详情请参阅 LICENSE 文件。
