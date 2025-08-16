# ESP32 Crane Control System with Micro-ROS

A sophisticated robotic crane control system built on ESP32 using Micro-ROS for real-time communication. This project integrates a 7-DOF robotic arm with crane motor control capabilities, enabling precise manipulation and positioning tasks.

## 🚀 Features

- **Multi-DOF Robotic Arm Control**: 7-servo robotic arm with precise angle control
- **Crane Motor System**: Dual motor control (DC motor + Stepper motor)
- **Micro-ROS Integration**: Real-time ROS2 communication over WiFi
- **FreeRTOS Multitasking**: Concurrent task management for smooth operation
- **Hardware Abstraction**: Clean separation between hardware control and business logic
- **Real-time Feedback**: Live status monitoring and control

## 🛠️ Hardware Requirements

### Core Components
- **ESP32 Development Board** (ESP32-DOIT-DEVKIT-V1)
- **Adafruit PWM Servo Driver** (PCA9685)
- **7x Servo Motors** (for robotic arm)
- **DC Motor** (for crane vertical movement)
- **Stepper Motor** (for crane horizontal movement)
- **Motor Driver Circuits**

### Pin Configuration

#### I2C Communication (Servo Control)
- **SDA**: GPIO 21
- **SCL**: GPIO 22

#### DC Motor Control
- **Motor Pin 1**: GPIO 32
- **Motor Pin 2**: GPIO 33

#### Stepper Motor Control
- **Pulse Pin**: GPIO 27
- **Direction Pin**: GPIO 26
- **Enable Pin**: GPIO 25

#### Status Indicator
- **LED Pin**: GPIO 2

## 📦 Software Dependencies

### PlatformIO Libraries
```ini
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
    https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
```

### ROS2 Message Types
- `trajectory_msgs/msg/JointTrajectoryPoint` - Arm joint control
- `std_msgs/msg/Int32` - Crane motor control

## 🏗️ Project Structure

```
crane_microRos/
├── include/
│   ├── armDriver.hpp          # Robotic arm control class
│   └── params.hpp             # System configuration parameters
├── src/
│   ├── main.cpp              # Main application logic
│   └── armDriver.cpp         # Arm control implementation
├── lib/
├── test/
└── platformio.ini            # Build configuration
```

## ⚙️ Configuration

### Network Settings
Edit `include/params.hpp` to configure your network:

```cpp
// WiFi Credentials
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

// Micro-ROS Agent Configuration
const IPAddress AGENT_IP(192, 168, 1, 100);  // Replace with your agent IP
const uint16_t AGENT_PORT = 8888;
```

### Hardware Parameters
Adjust servo limits and motor settings in `params.hpp`:

```cpp
// Servo Configuration
const size_t NUM_OF_SERVOS = 7;
const uint8_t jointMinAngles[NUM_OF_SERVOS] = {0, 0, 0, 0, 0, 0, 0};
const uint8_t jointMaxAngles[NUM_OF_SERVOS] = {180, 180, 180, 180, 180, 180, 180};
const uint8_t jointInitAngles[NUM_OF_SERVOS] = {90, 90, 90, 90, 90, 90, 90};
```

## 🚀 Getting Started

### 1. Setup Development Environment

```bash
# Install PlatformIO CLI
pip install platformio

# Clone the repository
git clone https://github.com/screamlab/crane_esp32_rtos.git
cd crane_esp32_rtos
```

### 2. Configure Network Settings

1. Open `include/params.hpp`
2. Update WiFi credentials
3. Set your Micro-ROS agent IP address

### 3. Build and Upload

```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

### 4. Setup Micro-ROS Agent


You can start the Micro-ROS agent using Docker or from source. For convenience, this project provides a startup script:

### 5. Start Micro-ROS Agent with Script (Docker)

Use the provided script `script/start_micro_ros_agent.sh` to quickly launch the Micro-ROS agent in Docker.

#### Steps
1. Make sure Docker is installed on your system.
2. Set the ROS_DISTRO environment variable (e.g. `humble`, `galactic`, etc.):
   ```bash
   export ROS_DISTRO=humble
   ```
3. Run the startup script:
   ```bash
   bash script/start_micro_ros_agent.sh
   ```

#### Script Content
```bash
#!/bin/bash

docker run -it --rm \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  --net=host \
  microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
```

This will start the Micro-ROS agent and listen for UDP connections on port 8888.

**Notes:**
- Make sure the ESP32 firmware is configured with the correct agent IP and port.
- To change the port, edit the `--port` parameter in the script.
- If you encounter permission issues, check `/dev` permissions or run the script as root.
- For serial or other transport options, refer to the official Micro-ROS agent documentation.

## 📡 ROS2 Interface

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/joint_angles` | `trajectory_msgs/JointTrajectoryPoint` | Robotic arm joint positions |
| `/crane_control` | `std_msgs/Int32` | Crane motor control commands |

### Control Commands

#### Robotic Arm Control
```bash
# Publish joint angles (in degrees)
ros2 topic pub /joint_angles trajectory_msgs/msg/JointTrajectoryPoint "{
  positions: [90.0, 45.0, 135.0, 90.0, 90.0, 90.0, 90.0]
}"
```

#### Crane Control
```bash
# Motor control states: -1 (reverse), 0 (stop), 1 (forward)
ros2 topic pub /crane_control std_msgs/msg/Int32 "data: 1"  # Move forward
ros2 topic pub /crane_control std_msgs/msg/Int32 "data: 0"  # Stop
ros2 topic pub /crane_control std_msgs/msg/Int32 "data: -1" # Move reverse
```

## 🔧 System Architecture

### Task Management
The system uses FreeRTOS with three main tasks:

1. **Micro-ROS Task** (Priority 5, 8KB stack)
   - Manages ROS2 communication
   - Handles agent connection/disconnection
   - Processes incoming messages

2. **Arm Control Task** (Priority 2, 4KB stack)
   - Controls 7-DOF robotic arm
   - Smooth servo movement
   - Angle constraint enforcement

3. **Crane Control Task** (Priority 2, 4KB stack)
   - DC motor control (vertical movement)
   - Stepper motor control (horizontal movement)
   - Real-time motor state execution

### State Machine
```
WAITING_AGENT → AGENT_AVAILABLE → AGENT_CONNECTED
      ↑                              ↓
      ←──────── AGENT_DISCONNECTED ←──
```

## 🎛️ Hardware Control API

### Robotic Arm
```cpp
ArmManager armManager(NUM_OF_SERVOS, jointMinAngles, jointMaxAngles, jointInitAngles);
armManager.setServoTargetAngle(servo_index, angle);
armManager.moveArm();
```

### Motor Control
```cpp
DC_motor_execute(state);     // -1: down, 0: stop, 1: up
Stepper_motor_execute(state); // -1: backward, 0: stop, 1: forward
```

## 🔍 Monitoring and Debugging

### Serial Monitor Output
- Task startup confirmations
- Micro-ROS connection status
- Error messages and diagnostics

### Status LED
- **ON**: Connected to Micro-ROS agent
- **OFF**: Disconnected or waiting for agent

### Debug Commands
```bash
# Monitor serial output
pio device monitor --baud 115200

# Check ROS2 topics
ros2 topic list
ros2 topic echo /joint_angles
ros2 topic echo /crane_control
```

## 🛠️ Troubleshooting

### Common Issues

1. **WiFi Connection Failed**
   - Verify SSID and password in `params.hpp`
   - Check network accessibility

2. **Micro-ROS Agent Not Found**
   - Ensure agent is running on correct IP/port
   - Check firewall settings
   - Verify network connectivity

3. **Servo Not Moving**
   - Check I2C connections (SDA=21, SCL=22)
   - Verify servo power supply
   - Check servo angle limits

4. **Motor Control Issues**
   - Verify GPIO pin connections
   - Check motor driver power
   - Ensure proper grounding

### Build Issues
```bash
# Clean build
pio run --target clean

# Verbose build output
pio run -v
```

## 📈 Performance Characteristics

- **Servo Update Rate**: 100 Hz
- **Motor Control Rate**: 100 Hz  
- **ROS2 Communication**: 5 Hz (ping) / Real-time (messages)
- **Memory Usage**: ~12KB RAM for tasks

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [Micro-ROS](https://micro.ros.org/) for embedded ROS2 implementation
- [Adafruit](https://www.adafruit.com/) for PWM servo driver library
- [PlatformIO](https://platformio.org/) for development environment

## 📞 Support

For questions and support:
- Open an issue on GitHub
- Check the [Micro-ROS documentation](https://micro.ros.org/docs/)
- Refer to [ESP32 Arduino documentation](https://docs.espressif.com/projects/arduino-esp32/)

---
**Made with ❤️ by ScreamLab**
