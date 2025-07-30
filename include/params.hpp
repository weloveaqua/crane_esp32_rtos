#pragma once
#include <stdint.h>
#include <IPAddress.h>

#define ESP32_LED 2
#define UPDATE_ARM_DELAY 0.01
const float ARM_MOVEMENT_STEP = 1.0;
const size_t NUM_OF_SERVOS = 7;
const uint8_t jointMinAngles[NUM_OF_SERVOS] = {0, 0, 0, 0, 0, 0, 0};
const uint8_t jointMaxAngles[NUM_OF_SERVOS] = {180, 180, 180, 180, 180, 180, 180};
const uint8_t jointInitAngles[NUM_OF_SERVOS] = {90, 90, 90, 90, 90, 90, 90};

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

// Define the agent IP address and port for Micro-ROS
// These values should match the configuration of your Micro-ROS agent
// The agent IP address and port are used to establish the connection between the ESP32 and the agent
// Make sure to replace these values with your actual agent's IP address and port
// Example: IPAddress agent_ip(192, 168, 1, 100);
// Example: uint16_t agent_port = 8888;

const IPAddress agent_ip(192, 168, 1, 100);
const uint16_t agent_port = 8888;
#define ssid "your_wifi_ssid"
#define pass "your_wifi_password"
