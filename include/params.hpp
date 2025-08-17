// ========================================
// ESP32 Crane Control System Parameters
// ========================================
#pragma once

// ========================================
// System Includes
// ========================================
#include <stdint.h>
#include <IPAddress.h>

// ========================================
// Hardware Pin Definitions
// ========================================
// General GPIO Pins
#define ESP32_LED 2

// DC Motor Control Pins
#define PIN_MOTOR1 32  // DC motor control pin 1
#define PIN_MOTOR2 33  // DC motor control pin 2

// Stepper Motor Control Pins
#define PIN_PULSE 27   // Stepper motor pulse pin
#define PIN_DIR 26     // Stepper motor direction pin
#define PIN_ENA 25     // Stepper motor enable pin

// ========================================
// System Timing Constants
// ========================================
#define UPDATE_ARM_DELAY 100           // Arm update delay in seconds
#define UPDATE_STEPPER_DELAY 50.0       // Stepper motor delay in microseconds

// ========================================
// Robotic Arm Configuration
// ========================================
// Servo Configuration
const size_t NUM_OF_SERVOS = 7;
const float ARM_MOVEMENT_STEP = 1.0;    // Movement step size in degrees

// Joint Angle Limits (in degrees)
const uint8_t jointMinAngles[NUM_OF_SERVOS] = {0, 0, 0, 0, 0, 0, 0};
const uint8_t jointMaxAngles[NUM_OF_SERVOS] = {180, 180, 180, 180, 180, 180, 180};
const uint8_t jointInitAngles[NUM_OF_SERVOS] = {90, 90, 90, 90, 90, 90, 90};

// ========================================
// Crane Control Configuration
// ========================================
#define NUM_OF_CRANE_MOTOR 2

// Global crane state variables
extern float CraneState[NUM_OF_CRANE_MOTOR];

// ========================================
// Micro-ROS State Machine
// ========================================
enum states {
    WAITING_AGENT,      // Waiting for Micro-ROS agent connection
    AGENT_AVAILABLE,    // Agent detected, attempting to connect
    AGENT_CONNECTED,    // Successfully connected to agent
    AGENT_DISCONNECTED  // Lost connection to agent
};

// ========================================
// Network Configuration
// ========================================
// WiFi Credentials
// TODO: Replace with your actual WiFi credentials
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

// Micro-ROS Agent Configuration
// TODO: Replace with your actual agent IP address
const IPAddress AGENT_IP(192, 168, 1, 100);
const uint16_t AGENT_PORT = 8888;

// Legacy definitions for backward compatibility
#define ssid WIFI_SSID
#define pass WIFI_PASSWORD
const IPAddress agent_ip = AGENT_IP;
const uint16_t agent_port = AGENT_PORT;
