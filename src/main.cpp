// ========================================
// Includes
// ========================================
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>
#include <armDriver.hpp>
#include <params.hpp>

// ========================================
// Macros and Constants
// ========================================
#define LED_PIN 2 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// ========================================
// Global Variables - Micro-ROS
// ========================================

rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// Subscribers
rclc_executor_t executor_sub_arm;
rcl_subscription_t subscriber_arm;
rclc_executor_t executor_sub_crane;
rcl_subscription_t subscriber_crane;

// Message variables
trajectory_msgs__msg__JointTrajectoryPoint joint_angles;
float joint_angles_data[NUM_OF_SERVOS] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
std_msgs__msg__Int32MultiArray crane_control_msg;

// Crane control variables
float CraneState[NUM_OF_CRANE_MOTOR] = {0.0, 0.0};

// State variables
bool micro_ros_init_successful = false;
const char* jointAnglesTopic = "joint_angles";
const char* craneTopic = "crane_control";
states state;

// ========================================
// Function Declarations
// ========================================
void subscription_arm_callback(const void* msgin);
void subscription_crane_callback(const void* msgin);
bool create_entities();
void destroy_entities();
void microROSTaskFunction(void *parameter);
void armControlTaskFunction(void *parameter);
void craneControlTaskFunction(void *parameter);
void motor_init();
void DC_motor_execute(int state);
void Stepper_motor_execute(int state);

// ========================================
// Callback Functions
// ========================================

void subscription_arm_callback(const void* msgin)
{
  const trajectory_msgs__msg__JointTrajectoryPoint* msg = (const trajectory_msgs__msg__JointTrajectoryPoint*)msgin;
  
  for (int i = 0; i < NUM_OF_SERVOS; i++) {
    joint_angles_data[i] = msg->positions.data[i];
    Serial.printf("Joint Angle[%d]: %f\n", i, joint_angles_data[i]);
  }
}

void subscription_crane_callback(const void* msgin)
{
  const std_msgs__msg__Int32MultiArray* msg = (const std_msgs__msg__Int32MultiArray*)msgin;
  // Update crane control data based on received message
  // Serial.println("Received crane control data:");
  for (int i = 0; i < NUM_OF_CRANE_MOTOR && i < msg->data.size; i++) {
    CraneState[i] = msg->data.data[i];
    Serial.printf("Crane State[%d]: %f\n", i, CraneState[i]);
  }
}

// ========================================
// Micro-ROS Entity Management
// ========================================

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "crane_node", "", &support));

  // === create subscriber ===
  executor_sub_arm = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub_arm, &support.context, 1, &allocator));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_arm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
    jointAnglesTopic));
  
  // Add subscription to executor
  RCCHECK(rclc_executor_add_subscription(
    &executor_sub_arm,
    &subscriber_arm,
    &joint_angles,
    &subscription_arm_callback,
    ON_NEW_DATA));
  
  executor_sub_crane = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub_crane, &support.context, 1, &allocator));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_crane,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    craneTopic));

  // Add subscription to executor
  RCCHECK(rclc_executor_add_subscription(
    &executor_sub_crane,
    &subscriber_crane,
    &crane_control_msg,
    &subscription_crane_callback,
    ON_NEW_DATA));
  
  // Allocate memory for the message
  joint_angles.positions.data = (double*)malloc(NUM_OF_SERVOS * sizeof(double));
  joint_angles.positions.size = NUM_OF_SERVOS;
  joint_angles.positions.capacity = NUM_OF_SERVOS;

  // Allocate memory for crane_control_msg
  crane_control_msg.data.data = (int32_t*)malloc(NUM_OF_CRANE_MOTOR * sizeof(int32_t));
  crane_control_msg.data.size = NUM_OF_CRANE_MOTOR;
  crane_control_msg.data.capacity = NUM_OF_CRANE_MOTOR;

  // Check if memory allocation was successful
  if (joint_angles.positions.data == NULL)
  {
    Serial.println("Failed to allocate memory for joint angles.");
    return false;
  }
  if (crane_control_msg.data.data == NULL)
  {
    Serial.println("Failed to allocate memory for crane control message.");
    return false;
  }

  return true;
}

void destroy_entities()
{
  // Set session destroy timeout to 0
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // Free executor resources
  RCSOFTCHECK(rclc_executor_fini(&executor_sub_arm));
  RCSOFTCHECK(rclc_executor_fini(&executor_sub_crane));
  RCSOFTCHECK(rcl_subscription_fini(&subscriber_arm, &node));
  RCSOFTCHECK(rcl_subscription_fini(&subscriber_crane, &node));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  // Free allocated memory for message
  if (joint_angles.positions.data != NULL) {
    free(joint_angles.positions.data);
    joint_angles.positions.data = NULL;  // Prevent dangling pointer
  }
}

// ========================================
// Task Functions
// ========================================

void microROSTaskFunction(void *parameter) {
  while (true) {
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          destroy_entities();
        };
        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor_sub_arm, RCL_MS_TO_NS(100));
          rclc_executor_spin_some(&executor_sub_crane, RCL_MS_TO_NS(100));
        }
        break;
      case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        break;
      default:
        break;
    }

    if (state == AGENT_CONNECTED) {
      digitalWrite(LED_PIN, 1);
    } else {
      digitalWrite(LED_PIN, 0);
    } 
  }
}

void armControlTaskFunction(void *parameter) {
  ArmManager armManager(uint8_t(NUM_OF_SERVOS), 
                        jointMinAngles, jointMaxAngles, jointInitAngles);

  while (true) {
      for (size_t i = 0; i < NUM_OF_SERVOS; ++i) {
          armManager.setServoTargetAngle(i, uint8_t(joint_angles_data[i]));
      }
      armManager.moveArm();

      // Wait for some time before the next iteration
      vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
  }
}

void craneControlTaskFunction(void *parameter) {
    // Create an instance of CraneManager
    // CraneManager craneManager(NUM_OF_CRANE_MOTOR);

    Serial.println("craneControlTaskFunction start");

    for (;;) {
        // Control the crane using CraneManager
        for (uint8_t i = 0; i < NUM_OF_CRANE_MOTOR; i++) {
            // craneManager.setCraneState(i, CraneState[i]);
            if (i == 0) {
                DC_motor_execute(CraneState[i]);
            } else if (i == 1) {
                Stepper_motor_execute(CraneState[i]);
            }
        }

        // Wait for some time before the next iteration
        vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
    }
}

// ========================================
// Setup and Main Functions
// ========================================

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Micro-ROS on ESP32 with WiFi...");
  
  // Initialize WiFi transport for Micro-ROS
  set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);
  delay(2000);
  
  // Initialize hardware
  pinMode(LED_PIN, OUTPUT);
  motor_init();

  // Set initial state
  state = WAITING_AGENT;

  // Create tasks
  Serial.println("Starting tasks...");

  // Micro-ROS communication task
  xTaskCreate(
    microROSTaskFunction,    // Function to implement the task 
    "microROSTaskFunction",  // Name of the task
    8192,                    // Stack size in words
    NULL,                    // Task input parameter
    5,                       // Priority of the task
    NULL);                   // Task handle
  Serial.println("Micro-ROS Task started successfully.");
  delay(100);

  // Arm control task
  xTaskCreate(
    armControlTaskFunction, // Function to implement the task
    "armControlTaskFunction", // Name of the task
    4096,                   // Stack size in words
    NULL,                   // Task input parameter
    2,                      // Priority of the task
    NULL                    // Task handle.
  );
  Serial.println("Arm Control Task started successfully.");
  delay(100);

  // Crane control task
  xTaskCreate(
    craneControlTaskFunction, // Function to implement the task
    "craneControlTaskFunction", // Name of the task
    4096,                    // Stack size in words
    NULL,                    // Task input parameter
    2,                       // Priority of the task
    NULL                     // Task handle.
  );
  Serial.println("Crane Control Task started successfully.");
}

void loop() {
  // Empty loop
}

// ========================================
// Hardware Control Functions
// ========================================

void motor_init() {
    // Set the motor pins as output
    pinMode(PIN_MOTOR1, OUTPUT);
    pinMode(PIN_MOTOR2, OUTPUT);

    pinMode(PIN_PULSE, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENA, OUTPUT);

    for (uint8_t i = 0; i < NUM_OF_CRANE_MOTOR; i++) {
        CraneState[i] = 0;
    }
}

void DC_motor_execute(int state) {
    // Execute the motor state
    switch (state) {
        case 0:
            // Stop the motor
            digitalWrite(PIN_MOTOR1, HIGH);
            digitalWrite(PIN_MOTOR2, LOW);
            break;
        case 1:
            // Move the motor up
            digitalWrite(PIN_MOTOR1, LOW);
            digitalWrite(PIN_MOTOR2, LOW);
            break;
        case -1:
            // Move the motor down
            digitalWrite(PIN_MOTOR1, HIGH);
            digitalWrite(PIN_MOTOR2, HIGH);
            break;
        default:
            break;
    }
    vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
}

void Stepper_motor_execute(int state) {
    // Execute the motor state
    switch (state) {
        case 0:
            // Stop the motor
            digitalWrite(PIN_ENA, HIGH);
            break;
        case 1:
            // Move the motor forward            
            digitalWrite(PIN_ENA, LOW);
            digitalWrite(PIN_DIR, HIGH);
            digitalWrite(PIN_PULSE, HIGH);
            delayMicroseconds(UPDATE_STEPPER_DELAY);
            digitalWrite(PIN_PULSE, LOW);
            delayMicroseconds(UPDATE_STEPPER_DELAY);         
            break;
        case -1:
            // Move the motor backward
            digitalWrite(PIN_ENA, LOW);
            digitalWrite(PIN_DIR, LOW);
            digitalWrite(PIN_PULSE, HIGH);
            delayMicroseconds(UPDATE_STEPPER_DELAY);            
            digitalWrite(PIN_PULSE, LOW);
            delayMicroseconds(UPDATE_STEPPER_DELAY);            
            break;
        default:
            break;
    }
}
