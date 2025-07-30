#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include <std_msgs/msg/int32.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>


#include <armDriver.hpp>
#include <params.hpp>

#define LED_PIN 2 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// subscriber
rclc_executor_t executor_sub;
rcl_subscription_t subscriber;




trajectory_msgs__msg__JointTrajectoryPoint joint_angles;
float joint_angles_data[NUM_OF_SERVOS] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

bool micro_ros_init_successful = false;

const char* jointAnglesTopic = "joint_angles";
states state;

void subscription_callback(const void* msgin)
{
  const trajectory_msgs__msg__JointTrajectoryPoint* msg = (const trajectory_msgs__msg__JointTrajectoryPoint*)msgin;
  
  for (int i = 0; i < NUM_OF_SERVOS; i++) {
    joint_angles_data[i] = msg->positions.data[i];
  }
}
// void connectToWiFi() {
//   WiFi.begin(ssid, pass);
//   Serial.print("Connecting to WiFi");
  
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(1000);
//     Serial.print(".");
//   }
  
//   Serial.println();
//   Serial.print("Connected to WiFi network with IP Address: ");
//   Serial.println(WiFi.localIP());
// }

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "crane_node", "", &support));

  // === create subscriber ===
  executor_sub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
    jointAnglesTopic));
  
  // Add subscription to executor
  RCCHECK(rclc_executor_add_subscription(
    &executor_sub,
    &subscriber,
    &joint_angles,
    &subscription_callback,
    ON_NEW_DATA));
  
  // Allocate memory for the message
  joint_angles.positions.data = (double*)malloc(NUM_OF_SERVOS * sizeof(double));
  joint_angles.positions.size = NUM_OF_SERVOS;
  joint_angles.positions.capacity = NUM_OF_SERVOS;

  // Check if memory allocation was successful
  if (joint_angles.positions.data == NULL)
  {
    Serial.println("Failed to allocate memory for joint angles.");
    return false;
  }

  return true;
}

void destroy_entities()
{
  // Set session destroy timeout to 0
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // Free publisher resources
  RCSOFTCHECK(rclc_executor_fini(&executor_sub));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  // Free allocated memory for message
  if (joint_angles.positions.data != NULL) {
    free(joint_angles.positions.data);
    joint_angles.positions.data = NULL;  // Prevent dangling pointer
  }
}

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
          
          rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100));
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

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Micro-ROS on ESP32 with WiFi...");
  
  set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);
  delay(2000);
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;

  Serial.println("Starting Micro-ROS Task...");

  xTaskCreate(
    microROSTaskFunction,    // Function to implement the task 
    "microROSTaskFunction",  // Name of the task
    8192,                    // Stack size in words
    NULL,                    // Task input parameter
    5,                       // Priority of the task
    NULL);                   // Task handle
  Serial.println("Micro-ROS Task started successfully.");
  delay(100);

  xTaskCreate(
    armControlTaskFunction, // Function to implement the task
    "armControlTaskFunction", // Name of the task
    4096,                   // Stack size in words
    NULL,                   // Task input parameter
    2,                      // Priority of the task
    NULL                    // Task handle.
  );
}

void loop() {
  // Empty loop
}