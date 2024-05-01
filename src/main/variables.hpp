// add arduino libs
#include <Arduino.h>
#include <Tasker.h>
#include <ServoInput.h>
#include <stdio.h>

// add microros libs
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int16.h>

// add extra libs 
#include "crc8/crc8.h"
#include "torqeedo/torqeedo.h"

#define ARRAY_LEN 200

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t feedback_error_pub;
std_msgs__msg__String feedback_error_msg;

rcl_publisher_t feedback_rpm_pub;
std_msgs__msg__Int16 feedback_rpm_msg;

rcl_publisher_t feedback_battery_charge_pub;
std_msgs__msg__Int16 feedback_battery_charge_msg;

rcl_subscription_t turn_sub;
std_msgs__msg__Int16 turn_msg;

rcl_subscription_t cmd_vel_sub;
std_msgs__msg__Int16 cmd_vel_msg;

rcl_timer_t timer;
rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} //{error_loop();}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

bool micro_ros_init_successful;

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

void sub_turn_callback(const void * msgin);
void sub_cmd_vel_callback(const void * msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
bool microros_create_entities();
void microros_destroy_entities();
void microros_loop();
void error_loop();
