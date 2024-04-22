// add arduino libs
#include <Arduino.h>
#include <Tasker.h>
#include <ServoInput.h>

// add microros libs
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int16.h>

// add extra libs 
#include "crc8/crc8.h"
#include "torqeedo/torqeedo.h"

#define ARRAY_LEN 200

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t feedback_pub;
std_msgs__msg__Int16 feedback_msg;

rcl_subscription_t turn_sub;
std_msgs__msg__Int16 turn_msg;

rcl_subscription_t cmd_vel_sub;
std_msgs__msg__Int16 cmd_vel_msg;

rcl_timer_t timer;

rclc_executor_t executor;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void microros_setup();
void microros_add_pubs();
void microros_add_subs();
void sub_turn_callback(const void * msgin);
void sub_cmd_vel_callback(const void * msgin);
void microros_add_executor();
void error_loop();
