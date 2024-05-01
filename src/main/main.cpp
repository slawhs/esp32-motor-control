#include "variables.hpp"

// ----- Run microros node -----
// ls /dev/serial/by-id/*                                                  -> get serial device name
// sudo chmod a+wr [serial device name]                                    -> give permissions to serial device
// ros2 run micro_ros_agent micro_ros_agent serial --dev [nombre del com]  -> start micro_ros agent and microros controller node

// ---- Send topic messages on terminal -----
// ros2 topic pub --once /wamv_caleuche/thrusters/turn_on_off std_msgs/msg/Int16 data:\ 1\    -> turn on the motor
// ros2 topic pub --once /wamv_caleuche/thrusters/turn_on_off std_msgs/msg/Int16 data:\ 0\    -> turn off the motor
// ros2 topic pub --once /wamv_caleuche/thrusters/right/cmd_vel std_msgs/msg/Int16 data:\ X\  -> set the motor speed to X (-1000 < X < 1000)


// String messages management based on:
// https://github.com/uhobeike/micro-ros-st-nucleo-f446re/blob/master/Core/Src/main.c#L92


CRC8 crc8;            // Initialize CRC8-Maxim calculation class
TorqeedoMotor motor;  // Initialize TorqeedoMotor class

const int input_vcc = 21; // Relay pin //! Por Implementar
//bool motor_status = false; // False = Motor apagado | True = Motor encendido //! Por Implementar
int16_t cmd_vel = 0; // Motor velocity command

// microROS handles
unsigned int num_handles = 2;   // 2 subscribers

void setup()
{
    // ---- MOTOR PCB SETUP ---- 
    pinMode(input_vcc, INPUT);
    motor.begin(2, 19, 18, 33);  // 1=ESPSerial, Tx=TX2, Rx=RX2, OnOff=33)
    delay(5000); 

    // ---- ESP SETUP ----
    pinMode(LED_BUILTIN, OUTPUT);    // Error LED
    digitalWrite(LED_BUILTIN, LOW);  // If there is an error, the LED will turn on
    delay(1000);
    
    int baud_rate = 115200;
    Serial2.begin(baud_rate);
    
    set_microros_serial_transports(Serial2);
    delay(2000);

    state = WAITING_AGENT;

    // microros_create_entities(); // Comment if using reconection manager
}

uint8_t received_buff[10];

// process a single byte received on serial port (In order to receive orders by serial protocol)
// return true if a complete message has been received (the message will be held in _received_buff)
// HEADING_CHAR $
// MOTOR_DESTINATION 1: LEFT, 2: RIGHT, 3: BOTH
// ORDER: +1000 :: -1000 (MUST INCLUDE NEGATIVE/POSITIVE SIGN)
// TERMINATION_CHAR ;


void loop()
{
    // ---- Microros Reconnection Manager ----
    // microros_loop();
    // ---- Executor ROS ----
    if (Serial2.available() > 0) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
        
        // ----- Publish feedback info on topics (comment if using reconnection manager) -----
        // // Publish error feedback
        // if (motor.feedback_getError() != nullptr){
        //     //creo el string con el error y lo mofifico
        //     const char * str = "Error: ";
        //     rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);
        //     ros_str = micro_ros_string_utilities_append(ros_str, motor.feedback_getError());
        //     feedback_error_msg.data.data = ros_str.data;          
        // }
        // else {
        //     //Creo el string con OK
        //     const char * str = "OK";
        //     rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);
        //     feedback_error_msg.data.data = ros_str.data;
        // }
        // RCSOFTCHECK(rcl_publish(&feedback_error_pub, &feedback_error_msg, NULL));

        // // Publish rpm feedback
        // feedback_rpm_msg.data = motor.feedback_getMotorRPM();
        // RCSOFTCHECK(rcl_publish(&feedback_rpm_pub, &feedback_rpm_msg, NULL));
    
        // // Publish battery charge feedback
        // feedback_battery_charge_msg.data = motor.feedback_getBatteryCharge();
        // RCSOFTCHECK(rcl_publish(&feedback_battery_charge_pub, &feedback_battery_charge_msg, NULL));
    }

    motor.loop(cmd_vel);
}


// ------------------------
// ------- MICROROS -------
// ------------------------

bool microros_create_entities(){
    
    // Node information
    const char *node_name = "thruster_controller_right";
    const char *node_ns = ""; //namespace
    
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
    RCCHECK(rclc_node_init_default(&node, node_name, node_ns, &support)); // create node
    
    // // ---- MICROROS PUBLISHERS ---- define as many publishers as you need
    // RCCHECK(rclc_publisher_init_default( // create publisher
    //     &feedback_error_pub,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), 
    //     "/wamv_caleuche/thrusters/right/feedback/error"));
      
    // // ---- MICROROS PUBLISHERS ---- define as many publishers as you need
    // RCCHECK(rclc_publisher_init_default( // create publisher
    //     &feedback_rpm_pub,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
    //     "/wamv_caleuche/thrusters/right/feedback/rpm"));
    
    // // ---- MICROROS PUBLISHERS ---- define as many publishers as you need
    // RCCHECK(rclc_publisher_init_default( // create publisher
    //     &feedback_battery_charge_pub,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
    //     "/wamv_caleuche/thrusters/right/feedback/battery_charge"));

    // ---- MICROROS SUBSCRIBERS ---- define as many subscribers as you need
    RCCHECK(rclc_subscription_init_default( // topic for turning on and off the thruster
        &turn_sub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        "/wamv_caleuche/thrusters/turn_on_off"));
    
    RCCHECK(rclc_subscription_init_default( // topic for giving velocity commands to the thruster
        &cmd_vel_sub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        "/wamv_caleuche/thrusters/right/cmd_vel"));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &turn_sub, &turn_msg, &sub_turn_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &sub_cmd_vel_callback, ON_NEW_DATA));

    return true;
}

// Destroys all microROS entities
void microros_destroy_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&feedback_error_pub, &node);
    rcl_publisher_fini(&feedback_rpm_pub, &node);
    rcl_publisher_fini(&feedback_battery_charge_pub, &node);
    rcl_subscription_fini(&turn_sub, &node);
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// Verifies and restablishes connection with microROS agent
void microros_loop(){
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 8)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == microros_create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        microros_destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 8)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        if (Serial2.available() > 0) {
          RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

          // ----- Publish feedback info on topics (comment if NOT using the reconnection manager) -----
          // // Publish error feedback
          // if (motor.feedback_getError() != nullptr){
          //     //creo el string con el error y lo mofifico
          //     const char * str = "Error: ";
          //     rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);
          //     ros_str = micro_ros_string_utilities_append(ros_str, motor.feedback_getError());
          //     feedback_error_msg.data.data = ros_str.data;          
          // }
          // else {
          //     //Creo el string con OK
          //     const char * str = "OK";
          //     rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);
          //     feedback_error_msg.data.data = ros_str.data;
          // }
          // RCSOFTCHECK(rcl_publish(&feedback_error_pub, &feedback_error_msg, NULL));

          // // Publish rpm feedback
          // feedback_rpm_msg.data = motor.feedback_getMotorRPM();
          // RCSOFTCHECK(rcl_publish(&feedback_rpm_pub, &feedback_rpm_msg, NULL));
      
          // // Publish battery charge feedback
          // feedback_battery_charge_msg.data = motor.feedback_getBatteryCharge();
          // RCSOFTCHECK(rcl_publish(&feedback_battery_charge_pub, &feedback_battery_charge_msg, NULL));
        } 
        
      }
      break;
      
    case AGENT_DISCONNECTED:
      microros_destroy_entities();
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}

// Runs every time a message is received on /wamv_caleuche/thrusters/turn_on_off
// Turns the motor on or off
void sub_turn_callback(const void * msgin){
    // Cast message pointer to expected type
    std_msgs__msg__Int16 * msg = (std_msgs__msg__Int16 *)msgin;
    if (msg->data == 1) {
        motor.On();
    }
    else if (msg->data == 0) {
        motor.Off();
    } 
}

// Runs every time a message is received on /wamv_caleuche/thrusters/right/cmd_vel
// Updates the motor velocity command
void sub_cmd_vel_callback(const void * msgin){
    // Cast message pointer to expected type
    std_msgs__msg__Int16 * msg = (std_msgs__msg__Int16 *)msgin;
    cmd_vel_msg.data = msg->data;
    cmd_vel = msg->data;
}

// ROS Error Handle Loop: If there is an error, the LED will turn on
void error_loop() {
    while(1) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
}
