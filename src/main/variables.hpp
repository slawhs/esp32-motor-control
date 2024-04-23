#include "variables.hpp"

// Initialize CRC8-Maxim calculation class
CRC8 crc8;
// Initialize TorqeedoMotor class
TorqeedoMotor motor;

const int input_vcc = 21; // Pin Relé //! Por Implementar
//bool motor_status = false; // False = Motor apagado | True = Motor encendido //! Por Implementar
int16_t cmd_vel = 0; // Velocidad de los motores

// ROS handles
unsigned int num_handles = 3;   // 1 subscriber, 2 publisher //? +1 publisher auxiliar

void setup()
{
    // ---- SETUP PINES MOTOR ---- 
    pinMode(input_vcc, INPUT);
    motor.begin(1, 17, 16, 33);  // 1=SerialESP/2=SoftwareSerial, Tx=TX2, Rx=RX2, OnOff 33)
    delay(5000); 

    // ---- SETUP MICROROS ----
    pinMode(LED_BUILTIN, OUTPUT);    // LED de funcionamiento
    digitalWrite(LED_BUILTIN, LOW);  // Si se enciende, hay un error 
    delay(1000);

    // ---- SETUP ESP ----
    int baud_rate = 115200;
    Serial.begin(baud_rate);
    
    set_microros_serial_transports(Serial);
    delay(2000);

    state = WAITING_AGENT;
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
    microros_loop();
    // ---- Executor ROS ----
    if (Serial.available() > 0) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
        feedback_msg.data = cmd_vel_msg.data;
        RCSOFTCHECK(rcl_publish(&feedback_pub, &feedback_msg, NULL));
    }
    motor.loop(cmd_vel);
}


// ------------------------
// ------- MICROROS -------
// ------------------------

bool microros_create_entities(){
    microros_setup();
    microros_add_pubs();
    microros_add_subs();
    microros_add_executor();

    return true;
}

void microros_destroy_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&feedback_pub, &node);
    rcl_subscription_fini(&turn_sub, &node);
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void microros_loop(){
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == microros_create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        microros_destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
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

// ----- MICROROS SETUP -----
void microros_setup() {
    // Configure serial transport
    const char *node_name = "motor_1_micro_ros";
    const char *node_ns = ""; //namespace
    
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
    RCCHECK(rclc_node_init_default(&node, node_name, node_ns, &support)); // create node
}

// ---- MICROROS PUB -----
void microros_add_pubs(){
    RCCHECK(rclc_publisher_init_default( // create publisher
        &feedback_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        "motor_feedback_info"));
}

// ---- MICROROS SUB -----
void microros_add_subs(){
    RCCHECK(rclc_subscription_init_default( // create subscriber
        &turn_sub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        "/motores/turn_on_off"));
    
    RCCHECK(rclc_subscription_init_default( // create subscriber
        &cmd_vel_sub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        "/motores/cmd_vel"));
}

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

void sub_cmd_vel_callback(const void * msgin){
    // Cast message pointer to expected type
    std_msgs__msg__Int16 * msg = (std_msgs__msg__Int16 *)msgin;
    cmd_vel_msg.data = msg->data;
    cmd_vel = msg->data;
}

// ---- MICROROS EXECUTOR -----
void microros_add_executor(){
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &turn_sub, &turn_msg, &sub_turn_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &sub_cmd_vel_callback, ON_NEW_DATA));
}

// ROS Error handle loop: el led parpadea si hay un error
void error_loop() {
    while(1) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

//CONEXIÓN CON ROSAGENT
// ls /dev/serial/by-id/*                                                  -> ver nombre del COM
// sudo chmod a+wr [nombre del COM]                                        -> dar permisos al com
// ros2 run micro_ros_agent micro_ros_agent serial --dev [nombre del com]  -> correr ros en el serial


// Manejo de mensaje string basado en:
// https://github.com/uhobeike/micro-ros-st-nucleo-f446re/blob/master/Core/Src/main.c#L92
