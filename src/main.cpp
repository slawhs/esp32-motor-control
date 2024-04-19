#include <Arduino.h>
#include <Tasker.h>
#include <ServoInput.h>
#include "crc8.h"
#include "torqeedo.h"
#include "variables.hpp"

#define ARRAY_LEN 200

// // OTA Updater
// #include <WiFi.h>
// #include <WebServer.h>
// #include <ESP2SOTA.h>
// const char* ssid = "TIGO-5F09";
// const char* password = "4D9697503124";
// WebServer server(80);

// Initialise the TinyPICO library
// TinyPICO tp = TinyPICO();
// Initialize CRC8-Maxim calculation class
CRC8 crc8;
// Initialize TorqeedoMotor class
TorqeedoMotor motor;

// Throttle
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
volatile int16_t throttleOrder = 0;

const int input_vcc = 21; // Pin Relé
bool motor_status = false; // False = Motor apagado | True = Motor encendido //! Por Implementar

// ROS handles
unsigned int num_handles = 3;   // 1 subscriber, 1 publisher //? +1 publisher auxiliar

char str[ARRAY_LEN]; // Variable auxiliar para el manejo del mensaje recibido

rcl_publisher_t publisher2;
std_msgs__msg__Int16 throttleOrder_msg;

// Función para verificar si un string es un número
bool is_number(String msg_input){
    char* p;
    long converted = strtol(msg_input.c_str(), &p, 10);
    if (*p) { return false; }
    else { return true;}

}

void setup()
{

    // ---- SETUP ESP ----
    int baud_rate = 115200;
    Serial.begin(baud_rate);

    // ---- SETUP PINES MOTOR ---- 
    pinMode(input_vcc, INPUT);
    motor.begin(1, 17, 16, 33);  // 1=SerialESP/2=SoftwareSerial, Tx=TX2, Rx=RX2, OnOff 33)
    delay(5000);                 // Conectar cable TX con pin TX y cable RX con pin RX

    // ---- SETUP MICROROS ----
    pinMode(LED_BUILTIN, OUTPUT);    // LED de funcionamiento
    digitalWrite(LED_BUILTIN, LOW);  // Si se enciende, hay un error 
    delay(1000);

    microros_setup();
    microros_add_pubs();
    microros_add_subs();
    microros_add_timers();
    microros_add_executor();
    
    // ---- ENCENDIDO MOTOR ----
    // Serial.println("Turning on motor");
    motor.On();
    // Serial.println("End setup");
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
    // ---- Executor ROS ----
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    // ---------------------------------------------
    // ------- CONTROL DE MOTORES POR SERIAL -------
    // ---------------------------------------------
    // if (Serial.available())
    // {
    //     String msg = Serial.readStringUntil('\n');
    //     if (msg == "on" and motor_status == false) {
    //         motor.On();
    //         motor_status = true;
    //     }
    //     else if (msg == "off" and motor_status == true) {
    //         motor.Off();
    //         motor_status = false;
    //     }
    //     else {
    //         throttleOrder = msg.toInt();
    //     }
    // }

    // ----------------------------------------------
    // --------- CONTROL DE MOTORES POR ROS ---------
    // ----------------------------------------------
    String msg_software = sub_msg.data.data;
    if (msg_software != "") {
        if (msg_software == "on" and motor_status == false) {
            motor.On();
        }
        else if (msg_software == "off" and motor_status == true) {
            motor.Off();
        }
        if (is_number(msg_software)){
            throttleOrder = msg_software.toInt();
        }
    }
    motor.loop(throttleOrder);
}


// ------------------------
// ------- MICROROS -------
// ------------------------

// ----- MICROROS SETUP -----
void microros_setup() {
    // Configure serial transport
    const char *node_name = "motor_1_micro_ros";
    const char *node_ns = ""; //namespace
    
    set_microros_serial_transports(Serial);
    delay(2000);
    
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
    RCCHECK(rclc_node_init_default(&node, node_name, node_ns, &support)); // create node
}

// ---- MICROROS PUB -----
void microros_add_pubs(){
    RCCHECK(rclc_publisher_init_default( // create publisher
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "motor_feedback_info"));
    RCCHECK(rclc_publisher_init_default( // create publisher
        &publisher2,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "throttle_order"));
}

// ---- MICROROS SUB -----
void microros_add_subs(){
    RCCHECK(rclc_subscription_init_default( // create subscriber
        &subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), 
        "/motores/cmd_vel"));
}

void sub_callback(const void * msgin){
    // Cast message pointer to expected type
    std_msgs__msg__String * msg = (std_msgs__msg__String *)msgin;
    pub_msg = *msg;
    strcpy(str, msg->data.data);
}

// ---- MICROROS TIMERS -----
void microros_add_timers(){
    const unsigned int timer_timeout = 1000; // create timer
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Publisher para verificar mensaje recibido desde software
        sprintf(pub_msg.data.data, "Received: %s", str);
        pub_msg.data.size = strlen(pub_msg.data.data);
        RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));

        // Publisher para verificar mensaje enviado a los motores
        throttleOrder_msg.data = throttleOrder;
        RCSOFTCHECK(rcl_publish(&publisher2, &throttleOrder_msg, NULL));
    }
}

// ---- MICROROS EXECUTOR -----
void microros_add_executor(){
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &sub_callback, ON_NEW_DATA));

    // Initialize message memory
    pub_msg.data.data = (char *)malloc(ARRAY_LEN * sizeof(char));
    pub_msg.data.size = 0;
    pub_msg.data.capacity = ARRAY_LEN;

    sub_msg.data.data = (char *)malloc(ARRAY_LEN * sizeof(char));
    sub_msg.data.size = 0;
    sub_msg.data.capacity = ARRAY_LEN;
}

// ROS Error handle loop: el led parpadea si hay un error
void error_loop() {
    while(1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
    }
}

//CONEXIÓN CON ROSAGENT
// ls /dev/serial/by-id/*                                                  -> ver nombre del COM
// sudo chmod a+wr [nombre del COM]                                        -> dar permisos al com
// ros2 run micro_ros_agent micro_ros_agent serial --dev [nombre del com]  -> correr ros en el serial


// Manejo de mensaje string basado en:
// https://github.com/uhobeike/micro-ros-st-nucleo-f446re/blob/master/Core/Src/main.c#L92