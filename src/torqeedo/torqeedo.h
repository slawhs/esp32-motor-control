#pragma once
#ifndef TORQEEDO_H
#define TORQEEDO_H
#define TORQEEDO_MESSAGE_LEN_MAX 35 // messages are no more than 35 bytes

/*
 * Motor L:
 * TX -> 25
 * RX -> 26
 *
 * Motor R
 * TX -> 27
 * RX -> 15
 *
 *
 */

#include "Arduino.h"

class TorqeedoMotor
{
private:
    int _tx;
    int _rx;
    int _onoff;
    int _ser;
    bool _mosfet_status;
    int _order;
    int32_t _startup_timer;

    int32_t _starttime;
    int16_t _throttleOrder;

    // parameters

    enum class ParseState
    {
        WAITING_FOR_HEADER = 0,
        WAITING_FOR_FOOTER,
    };

    // message addresses
    enum class MsgAddress : uint8_t
    {
        BUS_MASTER = 0x00,
        REMOTE1 = 0x14,
        LCD = 0x20,
        MOTOR = 0x30,
        BATTERY = 0x80
    };

    // Remote specific message ids
    enum class RemoteMsgId : uint8_t
    {
        INFO = 0x00,
        REMOTE = 0x01,
        SETUP = 0x02
    };

    // Motor specific message ids
    enum class MotorMsgId : uint8_t
    {
        INFO = 0x00,
        STATUS = 0x01,
        PARAM = 0x03,
        CONFIG = 0x04,
        DRIVE = 0x82
    };

    // Display specific message ids
    enum class DisplayMsgId : uint8_t
    {
        INFO = 0x00,
        SYSTEM_STATE = 0x41,
        SYSTEM_SETUP = 0x42
    };

    // TYPE parameter values
    enum class ConnectionType : uint8_t
    {
        TYPE_DISABLED = 0,
        TYPE_TILLER = 1,
        TYPE_MOTOR = 2
    };

    ConnectionType _type; // connector type used (0:disabled, 1:tiller connector, 2: motor connector)
    uint8_t _motor_power; // motor power (0 ~ 100).  only applied when using motor connection
    float _slew_time;     // slew rate specified as the minimum number of seconds required to increase the throttle from 0 to 100%.  A value of zero disables the limit
    float _dir_delay;     // direction change delay.  output will remain at zero for this many seconds when transitioning between forward and backwards rotation

    // message parsing members
    ParseState _parse_state;                          // current state of parsing
    bool _parse_escape_received;                      // true if the escape character has been received so we must XOR the next byte
    uint32_t _parse_error_count;                      // total number of parsing errors (for reporting)
    uint32_t _parse_success_count;                    // number of messages successfully parsed (for reporting)
    uint8_t _received_buff[TORQEEDO_MESSAGE_LEN_MAX]; // characters received
    uint8_t _received_buff_len;                       // number of characters received
    uint32_t _last_received_ms;                       // system time (in millis) that a message was successfully parsed (for health reporting)

    // reply message handling
    uint8_t _reply_msgid;          // replies expected msgid (reply often does not specify the msgid so we must record it)
    uint32_t _reply_wait_start_ms; // system time that we started waiting for a reply message

    // members
    bool _initialised;            // true once driver has been initialised
    bool _send_motor_speed;       // true if motor speed should be sent at next opportunity
    int16_t _motor_speed_desired; // desired motor speed (set from within update method)
    uint32_t _last_send_motor_ms; // system time (in millis) last motor speed command was sent (used for health reporting)
    bool _motor_clear_error;      // true if the motor error should be cleared (sent in "Drive" message)
    uint32_t _send_start_us;      // system time (in micros) when last message started being sent (used for timing to unset DE pin)
    uint32_t _send_delay_us;      // delay (in micros) to allow bytes to be sent after which pin can be unset.  0 if not delaying

    // motor speed limit variables
    float _motor_speed_limited;       // limited desired motor speed. this value is actually sent to the motor
    uint32_t _motor_speed_limited_ms; // system time that _motor_speed_limited was last updated
    int8_t _dir_limit;                // acceptable directions for output to motor (+1 = positive OK, -1 = negative OK, 0 = either positive or negative OK)
    uint32_t _motor_speed_zero_ms;    // system time that _motor_speed_limited reached zero.  0 if currently not zero

    // Display system state flags
    typedef union PACKED
    {
        struct
        {
            uint8_t set_throttle_stop : 1;   // 0, warning that user must set throttle to stop before motor can run
            uint8_t setup_allowed : 1;       // 1, remote is allowed to enter setup mode
            uint8_t in_charge : 1;           // 2, master is in charging state
            uint8_t in_setup : 1;            // 3, master is in setup state
            uint8_t bank_available : 1;      // 4
            uint8_t no_menu : 1;             // 5
            uint8_t menu_off : 1;            // 6
            uint8_t reserved7 : 1;           // 7, unused
            uint8_t temp_warning : 1;        // 8, motor or battery temp warning
            uint8_t batt_charge_valid : 1;   // 9, battery charge valid
            uint8_t batt_nearly_empty : 1;   // 10, battery nearly empty
            uint8_t batt_charging : 1;       // 11, battery charging
            uint8_t gps_searching : 1;       // 12, gps searching for satellites
            uint8_t gps_speed_valid : 1;     // 13, gps speed is valid
            uint8_t range_miles_valid : 1;   // 14, range (in miles) is valid
            uint8_t range_minutes_valid : 1; // 15, range (in minutes) is valid
        };
        uint16_t value;
    } DisplaySystemStateFlags;

    // Display system state
    struct DisplaySystemState
    {
        DisplaySystemStateFlags flags; // flags, see above for individual bit definitions
        uint8_t master_state;          // deprecated
        uint8_t master_error_code;     // error code (0=no error)
        float motor_voltage;           // motor voltage in volts
        float motor_current;           // motor current in amps
        uint16_t motor_power;          // motor power in watts
        int16_t motor_rpm;             // motor speed in rpm
        uint8_t motor_pcb_temp;        // motor pcb temp in C
        uint8_t motor_stator_temp;     // motor stator temp in C
        uint8_t batt_charge_pct;       // battery state of charge (0 to 100%)
        float batt_voltage;            // battery voltage in volts
        float batt_current;            // battery current in amps
        uint16_t gps_speed;            // gps speed in knots * 100
        uint16_t range_miles;          // range in nautical miles * 10
        uint16_t range_minutes;        // range in minutes (at current speed and current draw)
        uint8_t temp_sw;               // master PCB temp in C (close to motor power switches)
        uint8_t temp_rp;               // master PCB temp in C (close to reverse voltage protection)
        uint32_t last_update_ms;       // system time that system state was last updated
    } _display_system_state;

    // Display system setup
    struct DisplaySystemSetup
    {
        uint8_t flags;              // 0 : battery config valid, all other bits unused
        uint8_t motor_type;         // motor type (0 or 3:Unknown, 1:Ultralight, 2:Cruise2, 4:Cruise4, 5:Travel503, 6:Travel1003, 7:Cruise10kW)
        uint16_t motor_sw_version;  // motor software version
        uint16_t batt_capacity;     // battery capacity in amp hours
        uint8_t batt_charge_pct;    // battery state of charge (0 to 100%)
        uint8_t batt_type;          // battery type (0:lead acid, 1:Lithium)
        uint16_t master_sw_version; // master software version
    } _display_system_setup;

    // Motor status
    struct MotorStatus
    {
        union
        {
            uint8_t status_flags_value;
            struct
            {
                uint8_t temp_limit_motor : 1;  // 0, motor speed limited due to motor temp
                uint8_t temp_limit_pcb : 1;    // 1, motor speed limited tue to PCB temp
                uint8_t emergency_stop : 1;    // 2, motor in emergency stop (must be cleared by master)
                uint8_t running : 1;           // 3, motor running
                uint8_t power_limit : 1;       // 4, motor power limited
                uint8_t low_voltage_limit : 1; // 5, motor speed limited because of low voltage
                uint8_t tilt : 1;              // 6, motor is tilted
                uint8_t reserved7 : 1;         // 7, unused (always zero)
            } status_flags;
        };

        union
        {
            uint16_t error_flags_value;
            struct
            {
                uint8_t overcurrent : 1;          // 0, motor stopped because of overcurrent
                uint8_t blocked : 1;              // 1, motor stopped because it is blocked
                uint8_t overvoltage_static : 1;   // 2, motor stopped because voltage too high
                uint8_t undervoltage_static : 1;  // 3, motor stopped because voltage too low
                uint8_t overvoltage_current : 1;  // 4, motor stopped because voltage spiked high
                uint8_t undervoltage_current : 1; // 5, motor stopped because voltage spiked low
                uint8_t overtemp_motor : 1;       // 6, motor stopped because stator temp too high
                uint8_t overtemp_pcb : 1;         // 7, motor stopped because pcb temp too high
                uint8_t timeout_rs485 : 1;        // 8, motor stopped because Drive message not received for too long
                uint8_t temp_sensor_error : 1;    // 9, motor temp sensor is defective (motor will not stop)
                uint8_t tilt : 1;                 // 10, motor stopped because it was tilted
                uint8_t unused11to15 : 5;         // 11 ~ 15 (always zero)
            } error_flags;
        };
    } _motor_status;
    uint32_t _last_send_motor_status_request_ms; // system time (in milliseconds) that last motor status request was sent

    // Motor params
    struct MotorParam
    {
        int16_t rpm;             // motor rpm
        uint16_t power;          // motor power consumption in Watts
        float voltage;           // motor voltage in volts
        float current;           // motor current in amps
        float pcb_temp;          // pcb temp in C
        float stator_temp;       // stator temp in C
        uint32_t last_update_ms; // system time that above values were updated
    } _motor_param;
    uint32_t _last_send_motor_param_request_ms; // system time (in milliseconds) that last motor param request was sent

public:
    TorqeedoMotor() {}
    void begin(uint8_t ser, uint8_t tx, uint8_t rx, uint8_t onoff);

    // consume incoming messages from motor, reply with latest motor speed
    void loop(int16_t throttleOrder);

    // Return motor status according if On or off was used.
    bool MosfetStatus();

    // Turn On Battery using OnOff pin
    void On();

    // Turn On Battery using OnOff pin
    void Off();

    // process a single byte received on serial port
    // return true if a complete message has been received (the message will be held in _received_buff)
    bool parse_byte(uint8_t b);

    // process message held in _received_buff
    void parse_message();

    // mark reply received. should be called whenever a message is received regardless of whether we are actually waiting for a reply
    void set_reply_received();

    // Calculate CRC8-Maxim
    uint8_t crc8_maxim(const uint8_t *data, uint16_t length);

    // report changes in error codes to user
    void report_error_codes();

    bool get_batt_capacity_Ah(uint16_t &amp_hours);

    bool get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining);

    // returns true if it is safe to send a message
    bool safe_to_send() const { return ((_send_delay_us == 0) && (_reply_wait_start_ms == 0)); }

    // send a motor speed command as a value from -1000 to +1000
    // value is taken directly from SRV_Channel
    void send_motor_speed_cmd();

    void send_tiller_init();

    // send a message to the motor with the specified message contents
    // msg_contents should not include the header, footer or CRC
    // returns true on success
    bool send_message(const uint8_t msg_contents[], uint8_t num_bytes);

    // add a byte to a message buffer including adding the escape character (0xAE) if necessary
    // this should only be used when adding the contents to the buffer, not the header and footer
    // num_bytes is updated to the next free byte
    bool add_byte_to_message(uint8_t byte_to_add, uint8_t msg_buff[], uint8_t msg_buff_size, uint8_t &num_bytes) const;

    // set pin to enable sending a message
    // void send_start();

    // record msgid of message to wait for and set timer for reply timeout handling
    void set_expected_reply_msgid(uint8_t msg_id);

    // check for timeout waiting for reply
    void check_for_reply_timeout();

    // check for timeout after sending a message and unset pin if required
    void check_for_send_end();

    int16_t getOrder();

    void debug(uint16_t buffer_size);

    // error reporting
    DisplaySystemStateFlags _display_system_state_flags_prev; // backup of display system state flags
    uint8_t _display_system_state_master_error_code_prev;     // backup of display system state master_error_code
    uint32_t _last_error_report_ms;                           // system time that flag changes were last reported (used to prevent spamming user)
    MotorStatus _motor_status_prev;                           // backup of motor status

    const char *map_master_error_code_to_string(uint8_t code);

    // calculate the limited motor speed that is sent to the motors
    // desired_motor_speed argument and returned value are in the range -1000 to 1000
    int16_t calc_motor_speed_limited(int16_t desired_motor_speed);
    int16_t get_motor_speed_limited() const { return (int16_t)_motor_speed_limited; }

    uint16_t UINT16_VALUE(uint8_t byteH, uint8_t byteL);

    uint32_t calc_send_delay_us(uint8_t num_bytes);

    //    void tx();
    //    void rx();
};

#endif