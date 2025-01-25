#ifndef SERVO_MOTOR_HPP
#define SERVO_MOTOR_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

class ServoMotor {
public:
    ServoMotor(int gpio_num = 23);
    ~ServoMotor();

    void setup();
    void rotateServo();

private:
    uint32_t angleToCompare(int angle);
    
    int servo_gpio;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    
    static constexpr const char* TAG = "ServoMotor";
    
    static const int SERVO_MIN_PULSEWIDTH_US = 500;  
    static const int SERVO_MAX_PULSEWIDTH_US = 2500;  
    static const int SERVO_MIN_DEGREE = -90;  
    static const int SERVO_MAX_DEGREE = 90;
    
    static const int SERVO_TIMEBASE_RESOLUTION_HZ = 1000000;  
    static const int SERVO_TIMEBASE_PERIOD = 20000;  
};

#endif // SERVO_MOTOR_HPP
