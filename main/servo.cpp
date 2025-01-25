#include "servo.hpp"

ServoMotor::ServoMotor(int gpio_num) : servo_gpio(gpio_num) {
    ESP_LOGI(TAG, "Creating ServoMotor instance");
}

ServoMotor::~ServoMotor() {
    ESP_LOGI(TAG, "Destroying ServoMotor instance");
}

void ServoMotor::setup() {
    ESP_LOGI(TAG, "Setting up timer and operator");

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,  
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .intr_priority = 0,                      
        .flags = { 0 }                           
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = 0, 
        .intr_priority = 0,                      
        .flags = { 0 }                           
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connecting timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Creating comparator and generator");
    
    // Initialize all fields of comparator_config struct
    mcpwm_comparator_config_t comparator_config = {
        .flags = {
            .update_cmp_on_tez = true,  // This is a bit-field
        },

    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    // Initialize all fields of generator_config struct
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = servo_gpio,
        .flags = { 0 },                 // Initialize flags if not used
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angleToCompare(0)));

    ESP_LOGI(TAG, "Setting generator action on timer and compare event");
    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_timer_event(
            generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP, 
                MCPWM_TIMER_EVENT_EMPTY, 
                MCPWM_GEN_ACTION_HIGH
            )
        )
    );
    
    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_compare_event(
            generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP, 
                comparator, 
                MCPWM_GEN_ACTION_LOW
            )
        )
    );

    ESP_LOGI(TAG, "Enabling and starting timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

uint32_t ServoMotor::angleToCompare(int angle) {
    return (angle - SERVO_MIN_DEGREE)
             * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) 
             / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) 
             + SERVO_MIN_PULSEWIDTH_US;
}

void ServoMotor::rotateServo() {
    int angle = 30;  // Start at the minimum angle
    int step = 2;     // Step size for rotation
    bool increasing = true;  

    while (1) {
        ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angleToCompare(angle)));
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for servo to reach the desired position

        if (increasing) {
            angle += 20+step;  // Move clockwise (increase angle)
            if (angle >= 60) {
                increasing = false;  // Reverse direction
            }
        } else {
            angle -= 20+step;  // Move counterclockwise (decrease angle)
            if (angle <= -60) {
                increasing = true;  // Reverse direction
            }
        }
    }
}

