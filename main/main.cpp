#include "servo.hpp"

extern "C" void app_main() {
    ServoMotor servo_motor;
    servo_motor.setup();
    servo_motor.rotateServo();
}
