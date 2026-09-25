// motor_configuration.h
#ifndef MOTOR_CONFIGURATION_H_
#define MOTOR_CONFIGURATION_H_

#include "stdint.h"
#include "arm_configuration.h"
#include "motor_state.h"
#include "arm_controller.h"
#include <math.h>

#define MOTOR_MAX_SPEED 2000.0f
#define MOTOR_MIN_SPEED -2000.0f
#define MAX_CARTESIAN_VELOCITY 150.0f
#define NUMBER_OF_MOTORS 4
#define ENCODER_TICKS_PER_REVOLUTION (65536 * 101) // Number of encoder ticks per revolution (MUST BE IN PARANTHESIS)
static const uint16_t MOTOR_CAN_IDS[NUMBER_OF_MOTORS] = {10, 20, 30, 40};

int MotorConfiguration_FindIndexOfMotor(int id);
float convertEncoderTicksToDegrees(int32_t position, uint8_t index);

#endif
