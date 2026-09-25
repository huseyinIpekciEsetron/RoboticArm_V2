/*
 * homing.h
 *
 *  Created on: Jun 17, 2025
 *      Author: Huseyin
 */

#ifndef INC_HOMING_H_
#define INC_HOMING_H_

#include "stm32f1xx.h"
#include "arm_state_manager.h"
#include "motor_state.h"
#include "arm_configuration.h"
#include <stdbool.h>

#define MAX_MOTOR1_CURRENT 600
#define MAX_MOTOR2_CURRENT 250
#define MAX_MOTOR3_CURRENT 400

enum motor_index
{
	MOTOR_1 =0,
	MOTOR_2 ,
	MOTOR_3,
	MOTOR_4
};
typedef enum
{
	HOMING_IDLE=0,
	HOMING_PROGRESS,
	HOMING_SUCCES,
	HOMING_FAULT
}homing_Status;

#define MOTOR_CURRENT(motorNum) (MotorState_Get(motorNum).current * HOMING_DIRECTIONS[motorNum] * JOINT_DIRECTIONS[motorNum])

void Homing_HallEffectDetecter(uint16_t GPIO_Pin);
void Homing_FirstStageHallEffectDetect();
void Homing_SecondStageHallEffectDetect();
bool Homing_FirstStageMotorCurrentLimitControl();
bool Homing_SecondStageMotorCurrentLimitControl();
homing_Status Homing_getHomingState();
void Homing_setHomingState(homing_Status homeState);
void Homing_update_angular_velocity(float current_angle_deg, float dt_s);
#endif /* INC_HOMING_H_ */
