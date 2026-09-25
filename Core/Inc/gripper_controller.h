/*
 * gripper_controller.h
 *
 *  Created on: Jun 16, 2025
 *      Author: Huseyin
 */

#ifndef INC_GRIPPER_CONTROLLER_H_
#define INC_GRIPPER_CONTROLLER_H_

#include "stm32f1xx.h"
#include <math.h>

#define MAX_GRIPPER_DUTY_US 1800
#define MIN_GRIPPER_DUTY_US 1000
#define MAX_SPEED_MULTIPLIER 0.9f
#define ADC_FILTER_DEPTH 10
#define GRIPPER_MAX_POSITION 3950
#define GRIPPER_MIN_POSITION 80
#define MAX_GRIPPER_INCREMENT 3.0f
#define Kp 5.1f
#define Ki 1.1f
#define Dt 0.0025f;
#define MAX_INTEGRAL 300
#define MIN_INTEGRAL -300
#define ADC_DEADBAND 25


void GripperController_Init(TIM_HandleTypeDef* htim);
uint8_t GripperLevel_GetSendGripperPosition();
void GripperLevel_SetGripperPosition(uint32_t *adcData);
void GripperController_Move(float speed);
void GripperLevel_Set(uint32_t *adcData);
uint32_t pwm_us_to_ccr(uint32_t pulse_us);

#endif /* INC_GRIPPER_CONTROLLER_H_ */
