#ifndef SIMULATOR_MANAGER_H_
#define SIMULATOR_MANAGER_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"  // for CAN_HandleTypeDef

#define COMMAND_RX_BUFFER_LENGTH 64

enum message_type
{
	VELOCITY_COMMAND = 0x01,
	HOMING_SENSOR_MESSAGE = 0x02,
	HOMING_START_COMMAND = 0x03,
	GRIPPER_POSITION_COMMAND = 0x04
};

enum control_type
{
	CONTROLTYPE_JOINT = 0x00,
	CONTROLTYPE_CARTESIAN = 0x01
};

void SimulatorManager_Parse(UART_HandleTypeDef *hcan);
void SimulatorManager_InitializePortAndTimer(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim2);
void SimulatorManager_SendPositions(void);
void SimulatorManager_SetJointVelocities(uint8_t *message);
void SimulatorManager_SetTipVelocity(uint8_t *message);
void SimulatorManager_SetGripperPosition(uint8_t *message);



#endif
