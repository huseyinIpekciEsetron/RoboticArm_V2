#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"  // for CAN_HandleTypeDef

typedef void (*OffsetUpdateCallback)(uint8_t id);

enum message_index
{
	COIL_TEMPERATURE = 0x200C,
	CURRENT = 0x6078,
	ERROR_CODE = 0x603F,
	PCB_TEMPERATURE = 0x200D,
	POSITION = 0x6064,
	POSITION_OFFSET = 0x2008,
  STATUS_WORD = 0x6041,
	VELOCITY = 0x606C,
};

// General Functions
void MotorControl_InitalizeCanProtocol(CAN_HandleTypeDef *hcan);
void MotorControl_ProcessCanMessage(void);
void MotorControl_WriteControlWord(uint32_t id, uint16_t word);
void MotorControl_ReadStatusWord(uint32_t id);
void MotorControl_SetMode(uint32_t id, uint8_t mode);
void MotorControl_ProcessMessageBuffer(uint8_t id);
void MotorControl_SetAllMotorsToSpeedMode(void);
void MotorControl_ActiveMechanicalBreak(uint32_t id);
void MotorControl_DeactiveMechanicalBreak(uint32_t id);

// Position Mode Functions
void MotorControl_GoToPosition(uint32_t id, int32_t position);
void MotorControl_ReadPosition(uint32_t id);
void MotorControl_SetMinimumLimitPosition(uint32_t id, int32_t minimum_limit);
void MotorControl_SetMaximumLimitPosition(uint32_t id, int32_t maximum_limit);

// Speed Mode Functions
void MotorControl_SetMotorVelocity(uint32_t id, int32_t velocity);
void MotorControl_ReadMotorVelocity(uint32_t id);

// Set Parameters
void MotorControl_SetBaudrate(uint32_t id, uint16_t baudrate);
void MotorControl_SetNodeId(uint32_t id, uint8_t nodeID);
void MotorControl_SetProfileVelocity(uint32_t id, uint32_t profileVelocity);
void MotorControl_SetProfileAcceleration(uint32_t id, uint32_t acceleration_value);
void MotorControl_SetProfileDeceleration(uint32_t id, uint32_t deceleration_value);

// Get Parameters
void MotorControl_ReadCurrentValue(uint32_t id);
void MotorControl_ReadErrorCode(uint32_t id);
void MotorControl_ReadCoilTemperature(uint32_t id);
void MotorControl_ReadBoardTemperature(uint32_t id);

// Set Zero Point Functions
void MotorControl_ReadOffsetValue(uint32_t id);
void MotorControl_SetOffsetValue(uint32_t id, int32_t new_offset_value);
void MotorControl_SaveChanges(uint32_t id);

void MotorControl_StartOffsetUpdate(uint8_t id, OffsetUpdateCallback completed_callback);
void MotorControl_Update_OffsetUpdateContext(uint16_t motor_index, uint16_t message_index, int32_t *value);

void MotorControl_RequestMessage();

#endif
