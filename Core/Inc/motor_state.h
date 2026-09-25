// motor_state.h
#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define MOTOR_STATE_COIL_TEMPERATURE      offsetof(MotorState, coil_temperature)
#define MOTOR_STATE_CURRENT               offsetof(MotorState, current)
#define MOTOR_STATE_ERROR_CODE            offsetof(MotorState, error_code)
#define MOTOR_STATE_HOME_POSITION         offsetof(MotorState, home_position)
#define MOTOR_STATE_PCB_TEMPERATURE       offsetof(MotorState, pcb_temperature)
#define MOTOR_STATE_POSITION              offsetof(MotorState, position)
#define MOTOR_STATE_POSITION_DEGREES	  offsetof(MotorState, position_degree)
#define MOTOR_STATE_POSITION_OFFSET       offsetof(MotorState, position_offset)
#define MOTOR_STATE_STATUS_WORD           offsetof(MotorState, status_word)
#define MOTOR_STATE_VELOCITY              offsetof(MotorState, velocity)

typedef union
{
	uint16_t whole;
	struct {
		uint16_t ready_to_switch_on    : 1;
		uint16_t switched_on           : 1;
		uint16_t operation_enabled     : 1;
		uint16_t fault                 : 1;
		uint16_t voltage_enabled       : 1;
		uint16_t quick_stop            : 1;
		uint16_t switch_on_disabled    : 1;
		uint16_t warning               : 1;
		uint16_t remote                : 1;
		uint16_t target_reached        : 1;
		uint16_t internal_limit_active : 1;
	} flag;
} StatusWord;

typedef union
{
	uint16_t whole;
	struct {
		uint16_t generic_error           		: 1; //represents software errors such as motor run write FLASH etc
		uint16_t over_voltage_error     		: 1;
		uint16_t under_voltage_error     		: 1;
		uint16_t startup_error       			: 1;
		uint16_t speed_feedback_error    		: 1;
		uint16_t overflow_error					: 1;
		uint16_t reserved                		: 7;
		uint16_t encoder_communication_error	: 1;
		uint16_t motor_temperature_error 		: 1;
		uint16_t board_temperature_error 		: 1;
	} flag;
} ErrorCode;

typedef struct
{
	int16_t coil_temperature;
	int16_t current;
	ErrorCode error_code;
	int32_t home_position;
	int16_t pcb_temperature;
	int32_t position;
	float position_degree;
	int32_t position_offset;
	StatusWord status_word;
	int32_t velocity;
} MotorState;

extern bool MotorState_MechanicalBreak[4];

MotorState MotorState_Get(uint8_t id);
void MotorState_Set(uint8_t id, size_t offset, const void* data, size_t size);
void MotorState_SetHomeToCurrentPosition(uint8_t id);

#endif
