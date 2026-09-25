// arm_state_manager.h
#ifndef ARM_STATE_MANAGER_H
#define ARM_STATE_MANAGER_H
#include "stdint.h"
#include "motor_configuration.h"
#include "arm_configuration.h"
#include "motor_control.h"
#include "motor_state.h"
#include "homing.h"
#include "ika_comm.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
	ARMSTATE_IDLE = 0,
	ARMSTATE_FIRST_STAGE_HOMING_INITIALIZATION,
	ARMSTATE_WAIT_FOR_FIRST_STAGE_HOMING_CONFIRM,
	ARMSTATE_SECOND_STAGE_HOMING_INITIALIZATION,
	ARMSTATE_WAIT_FOR_SECOND_STAGE_HOMING_CONFIRM,
	ARMSTATE_RESETTING_OFFSETS,
	ARMSTATE_SAVING_HOME_POSITIONS,
	ARMSTATE_CENTERING_ARM,
	ARMSTATE_READY_POSITION,
	ARMSTATE_READY,
	ARMSTATE_CARTESIAN_CENTERING_ARM,
	ARMSTATE_PACKET_MODE_STEP_1,
	ARMSTATE_PACKET_MODE_STEP_2,
	ARMSTATE_DEMO_START_POSITION
} ArmState;

static const int8_t HOMING_DIRECTIONS[NUMBER_OF_MOTORS] = {1, 1, -1, 0}; // Directions for each motor during homing

ArmState ArmStateManager_GetCurrentState(void);
void ArmStateManager_SetCurrentState(ArmState armState);
void ArmStateManager_ClearHomedMotorMask();
void ArmStateManager_Initialize(void);
void ArmStateManager_Update(void);
void ArmStateManager_OnHomeSensorTriggered(uint8_t joint_index);
bool ArmStateManager_ArmReadyPosition(void);

#endif
