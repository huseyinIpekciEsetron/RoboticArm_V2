// arm_state_manager.c
#include "arm_state_manager.h"

#define CENTERING_ARM_SPEED 250
#define PACKET_MODE_SPEED 250
#define WILL_OFFSET false
#define FIRST_MOTOR_BACKOFF_MOVEMENT 93
#define SECOND_MOTOR_BACKOFF_MOVEMENT 30
#define THIRD_MOTOR_BACKOFF_MOVEMENT 45

#define FIRST_MOTOR_PACKET_MODE 93
#define SECOND_MOTOR_PACKET_MODE 0
#define THIRD_MOTOR_PACKET_MODE 0

static const float HOMING_SPEEDS[NUMBER_OF_MOTORS] = {300.0f, 300.0f, 150.0f};
static const bool FIRST_STAGE_HOMING_REQUIRED_MOTORS_MASK[NUMBER_OF_MOTORS] = {false, true, true, false}; // Mask indicating which motors require homing
static const bool SECOND_STAGE_HOMING_REQUIRED_MOTORS_MASK[NUMBER_OF_MOTORS] = {true, false, false, false}; // Mask indicating which motors require homing

static ArmState current_state = ARMSTATE_IDLE;
static volatile bool home_position_confirmed = false;
bool homed_motor_mask[NUMBER_OF_MOTORS] = {false};
bool offset_updated_motor_mask[NUMBER_OF_MOTORS] = {false};

ArmState ArmStateManager_GetCurrentState(void) {
	return current_state;
}

void ArmStateManager_SetCurrentState(ArmState armState)
{
	current_state = armState;
}

void ArmStateManager_ClearHomedMotorMask()
{
	memset(homed_motor_mask, 0, NUMBER_OF_MOTORS);
}

static bool ArmStateManager_ArmMotorPositionCheck(float firstMotorPosition, float secondMotorPosition, float thirdMotorPosition)
{
	MotorState Arm_first_motor_state = MotorState_Get(0);
	MotorState Arm_second_motor_state = MotorState_Get(1);
	MotorState Arm_third_motor_state = MotorState_Get(2);

	if( ! (((Arm_first_motor_state.home_position - Arm_first_motor_state.position) * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0]) <= ((firstMotorPosition+3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)) )
		return false;
	if( ! (((Arm_second_motor_state.home_position - Arm_second_motor_state.position) * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1]) <= ((secondMotorPosition+3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)) )
		return false;
	if( ! (((Arm_third_motor_state.home_position - Arm_third_motor_state.position) * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2]) <= ((thirdMotorPosition+3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)) )
		return false;

	return true;
}

void ArmStateManager_OnHomeSensorTriggered(uint8_t joint_index) {
	if (joint_index < 0 || joint_index >= NUMBER_OF_MOTORS ) return; // Invalid motor ID
	homed_motor_mask[joint_index] = true;
}

static void ArmStateManager_OnOffsetCompleted(uint8_t motor_id) {
	uint8_t motor_index = MotorConfiguration_FindIndexOfMotor(motor_id);
	if (motor_index < 0) return; // Invalid motor ID
	offset_updated_motor_mask[motor_index] = true;
}

void ArmStateManager_Initialize(void) {
	current_state = ARMSTATE_FIRST_STAGE_HOMING_INITIALIZATION;
	home_position_confirmed = false;
}

void ArmStateManager_Update(void) {
	switch (current_state) {
		case ARMSTATE_FIRST_STAGE_HOMING_INITIALIZATION:
			for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
				if (FIRST_STAGE_HOMING_REQUIRED_MOTORS_MASK[i]) {
					MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[i], HOMING_DIRECTIONS[i] * JOINT_DIRECTIONS[i] * HOMING_SPEEDS[i]);
				}
			}
			current_state = ARMSTATE_WAIT_FOR_FIRST_STAGE_HOMING_CONFIRM;
			break;
		case ARMSTATE_WAIT_FOR_FIRST_STAGE_HOMING_CONFIRM:
			if(Homing_FirstStageMotorCurrentLimitControl()) {
				ArmController_StopAllJoints();
				current_state = ARMSTATE_IDLE;
				Homing_setHomingState(HOMING_FAULT);
			}
			Homing_FirstStageHallEffectDetect();
			for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
				if (homed_motor_mask[i]) {
					MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[i], 0); // Stop motor when home sensor is triggered
				}
			}
			for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
				if (FIRST_STAGE_HOMING_REQUIRED_MOTORS_MASK[i] && !homed_motor_mask[i]) {
					return; // Wait for all motors to be homed
				}
			}
			current_state = ARMSTATE_SECOND_STAGE_HOMING_INITIALIZATION;
			break;
		case ARMSTATE_SECOND_STAGE_HOMING_INITIALIZATION:
			for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
				if (SECOND_STAGE_HOMING_REQUIRED_MOTORS_MASK[i]) {
					MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[i], HOMING_DIRECTIONS[i] * JOINT_DIRECTIONS[i] * HOMING_SPEEDS[i]);
				}
			}
			current_state = ARMSTATE_WAIT_FOR_SECOND_STAGE_HOMING_CONFIRM;
			break;
		case ARMSTATE_WAIT_FOR_SECOND_STAGE_HOMING_CONFIRM:
			if(Homing_SecondStageMotorCurrentLimitControl()) {
				ArmController_StopAllJoints();
				current_state = ARMSTATE_IDLE;
				Homing_setHomingState(HOMING_FAULT);
			}
			Homing_SecondStageHallEffectDetect();
			for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
				if (homed_motor_mask[i]) {
					MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[i], 0); // Stop motor when home sensor is triggered
				}
			}
			for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
				if (SECOND_STAGE_HOMING_REQUIRED_MOTORS_MASK[i] && !homed_motor_mask[i]) {
					return; // Wait for all motors to be homed
				}
			}
			if (WILL_OFFSET) {
				current_state = ARMSTATE_RESETTING_OFFSETS;
			} else {
				current_state = ARMSTATE_SAVING_HOME_POSITIONS;
			}
			break;
		case ARMSTATE_RESETTING_OFFSETS:
			for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
				MotorControl_StartOffsetUpdate(MOTOR_CAN_IDS[i], ArmStateManager_OnOffsetCompleted);
			}
			for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
				if (FIRST_STAGE_HOMING_REQUIRED_MOTORS_MASK[i] && !offset_updated_motor_mask[i]) {
					return; // Wait for all motors to be homed
				}
			}
			current_state = ARMSTATE_SAVING_HOME_POSITIONS;
			break;
		case ARMSTATE_SAVING_HOME_POSITIONS:
			for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
				MotorState_SetHomeToCurrentPosition(i);
			}
			current_state = ARMSTATE_CENTERING_ARM;
			break;
		case ARMSTATE_CENTERING_ARM:
			MotorState first_motor_state = MotorState_Get(0);
			MotorState second_motor_state = MotorState_Get(1);
			MotorState third_motor_state = MotorState_Get(2);
			uint8_t backoff_movement_flag = 0;

			if (((first_motor_state.home_position - first_motor_state.position) * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0]) < ((FIRST_MOTOR_BACKOFF_MOVEMENT-3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f) )
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[0], (-1 * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0] * CENTERING_ARM_SPEED * 1)); // Center the first motor
			else {
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[0], 0); // Stop the first motor if centered
				backoff_movement_flag |= (0x01U) << 0U;
			}

			if (((second_motor_state.home_position - second_motor_state.position) * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1]) < ((SECOND_MOTOR_BACKOFF_MOVEMENT-3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f) )
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[1], (-1 * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1] * CENTERING_ARM_SPEED * 1)); // Center the first motor
			else {
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[1], 0); // Stop the first motor if centered
				backoff_movement_flag |= (0x01U) << 1U;
			}

			if ((third_motor_state.home_position - third_motor_state.position) * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] < (THIRD_MOTOR_BACKOFF_MOVEMENT-3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[2], (-1 * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] * CENTERING_ARM_SPEED * 1)); // Center the first motor
			else {
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[2], 0); // Stop the first motor if centered
				backoff_movement_flag |= (0x01U) << 2U;
			}

			if(backoff_movement_flag == 7)
			{
				for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
					homed_motor_mask[i] = false;
					offset_updated_motor_mask[i] = false;
				}
				current_state = ARMSTATE_READY;
			}
			break;
		case ARMSTATE_READY:
			Homing_setHomingState(HOMING_SUCCES);
			break;
		case ARMSTATE_READY_POSITION:
			if(ArmStateManager_ArmReadyPosition()) {
				current_state = ARMSTATE_READY;
				IkaComm_SetArmOperationMode(ARM_MODE_READY_POSITION);
			}
			break;
		case ARMSTATE_CARTESIAN_CENTERING_ARM:
			if(ArmStateManager_ArmReadyPosition()) {
				IkaComm_SetArmOperationMode(ARM_MODE_CARTESIAN_VELOCITY);
				current_state = ARMSTATE_READY;
			}
			break;
		case ARMSTATE_DEMO_START_POSITION:
			if(ArmStateManager_ArmReadyPosition()) {
				IkaComm_SetArmOperationMode(ARM_MODE_DEMO);
				current_state = ARMSTATE_READY;
			}
			break;
		case ARMSTATE_PACKET_MODE_STEP_1:
			if( ! ArmStateManager_ArmMotorPositionCheck(FIRST_MOTOR_PACKET_MODE, SECOND_MOTOR_PACKET_MODE, THIRD_MOTOR_PACKET_MODE) ){
			if(ArmStateManager_ArmReadyPosition()) {
					//current_state = ARMSTATE_PACKET_MODE_STEP_2;
					IkaComm_SetArmOperationMode(ARM_MODE_PACKET);
				}
			}
			else {
				current_state = ARMSTATE_READY;
				IkaComm_SetArmOperationMode(ARM_MODE_PACKET);
			}
			break;
		case ARMSTATE_PACKET_MODE_STEP_2:
			MotorState PacketMode_first_motor_state = MotorState_Get(0);
			MotorState PacketMode_second_motor_state = MotorState_Get(1);
			MotorState PacketMode_third_motor_state = MotorState_Get(2);
			uint8_t PacketMode_backoff_movement_flag = 0;

			if (((PacketMode_first_motor_state.home_position - PacketMode_first_motor_state.position) * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0]) < ((FIRST_MOTOR_PACKET_MODE-3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f) )
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[0], (-1 * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0] * PACKET_MODE_SPEED * 1)); // Center the first motor
			else if( (PacketMode_first_motor_state.home_position - PacketMode_first_motor_state.position) * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0] > (FIRST_MOTOR_PACKET_MODE+3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f )
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[0], (1 * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0] * PACKET_MODE_SPEED * 2)); // Center the first motor
			else {
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[0], 0); // Stop the first motor if centered
				PacketMode_backoff_movement_flag |= (0x01U) << 0U;
			}

			if (((PacketMode_second_motor_state.home_position - PacketMode_second_motor_state.position) * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1]) < ((SECOND_MOTOR_PACKET_MODE-3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f) )
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[1], (-1 * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1] * PACKET_MODE_SPEED * 1)); // Center the first motor
			else if((PacketMode_second_motor_state.home_position - PacketMode_second_motor_state.position) * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1] > (SECOND_MOTOR_PACKET_MODE+3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[1], (1 * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1] * PACKET_MODE_SPEED * 2));
			else {
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[1], 0); // Stop the first motor if centered
				PacketMode_backoff_movement_flag |= (0x01U) << 1U;
			}

			if ((PacketMode_third_motor_state.home_position - PacketMode_third_motor_state.position) * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] < (THIRD_MOTOR_PACKET_MODE-3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[2], (-1 * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] * PACKET_MODE_SPEED * 1)); // Center the first motor
			else if((PacketMode_third_motor_state.home_position - PacketMode_third_motor_state.position) * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] > (THIRD_MOTOR_PACKET_MODE+3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[2], (1 * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] * PACKET_MODE_SPEED * 2));
			else {
				MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[2], 0); // Stop the first motor if centered
				PacketMode_backoff_movement_flag |= (0x01U) << 2U;
			}

			if(PacketMode_backoff_movement_flag == 7)
			{
				for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
					homed_motor_mask[i] = false;
					offset_updated_motor_mask[i] = false;
				}
				current_state = ARMSTATE_READY;
			}
			break;
		default:
			break;
	}
}

bool ArmStateManager_ArmReadyPosition(void)
{
	MotorState ArmReadyPosition_first_motor_state = MotorState_Get(0);
	MotorState ArmReadyPosition_second_motor_state = MotorState_Get(1);
	MotorState ArmReadyPosition_third_motor_state = MotorState_Get(2);
	uint8_t ArmReadyPosition_backoff_movement_flag = 0;

	if (((ArmReadyPosition_first_motor_state.home_position - ArmReadyPosition_first_motor_state.position) * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0]) < ((FIRST_MOTOR_BACKOFF_MOVEMENT-3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f) )
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[0], (-1 * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0] * CENTERING_ARM_SPEED * 1)); // Center the first motor
	else if( (ArmReadyPosition_first_motor_state.home_position - ArmReadyPosition_first_motor_state.position) * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0] > (FIRST_MOTOR_BACKOFF_MOVEMENT+3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f )
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[0], (1 * HOMING_DIRECTIONS[0] * JOINT_DIRECTIONS[0] * CENTERING_ARM_SPEED * 2)); // Center the first motor
	else {
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[0], 0); // Stop the first motor if centered
		ArmReadyPosition_backoff_movement_flag |= (0x01U) << 0U;
	}

	if (((ArmReadyPosition_second_motor_state.home_position - ArmReadyPosition_second_motor_state.position) * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1]) < ((SECOND_MOTOR_BACKOFF_MOVEMENT-3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f) )
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[1], (-1 * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1] * CENTERING_ARM_SPEED * 1)); // Center the first motor
	else if((ArmReadyPosition_second_motor_state.home_position - ArmReadyPosition_second_motor_state.position) * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1] > (SECOND_MOTOR_BACKOFF_MOVEMENT+3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[1], (1 * HOMING_DIRECTIONS[1] * JOINT_DIRECTIONS[1] * CENTERING_ARM_SPEED * 2));
	else {
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[1], 0); // Stop the first motor if centered
		ArmReadyPosition_backoff_movement_flag |= (0x01U) << 1U;
	}

	if ((ArmReadyPosition_third_motor_state.home_position - ArmReadyPosition_third_motor_state.position) * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] < (THIRD_MOTOR_BACKOFF_MOVEMENT-3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[2], (-1 * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] * CENTERING_ARM_SPEED * 1)); // Center the first motor
	else if((ArmReadyPosition_third_motor_state.home_position - ArmReadyPosition_third_motor_state.position) * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] > (THIRD_MOTOR_BACKOFF_MOVEMENT+3) * ENCODER_TICKS_PER_REVOLUTION / 360.0f)
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[2], (1 * HOMING_DIRECTIONS[2] * JOINT_DIRECTIONS[2] * CENTERING_ARM_SPEED * 2));
	else {
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[2], 0); // Stop the first motor if centered
		ArmReadyPosition_backoff_movement_flag |= (0x01U) << 2U;
	}

	if(ArmReadyPosition_backoff_movement_flag == 7)
	{
		for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
			homed_motor_mask[i] = false;
			offset_updated_motor_mask[i] = false;
		}
		return true;
	}
	return false;
}
