#include "simulator_manager.h"
#include "motor_control.h"
#include "motor_configuration.h"
#include "motor_state.h"
#include "arm_state_manager.h"
#include "arm_controller.h"
#include "linear_algebra.h"
#include <string.h>

#define LENGTH_OF_SINGLE_VELOCITY_COMMAND 5 // 1 byte for ID and 4 byte for value
#define min(a,b) (((a) < (b)) ? (a) : (b))

static UART_HandleTypeDef *uart_handle = NULL;

static uint8_t receive_buffer[COMMAND_RX_BUFFER_LENGTH];

void SimulatorManager_InitializePortAndTimer(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim2) {
  uart_handle = huart;
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
  HAL_TIM_Base_Start_IT(htim2);
  // HAL_UART_Receive_DMA(uart_handle, receive_buffer, COMMAND_RX_BUFFER_LENGTH);
}


void SimulatorManager_Parse(UART_HandleTypeDef *huart) {
  uint16_t received_length = COMMAND_RX_BUFFER_LENGTH - __HAL_DMA_GET_COUNTER(huart->hdmarx);
  if (received_length > 0) {
    switch(receive_buffer[0]) {
      case VELOCITY_COMMAND: {
        switch (receive_buffer[1]) {
          case CONTROLTYPE_JOINT:
            SimulatorManager_SetJointVelocities(&receive_buffer[2]);
            break;
          case CONTROLTYPE_CARTESIAN:
            SimulatorManager_SetTipVelocity(&receive_buffer[2]);
            break;
          default:
            break;
        }
        break;
      }
      case HOMING_START_COMMAND:
        ArmStateManager_Initialize();
        break;
      case HOMING_SENSOR_MESSAGE:
       // ArmStateManager_OnHomeSensorTriggered(receive_buffer[1]);
        break;
			case GRIPPER_POSITION_COMMAND:
				SimulatorManager_SetGripperPosition(&receive_buffer[1]);
				break;
      default:
        break;
    }
  }
   memset(receive_buffer, 0, COMMAND_RX_BUFFER_LENGTH); // Buffer'ı temizle
  HAL_UART_Receive_DMA(huart, receive_buffer, COMMAND_RX_BUFFER_LENGTH);
}

void SimulatorManager_SendPositions(void) {
  if (uart_handle == NULL) return; // Ensure UART handle is initialized
  uint8_t Transmit_Buffer_Len = 1 + NUMBER_OF_MOTORS * 4; // 1 byte for command type, 4 bytes for each motor position
  uint8_t Transmit_Buffer[Transmit_Buffer_Len]; // 1 byte for command type, 4 bytes for each motor position
  Transmit_Buffer[0] = 10; // Command type
  for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
    int32_t position = MotorState_Get(i).position; // MotorState_Get returns a struct with position field
    Transmit_Buffer[1 + i * 4] = position & 0xFF;
    Transmit_Buffer[2 + i * 4] = (position >> 8) & 0xFF;
    Transmit_Buffer[3 + i * 4] = (position >> 16) & 0xFF;
    Transmit_Buffer[4 + i * 4] = (position >> 24) & 0xFF;
  }
  HAL_UART_Transmit(uart_handle, Transmit_Buffer, Transmit_Buffer_Len, 1000);
}

void SimulatorManager_SetJointVelocities(uint8_t *message) {
  int32_t motor_speeds[NUMBER_OF_MOTORS] = {0};
  uint8_t number_of_joint_commands = message[0];
  for (uint8_t i = 0; i < min(number_of_joint_commands, NUMBER_OF_MOTORS); i++) {
    uint8_t joint_index = message[1 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND];
    int32_t joint_value = message[2 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND] |
                          (message[3 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND] << 8) |
                          (message[4 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND] << 16) |
                          (message[5 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND] << 24);
    motor_speeds[joint_index] = joint_value;
  }
  for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
    ArmController_ApplyJointVelocity(i, motor_speeds[i]);
  }
}

void SimulatorManager_SetTipVelocity(uint8_t *message) {
  Vec3 tip_velocity;
  uint8_t number_of_joint_commands = message[0];
  for (uint8_t i = 0; i < min(number_of_joint_commands, NUMBER_OF_MOTORS); i++) {
    uint8_t joint_index = message[1 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND];
    int32_t joint_value = message[2 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND] |
                          (message[3 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND] << 8) |
                          (message[4 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND] << 16) |
                          (message[5 + i * LENGTH_OF_SINGLE_VELOCITY_COMMAND] << 24);
    tip_velocity.data[joint_index] = joint_value;
  }
  ArmController_ApplyIKVelocity(tip_velocity);
}

void SimulatorManager_SetGripperPosition(uint8_t *message) {
	// ArmController_SetGripperPosition(message[0] | (message[1] << 8));
}
