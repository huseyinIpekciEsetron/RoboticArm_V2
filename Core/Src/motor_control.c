#include "motor_control.h"
#include "stm32f1xx_hal.h"
#include "motor_state.h"
#include "motor_configuration.h"
#include <string.h>

#define MOTOR_CAN_BUFFER_SIZE 16
#define MOTOR_ID_RX_CONSTANT 0x580
#define MOTOR_ID_TX_CONSTANT 0x600

typedef struct {
	uint16_t id;
	uint8_t data[8];
} MotorMessage;

typedef struct {
	MotorMessage messages[MOTOR_CAN_BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
} MotorMessageBuffer;
static MotorMessageBuffer message_buffers[NUMBER_OF_MOTORS];

typedef enum {
	OFFSET_IDLE,
	OFFSET_WAITING_POSITION,
	OFFSET_WAITING_OLD_OFFSET,
	OFFSET_UPDATED
} OffsetUpdateState;

typedef struct {
    OffsetUpdateState state;
    int32_t position;
    int32_t old_offset;
    OffsetUpdateCallback completed_callback;
} OffsetUpdateContext;

static OffsetUpdateContext offset_context[NUMBER_OF_MOTORS];

static CAN_HandleTypeDef *motor_can_handle = NULL;
volatile HAL_StatusTypeDef HalRet;

uint8_t TxData[16];  //is used to store the data, that we are going to transmit over the CAN bus

//CANBUS
CAN_RxHeaderTypeDef receive_header; //CAN Bus Receive Header
uint8_t receive_buffer[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
CAN_FilterTypeDef can_filter; //CAN Bus Filter


static void MotorControl_PushToMessageBuffer(uint16_t id, uint8_t *data) {
	uint16_t motor_index = MotorConfiguration_FindIndexOfMotor(id);
	MotorMessageBuffer *buffer = &message_buffers[motor_index];
	MotorMessage *message = &buffer->messages[buffer->tail];
  message->id = MOTOR_ID_TX_CONSTANT + id;
	memcpy(message->data, data, 8);
	buffer->tail = (buffer->tail + 1) % MOTOR_CAN_BUFFER_SIZE;
	// Overwrite oldest if buffer is full
	if (buffer->tail == buffer->head) {
		buffer->head = (buffer->head + 1) % MOTOR_CAN_BUFFER_SIZE;
	}
}

void MotorControl_ProcessMessageBuffer(uint8_t id) {
	uint16_t motor_index = MotorConfiguration_FindIndexOfMotor(id);
	if (motor_index < 0) return;
	MotorMessageBuffer *buffer = &message_buffers[motor_index];
	if (buffer->head == buffer->tail) return; // nothing to send, check the next motor

	MotorMessage *message = &buffer->messages[buffer->head];
	CAN_TxHeaderTypeDef header = {
		.TransmitGlobalTime = DISABLE,
		.StdId = message->id,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 8
	};
	uint32_t mailbox;
	if (HAL_CAN_AddTxMessage(motor_can_handle, &header, message->data, &mailbox) == HAL_OK) {
		buffer->head = (buffer->head + 1) % MOTOR_CAN_BUFFER_SIZE;
	}
}

void MotorControl_SetAllMotorsToSpeedMode(void) {
	for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
		MotorControl_WriteControlWord(MOTOR_CAN_IDS[i], 0x0006); 	// Shutdown
		MotorControl_SetMode(MOTOR_CAN_IDS[i], 0x03); 								// Set to profile velocity mode
		MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[i], 0); 					// Set initial speed to 0
		MotorControl_WriteControlWord(MOTOR_CAN_IDS[i], 0x000F);	// Switch On + Enable Operation
		MotorControl_WriteControlWord(MOTOR_CAN_IDS[i], 0x001F);	// Apply new setpoint
	}
}

void MotorControl_ActiveMechanicalBreak(uint32_t id)
{
	uint8_t CAN_ID_index = MotorConfiguration_FindIndexOfMotor(id);
	MotorState_MechanicalBreak[CAN_ID_index]=true;
	MotorControl_SetMotorVelocity(id, 0);
	MotorControl_WriteControlWord(id, 0x0006);
}

void MotorControl_DeactiveMechanicalBreak(uint32_t id)
{
	uint8_t CAN_ID_index = MotorConfiguration_FindIndexOfMotor(id);
	MotorState_MechanicalBreak[CAN_ID_index]=false;
	MotorControl_WriteControlWord(id, 0x001F);
}

void MotorControl_InitalizeCanProtocol(CAN_HandleTypeDef *hcan)
{
  motor_can_handle = hcan;

  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.FilterBank = 10;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow = 0;
  can_filter.FilterMaskIdHigh = 0x000<<5;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.SlaveStartFilterBank = 0;	
  
  HAL_CAN_ConfigFilter(motor_can_handle, &can_filter); 
  HAL_CAN_Start(motor_can_handle);
  HalRet = HAL_CAN_ActivateNotification(motor_can_handle, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void MotorControl_ProcessCanMessage(void) {
  if (HAL_CAN_GetRxMessage(motor_can_handle, CAN_RX_FIFO1, &receive_header, receive_buffer) == HAL_OK) {
    uint16_t motor_id = receive_header.StdId - MOTOR_ID_RX_CONSTANT;
    size_t motor_index = MotorConfiguration_FindIndexOfMotor(motor_id);
    if (motor_index < 0) return; // Ignore messages with invalid IDs

    uint8_t number_of_data_bytes = 0;
    switch (receive_buffer[0])
    {
      case 0x4F:
        number_of_data_bytes = 1;
        break;
      case 0x4B:
        number_of_data_bytes = 2;
        break;
      case 0x43:
        number_of_data_bytes = 4;
        break;
      default:
        return; // TODO: Handle success and exception responses
    }
    uint16_t message_index = (receive_buffer[2] << 8) | receive_buffer[1];

    switch (message_index)
    {
      case COIL_TEMPERATURE:
    	  MotorState_Set(motor_index, MOTOR_STATE_COIL_TEMPERATURE, &receive_buffer[4], number_of_data_bytes);
        break;
      case CURRENT:
    	  MotorState_Set(motor_index, MOTOR_STATE_CURRENT, &receive_buffer[4], number_of_data_bytes);
		break;
      case ERROR_CODE:
    	  MotorState_Set(motor_index, MOTOR_STATE_ERROR_CODE, &receive_buffer[4], number_of_data_bytes);
		break;
      case PCB_TEMPERATURE:
    	  MotorState_Set(motor_index, MOTOR_STATE_PCB_TEMPERATURE, &receive_buffer[4], number_of_data_bytes);
        break;
      case POSITION:
    	  MotorState_Set(motor_index, MOTOR_STATE_POSITION, &receive_buffer[4], number_of_data_bytes);
		  float position_degree = convertEncoderTicksToDegrees(*(int32_t*)(&receive_buffer[4]), motor_index);
    	  MotorState_Set(motor_index, MOTOR_STATE_POSITION_DEGREES, &position_degree, number_of_data_bytes);
		break;
      case POSITION_OFFSET:
    	  MotorState_Set(motor_index, MOTOR_STATE_POSITION_OFFSET, &receive_buffer[4], number_of_data_bytes);
		break;
      case STATUS_WORD:
    	  MotorState_Set(motor_index, MOTOR_STATE_STATUS_WORD, &receive_buffer[4], number_of_data_bytes);
		break;
      case VELOCITY:
    	  MotorState_Set(motor_index, MOTOR_STATE_VELOCITY, &receive_buffer[4], number_of_data_bytes);
        break;
      default:
        break;
    }
		MotorControl_Update_OffsetUpdateContext(motor_index, message_index, (int32_t *)&receive_buffer[4]);
  }
}

void MotorControl_WriteControlWord(uint32_t id, uint16_t word)
{
	uint8_t lsb = word & 0xFF;
	uint8_t msb = (word >> 8) & 0xFF;
	uint8_t data[8] = {0x2B, 0x40, 0x60, 0x00, lsb, msb, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
}
void MotorControl_ReadStatusWord(uint32_t id)
{
  uint8_t data[8]={0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
}

void MotorControl_SetMode(uint32_t id, uint8_t mode) //default value:1
{
	uint8_t data[8] = {0x2F, 0x60, 0x60, 0x00, mode, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
}

void MotorControl_SetBaudrate(uint32_t id, uint16_t baudrate)
{
	uint8_t lsb = baudrate & 0xFF;
	uint8_t msb = (baudrate >> 8) & 0xFF;
	uint8_t data[8] = {0x2B, 0x07, 0x20, 0x00, lsb, msb, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
  MotorControl_SaveChanges(id);
}

void MotorControl_SetNodeId(uint32_t id, uint8_t nodeID)
{
	if (nodeID > 127) return; // Node ID must be in the range 0-127
	uint8_t data[8] = {0x2B, 0x06, 0x20, 0x00, nodeID, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
  MotorControl_SaveChanges(id);	
}

void MotorControl_GoToPositionInDegrees(uint32_t id, int32_t position)
{
	int32_t position_in_ticks = (position * 65536 * 101) / 360; // Convert degrees to ticks (assuming 65536 * 101 ticks per revolution)
	MotorControl_GoToPosition(id, position_in_ticks);
}

void MotorControl_GoToPosition(uint32_t id, int32_t position)
{
	uint8_t data[8] = {
		0x23, // Header
		0x7A, // LSB of index for  target position
		0x60, // MSB of index for target position
		0x00, // Subindex
		position & 0xFF,
		(position >> 8) & 0xFF,
		(position >> 16) & 0xFF,
		(position >> 24) & 0xFF
	};
  MotorControl_PushToMessageBuffer(id, data);
  MotorControl_WriteControlWord(id, 0x001F);
}

void MotorControl_ReadPosition(uint32_t id)
{
	uint8_t data[8] = {0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
}

void MotorControl_SetMinimumLimitPosition(uint32_t id, int32_t limit)
{
	uint8_t data[8] = {
		0x23, // Header
		0x7D, // LSB of index for limit position
		0x60, // MSB of index for limit position
		0x01, // Subindex for minimum limit
		limit & 0xFF,
		(limit >> 8) & 0xFF,
		(limit >> 16) & 0xFF,
		(limit >> 24) & 0xFF
	};
  MotorControl_PushToMessageBuffer(id, data);
}

void MotorControl_SetMaximumLimitPosition(uint32_t id, int32_t limit)
{
	uint8_t data[8] = {
		0x23, // Header
		0x7D, // LSB of index for limit position
		0x60, // MSB of index for limit position
		0x02, // Subindex maximum limit
		limit & 0xFF,
		(limit >> 8) & 0xFF,
		(limit >> 16) & 0xFF,
		(limit >> 24) & 0xFF
	};
  MotorControl_PushToMessageBuffer(id, data);
}

void MotorControl_SetProfileVelocity(uint32_t id, uint32_t profileVelocity) //default value:1.000
{
  TxData[0] = 0x22;
  TxData[1] = 0x81;
  TxData[2] = 0x60;
  TxData[3] = 0x00;
  TxData[4] = (profileVelocity & 0x000000FF);	
  TxData[5] = (profileVelocity & 0x0000FF00) >> 8;
  TxData[6] = (profileVelocity & 0x00FF0000) >> 16;
  TxData[7] = (profileVelocity & 0xFF000000) >> 24;
  MotorControl_PushToMessageBuffer(id, TxData);
}

void MotorControl_SetProfileAcceleration(uint32_t id, uint32_t acceleration_value) //default value:10.000
{
  TxData[0] = 0x22;
  TxData[1] = 0x83;
  TxData[2] = 0x60;
  TxData[3] = 0x00;
  TxData[4] = (acceleration_value & 0x000000FF);	
  TxData[5] = (acceleration_value & 0x0000FF00) >> 8;
  TxData[6] = (acceleration_value & 0x00FF0000) >> 16;
  TxData[7] = (acceleration_value & 0xFF000000) >> 24;
  MotorControl_PushToMessageBuffer(id, TxData);
}
void MotorControl_SetProfileDeceleration(uint32_t id, uint32_t deceleration_value) //default value:10.000
{
  TxData[0] = 0x22;
  TxData[1] = 0x84;
  TxData[2] = 0x60;
  TxData[3] = 0x00;
  TxData[4] = (deceleration_value & 0x000000FF);
  TxData[5] = (deceleration_value & 0x0000FF00) >> 8;	
  TxData[6] = (deceleration_value & 0x00FF0000) >> 16;	
  TxData[7] = (deceleration_value & 0xFF000000) >> 24;
  MotorControl_PushToMessageBuffer(id, TxData);
}

void MotorControl_ReadCurrentValue(uint32_t id)
{
	uint8_t data[8] = {0x40, 0x78, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
}

void MotorControl_ReadOffsetValue(uint32_t id)
{
	uint8_t data[8] = {0x40, 0x08, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
}
void MotorControl_SetOffsetValue(uint32_t id, int32_t new_offset_value)
{
	uint8_t data[8] = {
   0x23,
   0x08,
   0x20,
   0x00,
	 new_offset_value & 0xFF,
	 (new_offset_value >> 8) & 0xFF,
	 (new_offset_value >> 16) & 0xFF,
	 (new_offset_value >> 24) & 0xFF
	};
  MotorControl_PushToMessageBuffer(id, data);
  MotorControl_SaveChanges(id);
}
void MotorControl_SaveChanges(uint32_t id)
{
	uint8_t data[8] = {0x23, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
	MotorControl_PushToMessageBuffer(id, data);
}
void MotorControl_ReadErrorCode(uint32_t id)
{
	uint8_t data[8] = {0x40, 0x3F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
}
void MotorControl_SetMotorVelocity(uint32_t id, int32_t velocity)
{
	uint8_t CAN_ID_index = MotorConfiguration_FindIndexOfMotor(id);
	velocity = (MotorState_MechanicalBreak[CAN_ID_index] == true) ? 0:velocity;
  TxData[0] = 0x23; //header
  TxData[1] = 0xFF; //low add
  TxData[2] = 0x60; //high add
  TxData[3] = 0x00; //subindex
  TxData[4] = (velocity & 0x000000FF);		
  TxData[5] = (velocity & 0x0000FF00) >> 8;
  TxData[6] = (velocity & 0x00FF0000) >> 16;	
  TxData[7] = (velocity & 0xFF000000) >> 24;
  MotorControl_PushToMessageBuffer(id, TxData);
  // MotorControl_WriteControlWord(id, 0x001F);
}
void MotorControl_ReadMotorVelocity(uint32_t id)
{
	uint8_t data[8] = {0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);	
}

void MotorControl_ReadCoilTemperature(uint32_t id)
{
	uint8_t data[8] = {0x40, 0x0C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
}
void MotorControl_ReadBoardTemperature(uint32_t id)
{
	uint8_t data[8] = {0x40, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
  MotorControl_PushToMessageBuffer(id, data);
}

/*
void ConvertLittleEndianArrayToValue(const uint8_t* in_array, void* out_value, size_t length) {
  if (length > 4) length = 4;  // Limit to 4 bytes
  uint32_t result = 0;
  for (size_t i = 0; i < length; ++i) {
      result |= ((uint32_t)in_array[i]) << (8 * i);
  }
  switch (num_bytes) {
    case 1:
        *(uint8_t*)out_value = (uint8_t)result;
        break;
    case 2:
        *(uint16_t*)out_value = (uint16_t)result;
        break;
    case 3: // Fall through
    case 4:
        *(uint32_t*)out_value = result;
        break;
    default:
        // Optional: handle 0 bytes or invalid cases
        break;
  }
}
*/

void MotorControl_StartOffsetUpdate(uint8_t id, OffsetUpdateCallback completed_callback) {
	uint16_t motor_index = MotorConfiguration_FindIndexOfMotor(id);
	if (motor_index < 0 ) return;
	offset_context[motor_index].state = OFFSET_WAITING_POSITION;
	offset_context[motor_index].completed_callback = completed_callback;
	MotorControl_ReadPosition(id);
}

void MotorControl_Update_OffsetUpdateContext(uint16_t motor_index, uint16_t message_index, int32_t *value)
{
	OffsetUpdateContext *context = &offset_context[motor_index];
	switch (context->state)
	{
		case OFFSET_WAITING_POSITION:
			if (message_index == POSITION) {
				context->position = *value;
				context->state = OFFSET_WAITING_OLD_OFFSET;
				MotorControl_ReadOffsetValue(MOTOR_CAN_IDS[motor_index]); // Request old offset value
			}
			break;
		case OFFSET_WAITING_OLD_OFFSET:
			if (message_index == POSITION_OFFSET) {
				context->old_offset = *value;
				int32_t new_offset = context->position + context->old_offset;
				MotorControl_SetOffsetValue(MOTOR_CAN_IDS[motor_index], new_offset); // Save new offset value
				context->state = OFFSET_UPDATED;
				if(context->completed_callback) {
					context->completed_callback(MOTOR_CAN_IDS[motor_index]);
				}
			}
			break;
		case OFFSET_UPDATED:
			MotorControl_ReadOffsetValue(MOTOR_CAN_IDS[motor_index]); // Reset state by reading offset again
			MotorControl_ReadPosition(MOTOR_CAN_IDS[motor_index]); // Request new position
			context->state = OFFSET_IDLE; // Reset to waiting for position
			break;
		default:
			break;
	}
}
/*
void MotorControl_RequestMessage()
{
	static uint32_t currentTime=0;
	static uint8_t step = 0;
	if((HAL_GetTick()-currentTime)>200)
	{
		switch(step)
		{
			case 0:
				for(uint8_t i = 0; i < NUMBER_OF_MOTORS; i++)
					MotorControl_ReadErrorCode(MOTOR_CAN_IDS[i]);
				step=1;
				break;
			case 1:
				for(uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
					MotorControl_ReadCurrentValue(MOTOR_CAN_IDS[i]);
					MotorControl_ReadMotorVelocity(MOTOR_CAN_IDS[i]);
				}
				step=2;
				break;
			case 2:
				for(uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
					MotorControl_ReadBoardTemperature(MOTOR_CAN_IDS[i]);
					MotorControl_ReadCoilTemperature(MOTOR_CAN_IDS[i]);
				}
				step=0;
				break;
		}
		currentTime = HAL_GetTick();
	}
}
*/
void MotorControl_RequestMessage()
{
    static uint8_t step = 0;

    void (*MotorControlRead_functions[])(uint32_t) = {
        MotorControl_ReadErrorCode,
        MotorControl_ReadCurrentValue,
        MotorControl_ReadBoardTemperature,
        MotorControl_ReadMotorVelocity,
        MotorControl_ReadCoilTemperature
    };

    const uint8_t num_steps = sizeof(MotorControlRead_functions) / sizeof(MotorControlRead_functions[0]);

    for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
    	MotorControlRead_functions[step](MOTOR_CAN_IDS[i]);
    }

    step = (step + 1) % num_steps;
}
