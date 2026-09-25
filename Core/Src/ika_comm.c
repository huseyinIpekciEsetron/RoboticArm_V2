/*
 * comm.c
 *
 *  Created on: Jun 12, 2025
 *      Author: Huseyin
 */

#include <ika_comm.h>
#include <linear_algebra.h>

Vec3 target_position = { {0.0f, 0.0f, 0.0f} };
float speed = 0.0f;

uint32_t  ComTesttime = 0;
uint32_t ComPeriodTime = 0;

static uint8_t IKA_receive_buffer[RX_MESSAGE_LENGTH+10];

static IKAMessage IKARecMsg; // debug'da izlemek için  
static RoboticArmMessage roboticArmMsg;

static UART_HandleTypeDef *IkaCommUart;

static ArmOperationMode currentArmOperationMode = ARM_MODE_IDLE;

static uint8_t demo_index = 0;

/* Gonderim tamponu: 202 paketi + (her N'de bir) 203 kiskac paketi.
 * HAL_UART_Transmit_IT ile gonderilir, bitene kadar dokunulmaz. */
static uint8_t  ika_tx_buffer[TX_MESSAGE_LENGTH + GRIPPER_PACKET_MAX_LEN];
static uint8_t  gripper_packet_counter = 0;
uint32_t        IkaComm_TxSkipped = 0;   /* UART hala mesgulken atlanan gonderimler */

static uint8_t calculate_crc(uint8_t* data)
{
	uint8_t ck_value=0;

	for(uint8_t i=2; i<12; i++)
		ck_value = (ck_value + data[i]) % 256;

	return ck_value;
}

static MessageStatus CheckSum_is_valid_message(uint8_t* buffer, size_t length) {
    if (length != RX_MESSAGE_LENGTH) return 0;
    if (buffer[0] != HEADER1 || buffer[1] != HEADER2) return 0;
    if (buffer[13] != FOOTER1 || buffer[14] != FOOTER2) return 0;
    if (buffer[2] != 102 ) return 0;
    uint8_t expected_crc = calculate_crc(buffer);
    return (buffer[12] == expected_crc);
}

void IkaComm_SetArmOperationMode(ArmOperationMode armOperatinMode)
{
	currentArmOperationMode = armOperatinMode;
}

void IkaComm_InitializePort(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim)
{
	IkaCommUart = huart;
  __HAL_UART_ENABLE_IT(IkaCommUart, UART_IT_IDLE);
  HAL_UART_Receive_DMA(IkaCommUart, IKA_receive_buffer, RX_MESSAGE_LENGTH);
  HAL_TIM_Base_Start_IT(htim);
}
/*
 * joystick simulatörü ile denediğimizde UART_HandleTypeDef *huart parametresini vermeyince
 * dosyada tanımlanan IkaCommUart adresini doğru bulamadı nedenini araştır
 */
void IkaComm_Parse(UART_HandleTypeDef *huart)
{
	uint16_t received_length = RX_MESSAGE_LENGTH - __HAL_DMA_GET_COUNTER(huart->hdmarx);
	if (received_length > 0) {
		if(IkaComm_decodeMessage(IKA_receive_buffer) != MSG_VALID)
			return;
		ArmState ArmStateValue = ArmStateManager_GetCurrentState();
		switch(IKARecMsg.mode)
		{
			case JOINT_VELOCITY:
				if(ArmStateValue >= ARMSTATE_READY) {
					ArmStateManager_SetCurrentState(ARMSTATE_READY);
					int32_t motor_speeds[NUMBER_OF_MOTORS] = {0};
					motor_speeds[0] = IKARecMsg.joint_velocity.axis_x_motor_1;
					motor_speeds[1] = IKARecMsg.joint_velocity.axis_y_motor_2;
					motor_speeds[2] = IKARecMsg.joint_velocity.axis_z_motor_3;
					motor_speeds[3] = IKARecMsg.motor_4;
					for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++)
						ArmController_ApplyJointVelocity(i, motor_speeds[i]);
					GripperLink_OperatorInput(IKARecMsg.gripper);
				}
				break;
			case GO_HOME:
				if( (ArmStateValue == ARMSTATE_IDLE) && (currentArmOperationMode == ARM_MODE_IDLE) ){ //( ((ArmStateValue == ARMSTATE_READY) || (ArmStateValue == ARMSTATE_IDLE)) && (ArmStateIdleFlag) )
					 ArmStateManager_Initialize();
					 Homing_setHomingState(HOMING_PROGRESS);
				}
				else if( (ArmStateValue == ARMSTATE_READY) )
					ArmStateManager_SetCurrentState(ARMSTATE_READY_POSITION);

				break;
			case CARTESIAN_VELOCITY:
				if( (ArmStateValue == ARMSTATE_READY) && (currentArmOperationMode != ARM_MODE_CARTESIAN_VELOCITY) )
					ArmStateManager_SetCurrentState(ARMSTATE_CARTESIAN_CENTERING_ARM);

				else if((ArmStateValue == ARMSTATE_READY) && (currentArmOperationMode == ARM_MODE_CARTESIAN_VELOCITY))
				{
					Vec3 tip_velocity;
					tip_velocity.data[0] = IKARecMsg.cartesian_velocity.Y_axis;
					tip_velocity.data[1] = IKARecMsg.cartesian_velocity.X_axis;
					tip_velocity.data[2] = IKARecMsg.cartesian_velocity.Z_axis;
					//ArmController_ApplyIKVelocity(tip_velocity);
					ArmController_DriveCartesianClosedLoop(tip_velocity, 0.031f);
					ArmController_ApplyJointVelocity(3, IKARecMsg.motor_4);
					GripperLink_OperatorInput(IKARecMsg.gripper);
				}
				break;
			case MOTOR_STOP_HOMING_CLEAR:
				ArmController_StopAllJoints();
				GripperLink_RequestCommand(GCAN_CMD_STOP);
				ArmStateManager_SetCurrentState(ARMSTATE_IDLE);
				ArmStateManager_ClearHomedMotorMask();
				Homing_setHomingState(HOMING_IDLE);
				currentArmOperationMode = ARM_MODE_IDLE;
				break;
			case PACKET:
				if(currentArmOperationMode == ARM_MODE_PACKET)
					ArmStateManager_SetCurrentState(ARMSTATE_PACKET_MODE_STEP_2);
				else if(ArmStateValue >= ARMSTATE_READY_POSITION) {
					ArmStateManager_SetCurrentState(ARMSTATE_PACKET_MODE_STEP_1);
					ArmController_ApplyJointVelocity(3,0);
				}
				break;
			case DEMO:
				if(ArmStateValue == ARMSTATE_READY && (currentArmOperationMode != ARM_MODE_DEMO))
					ArmStateManager_SetCurrentState(ARMSTATE_DEMO_START_POSITION);

				else if( (ArmStateValue == ARMSTATE_READY) && (currentArmOperationMode == ARM_MODE_DEMO)) {
					bool is_arrived = ArmController_MoveTipToPosition(DEMO_POSITIONS[demo_index], (DEMO_SPEED * IKARecMsg.speed_multiplier), false);
					ArmController_ApplyJointVelocity(3, DEMO_SPEED * IKARecMsg.speed_multiplier * 5);
					if (is_arrived)
						demo_index = (demo_index + 1) % 3;
					/* Kiskac uca dayandikca yon degistirir (gripper_link.c DemoLogic) */
					GripperLink_DemoTick();
				}
				break;
			case IDLE:
				if(currentArmOperationMode == ARM_MODE_DEMO){
					ArmController_StopAllJoints();
				}
				currentArmOperationMode = ARM_MODE_IDLE;
				break;
			default:
				break;
		}

	}
	memset(IKA_receive_buffer, '\0', RX_MESSAGE_LENGTH);
	HAL_UART_Receive_DMA(huart, IKA_receive_buffer, RX_MESSAGE_LENGTH);

	ComPeriodTime = HAL_GetTick() - ComTesttime;
	ComTesttime = HAL_GetTick();
}

MessageStatus IkaComm_encodeMessage(uint8_t* sendBuffer)
{
	if (!sendBuffer)
		return MSG_INVALID;

	sendBuffer[0] = HEADER1;
	sendBuffer[1] = HEADER2;
	sendBuffer[2] = roboticArmMsg.packet_id;

	for(uint8_t i=0; i<4; i++){
		sendBuffer[3 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].position >> 0U) & 0xFFU);
		sendBuffer[4 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].position >> 8U) & 0xFFU);
		sendBuffer[5 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].position >> 16U) & 0xFFU);
		sendBuffer[6 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].position >> 24U) & 0xFFU);
		sendBuffer[7 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].speed >> 0U) & 0xFFU);
		sendBuffer[8 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].speed >> 8U) & 0xFFU);
		sendBuffer[9 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].current >> 0U) & 0xFFU);
		sendBuffer[10 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].current >> 8U) & 0xFFU);
		sendBuffer[11 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].error.whole >> 0U) & 0xFFU);
		sendBuffer[12 + (i*12)] = ((roboticArmMsg.armMotorMsg[i].error.whole >> 8U) & 0xFFU);
		sendBuffer[13 + (i*12)] = roboticArmMsg.armMotorMsg[i].coil_temp;
		sendBuffer[14 + (i*12)] = roboticArmMsg.armMotorMsg[i].board_temp;
	}
	//roboticArmMsg.statusWord = (ArmStateManager_GetCurrentState() == ARMSTATE_READY)? 0x01U:0x00U;
	sendBuffer[51] = roboticArmMsg.statusWord;
	sendBuffer[52] = roboticArmMsg.gripperFlags;
	sendBuffer[53] = roboticArmMsg.gripperState;
	sendBuffer[54] = calculate_crc(sendBuffer);
	sendBuffer[55] = FOOTER1;
	sendBuffer[56] = FOOTER2;

	return MSG_VALID;
}

void IkaComm_buildMotorMessage(uint8_t motorStateIndex)
{
	roboticArmMsg.armMotorMsg[motorStateIndex].position = (int32_t)((float)(MotorState_Get(motorStateIndex).position_degree)*10);
	roboticArmMsg.packet_id = PACKET_ID;
	roboticArmMsg.armMotorMsg[motorStateIndex].board_temp = (int8_t)(MotorState_Get(motorStateIndex).pcb_temperature);
	roboticArmMsg.armMotorMsg[motorStateIndex].coil_temp = (int8_t)(MotorState_Get(motorStateIndex).coil_temperature);
	roboticArmMsg.armMotorMsg[motorStateIndex].error.whole = MotorState_Get(motorStateIndex).error_code.whole;
	roboticArmMsg.armMotorMsg[motorStateIndex].speed = (int16_t)((MotorState_Get(motorStateIndex).velocity * 360) / (101*100));
	roboticArmMsg.armMotorMsg[motorStateIndex].current = MotorState_Get(motorStateIndex).current;
	roboticArmMsg.statusWord = (uint8_t)Homing_getHomingState();

	GripperLinkStatus_t gs;
	GripperLink_GetStatus(&gs);
	roboticArmMsg.gripperFlags = gs.online ? gs.flags : 0U;
	roboticArmMsg.gripperState = (uint8_t)((gs.state & 0x07U) | (gs.online ? 0x08U : 0x00U) |
	                                       ((gs.stop_reason & 0x07U) << 4));
}

/* 203 paketi: kiskac durumu + tum mesafe verisi. Donus: yazilan byte sayisi.
 *  0-1 : 0xAA 0xBB            2 : 203
 *  3   : bit0 kiskac online, bit1 mesafe verisi var
 *  4   : state   5 : motion   6 : stop_reason   7 : flags (GCAN_FLAG_*)
 *  8-9 : akim [mA]            10: duty [%]      11: ariza sayisi
 *  12  : olcum sayaci         13: bolge sayisi (0/16/64)   14: gecerli bolge
 *  15-16: en yakin mesafe [mm] 17: en yakin bolge   18: ToF durum
 *  19.. : bolge sayisi x uint16 mesafe [mm] (0 = gecersiz), little-endian
 *  son 3: CRC (byte 2'den CRC'ye kadar toplam % 256), 0xCC, 0xDD          */
static uint16_t IkaComm_encodeGripperPacket(uint8_t *b)
{
	GripperLinkStatus_t gs;
	GripperLinkToF_t    tof;
	uint16_t n = 0;

	GripperLink_GetStatus(&gs);
	GripperLink_GetToF(&tof);

	uint8_t zones = gs.online ? tof.zone_count : 0U;

	b[n++] = HEADER1;
	b[n++] = HEADER2;
	b[n++] = GRIPPER_PACKET_ID;
	b[n++] = (uint8_t)((gs.online ? 0x01U : 0x00U) | ((zones > 0U) ? 0x02U : 0x00U));
	b[n++] = gs.state;
	b[n++] = gs.motion;
	b[n++] = gs.stop_reason;
	b[n++] = gs.flags;
	b[n++] = (uint8_t)(gs.current_mA & 0xFFU);
	b[n++] = (uint8_t)(gs.current_mA >> 8);
	b[n++] = gs.duty;
	b[n++] = gs.fault_count;
	b[n++] = tof.meas_counter;
	b[n++] = zones;
	b[n++] = tof.valid_count;
	b[n++] = (uint8_t)(tof.min_distance_mm & 0xFFU);
	b[n++] = (uint8_t)(tof.min_distance_mm >> 8);
	b[n++] = tof.min_zone;
	b[n++] = tof.tof_state;
	for (uint8_t z = 0; z < zones; z++) {
		b[n++] = (uint8_t)(tof.distance_mm[z] & 0xFFU);
		b[n++] = (uint8_t)(tof.distance_mm[z] >> 8);
	}
	uint8_t crc = 0;
	for (uint16_t i = 2; i < n; i++)
		crc = (uint8_t)(crc + b[i]);
	b[n++] = crc;
	b[n++] = FOOTER1;
	b[n++] = FOOTER2;
	return n;
}

/* TIM4 kesmesinden (50 Hz) cagrilir. Eskiden HAL_UART_Transmit ile kesme
 * icinde ~5 ms bloklaniyordu (CAN kesmeleri bekliyordu). Artik kesme ile
 * gonderiliyor, fonksiyon hemen donuyor. */
void IkaComm_SendMsg(UART_HandleTypeDef *HUART)
{
	if (HUART->gState != HAL_UART_STATE_READY) {
		IkaComm_TxSkipped++;          /* onceki paket hala gidiyor: bu turu atla */
		return;
	}

	for(uint8_t i=0; i<NUMBER_OF_MOTORS; i++)
		IkaComm_buildMotorMessage(i);

	IkaComm_encodeMessage(ika_tx_buffer);
	uint16_t len = TX_MESSAGE_LENGTH;

	if (++gripper_packet_counter >= GRIPPER_PACKET_EVERY_N) {
		gripper_packet_counter = 0;
		len += IkaComm_encodeGripperPacket(&ika_tx_buffer[TX_MESSAGE_LENGTH]);
	}

	(void)HAL_UART_Transmit_IT(HUART, ika_tx_buffer, len);
}

MessageStatus IkaComm_decodeMessage(uint8_t* receiveBuffer)
{
	if (!receiveBuffer)
		return MSG_INVALID;

	//if (!is_valid_message(receiveBuffer, RX_MESSAGE_LENGTH))
	//	return MSG_CRC_ERROR;
	if(receiveBuffer[2] != 102)
		return 0;
	IKARecMsg.speed_multiplier = (float)(receiveBuffer[10]/255.0f);
	IKARecMsg.packet_id = receiveBuffer[2];
	IKARecMsg.mode = receiveBuffer[9];

	switch(IKARecMsg.mode)
	{
		case JOINT_VELOCITY:
			IKARecMsg.joint_velocity.axis_x_motor_1 = MAP_JOYSTICK_TO_VELOCITY(-1*(int8_t)receiveBuffer[3], IKARecMsg.speed_multiplier);
			IKARecMsg.joint_velocity.axis_y_motor_2 = MAP_JOYSTICK_TO_VELOCITY((int8_t)receiveBuffer[4], IKARecMsg.speed_multiplier);
			IKARecMsg.joint_velocity.axis_z_motor_3 = MAP_JOYSTICK_TO_VELOCITY((int8_t)receiveBuffer[6], IKARecMsg.speed_multiplier);

			IKARecMsg.cartesian_velocity.X_axis = 0;
			IKARecMsg.cartesian_velocity.Y_axis = 0;
			IKARecMsg.cartesian_velocity.Z_axis = 0;
			break;
		case CARTESIAN_VELOCITY:
			IKARecMsg.cartesian_velocity.X_axis = MAP_MOTOR_VELOCITY_TO_CARTESIAN_VELOCITY(-1*(int8_t)receiveBuffer[3], IKARecMsg.speed_multiplier);
			IKARecMsg.cartesian_velocity.Y_axis = MAP_MOTOR_VELOCITY_TO_CARTESIAN_VELOCITY((int8_t)receiveBuffer[4], IKARecMsg.speed_multiplier);
			IKARecMsg.cartesian_velocity.Z_axis = MAP_MOTOR_VELOCITY_TO_CARTESIAN_VELOCITY((int8_t)receiveBuffer[6], IKARecMsg.speed_multiplier);

			IKARecMsg.joint_velocity.axis_x_motor_1 = 0;
			IKARecMsg.joint_velocity.axis_y_motor_2 = 0;
			IKARecMsg.joint_velocity.axis_z_motor_3=  0;
			break;
	}

	IKARecMsg.motor_4 = MAP_JOYSTICK_TO_VELOCITY((int8_t)receiveBuffer[5], IKARecMsg.speed_multiplier) * 2;

	//IKARecMsg.gripper = (int8_t)receiveBuffer[7] / 127.0f;
	IKARecMsg.gripper = (receiveBuffer[7] == 2) ? -1 : receiveBuffer[7];
	IKARecMsg.left_switch = receiveBuffer[8];

	IKARecMsg.reserved = receiveBuffer[11];
	IKARecMsg.crc = receiveBuffer[12];

	return MSG_VALID;
}
