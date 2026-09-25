/*
 * comm.h
 *
 *  Created on: Jun 12, 2025
 *      Author: Huseyin
 */

#ifndef INC_IKA_COMM_H_
#define INC_IKA_COMM_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "stm32f1xx.h"
#include "motor_state.h"
#include "arm_configuration.h"
#include "motor_configuration.h"
#include "arm_state_manager.h"
#include "gripper_link.h"
#include "homing.h"

#define RX_MESSAGE_LENGTH 	255
#define TX_MESSAGE_LENGTH	57
#define HEADER1 			0xAA
#define HEADER2 			0xBB
#define FOOTER1 			0xCC
#define FOOTER2 			0xDD
#define PACKET_ID			202

/* Kiskac + mesafe sensoru paketi (operator tarafina, 202'ye ek) */
#define GRIPPER_PACKET_ID		203
#define GRIPPER_PACKET_EVERY_N	5U      /* TIM4 50 Hz -> her 5. gonderimde = 10 Hz */
#define GRIPPER_PACKET_HDR_LEN	19U     /* ToF mesafelerinden onceki byte sayisi */
#define GRIPPER_PACKET_MAX_LEN	(GRIPPER_PACKET_HDR_LEN + 2U * GCAN_TOF_MAX_ZONES + 3U)

#define MAP_JOYSTICK_TO_VELOCITY(raw, multiplier)  ((int32_t)(((float)((int8_t)(raw)) / 127.0f) * ((float)((multiplier) * MOTOR_MAX_SPEED))))
#define MAP_MOTOR_VELOCITY_TO_CARTESIAN_VELOCITY(raw, multiplier)  ((float)(((float)((int8_t)(raw)) / 127.0f) * ((float)((multiplier) * MAX_CARTESIAN_VELOCITY))))
#define DEMO_SPEED 100.0f

// static const Vec3 DEMO_POSITIONS[12] = {
// 	{ {350.0f, 26.0f, 3.0f} },
// 	{ {350.0f, -26.0f, 3.0f} },
// 	{ {350.0f, -71.0f, 29.0f} },
// 	{ {350.0f, -97.0f, 74.0f} },
// 	{ {350.0f, -97.0f, 126.0f} },
// 	{ {350.0f, -71.0f, 171.0f} },
// 	{ {350.0f, -26.0f, 197.0f} },
// 	{ {350.0f, 26.0f, 197.0f} },
// 	{ {350.0f, 71.0f, 171.0f} },
// 	{ {350.0f, 97.0f, 126.0f} },
// 	{ {350.0f, 97.0f, 74.0f} },
// 	{ {350.0f, 71.0f, 29.0f} }
// };

static const Vec3 DEMO_POSITIONS[3] = {
	{ {280.0f, 0.0f, -150.0f} },
	{ {280.0f, -217.0f, 225.0f} },
	{ {280.0f, 217.0f, 225.0f} }
};


enum message_types
{
	IDLE = 0x00,
	GO_HOME  = 0x04,
	JOINT_VELOCITY = 0x01,
	CARTESIAN_VELOCITY = 0x02,
	MOTOR_STOP_HOMING_CLEAR = 0x05,
	DEMO = 0x03,
	PACKET = 0x06
};

typedef enum {
    ARM_MODE_IDLE = 0x00,
	ARM_MODE_READY_POSITION,
    ARM_MODE_CARTESIAN_VELOCITY,
	ARM_MODE_PACKET,
    ARM_MODE_DEMO
} ArmOperationMode;

typedef struct {
	int32_t position;
	int16_t speed;
	int16_t current;
	ErrorCode error;
	int8_t coil_temp;
	int8_t board_temp;
}ArmMotorMessage;

typedef struct {
	uint8_t packet_id;
	ArmMotorMessage armMotorMsg[4];
	uint8_t statusWord;
	uint8_t gripperFlags;     /* byte 52: GCAN_FLAG_* (kiskac bayraklari)            */
	uint8_t gripperState;     /* byte 53: bit0-2 state, bit3 online, bit4-6 stop_reason */

} RoboticArmMessage;

typedef struct {
	int32_t axis_x_motor_1;
	int32_t axis_y_motor_2;
	int32_t axis_z_motor_3;

}JointVelocity;

typedef struct {
	int32_t X_axis;
	int32_t Y_axis;
	int32_t Z_axis;

}CaretesianVelocity;

typedef struct {
	uint8_t packet_id;
	JointVelocity joint_velocity;
	CaretesianVelocity cartesian_velocity;
	int32_t motor_4;
	int8_t gripper;
	uint8_t left_switch;
	uint8_t mode;
	float speed_multiplier;
	uint8_t reserved;
	uint8_t crc;
}IKAMessage;

typedef enum {
    MSG_INCOMPLETE         = 0,   // Henüz tamamlanmamış mesaj
    MSG_VALID              = 1,   // Doğrulandı, kullanılabilir
    MSG_INVALID            = -1,  // Geçersiz format vs.
    MSG_CRC_ERROR          = -2   // CRC kontrolü başarısız
} MessageStatus;

void IkaComm_SetArmOperationMode(ArmOperationMode armOperatinMode);
void IkaComm_InitializePort(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);
void IkaComm_Parse(UART_HandleTypeDef *huart);
MessageStatus IkaComm_encodeMessage(uint8_t* sendBuffer);
void IkaComm_buildMotorMessage(uint8_t motorID);
//void IkaComm_SendMsg();
void IkaComm_SendMsg(UART_HandleTypeDef *HUART);
MessageStatus IkaComm_decodeMessage(uint8_t* receiveBuffer);

#endif /* INC_IKA_COMM_H_ */
