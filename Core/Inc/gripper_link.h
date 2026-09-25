/**
  ******************************************************************************
  * @file    gripper_link.h
  * @brief   Master tarafi kiskac baglantisi (CAN). gripper_controller'in yerini alir.
  *
  *  - Kiskaca komut gonderir (50 ms'de bir tekrarlanir = canlilik sinyali)
  *  - Kiskacin durum, tani ve ToF mesajlarini toplar
  *  - Operator (UART1) kiskac tusu -> kiskac komutu
  *
  *  ISR'den cagrilabilen fonksiyonlar (sadece istek birakir, CAN'e dokunmaz):
  *    GripperLink_OperatorInput, GripperLink_RequestCommand, GripperLink_DemoTick,
  *    GripperLink_OnCanRx (CAN RX0 kesmesinden)
  *  Ana donguden cagrilacak: GripperLink_Task
  ******************************************************************************
  */
#ifndef GRIPPER_LINK_H
#define GRIPPER_LINK_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "gripper_can_defs.h"

/* ============================ AYARLAR ==================================== */
#define GLINK_CMD_PERIOD_MS        50U    /* komut/canlilik cercevesi periyodu */
#define GLINK_OFFLINE_TIMEOUT_MS   300U   /* bu kadar durum gelmezse kiskac cevrimdisi */
#define GLINK_OPERATOR_TIMEOUT_MS  500U   /* operator paketi kesilirse kiskaci durdur */
#define GLINK_FILTER_BANK          11U    /* motor filtresi bank 10'da */

/* Operator kiskac baytinin yonu: +1 (UART'ta 1) kapatir mi? Ters ise 0 yap */
#define GLINK_OPERATOR_POSITIVE_IS_CLOSE  1
/* 1: operator tusu birakinca kiskac durur (eski servo davranisi gibi)
 * 0: tek basis yeter, kiskac akim limitine kadar gider                    */
#define GLINK_OPERATOR_RELEASE_STOPS      1

typedef struct
{
  bool     online;            /* son GLINK_OFFLINE_TIMEOUT_MS icinde durum geldi */
  uint8_t  state;             /* GCAN_STATE_*  */
  uint8_t  motion;            /* GCAN_MOTION_* */
  uint8_t  stop_reason;       /* GCAN_STOP_*   */
  uint8_t  flags;             /* GCAN_FLAG_*   */
  uint16_t current_mA;
  uint8_t  duty;
  uint8_t  ack_seq;
  /* tani */
  uint16_t peak_mA;
  uint16_t move_time_ms;
  uint8_t  fault_count;
  uint8_t  tof_state;         /* bit0-1 state, bit2-4 hata */
  uint8_t  diag_flags;        /* GCAN_DIAG_* */
  uint8_t  fw_version;
  /* master tarafi */
  uint8_t  last_cmd;          /* son gonderilen komut */
  uint8_t  last_seq;
  uint32_t status_count;
  uint32_t rx_dropped;
} GripperLinkStatus_t;

typedef struct
{
  uint8_t  meas_counter;
  uint8_t  zone_count;        /* 0 = henuz veri yok */
  uint8_t  valid_count;
  uint8_t  min_zone;          /* 0xFF = yok */
  uint8_t  tof_state;
  uint16_t min_distance_mm;   /* 0 = yok */
  uint16_t distance_mm[GCAN_TOF_MAX_ZONES];   /* 0 = gecersiz */
  uint32_t timestamp_ms;
  uint32_t frame_count;
} GripperLinkToF_t;

void GripperLink_Init(CAN_HandleTypeDef *hcan);
void GripperLink_Task(void);

/* ISR guvenli */
void GripperLink_OnCanRx(CAN_HandleTypeDef *hcan);
void GripperLink_OperatorInput(int8_t dir);       /* +1 / -1 / 0 */
void GripperLink_RequestCommand(uint8_t cmd);     /* GCAN_CMD_* */
void GripperLink_DemoTick(void);                  /* DEMO modunda her operator paketinde */

/* Okuma (ISR'den de guvenli: kopya kritik bolgede alinir) */
void GripperLink_GetStatus(GripperLinkStatus_t *out);
void GripperLink_GetToF(GripperLinkToF_t *out);

#endif /* GRIPPER_LINK_H */
