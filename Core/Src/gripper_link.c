/**
  ******************************************************************************
  * @file    gripper_link.c
  * @brief   Master tarafi kiskac baglantisi (CAN)
  ******************************************************************************
  */
#include "gripper_link.h"
#include <string.h>

#define RX_RING_SIZE   32U        /* 2'nin kuvveti. 8x8 ToF = 17 cerceve */
#define REQ_NONE       0xFFU
#define DEMO_RETRY_MS  500U       /* demo: kiskac komutu almadiysa tekrar dene */

typedef struct
{
  uint16_t id;
  uint8_t  dlc;
  uint8_t  data[8];
} GLinkFrame_t;

static CAN_HandleTypeDef   *s_hcan = NULL;

/* CAN RX0 kesmesi -> ana dongu */
static GLinkFrame_t         s_rx[RX_RING_SIZE];
static volatile uint8_t     s_rx_head = 0U;
static volatile uint8_t     s_rx_tail = 0U;
static volatile uint32_t    s_rx_dropped = 0U;

/* ISR'lerden gelen istekler */
static volatile uint8_t     s_req_cmd = REQ_NONE;
static volatile int8_t      s_op_dir = 0;
static volatile uint32_t    s_op_tick = 0U;
static volatile bool        s_op_seen = false;
static volatile uint32_t    s_demo_tick = 0U;
static volatile bool        s_demo_seen = false;

/* Ana dongu durumu */
static GripperLinkStatus_t  s_stat;
static GripperLinkToF_t     s_tof;            /* yayinlanan (tam) olcum */
static GripperLinkToF_t     s_tof_work;       /* birlestirilen olcum */
static uint16_t             s_tof_seg_mask = 0U;
static uint8_t              s_tof_segs = 0U;
static bool                 s_tof_active = false;
static uint8_t              s_cur_cmd = GCAN_CMD_NOP;
static uint8_t              s_seq = 0U;
static uint32_t             s_last_tx_tick = 0U;
static bool                 s_tx_pending = true;
static uint32_t             s_last_status_tick = 0U;
static uint32_t             s_last_issue_tick = 0U;
static int8_t               s_op_applied = 0;
static bool                 s_demo_active = false;

static void Issue(uint8_t cmd);
static void SendCommandFrame(void);
static void HandleFrame(const GLinkFrame_t *f);
static void OperatorLogic(uint32_t now);
static void DemoLogic(uint32_t now);

/* ============================ INIT ======================================= */

void GripperLink_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef f = {0};

  s_hcan = hcan;
  memset(&s_stat, 0, sizeof(s_stat));
  memset(&s_tof, 0, sizeof(s_tof));
  s_tof.min_zone = 0xFFU;

  /* Kiskac mesajlari (0x680-0x69F) -> FIFO0. Motorlar FIFO1'de (bank 10). */
  f.FilterBank           = GLINK_FILTER_BANK;
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh         = (uint32_t)GCAN_FILTER_ID << 5;
  f.FilterIdLow          = 0x0000U;
  f.FilterMaskIdHigh     = (uint32_t)GCAN_FILTER_MASK << 5;
  f.FilterMaskIdLow      = 0x0006U;                 /* IDE=0, RTR=0 zorunlu */
  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  f.FilterActivation     = CAN_FILTER_ENABLE;
  f.SlaveStartFilterBank = 14U;
  (void)HAL_CAN_ConfigFilter(s_hcan, &f);
  (void)HAL_CAN_ActivateNotification(s_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  s_tx_pending = true;
}

/* ============================ ISR TARAFI ================================= */

/* HAL_CAN_RxFifo0MsgPendingCallback'ten cagrilir */
void GripperLink_OnCanRx(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef h;
  uint8_t d[8];

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &h, d) != HAL_OK)
    {
      break;
    }
    if ((h.IDE != CAN_ID_STD) || (h.RTR != CAN_RTR_DATA))
    {
      continue;
    }
    uint8_t head = s_rx_head;
    uint8_t next = (uint8_t)((head + 1U) & (RX_RING_SIZE - 1U));
    if (next == s_rx_tail)
    {
      s_rx_dropped++;
      continue;
    }
    s_rx[head].id  = (uint16_t)h.StdId;
    s_rx[head].dlc = (h.DLC > 8U) ? 8U : (uint8_t)h.DLC;
    memcpy(s_rx[head].data, d, 8U);
    __DMB();
    s_rx_head = next;
  }
}

void GripperLink_OperatorInput(int8_t dir)
{
  s_op_dir  = (dir > 0) ? 1 : ((dir < 0) ? -1 : 0);
  s_op_tick = HAL_GetTick();
  s_op_seen = true;
}

void GripperLink_RequestCommand(uint8_t cmd)
{
  s_req_cmd = cmd;
}

void GripperLink_DemoTick(void)
{
  s_demo_tick = HAL_GetTick();
  s_demo_seen = true;
}

void GripperLink_GetStatus(GripperLinkStatus_t *out)
{
  if (out == NULL) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *out = s_stat;
  __set_PRIMASK(primask);
}

void GripperLink_GetToF(GripperLinkToF_t *out)
{
  if (out == NULL) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *out = s_tof;
  __set_PRIMASK(primask);
}

/* ============================ ANA DONGU ================================== */

void GripperLink_Task(void)
{
  uint32_t now;

  if (s_hcan == NULL)
  {
    return;
  }

  /* 1) Gelen cerceveler */
  while (s_rx_tail != s_rx_head)
  {
    uint8_t tail = s_rx_tail;
    GLinkFrame_t f;
    __DMB();
    f = s_rx[tail];
    __DMB();
    s_rx_tail = (uint8_t)((tail + 1U) & (RX_RING_SIZE - 1U));
    HandleFrame(&f);
  }

  now = HAL_GetTick();

  /* 2) Cevrimici mi */
  {
    bool online = (s_stat.status_count > 0U) &&
                  ((now - s_last_status_tick) <= GLINK_OFFLINE_TIMEOUT_MS);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_stat.online     = online;
    s_stat.rx_dropped = s_rx_dropped;
    __set_PRIMASK(primask);
  }

  /* 3) Istekler: dogrudan komut > demo > operator */
  uint8_t req = s_req_cmd;
  if (req != REQ_NONE)
  {
    s_req_cmd = REQ_NONE;
    Issue(req);
  }
  DemoLogic(now);
  if (!s_demo_active)
  {
    OperatorLogic(now);
  }

  /* 4) Komut cercevesi: yeni komutta hemen, yoksa periyodik (canlilik) */
  if (s_tx_pending || ((now - s_last_tx_tick) >= GLINK_CMD_PERIOD_MS))
  {
    SendCommandFrame();
  }
}

/* Yeni komut = yeni seq. Kiskac ayni seq'i tekrar calistirmaz. */
static void Issue(uint8_t cmd)
{
  s_cur_cmd = cmd;
  s_seq++;
  s_tx_pending = true;
  s_last_issue_tick = HAL_GetTick();

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_stat.last_cmd = s_cur_cmd;
  s_stat.last_seq = s_seq;
  __set_PRIMASK(primask);
}

static void SendCommandFrame(void)
{
  CAN_TxHeaderTypeDef h = {0};
  uint8_t d[8] = {0};
  uint32_t mailbox;

  if (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) == 0U)
  {
    return;                                   /* motor trafigi dolu: sonra dene */
  }
  h.StdId              = GCAN_ID_CMD;
  h.IDE                = CAN_ID_STD;
  h.RTR                = CAN_RTR_DATA;
  h.DLC                = 2U;
  h.TransmitGlobalTime = DISABLE;
  d[0] = s_cur_cmd;
  d[1] = s_seq;

  if (HAL_CAN_AddTxMessage(s_hcan, &h, d, &mailbox) == HAL_OK)
  {
    s_last_tx_tick = HAL_GetTick();
    s_tx_pending   = false;
  }
}

/* Operator tusu: sadece degisimde komut uretir (tus basili tutuldukca
 * ayni komut tekrar edilmez -> akim limitinde duran kiskac zorlanmaz). */
static void OperatorLogic(uint32_t now)
{
  if (!s_op_seen)
  {
    return;
  }

  int8_t dir = s_op_dir;
  bool   timed_out = ((now - s_op_tick) > GLINK_OPERATOR_TIMEOUT_MS);

  /* Operator paketi kesildi: her modda hareketi durdur */
  if (timed_out)
  {
    dir = 0;
  }

  if (dir == s_op_applied)
  {
    return;
  }
  s_op_applied = dir;

  if (dir == 0)
  {
    bool moving_cmd = (s_cur_cmd == GCAN_CMD_OPEN) || (s_cur_cmd == GCAN_CMD_CLOSE);
#if GLINK_OPERATOR_RELEASE_STOPS
    if (moving_cmd)
#else
    if (moving_cmd && timed_out)
#endif
    {
      Issue(GCAN_CMD_STOP);
    }
    return;
  }

#if GLINK_OPERATOR_POSITIVE_IS_CLOSE
  Issue((dir > 0) ? GCAN_CMD_CLOSE : GCAN_CMD_OPEN);
#else
  Issue((dir > 0) ? GCAN_CMD_OPEN : GCAN_CMD_CLOSE);
#endif
}

/* DEMO: kiskac bir uca dayaninca ters yone gec */
static void DemoLogic(uint32_t now)
{
  bool active = s_demo_seen && ((now - s_demo_tick) <= GLINK_OPERATOR_TIMEOUT_MS);

  if (!active)
  {
    if (s_demo_active)
    {
      s_demo_active = false;
      Issue(GCAN_CMD_STOP);                   /* demodan cikinca durdur */
    }
    return;
  }
  s_demo_active = true;
  s_op_applied  = 0;

  if (!s_stat.online || ((s_stat.flags & GCAN_FLAG_MOVING) != 0U) ||
      (s_stat.state != GCAN_STATE_IDLE))
  {
    return;                                   /* hareket bitene kadar bekle */
  }

  uint8_t want;
  if ((s_stat.flags & GCAN_FLAG_BLOCKED_CLOSE) != 0U)      want = GCAN_CMD_OPEN;
  else if ((s_stat.flags & GCAN_FLAG_BLOCKED_OPEN) != 0U)  want = GCAN_CMD_CLOSE;
  else want = (s_stat.motion == GCAN_MOTION_CLOSE) ? GCAN_CMD_OPEN : GCAN_CMD_CLOSE;

  /* Yeni yon -> hemen. Ayni yon ama kiskac hala bosta (orn. yeniden
   * baslayip senkronize oldu) -> ara ile tekrar dene. */
  if ((want != s_cur_cmd) || ((now - s_last_issue_tick) >= DEMO_RETRY_MS))
  {
    Issue(want);
  }
}

static void HandleFrame(const GLinkFrame_t *f)
{
  const uint8_t *d = f->data;
  uint32_t primask;

  if (f->id == GCAN_ID_STATUS)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    s_stat.state       = d[0];
    s_stat.motion      = d[1];
    s_stat.stop_reason = d[2];
    s_stat.flags       = d[3];
    s_stat.current_mA  = GCan_GetU16(&d[4]);
    s_stat.duty        = d[6];
    s_stat.ack_seq     = d[7];
    s_stat.status_count++;
    __set_PRIMASK(primask);
    s_last_status_tick = HAL_GetTick();
  }
  else if (f->id == GCAN_ID_DIAG)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    s_stat.peak_mA      = GCan_GetU16(&d[0]);
    s_stat.move_time_ms = GCan_GetU16(&d[2]);
    s_stat.fault_count  = d[4];
    s_stat.tof_state    = d[5];
    s_stat.diag_flags   = d[6];
    s_stat.fw_version   = d[7];
    __set_PRIMASK(primask);
  }
  else if (f->id == GCAN_ID_TOF_HDR)
  {
    uint8_t zones = d[1];
    uint8_t segs  = d[2];
    if ((zones == 0U) || (zones > GCAN_TOF_MAX_ZONES) || (segs == 0U) ||
        (segs > GCAN_TOF_MAX_SEGS) || ((uint16_t)segs * GCAN_TOF_ZONES_PER_SEG != zones))
    {
      s_tof_active = false;
      return;
    }
    memset(&s_tof_work, 0, sizeof(s_tof_work));
    s_tof_work.meas_counter    = d[0];
    s_tof_work.zone_count      = zones;
    s_tof_work.valid_count     = d[3];
    s_tof_work.min_distance_mm = GCan_GetU16(&d[4]);
    s_tof_work.min_zone        = d[6];
    s_tof_work.tof_state       = d[7];
    s_tof_segs     = segs;
    s_tof_seg_mask = 0U;
    s_tof_active   = true;
  }
  else if ((f->id >= GCAN_ID_TOF_DATA(0U)) && (f->id <= GCAN_ID_LAST))
  {
    uint8_t seg = (uint8_t)(f->id - GCAN_ID_TOF_DATA(0U));
    if (!s_tof_active || (seg >= s_tof_segs))
    {
      return;                                 /* baslik kacirildi: olcumu atla */
    }
    for (uint8_t k = 0U; k < GCAN_TOF_ZONES_PER_SEG; k++)
    {
      s_tof_work.distance_mm[seg * GCAN_TOF_ZONES_PER_SEG + k] = GCan_GetU16(&d[k * 2U]);
    }
    s_tof_seg_mask |= (uint16_t)(1U << seg);

    if (s_tof_seg_mask == (uint16_t)((1UL << s_tof_segs) - 1UL))
    {
      /* Olcum tamam: yayinla */
      s_tof_work.timestamp_ms = HAL_GetTick();
      s_tof_work.frame_count  = s_tof.frame_count + 1U;
      primask = __get_PRIMASK();
      __disable_irq();
      s_tof = s_tof_work;
      __set_PRIMASK(primask);
      s_tof_active = false;
    }
  }
}
