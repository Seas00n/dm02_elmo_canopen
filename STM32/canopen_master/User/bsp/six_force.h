#ifndef __SIX_FORCE_H__
#define __SIX_FORCE_H__
#include "main.h"
#include "fdcan.h"
extern volatile uint8_t sf_update_flag;
typedef struct
{
    float f_val[2];
    uint16_t u16_val[4];
    uint8_t u8_val[8];
} data_f2;

typedef struct
{
    float f_x;
    float f_y;
    float f_z;
    float m_x;
    float m_y;
    float m_z;
} six_force_node_t;

#define CFIDL_ID 0x700
#define CTXIDL_ID1 0x291
#define CTXIDL_ID2 0x292
#define CTXIDL_ID3 0x293

void six_force_bsp_init(void);
void six_force_zero(void);
void six_force_require(void);
void six_forcer_loop_start(void);
void six_force_loop_stop(void);
uint8_t six_force_can_send(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint32_t len);
void six_force_fdcan_rx_callback();
uint8_t six_force_read(void);
#endif
