#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"
#include "tsja_func.h"

typedef union
{
	float f_val;
	uint32_t u32_val;
	int32_t i32_val;
	uint8_t u8_val[4];
} data_u;
typedef union
{
	uint16_t u16_val;
	int16_t i16_val;
	uint8_t u8_val[2];
} data_u16;

typedef union
{
	uint8_t u8_val;
	int8_t i8_val;
} data_u8;

typedef enum
{
	M1,
	M2,
	num
} motor_num;

// 电机回传信息结构体
typedef struct
{
	uint16_t status_word;
	uint16_t mode;

	float pos;
	float vel;
	float tor;

	float Tmos;
	float Tcoil;
} motor_fbpara_t;

// 电机参数设置结构体
typedef struct
{
	uint16_t word;
	uint16_t mode;

	float pos_set;
	float vel_set;
	float tor_set;

} motor_ctrl_t;

typedef struct
{
	uint8_t id;
	motor_fbpara_t para;
	motor_ctrl_t ctrl;
	motor_ctrl_t cmd;
} motor_t;

typedef struct
{
	uint8_t id;
	motor_ctrl_t axdr_ctrl;
	motor_fbpara_t axdr_para;
	uint8_t axdr_mode_done;
	float axdr_Iq;
	float axdr_Iq_set;
	float axdr_Id;
	float axdr_Id_set;
	float axdr_Ia;
	float axdr_Ib;
	float axdr_Ic;
	float axdr_kp_imp;
	float axdr_kd_imp;
	float axdr_qe_imp;
	float axdr_kd_emd;
	float axdr_pwm_emd;
	float axdr_freq_emd;
} axdr_motor_t;

extern motor_t motor[1];
extern axdr_motor_t axdr_motor[1];

tsja_init_ret_t tsja_slave_init_poll(motor_t *motor, uint8_t id);
uint8_t tsja_rpdo_init(motor_t *motor, uint8_t id);
uint8_t tsja_tpdo_init(motor_t *motor, uint8_t id);
void tsja_motor_ctrl_send(motor_t *motor);
void tsja_motor_fbdata(motor_t *motor);
HAL_StatusTypeDef CANopen_SendNMT_Start(FDCAN_HandleTypeDef *hfdcan, uint8_t node_id);

void open_axdr_tpdo(axdr_motor_t *motor);
void axdr_motor_ctrl_send(axdr_motor_t *motor);
void axdr_motor_fbdata(axdr_motor_t *motor);

/* =========================
 * OD address getter
 * ========================= */

uint32_t *
get_od_32addr(uint16_t mainindex, uint16_t subindex);
uint16_t *get_od_16addr(uint16_t mainindex, uint16_t subindex);
uint8_t *get_od_8addr(uint16_t mainindex, uint16_t subindex);

/* =========================
 * 32-bit OD read/write
 * ========================= */

void write_od_i32(uint16_t mainindex, uint16_t subindex, int32_t data);
int32_t read_od_i32(uint16_t mainindex, uint16_t subindex);

void write_od_u32(uint16_t mainindex, uint16_t subindex, uint32_t data);
uint32_t read_od_u32(uint16_t mainindex, uint16_t subindex);

/* =========================
 * 16-bit OD read/write
 * ========================= */

void write_od_i16(uint16_t mainindex, uint16_t subindex, int16_t data);
int16_t read_od_i16(uint16_t mainindex, uint16_t subindex);

void write_od_u16(uint16_t mainindex, uint16_t subindex, uint16_t data);
uint16_t read_od_u16(uint16_t mainindex, uint16_t subindex);

/* =========================
 * 8-bit OD read/write
 * ========================= */

void write_od_i8(uint16_t mainindex, uint16_t subindex, int8_t data);
int8_t read_od_i8(uint16_t mainindex, uint16_t subindex);

void write_od_u8(uint16_t mainindex, uint16_t subindex, uint8_t data);
uint8_t read_od_u8(uint16_t mainindex, uint16_t subindex);

#endif /* __DM4310_DRV_H__ */
