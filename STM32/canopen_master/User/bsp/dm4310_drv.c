#include "dm4310_drv.h"
#include "fdcan.h"
#include "co_app.h"
#include "co_bsp.h"
#include "301/CO_SDOclient.h"
#include "OD.h"
#include "tsja_func.h"

extern CO_t *CO;

motor_t motor[1];

axdr_motor_t axdr_motor[1];

uint8_t tsja_rpdo_init(motor_t *motor, uint8_t id)
{
	motor->id = id;

	uint8_t node_id = motor->id;

	uint8_t u8_val;
	uint32_t u32_val;

	/*
	 * =========================
	 * 配置 Slave RPDO1
	 * 对应 Master TPDO1
	 *
	 * COB-ID = 0x180
	 *
	 * Mapping:
	 * 0x6040:00 Controlword        16 bit
	 * 0x6060:00 Modes of operation 8 bit
	 * 0x6071:00 Target torque      16 bit
	 * =========================
	 */

	// 1. 先关闭 RPDO1
	u32_val = PDO_DISABLE_BIT | TSJA_SLAVE_RPDO1_COB(node_id); // 首位1，终止RPDO
	write_sdo(node_id, 0x1400, 0x01, (uint8_t *)&u32_val, 4);

	// 2. 清空 RPDO1 映射数量
	u8_val = 0;
	write_sdo(node_id, 0x1600, 0x00, (uint8_t *)&u8_val, 1);

	// 3. 配置 RPDO1 映射内容
	u32_val = 0x60400010; // 0x6040:00 Controlword, 16 bit
	write_sdo(node_id, 0x1600, 0x01, (uint8_t *)&u32_val, 4);

	u32_val = 0x60600008; // 0x6060:00 Modes of operation, 8 bit
	write_sdo(node_id, 0x1600, 0x02, (uint8_t *)&u32_val, 4);

	u32_val = 0x60600008; // 0x6060:00 Modes of operation, 8 bit
	write_sdo(node_id, 0x1600, 0x03, (uint8_t *)&u32_val, 4);

	u32_val = 0x60600008; // 0x6060:00 Modes of operation, 8 bit
	write_sdo(node_id, 0x1600, 0x04, (uint8_t *)&u32_val, 4);

	u32_val = 0x60600008; // 0x6060:00 Modes of operation, 8 bit
	write_sdo(node_id, 0x1600, 0x05, (uint8_t *)&u32_val, 4);

	u32_val = 0x60710010; // 0x6071:00 Target torque, 16 bit
	write_sdo(node_id, 0x1600, 0x06, (uint8_t *)&u32_val, 4);

	// 4. 设置 RPDO1 映射数量 = 6
	u8_val = 6;
	write_sdo(node_id, 0x1600, 0x00, (uint8_t *)&u8_val, 1);

	// 5. 设置 RPDO1 传输类型
	u8_val = RPDO_TRANS_TYPE_ASYNC;
	write_sdo(node_id, 0x1400, 0x02, (uint8_t *)&u8_val, 1);

	// 6. 重新打开 RPDO1
	u32_val = TSJA_SLAVE_RPDO1_COB(node_id);
	write_sdo(node_id, 0x1400, 0x01, (uint8_t *)&u32_val, 4);

	/*
	 * =========================
	 * 配置 Slave RPDO2
	 * 对应 Master TPDO2
	 *
	 * COB-ID = 0x280
	 *
	 * Mapping:
	 * 0x60FF:00 Target velocity    32 bit
	 * 0x607A:00 Target position    32 bit
	 * =========================
	 */

	// 1. 先关闭 RPDO2
	u32_val = PDO_DISABLE_BIT | TSJA_SLAVE_RPDO2_COB(node_id);
	write_sdo(node_id, 0x1401, 0x01, (uint8_t *)&u32_val, 4);

	// 2. 清空 RPDO2 映射数量
	u8_val = 0;
	write_sdo(node_id, 0x1601, 0x00, (uint8_t *)&u8_val, 1);

	// 3. 配置 RPDO2 映射内容
	u32_val = 0x60FF0020; // 0x60FF:00 Target velocity, 32 bit
	write_sdo(node_id, 0x1601, 0x01, (uint8_t *)&u32_val, 4);

	u32_val = 0x607A0020; // 0x607A:00 Target position, 32 bit
	write_sdo(node_id, 0x1601, 0x02, (uint8_t *)&u32_val, 4);

	// 4. 设置 RPDO2 映射数量 = 2
	u8_val = 2;
	write_sdo(node_id, 0x1601, 0x00, (uint8_t *)&u8_val, 1);

	// 5. 设置 RPDO2 传输类型
	u8_val = RPDO_TRANS_TYPE_ASYNC;
	write_sdo(node_id, 0x1401, 0x02, (uint8_t *)&u8_val, 1);

	// 6. 重新打开 RPDO2
	u32_val = TSJA_SLAVE_RPDO2_COB(node_id);
	write_sdo(node_id, 0x1401, 0x01, (uint8_t *)&u32_val, 4);

	return 0;
}

uint8_t tsja_tpdo_init(motor_t *motor, uint8_t id)
{
	motor->id = id;

	uint8_t node_id = motor->id;

	uint8_t u8_val;
	uint16_t u16_val;
	uint32_t u32_val;

	/*
	 * =========================
	 * 配置 Slave TPDO1
	 * 对应 Master RPDO1
	 *
	 * COB-ID = 0x200
	 *
	 * Mapping:
	 * 0x6041:00 Statusword              16 bit
	 * 0x6061:00 Modes of operation disp 8 bit
	 * 0x6077:00 Torque actual value     16 bit
	 * =========================
	 */

	// 1. 先关闭 TPDO1
	u32_val = PDO_DISABLE_BIT | TSJA_SLAVE_TPDO1_COB(node_id);
	write_sdo(node_id, 0x1800, 0x01, (uint8_t *)&u32_val, 4);

	// 2. 清空 TPDO1 映射数量
	u8_val = 0;
	write_sdo(node_id, 0x1A00, 0x00, (uint8_t *)&u8_val, 1);

	// 3. 配置 TPDO1 映射内容
	u32_val = 0x60410010; // 0x6041:00 Statusword, 16 bit
	write_sdo(node_id, 0x1A00, 0x01, (uint8_t *)&u32_val, 4);

	u32_val = 0x60610008; // 0x6061:00 Modes of operation display, 8 bit
	write_sdo(node_id, 0x1A00, 0x02, (uint8_t *)&u32_val, 4);

	u32_val = 0x60610008; // 0x6061:00 Modes of operation display, 8 bit
	write_sdo(node_id, 0x1A00, 0x03, (uint8_t *)&u32_val, 4);

	u32_val = 0x60610008; // 0x6061:00 Modes of operation display, 8 bit
	write_sdo(node_id, 0x1A00, 0x04, (uint8_t *)&u32_val, 4);

	u32_val = 0x60610008; // 0x6061:00 Modes of operation display, 8 bit
	write_sdo(node_id, 0x1A00, 0x05, (uint8_t *)&u32_val, 4);

	u32_val = 0x60770010; // 0x6077:00 Torque actual value, 16 bit
	write_sdo(node_id, 0x1A00, 0x06, (uint8_t *)&u32_val, 4);

	// 4. 设置 TPDO1 映射数量 = 6
	u8_val = 6;
	write_sdo(node_id, 0x1A00, 0x00, (uint8_t *)&u8_val, 1);

	// 5. 设置 TPDO1 传输类型
	u8_val = TPDO_TRANS_TYPE_ASYNC;
	write_sdo(node_id, 0x1800, 0x02, (uint8_t *)&u8_val, 1);

	// 6. 设置 TPDO1 事件定时器
	u16_val = TPDO_EVENT_TIMER_MS;
	write_sdo(node_id, 0x1800, 0x05, (uint8_t *)&u16_val, 2);

	// 7. 重新打开 TPDO1
	u32_val = TSJA_SLAVE_TPDO1_COB(node_id);
	write_sdo(node_id, 0x1800, 0x01, (uint8_t *)&u32_val, 4);

	/*
	 * =========================
	 * 配置 Slave TPDO2
	 * 对应 Master RPDO2
	 *
	 * COB-ID = 0x300
	 *
	 * Mapping:
	 * 0x606C:00 Velocity actual value   32 bit
	 * 0x6064:00 Position actual value   32 bit
	 * =========================
	 */

	// 1. 先关闭 TPDO2
	u32_val = PDO_DISABLE_BIT | TSJA_SLAVE_TPDO2_COB(node_id);
	write_sdo(node_id, 0x1801, 0x01, (uint8_t *)&u32_val, 4);

	// 2. 清空 TPDO2 映射数量
	u8_val = 0;
	write_sdo(node_id, 0x1A01, 0x00, (uint8_t *)&u8_val, 1);

	// 3. 配置 TPDO2 映射内容
	u32_val = 0x606C0020; // 0x606C:00 Velocity actual value, 32 bit
	write_sdo(node_id, 0x1A01, 0x01, (uint8_t *)&u32_val, 4);

	u32_val = 0x60640020; // 0x6064:00 Position actual value, 32 bit
	write_sdo(node_id, 0x1A01, 0x02, (uint8_t *)&u32_val, 4);

	// 4. 设置 TPDO2 映射数量 = 2
	u8_val = 2;
	write_sdo(node_id, 0x1A01, 0x00, (uint8_t *)&u8_val, 1);

	// 5. 设置 TPDO2 传输类型
	u8_val = TPDO_TRANS_TYPE_ASYNC;
	write_sdo(node_id, 0x1801, 0x02, (uint8_t *)&u8_val, 1);

	// 6. 设置 TPDO2 事件定时器
	u16_val = TPDO_EVENT_TIMER_MS;
	write_sdo(node_id, 0x1801, 0x05, (uint8_t *)&u16_val, 2);

	// 7. 重新打开 TPDO2
	u32_val = TSJA_SLAVE_TPDO2_COB(node_id);
	write_sdo(node_id, 0x1801, 0x01, (uint8_t *)&u32_val, 4);

	return 0;
}

HAL_StatusTypeDef CANopen_SendNMT_Start(FDCAN_HandleTypeDef *hfdcan, uint8_t node_id)
{
	FDCAN_TxHeaderTypeDef txHeader;
	uint8_t txData[2];

	txHeader.Identifier = 0x000;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_2;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0;

	txData[0] = 0x01;	 // Start Remote Node
	txData[1] = node_id; // 0x01 启动节点1，0x00广播启动所有节点

	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData);
}

tsja_init_ret_t tsja_slave_init_poll(motor_t *motor, uint8_t id)
{
	static tsja_init_state_t init_state = TSJA_INIT_IDLE;
	static uint32_t state_tick = 0;

	uint16_t status = motor->para.status_word & TSJA_SW_MASK;

	switch (init_state)
	{
	case TSJA_INIT_IDLE:
		motor->id = id;
		motor->ctrl.word = TSJA_CW_SHUTDOWN;
		state_tick = HAL_GetTick();
		init_state = TSJA_INIT_SHUTDOWN;
		return TSJA_INIT_RET_RUNNING;

	case TSJA_INIT_SHUTDOWN:
		if (status == TSJA_SW_READY)
		{
			motor->ctrl.word = TSJA_CW_SWITCH_ON;
			state_tick = HAL_GetTick();
			init_state = TSJA_INIT_SWITCH_ON;
		}
		else if (HAL_GetTick() - state_tick > TSJA_INIT_TIMEOUT_MS)
		{
			init_state = TSJA_INIT_ERROR;
			return TSJA_INIT_RET_ERR_SHUTDOWN;
		}
		return TSJA_INIT_RET_RUNNING;

	case TSJA_INIT_SWITCH_ON:
		if (status == TSJA_SW_SWITCHED_ON)
		{
			motor->ctrl.word = TSJA_CW_ENABLE_OPERATION;
			state_tick = HAL_GetTick();
			init_state = TSJA_INIT_ENABLE_OPERATION;
		}
		else if (HAL_GetTick() - state_tick > TSJA_INIT_TIMEOUT_MS)
		{
			init_state = TSJA_INIT_ERROR;
			return TSJA_INIT_RET_ERR_SWITCH_ON;
		}
		return TSJA_INIT_RET_RUNNING;

	case TSJA_INIT_ENABLE_OPERATION:
		if (status == TSJA_SW_OPERATION_ENABLE)
		{
			init_state = TSJA_INIT_DONE;
			return TSJA_INIT_RET_OK;
		}
		else if (HAL_GetTick() - state_tick > TSJA_INIT_TIMEOUT_MS)
		{
			init_state = TSJA_INIT_ERROR;
			return TSJA_INIT_RET_ERR_ENABLE_OPERATION;
		}
		return TSJA_INIT_RET_RUNNING;

	case TSJA_INIT_DONE:
		return TSJA_INIT_RET_OK;

	case TSJA_INIT_ERROR:
	default:
		return TSJA_INIT_RET_ERR_ENABLE_OPERATION;
	}
}

void tsja_motor_ctrl_send(motor_t *motor)
{
	write_od_u16(0x2001, 0x01, (uint16_t)motor->ctrl.word);
	write_od_i8(0x2001, 0x03, (int8_t)motor->ctrl.mode);
	write_od_i16(0x2001, 0x05, (int16_t)motor->ctrl.tor_set);

	write_od_i32(0x2001, 0x06, (int32_t)motor->ctrl.vel_set);
	write_od_i32(0x2001, 0x07, (int32_t)motor->ctrl.pos_set);

	CO_TPDOsendRequest(&CO->TPDO[0]);
	CO_TPDOsendRequest(&CO->TPDO[1]);
}

void tsja_motor_fbdata(motor_t *motor)
{
	motor->para.status_word = read_od_u16(0x2001, 0x02);
	motor->para.mode = read_od_i8(0x2001, 0x04);

	motor->para.tor = read_od_i16(0x2001, 0x08);
	motor->para.vel = read_od_i32(0x2001, 0x09);
	motor->para.pos = read_od_i32(0x2001, 0x0A);
}

/// @brief //////////////////////////

void open_axdr_tpdo(axdr_motor_t *axdr_motor)
{
	uint32_t val1 = TSJA_SLAVE_TPDO1_COB(axdr_motor->id);
	uint32_t val2 = TSJA_SLAVE_TPDO2_COB(axdr_motor->id);
	uint32_t val3 = TSJA_SLAVE_TPDO3_COB(axdr_motor->id);

	write_sdo(axdr_motor->id, 0x1800, 0x01, (uint8_t *)&val1, 4);
	write_sdo(axdr_motor->id, 0x1801, 0x01, (uint8_t *)&val2, 4);
	write_sdo(axdr_motor->id, 0x1802, 0x01, (uint8_t *)&val3, 4);
}

void axdr_motor_ctrl_send(axdr_motor_t *axdr_motor)
{
	write_od_i8(0x2002, 0x01, (int8_t)axdr_motor->axdr_ctrl.mode);
	write_od_i16(0x2002, 0x06, (int16_t)axdr_motor->axdr_ctrl.tor_set);
	write_od_i32(0x2003, 0x01, (int32_t)axdr_motor->axdr_Iq_set);
	write_od_i32(0x2002, 0x05, (int32_t)axdr_motor->axdr_ctrl.vel_set);
	write_od_i32(0x2002, 0x04, (int32_t)axdr_motor->axdr_ctrl.pos_set);
	write_od_i16(0x2004, 0x01, (int16_t)axdr_motor->axdr_kp_imp);
	write_od_i16(0x2004, 0x02, (int16_t)axdr_motor->axdr_kd_imp);
	write_od_i16(0x2004, 0x03, (int16_t)axdr_motor->axdr_qe_imp);
	write_od_i16(0x2004, 0x04, (int16_t)axdr_motor->axdr_kd_emd);
	CO_TPDOsendRequest(&CO->TPDO[2]);
	CO_TPDOsendRequest(&CO->TPDO[3]);
	CO_TPDOsendRequest(&CO->TPDO[4]);
}

void axdr_motor_fbdata(axdr_motor_t *axdr_motor)
{
	axdr_motor->axdr_para.mode = read_od_i8(0x2002, 0x02);
	axdr_motor->axdr_mode_done = read_od_i8(0x2002, 0x03);
	axdr_motor->axdr_para.tor = read_od_i16(0x2002, 0x09);
	axdr_motor->axdr_para.vel = read_od_i32(0x2002, 0x08);
	axdr_motor->axdr_para.pos = read_od_i32(0x2002, 0x07);
	axdr_motor->axdr_Iq = read_od_i32(0x2003, 0x02);
	axdr_motor->axdr_Ia = read_od_i16(0x2003, 0x05);
	axdr_motor->axdr_Ib = read_od_i16(0x2003, 0x06);
	axdr_motor->axdr_pwm_emd = read_od_i16(0x2004, 0x05);
	axdr_motor->axdr_freq_emd = read_od_i16(0x2004, 0x06);
}

/// @brief //////////////////////////
static uint32_t nullvar32 = 0;
static uint16_t nullvar16 = 0;
static uint8_t nullvar8 = 0;

uint32_t *get_od_32addr(uint16_t mainindex, uint16_t subindex)
{
	switch (mainindex)
	{
	case 0x2001:
		switch (subindex)
		{
		case 0x06:
			return (uint32_t *)&(OD_RAM.x2001_M1.vel_set);
		case 0x07:
			return (uint32_t *)&(OD_RAM.x2001_M1.pos_set);
		case 0x09:
			return (uint32_t *)&(OD_RAM.x2001_M1.vel);
		case 0x0A:
			return (uint32_t *)&(OD_RAM.x2001_M1.pos);
		default:
			return &nullvar32;
		}
	case 0x2002:
		switch (subindex)
		{
		case 0x04:
			return (uint32_t *)&(OD_RAM.x2002_M2.axdr_pos_set);
		case 0x05:
			return (uint32_t *)&(OD_RAM.x2002_M2.axdr_vel_set);
		case 0x07:
			return (uint32_t *)&(OD_RAM.x2002_M2.axdr_pos);
		case 0x08:
			return (uint32_t *)&(OD_RAM.x2002_M2.axdr_vel);
		default:
			return &nullvar32;
		}
	case 0x2003:
		switch (subindex)
		{
		case 0x01:
			return (uint32_t *)&(OD_RAM.x2003_M2_current_para.axdr_Iq_des);
		case 0x02:
			return (uint32_t *)&(OD_RAM.x2003_M2_current_para.axdr_Iq);
		case 0x03:
			return (uint32_t *)&(OD_RAM.x2003_M2_current_para.axdr_Id_des);
		case 0x04:
			return (uint32_t *)&(OD_RAM.x2003_M2_current_para.axdr_Id);
		default:
			return &nullvar32;
		}
	default:
		return &nullvar32;
	}
}

uint16_t *get_od_16addr(uint16_t mainindex, uint16_t subindex)
{

	switch (mainindex)
	{
	case 0x2001:
		switch (subindex)
		{
		case 0x01:
			return (uint16_t *)&OD_RAM.x2001_M1.control_word;
		case 0x02:
			return (uint16_t *)&OD_RAM.x2001_M1.status_word;
		case 0x05:
			return (uint16_t *)&(OD_RAM.x2001_M1.tor_set);
		case 0x08:
			return (uint16_t *)&(OD_RAM.x2001_M1.tor);
		default:
			return &nullvar16;
		}
	case 0x2002:
		switch (subindex)
		{
		case 0x06:
			return (uint16_t *)&(OD_RAM.x2002_M2.axdr_tor_set);
		case 0x09:
			return (uint16_t *)&(OD_RAM.x2002_M2.axdr_tor);
		default:
			return &nullvar16;
		}
	case 0x2003:
		switch (subindex)
		{
		case 0x05:
			return (uint16_t *)&(OD_RAM.x2003_M2_current_para.axdr_Ia);
		case 0x06:
			return (uint16_t *)&(OD_RAM.x2003_M2_current_para.axdr_Ib);
		case 0x07:
			return (uint16_t *)&(OD_RAM.x2003_M2_current_para.axdr_Ic);
		default:
			return &nullvar16;
		}
	case 0x2004:
		switch (subindex)
		{
		case 0x01:
			return (uint16_t *)&(OD_RAM.x2004_M2_imp.axdr_Kp_imp);
		case 0x02:
			return (uint16_t *)&(OD_RAM.x2004_M2_imp.axdr_Kd_imp);
		case 0x03:
			return (uint16_t *)&(OD_RAM.x2004_M2_imp.axdr_qe_imp);
		case 0x04:
			return (uint16_t *)&(OD_RAM.x2004_M2_imp.axdr_Kd_emd);
		case 0x05:
			return (uint16_t *)&(OD_RAM.x2004_M2_imp.axdr_pwm_emd);
		case 0x06:
			return (uint16_t *)&(OD_RAM.x2004_M2_imp.axdr_freq_emd);
		default:
			return &nullvar16;
		}
	default:
		return &nullvar16;
	}
}

uint8_t *get_od_8addr(uint16_t mainindex, uint16_t subindex)
{
	switch (mainindex)
	{
	case 0x2001:
		switch (subindex)
		{
		case 0x03:
			return (uint8_t *)&OD_RAM.x2001_M1.mode_operation;
		case 0x04:
			return (uint8_t *)&OD_RAM.x2001_M1.mode_operation_display;
		default:
			return &nullvar8;
		}
	case 0x2002:
		switch (subindex)
		{
		case 0x01:
			return (uint8_t *)&OD_RAM.x2002_M2.axdr_mode_operation;
		case 0x02:
			return (uint8_t *)&OD_RAM.x2002_M2.axdr_mode_operation_display;
		case 0x03:
			return (uint8_t *)&OD_RAM.x2002_M2.axdr_mode_done;
		default:
			return &nullvar8;
		}
	default:
		return &nullvar8;
	}
}

void write_od_i32(uint16_t mainindex, uint16_t subindex, int32_t data)
{
	data_u d;

	d.i32_val = data;
	*get_od_32addr(mainindex, subindex) = d.u32_val;
}

int32_t read_od_i32(uint16_t mainindex, uint16_t subindex)
{
	data_u d;

	d.u32_val = *get_od_32addr(mainindex, subindex);
	return d.i32_val;
}

void write_od_u32(uint16_t mainindex, uint16_t subindex, uint32_t data)
{
	data_u d;

	d.u32_val = data;
	*get_od_32addr(mainindex, subindex) = d.u32_val;
}

uint32_t read_od_u32(uint16_t mainindex, uint16_t subindex)
{
	data_u d;

	d.u32_val = *get_od_32addr(mainindex, subindex);
	return d.u32_val;
}

void write_od_u16(uint16_t mainindex, uint16_t subindex, uint16_t data)
{
	data_u16 d;

	d.u16_val = data;
	*get_od_16addr(mainindex, subindex) = d.u16_val;
}

uint16_t read_od_u16(uint16_t mainindex, uint16_t subindex)
{
	data_u16 d;

	d.u16_val = *get_od_16addr(mainindex, subindex);
	return d.u16_val;
}

void write_od_i16(uint16_t mainindex, uint16_t subindex, int16_t data)
{
	data_u16 d;

	d.i16_val = data;
	*get_od_16addr(mainindex, subindex) = d.u16_val;
}

int16_t read_od_i16(uint16_t mainindex, uint16_t subindex)
{
	data_u16 d;

	d.u16_val = *get_od_16addr(mainindex, subindex);
	return d.i16_val;
}

void write_od_i8(uint16_t mainindex, uint16_t subindex, int8_t data)
{
	data_u8 d;

	d.i8_val = data;
	*get_od_8addr(mainindex, subindex) = d.u8_val;
}

int8_t read_od_i8(uint16_t mainindex, uint16_t subindex)
{
	data_u8 d;

	d.u8_val = *get_od_8addr(mainindex, subindex);
	return d.i8_val;
}

void write_od_u8(uint16_t mainindex, uint16_t subindex, uint8_t data)
{
	data_u8 d;

	d.u8_val = data;
	*get_od_8addr(mainindex, subindex) = d.u8_val;
}

uint8_t read_od_u8(uint16_t mainindex, uint16_t subindex)
{
	data_u8 d;

	d.u8_val = *get_od_8addr(mainindex, subindex);
	return d.u8_val;
}