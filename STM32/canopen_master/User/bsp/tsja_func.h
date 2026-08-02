#ifndef _TSJA_FUNC_H_
#define _TSJA_FUNC_H_
#define TSJA_MODE_CSP 0x0008 // 0x6060 = 0x08 周期同步转矩模式
#define TSJA_MODE_CSV 0x0009 // 0x6060 = 0x09 周期同步速度模式
#define TSJA_MODE_CST 0x000A // 0x6060 = 0x0A，周期同步力矩模式

#define TSJA_CW_SHUTDOWN 0x0006
#define TSJA_CW_SWITCH_ON 0x0007
#define TSJA_CW_ENABLE_OPERATION 0x000F

#define TSJA_SW_MASK 0x03FF

#define TSJA_SW_READY 0x0221
#define TSJA_SW_SWITCHED_ON 0x0233
#define TSJA_SW_OPERATION_ENABLE 0x0237

#define TSJA_INIT_TIMEOUT_MS 2000

typedef enum
{
    TSJA_INIT_IDLE = 0,
    TSJA_INIT_SHUTDOWN,
    TSJA_INIT_SWITCH_ON,
    TSJA_INIT_ENABLE_OPERATION,
    TSJA_INIT_DONE,
    TSJA_INIT_ERROR
} tsja_init_state_t;

typedef enum
{
    TSJA_INIT_RET_RUNNING = 0,
    TSJA_INIT_RET_OK,
    TSJA_INIT_RET_ERR_SHUTDOWN,
    TSJA_INIT_RET_ERR_SWITCH_ON,
    TSJA_INIT_RET_ERR_ENABLE_OPERATION
} tsja_init_ret_t;

#define PDO_DISABLE_BIT 0x80000000UL

#define TSJA_SLAVE_RPDO1_COB(node_id) (0x200UL + (node_id))
#define TSJA_SLAVE_RPDO2_COB(node_id) (0x300UL + (node_id))
#define TSJA_SLAVE_RPDO3_COB(node_id) (0x400UL + (node_id))

#define RPDO_TRANS_TYPE_ASYNC 254U

#define TSJA_SLAVE_TPDO1_COB(node_id) (0x180UL + (node_id))
#define TSJA_SLAVE_TPDO2_COB(node_id) (0x280UL + (node_id))
#define TSJA_SLAVE_TPDO3_COB(node_id) (0x380UL + (node_id))

#define TPDO_TRANS_TYPE_ASYNC 254U
#define TPDO_EVENT_TIMER_MS 10U

#endif