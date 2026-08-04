#ifndef _ELMO_FUNC_H_
#define _ELMO_FUNC_H_

#define ELMO_MODE_Profile_Position 0x01
#define ELMO_MODE_Profile_Velocity 0x03
#define ELMO_MODE_Profile_Torque 0x04

#define ELMO_CONTROL_SHUTDOWN 0x0006
#define ELMO_CONTROL_SWITCH_ON 0x0007
#define ELMO_CONTROL_ENABLE_OPERATION 0x000F

#define ELMO_STATUS_MASK 0x03FF

#define ELMO_STATUS_READY 0x0221
#define ELMO_STATUS_SWITCHED_ON 0x0233
#define ELMO_STATUS_OPERATION_ENABLE 0x0237

#define ELMO_INIT_TIMEOUT_MS 2000

typedef enum
{
    ELMO_INIT_IDLE = 0,
    ELMO_INIT_SHUTDOWN,
    ELMO_INIT_SWITCH_ON,
    ELMO_INIT_ENABLE_OPERATION,
    ELMO_INIT_DONE,
    ELMO_INIT_ERROR
} elmo_init_state_t;

typedef enum
{
    ELMO_INIT_RET_RUNNING = 0,
    ELMO_INIT_RET_OK,
    ELMO_INIT_RET_ERR_SHUTDOWN,
    ELMO_INIT_RET_ERR_SWITCH_ON,
    ELMO_INIT_RET_ERR_ENABLE_OPERATION

} elmo_init_ret_t;

#define PDO_DISABLE_BIT 0x80000000UL
#define ELMO_SLAVE_RPDO1_COB(node_id) (0x200UL + (node_id))
#define ELMO_SLAVE_RPDO2_COB(node_id) (0x300UL + (node_id))
#define ELMO_SLAVE_RPDO3_COB(node_id) (0x400UL + (node_id))
#define RPDO_TRANS_TYPE_ASYNC 254U

#define ELMO_SLAVE_TPDO1_COB(node_id) (0x180UL + (node_id))
#define ELMO_SLAVE_TPDO2_COB(node_id) (0x280UL + (node_id))
#define ELMO_SLAVE_TPDO3_COB(node_id) (0x380UL + (node_id))
#define TPDO_TRANS_TYPE_ASYNC 254U
#define TPDO_EVENT_TIMER_MS 5U

#endif