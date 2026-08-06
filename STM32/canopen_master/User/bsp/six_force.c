#include "six_force.h"

six_force_node_t sf_node;
data_f2 sf_data_temp;
six_force_node_t sf_offset;

volatile uint8_t sf_update_flag = 0;

void six_force_bsp_init(void)
{
    sf_node.f_x = 0.0f;
    sf_node.f_y = 0.0f;
    sf_node.f_z = 0.0f;
    sf_node.m_x = 0.0f;
    sf_node.m_y = 0.0f;
    sf_node.m_z = 0.0f;
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void six_force_fdcan_rx_callback(void)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint16_t rec_id;

    if (HAL_FDCAN_GetRxMessage(
            &hfdcan2,
            FDCAN_RX_FIFO0,
            &rxHeader,
            (uint8_t *)&sf_data_temp) != HAL_OK)
    {
        return;
    }

    rec_id = rxHeader.Identifier;

    if (rxHeader.DataLength != FDCAN_DLC_BYTES_8)
    {
        return;
    }

    if (rec_id == CTXIDL_ID1)
    {
        sf_node.f_x = sf_data_temp.f_val[0];
        sf_node.f_y = sf_data_temp.f_val[1];
    }
    else if (rec_id == CTXIDL_ID2)
    {
        sf_node.f_z = sf_data_temp.f_val[0];
        sf_node.m_x = sf_data_temp.f_val[1];
    }
    else if (rec_id == CTXIDL_ID3)
    {
        sf_node.m_y = sf_data_temp.f_val[0];
        sf_node.m_z = sf_data_temp.f_val[1];
    }
    else
    {
        return;
    }

    // 表示收到新的力数据
    sf_update_flag = 1;
}

uint8_t six_force_can_send(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier = 0x700;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = len;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, data) != HAL_OK)
    {
        // 发送失败处理
        return 1;
    }
    return 0;
}

void six_force_require(void)
{
    uint8_t data[8] = {1};
    six_force_can_send(&hfdcan2, data, 1);
}

void six_force_loop_start(void)
{
    uint8_t data[8] = {2};
    six_force_can_send(&hfdcan2, data, 1);
}

void six_force_loop_stop(void)
{
    uint8_t data[8] = {0};
    six_force_can_send(&hfdcan2, data, 1);
}

uint8_t six_force_read(void)
{
    six_force_require();
    uint32_t timeout = 0;

    while (sf_update_flag == 0)
    {
        timeout++;
        if (timeout > 100000)
        {
            return 0;
        }
    }

    sf_update_flag = 0;
    return 1;
}

void six_force_zero(void)
{
    float sum[6] = {0.0f};

    uint16_t count = 0;

    while (count < 500)
    {
        if (six_force_read())
        {
            sum[0] += sf_node.f_x;
            sum[1] += sf_node.f_y;
            sum[2] += sf_node.f_z;
            sum[3] += sf_node.m_x;
            sum[4] += sf_node.m_y;
            sum[5] += sf_node.m_z;

            count++;
        }
    }

    sf_offset.f_x = sum[0] / 500.0f;
    sf_offset.f_y = sum[1] / 500.0f;
    sf_offset.f_z = sum[2] / 500.0f;

    sf_offset.m_x = sum[3] / 500.0f;
    sf_offset.m_y = sum[4] / 500.0f;
    sf_offset.m_z = sum[5] / 500.0f;
}