//
// Created by LEGION on 2021/10/10.
//

#include "RemoteControl.h"

RC_Info_t RemoteControl::rcInfo;
RC_ctrl_t RemoteControl::rc_ctrl;

uint8_t RemoteControl::sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void RemoteControl::init() {
    //enable the DMA transfer for the receiver request
    //使能 DMA 串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    //disable DMA
    //失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区 1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(sbus_rx_buf[0]);
    //memory buffer 2
    //内存缓冲区 2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(sbus_rx_buf[1]);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
    //enable double memory buffer
    //使能双缓冲区
    CLEAR_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CIRC);
    //enable DMA
    //使能 DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}


void RemoteControl::sbus_to_rc(const volatile uint8_t *sbus_buf) {
    // 1. 解析 11 位通道数据
    rc_ctrl.rc.ch[0] = (sbus_buf[1] | sbus_buf[2] << 8) & 0x07ff;       // CH1 (J1)
    rc_ctrl.rc.ch[1] = (sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07ff;  // CH2 (J3)
    rc_ctrl.rc.ch[2] = (sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07ff; // CH3 (J2)
    rc_ctrl.rc.ch[3] = (sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07ff;  // CH4 (J4)
    rc_ctrl.rc.s[0] = (sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07ff;   // CH5 (SB)
    rc_ctrl.rc.s[1] = (sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07ff;   // CH6 (SC)
    
    rc_ctrl.rc.optional[0] = (sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07ff;   // CH7
    rc_ctrl.rc.optional[1] = (sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07ff;  // CH8
    
    // 2. 减去中值偏移量 1024
    int16_t ch0 = rc_ctrl.rc.ch[0] - RC_CH_VALUE_OFFSET;
    int16_t ch1 = rc_ctrl.rc.ch[1] - RC_CH_VALUE_OFFSET;
    int16_t ch2 = rc_ctrl.rc.ch[2] - RC_CH_VALUE_OFFSET;
    int16_t ch3 = rc_ctrl.rc.ch[3] - RC_CH_VALUE_OFFSET;

    // --- 核心修复：加入死区过滤，防止底盘狂转 ---
    if(ch0 > -15 && ch0 < 15) ch0 = 0;
    if(ch1 > -15 && ch1 < 15) ch1 = 0;
    if(ch2 > -15 && ch2 < 15) ch2 = 0;
    if(ch3 > -15 && ch3 < 15) ch3 = 0;

    // 3. 映射到标准浮点数
    rcInfo.right_rol = ch0 / 671.0f; 
    rcInfo.left_col  = ch1 / 671.0f; 
    rcInfo.right_col = ch2 / 671.0f; 
    rcInfo.left_rol  = ch3 / 671.0f; 
    // 阈值以 1000 为界（天地飞两档开关通常在 300~1700 之间跳变）
rcInfo.switch_SA = (rc_ctrl.rc.optional[0] < 1000); // 向上拨数值小，设为 true
rcInfo.switch_SD = (rc_ctrl.rc.optional[1] < 1000);
    
    // 4. 修复拨杆极性反转问题（天地飞拨杆向上为低数值，向下为高数值）
    if (rc_ctrl.rc.s[0] < 500) rcInfo.sLeft = UP_POS;
    else if (rc_ctrl.rc.s[0] > 1500) rcInfo.sLeft = DOWN_POS;
    else rcInfo.sLeft = MID_POS;

    if (rc_ctrl.rc.s[1] < 500) rcInfo.sRight = UP_POS;
    else if (rc_ctrl.rc.s[1] > 1500) rcInfo.sRight = DOWN_POS;
    else rcInfo.sRight = MID_POS;

    rcInfo.optional[0] = rc_ctrl.rc.optional[0];
    rcInfo.optional[1] = rc_ctrl.rc.optional[1];
}
void RemoteControl::ITHandle() {
    if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    } else if (USART3->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (sbus_rx_buf[0][23]  == 0) {
                HAL_IWDG_Refresh(&hiwdg);
            }
            if (this_time_rx_len == RC_FRAME_LENGTH) {
                RemoteControl::sbus_to_rc(RemoteControl::sbus_rx_buf[0]);
            }
        } else {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (sbus_rx_buf[1][23]  == 0) {
                HAL_IWDG_Refresh(&hiwdg);
            }
            if (this_time_rx_len == RC_FRAME_LENGTH) {
                //处理遥控器数据
                RemoteControl::sbus_to_rc(RemoteControl::sbus_rx_buf[1]);
            }
        }
    }
}
/**
  * @brief This function handles USART3 global interrupt.
  */

void USART3_IRQHandler(void)
{
    RemoteControl::ITHandle();
}
