//
// 天地飞 RF206S PWM 接收机驱动（校内赛框架移植）
//
#include "PWMRemote.h"

/* ---- 静态成员定义 ---- */
volatile uint16_t PWMRemote::pulseCH1 = PWM_MID_US;
volatile uint16_t PWMRemote::pulseCH2 = PWM_MID_US;
volatile uint32_t PWMRemote::tickCH1  = 0;
volatile uint32_t PWMRemote::tickCH2  = 0;

/**
 * @brief 初始化 TIM2/TIM3 输入捕获并启动中断
 * @note  应在设备初始化阶段（Device.cpp 的初始化流程里）调用一次
 */
void PWMRemote::init()
{
    MX_PWMRemote_TIM_Init();
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}

/**
 * @brief 输入捕获回调：双沿切换测脉宽
 *
 * 只配上升沿捕获时，读到的计数值是"周期"而不是"脉宽"，
 * 所以这里在回调里翻转极性：
 *   上升沿记时刻 -> 切下降沿 -> 下降沿算脉宽 -> 切回上升沿
 */
void PWMRemote::ITHandle(TIM_HandleTypeDef *htim)
{
    if (htim == nullptr) return;

    if (htim->Instance == TIM2)
    {
        static uint32_t riseCH1 = 0;
        static uint8_t  waitFall1 = 0;

        if (waitFall1 == 0u)
        {
            riseCH1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            waitFall1 = 1u;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            uint32_t fall  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            uint32_t width = (fall >= riseCH1) ? (fall - riseCH1)
                                               : (0xFFFFu - riseCH1 + fall);
            waitFall1 = 0u;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

            if ((width >= PWM_MIN_VALID_US) && (width <= PWM_MAX_VALID_US))
            {
                pulseCH1 = (uint16_t)width;
                tickCH1  = HAL_GetTick();
            }
        }
    }
    else if (htim->Instance == TIM3)
    {
        static uint32_t riseCH2 = 0;
        static uint8_t  waitFall2 = 0;

        if (waitFall2 == 0u)
        {
            riseCH2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            waitFall2 = 1u;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            uint32_t fall  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            uint32_t width = (fall >= riseCH2) ? (fall - riseCH2)
                                               : (0xFFFFu - riseCH2 + fall);
            waitFall2 = 0u;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

            if ((width >= PWM_MIN_VALID_US) && (width <= PWM_MAX_VALID_US))
            {
                pulseCH2 = (uint16_t)width;
                tickCH2  = HAL_GetTick();
            }
        }
    }
}

/**
 * @brief 脉宽 -> 归一化轴量（-1.0 ~ +1.0），含死区
 */
float PWMRemote::toAxis(uint16_t pulseUs)
{
    int32_t delta = (int32_t)pulseUs - (int32_t)PWM_MID_US;

    /* 死区：摇杆在中位附近视为 0，防止漂移 */
    if ((delta > -(int32_t)PWM_DEADZONE_US) && (delta < (int32_t)PWM_DEADZONE_US))
    {
        return 0.0f;
    }
    if (delta >  (int32_t)PWM_HALF_RANGE) { delta =  (int32_t)PWM_HALF_RANGE; }
    if (delta < -(int32_t)PWM_HALF_RANGE) { delta = -(int32_t)PWM_HALF_RANGE; }

    return (float)delta / PWM_HALF_RANGE;
}

/**
 * @brief 前后方向（CH2），-1.0 ~ +1.0
 */
float PWMRemote::getFB()
{
    if (!isAlive()) return 0.0f;
    return toAxis(pulseCH2);
}

/**
 * @brief 左右横移（CH1），-1.0 ~ +1.0
 */
float PWMRemote::getLR()
{
    if (!isAlive()) return 0.0f;
    return toAxis(pulseCH1);
}

/**
 * @brief 遥控在线判定：两路都在超时窗口内才算在线
 * @note  接收机断电 / 信号丢失时，脉宽不再更新，这里会返回 false，
 *        控制任务据此触发急停
 */
bool PWMRemote::isAlive()
{
    uint32_t now = HAL_GetTick();
    return ((now - tickCH1) < PWM_TIMEOUT_MS) &&
           ((now - tickCH2) < PWM_TIMEOUT_MS);
}

/* ---- HAL 回调转发 ---- */
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    PWMRemote::ITHandle(htim);
}

/* ---- 供 C 文件调用的接口 ---- */
extern "C" void PWMRemote_Init(void)
{
    PWMRemote::init();
}

extern "C" uint8_t PWMRemote_IsAlive(void)
{
    return PWMRemote::isAlive() ? 1u : 0u;
}
