//
// 天地飞 RF206S PWM 接收机驱动（校内赛框架移植）
//
// 硬件链路：接收机 PWM -> STM32 输入捕获 -> 归一化轴量 -> 底盘解算
//   CH1 横移 : PA0 -> TIM2_CH1
//   CH2 前后 : PA6 -> TIM3_CH1
//   PWM 范围 1000~2000us，1500us 为中位
//
// 用法（对标 RemoteControl::rcInfo）：
//   PWMRemote::init();              // 在设备初始化阶段调用一次
//   float fb = PWMRemote::getFB();  // 前后   -1.0 ~ +1.0
//   float lr = PWMRemote::getLR();  // 横移   -1.0 ~ +1.0
//   PWMRemote::isAlive();           // 失控判定（超过 100ms 无有效脉宽返回 false）
//

#ifndef RM_FRAME_C_PWMREMOTE_H
#define RM_FRAME_C_PWMREMOTE_H

#include "Device.h"
#include "tim.h"

#ifdef __cplusplus

/* ---- 接收机 PWM 参数（可按实际遥控器微调）---- */
#define PWM_MID_US        1500u   /* 中位脉宽 */
#define PWM_HALF_RANGE    500.0f  /* 1000~2000 -> -1~+1 */
#define PWM_DEADZONE_US   40u     /* |脉宽-1500| < 40us 视为中位，防漂移 */
#define PWM_MIN_VALID_US  800u    /* 有效脉宽下限，滤掉毛刺 */
#define PWM_MAX_VALID_US  2200u   /* 有效脉宽上限 */
#define PWM_TIMEOUT_MS    100u    /* 超过 100ms 无有效脉宽 -> 判定失控 */

class PWMRemote {
public:
    /* 初始化 TIM2/TIM3 输入捕获并启动中断 */
    static void init();

    /* 输入捕获回调，由 HAL_TIM_IC_CaptureCallback 转发进来 */
    static void ITHandle(TIM_HandleTypeDef *htim);

    /* 捕获结果：脉宽(us) 与 最近一次更新的时间戳(ms) */
    static volatile uint16_t pulseCH1;
    static volatile uint16_t pulseCH2;
    static volatile uint32_t tickCH1;
    static volatile uint32_t tickCH2;

    /* 归一化轴量：-1.0 ~ +1.0，含死区；失控时返回 0 */
    static float getFB();    /* 前后   (CH2) */
    static float getLR();    /* 横移   (CH1) */

    /* 遥控在线判定：两路都在超时窗口内才返回 true */
    static bool isAlive();

private:
    static float toAxis(uint16_t pulseUs);
};

extern "C" {
#endif

/* 供 C 文件（如 stm32f4xx_it.c）可选调用 */
void PWMRemote_Init(void);
uint8_t PWMRemote_IsAlive(void);

#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_PWMREMOTE_H
