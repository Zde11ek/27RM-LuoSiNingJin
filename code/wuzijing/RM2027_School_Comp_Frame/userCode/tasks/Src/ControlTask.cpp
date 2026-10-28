//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"
#include "PWMRemote.h"

/* 底盘最大线速度（m/s）
 * M3508 减速比 19、轮径 0.1525m，理论满载约 3.5~3.8 m/s。
 * 首次试车建议先用小值（如 1.0），确认四轮方向正确后再逐步加大。 */
#define CHASSIS_MAX_FB_SPEED   2.5f
#define CHASSIS_MAX_LR_SPEED   2.5f

void CtrlHandle(){
    /* 失控保护：接收机断电 / 信号丢失 / 未接接收机 -> 全部急停
     * 遥控器侧也请把通道2（油门）失控值设为 1500us，软硬件双保险 */
    if (!PWMRemote::isAlive()){
        ChassisStop();
        UserStop();
        return;
    }

    /* 天地飞 RF206S：
     *   CH1 -> 左右横移   CH2 -> 前后油门
     *   CH3 未接，旋转固定为 0；接上后把第三路捕获值映射为旋转速度即可 */
    float fbV = PWMRemote::getFB() * CHASSIS_MAX_FB_SPEED;   // 前后
    float lrV = PWMRemote::getLR() * CHASSIS_MAX_LR_SPEED;   // 横移
    float rtV = 0.0f;                                        // 旋转（暂不用）

    ChassisSetVelocity(fbV, lrV, rtV);
}
