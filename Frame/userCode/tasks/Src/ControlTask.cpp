//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"

void CtrlHandle(){
    // 1. 最高优先级：右侧 SC 下三档急停
    if (RemoteControl::rcInfo.sRight == DOWN_POS){
        ChassisStop();
        UserStop();
        return;
    }

    // 2. 底盘移动控制：彻底解绑 SB，摇杆常态有效
    ChassisSetVelocity(RemoteControl::rcInfo.right_col * 2.0f,
                       RemoteControl::rcInfo.right_rol * 2.0f,
                       RemoteControl::rcInfo.left_rol  * 25.0f);

    // 3. 机械臂与夹爪逻辑（待底层电机驱动/舵机控制函数完成后填入）
    /*
    switch (RemoteControl::rcInfo.sLeft) { // SB 档位
        case UP_POS:   break; // 一级放矿
        case MID_POS:  break; // 二级放矿
        case DOWN_POS: break; // 三级放矿
    }
    if (RemoteControl::rcInfo.optional[0] < 1000) { // SA 拨杆
        // 夹爪闭合
    } else {
        // 夹爪张开
    }
    */
}

