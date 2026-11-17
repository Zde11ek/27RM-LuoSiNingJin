#include "ControlTask.h"
#include "UserTask.h"

// 底盘最大速度系数（若现场觉得车速偏快或偏慢，直接改这 3 个数即可）
#define CHASSIS_VX_GAIN      2.0f   // 前后平移速度系数 (右摇杆上下 right_col)
#define CHASSIS_VY_GAIN      2.0f   // 左右横移速度系数 (右摇杆左右 right_rol)
#define CHASSIS_VW_GAIN      18.0f   // 原地自转速度系数 (左摇杆左右 left_rol)
#define RC_DEADZONE          0.08f  // 摇杆防误触死区 (防止调手腕时误触发底盘旋转)

static inline float ApplyDeadzone(float val, float deadzone) {
    if (val > -deadzone && val < deadzone) return 0.0f;
    return val;
}

void CtrlHandle(){
    if (RemoteControl::rcInfo.sRight == DOWN_POS){
        // SC 最下档：全局安全急停（底盘断电、机械臂锁止）
        ChassisStop();
        UserStop();
    } else {
        // ================= 1. 麦轮底盘全向移动控制 =================
        float vx = ApplyDeadzone(RemoteControl::rcInfo.right_col, RC_DEADZONE) * CHASSIS_VX_GAIN;
        float vy = ApplyDeadzone(RemoteControl::rcInfo.right_rol, RC_DEADZONE) * CHASSIS_VY_GAIN;
        float vw = ApplyDeadzone(RemoteControl::rcInfo.left_rol,  RC_DEADZONE) * CHASSIS_VW_GAIN;
        ChassisSetVelocity(vx, vy, vw);

        // ================= 2. 夹爪开合控制 (SA 两档开关) =================
        if (RemoteControl::rcInfo.optional[0] < 1000) {
            Claw_SetOpen(false); // SA 上拨：深压夹紧矿石
        } else {
            Claw_SetOpen(true);  // SA 下拨：张开夹爪
        }

        // ================= 3. 大臂步进电机无限位控制 (SB 三档开关) =================
        if (RemoteControl::rcInfo.sLeft == UP_POS) {
            Arm_SetBigArmMove(-1); // SB 上档：向原角度减小方向平稳转动
        } else if (RemoteControl::rcInfo.sLeft == DOWN_POS) {
            Arm_SetBigArmMove(1);  // SB 下档：向原角度增大方向平稳转动
        } else {
            Arm_SetBigArmMove(0);  // SB 中档：立即停止并自锁
        }

        // ================= 4. 手腕无级俯仰控制 (左摇杆上下，保留 30°~250° 限位) =================
        float wristInput = ApplyDeadzone(RemoteControl::rcInfo.left_col, RC_DEADZONE);
        Wrist_AdjustAngle(wristInput * 0.08f);
    }
}
