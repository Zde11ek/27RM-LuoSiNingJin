#ifndef RM_FRAME_C_USERTASK_H
#define RM_FRAME_C_USERTASK_H

#include "main.h"
#include "Device.h"
#include "Servo.h"

// Keil窗口实时监控变量
extern volatile float  bigArmCurrentDeg;  // 大臂开机后累计转过的相对角度
extern volatile int8_t bigArmMoveDir;     // 大臂当前运动方向
extern volatile float  wristCurrentDeg;   // 手腕当前平滑角度

// 机械臂三轴控制接口 (大臂 + 手腕 + 夹爪)
void Claw_SetOpen(bool isOpen);           // 夹爪开合 (true:张开, false:夹紧)
void Wrist_AdjustAngle(float deltaDeg);   // 手腕增量无级旋转 (带软限位)
void Wrist_SetAngle(float targetDeg);     // 手腕指定角度 (带软限位)
void Arm_SetBigArmMove(int8_t dir);       // 大臂无限位连续转动

void UserStop();
void UserHandle();
void UserInit();

#endif // RM_FRAME_C_USERTASK_H
