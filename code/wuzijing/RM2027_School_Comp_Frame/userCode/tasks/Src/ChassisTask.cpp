//
// Created by LEGION on 2021/10/4.
//
#include "ChassisTask.h"

PID_Regulator_t pidRegulator = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 60,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 16384,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 16384 //3508电机输出电流上限，可以调小，勿调大
};

MOTOR_INIT_t chassisMotorInit = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 19.0f,
        .ctrlType = SPEED_Single,
};
Motor CMFL(MOTOR_ID_1,&chassisMotorInit);//定义左前轮电机
Motor CMFR(MOTOR_ID_2,&chassisMotorInit);//定义右前轮电机
Motor CMBL(MOTOR_ID_3,&chassisMotorInit);//定义左后轮电机
Motor CMBR(MOTOR_ID_4,&chassisMotorInit);//定义右后轮电机

uint8_t ChassisStopFlag = 1;
float FBVelocity,LRVelocity,RTVelocity;

void ChassisStart(){

}
/**
 * @brief 底盘任务的处理函数，定时执行
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) in Device.cpp
 */
void ChassisHandle() {
    if(ChassisStopFlag == 0) {
        WheelsSpeedCalc(FBVelocity, LRVelocity, RTVelocity);
    }
    CMFL.Handle();
    CMFR.Handle();
    CMBL.Handle();
    CMBR.Handle();
}
/**
 * @brief 用于控制任务控制底盘速度
 * @param _fbV 底盘前后方向速度
 * @param _lrV 底盘左右方向速度
 * @param _rtV 底盘旋转速度
 */
void ChassisSetVelocity(float _fbV,float _lrV,float _rtV){
    ChassisStopFlag = 0;
    FBVelocity = _fbV;
    LRVelocity = _lrV;
    RTVelocity = _rtV;
}
/**
 * @brief 执行急停模式的底盘任务处理
 */
void ChassisStop(){
    ChassisStopFlag = 1;
    CMFL.Stop();
    CMFR.Stop();
    CMBL.Stop();
    CMBR.Stop();
}

void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity) {
    float CMFLSpeed, CMFRSpeed, CMBLSpeed, CMBRSpeed;

    rtVelocity = RPM2RADpS(rtVelocity);

    //计算四个轮子线速度，单位：m/s
    /**
     * @brief 此处四句代码需要结合底盘的三个速度，计算处四个轮子的位置对应的线速度。
     * @param fbVelocity,lrVelocity,rtVelocity
     * @return CMFLSpeed CMFRSpeed CMBLSpeed CMBRSpeed
     */
    /**
     * 麦克纳姆轮（X 型安装）逆运动学：由底盘三速度反解四轮线速度
     *   约定：fb+ 前进，lr+ 右移，rt+ 逆时针
     *   旋转分量 = 角速度(rad/s) x (半轮距 + 半轴距)
     *
     * 若实车方向不符，按以下顺序排查：
     *   1) 前后反了  -> fbVelocity 整体取反
     *   2) 横移反了  -> lrVelocity 整体取反
     *   3) 旋转反了  -> rotateComp 整体取反
     *   4) 单个轮反了 -> 调换该电机任意两根相线
     */
    const float halfTrack  = TRACK_WIDTH / 2.0f;   // 半轮距 m
    const float halfBase   = WHEEL_BASE  / 2.0f;   // 半轴距 m
    const float rotateComp = rtVelocity * (halfTrack + halfBase);

    CMFLSpeed = fbVelocity - lrVelocity - rotateComp;  // 左前
    CMFRSpeed = fbVelocity + lrVelocity + rotateComp;  // 右前
    CMBLSpeed = fbVelocity + lrVelocity - rotateComp;  // 左后
    CMBRSpeed = fbVelocity - lrVelocity + rotateComp;  // 右后

    //计算四个轮子角速度，单位：rad/s
    CMFLSpeed = CMFLSpeed /(WHEEL_DIAMETER/2.0f);
    CMFRSpeed = CMFRSpeed /(WHEEL_DIAMETER/2.0f);
    CMBLSpeed = CMBLSpeed /(WHEEL_DIAMETER/2.0f);
    CMBRSpeed = CMBRSpeed /(WHEEL_DIAMETER/2.0f);
    //控制底盘电机转速
    CMFL.SetTargetSpeed(RADpS2RPM(CMFLSpeed));
    CMFR.SetTargetSpeed(RADpS2RPM(CMFRSpeed));
    CMBL.SetTargetSpeed(RADpS2RPM(CMBLSpeed));
    CMBR.SetTargetSpeed(RADpS2RPM(CMBRSpeed));
}
