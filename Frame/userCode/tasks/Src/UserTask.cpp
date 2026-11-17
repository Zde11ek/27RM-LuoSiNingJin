#include "UserTask.h"

/* ================= 1. 夹爪与手腕双舵机配置 ================= */
#define CLAW_OPEN_ANGLE      40.0f   // 夹爪张开角度
#define CLAW_CLOSE_ANGLE     5.0f   // 夹爪深压夹紧角度 

#define WRIST_HORIZ_ANGLE    140.0f  // 手腕开机默认基准角度
#define WRIST_MIN_DEG        30.0f   // 手腕最小安全角度限位 (保留)
#define WRIST_MAX_DEG        250.0f  // 手腕最大安全角度限位 (保留)(实际发现这个限位比机械限位范围还要大，但是不想改了也没啥必要改)

SERVO_INIT_T Claw_Servo_Init = {
    .servoType = POSITION_180,
    .servoID = SERVO_ID_1,
    .firstAngle = CLAW_OPEN_ANGLE,   // 开机默认张开 (65°)
    .angleLimit_Min = 0.0f,
    .angleLimit_Max = 270.0f
};
Servo ClawServo(&Claw_Servo_Init);

SERVO_INIT_T Wrist_Servo_Init = {
    .servoType = POSITION_180,
    .servoID = SERVO_ID_2,
    .firstAngle = WRIST_HORIZ_ANGLE, // 开机默认基准位 (140°)
    .angleLimit_Min = 0.0f,
    .angleLimit_Max = 270.0f
};
Servo WristServo(&Wrist_Servo_Init);

float wristTargetDeg = WRIST_HORIZ_ANGLE;
float wristRampDeg   = WRIST_HORIZ_ANGLE;
volatile float wristCurrentDeg = WRIST_HORIZ_ANGLE;

/* ================= 2. 大臂步进电机配置 (500Hz 高频防共振 + 800 细分降速 50%) ================= */
// SERVO_ID_3 (PE13) 发脉冲 PUL，SERVO_ID_4 (PE14) 发方向 DIR
#define STEPPER_PUL_PORT        GPIOE
#define STEPPER_PUL_PIN         GPIO_PIN_13
#define STEPPER_DIR_PORT        GPIOE
#define STEPPER_DIR_PIN         GPIO_PIN_14

// 请将 TB6600 驱动器细分拨码 (SW4~SW6) 拨到 800 (4细分)：既将速度减慢 50%，又彻底消除 250Hz 低频共振失步
#define STEPPER_PULSES_PER_REV  800.0f
#define BIGARM_REDUCTION_RATIO  12.0f
#define BIGARM_STEPS_PER_DEG    ((STEPPER_PULSES_PER_REV * BIGARM_REDUCTION_RATIO) / 360.0f)

// 方向反转开关(原来代码转反了)
#define BIGARM_DIR_INVERT       1

volatile float  bigArmCurrentDeg  = 0.0f; // 仅用于 Watch 1 观察累计角度，无软限位拦截
volatile int8_t bigArmMoveDir     = 0;    // -1: SB上档方向, 0: 停止, 1: SB下档方向

int32_t bigArmCurrentStep = 0;
uint8_t stepperStopFlag   = 1;
uint8_t pulPinLevel       = 0;
uint8_t armStopFlag       = 1;

/* ================= 3. 外部控制接口 ================= */
void Claw_SetOpen(bool isOpen) {
    ClawServo.SetTargetAngle(isOpen ? CLAW_OPEN_ANGLE : CLAW_CLOSE_ANGLE);
}

void Wrist_AdjustAngle(float deltaDeg) {
    armStopFlag = 0;
    wristTargetDeg += deltaDeg;
    INRANGE(wristTargetDeg, WRIST_MIN_DEG, WRIST_MAX_DEG); // 严格保留手腕软限位
}

void Wrist_SetAngle(float targetDeg) {
    armStopFlag = 0;
    wristTargetDeg = targetDeg;
    INRANGE(wristTargetDeg, WRIST_MIN_DEG, WRIST_MAX_DEG); // 严格保留手腕软限位
}

void Arm_SetBigArmMove(int8_t dir) {
    armStopFlag     = 0;
    stepperStopFlag = 0;
    bigArmMoveDir   = dir;
}

/* ================= 4. 初始化与周期任务 ================= */
void UserInit() {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = STEPPER_PUL_PIN | STEPPER_DIR_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(STEPPER_PUL_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, GPIO_PIN_RESET);
}

void UserStop() {
    armStopFlag = 1;
    ClawServo.stop();
    WristServo.stop();
    wristTargetDeg = wristRampDeg;

    stepperStopFlag = 1;
    bigArmMoveDir   = 0;
    pulPinLevel     = 0;
    HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_RESET);
}

void UserHandle() {
    // 1. 大臂步进电机无限位脉冲发生器 (500Hz 高频平稳输出，配合驱动器 800 细分实现不丢步降速 50%)
    if (stepperStopFlag == 0) {
        if (bigArmMoveDir == 0) {
            pulPinLevel = 0;
            HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_RESET);
        } else {
            bool moveIncrease = (bigArmMoveDir > 0);
            GPIO_PinState dirState;
            if (BIGARM_DIR_INVERT == 0) {
                dirState = moveIncrease ? GPIO_PIN_SET : GPIO_PIN_RESET;
            } else {
                dirState = moveIncrease ? GPIO_PIN_RESET : GPIO_PIN_SET;
            }
            HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, dirState);

            if (pulPinLevel == 1) {
                pulPinLevel = 0;
                HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_RESET);
            } else {
                pulPinLevel = 1;
                HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_SET);
                if (moveIncrease) bigArmCurrentStep++;
                else              bigArmCurrentStep--;

                bigArmCurrentDeg = (float)bigArmCurrentStep / BIGARM_STEPS_PER_DEG;
            }
        }
    }

    // 2. 手腕斜坡平滑与软限位保护 (80°/s 平滑过渡，限位 30° ~ 250°)
    INRANGE(wristTargetDeg, WRIST_MIN_DEG, WRIST_MAX_DEG);
    float wristDiff = wristTargetDeg - wristRampDeg;
    const float wristMaxStep = 0.08f;
    if (wristDiff > wristMaxStep)       wristRampDeg += wristMaxStep;
    else if (wristDiff < -wristMaxStep) wristRampDeg -= wristMaxStep;
    else                                wristRampDeg = wristTargetDeg;

    wristCurrentDeg = wristRampDeg;

    // 3. 非急停状态下更新手腕舵机目标角度
    if (armStopFlag == 0) {
        WristServo.SetTargetAngle(wristRampDeg);
    }

    ClawServo.Handle();
    WristServo.Handle();
}
