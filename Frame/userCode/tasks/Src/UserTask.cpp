#include "UserTask.h"

/* ================= 1. 夹爪与手腕双舵机配置 (已锁定 40.0° / 5.0°) ================= */
#define CLAW_OPEN_ANGLE           40.0f   // 夹爪张开角度 (已确认)
#define CLAW_CLOSE_ANGLE          5.0f    // 夹爪闭合角度 (已确认)

#define WRIST_HORIZ_ANGLE         140.0f  // 手腕开机默认基准角度
#define WRIST_MIN_DEG             30.0f   // 手腕最小安全角度限位
#define WRIST_MAX_DEG             250.0f  // 手腕最大安全角度限位

SERVO_INIT_T Claw_Servo_Init = {
    .servoType = POSITION_180,
    .servoID = SERVO_ID_1,
    .firstAngle = CLAW_OPEN_ANGLE,   // 开机默认张开 (40.0°)
    .angleLimit_Min = 0.0f,
    .angleLimit_Max = 270.0f
};
Servo ClawServo(&Claw_Servo_Init);

SERVO_INIT_T Wrist_Servo_Init = {
    .servoType = POSITION_180,
    .servoID = SERVO_ID_2,
    .firstAngle = WRIST_HORIZ_ANGLE, // 开机默认基准位 (140.0°)
    .angleLimit_Min = 0.0f,
    .angleLimit_Max = 270.0f
};
Servo WristServo(&Wrist_Servo_Init);

float wristTargetDeg = WRIST_HORIZ_ANGLE;
float wristRampDeg   = WRIST_HORIZ_ANGLE;
volatile float wristCurrentDeg = WRIST_HORIZ_ANGLE;

/* ================= 2. 大臂步进电机配置 (800细分 + S型加减速 + 变向消隙缓冲) ================= */
#define STEPPER_PUL_PORT          GPIOE
#define STEPPER_PUL_PIN           GPIO_PIN_13
#define STEPPER_DIR_PORT          GPIOE
#define STEPPER_DIR_PIN           GPIO_PIN_14

#define STEPPER_PULSES_PER_REV    800.0f  // 驱动器保持 800 细分 (SW1~6: ON OFF OFF OFF ON OFF)
#define BIGARM_REDUCTION_RATIO    12.0f
#define BIGARM_STEPS_PER_DEG      ((STEPPER_PULSES_PER_REV * BIGARM_REDUCTION_RATIO) / 360.0f)

#define BIGARM_DIR_INVERT         1       // 1: 方向已反转

// 【调速与斜坡核心参数】
#define STEPPER_MAX_FREQ          625.0f  // 最高匀速脉冲频率 (Hz)(支持 100~1000Hz)
#define STEPPER_START_FREQ        50.0f   // 起步超低频脉冲 (Hz)，消除起步瞬间惯性冲击
#define STEPPER_ACCEL_MS          400.0f  // S型加速总时间 (ms)：起步柔和托起矿石
#define STEPPER_DECEL_MS          80.0f   // 回中减速总时间 (ms)：快速刹停不拖泥带水
#define STEPPER_REVERSE_PAUSE_MS  100      // 直接变向时的零速消隙停顿时间 (ms)，消除减速箱背隙砸齿

#define STEPPER_ACCEL_STEP        (1.0f / STEPPER_ACCEL_MS)
#define STEPPER_DECEL_STEP        (1.0f / STEPPER_DECEL_MS)

volatile float  bigArmCurrentDeg  = 0.0f;
volatile int8_t bigArmMoveDir     = 0;

float    stepperRampT       = 0.0f;   // 归一化 S 型斜坡进度 [0.0f ~ 1.0f]
float    stepperCurrFreq    = 0.0f;   // 当前实时脉冲频率 (Hz)
int8_t   stepperActualDir   = 0;      // 当前实际输出方向 (-1:上, 0:静止, 1:下)
uint16_t stepperPauseMs     = 0;      // 变向零速缓冲倒计时 (ms)
float    stepperPhaseAccum  = 0.0f;   // 完整脉冲相位累加器
int32_t  bigArmCurrentStep  = 0;
uint8_t  stepperStopFlag    = 1;
uint8_t  armStopFlag        = 1;

// STM32F407 (168MHz) 微秒级短延时，满足 TB6600 硬件建立与脉宽时序
static inline void Stepper_DelayUs(uint32_t us) {
    volatile uint32_t count = us * 42;
    while (count--) {
        __NOP();
    }
}

/* ================= 3. 外部控制接口 ================= */
void Claw_SetOpen(bool isOpen) {
    ClawServo.SetTargetAngle(isOpen ? CLAW_OPEN_ANGLE : CLAW_CLOSE_ANGLE);
}

void Wrist_AdjustAngle(float deltaDeg) {
    armStopFlag = 0;
    wristTargetDeg += deltaDeg;
    INRANGE(wristTargetDeg, WRIST_MIN_DEG, WRIST_MAX_DEG);
}

void Wrist_SetAngle(float targetDeg) {
    armStopFlag = 0;
    wristTargetDeg = targetDeg;
    INRANGE(wristTargetDeg, WRIST_MIN_DEG, WRIST_MAX_DEG);
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
    // 急停保持手腕当前角度不断流，防止回中跳变
    wristTargetDeg = wristRampDeg;

    // 步进电机急停：立即清零斜坡、停顿计时与脉冲状态
    stepperStopFlag   = 1;
    bigArmMoveDir     = 0;
    stepperActualDir  = 0;
    stepperRampT      = 0.0f;
    stepperCurrFreq   = 0.0f;
    stepperPauseMs    = 0;
    stepperPhaseAccum = 0.0f;
    HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_RESET);
}

void UserHandle() {
    // 1. 大臂步进电机 S 型加减速斜坡 + 变向缓冲状态机 (1kHz 周期执行)
    if (stepperStopFlag == 0) {
        if (stepperPauseMs > 0) {
            // 状态 A：处于直接变向后的零速消隙缓冲期，锁死电机等待惯性与背隙平息
            stepperPauseMs--;
            stepperRampT      = 0.0f;
            stepperPhaseAccum = 0.0f;
        } else if (bigArmMoveDir == 0) {
            // 状态 B：SB 回中，快速 S 型减速刹停，保证点动微调精准对位
            if (stepperRampT > STEPPER_DECEL_STEP) {
                stepperRampT -= STEPPER_DECEL_STEP;
            } else {
                stepperRampT      = 0.0f;
                stepperActualDir  = 0;
                stepperPhaseAccum = 0.0f;
                HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_RESET);
            }
        } else {
            // 状态 C：SB 拨向上升或下降
            if (stepperActualDir != 0 && stepperActualDir != bigArmMoveDir) {
                // 检测到直接反向拨杆：先快速刹停当前方向，归零后触发消隙停顿
                if (stepperRampT > STEPPER_DECEL_STEP) {
                    stepperRampT -= STEPPER_DECEL_STEP;
                } else {
                    stepperRampT      = 0.0f;
                    stepperActualDir  = 0;
                    stepperPhaseAccum = 0.0f;
                    stepperPauseMs    = STEPPER_REVERSE_PAUSE_MS;
                    HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_RESET);
                }
            } else {
                // 静止起步或同向继续运行：S 型平滑加速
                stepperActualDir = bigArmMoveDir;
                stepperRampT += STEPPER_ACCEL_STEP;
                if (stepperRampT > 1.0f) {
                    stepperRampT = 1.0f;
                }
            }
        }

        // 计算当前 S 型曲线实时频率：s(t) = 3t^2 - 2t^3
        if (stepperActualDir != 0 && stepperRampT > 0.0f) {
            float s = stepperRampT * stepperRampT * (3.0f - 2.0f * stepperRampT);
            stepperCurrFreq = STEPPER_START_FREQ + (STEPPER_MAX_FREQ - STEPPER_START_FREQ) * s;
        } else {
            stepperCurrFreq = 0.0f;
        }

        // 若当前频率大于 0，输出完整窄脉冲
        if (stepperCurrFreq > 0.0f && stepperActualDir != 0) {
            bool moveIncrease = (stepperActualDir > 0);
            GPIO_PinState dirState;
            if (BIGARM_DIR_INVERT == 0) {
                dirState = moveIncrease ? GPIO_PIN_SET : GPIO_PIN_RESET;
            } else {
                dirState = moveIncrease ? GPIO_PIN_RESET : GPIO_PIN_SET;
            }
            HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, dirState);

            // 每 1ms 累加 (freq / 1000) 个完整脉冲周期，满 1.0 输出一个完整高+低脉冲
            stepperPhaseAccum += (stepperCurrFreq * 0.001f);
            if (stepperPhaseAccum >= 1.0f) {
                stepperPhaseAccum -= 1.0f;

                // 满足 TB6600 DIR 提前 PUL 至少 5us 的建立时间要求
                Stepper_DelayUs(5);
                HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_SET);
                Stepper_DelayUs(6);  // 高电平脉宽 6us (大于 TB6600 2.2us 最低要求)
                HAL_GPIO_WritePin(STEPPER_PUL_PORT, STEPPER_PUL_PIN, GPIO_PIN_RESET);

                if (moveIncrease) bigArmCurrentStep++;
                else              bigArmCurrentStep--;

                bigArmCurrentDeg = (float)bigArmCurrentStep / BIGARM_STEPS_PER_DEG;
            }
        }
    }

    // 2. 手腕斜坡平滑与软限位保护 (30° ~ 250°)
    INRANGE(wristTargetDeg, WRIST_MIN_DEG, WRIST_MAX_DEG);
    float wristDiff = wristTargetDeg - wristRampDeg;
    const float wristMaxStep = 0.05f;
    if (wristDiff > wristMaxStep)       wristRampDeg += wristMaxStep;
    else if (wristDiff < -wristMaxStep) wristRampDeg -= wristMaxStep;
    else                                wristRampDeg = wristTargetDeg;

    wristCurrentDeg = wristRampDeg;

    // 3. 持续输出手腕目标角度
    WristServo.SetTargetAngle(wristRampDeg);

    ClawServo.Handle();
    WristServo.Handle();
}
