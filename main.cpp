#include "mbed.h"
#include "math.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "rxDualshock4.hpp"
using namespace snct;

// 定数・設定
#define ENCODER_PPR   8192.0f
#define GEAR_RATIO    2.5f//: 1
#define TO_DEG (360.0f / (ENCODER_PPR * GEAR_RATIO))
#define STICK_DEADBAND 10.0f // スティックデッドバンド
#define UNIT_COUNT 4 // ユニット数
#define L 0.13f // ステアユニット中心間距離(縦)
#define W 0.13f // ステアユニット中心間距離(横)

// ステアユニット座標　　　　　　　　　　　　　//       [0]       [1]
const float pos_x[4] = {  L, -L, -L,  L }; //    　　　　　↑x
const float pos_y[4] = {  W,  W, -W, -W }; // 　　　　　y← O
                                           //       [3]       [2]
// 構造体
struct MotorFeedback {
    uint16_t encoder;
    int16_t speed_rpm;
    int16_t torque;
    uint8_t temp;
};

struct PID {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float out_min, out_max;
    float prev_meas;
};

// タイマーハンドル
TIM_HandleTypeDef htim1, htim2, htim3, htim8;

// 通信
UnbufferedSerial pc(USBTX, USBRX, 115200);
RxDualshock4::DS4 cont;
UnbufferedSerial fep(PC_10, PC_5, 38400);
RxDualshock4 tako(fep, &cont, RxController::GET_TYPE::polling);
CAN can1(PA_11, PA_12);
CAN can2(PB_12, PB_13);
const int MOTOR_CAN_ID_BASE = 0x200;
const int MOTOR_CAN_ID_BASE2 = 0x1FF;
const int MOTOR_FEEDBACK_ID_BASE = 0x201;

// グローバル
PID angle_pid[UNIT_COUNT];
PID motor_pid[UNIT_COUNT*2];
MotorFeedback motor_fb[UNIT_COUNT*2];
int16_t motor_cmd[UNIT_COUNT*2] = {0};
int16_t target_speed[UNIT_COUNT*2] = {0};
float target_angle[UNIT_COUNT] = {0.0f};
float angle_deg[UNIT_COUNT] = {0.0f};
float wheel_drive[UNIT_COUNT] = {0.0f};
float drive_target_filtered[UNIT_COUNT] = {0.0f};

// 関数プロトタイプ
float pid_calculate(PID &pid, float target, float measurement, float dt);
void set_robomaster_speed(int16_t cmd[UNIT_COUNT*2]);
void init_encoder_TIM1();
void init_encoder_TIM2();
void init_encoder_TIM3();
void init_encoder_TIM8();
void pc_printf(const char *format, ...);

// ====== メイン ======
int main() {
    Timer loop_timer;
    loop_timer.start();

    HAL_Init();

    init_encoder_TIM1();
    init_encoder_TIM2();
    init_encoder_TIM3();
    init_encoder_TIM8();

    can1.frequency(1000000);
    can2.frequency(1000000);

    uint16_t prev_count[4] = {
        (uint16_t)htim1.Instance->CNT,
        (uint16_t)htim2.Instance->CNT,
        (uint16_t)htim3.Instance->CNT,
        (uint16_t)htim8.Instance->CNT
    };

    // 速度PID
    // m2006
    for (int i = 0; i < UNIT_COUNT; i++) {
        motor_pid[i].kp = 0.4f;
        motor_pid[i].ki = 0.0f;
        motor_pid[i].kd = 0.0f;
        motor_pid[i].integral = 0;
        motor_pid[i].prev_error = 0;
        motor_pid[i].prev_meas = 0;
        motor_pid[i].out_min = -10000;
        motor_pid[i].out_max = 10000;
    }
    // m3508
    for (int i = UNIT_COUNT; i < UNIT_COUNT*2; i++) {
        motor_pid[i].kp = 0.3f;
        motor_pid[i].ki = 0.05f;
        motor_pid[i].kd = 0.0f;
        motor_pid[i].integral = 0;
        motor_pid[i].prev_error = 0;
        motor_pid[i].prev_meas = 0;
        motor_pid[i].out_min = -16000;
        motor_pid[i].out_max = 16000;
    }

    // 角度PID
    for (int i = 0; i < UNIT_COUNT; i++) {
        angle_pid[i].integral = 0;
        angle_pid[i].prev_error = 0;
        angle_pid[i].prev_meas = 0;
        angle_pid[i].out_min = -5000;
        angle_pid[i].out_max =  5000;
    }
        angle_pid[0].kp = 40.0f;
        angle_pid[0].ki = 0.0f;
        angle_pid[0].kd = 0.0f;

        angle_pid[1].kp = 60.0f;
        angle_pid[1].ki = 0.0f;
        angle_pid[1].kd = 0.0f;

        angle_pid[2].kp = 50.0f;
        angle_pid[2].ki = 0.0f;
        angle_pid[2].kd = 0.0f;

        angle_pid[3].kp = 30.0f;
        angle_pid[3].ki = 0.0f;
        angle_pid[3].kd = 0.0f;    

    // ループ開始
    while(1){
        // dt取得
        float dt = loop_timer.elapsed_time().count() / 1e3f;
        loop_timer.reset();
        if(dt < 0.001f) dt = 0.001f;  // min 1ms
        if(dt > 0.005f) dt = 0.005f;  // max 5ms

        // CANフィードバック受信
        CANMessage msg;
        while(can2.read(msg)){
            if(msg.id >= MOTOR_FEEDBACK_ID_BASE && msg.id < MOTOR_FEEDBACK_ID_BASE + (UNIT_COUNT*2)){
                int id = msg.id - MOTOR_FEEDBACK_ID_BASE;
                motor_fb[id].encoder   = (msg.data[0]<<8)|msg.data[1];
                motor_fb[id].speed_rpm = (msg.data[2]<<8)|msg.data[3];
                motor_fb[id].torque    = (msg.data[4]<<8)|msg.data[5];
                motor_fb[id].temp      = msg.data[6];
            }
        }

        // エンコーダ更新
        uint16_t cnts[4] = {
            (uint16_t)htim1.Instance->CNT,
            (uint16_t)htim2.Instance->CNT,
            (uint16_t)htim3.Instance->CNT,
            (uint16_t)htim8.Instance->CNT
        };

        for(int i=0;i<UNIT_COUNT;i++){
            int16_t delta = (int16_t)(cnts[i] - prev_count[i]);
            angle_deg[i] += delta * TO_DEG;
            if(angle_deg[i]>=360) angle_deg[i]-=360;
            if(angle_deg[i]<0) angle_deg[i]+=360;
            prev_count[i]=cnts[i];
        }

        // スティック取得
        float LX=0, LY=0, RX=0;
        float Vx=0, Vy=0, omega=0;

        if(tako.getDS4()==1){

            LX = cont.LX - 127.5f;
            LY = cont.LY - 127.5f;
            RX = cont.RX - 127.5f;

            if(fabs(LX) > STICK_DEADBAND || fabs(LY) > STICK_DEADBAND ){

                float LXc = -LX / 127.5f;
                float LYc = -LY / 127.5f;

                float mag = sqrtf(LXc*LXc + LYc*LYc);
                if(mag > 1.0f){
                    LXc /= mag;
                    LYc /= mag;
                }

                const float MAX_V = 4000.0f;
                Vx = LXc * MAX_V;
                Vy = LYc * MAX_V;
            }

            if(fabs(RX) > STICK_DEADBAND){
                float RXc = RX / 127.5f;
                const float MAX_R = 2000.0f;
                omega = RXc * MAX_R / sqrtf(L*L + W*W);
            }
        }

        // 各輪ベクトル合成
        for(int i=0;i<UNIT_COUNT;i++){

            float Vx_i = Vx + (-omega * pos_y[i]);
            float Vy_i = Vy + ( omega * pos_x[i]);

            float ang = atan2f(Vx_i, Vy_i) * 180.0f/M_PI;
            if(ang<0) ang+=360;

            float drive_mag = sqrtf(Vx_i*Vx_i + Vy_i*Vy_i);

            float err = ang - angle_deg[i];
            if(err>180) err-=360;
            if(err<-180) err+=360;

            // 有限回転処理
            bool invert_drive = false;
            if(fabs(err) > 90.0f){
                if(err > 0) err -= 180.0f;
                else err += 180.0f;
                invert_drive = true;
            }
            if(invert_drive) drive_mag = -drive_mag;

            target_angle[i] = angle_deg[i] + err;
            if(target_angle[i]>=360) target_angle[i]-=360;
            if(target_angle[i]<0) target_angle[i]+=360;

            wheel_drive[i] = drive_mag;

            // 角度PID
            float ff_steer  = err * angle_pid[i].kp;//pid_calculate(angle_pid[i], 0, -err, dt);

            // 安全上限
            const float MAX_STEER = 4000.0f;
            if(ff_steer>MAX_STEER) ff_steer=MAX_STEER;
            if(ff_steer<-MAX_STEER) ff_steer=-MAX_STEER;

            target_speed[i] = (int16_t)ff_steer;
        }

        // 台形
        const float MAX_ACCEL = 3000.0f;
        for(int i=0;i<UNIT_COUNT;i++){

            float diff = wheel_drive[i] - drive_target_filtered[i];
            float max_step = MAX_ACCEL * dt;
            if(diff > max_step)  diff = max_step;
            if(diff < -max_step) diff = -max_step;
            drive_target_filtered[i] += diff;

            target_speed[i+UNIT_COUNT] = (int16_t)drive_target_filtered[i];
        }

        // 速度PID
        for(int i=0;i<UNIT_COUNT*2;i++){
            float out = pid_calculate(motor_pid[i], (float)target_speed[i], (float)motor_fb[i].speed_rpm, dt);
            motor_cmd[i] = (int16_t)out;
        }

        // CAN送信
        set_robomaster_speed(motor_cmd);

        // 角度デバッグ出力（重いのでコメントアウト推奨）
        pc_printf("t1=%3d t2=%3d t3=%3d t4=%3d\r\n",
                  (int)target_angle[0],
                  (int)target_angle[1],
                  (int)target_angle[2],
                  (int)target_angle[3]);
        pc_printf("r1=%3d r2=%3d r3=%3d r4=%3d\r\n",
                  (int)angle_deg[0],
                  (int)angle_deg[1],
                  (int)angle_deg[2],
                  (int)angle_deg[3]);

        ThisThread::sleep_for(1ms);
    }
}
// ====================

// ===== 関数コーナー =====
// エンコーダ
void init_encoder_TIM1() {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_Encoder_InitTypeDef encoderConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFF;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC1Filter = 2;
    encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC2Filter = 2;

    HAL_TIM_Encoder_Init(&htim1, &encoderConfig);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

    pc_printf("TIM1 encoder init (PA8,PA9)\r\n");
}

void init_encoder_TIM2() {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA5 → TIM2_CH1
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB9 → TIM2_CH2
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM_Encoder_InitTypeDef encoderConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC1Filter = 2;
    encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC2Filter = 2;

    HAL_TIM_Encoder_Init(&htim2, &encoderConfig);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    pc_printf("TIM2 encoder init (PA5,PB9)\r\n");
}

void init_encoder_TIM3() {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM_Encoder_InitTypeDef encoderConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC1Filter = 2;
    encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC2Filter = 2;

    HAL_TIM_Encoder_Init(&htim3, &encoderConfig);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    pc_printf("TIM3 encoder init (PB4,PB5)\r\n");
}

void init_encoder_TIM8() {
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    TIM_Encoder_InitTypeDef encoderConfig = {0};

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 0xFFFF;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;

    encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC1Filter = 5;

    encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC2Filter = 5;

    if (HAL_TIM_Encoder_Init(&htim8, &encoderConfig) != HAL_OK)
    {
        pc_printf("TIM8 init ERROR\r\n");
        return;
    }

    __HAL_TIM_SET_COUNTER(&htim8, 0);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    pc_printf("TIM8 encoder init (PC6, PC7)\r\n");
}

// PID
float pid_calculate(PID &pid, float target, float measurement, float dt)
{
    if (dt <= 0.0f) dt = 0.001f;

    // 測定値ローパス（ノイズ対策）
    const float alpha = 0.4f;
    float meas_f = alpha * measurement + (1.0f - alpha) * pid.prev_meas;

    // 誤差
    float error = target - meas_f;

    // 微分先行
    float derivative = -(meas_f - pid.prev_meas) / dt;

    // 積分
    float tentative_integral = pid.integral + error * dt;

    const float INTEGRAL_LIMIT = 10000.0f;
    if (tentative_integral > INTEGRAL_LIMIT)
        tentative_integral = INTEGRAL_LIMIT;
    if (tentative_integral < -INTEGRAL_LIMIT)
        tentative_integral = -INTEGRAL_LIMIT;

    // PID合成
    float output =
        pid.kp * error +
        pid.ki * tentative_integral +
        pid.kd * derivative;

    // 出力制限＋アンチワインドアップ
    if (output > pid.out_max)
        output = pid.out_max;
    else if (output < pid.out_min)
        output = pid.out_min;
    else
        pid.integral = tentative_integral;

    // 状態更新
    pid.prev_meas = meas_f;
    pid.prev_error = error;

    return output;
}

// CAN送信
void set_robomaster_speed(int16_t cmd[UNIT_COUNT*2]) {

    CANMessage msg;
    msg.len = 8;
    // 0x200 -> モーター0~3
    msg.id = MOTOR_CAN_ID_BASE;
    for (int i = 0; i < 4; i++) {
        uint16_t u = (uint16_t)cmd[i];
        msg.data[2*i]   = (u >> 8) & 0xFF;
        msg.data[2*i+1] = u & 0xFF;
    }

    if (!can2.write(msg)) {
        pc_printf("can write fail (id : 0x200)\r\n");
    }

    // 0x1FF -> モーター4~7
    msg.id = MOTOR_CAN_ID_BASE2;
    for (int i = 0; i < 4; i++) {
        uint16_t u = (uint16_t)cmd[i+4];
        msg.data[2*i]   = (u >> 8) & 0xFF;
        msg.data[2*i+1] = u & 0xFF;
    }

    if (!can2.write(msg)) {
        pc_printf("can write fail (id : 0x1FF)\r\n");
    }
}

// printf互換
void pc_printf(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    pc.write(buf, len);
}
// =======================