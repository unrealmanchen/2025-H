#include "can.h"
#include "motor.h"
#include <string.h>
#include "filter.h"
#include <math.h>
#include "pid.h"

float QD4310_set_deg(float target_angle, float current_angle);
uint8_t datas[3]; //?

Motor_Ins motor_yaw, motor_pitch;
void motor_init()
{

    PID_init(&motor_yaw.pid_deg, 10, 0.001, 100, 0, 950, 50, 1000);
    PID_init(&motor_yaw.pid_w, 0.012, 0, 0, 0, 2, 1, 0.3);
    motor_yaw.filter_angle = LowPassFilter_Create_A(0.7);
    motor_yaw.filter_w = LowPassFilter_Create_A(0.7);

    PID_init(&motor_pitch.pid_deg, 10, 0.001, 0, 0, 950, 50, 1000);
    PID_init(&motor_pitch.pid_w, 0.0025, 0, 0, 0, 1, 1, 0.2);
    motor_pitch.filter_angle = LowPassFilter_Create_A(0.7);
    motor_pitch.filter_w = LowPassFilter_Create_A(0.7);
}

void QD4310_control_deg(enum ID_t ID, float deg)
{
    struct motor_t *motor_;
    float deg_ = deg;
    if (ID == 0)
    {
        motor_ = &motor_pitch;
        if (deg > 35.0f)
            deg_ = 35.0f;
        if (deg < -35.0f)
            deg_ = -35.0f;

        float deg_input = QD4310_set_deg(deg_ + 180.0f, motor_->data.angle);
        float w = PID_calc(&(motor_->pid_deg), deg_input, 0);
        fp32 current_data = PID_calc(&(motor_->pid_w), motor_->data.speed, w);
        QD4310_Current(0, current_data);

        // QD4310_Deg((int)pitch,deg_ + 180.0f);
        return;
    }
    else if (ID == 1)
    {
        motor_ = &motor_yaw;

        float deg_input = QD4310_set_deg(deg_, motor_->data.angle);
        float w = PID_calc(&(motor_->pid_deg), deg_input, 0);
        fp32 current_data = PID_calc(&(motor_->pid_w), motor_yaw.data.speed, w);
        QD4310_Current(1, current_data);
    }
    else
        Error_Handler();
}

void QD4310_control_w(enum ID_t ID, float w)
{
    struct motor_t *motor_;

    if (ID == 0)
        motor_ = &motor_pitch;
    else if (ID == 1)
        motor_ = &motor_yaw;
    else
        Error_Handler();
    fp32 current_data = PID_calc(&(motor_->pid_w), motor_->data.speed, w);
    QD4310_Current(ID, current_data);
}

float QD4310_set_deg(float target_angle, float current_angle)
{

    // 归一化到 [0, 360)
    while (target_angle >= 360.0f)
        target_angle -= 360.0f;
    while (target_angle < 0)
        target_angle += 360.0f;
    while (current_angle >= 360.0f)
        current_angle -= 360.0f;
    while (current_angle < 0)
        current_angle += 360.0f;

    float error = target_angle - current_angle;

    // 环绕处理
    if (error > 180.0f)
        error -= 360.0f;
    if (error < -180.0f)
        error += 360.0f;

    return error * -1.0f; // 返回 -180° ~ 180°
}

void QD4310_Current(uint8_t ID, float data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[3];
    uint32_t send_mail_box;
    tx_header.StdId = ID + 0x400;
    tx_header.ExtId = 0x400 + ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x03;
    tx_header.TransmitGlobalTime = DISABLE;
    tx_data[0] = 0x03;
    tx_data[1] = ((int16_t)(data / 10.0 * 32767.0)) & 0xFF;
    tx_data[2] = ((int16_t)(data / 10.0 * 32767.0)) >> 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
        ;
}

void QD4310_Deg(uint8_t ID, float data)
{
    float rad = data / 180.0f * 3.1415926f;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[3];
    uint32_t send_mail_box;
    tx_header.StdId = ID + 0x400;
    tx_header.ExtId = 0x400 + ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x03;
    tx_header.TransmitGlobalTime = DISABLE;
    tx_data[0] = 0x05;
    tx_data[1] = ((int16_t)(rad / (2.0f * 3.14f) * 65535.0f)) & 0xFF;
    tx_data[2] = ((int16_t)(rad / (2.0f * 3.14f) * 65535.0f)) >> 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
        ;
}

void QD4310_CMD(uint8_t ID, int CMD)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[3];
    uint32_t send_mail_box;
    tx_header.StdId = ID + 0x400;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x03;
    tx_data[0] = CMD;
    tx_data[1] = 0;
    tx_data[2] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}

void parseMotorFeedback(int ID, const uint8_t *data)
{
    Motor_Ins *Ins = NULL;
    if (ID == 0x501)
        Ins = &motor_yaw;
    else if (ID == 0x500)
        Ins = &motor_pitch;
    else
        Error_Handler();

    memcpy(&(Ins->raw_data), data, sizeof(MotorFeedback_t));
    Ins->data.angle = (Ins->raw_data.angle / 65535.0f) * 2.0f * 3.14159265359f;
    Ins->data.current = (Ins->raw_data.current / 32767.0f) * 10.0f;
    Ins->data.speed = (Ins->raw_data.speed / 32767.0f) * 1000.0f;
    Ins->data.speed = LowPassFilter_Process(Ins->filter_w, Ins->data.speed);
    Ins->data.angle = LowPassFilter_Process(Ins->filter_angle, Ins->data.angle);
    Ins->data.angle *= 180.0f / 3.1415926f;

    return;
}

uint8_t rx_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan->Instance == CAN1)
    {
        parseMotorFeedback(rx_header.StdId, rx_data);
    }
}


