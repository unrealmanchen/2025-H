#ifndef MOTOR
#define MOTOR

#include "typedef.h"
#include "can.h"
#include "pid.h"
#include "filter.h"
enum ID_t{
    pitch=0,
    yaw
};

typedef struct __attribute__((packed)) {
    float angle;      
    int speed;      
    float current;    
} Motor_data_t;

typedef struct __attribute__((packed)) {
    uint8_t status;  
    uint8_t reserved;    
    int16_t current;   
    int16_t speed;  
    uint16_t angle;      
     
} MotorFeedback_t;

typedef volatile struct motor_t
{
    volatile pid_t pid_deg,pid_w;
    volatile MotorFeedback_t raw_data;
    volatile Motor_data_t data;
    Filter filter_angle,filter_w;
} Motor_Ins;

void QD4310_Current(uint8_t ID,float data);
void QD4310_Deg(uint8_t ID,float data);
void QD4310_CMD(uint8_t ID,int CMD );
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void QD4310_control_deg(enum ID_t ID,float deg);
void QD4310_control_w(enum ID_t ID,float w);
float QD4310_set_deg(float target_angle, float current_angle);



void motor_init(void);
extern Motor_Ins motor_yaw,motor_pitch;
#endif
