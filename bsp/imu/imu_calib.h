/**
 ******************************************************************************
 * @file    imu_calib.h
 * @brief   ICM42688 零漂校准 (兼容原 BMI088 Calibrate_MPU_Offset 接口)
 * @note    使用已有的 bsp_IcmGetRawData_array / bsp_IcmGetTemperature / DWT 函数
 ******************************************************************************
 */
#ifndef __IMU_CALIB_H
#define __IMU_CALIB_H

#include <stdint.h>

/* IMU数据结构体 (兼容原 BMI088 IMU_Data_t) */
typedef struct
{
    float Accel[3];          // 加速度 (m/s²)
    float Gyro[3];           // 角速度 (rad/s)
    float GyroOffset[3];     // 陀螺零偏 (rad/s)
    float gNorm;             // 重力模长
    float AccelScale;        // 加速度比例因子
    float TempWhenCali;      // 校准时的温度
} IMU_Data_t;

int Calibrate_ICM42688_Offset(IMU_Data_t *icm);

#endif /* __IMU_CALIB_H */
