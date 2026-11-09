/**
 ******************************************************************************
 * @file    imu_calib.c
 * @brief   ICM42688 零漂校准函数
 * @note    基于原 BMI088 Calibrate_MPU_Offset 逻辑重写,
 *          使用已有的 bsp_IcmGetRawData_array / bsp_IcmGetTemperature / DWT
 ******************************************************************************
 */

#include "imu_calib.h"
#include "icm42688.h"
#include "bsp_dwt.h"
#include <math.h>
#include <stdio.h>   // printf 用于 LOGERROR / LOGWARNING

/* ---------------- 校准参数 ----------------- */
#define CALI_TIMES      12000u      // 单次校准采样数
#define CALI_TIMEOUT_S  0.0f      // 超时时间 (秒)
#define CALI_SAMPLE_DT  0.0005f    // 采样间隔 (秒)

/* 运动检测阈值 */
#define GNORM_DIFF_MAX    3.0f     // 重力模长抖动阈值 (m/s²)
#define GYRO_DIFF_MAX     0.1f    // 陀螺抖动阈值 (rad/s, ≈2.86°/s)
#define GYRO_OFFSET_MAX   0.1f    // 陀螺零偏异常阈值 (rad/s)
#define GNORM_ERR_MAX     0.7f     // |gNorm - 9.8| 允许误差 (m/s²)

/* 离线参数 (校准失败时使用) */
#define GxOFFSET_FALLBACK  0.00555596082f
#define GyOFFSET_FALLBACK  -0.00321971043f
#define GzOFFSET_FALLBACK  -0.00998730678f
#define gNORM_FALLBACK     9.70048046f


/**
 * @brief  ICM42688 静止零漂校准
 * @param  icm  IMU_Data_t 指针, 校准结果写入 GyroOffset / gNorm / AccelScale / TempWhenCali
 * @note   须在传感器完全静止时调用, 否则会反复重试直到超时
 */

 float GNorm_T=0;float gyroDiff[3];
int Calibrate_ICM42688_Offset(IMU_Data_t *icm)
{
    static float startTime;
    static uint16_t caliCount;          // 记录重试次数 (调试用)
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;
    float gNormDiff;
    

    startTime = DWT_GetTimeline_s();

    do
    {
        /* ---------- 超时保护 ---------- */
        if (DWT_GetTimeline_s() - startTime > CALI_TIMEOUT_S)
        {
            icm->GyroOffset[0] = GxOFFSET_FALLBACK;
            icm->GyroOffset[1] = GyOFFSET_FALLBACK;
            icm->GyroOffset[2] = GzOFFSET_FALLBACK;
            icm->gNorm          = gNORM_FALLBACK;
            icm->TempWhenCali   = 40.0f;
            
            icm->AccelScale = 9.81f / icm->gNorm;
            return 1;
            break;
        }

        DWT_Delay(0.005f);
        GNorm_T = icm->gNorm;
        /* 清零累加器 */
        icm->gNorm = 0.0f;
        icm->GyroOffset[0] = 0.0f;
        icm->GyroOffset[1] = 0.0f;
        icm->GyroOffset[2] = 0.0f;

        /* ---------- 主采样循环 ---------- */
        for (uint16_t i = 0; i < CALI_TIMES; ++i)
        {
            float data[6];
            bsp_IcmGetRawData_array(data);   // [0]=ax, [1]=ay, [2]=az, [3]=gx, [4]=gy, [5]=gz

            icm->Accel[0] = data[0];
            icm->Accel[1] = data[1];
            icm->Accel[2] = data[2];
            icm->Gyro[0]  = data[3];
            icm->Gyro[1]  = data[4];
            icm->Gyro[2]  = data[5];

            /* 累加 gNorm */
            gNormTemp = sqrtf(icm->Accel[0] * icm->Accel[0] +
                              icm->Accel[1] * icm->Accel[1] +
                              icm->Accel[2] * icm->Accel[2]);
            icm->gNorm += gNormTemp;

            /* 累加陀螺零偏 */
            icm->GyroOffset[0] += icm->Gyro[0];
            icm->GyroOffset[1] += icm->Gyro[1];
            icm->GyroOffset[2] += icm->Gyro[2];

            /* 首次采样: 初始化最大最小值 */
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                gyroMax[0] = icm->Gyro[0];
                gyroMin[0] = icm->Gyro[0];
                gyroMax[1] = icm->Gyro[1];
                gyroMin[1] = icm->Gyro[1];
                gyroMax[2] = icm->Gyro[2];
                gyroMin[2] = icm->Gyro[2];
            }
            else
            {
                if (gNormTemp > gNormMax) gNormMax = gNormTemp;
                if (gNormTemp < gNormMin) gNormMin = gNormTemp;

                for (uint8_t j = 0; j < 3; ++j)
                {
                    if (icm->Gyro[j] > gyroMax[j]) gyroMax[j] = icm->Gyro[j];
                    if (icm->Gyro[j] < gyroMin[j]) gyroMin[j] = icm->Gyro[j];
                }
            }

            /* 计算峰峰值 */
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; ++j)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];

            /* 运动检测: 任一轴抖动过大则中断本轮 */
            if (gNormDiff   > GNORM_DIFF_MAX ||
                gyroDiff[0] > GYRO_DIFF_MAX  ||
                gyroDiff[1] > GYRO_DIFF_MAX  ||
                gyroDiff[2] > GYRO_DIFF_MAX)
            {
                //printf("[icm42688] calibration was interrupted (motion detected)\r\n");
                break;
            }

            DWT_Delay(CALI_SAMPLE_DT);
        }

        /* 求均值 */
        icm->gNorm /= (float)CALI_TIMES;
        for (uint8_t i = 0; i < 3; ++i)
            icm->GyroOffset[i] /= (float)CALI_TIMES;

        /* 读取温度 */
        {
            int16_t rawTemp;
            bsp_IcmGetTemperature(&rawTemp);
            icm->TempWhenCali = (float)rawTemp;
        }

        caliCount++;

    } while (gNormDiff                    > GNORM_DIFF_MAX  ||
             fabsf(icm->gNorm - 9.8f)     > GNORM_ERR_MAX   ||
             gyroDiff[0]                  > GYRO_DIFF_MAX   ||
             gyroDiff[1]                  > GYRO_DIFF_MAX   ||
             gyroDiff[2]                  > GYRO_DIFF_MAX   ||
             fabsf(icm->GyroOffset[0])    > GYRO_OFFSET_MAX ||
             fabsf(icm->GyroOffset[1])    > GYRO_OFFSET_MAX ||
             fabsf(icm->GyroOffset[2])    > GYRO_OFFSET_MAX);

    icm->AccelScale = 9.81f / icm->gNorm;
    return 0;
    /* 计算加速度比例因子, 用于将 gNorm 归一化到 9.81 */
    
}
