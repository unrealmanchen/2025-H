#include "icm42688.h"
#include "usart.h"
#include "spi.h"
#include <math.h>


static float accSensitivity   = (9.69293118f / 4096.0f);  
static float gyroSensitivity  = 16.4f;    //陀螺仪的最小分辨率


/*ICM42688使用的ms级延时函数，须由用户提供。*/
#define ICM42688DelayMs(_nms)  HAL_Delay(_nms)


uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    // 额外 dummy (复位状态机)

    HAL_SPI_TransmitReceive(&hspi3,&TxData,&Rxdata,1, 1000);       
 	return Rxdata;          		    //返回收到的数据		
}


/*******************************************************************************
* 名    称： Icm_Spi_ReadWriteNbytes
* 功    能： 使用SPI读写n个字节
* 入口参数： pBuffer: 写入的数组  len:写入数组的长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
static void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len)
{
    uint8_t i = 0;

    for(i = 0; i < len; i ++)
    {
		*pBuffer = SPI1_ReadWriteByte(*pBuffer);
        pBuffer++;
    }

}




/*******************************************************************************
* 名    称： icm42688_read_reg
* 功    能： 读取单个寄存器的值
* 入口参数： reg: 寄存器地址
* 出口参数： 当前寄存器地址的值
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page51.
*******************************************************************************/
static uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t regval = 0;

    GPIOA->BSRR = GPIO_PIN_15<< 16u; 
    reg |= 0x80;
    SPI1_ReadWriteByte(reg);
    regval = SPI1_ReadWriteByte(0xFF);
    GPIOA->BSRR = GPIO_PIN_15; 


    return regval;
}

/*******************************************************************************
* 名    称： icm42688_read_regs
* 功    能： 连续读取多个寄存器的值
* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
static void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{

    GPIOA->BSRR = GPIO_PIN_15<< 16u; 
    reg |= 0x80;
    
    /* 写入要读的寄存器地址 */
    SPI1_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    while(len)
	{
		*buf = SPI1_ReadWriteByte(0x00);
		len--;
		buf++;
	}
    GPIOA->BSRR = GPIO_PIN_15; 

}


/*******************************************************************************
* 名    称： icm42688_write_reg
* 功    能： 向单个寄存器写数据
* 入口参数： reg: 寄存器地址 value:数据
* 出口参数： 0
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{

     GPIOA->BSRR = GPIO_PIN_15<< 16u; 
    //SPI_IMU_CS=0;
    /* 写入要读的寄存器地址 */
    /* 写入要读的寄存器地址 */
    SPI1_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    SPI1_ReadWriteByte(value);
    GPIOA->BSRR = GPIO_PIN_15; 
    //SPI_IMU_CS=1;
    return 0;
}



float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch(Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 2000 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4000 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8000 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16000 / 32768.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch(Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
    return gyroSensitivity;
}

/*******************************************************************************
* 名    称： bsp_Icm42688RegCfg
* 功    能： Icm42688 寄存器配置
* 入口参数： 无
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
int8_t bsp_Icm42688RegCfg(void)
{
    uint8_t reg_val = 0;
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //设置bank 0区域寄存器
    icm42688_write_reg(ICM42688_DEVICE_CONFIG, 0x01); //软复位传感器
    ICM42688DelayMs(100);
    /* 读取 who am i 寄存器 */
    reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);
	//printf("reg_val:%d\n",reg_val);
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //设置bank 0区域寄存器
    icm42688_write_reg(ICM42688_DEVICE_CONFIG, 0x01); //软复位传感器
    ICM42688DelayMs(100);


    if(reg_val == ICM42688_ID)
    {
		// icm42688_write_reg(ICM42688_INT_CONFIG, 0x30);
        // icm42688_write_reg(ICM42688_INT_SOURCE0, 0x08); // 配置INT1 DRDY INT
		
        //bsp_Icm42688GetAres(AFS_4G);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        //reg_val = icm42688_read_reg(ICM42688_ACCEL_CONFIG0);//page74
        reg_val = (AFS_8G << 5);   //量程 ±8g
        reg_val |= (AODR_4000Hz);     //输出速率 100HZ
        icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

        //bsp_Icm42688GetGres(GFS_1000DPS);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        //reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG0);//page73
        reg_val = (GFS_2000DPS << 5);   //量程 ±2000dps
        reg_val |= (GODR_4000Hz);     //输出速率 1000HZ
        icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0); //读取PWR—MGMT0当前寄存器的值(page72)
        reg_val &= ~(1 << 5);//使能温度测量
        reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
        reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
        icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
        ICM42688DelayMs(1); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

        return 0;
    }
    return -1;
}
													 



int8_t bsp_Icm42688Init(void)
{
	//ICM426XX_Init();
    return(bsp_Icm42688RegCfg());

}


int8_t bsp_IcmGetTemperature(int16_t* pTemp)
{
    uint8_t buffer[2] = {0};

    icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);

    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
    return 0;
}

/*******************************************************************************
* 名    称： bsp_IcmGetAccelerometer
* 功    能： 读取Icm42688 加速度的值
* 入口参数： 三轴加速度的值
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);

    accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

    accData->x = (int16_t)(accData->x * accSensitivity);
    accData->y = (int16_t)(accData->y * accSensitivity);
    accData->z = (int16_t)(accData->z * accSensitivity);

    return 0;
}


int8_t bsp_IcmGetGyroscope(icm42688RawData_t* GyroData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 6);

    GyroData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    GyroData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    GyroData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);
    return 0;
}

void bsp_IcmGetRawData_array(float data[6]){
    icm42688RawData_t acc ,gyro;
    bsp_IcmGetRawData(&acc,&gyro);
    data[0]=acc.x;
    data[1]=acc.y;
    data[2]=acc.z;
    data[3]=gyro.x;
    data[4]=gyro.y;
    data[5]=gyro.z;
}
#define M_PI (float)3.1415926535
int8_t bsp_IcmGetRawData(icm42688RawData_t* accData, icm42688RawData_t* GyroData)
{
    uint8_t buffer[12] = {0};

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);
    icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer+6, 6);
    int16_t accData_x  = ((uint16_t)buffer[0] << 8)  | buffer[1];
    int16_t accData_y  = ((uint16_t)buffer[2] << 8)  | buffer[3];
    int16_t accData_z  = ((uint16_t)buffer[4] << 8)  | buffer[5];
    int16_t GyroData_x = ((uint16_t)buffer[6] << 8)  | buffer[7];
    int16_t GyroData_y = ((uint16_t)buffer[8] << 8)  | buffer[9];
    int16_t GyroData_z = ((uint16_t)buffer[10] << 8) | buffer[11];


    accData->x = (float)(accData_x * accSensitivity); //  m/s^2
    accData->y = (float)(accData_y * accSensitivity);
    accData->z = (float)(accData_z * accSensitivity);

    GyroData->x = (float)(GyroData_x / gyroSensitivity) / 180.0f * M_PI; // rad/s
    GyroData->y = (float)(GyroData_y / gyroSensitivity) / 180.0f * M_PI;
    GyroData->z = (float)(GyroData_z / gyroSensitivity) / 180.0f * M_PI;
    return 0;
}

