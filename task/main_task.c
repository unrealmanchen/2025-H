#include "cmsis_os.h"
#include "bsp.h"
#include "pid.h"
#include "icm42688.h"
#include "bsp_led.h"
#include "main_task.h"
#include "usart.h"
#include "motor.h"
#include "string.h"
#include "usart.h"
#include "bsp_dwt.h"
#include "ins_task.h"


osThreadId main_task_hld;
osThreadId ins_task_hld;

QueueHandle_t imu_data_Queue;

volatile char send[30];
void VOFA_send(float *data, int num)
{
    int total_bytes = (num + 1) * 4;

    // 先拷贝所有 float 数据
    memcpy(send, data, num * 4);

    // 添加帧尾
    send[total_bytes - 4] = 0x00;
    send[total_bytes - 3] = 0x00;
    send[total_bytes - 2] = 0x80;
    send[total_bytes - 1] = 0x7f;

    HAL_UART_Transmit_DMA(&huart1, (char *)send, total_bytes);
}

char buffer[50];
pid_t yaw_deg, pitch_deg;
void main_task_init()
{

    LED_toggle(RED, 0);
    LED_toggle(BLUE, 0);
    LED_toggle(GREEN, 0);

    imu_data_Queue = xQueueCreate(1, sizeof(float)* 5 );
    osThreadDef(maintask, main_task, osPriorityNormal, 0, 256);
    main_task_hld = osThreadCreate(osThread(maintask), NULL);

    osThreadDef(ins_task, INS_Task_instance, osPriorityAboveNormal, 0, 4096);
    ins_task_hld = osThreadCreate(osThread(ins_task), NULL);


    if(imu_data_Queue == NULL || main_task_hld == NULL || ins_task_hld == NULL)
    {
        LED_toggle(BLUE,1);
        Error_Handler();
    }
    

    PID_init(&yaw_deg, 0.13, 0.000006, 0, 0, 100, 30, 1000);
    PID_init(&pitch_deg, 0.04, 0, 0, 0, 100, 30, 1000);
}

volatile int x = 9999, y = 9999;
float x_out = 0, y_out = 0;
volatile int flag =0;
float y_leg = 0;
float quene_data[5] = {0};


float ins_init[2]= {0, 0};

void main_task()
{
    uint32_t flag  = 0;
    while (flag == 0)
    {
        flag = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, buffer, sizeof(buffer));
    //__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

    can_filter_init();

    QD4310_CMD(1, 2);
    QD4310_CMD(0, 2);
    HAL_Delay(100);
    QD4310_CMD(1, 1);
    QD4310_CMD(0, 1);

    motor_init();

    for(int i = 0; i < 2000; i++)
    {
        QD4310_CMD(1, 1);
        QD4310_CMD(0, 1);
        QD4310_control_deg(yaw, 0);
        QD4310_control_deg(pitch, 0);
        osDelay(1);
    }

    xQueueReceive(imu_data_Queue, quene_data, 0);
    ins_init[0] = quene_data[0];
    ins_init[1] = quene_data[1];
    
    int count = 0;
    while (1)
    {
        xQueueReceive(imu_data_Queue, quene_data, 0);
        QD4310_CMD(1, 1);
        QD4310_CMD(0, 1);



        float deg_input = QD4310_set_deg(0, quene_data[0]-ins_init[0]);
        float w = PID_calc(&(motor_yaw.pid_deg), deg_input, 0);
        fp32 current_data = PID_calc(&(motor_yaw.pid_w), motor_yaw.data.speed, w - quene_data[4] * 9.5493f);//-quene_data[4] * 9.5493f
        QD4310_Current(1, current_data);



        float deg_input_p = QD4310_set_deg(180.0f+ quene_data[1] - ins_init[1], motor_pitch.data.angle);
        float w_p = PID_calc(&(motor_pitch.pid_deg), deg_input_p, 0);
        fp32 current_data_p = PID_calc(&(motor_pitch.pid_w), motor_pitch.data.speed, w_p + quene_data[3] * 9.5493f );//+
        QD4310_Current(0, current_data_p);






        #ifdef DEBUG
        if (  (!(x == 9999 || y == 9999)))
        {
            flag =0;
            count = 0;
            x_out = PID_calc(&yaw_deg, 0,x) * -1.0f;
            //x_out = 0;
            y_out = PID_calc(&pitch_deg, 0, y);
            y_out = y_leg*0.5f + y_out*0.5f;
            y_leg = y_out;
            QD4310_control_w(yaw, x_out);
            QD4310_control_w(pitch, y_out);
        }
        else
        {
            if (count++ > 500)
            {
                QD4310_control_deg(yaw, 0);
                QD4310_control_deg(pitch, 0);
            }
            else
            {
                QD4310_control_w(yaw, 0);
                QD4310_control_w(pitch, 0);
            }
        }
        #endif
        // if(motor_pitch.data.angle > -35)
        // QD4310_control_w(enum ID_t ID,float w);
        osDelay(1);
    }
}

uint32_t getDWTCountDx(uint32_t startTick, uint32_t stopTick)
{
    uint32_t tick;
    if (stopTick < startTick)
    {
        tick = (0xffffffff - startTick) + stopTick + 1;
    }
    else
    {
        tick = stopTick - startTick;
    }

    return tick;
}

volatile long int tick, start, stop;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2)
    {

        // start = DWT->CYCCNT;
        // stop = DWT->CYCCNT;
        // tick = getDWTCountDx(start,stop) / 168;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, buffer, sizeof(buffer));
        int a, b;
        sscanf(buffer, "dx=%d dy=%d", &a, &b);
        x = a;
        y = b;
        memset(buffer, 0, sizeof(buffer));
        flag =1;
    }
}