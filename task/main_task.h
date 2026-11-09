#ifndef MAIN
#define MAIN
void main_task(void);
void main_task_init(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);


extern QueueHandle_t imu_data_Queue;

extern osThreadId main_task_hld;
extern osThreadId ins_task_hld;
#endif
