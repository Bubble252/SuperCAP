#ifndef __BSP_ADC_H
#define __BSP_ADC_H
#include "stm32f3xx_hal.h"
#include "adc.h"


/************************采样变量**************************/

extern uint16_t ADC1_RESULT[4];
extern uint16_t ADC2_RESULT[2];
extern float V_OUT;
extern float V_IN;
extern float I_IN;
extern float I_OUT;
extern float POWER_IN;
extern float POWER_OUT;
void ADCSample(void);

#define WINDOW_SIZE 16  // 增加窗口大小，例如16


typedef struct
{
    float Input;        //输入数据
    float Output;       //滤波输出的数据
    float RC;           //滤波参数 RC = 1/omegac
    float Frame_Period; //滤波的时间间隔 单位 s
} First_Order_Filter_t;
void First_Order_Filter_Init(First_Order_Filter_t *first_order_filter, float frame_period, float num);
float First_Order_Filter_Calculate(First_Order_Filter_t *first_order_filter, float input);
extern First_Order_Filter_t I_WHEEL_Filter;

#define CCMRAM  __attribute__((section("ccmram")))
#endif