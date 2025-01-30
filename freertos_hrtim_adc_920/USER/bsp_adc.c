#include "bsp_adc.h"


uint16_t ADC1_RESULT[4]={0,0,0,0};//ADC采样外设到内存的DMA数据保存寄存器
uint16_t ADC2_RESULT[2]={0,0};
float V_IN=0.0;
float SHUNT_IN=0.0;
float I_IN=0.0;

float V_OUT=0.0;
float SHUNT_OUT=0.0;
float I_OUT=0.0;

float SHUNT_WHEEL=0.0;
float I_WHEEL=0.0;

float V_OUT_REAL=0.0;
float V_IN_REAL=0.0;

float POWER_IN=0.0;
float POWER_OUT=0.0;
float POWER_WHEEL=0.0;

float watch=0;

First_Order_Filter_t I_WHEEL_Filter;

CCMRAM void ADCSample(void)
{

/***********************************************************************************************************/
//    static float sum_VIN = 0, sum_SHUNTIN = 0, sum_VOUT = 0, sum_SHUNTOUT = 0, sum_SHUNTWHEEL=0;
//    static float avg_VIN = 0, avg_SHUNTIN = 0, avg_VOUT = 0, avg_SHUNTOUT = 0, avg_SHUNTWHEEL=0;
//    
//		sum_VIN = sum_VIN + ADC2_RESULT[1] - (sum_VIN / 4);
//    avg_VIN = sum_VIN / 4;
//		
//		sum_SHUNTIN = sum_SHUNTIN + ADC1_RESULT[2] - (sum_SHUNTIN / 4);
//    avg_SHUNTIN = sum_SHUNTIN / 4;

//    sum_VOUT = sum_VOUT + ADC2_RESULT[0] - (sum_VOUT / 4);
//    avg_VOUT = sum_VOUT / 4;
//						
//		sum_SHUNTOUT = sum_SHUNTOUT + (ADC1_RESULT[0]-ADC1_RESULT[1]) - (sum_SHUNTOUT / 4);
//    avg_SHUNTOUT = sum_SHUNTOUT / 4;
//	
//		sum_SHUNTWHEEL = sum_SHUNTWHEEL + (ADC1_RESULT[3]-ADC1_RESULT[1]) - (sum_SHUNTWHEEL / 4);
//    avg_SHUNTWHEEL = sum_SHUNTWHEEL / 4;
//		
////		 V_IN = avg_VIN * 1.03f * 3.3f * 12 / 4095.f;
//  	V_IN = avg_VIN * 1.0f * 3.3f * 16 / 4096.f;
//	
////     SHUNT_IN = avg_SHUNTIN * 1.03f * 3.3f / 4095.f+0.05f;
//	  SHUNT_IN = avg_SHUNTIN * 1.0f * 3.3f / 4096.f;
//    
//		watch = avg_SHUNTIN;

////    V_OUT = avg_VOUT * 1.03f * 3.3f * 12 / 4095.f;
//    V_OUT = avg_VOUT * 1.0f * 3.3f * 16 / 4096.f;
////    SHUNT_OUT = avg_SHUNTOUT * 1.03f * 3.3f / 4095.f;
//    SHUNT_OUT = avg_SHUNTOUT * 1.0f * 3.3f / 4096.f;
//		
//		SHUNT_WHEEL=avg_SHUNTWHEEL * 1.0f * 3.3f / 4096.f;
//		 
////		  I_IN = SHUNT_IN / 0.01f / 50.f;
////     I_OUT = SHUNT_OUT / 0.01f / 50.f;
////     I_IN = SHUNT_IN / 0.002f / 50.f;
////     I_OUT = SHUNT_OUT / 0.002f / 50.f;
//		 
//		 I_IN = SHUNT_IN / 0.002f / 20.f;
//     I_OUT = SHUNT_OUT / 0.002f / 20.f;
//		 I_WHEEL=SHUNT_WHEEL / 0.002f / 20.f;
//		 
////		 POWER_IN=V_IN*I_IN;
//		 POWER_OUT=V_OUT*I_OUT;
//		 POWER_WHEEL=V_IN*I_WHEEL;
//		 POWER_IN=POWER_WHEEL+POWER_OUT;
//		 
		 
/***********************************************************************************************************/
static float sum_VIN = 0, sum_SHUNTIN = 0, sum_VOUT = 0, sum_SHUNTOUT = 0, sum_SHUNTWHEEL = 0;
    static float avg_VIN = 0, avg_SHUNTIN = 0, avg_VOUT = 0, avg_SHUNTOUT = 0, avg_SHUNTWHEEL = 0;

    sum_VIN = sum_VIN + ADC2_RESULT[1] - (sum_VIN / WINDOW_SIZE);
    avg_VIN = sum_VIN / WINDOW_SIZE;
    
    sum_SHUNTIN = sum_SHUNTIN + ADC1_RESULT[2] - (sum_SHUNTIN / WINDOW_SIZE);
    avg_SHUNTIN = sum_SHUNTIN / WINDOW_SIZE;

    sum_VOUT = sum_VOUT + ADC2_RESULT[0] - (sum_VOUT / WINDOW_SIZE);
    avg_VOUT = sum_VOUT / WINDOW_SIZE;
    
    sum_SHUNTOUT = sum_SHUNTOUT + (ADC1_RESULT[0] - ADC1_RESULT[1]) - (sum_SHUNTOUT / WINDOW_SIZE);
    avg_SHUNTOUT = sum_SHUNTOUT / WINDOW_SIZE;

    sum_SHUNTWHEEL = sum_SHUNTWHEEL + (ADC1_RESULT[3] - ADC1_RESULT[1]) - (sum_SHUNTWHEEL / WINDOW_SIZE);
    avg_SHUNTWHEEL = sum_SHUNTWHEEL / WINDOW_SIZE;

    // 后续计算
//    V_IN = 1.07f*(avg_VIN * 1.0f * 3.3f * 16 / 4096.f)+1.45f;
    V_IN = avg_VIN * 1.0f * 3.3f * 16 / 4096.f;
    SHUNT_IN = avg_SHUNTIN * 1.0f * 3.3f / 4096.f;
    V_OUT = avg_VOUT * 1.0f * 3.3f * 16 / 4096.f;
    SHUNT_OUT = avg_SHUNTOUT * 1.0f * 3.3f / 4096.f;
    SHUNT_WHEEL = avg_SHUNTWHEEL * 1.0f * 3.3f / 4096.f;

    I_IN = SHUNT_IN / 0.002f / 20.f;
    I_OUT = SHUNT_OUT / 0.002f / 20.f;
//    I_WHEEL = 1.136f*(SHUNT_WHEEL / 0.002f / 20.f)-0.0471f;
		I_WHEEL = SHUNT_WHEEL / 0.002f / 20.f;
//		I_WHEEL = First_Order_Filter_Calculate(&I_WHEEL_Filter, I_WHEEL);

    POWER_OUT = V_OUT * I_OUT;
    POWER_WHEEL = V_IN * I_WHEEL;
    POWER_IN = POWER_WHEEL + POWER_OUT;
		
		if(POWER_IN<=0)
		{POWER_IN=0;}
}

/**
  * @brief          一阶低通滤波初始化
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波系数
  * @retval         返回空
  */
void First_Order_Filter_Init(First_Order_Filter_t *first_order_filter, float frame_period, float num)
{
  first_order_filter->Frame_Period = frame_period;
  first_order_filter->RC = num;
  first_order_filter->Input = 0.0f;
  first_order_filter->Output = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @param[in]      一阶低通滤波结构体
  * @param[in]      测量值
  * @retval         返回滤波输出
  */
float First_Order_Filter_Calculate(First_Order_Filter_t *first_order_filter, float input)
{
  first_order_filter->Input = input;

  // x(t) = x(t-1)*dt/(dt + omega) + u*omega/(dt + omega)
  // X(s) = omega/(s + omega)
  first_order_filter->Output =
      first_order_filter->Output * first_order_filter->RC /
          (first_order_filter->RC + first_order_filter->Frame_Period) +
      first_order_filter->Input * first_order_filter->Frame_Period /
          (first_order_filter->RC + first_order_filter->Frame_Period);

  return first_order_filter->Output;
}

