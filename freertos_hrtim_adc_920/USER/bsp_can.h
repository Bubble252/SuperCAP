#ifndef __POWER_CAL_H
#define __POWER_CAL_H

#include "can.h"
#include "stm32f3xx_hal.h"
#include "power_cal.h"
#include "bsp_adc.h"


extern float cap_voltage;
extern float cap_current;
extern float bat_voltage;
extern float bat_current;

extern uint16_t power_limit_recieve;//接收的功率上限
extern uint16_t power_buffer_recieve;//接收的功率上限
extern uint8_t CAP_manage_Cmd;//控制码 默认开超电
extern uint8_t CAP_state;

uint8_t data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void CANFilter_Config(void);

void Send_Cap_Data(CAN_HandleTypeDef *_hcan, float Cap_Vol, float Power_3508, uint8_t state);

void CAN1_Send_Test(void);

extern int can_state_cnt;

#endif