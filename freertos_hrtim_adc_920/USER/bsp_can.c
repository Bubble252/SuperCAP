#include "bsp_can.h"

uint8_t CAP_recieve[8];//接收缓冲区
uint16_t power_limit_recieve=50.f;//接收的功率上限
uint16_t power_buffer_recieve=60;//接收的功率上限
uint8_t CAP_manage_Cmd=3;// 控制码 3 默认开超电  2 直连
uint8_t CAP_state=3;//状态码 3 默认开超电 0 直连 9 错误


CAN_RxHeaderTypeDef RxMessage;
CAN_TxHeaderTypeDef TxMessage0;
typedef struct
{
    uint16_t can_id;//电机ID
    int16_t  set_voltage;//设定的电压值
    uint16_t rotor_angle;//机械角度
    int16_t  rotor_speed;//转速
    int16_t  torque_current;//扭矩电流
    uint8_t  temp;//温度
}moto_info_t;
moto_info_t motor_yaw_info;



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data);
	
	  switch(RxMessage.StdId)
	{
//	  case 0x205:	 
//	{
//    motor_yaw_info.rotor_angle    = ((data[0] << 8) | data[1]);
//    motor_yaw_info.rotor_speed    = ((data[2] << 8) | data[3]);
//    motor_yaw_info.torque_current = ((data[4] << 8) | data[5]);
//    motor_yaw_info.temp           =   data[6];
//		break;
//	}
		case 0x302:
	{
		can_state_cnt++;
//		power_buffer_recieve = (uint16_t)(CAP_recieve[0] << 8 | CAP_recieve[1]);
//    power_limit_recieve = (uint16_t)(CAP_recieve[2] << 8 | CAP_recieve[3]);
//    CAP_manage_Cmd = (uint8_t)(CAP_recieve[4]);
		
		
		power_buffer_recieve = (uint16_t)(data[0] << 8 | data[1]);
    power_limit_recieve = (uint16_t)(data[2] << 8 | data[3]);
    CAP_manage_Cmd = (uint8_t)(data[4]);
    break;
	}

	}
}
/****************************************************************/
int v1=10000;
int v2=10000;
 uint8_t             tx_data[8] = {0};
void CAN1_Send_Test(void)
{
	uint32_t CAN_TX_BOX=0;
	tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
	tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
//	uint8_t data[8] ={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
	TxMessage0.IDE= CAN_ID_STD;
	TxMessage0.StdId = 0x1ff;
	TxMessage0.RTR =CAN_RTR_DATA;
	TxMessage0.DLC = 0x08;
	//	TxMessage.TransmitGlobalTime = DISABLE;
HAL_CAN_AddTxMessage(&hcan,&TxMessage0, tx_data,&CAN_TX_BOX);
//	if (HAL_CAN_AddTxMessage(&hcan,&TxMessage, data,&CAN_TX_BOX) != HAL_OK){
//		a=7;
//		Error_Handler();
//	}
}
/****************************************************************/

 void CANFilter_Config(void)
{

	  CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;	
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan, &can_filter_st);
 
}
uint8_t what=0;
uint8_t what2=0;
uint16_t change=0;
void Send_Cap_Data(CAN_HandleTypeDef *_hcan, float Cap_Vol, float Power_3508, uint8_t state)
{
 static CAN_TxHeaderTypeDef TxMessage;
 static uint8_t CAN_Send_Data[8];
 static uint16_t Cap_Vol_send;
 static uint16_t Dipan_W;
 uint32_t send_mail_box;
	
	
 Cap_Vol_send = (int16_t)(V_OUT * 1000); // 将浮点型*1000，并转化为 int16 类型
 Dipan_W = (int16_t)(Power_3508 * 1000);
	
//	Dipan_W = (int16_t)(Power_3508 * 1000);

	TxMessage.IDE= CAN_ID_STD;
	TxMessage.StdId = 0x301;
	TxMessage.RTR =CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	
//	CAN_Send_Data[0] = (Cap_Vol_send >> 8); ////将 Cap_Vol_send 的高字节移动到低字节位置
//  CAN_Send_Data[1] = Cap_Vol_send;
	CAN_Send_Data[0] = (Cap_Vol_send >> 8); ////将 Cap_Vol_send 的高字节移动到低字节位置
  CAN_Send_Data[1] = Cap_Vol_send;
  CAN_Send_Data[2] = (Dipan_W >> 8);
  CAN_Send_Data[3] = Dipan_W;
  CAN_Send_Data[4] = state;
	
		what=Cap_Vol_send;
what2=CAN_Send_Data[4];
//  HAL_CAN_AddTxMessage(&hcan,&TxMessage, tx_data,&CAP_TX_BOX);
  HAL_CAN_AddTxMessage(&hcan,&TxMessage, CAN_Send_Data,&send_mail_box);

}