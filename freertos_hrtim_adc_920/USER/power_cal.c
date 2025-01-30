#include "power_cal.h"

   
volatile unsigned int CompareValue;
float cap_percent=0.0;

PID cap_vol_h;
PID cap_vol_l;
PID inp_power;
PID cap_cur;
float cap_voltage;
float cap_current;
float bat_voltage;
float bat_current;
uint8_t isPWD = 1;//停止供电标识符
uint8_t isCAP_START = 0;//电容是否高于最低电压
uint16_t PWUcnt = 0;//开启供电计数
float kf = 0.9;//前馈参数
float power_limit = DEFAULT_RECIEVE;//初始功率限制
float power_ref = 5;//功率参考
float duty;
float buck_duty;
float boost_duty;

float default_power_limit=DEFAULT_RECIEVE;

int charge_state=1;//默认充电

int can_state_cnt=0;

void CAL_INIT(void)
{
	pid_init(&cap_vol_h,PID_DELTA2, 0.5,0.05,0,0,10,-18);
	pid_init(&cap_vol_l,PID_DELTA2, 0.5,0.05,0,0,10,-18);
	pid_init(&inp_power,PID_DELTA2, 0.04,0.008,0,0,10,-18);
	pid_init(&cap_cur,PID_DELTA2, 0.005,0.0005,0,0,1.7,0);
	cap_vol_h.ref = 22.0;
	cap_vol_l.ref = 8.0;
	inp_power.ref = 5.0;
	duty = 0.9;

}


void Ctrloop(void)
{
	power_limit = power_limit_recieve;
		// 接入功率限制
	if (power_limit >= 10240)
	{		// 裁判系统出现异常状况，未除以256
		power_limit /= 256;
	}
	
	if (power_limit >= 120)
	{ // 功率限制大于120的情况默认为120W
		power_limit = 120;
	}
	else
	{
		power_limit = power_limit_recieve-5; // 若数据正常则接入CAN发送的数据
	}
	
	if (power_limit == 0)
	{
		power_limit = default_power_limit-5; // 裁判系统掉线功率限制
	}
	
	//通信控制是否直连
	if(CAP_manage_Cmd==3)//电容模式+功率缓启动
	{
		CAP_state=3;
	}
		
	if(CAP_manage_Cmd==2)//直连模式
	{
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);//关闭半桥
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
		CAP_state=0;
		
		isPWD = 1;
		PWUcnt = 0;
	}
////开环输出
//buckboost_pwm_update(1.2f);
		
////恒压输出
//	control.vloop_ratio = PID_calc(&control.voltout_loop, V_OUT, 17) / V_IN;
//	control.volt_ratio = control.vloop_ratio;
//  buckboost_pwm_update(control.volt_ratio);
	
////恒流输入
//	control.cloop_ratio = PID_calc(&control.currout_loop, I_OUT, 0.4) / V_IN;
//	control.volt_ratio = control.cloop_ratio;
//	buckboost_pwm_update(control.volt_ratio);

////恒流输出
//	control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, 0.4) / V_IN;
//	control.volt_ratio = control.cloop_ratio;
//	buckboost_pwm_update(control.volt_ratio);


////恒功率输入 西交利物浦
///*****************************************/
//	  control.dcdc_power = PID_calc(&control.powerin_loop, POWER_IN, 40);
//    control.dcdc_max_curr = fmaxf((V_OUT / V_IN) * (3.0f - 1.0f), 1.0f);
//    control.dcdc_curr = fmaxf(fminf((control.dcdc_power) / V_IN, control.dcdc_max_curr), -control.dcdc_max_curr);
//	
//	  control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, control.dcdc_curr) / V_IN;
//    control.vloop_ratio = PID_calc(&control.voltout_loop, V_OUT, 17) / V_IN;

//	if (control.dcdc_power > 0) {
//	      if (control.vloop_ratio <= control.cloop_ratio) {
//            //恒压
//            control.volt_ratio = control.vloop_ratio;         
//        } 
//	    	else {
//            //恒流
//            control.volt_ratio = control.cloop_ratio;   
//        }
//        //更新PWM
//        buckboost_pwm_update(control.volt_ratio);
//	}
//	else if (control.dcdc_power < 0) {
//        
//        //单电流环PID
//        control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, control.dcdc_curr) / V_OUT;
//        control.volt_ratio = control.cloop_ratio;
//        //更新PWM
//        buckboost_pwm_update(control.volt_ratio);
//    }

///*****************************************/

//恒功率输入 大连理工
/*****************************************/
//	  control.dcdc_power = PID_calc(&control.powerin_loop, POWER_IN, 40);
//    control.dcdc_curr = (control.dcdc_power) / V_IN;

//	  control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, control.dcdc_curr) / V_IN;
//    control.vloop_ratio = PID_calc(&control.voltout_loop, V_OUT, 17) / V_IN;

//	if (control.dcdc_power > 0) {
//		control.dcdc_curr = fminf(control.dcdc_curr, 5);
//    control.dcdc_curr = fmaxf(control.dcdc_curr, -20);
//	      if (control.vloop_ratio <= control.cloop_ratio) {
//            //恒压
//            control.volt_ratio = control.vloop_ratio;         
//        } 
//	    	else {
//            //恒流
//            control.volt_ratio = control.cloop_ratio;   
//        }
//        //更新PWM
//        buckboost_pwm_update(control.volt_ratio);
//	}
//	else if (control.dcdc_power < 0) {
//        
//        //单电流环PID
//        control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, control.dcdc_curr) / V_OUT;
//        control.volt_ratio = control.cloop_ratio;
//        //更新PWM
//        buckboost_pwm_update(control.volt_ratio);		
//    }

/*****************************************/
	
	
	
/***********************************************************************************************************************/
    if(bat_voltage < 17.f || bat_voltage > 30.f)//掉电及过压保护
		{

			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);//关闭半桥
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
			isPWD = 1;
			PWUcnt = 0;
			CAP_state=9;
		}
		
		
		//状态清除
		//恢复正常电压会进行重置PID、开启半桥、功率缓启动
		else if((CAP_state==3)&&bat_voltage < 29.f && bat_voltage > 18.f)
		{
			//重置pid和缓启动
			if(isPWD)
			{
				PWUcnt++;
				if(PWUcnt > 1000)
				{
					power_ref = 5;
					pid_reset(&inp_power);
					pid_reset(&cap_vol_h);
					pid_reset(&cap_vol_l);
					pid_reset(&cap_cur);
					float output_preload = kf * cap_voltage/bat_voltage;

					cap_cur.output = output_preload;
			
					HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);//开启半桥
					HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
					isPWD = 0;
					PWUcnt = 0;
					isCAP_START = 0;
					
					CAP_state=3;
					
				}
			}
			if(cap_voltage>10.0f)
				isCAP_START = 1;
		}
		//短路保护
		
		
		
		
		//功率缓启动 （一点一点增加输入功率环的PID目标值）
		if(power_ref < power_limit)
			power_ref += 0.01f;
		else if(power_ref > power_limit)
			power_ref = power_limit;//功率缓启动

		cap_voltage = V_OUT;//电容电压
		bat_voltage = V_IN;//电管输入电压
		bat_current = I_IN ;//电管输入电流
		cap_current = I_OUT;//电容电流
    
		pid_calculate(&cap_vol_h, cap_voltage);//计算电容高压电压环
		pid_calculate(&cap_vol_l, cap_voltage);//计算电容低压电压环
		inp_power.ref = power_ref;
		//inp_power.ref = 20;
//		pid_calculate(&inp_power, bat_current * bat_voltage);//计算功率环pid
		pid_calculate(&inp_power, POWER_IN);
		float current_ref = inp_power.output;
		
		//三环并联 功率环 高压电压环 低压电压环
		if(current_ref > cap_vol_h.output)
		{
			current_ref = cap_vol_h.output;
			inp_power.output = current_ref;
		}
		else if(isCAP_START && (current_ref < cap_vol_l.output))//高于电容最低电压 且功率环输出（作为负值小于）低压环（负）
			//（可以理解为非常需要放电 但是已经到了接近最低电压的位置）
		{
			current_ref = cap_vol_l.output;
			inp_power.output = current_ref;
		}
		else//已经小于最低电压10V
		{
			cap_vol_h.output = current_ref;
			cap_vol_l.output = current_ref;
		}
		
		//最后的电流环 将电流需求转化为占空比的控制
		//current_ref = 0.2;
		cap_cur.ref = current_ref;
		pid_calculate(&cap_cur, cap_current);//计算低侧电流环pid
		duty = cap_cur.output;//（广义）占空比为高侧电流环输出
		//广义占空比转换为buck和boost占空比
		if(duty>1.5f)
			duty = 1.5f;
		else if(duty<0.2f)
			duty = 0.2f;
		if(duty>0.9f)
		{
			buck_duty = 0.9f;
			boost_duty = 0.81f / duty;
		}
		else
		{
			buck_duty = duty;
			boost_duty = 0.9f;
		}
		
		//设定HRTIM占空比
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, 12800 - 12800 * buck_duty);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, 12800 + 12800 * buck_duty);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, 12800 - 12800 * boost_duty);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, 12800 + 12800 * boost_duty);
/********************************************************************************************************************/

	
	    //计算电容剩余能量
    cap_percent = (int)(((V_OUT * V_OUT) / (25.0f * 25.0f)) * 100.0f);
    if (cap_percent < 0) cap_percent = 0;
    if (cap_percent > 100) cap_percent = 100;

//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, 12800 - 12800 * 0.9);
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, 12800 + 12800 * 0.9);
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, 12800 - 12800 * boost_duty);
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, 12800 + 12800 * boost_duty);
	
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, 12800 - 12800 * 0.5);
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, 12800 + 12800 * 0.5);
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, 12800 - 12800 * 0.9);
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, 12800 + 12800 * 0.9);
	
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, 12800 - 12800 * buck_duty);
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, 12800 + 12800 * buck_duty);
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, 12800 - 12800 * 0.9);
//	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, 12800 + 12800 * 0.9);
    if(I_OUT>=0.5f){charge_state=1;}
		if(I_OUT<=-0.5f){charge_state=0;}
}



/**
  * @brief          计算并更新buckboost的pwm
  * @param[in,out]  volt_ratio: 电压比值 (Vout/Vin)
  * @retval         无
  */
void buckboost_pwm_update(float volt_ratio)
{

    volt_ratio = fminf(max_volt_ratio, volt_ratio);
    volt_ratio = fmaxf(min_volt_ratio, volt_ratio);

    /*
     * 升降压的PWM方案：
     * 1.由于MOS采用自举电容驱动方式，故上管不能100%占空比导通，这里我们设为95%。
     * 2.本函数的输入参数为总的占空比，需要分别算出两个半桥的占空比。
     * 3.降压工作时，BOOST的半桥的占空比是固定的为95%，通过改变BUCK半桥占空比实现稳压。
     * 3.升压或等压工作时，BUCK半桥的占空比是固定的为95%，通过改变BOOST半桥占空比实现稳压。
     */
    if (volt_ratio >= 1.0f)
        CompareValue = DP_PWM_PER * (shift_duty_2 - shift_duty / volt_ratio);
    else
        CompareValue = shift_duty * volt_ratio * DP_PWM_PER;

    //计算boost半桥占空比
    if ((float) CompareValue > shift_duty * DP_PWM_PER)
        //BOOST半桥占空比，Dboost = D总 - Dbuck
        boost_duty = shift_duty_2 * DP_PWM_PER - (float) CompareValue;
    else
        //BUCK模式下boost_duty给固定占空比
        boost_duty = shift_duty * DP_PWM_PER;

    //计算buck半桥占空比
    if ((float) CompareValue > shift_duty * DP_PWM_PER)
        //BOOST模式下buck_duty固定占空比
        buck_duty = shift_duty * DP_PWM_PER;
    else
        //BUCK半桥占空比
        buck_duty = CompareValue;

    //占空比限制
    if (boost_duty > MAX_PWM_CMP)
        boost_duty = MAX_PWM_CMP;
    if (boost_duty < MIN_PWM_CMP)
        boost_duty = MIN_PWM_CMP;
    if (buck_duty > MAX_PWM_CMP)
        buck_duty = MAX_PWM_CMP;
    if (buck_duty < MIN_PWM_CMP)
        buck_duty = MIN_PWM_CMP;
		
  	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, (DP_PWM_PER - buck_duty) / 2u);
  	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, (DP_PWM_PER + buck_duty) / 2u);
  	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, (DP_PWM_PER - boost_duty) / 2u);
  	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, (DP_PWM_PER + boost_duty) / 2u);
}

void LED_state_check(void)
{
	if(charge_state==1)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);//charge
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//outcharge
	}
	if(charge_state==0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);//charge
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//outcharge			
	}
	
	if(CAP_state==3)//电容
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);//RED
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);//BLUE
  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);//GREEN
	}
		
	if(CAP_state==0)//直连
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);//RED
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);//BLUE
  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);//GREEN
	}
		
	if(CAP_state==9)//错误
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);//RED
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);//BLUE
  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);//GREEN
	}
	

	if(can_state_cnt==100)
	{
		can_state_cnt=0;
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);//wrong 通讯正常
	}
	
}