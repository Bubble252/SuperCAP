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
uint8_t isPWD = 1;//ֹͣ�����ʶ��
uint8_t isCAP_START = 0;//�����Ƿ������͵�ѹ
uint16_t PWUcnt = 0;//�����������
float kf = 0.9;//ǰ������
float power_limit = DEFAULT_RECIEVE;//��ʼ��������
float power_ref = 5;//���ʲο�
float duty;
float buck_duty;
float boost_duty;

float default_power_limit=DEFAULT_RECIEVE;

int charge_state=1;//Ĭ�ϳ��

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
		// ���빦������
	if (power_limit >= 10240)
	{		// ����ϵͳ�����쳣״����δ����256
		power_limit /= 256;
	}
	
	if (power_limit >= 120)
	{ // �������ƴ���120�����Ĭ��Ϊ120W
		power_limit = 120;
	}
	else
	{
		power_limit = power_limit_recieve-5; // ���������������CAN���͵�����
	}
	
	if (power_limit == 0)
	{
		power_limit = default_power_limit-5; // ����ϵͳ���߹�������
	}
	
	//ͨ�ſ����Ƿ�ֱ��
	if(CAP_manage_Cmd==3)//����ģʽ+���ʻ�����
	{
		CAP_state=3;
	}
		
	if(CAP_manage_Cmd==2)//ֱ��ģʽ
	{
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);//�رհ���
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
		CAP_state=0;
		
		isPWD = 1;
		PWUcnt = 0;
	}
////�������
//buckboost_pwm_update(1.2f);
		
////��ѹ���
//	control.vloop_ratio = PID_calc(&control.voltout_loop, V_OUT, 17) / V_IN;
//	control.volt_ratio = control.vloop_ratio;
//  buckboost_pwm_update(control.volt_ratio);
	
////��������
//	control.cloop_ratio = PID_calc(&control.currout_loop, I_OUT, 0.4) / V_IN;
//	control.volt_ratio = control.cloop_ratio;
//	buckboost_pwm_update(control.volt_ratio);

////�������
//	control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, 0.4) / V_IN;
//	control.volt_ratio = control.cloop_ratio;
//	buckboost_pwm_update(control.volt_ratio);


////�㹦������ ����������
///*****************************************/
//	  control.dcdc_power = PID_calc(&control.powerin_loop, POWER_IN, 40);
//    control.dcdc_max_curr = fmaxf((V_OUT / V_IN) * (3.0f - 1.0f), 1.0f);
//    control.dcdc_curr = fmaxf(fminf((control.dcdc_power) / V_IN, control.dcdc_max_curr), -control.dcdc_max_curr);
//	
//	  control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, control.dcdc_curr) / V_IN;
//    control.vloop_ratio = PID_calc(&control.voltout_loop, V_OUT, 17) / V_IN;

//	if (control.dcdc_power > 0) {
//	      if (control.vloop_ratio <= control.cloop_ratio) {
//            //��ѹ
//            control.volt_ratio = control.vloop_ratio;         
//        } 
//	    	else {
//            //����
//            control.volt_ratio = control.cloop_ratio;   
//        }
//        //����PWM
//        buckboost_pwm_update(control.volt_ratio);
//	}
//	else if (control.dcdc_power < 0) {
//        
//        //��������PID
//        control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, control.dcdc_curr) / V_OUT;
//        control.volt_ratio = control.cloop_ratio;
//        //����PWM
//        buckboost_pwm_update(control.volt_ratio);
//    }

///*****************************************/

//�㹦������ ������
/*****************************************/
//	  control.dcdc_power = PID_calc(&control.powerin_loop, POWER_IN, 40);
//    control.dcdc_curr = (control.dcdc_power) / V_IN;

//	  control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, control.dcdc_curr) / V_IN;
//    control.vloop_ratio = PID_calc(&control.voltout_loop, V_OUT, 17) / V_IN;

//	if (control.dcdc_power > 0) {
//		control.dcdc_curr = fminf(control.dcdc_curr, 5);
//    control.dcdc_curr = fmaxf(control.dcdc_curr, -20);
//	      if (control.vloop_ratio <= control.cloop_ratio) {
//            //��ѹ
//            control.volt_ratio = control.vloop_ratio;         
//        } 
//	    	else {
//            //����
//            control.volt_ratio = control.cloop_ratio;   
//        }
//        //����PWM
//        buckboost_pwm_update(control.volt_ratio);
//	}
//	else if (control.dcdc_power < 0) {
//        
//        //��������PID
//        control.cloop_ratio = PID_calc(&control.currout_loop, I_IN, control.dcdc_curr) / V_OUT;
//        control.volt_ratio = control.cloop_ratio;
//        //����PWM
//        buckboost_pwm_update(control.volt_ratio);		
//    }

/*****************************************/
	
	
	
/***********************************************************************************************************************/
    if(bat_voltage < 17.f || bat_voltage > 30.f)//���缰��ѹ����
		{

			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);//�رհ���
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
			isPWD = 1;
			PWUcnt = 0;
			CAP_state=9;
		}
		
		
		//״̬���
		//�ָ�������ѹ���������PID���������š����ʻ�����
		else if((CAP_state==3)&&bat_voltage < 29.f && bat_voltage > 18.f)
		{
			//����pid�ͻ�����
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
			
					HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);//��������
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
		//��·����
		
		
		
		
		//���ʻ����� ��һ��һ���������빦�ʻ���PIDĿ��ֵ��
		if(power_ref < power_limit)
			power_ref += 0.01f;
		else if(power_ref > power_limit)
			power_ref = power_limit;//���ʻ�����

		cap_voltage = V_OUT;//���ݵ�ѹ
		bat_voltage = V_IN;//��������ѹ
		bat_current = I_IN ;//����������
		cap_current = I_OUT;//���ݵ���
    
		pid_calculate(&cap_vol_h, cap_voltage);//������ݸ�ѹ��ѹ��
		pid_calculate(&cap_vol_l, cap_voltage);//������ݵ�ѹ��ѹ��
		inp_power.ref = power_ref;
		//inp_power.ref = 20;
//		pid_calculate(&inp_power, bat_current * bat_voltage);//���㹦�ʻ�pid
		pid_calculate(&inp_power, POWER_IN);
		float current_ref = inp_power.output;
		
		//�������� ���ʻ� ��ѹ��ѹ�� ��ѹ��ѹ��
		if(current_ref > cap_vol_h.output)
		{
			current_ref = cap_vol_h.output;
			inp_power.output = current_ref;
		}
		else if(isCAP_START && (current_ref < cap_vol_l.output))//���ڵ�����͵�ѹ �ҹ��ʻ��������Ϊ��ֵС�ڣ���ѹ��������
			//���������Ϊ�ǳ���Ҫ�ŵ� �����Ѿ����˽ӽ���͵�ѹ��λ�ã�
		{
			current_ref = cap_vol_l.output;
			inp_power.output = current_ref;
		}
		else//�Ѿ�С����͵�ѹ10V
		{
			cap_vol_h.output = current_ref;
			cap_vol_l.output = current_ref;
		}
		
		//���ĵ����� ����������ת��Ϊռ�ձȵĿ���
		//current_ref = 0.2;
		cap_cur.ref = current_ref;
		pid_calculate(&cap_cur, cap_current);//����Ͳ������pid
		duty = cap_cur.output;//�����壩ռ�ձ�Ϊ�߲���������
		//����ռ�ձ�ת��Ϊbuck��boostռ�ձ�
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
		
		//�趨HRTIMռ�ձ�
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, 12800 - 12800 * buck_duty);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, 12800 + 12800 * buck_duty);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, 12800 - 12800 * boost_duty);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, 12800 + 12800 * boost_duty);
/********************************************************************************************************************/

	
	    //�������ʣ������
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
  * @brief          ���㲢����buckboost��pwm
  * @param[in,out]  volt_ratio: ��ѹ��ֵ (Vout/Vin)
  * @retval         ��
  */
void buckboost_pwm_update(float volt_ratio)
{

    volt_ratio = fminf(max_volt_ratio, volt_ratio);
    volt_ratio = fmaxf(min_volt_ratio, volt_ratio);

    /*
     * ����ѹ��PWM������
     * 1.����MOS�����Ծٵ���������ʽ�����Ϲܲ���100%ռ�ձȵ�ͨ������������Ϊ95%��
     * 2.���������������Ϊ�ܵ�ռ�ձȣ���Ҫ�ֱ�����������ŵ�ռ�ձȡ�
     * 3.��ѹ����ʱ��BOOST�İ��ŵ�ռ�ձ��ǹ̶���Ϊ95%��ͨ���ı�BUCK����ռ�ձ�ʵ����ѹ��
     * 3.��ѹ���ѹ����ʱ��BUCK���ŵ�ռ�ձ��ǹ̶���Ϊ95%��ͨ���ı�BOOST����ռ�ձ�ʵ����ѹ��
     */
    if (volt_ratio >= 1.0f)
        CompareValue = DP_PWM_PER * (shift_duty_2 - shift_duty / volt_ratio);
    else
        CompareValue = shift_duty * volt_ratio * DP_PWM_PER;

    //����boost����ռ�ձ�
    if ((float) CompareValue > shift_duty * DP_PWM_PER)
        //BOOST����ռ�ձȣ�Dboost = D�� - Dbuck
        boost_duty = shift_duty_2 * DP_PWM_PER - (float) CompareValue;
    else
        //BUCKģʽ��boost_duty���̶�ռ�ձ�
        boost_duty = shift_duty * DP_PWM_PER;

    //����buck����ռ�ձ�
    if ((float) CompareValue > shift_duty * DP_PWM_PER)
        //BOOSTģʽ��buck_duty�̶�ռ�ձ�
        buck_duty = shift_duty * DP_PWM_PER;
    else
        //BUCK����ռ�ձ�
        buck_duty = CompareValue;

    //ռ�ձ�����
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
	
	if(CAP_state==3)//����
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);//RED
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);//BLUE
  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);//GREEN
	}
		
	if(CAP_state==0)//ֱ��
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);//RED
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);//BLUE
  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);//GREEN
	}
		
	if(CAP_state==9)//����
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);//RED
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);//BLUE
  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);//GREEN
	}
	

	if(can_state_cnt==100)
	{
		can_state_cnt=0;
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);//wrong ͨѶ����
	}
	
}