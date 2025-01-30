#include "pid2.h"

void pid_calculate(PID *pid, float fdb)
{
	pid->fdb = fdb;
	pid->error[2] = pid->error[1];       //���ϴ����
  pid->error[1] = pid->error[0];       //�ϴ����
  pid->error[0] = pid->ref - pid->fdb; //�������
	if(pid->PID_Mode == PID_POSITION2)
	{
		float error_delta=pid->error[0]-pid->error[1];
		pid->error_sum += pid->KI * pid->error[0];
		//�������޿���
		if (pid->error_sum > pid->error_max)
			pid->error_sum = pid->error_max;
		else if (pid->error_sum < -pid->error_max)
			pid->error_sum = -pid->error_max;
		pid->output = pid->KP * pid->error[0] + pid->error_sum 
								+ pid->KD * error_delta + pid->feedforward;
	}
	else if (pid->PID_Mode == PID_DELTA2)
	{
		pid->output += pid->KP * (pid->error[0] - pid->error[1]) + 
		pid->KD * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) + pid->KI * pid->error[0]+ pid->feedforward;
	}
	//������޿���
	if(pid->output > pid->outputmax)
		pid->output = pid->outputmax;
	else if(pid->output < pid->outputmin)
		pid->output = pid->outputmin;
	return;
}

void pid_init(PID *pid, PID_MODE PID_Mode,float KP,float KI,float KD,float error_max,float outputmax,float outputmin)
{
	pid->PID_Mode = PID_Mode;
	pid->KP = KP;
	pid->KI = KI;
	pid->KD = KD;
	pid->error_max = error_max;
	pid->outputmax = outputmax;
	pid->outputmin = outputmin;
	return;
}

void pid_reset(PID *pid)
{
	pid->error_sum = 0;
	pid->output = 0;
	for(int i=0;i<3;i++)
	{
		pid->error[i] = 0;
	}
}

void pid_setfeedforward(PID *pid ,float feedforward)
{
	pid->feedforward = feedforward;
}
