#include "pid.h"



//void pid_calculate(PID *pid, float fdb)
//{
//	pid->fdb = fdb;
//	pid->error[2] = pid->error[1];       //上上次误差
//  pid->error[1] = pid->error[0];       //上次误差
//  pid->error[0] = pid->ref - pid->fdb; //本次误差
//	if(pid->PID_Mode == PID_POSITION)
//	{
//		float error_delta=pid->error[0]-pid->error[1];
//		pid->error_sum += pid->KI * pid->error[0];
//		//积分上限控制
//		if (pid->error_sum > pid->error_max)
//			pid->error_sum = pid->error_max;
//		else if (pid->error_sum < -pid->error_max)
//			pid->error_sum = -pid->error_max;
//		pid->output = pid->KP * pid->error[0] + pid->error_sum 
//								+ pid->KD * error_delta + pid->feedforward;
//	}
//	else if (pid->PID_Mode == PID_DELTA)
//	{
//		pid->output += pid->KP * (pid->error[0] - pid->error[1]) + 
//		pid->KD * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) + pid->KI * pid->error[0]+ pid->feedforward;
//	}
//	//输出上限控制
//	if(pid->output > pid->outputmax)
//		pid->output = pid->outputmax;
//	else if(pid->output < pid->outputmin)
//		pid->output = pid->outputmin;
//	return;
//}

//void pid_init(PID *pid, PID_MODE PID_Mode,float KP,float KI,float KD,float error_max,float outputmax,float outputmin)
//{
//	pid->PID_Mode = PID_Mode;
//	pid->KP = KP;
//	pid->KI = KI;
//	pid->KD = KD;
//	pid->error_max = error_max;
//	pid->outputmax = outputmax;
//	pid->outputmin = outputmin;
//	return;
//}

//void pid_reset(PID *pid)
//{
//	pid->error_sum = 0;
//	pid->output = 0;
//	for(int i=0;i<3;i++)
//	{
//		pid->error[i] = 0;
//	}
//}

//void pid_setfeedforward(PID *pid ,float feedforward)
//{
//	pid->feedforward = feedforward;
//}


#define LimitMax(input, max) \
    do {                          \
    if ((input) > (max))         \
    {                        \
      (input) = (max);           \
    }                        \
    else if ((input) < -(max))   \
    {                        \
      (input) = -(max);          \
    }                        \
    } while(0)


/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         无
  */
void PID_init(pid_type_def *pid, unsigned char mode, const float p, const float i, const float d, float max_out, float max_iout,
              float min_out, float min_iout) {
    if (pid == 0) {
        return;
    }
    pid->mode = mode;
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->min_out = min_out;
    pid->min_iout = min_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}


/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
float PID_calc(pid_type_def *pid, float ref, float set) {
    if (pid == 0) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION) {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    } else if (pid->mode == PID_DELTA) {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}


/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         无
  */
void PID_clear(pid_type_def *pid) {
    if (pid == 0) {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

