#ifndef PID_H_
#define PID_H_

#include <stdint.h>



//typedef enum pid_mode{PID_POSITION,PID_DELTA}PID_MODE;
//typedef struct pid_t
//{
//	PID_MODE PID_Mode; 
//	float KP;
//  float KI;
//  float KD;
//  float error[3];
//  float error_sum;
//  float error_max;
//  float fdb;
//  float ref;
//  float output;
//  float outputmax;
//	float outputmin;
//	float feedforward;
//}PID;

//extern PID CAP_V_OUT_PID;

//void pid_calculate(PID *pid, float fdb);
//void pid_init(PID *pid, PID_MODE PID_Mode, float KP,float KI,float KD,float error_max,float outputmax,float outputmin);
//void pid_reset(PID *pid);
//void pid_setfeedforward(PID *pid ,float feedforward);

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};


typedef struct
{
    unsigned char mode;
    float Kp;
    float Ki;
    float Kd;
    float T;

    float max_out;
    float min_out;
    float max_iout;
    float min_iout;
    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];

} pid_type_def;


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
extern void PID_init(pid_type_def *pid, unsigned char mode, const float p, const float i, const float d, float max_out, float max_iout, float min_out, float min_iout);


/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern float PID_calc(pid_type_def *pid, float ref, float set);


/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         无
  */
extern void PID_clear(pid_type_def *pid);

#endif
