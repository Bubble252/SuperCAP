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
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         ��
  */
extern void PID_init(pid_type_def *pid, unsigned char mode, const float p, const float i, const float d, float max_out, float max_iout, float min_out, float min_iout);


/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern float PID_calc(pid_type_def *pid, float ref, float set);


/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         ��
  */
extern void PID_clear(pid_type_def *pid);

#endif
