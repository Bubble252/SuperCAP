#ifndef PID_2_H_
#define PID_2_H_
//#include "pid2.h"
#include <stdint.h>
typedef enum pid_mode{PID_POSITION2,PID_DELTA2}PID_MODE;
typedef struct pid_t
{
	PID_MODE PID_Mode; 
	float KP;
  float KI;
  float KD;
  float error[3];
  float error_sum;
  float error_max;
  float fdb;
  float ref;
  float output;
  float outputmax;
	float outputmin;
	float feedforward;
}PID;

void pid_calculate(PID *pid, float fdb);
void pid_init(PID *pid, PID_MODE PID_Mode, float KP,float KI,float KD,float error_max,float outputmax,float outputmin);
void pid_reset(PID *pid);
void pid_setfeedforward(PID *pid ,float feedforward);
#endif
