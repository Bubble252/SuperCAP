#ifndef __POWER_CAL_H
#define __POWER_CAL_H
#include "stm32f3xx_hal.h"
#include "hrtim.h"
#include "bsp_adc.h"
//#include "pid.h"
#include "pid2.h"
#include "math.h"
#include "bsp_can.h"
#define DEFAULT_RECIEVE 50.0f
extern float cap_voltage;
extern float cap_current;
extern float bat_voltage;
extern float bat_current;

extern PID cap_vol_h;
extern PID cap_vol_l;
extern PID inp_power;
extern PID cap_cur;

extern uint16_t power_limit_recieve;
extern uint8_t CAP_manage_Cmd;//控制码 默认开超电
extern uint8_t CAP_state;

extern int can_state_cnt;
/*PWM相关*/
#define shift_duty 0.95f                                    //最大占空比
#define shift_duty_2 2 * shift_duty                         //最大占空比 * 2
#define DP_PWM_PER 25600                                     //定时器HRTIM周期
#define MAX_PWM_CMP (uint32_t)(shift_duty * DP_PWM_PER)           //PWM最大比较值
#define MIN_PWM_CMP (uint32_t)((1 - shift_duty) * DP_PWM_PER)     //PWM最小比较值
#define max_volt_ratio 1.2f                                 //最大电压比值
#define min_volt_ratio 0.1f                                 //最小电压比值

//硬件限制相关
#define INC_MAX                                     7.5f                                //输入过流阈值
#define OUTC_MAX                                    8.0f                                //输出过流阈值
#define INV_MAX                                     30.0f                               //输入过压阈值
#define INV_MIN                                     19.0f                               //输入欠压阈值
#define CAPV_MAX                                    25.0f                               //电容最大电压
#define ICAP_MAX                                    4.0f                                //电容最大电流

//错误码和状态码
#define ERR_OVER_VOLTAGE                            (1u << 0)
#define ERR_LOW_VOLTAGE                             (1u << 1)
#define ERR_CURRENT_SENSOR                          (1u << 2)
#define ERR_OVER_CURRENT                            (1u << 3)
#define ERR_BUCKBOOST                               (1u << 4)
#define ERR_CAN                                     (1u << 5)
#define ERR_MASK                                    (0x0000FFFF)
#define STATUS_CAP_LOW                              (1u << 16)
#define STATUS_CAP_CHARGING                         (1u << 17)
#define STATUS_CAP_DISCHARGING                      (1u << 18)
#define STATUS_CAP_FULL                             (1u << 19)
#define STATUS_MASK                                 (0xFFFF0000)

void buckboost_pwm_update(float volt_ratio);
void CAL_INIT(void);
void Ctrloop(void);
void LED_state_check(void);

#define CCMRAM  __attribute__((section("ccmram")))
#endif