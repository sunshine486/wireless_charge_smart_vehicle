/**
******************************************************************************
* @file             : CapCharge.h
* @brief            : 无线充电控制程序头文件
* @author           : 针针扎是带啥纸
* @date             : 2022.8.18
* @version          : 1.1
* @note             : 超级电容恒功率充电，充电过程先恒流，达到设定功率后恒功率，达到设定电压后恒压
* @note             : 当前适配硬件-HGF充电板V8.1
* @note             : 使用标准单位值进行运算(如使用3.3V而非4096)
******************************************************************************
*/
#ifndef _CHARGE_H
#define _CHARGE_H

#include <stdio.h>
#include <stdint.h>
#include <math.h>

extern uint8_t Charge_Flag; //充电标志
extern uint8_t Charge_Flag_start;
extern uint8_t CHARGE_P;

/* 设备号（端口/引脚）定义 */
#define CAPCHARGEBridgeSD P00_7
#define CAPCHARGEBridgeIN IfxGtm_ATOM0_5_TOUT15_P00_6_OUT
#define CAPCHARGEInVoltADC ADC36
#define CAPCHARGEOutVoltADC ADC37
#define CAPCHARGEOutCurrADC ADC38

/* ADC采样缩放比例 */
#define CAPCHARGEInVoltPROP 28    // 27k,1k
#define CAPCHARGEOutCurrPROP 3.33 // INA180A3,100倍,0.003R:(1/100*0.003)
#define CAPCHARGEOutVoltPROP 5.7  // 4.7k,1k:((1k+4.7k)/1k)

/* ADC采样限幅 */
#define CAPCHARGEInVoltLimit 90
#define CAPCHARGEOutCurrLimit 12
#define CAPCHARGEOutVoltLimit 15

/* 中断时间,S */
#define CHARGE_INTERRUPT_TIM 0.005

/* PWM开关频率,Hz */
#define CAPCHARGEPWMFreq 100000

/* 开关使能极性 */
#define MOSEnable 0 // V8充电板添加了反相器
#define MOSDisable 1

/* 限幅函数 */
#define CapChargeLimit(num, max, min) (num > min ? num : min) < max ? num : max

/* PID结构体 */
typedef struct
{
    float target_val; //目标值
    float err;        //偏差值
    float err_last;   //上一个偏差值
    float Kp, Ki, Kd;
    float integral;   //积分值
    float output_val; //输出值
} ChargePID;

/* 充电状态枚举 */
enum ChargeStatus //充电状态枚举
{
    WAIT = 1,    //未充电
    CHARGE_CURR, //正在恒电流充电
    CHARGE_POW,  //正在恒功率充电
    CHARGE_VOLT, //正在恒压充电
    PROTECT,     //故障保护
};

/* 监视器 */
typedef struct
{
    float InVolt;   //输入电压
    float InCurr;   //输入电流
    float OutVolt;  //输出电压
    float OutCurr;  //输出电流
    float OutPower; //输出功率
} ChargeMonitor;

extern uint8_t Charge_Flag；
extern float Charge_PWM;
extern enum ChargeStatus CapChargeStatus;
extern ChargeMonitor CapChargeMonitor;
extern char monitor_txt[];

void Get_All(void);
float SeqIntPID(ChargePID *pid, float actual_val);
void Charge_Ctrl(float GiveValue, float ActualValue);
void Charge_Protect(void);
void Charge_Interface(float Charge_Value);
void Chrage_PID_Init();
void Charge_Init();
void Charge_Check(void);
void Charge_Callback(void);

#endif /* CODE_CHARGE_H_ */
