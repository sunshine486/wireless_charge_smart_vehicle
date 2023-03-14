/**
******************************************************************************
* @file             : CapCharge.c
* @brief            : 无线充电控制程序主体
* @author           : 针针扎是带啥纸
* @date             : 2022.8.18
* @version          : 1.1
* @note             : 超级电容恒功率充电，充电过程先恒流，达到设定功率后恒功率，达到设定电压后恒压
* @note             : 当前适配硬件-HGF充电板V8.1
* @note             : 使用标准单位值进行运算(如使用3.3V而非4096)
******************************************************************************
*/
#include "charge.h"
#include <IfxGtm_PinMap.h>
#include <_Impl/IfxPort_cfg.h>
#include <ANO_DT.h>
#include <LQ_GTM.h>
#include <LQ_STM.h>
#include <LQ_ADC.h>
#include <LQ_CCU6.h>
#include <LQ_GPIO.h>
#include <LQ_UART.h>
#include <LQ_GPIO_LED.h>

/////////////////////////////  调节参数  /////////////////////////////
uint16_t TargetCap = 2;       //目标满电电容2F
uint16_t TargetOutPower = 58; //目标输出功率W
uint16_t TargetOutVolt = 12;  //目标满电电压V

/* PID初始参数设置 */
float POW_KP = 2, POW_KI = 0.1, POW_KD = 0;
float LALALA = 3;
//////////////////////////////  变量  //////////////////////////////
char monitor_txt[200];

ChargePID CapChargePID_Cur;     //电流PID控制
ChargePID CapChargePID_Pow;     //功率PID控制
ChargeMonitor CapChargeMonitor; //监控接口

uint16_t CapChargeInVolt12 = 0;  //输入电压12位
uint16_t CapChargeInCurr12 = 0;  //等效输入电流12位
uint16_t CapChargeOutVolt12 = 0; //输出电压12位
uint16_t CapChargeOutCurr12 = 0; //输出电流12位
uint16_t CapChargeOutPow12 = 0;  //输出功率12位
uint16_t CapChargePwmDuty = 0;   //输出PWM占空比(0-9999)

enum ChargeStatus CapChargeStatus = WAIT;

/************************************************************************************/
/************************************** 运算 ****************************************/
/************************************************************************************/

/**
 * @brief  5数冒泡排序函数
 * @param  a： 输入数
 * @param  b： 输入数
 * @note    static
 * @retval NULL
 */
static void CapCharge5NumSort(int *num_arr)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        for (uint8_t j = 0; j < 4 - i; j++)
        {
            if (num_arr[j] > num_arr[j + 1])
            {                                   // 相邻元素两两对比
                uint16_t temp = num_arr[j + 1]; // 元素交换
                num_arr[j + 1] = num_arr[j];
                num_arr[j] = temp;
            }
        }
    }
}

/**
 * @brief  初始化参数
 * @param  val      目标值
 * @note   NULL
 * @retval NULL
 */
void CapchargePID_param_init(ChargePID *pid)
{
    pid->target_val = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->Kp = 0;
    pid->Ki = 0;
    pid->Kd = 0;
    pid->integral = 0;
    pid->output_val = 0;
}

/**
 * @brief   积分分离 PID 控制实现
 * @param  NULL
 * @note    当系统处于启动、结束或大幅增减设定的过程中时，短时间内系统输出有很大的偏差，如此便会在短时间内产生较大的积分累计，
 *          导致控制量超过执行机构可能允许的最大动作范围对应的极限控制量，进而引起系统较大超调，甚至造成系统的震荡。
 *          引入积分分离的目的在于解决上述问题，其基本思想是，当控制量接近给定值时，引入积分控制，消除静态误差；
 *          当控制量与给定值相差较大时，取消积分作用，避免积分累加和过大造成的系统不稳定因素增加。
 * @retval NULL
 */
float SeqIntPIDErrADD = 0.0;
float SeqIntErrBack = 0.0;
float SeqIntPID(ChargePID *pid, float actual_val)
{
    float KpWork, KiWork, KdWork;
    pid->err = pid->target_val - actual_val;
    if (fabs(pid->err) >= pid->target_val * 0.8) // 消除在启动时，积分过大导致超调，误差大于 CHARGE_P * 0.8时I为0。例如50w时，50 * 0.8=40，超过40以上，I不起作用
    {
        pid->output_val = pid->Kp * pid->err + pid->Kd * (pid->err - SeqIntErrBack);
        SeqIntPIDErrADD = 0;
    }
    else
    {
        pid->output_val = pid->Kp * pid->err + pid->Ki * SeqIntPIDErrADD + pid->Kd * (pid->err - SeqIntErrBack);
        SeqIntPIDErrADD = SeqIntPIDErrADD + pid->err;
    }
    SeqIntErrBack = pid->err;
    return pid->output_val;
}

/************************************************************************************/
/************************************** 外设 ****************************************/
/************************************************************************************/

/**
 * @brief      pwm控制输出接口
 * @param      Charge_Value    给定电压
 * @return     void
 * @note       给定电压,转换为占空比控制
 * Sample usage:  Charge_Interface(40) //电压为40
 */
void Charge_Interface(float Charge_Value)
{
    Charge_PWM = LALALA * Charge_Value + 10; //电压转PWM占空比，加8跳过理想二极管导通，可修改
    Charge_Protect();
    ATOM_PWM_SetDuty(CAPCHARGEBridgeIN, Charge_PWM * 100, CAPCHARGEPWMFreq); //设置占空比为百分之Charge_PWM
}

/**
 * @brief   ADC读取滤波函数
 * @param   ADC_DEVICE:    ADC设备号
 * @note    static
 * @retval  ADC读取值
 */
static int ADC_read(int ADC_DEVICE)
{
    int ADC_VAL[5];

    /* 测量及中值滤波 */
    for (uint16_t ii = 0; ii < 5; ii++)
        ADC_VAL[ii] = CapChargeLimit(ADC_Read(ADC_DEVICE), 4095, 0);
    CapCharge5NumSort(ADC_VAL);

    return ADC_VAL[2];
}

/**
 * @brief   输入电压测量
 * @param   NULL
 * @note    NULL
 * @retval  输入电压值
 */
static uint16_t CapChargeInVolt_read()
{
    return (uint16_t)ADC_read(CAPCHARGEInVoltADC);
}

/**
 * @brief   输出电流测量
 * @param   NULL
 * @note    NULL
 * @retval  输出电流值
 */
static uint16_t CapChargeOutCurr_read()
{
    return (uint16_t)ADC_read(CAPCHARGEOutCurrADC);
}

/**
 * @brief   输出电压测量
 * @param   NULL
 * @note    NULL
 * @retval  输出电压值
 */
static uint16_t CapChargeOutVolt_read()
{
    return (uint16_t)ADC_read(CAPCHARGEOutVoltADC);
}

/**
 * @brief   全部测量
 * @param   NULL
 * @note    NULL
 * @retval  NULL
 */
void Get_All(void)
{
    /********** 数据读取 **********/
    CapChargeInVolt12 = CapChargeInVolt_read();
    CapChargeOutVolt12 = CapChargeOutVolt_read();
    CapChargeOutCurr12 = CapChargeOutCurr_read();
    CapChargeOutPow12 = CapChargeOutCurr12 * CapChargeOutVolt12;

    CapChargeMonitor.OutCurr = CapChargeOutCurr12 * 3.3 * CAPCHARGEOutCurrPROP / 4096*1000;
    CapChargeMonitor.InVolt = CapChargeInVolt12 * 3.3 * CAPCHARGEInVoltPROP / 4096*1000;
    CapChargeMonitor.OutVolt = CapChargeOutVolt12 * 3.3 * CAPCHARGEOutVoltPROP / 4096*1000;
    CapChargeMonitor.OutPower = CapChargeMonitor.OutCurr * CapChargeMonitor.OutVolt;
    CapChargeMonitor.InCurr = CapChargeMonitor.OutCurr * CapChargeMonitor.OutVolt / CapChargeMonitor.InVolt;
    if (CapChargeMonitor.InCurr > 20)
        CapChargeMonitor.InCurr = 0;
}

/************************************************************************************/
/************************************** 主体 ****************************************/
/************************************************************************************/

/**
 * @brief   充电保护函数
 * @param   NULL
 * @note    NULL
 * @retval  NULL
 */
uint8_t Charge_protect = 0;
float Charge_PWM = 0;
void Charge_Protect(void)
{
    if (Charge_PWM >= 90) //限制输出最大占空比
    {
        Charge_PWM = 90;
        if (Charge_protect >= 50) //开始计数，当发现达不到功率时，降低功率以保护电路
        {
            Charge_protect = 0;
            CHARGE_P -= 20;
        }
        else
            Charge_protect++;
    }
    else
        Charge_protect = 0;

    if (Charge_PWM <= 0) //限制最小占空比
        Charge_PWM = 0;
}

/**
 * @brief      恒功率充电控制
 * @param      GiveValue       预定值
 * @param      ActualValue     实际值
 * @note       给定一个功率值与实际功率（用电压表示）比较，得出PID控制值，用以控制恒功率充电模块
 *             由于实际功率只能依靠电压值（与占空比成正比）来调节，当给出一个功率误差后，P*t=1/2*C*U*U
 *             得到P与U是非线性。当t变化很小时，近视认为是线性，可以得到功率->电压->占空比线性关系。
 * @retval     NULL
 * Sample usage:               Charge_Ctrl(CHARGE_P, CapChargeMonitor.InVolt*1.0/1000*CapChargeMonitor.InCurr/1000); //给定功率为CHARGE_P，实际功率为CapChargeMonitor.InVolt*1.0/1000*CapChargeMonitor.InCurr/1000
 */
float Charge_Time = 0; //充电持续时间
float CHARGE_TIME = 0; //充电持续时间
uint8_t CHARGE_P;      //设定功率
//void Charge_Ctrl(float GiveValue, float ActualValue)
//{
//    CapChargePID_Pow.output_val = sqrt(2 * GiveValue * (CHARGE_TIME + Charge_Time) / TargetCap); //前馈控制P*t=1/2*C*U*U
//    Charge_Time += CHARGE_INTERRUPT_TIM;                                             //时间累加
//
//    Charge_Interface(CapChargePID_Pow.output_val); //给驱动功率值
//}
void Charge_Ctrl(float GiveValue, float ActualValue)
{
    CapChargePID_Pow.target_val = GiveValue;
    SeqIntPID(&CapChargePID_Pow, ActualValue);
    CapChargePID_Pow.output_val += ActualValue;
    CapChargePID_Pow.output_val = sqrt(2 * CapChargePID_Pow.output_val * (CHARGE_TIME + Charge_Time) / TargetCap); //前馈控制P*t=1/2*C*U*U
    Charge_Time += CHARGE_INTERRUPT_TIM;                                             //时间累加

    Charge_Interface(CapChargePID_Pow.output_val); //给驱动功率值
}

/**
 * @brief   充电PID初始化
 * @param   NULL
 * @note    NULL
 * @retval  NULL
 */
void Chrage_PID_Init()
{
    CapchargePID_param_init(&CapChargePID_Pow);

    CapChargePID_Pow.Kp = POW_KP;
    CapChargePID_Pow.Ki = POW_KI;
    CapChargePID_Pow.Kd = POW_KD;

    CapChargePID_Pow.target_val = TargetOutPower;
}

/**
 * @brief   充电初始化
 * @param   NULL
 * @note    NULL
 * @retval  NULL
 */
void Charge_Init()
{
    ADC_InitConfig(CAPCHARGEInVoltADC, 80000);
    ADC_InitConfig(CAPCHARGEOutCurrADC, 80000);
    ADC_InitConfig(CAPCHARGEOutVoltADC, 80000);
    PIN_InitConfig(CAPCHARGEBridgeSD, PIN_MODE_OUTPUT, MOSDisable);
    ATOM_PWM_InitConfig(CAPCHARGEBridgeIN, 0, CAPCHARGEPWMFreq);

    Chrage_PID_Init();
}

/**
 * @brief   充电开始结束检测函数
 * @param   NULL
 * @note    NULL
 * @retval  NULL
 */
uint8_t Charge_Flag = 0;       //充电开始标志
uint8_t Charge_Flag_start = 0; //真实充电开始标志
uint8_t Charge_Full = 0;       //充满标志
uint8_t j;
void Charge_Check(void)
{
    //充满检测算法。窗口比较，仿抖动
    if (CapChargeMonitor.OutVolt / 1000.0 < TargetOutVolt - 2)
        Charge_Full = 0;
    else if (CapChargeMonitor.OutVolt / 1000.0 > TargetOutVolt)
    {
        if (!Charge_Full)
        {
            //打印时间和电容电压值
            sprintf(monitor_txt, "%.3f,%.3f\r\n", Charge_Time, CapChargeMonitor.OutVolt / 1000.0);
            UART_PutStr(UART0, monitor_txt); //串口发送
        }
        Charge_Full = 1;
    }

    //开始充电判断算法
    if (Charge_Flag && !Charge_Full) //当标志位开启，且未充满时
    {
        //判断是否有输入，若无，则断开50次，开启3次，检测是否有输入
//        if ((CapChargeMonitor.InVolt/1000) < (TargetOutVolt+2))
//        {
//            Charge_Flag_start = 0;
//        }
//        else //满足要求，开始充电
//        {
            Charge_Flag_start = 1;
//        }
    }
    else //关闭充电
        Charge_Flag_start = 0;

    //初始化参数，开始充电
    if (Charge_Flag_start) //开始充电
    {
        if (CapChargeMonitor.OutVolt / 1000 < TargetOutPower / 9 - TargetCap / 2) //减少刚开始充电过冲
            CHARGE_P = TargetOutPower * 0.8;
        else
            CHARGE_P = TargetOutPower;
        PIN_Write(CAPCHARGEBridgeSD, MOSEnable);
        Charge_Ctrl(CHARGE_P, CapChargeMonitor.InVolt * 1.0 / 1000 * CapChargeMonitor.InCurr / 1000); //充电控制
    }
    else //充电未开始，初始化参数
    {
        PIN_Write(CAPCHARGEBridgeSD, MOSDisable);
        Charge_Ctrl(0, 0);
        Charge_Time = 0;
        CHARGE_TIME = CapChargeMonitor.OutVolt / 1000 * CapChargeMonitor.OutVolt / 1000.0 * TargetCap / 100; // P*t=1/2*C*U*U，采集电压，计算大概当前时间.乘一个比例系数
    }
}

/**
 * @brief   充电回调函数
 * @param   NULL
 * @note    在中断中调用即可
 * @retval  NULL
 */
void Charge_Callback(void)
{
    Get_All();
//    if (!Charge_Full)
//    {
        sprintf(monitor_txt, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", CapChargeMonitor.InCurr, CapChargeMonitor.InVolt, CapChargeMonitor.OutVolt, CapChargeMonitor.OutPower, CapChargePID_Pow.output_val, Charge_Time);
        UART_PutStr(UART0, monitor_txt);
//    }

    Charge_Check();
}
