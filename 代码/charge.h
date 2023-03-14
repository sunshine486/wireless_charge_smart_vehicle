/**
******************************************************************************
* @file             : CapCharge.h
* @brief            : ���߳����Ƴ���ͷ�ļ�
* @author           : �������Ǵ�ɶֽ
* @date             : 2022.8.18
* @version          : 1.1
* @note             : �������ݺ㹦�ʳ�磬�������Ⱥ������ﵽ�趨���ʺ�㹦�ʣ��ﵽ�趨��ѹ���ѹ
* @note             : ��ǰ����Ӳ��-HGF����V8.1
* @note             : ʹ�ñ�׼��λֵ��������(��ʹ��3.3V����4096)
******************************************************************************
*/
#ifndef _CHARGE_H
#define _CHARGE_H

#include <stdio.h>
#include <stdint.h>
#include <math.h>

extern uint8_t Charge_Flag; //����־
extern uint8_t Charge_Flag_start;
extern uint8_t CHARGE_P;

/* �豸�ţ��˿�/���ţ����� */
#define CAPCHARGEBridgeSD P00_7
#define CAPCHARGEBridgeIN IfxGtm_ATOM0_5_TOUT15_P00_6_OUT
#define CAPCHARGEInVoltADC ADC36
#define CAPCHARGEOutVoltADC ADC37
#define CAPCHARGEOutCurrADC ADC38

/* ADC�������ű��� */
#define CAPCHARGEInVoltPROP 28    // 27k,1k
#define CAPCHARGEOutCurrPROP 3.33 // INA180A3,100��,0.003R:(1/100*0.003)
#define CAPCHARGEOutVoltPROP 5.7  // 4.7k,1k:((1k+4.7k)/1k)

/* ADC�����޷� */
#define CAPCHARGEInVoltLimit 90
#define CAPCHARGEOutCurrLimit 12
#define CAPCHARGEOutVoltLimit 15

/* �ж�ʱ��,S */
#define CHARGE_INTERRUPT_TIM 0.005

/* PWM����Ƶ��,Hz */
#define CAPCHARGEPWMFreq 100000

/* ����ʹ�ܼ��� */
#define MOSEnable 0 // V8��������˷�����
#define MOSDisable 1

/* �޷����� */
#define CapChargeLimit(num, max, min) (num > min ? num : min) < max ? num : max

/* PID�ṹ�� */
typedef struct
{
    float target_val; //Ŀ��ֵ
    float err;        //ƫ��ֵ
    float err_last;   //��һ��ƫ��ֵ
    float Kp, Ki, Kd;
    float integral;   //����ֵ
    float output_val; //���ֵ
} ChargePID;

/* ���״̬ö�� */
enum ChargeStatus //���״̬ö��
{
    WAIT = 1,    //δ���
    CHARGE_CURR, //���ں�������
    CHARGE_POW,  //���ں㹦�ʳ��
    CHARGE_VOLT, //���ں�ѹ���
    PROTECT,     //���ϱ���
};

/* ������ */
typedef struct
{
    float InVolt;   //�����ѹ
    float InCurr;   //�������
    float OutVolt;  //�����ѹ
    float OutCurr;  //�������
    float OutPower; //�������
} ChargeMonitor;

extern uint8_t Charge_Flag��
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
