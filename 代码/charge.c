/**
******************************************************************************
* @file             : CapCharge.c
* @brief            : ���߳����Ƴ�������
* @author           : �������Ǵ�ɶֽ
* @date             : 2022.8.18
* @version          : 1.1
* @note             : �������ݺ㹦�ʳ�磬�������Ⱥ������ﵽ�趨���ʺ�㹦�ʣ��ﵽ�趨��ѹ���ѹ
* @note             : ��ǰ����Ӳ��-HGF����V8.1
* @note             : ʹ�ñ�׼��λֵ��������(��ʹ��3.3V����4096)
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

/////////////////////////////  ���ڲ���  /////////////////////////////
uint16_t TargetCap = 2;       //Ŀ���������2F
uint16_t TargetOutPower = 58; //Ŀ���������W
uint16_t TargetOutVolt = 12;  //Ŀ�������ѹV

/* PID��ʼ�������� */
float POW_KP = 2, POW_KI = 0.1, POW_KD = 0;
float LALALA = 3;
//////////////////////////////  ����  //////////////////////////////
char monitor_txt[200];

ChargePID CapChargePID_Cur;     //����PID����
ChargePID CapChargePID_Pow;     //����PID����
ChargeMonitor CapChargeMonitor; //��ؽӿ�

uint16_t CapChargeInVolt12 = 0;  //�����ѹ12λ
uint16_t CapChargeInCurr12 = 0;  //��Ч�������12λ
uint16_t CapChargeOutVolt12 = 0; //�����ѹ12λ
uint16_t CapChargeOutCurr12 = 0; //�������12λ
uint16_t CapChargeOutPow12 = 0;  //�������12λ
uint16_t CapChargePwmDuty = 0;   //���PWMռ�ձ�(0-9999)

enum ChargeStatus CapChargeStatus = WAIT;

/************************************************************************************/
/************************************** ���� ****************************************/
/************************************************************************************/

/**
 * @brief  5��ð��������
 * @param  a�� ������
 * @param  b�� ������
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
            {                                   // ����Ԫ�������Ա�
                uint16_t temp = num_arr[j + 1]; // Ԫ�ؽ���
                num_arr[j + 1] = num_arr[j];
                num_arr[j] = temp;
            }
        }
    }
}

/**
 * @brief  ��ʼ������
 * @param  val      Ŀ��ֵ
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
 * @brief   ���ַ��� PID ����ʵ��
 * @param  NULL
 * @note    ��ϵͳ�����������������������趨�Ĺ�����ʱ����ʱ����ϵͳ����кܴ��ƫ���˱���ڶ�ʱ���ڲ����ϴ�Ļ����ۼƣ�
 *          ���¿���������ִ�л�������������������Χ��Ӧ�ļ��޿���������������ϵͳ�ϴ󳬵����������ϵͳ���𵴡�
 *          ������ַ����Ŀ�����ڽ���������⣬�����˼���ǣ����������ӽ�����ֵʱ��������ֿ��ƣ�������̬��
 *          �������������ֵ���ϴ�ʱ��ȡ���������ã���������ۼӺ͹�����ɵ�ϵͳ���ȶ��������ӡ�
 * @retval NULL
 */
float SeqIntPIDErrADD = 0.0;
float SeqIntErrBack = 0.0;
float SeqIntPID(ChargePID *pid, float actual_val)
{
    float KpWork, KiWork, KdWork;
    pid->err = pid->target_val - actual_val;
    if (fabs(pid->err) >= pid->target_val * 0.8) // ����������ʱ�����ֹ����³����������� CHARGE_P * 0.8ʱIΪ0������50wʱ��50 * 0.8=40������40���ϣ�I��������
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
/************************************** ���� ****************************************/
/************************************************************************************/

/**
 * @brief      pwm��������ӿ�
 * @param      Charge_Value    ������ѹ
 * @return     void
 * @note       ������ѹ,ת��Ϊռ�ձȿ���
 * Sample usage:  Charge_Interface(40) //��ѹΪ40
 */
void Charge_Interface(float Charge_Value)
{
    Charge_PWM = LALALA * Charge_Value + 10; //��ѹתPWMռ�ձȣ���8������������ܵ�ͨ�����޸�
    Charge_Protect();
    ATOM_PWM_SetDuty(CAPCHARGEBridgeIN, Charge_PWM * 100, CAPCHARGEPWMFreq); //����ռ�ձ�Ϊ�ٷ�֮Charge_PWM
}

/**
 * @brief   ADC��ȡ�˲�����
 * @param   ADC_DEVICE:    ADC�豸��
 * @note    static
 * @retval  ADC��ȡֵ
 */
static int ADC_read(int ADC_DEVICE)
{
    int ADC_VAL[5];

    /* ��������ֵ�˲� */
    for (uint16_t ii = 0; ii < 5; ii++)
        ADC_VAL[ii] = CapChargeLimit(ADC_Read(ADC_DEVICE), 4095, 0);
    CapCharge5NumSort(ADC_VAL);

    return ADC_VAL[2];
}

/**
 * @brief   �����ѹ����
 * @param   NULL
 * @note    NULL
 * @retval  �����ѹֵ
 */
static uint16_t CapChargeInVolt_read()
{
    return (uint16_t)ADC_read(CAPCHARGEInVoltADC);
}

/**
 * @brief   �����������
 * @param   NULL
 * @note    NULL
 * @retval  �������ֵ
 */
static uint16_t CapChargeOutCurr_read()
{
    return (uint16_t)ADC_read(CAPCHARGEOutCurrADC);
}

/**
 * @brief   �����ѹ����
 * @param   NULL
 * @note    NULL
 * @retval  �����ѹֵ
 */
static uint16_t CapChargeOutVolt_read()
{
    return (uint16_t)ADC_read(CAPCHARGEOutVoltADC);
}

/**
 * @brief   ȫ������
 * @param   NULL
 * @note    NULL
 * @retval  NULL
 */
void Get_All(void)
{
    /********** ���ݶ�ȡ **********/
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
/************************************** ���� ****************************************/
/************************************************************************************/

/**
 * @brief   ��籣������
 * @param   NULL
 * @note    NULL
 * @retval  NULL
 */
uint8_t Charge_protect = 0;
float Charge_PWM = 0;
void Charge_Protect(void)
{
    if (Charge_PWM >= 90) //����������ռ�ձ�
    {
        Charge_PWM = 90;
        if (Charge_protect >= 50) //��ʼ�����������ִﲻ������ʱ�����͹����Ա�����·
        {
            Charge_protect = 0;
            CHARGE_P -= 20;
        }
        else
            Charge_protect++;
    }
    else
        Charge_protect = 0;

    if (Charge_PWM <= 0) //������Сռ�ձ�
        Charge_PWM = 0;
}

/**
 * @brief      �㹦�ʳ�����
 * @param      GiveValue       Ԥ��ֵ
 * @param      ActualValue     ʵ��ֵ
 * @note       ����һ������ֵ��ʵ�ʹ��ʣ��õ�ѹ��ʾ���Ƚϣ��ó�PID����ֵ�����Կ��ƺ㹦�ʳ��ģ��
 *             ����ʵ�ʹ���ֻ��������ѹֵ����ռ�ձȳ����ȣ������ڣ�������һ����������P*t=1/2*C*U*U
 *             �õ�P��U�Ƿ����ԡ���t�仯��Сʱ��������Ϊ�����ԣ����Եõ�����->��ѹ->ռ�ձ����Թ�ϵ��
 * @retval     NULL
 * Sample usage:               Charge_Ctrl(CHARGE_P, CapChargeMonitor.InVolt*1.0/1000*CapChargeMonitor.InCurr/1000); //��������ΪCHARGE_P��ʵ�ʹ���ΪCapChargeMonitor.InVolt*1.0/1000*CapChargeMonitor.InCurr/1000
 */
float Charge_Time = 0; //������ʱ��
float CHARGE_TIME = 0; //������ʱ��
uint8_t CHARGE_P;      //�趨����
//void Charge_Ctrl(float GiveValue, float ActualValue)
//{
//    CapChargePID_Pow.output_val = sqrt(2 * GiveValue * (CHARGE_TIME + Charge_Time) / TargetCap); //ǰ������P*t=1/2*C*U*U
//    Charge_Time += CHARGE_INTERRUPT_TIM;                                             //ʱ���ۼ�
//
//    Charge_Interface(CapChargePID_Pow.output_val); //����������ֵ
//}
void Charge_Ctrl(float GiveValue, float ActualValue)
{
    CapChargePID_Pow.target_val = GiveValue;
    SeqIntPID(&CapChargePID_Pow, ActualValue);
    CapChargePID_Pow.output_val += ActualValue;
    CapChargePID_Pow.output_val = sqrt(2 * CapChargePID_Pow.output_val * (CHARGE_TIME + Charge_Time) / TargetCap); //ǰ������P*t=1/2*C*U*U
    Charge_Time += CHARGE_INTERRUPT_TIM;                                             //ʱ���ۼ�

    Charge_Interface(CapChargePID_Pow.output_val); //����������ֵ
}

/**
 * @brief   ���PID��ʼ��
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
 * @brief   ����ʼ��
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
 * @brief   ��翪ʼ������⺯��
 * @param   NULL
 * @note    NULL
 * @retval  NULL
 */
uint8_t Charge_Flag = 0;       //��翪ʼ��־
uint8_t Charge_Flag_start = 0; //��ʵ��翪ʼ��־
uint8_t Charge_Full = 0;       //������־
uint8_t j;
void Charge_Check(void)
{
    //��������㷨�����ڱȽϣ��¶���
    if (CapChargeMonitor.OutVolt / 1000.0 < TargetOutVolt - 2)
        Charge_Full = 0;
    else if (CapChargeMonitor.OutVolt / 1000.0 > TargetOutVolt)
    {
        if (!Charge_Full)
        {
            //��ӡʱ��͵��ݵ�ѹֵ
            sprintf(monitor_txt, "%.3f,%.3f\r\n", Charge_Time, CapChargeMonitor.OutVolt / 1000.0);
            UART_PutStr(UART0, monitor_txt); //���ڷ���
        }
        Charge_Full = 1;
    }

    //��ʼ����ж��㷨
    if (Charge_Flag && !Charge_Full) //����־λ��������δ����ʱ
    {
        //�ж��Ƿ������룬���ޣ���Ͽ�50�Σ�����3�Σ�����Ƿ�������
//        if ((CapChargeMonitor.InVolt/1000) < (TargetOutVolt+2))
//        {
//            Charge_Flag_start = 0;
//        }
//        else //����Ҫ�󣬿�ʼ���
//        {
            Charge_Flag_start = 1;
//        }
    }
    else //�رճ��
        Charge_Flag_start = 0;

    //��ʼ����������ʼ���
    if (Charge_Flag_start) //��ʼ���
    {
        if (CapChargeMonitor.OutVolt / 1000 < TargetOutPower / 9 - TargetCap / 2) //���ٸտ�ʼ������
            CHARGE_P = TargetOutPower * 0.8;
        else
            CHARGE_P = TargetOutPower;
        PIN_Write(CAPCHARGEBridgeSD, MOSEnable);
        Charge_Ctrl(CHARGE_P, CapChargeMonitor.InVolt * 1.0 / 1000 * CapChargeMonitor.InCurr / 1000); //������
    }
    else //���δ��ʼ����ʼ������
    {
        PIN_Write(CAPCHARGEBridgeSD, MOSDisable);
        Charge_Ctrl(0, 0);
        Charge_Time = 0;
        CHARGE_TIME = CapChargeMonitor.OutVolt / 1000 * CapChargeMonitor.OutVolt / 1000.0 * TargetCap / 100; // P*t=1/2*C*U*U���ɼ���ѹ�������ŵ�ǰʱ��.��һ������ϵ��
    }
}

/**
 * @brief   ���ص�����
 * @param   NULL
 * @note    ���ж��е��ü���
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
