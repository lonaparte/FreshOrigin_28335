//###########################################################################
//
// FILE:   FreshOrigin_28335.c
//
// TITLE:  The main file of a project to import all linked CCS libraries
//
//###########################################################################
// $TI Release: F2833x Support Library v2.02.00.00 $
// $Release Date: Fri Feb 12 19:15:21 IST 2021 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "epwm.h"
#include "adc.h"
#include "math.h"

//spwm�������ݴ���
int Points_sina=400;
float sinaA[400];//50Hz���ҵ���бȽ�
float sinaB[400];
float sinaC[400];

float Usin_ref = 0;
float Isin_ref =0;
int spwm_count=0;
int counter=0;
float sum=0;
float M=0.8;//���Ʊ�


//PI����������
#define  Kp_V     0.01                // ��ѹ������ϵ��
//#define  tau_V    0.00036             // ��ѹ�����������
#define  Ki_V     2          // ��ѹ������ϵ��
float  Kp_d=0.01;         // ����������ϵ��
float  Ki_d=100;          // ����������ϵ��
float  Kp_q=0.01;          // ����������ϵ��
float  Ki_q=100;          // ����������ϵ��



//�޷�ֵ
#define  Iref_Max    2.3                 // ��ѹ������޷�
#define  Iref_Min   -2.3
#define  Vc_Invert_Max    0.1               // ����������޷�
#define  Vc_Invert_Min   -0.1
//#define  Umax       250                 // ��ѹ������ֵ
#define  Imax       4.5                // ����������ֵ



/*-------------------------------�������弰��ʼ��----------------------------------------*/
float  I1a=0;                          // ֱ�����ѹ
float  I1b=0;
float  I1c=0;
float  theta=0;

float  Usign=0;
float  IL_Invert=0;                          // ��е���


float  Uacavl=0;
int    I1a_AD=0;
int    I1b_AD=0;
int    I1c_AD=0;
int    Usign_AD=0;



int count=0;
//float U0_result[2000];
#define WATCH_LEN 2000
float U1_result[2000];

int Uo_resultave=0;
int Ul_resultave=0;
int I_resultave=0;



int    Phase_index = 0;               // ����ָ��
float  id=0;
float  iq=0;
float  i0=0;
float  id_ref = 4;                     //
float  iq_ref = 0;                     //
float  ek_id=0;
float  ek_iq=0;
float  ek_idpre=0;
float  ek_iqpre=0;

float  Vc_d=0;
float  Vc_q=0;
float  Vc_phasea=0;
float  Vc_phaseb=0;
float  Vc_phasec=0;
float  Kw_d=1;
float  Kw_q=1;
float  Vc_Max=1;
float  Vc_Min=0;

float  ek_Uo=0, ek_Uo_pre=0;          // ����ѹ e(k)��e(k-1)
float  ek_IL=0, ek_IL_pre=0;          // ������ e(k)��e(k-1)
float  Vc_Invert = 0;                          // ���Ʋ���ֵ Vc(t)
float  Vm_InvertA = 0;                          // ʵ�ʵ��Ʋ�
float  Vm_InvertB = 0;                          // ʵ�ʵ��Ʋ�
float  Vm_InvertC = 0;                          // ʵ�ʵ��Ʋ�

int    start_flag = 0;                // ������־��Ϊ1��ʾװ����������
int    protect_flag = 0;              // ������־��Ϊ1��ʾ����������


#define  PWM_CLK_FRQ  20e3                //
// pll variables
/*******************************************************************************

 * �� �� ��         : LED_Init
 * ��������         : LED��ʼ������
 * ��    ��         : ��
 * ��    ��         : ��
 *******************************************************************************/
void LED_Init(void)
{
    EALLOW;//�ر�д����
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;    // ����GPIOʱ��

    //LED1�˿�����
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;//����Ϊͨ��GPIO����
    GpioCtrlRegs.GPADIR.bit.GPIO0=1;//����GPIO����Ϊ���
    GpioCtrlRegs.GPAPUD.bit.GPIO0=0;//ʹ��GPIO��������

    GpioDataRegs.GPASET.bit.GPIO0=1;//����GPIO����ߵ�ƽ

    EDIS;//����д����
}



/*******************************************************************************
 * �� �� ��         : main
 * ��������         : ������
 * ��    ��         : ��
 * ��    ��         : ��
 *******************************************************************************/
void main()

{

    //Step 1. ��ʼ��ϵͳ����
    InitSysCtrl();//ϵͳʱ�ӳ�ʼ����Ĭ���ѿ���F28335��������ʱ��



    //Step 2. ��ʼ��GPIO
    InitEPwm1Gpio();                      // �� GPIO0 �� GPIO1 ����ΪePWM����
    InitEPwm2Gpio();                     // ��GPIO2 ��GPIO3����ΪePWM����
    InitEPwm3Gpio();                      // ��GPIO4 ��GPIO5 ����ΪePWM���ܣ�����������

    //Step 3. ��������жϲ���ʼ���ж�������
    DINT;                                 // ����CPU���ж�
    InitPieCtrl();                        // �ر�����PIEģ����жϣ��������PIE�жϱ�־λ
    IER = 0x0000;                         // ����CPU�жϲ�����жϱ�־
    IFR = 0x0000;
    InitPieVectTable();                   // ��ʼ���ж�������

//    MemCopy(&RamfuncsLoadStart,&RamfuncsLoadEnd,&RamfuncsRunStart);
//    InitFlash();

//    EALLOW;
//    PieVectTable.ADCINT = &adc_isr;       // ����������Ҫ���ж�ӳ�䵽�ж�������
//    EDIS;

    //Step 6. ѭ���ȴ��ж�
    for(; ;)
    {
        //����
        AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;

        asm("          NOP");
    }

}
