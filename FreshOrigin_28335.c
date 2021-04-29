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

    MemCopy(&RamfuncsLoadStart,&RamfuncsLoadEnd,&RamfuncsRunStart);
    InitFlash();

    EALLOW;
    PieVectTable.ADCINT = &adc_isr;       // ����������Ҫ���ж�ӳ�䵽�ж�������
    EDIS;

    //Step 4. ��ʼ������

    InitAdc();                            // ��ʼ��ADCģ��(ʹ��ADCʱ�ӣ�У��ADC������ADC�ϵ�)
    Setup_Adc();                          // ����ADCģ��
    Setup_ePWM1();                        // ����ePWM1ģ��
    Setup_ePWM2();                        // ����ePWM2ģ��
    Setup_ePWM3();                        // ����ePWM3ģ��
    Setup_ePWM4();                        // ����ePWM4ģ�����ڴ���adc��������ʱ
    EPwm3Regs.TBCTR = 0;                  // ������ͬʱ����
    EPwm2Regs.TBCTR = 0;
    EPwm4Regs.TBCTR = 0;


    //����
    //����spwm��
    for(spwm_count=0;spwm_count<Points_sina;spwm_count++)
    {
        sinaA[spwm_count]=(float)(M*sin(2.0*3.1416*(float)spwm_count/Points_sina));//(float)SP*(1.0+0.8*sin(2.0*3.1416*(float)spwm_count/1000.0))/2.0;
        sinaB[spwm_count]=(float)(M*sin(2.0*3.1416*(float)spwm_count/Points_sina+4.1888));
        sinaC[spwm_count]=(float)(M*sin(2.0*3.1416*(float)spwm_count/Points_sina+2.0944));
    }

    //Step 5. ʹ���ж�2
    IER |= M_INT1;                        // ʹ��CPU�жϣ�ADCINT�ڵ�1���ж�
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;    // ʹ��PIE�жϣ�ADCINT��PIE��1��ĵ�6���ж�
    EINT;                                 // ʹ�����ж� INTM
    ERTM;

    EPwm1Regs.CMPA.half.CMPA =sinaA[0];
    EPwm2Regs.CMPA.half.CMPA =sinaB[0];
    EPwm3Regs.CMPA.half.CMPA =sinaC[0];
    EPwm4Regs.CMPA.half.CMPA =0;

    //Step 6. ѭ���ȴ��ж�
    for(; ;)
    {
        //����
        AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;

        asm("          NOP");
    }

}

#pragma CODE_SECTION(adc_isr, "ramfuncs");
interrupt void  adc_isr(void)
{
    I1a_AD = AdcRegs.ADCRESULT1 >> 4;         // ��ȡADת�����  B0
    I1b_AD = AdcRegs.ADCRESULT3 >> 4;         //��ȡ����              B1
    I1c_AD = AdcRegs.ADCRESULT5 >> 4;         //��ȡ����              B2

    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;          //������λ��������

    I1a = (I1a_AD-2266)*0.0113924 ;             //
    I1b = (I1b_AD-2266)*0.0113924 ;              //
    I1c = (I1c_AD-2252)*0.0113924 ;

    U1_result[Phase_index] = I1a ;//Vm_pre1; //Vm;//�������໷����������ź�

    Phase_index = Phase_index + 1;
    if(Phase_index > (int)(WATCH_LEN-1))
    {Phase_index=0;}

//  ���������ʱ��
    spwm_count++;
    if(spwm_count>=Points_sina)
    {
        spwm_count=0;
    }


    theta=2*3.1416*spwm_count/Points_sina;

    id=0.6667*(cos(theta)*I1a+cos(theta+4.1888)*I1b+cos(theta+2.0944)*I1c);
   // id=cos(theta);
    iq=0.6667*(sin(theta)*I1a+sin(theta+4.1888)*I1b+sin(theta+2.0944)*I1c);
   // i0=1/3*(I1a+I1b+I1c);

    ek_id=id_ref-id;
    Vc_d += Kp_d*(ek_id-ek_idpre) + Ki_d*Tsam* ek_id*Kw_d; // ������Ƶ�ѹ
    ek_idpre = ek_id;

    ek_iq=iq_ref-iq;
    Vc_q += Kp_q*(ek_iq-ek_iqpre) + Ki_q*Tsam* ek_iq*Kw_q; // ������Ƶ�ѹ
    ek_iqpre = ek_iq;

    if(Vc_d > Vc_Max)  {Vc_d = Vc_Max;            // �Ե�������������������޷�
    Kw_d=0;
    }
    else if(Vc_d < Vc_Min)  {Vc_d = Vc_Min;
    Kw_d=0;
    }
    else
    {
    Kw_d=1;
    }

    if(Vc_q > Vc_Max)  {Vc_q = Vc_Max;            // �Ե�������������������޷�
    Kw_q=0;
    }
    else if(Vc_q < Vc_Min)  {Vc_q = Vc_Min;
    Kw_q=0;
    }
    else
    {
    Kw_q=1;
    }



    Vc_phasea =cos(theta)*Vc_d + sin(theta)*Vc_q;
    Vc_phaseb =cos(theta-2.0944)*Vc_d + sin(theta-2.0944)*Vc_q;
    Vc_phasec =cos(theta+2.0944)*Vc_d + sin(theta+2.0944)*Vc_q;



    {start_flag = 1;}

    if (start_flag == 1)
    {

        Vm_InvertA = (1.0 + Vc_phasea)/2.0;
        Vm_InvertB = (1.0 + Vc_phaseb)/2.0;
        Vm_InvertC = (1.0 + Vc_phasec)/2.0;

        if(Vm_InvertA  > 0.95)  Vm_InvertA = 0.95;    // �Ե�������������������޷�
        else if( Vm_InvertA < 0.05)  Vm_InvertA = 0.05;
        if(Vm_InvertB  > 0.95)  Vm_InvertB = 0.95;    // �Ե�������������������޷�
        else if( Vm_InvertB < 0.05)  Vm_InvertB = 0.05;
        if(Vm_InvertC  > 0.95)  Vm_InvertC = 0.95;    // �Ե�������������������޷�
        else if( Vm_InvertC < 0.05)  Vm_InvertC = 0.05;

        EPwm1Regs.CMPA.half.CMPA = (SP*Vm_InvertA); // ���رȽϼĴ�����ֵ   A���Ӧepwm1AB
        EPwm2Regs.CMPA.half.CMPA = (SP*Vm_InvertB); // ���رȽϼĴ�����ֵ   B���Ӧepwm2AB
        EPwm3Regs.CMPA.half.CMPA = (SP*Vm_InvertC); // ���رȽϼĴ�����ֵ   C���Ӧepwm3AB

    }
    // Reinitialize for next ADC sequence
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;   // Acknowledge interrupt to PIE
    //  GpioDataRegs.GPACLEAR.bit.GPIO2=1;//����GPIO����͵�ƽ

}
