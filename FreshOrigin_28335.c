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

//spwm波的数据储存
int Points_sina=400;
float sinaA[400];//50Hz正弦点进行比较
float sinaB[400];
float sinaC[400];

float Usin_ref = 0;
float Isin_ref =0;
int spwm_count=0;
int counter=0;
float sum=0;
float M=0.8;//调制比


//PI调节器参数
#define  Kp_V     0.01                // 电压环比例系数
//#define  tau_V    0.00036             // 电压环补偿器零点
#define  Ki_V     2          // 电压环积分系数
float  Kp_d=0.01;         // 电流环比例系数
float  Ki_d=100;          // 电流环积分系数
float  Kp_q=0.01;          // 电流环积分系数
float  Ki_q=100;          // 电流环积分系数



//限幅值
#define  Iref_Max    2.3                 // 电压环输出限幅
#define  Iref_Min   -2.3
#define  Vc_Invert_Max    0.1               // 电流环输出限幅
#define  Vc_Invert_Min   -0.1
//#define  Umax       250                 // 过压保护阈值
#define  Imax       4.5                // 过流保护阈值



/*-------------------------------变量定义及初始化----------------------------------------*/
float  I1a=0;                          // 直流侧电压
float  I1b=0;
float  I1c=0;
float  theta=0;

float  Usign=0;
float  IL_Invert=0;                          // 电感电流


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



int    Phase_index = 0;               // 数组指针
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

float  ek_Uo=0, ek_Uo_pre=0;          // 误差电压 e(k)和e(k-1)
float  ek_IL=0, ek_IL_pre=0;          // 误差电流 e(k)和e(k-1)
float  Vc_Invert = 0;                          // 调制波幅值 Vc(t)
float  Vm_InvertA = 0;                          // 实际调制波
float  Vm_InvertB = 0;                          // 实际调制波
float  Vm_InvertC = 0;                          // 实际调制波

int    start_flag = 0;                // 启动标志，为1表示装置正常运行
int    protect_flag = 0;              // 保护标志，为1表示已启动保护

#define  PWM_CLK_FRQ  20e3                //
// pll variables
/*******************************************************************************

 * 函 数 名         : LED_Init
 * 函数功能         : LED初始化函数
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void LED_Init(void)
{
    EALLOW;//关闭写保护
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;    // 开启GPIO时钟

    //LED1端口配置
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;//设置为通用GPIO功能
    GpioCtrlRegs.GPADIR.bit.GPIO0=1;//设置GPIO方向为输出
    GpioCtrlRegs.GPAPUD.bit.GPIO0=0;//使能GPIO上拉电阻

    GpioDataRegs.GPASET.bit.GPIO0=1;//设置GPIO输出高电平

    EDIS;//开启写保护
}



/*******************************************************************************
 * 函 数 名         : main
 * 函数功能         : 主函数
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void main()

{

    //Step 1. 初始化系统控制
    InitSysCtrl();//系统时钟初始化，默认已开启F28335所有外设时钟



    //Step 2. 初始化GPIO
    InitEPwm1Gpio();                      // 将 GPIO0 和 GPIO1 配置为ePWM功能
    InitEPwm2Gpio();                     // 将GPIO2 和GPIO3配置为ePWM功能
    InitEPwm3Gpio();                      // 将GPIO4 和GPIO5 配置为ePWM功能，仅触发采样

    //Step 3. 清除所有中断并初始化中断向量表
    DINT;                                 // 禁用CPU总中断
    InitPieCtrl();                        // 关闭所有PIE模块的中断，清除所有PIE中断标志位
    IER = 0x0000;                         // 禁用CPU中断并清除中断标志
    IFR = 0x0000;
    InitPieVectTable();                   // 初始化中断向量表

    MemCopy(&RamfuncsLoadStart,&RamfuncsLoadEnd,&RamfuncsRunStart);
    InitFlash();

    EALLOW;
    PieVectTable.ADCINT = &adc_isr;       // 将程序中需要的中断映射到中断向量表
    EDIS;

    //Step 4. 初始化外设

    InitAdc();                            // 初始化ADC模块(使能ADC时钟，校验ADC，并给ADC上电)
    Setup_Adc();                          // 配置ADC模块
    Setup_ePWM1();                        // 配置ePWM1模块
    Setup_ePWM2();                        // 配置ePWM2模块
    Setup_ePWM3();                        // 配置ePWM3模块
    Setup_ePWM4();                        // 配置ePWM4模块用于触发adc，单纯计时
    EPwm3Regs.TBCTR = 0;                  // 计数器同时清零
    EPwm2Regs.TBCTR = 0;
    EPwm4Regs.TBCTR = 0;


    //开环
    //生成spwm表
    for(spwm_count=0;spwm_count<Points_sina;spwm_count++)
    {
        sinaA[spwm_count]=(float)(M*sin(2.0*3.1416*(float)spwm_count/Points_sina));//(float)SP*(1.0+0.8*sin(2.0*3.1416*(float)spwm_count/1000.0))/2.0;
        sinaB[spwm_count]=(float)(M*sin(2.0*3.1416*(float)spwm_count/Points_sina+4.1888));
        sinaC[spwm_count]=(float)(M*sin(2.0*3.1416*(float)spwm_count/Points_sina+2.0944));
    }

    //Step 5. 使能中断2
    IER |= M_INT1;                        // 使能CPU中断：ADCINT在第1组中断
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;    // 使能PIE中断：ADCINT是PIE第1组的第6个中断
    EINT;                                 // 使能总中断 INTM
    ERTM;

    EPwm1Regs.CMPA.half.CMPA =sinaA[0];
    EPwm2Regs.CMPA.half.CMPA =sinaB[0];
    EPwm3Regs.CMPA.half.CMPA =sinaC[0];
    EPwm4Regs.CMPA.half.CMPA =0;

    //Step 6. 循环等待中断
    for(; ;)
    {
        //开环
        AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;

        asm("          NOP");
    }

}

#pragma CODE_SECTION(adc_isr, "ramfuncs");
interrupt void  adc_isr(void)
{
    I1a_AD = AdcRegs.ADCRESULT1 >> 4;         // 读取AD转换结果  B0
    I1b_AD = AdcRegs.ADCRESULT3 >> 4;         //读取电流              B1
    I1c_AD = AdcRegs.ADCRESULT5 >> 4;         //读取电流              B2

    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;          //立即复位到欲触发

    I1a = (I1a_AD-2266)*0.0113924 ;             //
    I1b = (I1b_AD-2266)*0.0113924 ;              //
    I1c = (I1c_AD-2252)*0.0113924 ;

    U1_result[Phase_index] = I1a ;//Vm_pre1; //Vm;//储存锁相环输出的锁相信号

    Phase_index = Phase_index + 1;
    if(Phase_index > (int)(WATCH_LEN-1))
    {Phase_index=0;}

//  调开环逆变时用
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
    Vc_d += Kp_d*(ek_id-ek_idpre) + Ki_d*Tsam* ek_id*Kw_d; // 计算控制电压
    ek_idpre = ek_id;

    ek_iq=iq_ref-iq;
    Vc_q += Kp_q*(ek_iq-ek_iqpre) + Ki_q*Tsam* ek_iq*Kw_q; // 计算控制电压
    ek_iqpre = ek_iq;

    if(Vc_d > Vc_Max)  {Vc_d = Vc_Max;            // 对电流调节器的输出进行限幅
    Kw_d=0;
    }
    else if(Vc_d < Vc_Min)  {Vc_d = Vc_Min;
    Kw_d=0;
    }
    else
    {
    Kw_d=1;
    }

    if(Vc_q > Vc_Max)  {Vc_q = Vc_Max;            // 对电流调节器的输出进行限幅
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

        if(Vm_InvertA  > 0.95)  Vm_InvertA = 0.95;    // 对电流调节器的输出进行限幅
        else if( Vm_InvertA < 0.05)  Vm_InvertA = 0.05;
        if(Vm_InvertB  > 0.95)  Vm_InvertB = 0.95;    // 对电流调节器的输出进行限幅
        else if( Vm_InvertB < 0.05)  Vm_InvertB = 0.05;
        if(Vm_InvertC  > 0.95)  Vm_InvertC = 0.95;    // 对电流调节器的输出进行限幅
        else if( Vm_InvertC < 0.05)  Vm_InvertC = 0.05;

        EPwm1Regs.CMPA.half.CMPA = (SP*Vm_InvertA); // 加载比较寄存器的值   A相对应epwm1AB
        EPwm2Regs.CMPA.half.CMPA = (SP*Vm_InvertB); // 加载比较寄存器的值   B相对应epwm2AB
        EPwm3Regs.CMPA.half.CMPA = (SP*Vm_InvertC); // 加载比较寄存器的值   C相对应epwm3AB

    }
    // Reinitialize for next ADC sequence
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;   // Acknowledge interrupt to PIE
    //  GpioDataRegs.GPACLEAR.bit.GPIO2=1;//设置GPIO输出低电平

}
