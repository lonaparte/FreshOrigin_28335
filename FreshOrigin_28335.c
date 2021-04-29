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

//    MemCopy(&RamfuncsLoadStart,&RamfuncsLoadEnd,&RamfuncsRunStart);
//    InitFlash();

//    EALLOW;
//    PieVectTable.ADCINT = &adc_isr;       // 将程序中需要的中断映射到中断向量表
//    EDIS;

    //Step 6. 循环等待中断
    for(; ;)
    {
        //开环
        AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;

        asm("          NOP");
    }

}
