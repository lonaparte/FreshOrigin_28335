/*
 * epwm.c
 *
 *  Created on: 2018-1-26
 *      Author: Administrator
 */


#include "epwm.h"



interrupt void epwm1_isr(void);
void EPWM1_Init(Uint16 tbprd)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Disable TBCLK within the ePWM
    SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // ePWM1
    EDIS;

    InitEPwm1Gpio();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;

    // Setup Sync
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;  // Pass through
    // Allow each timer to be sync'ed
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm1Regs.TBPHS.half.TBPHS = 0;
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm1Regs.TBPRD = tbprd;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV=TB_DIV1;

    // Setup shadow register load on ZERO
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set Compare values
    EPwm1Regs.CMPA.half.CMPA = 0;    // Set compare A value
    EPwm1Regs.CMPB = 0;              // Set Compare B value

    // Set actions
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;          // Clear PWM1B on event B, up count

    // Active Low PWMs - Setup Deadband
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LO;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED = 1000;
    EPwm1Regs.DBFED = 0;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;  // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;         // Start all the timers synced
    EDIS;

    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    IER |= M_INT3;

    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
}

interrupt void epwm1_isr(void)
{
    static Uint16 cnt=0;

    cnt++;
    if(cnt==5000)
    {
        cnt=0;

    }

    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


void EPwm1A_SetCompare(Uint16 val)
{
    EPwm1Regs.CMPA.half.CMPA = val;  //设置占空比
}
void EPwm1B_SetCompare(Uint16 val)
{
    EPwm1Regs.CMPB = val;  //设置占空比
}

void Setup_ePWM1(void)
{
    //时基模块TB
    EPwm1Regs.TBPRD = SP;               // 周期值 SP = CPU_CLK/(2*PWM_CLK)
    EPwm1Regs.TBPHS.half.TBPHS = 0;     // 相位为0
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;  // 时基时钟 TBCLK=SYSCLKOUT=1/150M
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTL.bit.SYNCOSEL=0;     //同步信号
    EPwm1Regs.TBCTL.bit.PHSEN = 0;      // 禁用相位寄存器
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;    // 增减计数

    //计数比较模块CC
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0; // 影子装载模式
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 2; // CTR = 0 或 CTR = PRD时装载
    EPwm1Regs.CMPA.half.CMPA =0;        // 设置初始占空比为0

    //动作模块AQ
    EPwm1Regs.AQCTLA.bit.CAU = 1;       // 向上计数且CTR=CMPA时，ePWM1A置低
    EPwm1Regs.AQCTLA.bit.CAD = 2;       // 向下计数且CTR=CMPA时，ePWM1A置高

    //死区产生模块DB
    EPwm1Regs.DBCTL.bit.IN_MODE = 0;    // ePWM1A是双边沿输入
    EPwm1Regs.DBCTL.bit.POLSEL = 2;     // ePWM1A不翻转，ePWM1B翻转
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0x3;   // 使能双边沿延时(仅使用A)
    EPwm1Regs.DBRED = 30;               // 上升沿延时 DBRED*TBCLK = 500ns
    EPwm1Regs.DBFED = 30;               // 下降沿延时 DBRED*TBCLK = 500ns

    //错误联防模块TZ
    EALLOW;
    EPwm1Regs.TZCTL.bit.TZA = 30;        // 错误事件发生时，强制ePWM1A低状态
    EPwm1Regs.TZCTL.bit.TZB = 30;        // 错误事件发生时，强制ePWM1B低状态
    EDIS;
}

void Setup_ePWM2(void)
{
    //时基模块TB
    EPwm2Regs.TBPRD = SP;               // 周期值 SP = CPU_CLK/(2*PWM_CLK)
    EPwm2Regs.TBPHS.half.TBPHS = 0;     // 相位为0
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;  // 时基时钟 TBCLK=SYSCLKOUT=1/150M
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTL.bit.SYNCOSEL=0;     //同步信号
    EPwm2Regs.TBCTL.bit.PHSEN = 0;      // 禁用相位寄存器
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;    // 增减计数

    //计数比较模块CC
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0; // 影子装载模式
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 2; // CTR = 0 或 CTR = PRD时装载
    EPwm2Regs.CMPA.half.CMPA =0;        // 设置初始占空比为0

    //动作模块AQ
    EPwm2Regs.AQCTLA.bit.CAU = 1;       // 向上计数且CTR=CMPA时，ePWM1A置低
    EPwm2Regs.AQCTLA.bit.CAD = 2;       // 向下计数且CTR=CMPA时，ePWM1A置高

    //死区产生模块DB
    EPwm2Regs.DBCTL.bit.IN_MODE = 0;    // ePWM1A是双边沿输入
    EPwm2Regs.DBCTL.bit.POLSEL = 2;     // ePWM1A不翻转，ePWM1B翻转
    EPwm2Regs.DBCTL.bit.OUT_MODE = 0x3;   // 使能双边沿延时(仅使用A)
    EPwm2Regs.DBRED = 30;               // 上升沿延时 DBRED*TBCLK = 500ns
    EPwm2Regs.DBFED = 30;               // 下降沿延时 DBRED*TBCLK = 500ns

    //错误联防模块TZ
    EALLOW;
    EPwm2Regs.TZCTL.bit.TZA = 30;        // 错误事件发生时，强制ePWM1A低状态
    EPwm2Regs.TZCTL.bit.TZB = 30;        // 错误事件发生时，强制ePWM1B低状态
    EDIS;
}
void Setup_ePWM3(void)
{
    //时基模块TB
    EPwm3Regs.TBPRD = SP;               // 周期值 SP = CPU_CLK/(2*PWM_CLK)
    EPwm3Regs.TBPHS.half.TBPHS = 0;     // 相位为0
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;  // 时基时钟 TBCLK=SYSCLKOUT=1/150M
    EPwm3Regs.TBCTL.bit.CLKDIV = 0;
    EPwm3Regs.TBCTL.bit.SYNCOSEL=0;     //同步信号
    EPwm3Regs.TBCTL.bit.PHSEN = 0;      // 禁用相位寄存器
    EPwm3Regs.TBCTL.bit.CTRMODE = 2;    // 增减计数

    //计数比较模块CC
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0; // 影子装载模式
    EPwm3Regs.CMPCTL.bit.LOADAMODE = 2; // CTR = 0 或 CTR = PRD时装载
    EPwm3Regs.CMPA.half.CMPA =0;        // 设置初始占空比为0

    //动作模块AQ
    EPwm3Regs.AQCTLA.bit.CAU = 1;       // 向上计数且CTR=CMPA时，ePWM1A置低
    EPwm3Regs.AQCTLA.bit.CAD = 2;       // 向下计数且CTR=CMPA时，ePWM1A置高

    //死区产生模块DB
    EPwm3Regs.DBCTL.bit.IN_MODE = 0;    // ePWM1A是双边沿输入
    EPwm3Regs.DBCTL.bit.POLSEL = 2;     // ePWM1A不翻转，ePWM1B翻转
    EPwm3Regs.DBCTL.bit.OUT_MODE = 0x3;   // 使能双边沿延时(仅使用A)
    EPwm3Regs.DBRED = 30;               // 上升沿延时 DBRED*TBCLK = 500ns
    EPwm3Regs.DBFED = 30;               // 下降沿延时 DBRED*TBCLK = 500ns

    //错误联防模块TZ
    EALLOW;
    EPwm3Regs.TZCTL.bit.TZA = 30;        // 错误事件发生时，强制ePWM1A低状态
    EPwm3Regs.TZCTL.bit.TZB = 30;        // 错误事件发生时，强制ePWM1B低状态
    EDIS;
}

void Setup_ePWM4(void)  // ePWM2模块配置子函数
{
    //时基模块TB
    EPwm4Regs.TBPRD = 1*SP;               // 周期值 SP = CPU_CLK/(2*PWM_CLK)
    EPwm4Regs.TBPHS.half.TBPHS = 0;     // 相位为0
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;  // 时基时钟 TBCLK=SYSCLKOUT=1/150M
    EPwm4Regs.TBCTL.bit.CLKDIV = 0;
    EPwm4Regs.TBCTL.bit.SYNCOSEL=0;     //同步信号
    EPwm4Regs.TBCTL.bit.PHSEN = 0;      // 禁用相位寄存器
    EPwm4Regs.TBCTL.bit.CTRMODE = 2;    // 增减计数


    //事件触发模块ET
    EPwm4Regs.ETSEL.bit.SOCAEN = 1;     // 使能 ePWM2SOCA 信号产生
    EPwm4Regs.ETSEL.bit.SOCASEL = 1;    // 当TBCTR=0时产生 ePWM2SOCA信号
    EPwm4Regs.ETPS.bit.SOCAPRD = 1;     // 在第1个事件时产生 ePWM2SOCA信号


}



