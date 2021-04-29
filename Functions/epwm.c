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
    EPwm1Regs.CMPA.half.CMPA = val;  //����ռ�ձ�
}
void EPwm1B_SetCompare(Uint16 val)
{
    EPwm1Regs.CMPB = val;  //����ռ�ձ�
}

void Setup_ePWM1(void)
{
    //ʱ��ģ��TB
    EPwm1Regs.TBPRD = SP;               // ����ֵ SP = CPU_CLK/(2*PWM_CLK)
    EPwm1Regs.TBPHS.half.TBPHS = 0;     // ��λΪ0
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;  // ʱ��ʱ�� TBCLK=SYSCLKOUT=1/150M
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTL.bit.SYNCOSEL=0;     //ͬ���ź�
    EPwm1Regs.TBCTL.bit.PHSEN = 0;      // ������λ�Ĵ���
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;    // ��������

    //�����Ƚ�ģ��CC
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0; // Ӱ��װ��ģʽ
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 2; // CTR = 0 �� CTR = PRDʱװ��
    EPwm1Regs.CMPA.half.CMPA =0;        // ���ó�ʼռ�ձ�Ϊ0

    //����ģ��AQ
    EPwm1Regs.AQCTLA.bit.CAU = 1;       // ���ϼ�����CTR=CMPAʱ��ePWM1A�õ�
    EPwm1Regs.AQCTLA.bit.CAD = 2;       // ���¼�����CTR=CMPAʱ��ePWM1A�ø�

    //��������ģ��DB
    EPwm1Regs.DBCTL.bit.IN_MODE = 0;    // ePWM1A��˫��������
    EPwm1Regs.DBCTL.bit.POLSEL = 2;     // ePWM1A����ת��ePWM1B��ת
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0x3;   // ʹ��˫������ʱ(��ʹ��A)
    EPwm1Regs.DBRED = 30;               // ��������ʱ DBRED*TBCLK = 500ns
    EPwm1Regs.DBFED = 30;               // �½�����ʱ DBRED*TBCLK = 500ns

    //��������ģ��TZ
    EALLOW;
    EPwm1Regs.TZCTL.bit.TZA = 30;        // �����¼�����ʱ��ǿ��ePWM1A��״̬
    EPwm1Regs.TZCTL.bit.TZB = 30;        // �����¼�����ʱ��ǿ��ePWM1B��״̬
    EDIS;
}

void Setup_ePWM2(void)
{
    //ʱ��ģ��TB
    EPwm2Regs.TBPRD = SP;               // ����ֵ SP = CPU_CLK/(2*PWM_CLK)
    EPwm2Regs.TBPHS.half.TBPHS = 0;     // ��λΪ0
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;  // ʱ��ʱ�� TBCLK=SYSCLKOUT=1/150M
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTL.bit.SYNCOSEL=0;     //ͬ���ź�
    EPwm2Regs.TBCTL.bit.PHSEN = 0;      // ������λ�Ĵ���
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;    // ��������

    //�����Ƚ�ģ��CC
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0; // Ӱ��װ��ģʽ
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 2; // CTR = 0 �� CTR = PRDʱװ��
    EPwm2Regs.CMPA.half.CMPA =0;        // ���ó�ʼռ�ձ�Ϊ0

    //����ģ��AQ
    EPwm2Regs.AQCTLA.bit.CAU = 1;       // ���ϼ�����CTR=CMPAʱ��ePWM1A�õ�
    EPwm2Regs.AQCTLA.bit.CAD = 2;       // ���¼�����CTR=CMPAʱ��ePWM1A�ø�

    //��������ģ��DB
    EPwm2Regs.DBCTL.bit.IN_MODE = 0;    // ePWM1A��˫��������
    EPwm2Regs.DBCTL.bit.POLSEL = 2;     // ePWM1A����ת��ePWM1B��ת
    EPwm2Regs.DBCTL.bit.OUT_MODE = 0x3;   // ʹ��˫������ʱ(��ʹ��A)
    EPwm2Regs.DBRED = 30;               // ��������ʱ DBRED*TBCLK = 500ns
    EPwm2Regs.DBFED = 30;               // �½�����ʱ DBRED*TBCLK = 500ns

    //��������ģ��TZ
    EALLOW;
    EPwm2Regs.TZCTL.bit.TZA = 30;        // �����¼�����ʱ��ǿ��ePWM1A��״̬
    EPwm2Regs.TZCTL.bit.TZB = 30;        // �����¼�����ʱ��ǿ��ePWM1B��״̬
    EDIS;
}
void Setup_ePWM3(void)
{
    //ʱ��ģ��TB
    EPwm3Regs.TBPRD = SP;               // ����ֵ SP = CPU_CLK/(2*PWM_CLK)
    EPwm3Regs.TBPHS.half.TBPHS = 0;     // ��λΪ0
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;  // ʱ��ʱ�� TBCLK=SYSCLKOUT=1/150M
    EPwm3Regs.TBCTL.bit.CLKDIV = 0;
    EPwm3Regs.TBCTL.bit.SYNCOSEL=0;     //ͬ���ź�
    EPwm3Regs.TBCTL.bit.PHSEN = 0;      // ������λ�Ĵ���
    EPwm3Regs.TBCTL.bit.CTRMODE = 2;    // ��������

    //�����Ƚ�ģ��CC
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0; // Ӱ��װ��ģʽ
    EPwm3Regs.CMPCTL.bit.LOADAMODE = 2; // CTR = 0 �� CTR = PRDʱװ��
    EPwm3Regs.CMPA.half.CMPA =0;        // ���ó�ʼռ�ձ�Ϊ0

    //����ģ��AQ
    EPwm3Regs.AQCTLA.bit.CAU = 1;       // ���ϼ�����CTR=CMPAʱ��ePWM1A�õ�
    EPwm3Regs.AQCTLA.bit.CAD = 2;       // ���¼�����CTR=CMPAʱ��ePWM1A�ø�

    //��������ģ��DB
    EPwm3Regs.DBCTL.bit.IN_MODE = 0;    // ePWM1A��˫��������
    EPwm3Regs.DBCTL.bit.POLSEL = 2;     // ePWM1A����ת��ePWM1B��ת
    EPwm3Regs.DBCTL.bit.OUT_MODE = 0x3;   // ʹ��˫������ʱ(��ʹ��A)
    EPwm3Regs.DBRED = 30;               // ��������ʱ DBRED*TBCLK = 500ns
    EPwm3Regs.DBFED = 30;               // �½�����ʱ DBRED*TBCLK = 500ns

    //��������ģ��TZ
    EALLOW;
    EPwm3Regs.TZCTL.bit.TZA = 30;        // �����¼�����ʱ��ǿ��ePWM1A��״̬
    EPwm3Regs.TZCTL.bit.TZB = 30;        // �����¼�����ʱ��ǿ��ePWM1B��״̬
    EDIS;
}

void Setup_ePWM4(void)  // ePWM2ģ�������Ӻ���
{
    //ʱ��ģ��TB
    EPwm4Regs.TBPRD = 1*SP;               // ����ֵ SP = CPU_CLK/(2*PWM_CLK)
    EPwm4Regs.TBPHS.half.TBPHS = 0;     // ��λΪ0
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;  // ʱ��ʱ�� TBCLK=SYSCLKOUT=1/150M
    EPwm4Regs.TBCTL.bit.CLKDIV = 0;
    EPwm4Regs.TBCTL.bit.SYNCOSEL=0;     //ͬ���ź�
    EPwm4Regs.TBCTL.bit.PHSEN = 0;      // ������λ�Ĵ���
    EPwm4Regs.TBCTL.bit.CTRMODE = 2;    // ��������


    //�¼�����ģ��ET
    EPwm4Regs.ETSEL.bit.SOCAEN = 1;     // ʹ�� ePWM2SOCA �źŲ���
    EPwm4Regs.ETSEL.bit.SOCASEL = 1;    // ��TBCTR=0ʱ���� ePWM2SOCA�ź�
    EPwm4Regs.ETPS.bit.SOCAPRD = 1;     // �ڵ�1���¼�ʱ���� ePWM2SOCA�ź�


}



