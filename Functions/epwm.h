/*
 * epwm.h
 *
 *  Created on: 2018-1-26
 *      Author: Administrator
 */

#ifndef EPWM_H_
#define EPWM_H_


#include "DSP2833x_Device.h"     // DSP2833x ͷ�ļ�
#include "DSP2833x_Examples.h"   // DSP2833x �������ͷ�ļ�
/*----------------------------------�궨��----------------------------------------------*/
#define  CPU_CLK  150e6               // ϵͳʱ�� 150MHz
#define  PWM_CLK  20e3                // ����Ƶ�� 20kHz

#define  Tsam     1/(PWM_CLK)       // �������� Tsam=1/20k=50us
#define  SP       CPU_CLK/(2*PWM_CLK) // ePWM���ڼĴ�����ֵ 3750


void EPWM1_Init(Uint16 tbprd);
void EPwm1A_SetCompare(Uint16 val);
void EPwm1B_SetCompare(Uint16 val);
void Setup_ePWM1(void);
void Setup_ePWM2(void);
void Setup_ePWM3(void);
void Setup_ePWM4(void);

void EPWM6_Init(Uint16 tbprd);
void EPwm6A_SetCompare(Uint16 val);
void EPwm6B_SetCompare(Uint16 val);

#endif /* EPWM_H_ */
