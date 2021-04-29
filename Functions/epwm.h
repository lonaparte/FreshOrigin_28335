/*
 * epwm.h
 *
 *  Created on: 2018-1-26
 *      Author: Administrator
 */

#ifndef EPWM_H_
#define EPWM_H_


#include "DSP2833x_Device.h"     // DSP2833x 头文件
#include "DSP2833x_Examples.h"   // DSP2833x 例子相关头文件
/*----------------------------------宏定义----------------------------------------------*/
#define  CPU_CLK  150e6               // 系统时钟 150MHz
#define  PWM_CLK  20e3                // 开关频率 20kHz

#define  Tsam     1/(PWM_CLK)       // 采样周期 Tsam=1/20k=50us
#define  SP       CPU_CLK/(2*PWM_CLK) // ePWM周期寄存器的值 3750


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
