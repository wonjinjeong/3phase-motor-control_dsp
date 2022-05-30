//###########################################################################
//
// FILE:   Example_F2806xLaunchPadDemo.c
//
// TITLE:   F2806x LaunchPad Out of Box Demo
//
//! \addtogroup f2806x_example_list
//! <h1>LaunchPad Demo (launchxl_f28069m)</h1>
//!
//!  This program is the demo program that comes pre-loaded on the F28069M
//!  LaunchPad development kit.  The program starts by flashing the two user
//!  LEDs. After a few seconds the LEDs stop flashing and samples the
//!  device's internal temperature sensor to establish a reference.  After
//!  this, any difference between the current temperature and the reference
//!  is shown as a change in the intensity of the two user LEDs.
//!
//
//###########################################################################
// $TI Release: F2806x Support Library v2.06.00.00 $
// $Release Date: Fri Feb 12 19:15:11 IST 2021 $
// $Copyright:
// Copyright (C) 2009-2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <file.h>
#include <math.h>
#include <IQmathLib.h>
#include <DCL.h>
#include <ipark.h>
#include <svgen.h>
#include <park.h>
#include <clarke.h>

//C:\ti\c2000\C2000Ware_MotorControl_SDK_3_03_00_00\libraries\utilities\types\include

#include "DSP28x_Project.h"     // DSP28x Headerfile

//
// Globals
//

#define PI 3.14159265359
#define SQRT3 0.57735
#define INV3 0.33333

#define SQRT3T2 0.8660254038
#define S2T3 0.6666666
#define SQRT3H  0.866025403784439   /* sqrt(3)/2 */
#define SIGN(x)  (((x) > 0) ? 1:0)

extern void DSP28x_usDelay(Uint32 Count);

// uvw_duty function
void ipark_svgen(float32_t vd, float32_t vq);
void clarke_park(float32_t in_a, float32_t in_b, float32_t in_c);
void abc2dq(float32_t in_a1, float32_t in_b1, float32_t in_c1);
void dq2abc(float32_t in_d1, float32_t in_q1);

IPARK ipark;
SVGEN svgen;
PARK park;
CLARKE clarke;
// ADC setup
void configureADC(void);
void configureEPWM(void);

__interrupt void adc1_isr(void);


// current EPWM setup
void initEPWM4(void);
void initEPWM5(void);
void initEPWM6(void);
void initEPWM7(void);
void initEPWM8(void);

__interrupt void epwm4_isr(void);
__interrupt void epwm5_isr(void);
__interrupt void epwm6_isr(void);
__interrupt void epwm7_isr(void);
__interrupt void epwm8_isr(void);
//
// Main
//
float ref;
float ref2;
float fun_amp;

float Wq;
float Wc = 10000;
float Ts = 0.00005;

float32_t a_gain, c_gain;
float ADCINTvalue;
float ADCOUTvalue;

float i_a;
float i_b;
float i_c;

float adc_ia;
float adc_ib;
float adc_ic;

float adcout_ia;
float adcout_ib;
float adcout_ic;

float iv_gain;

float i_q;
float i_qq;
float error_q;
float error_d;
float error_pre;
float error_sum;

float Kv;
float Kp;
float Ki;
float T;
Uint16 w;

float Umax = 0.2;
float Umin = -0.2;

float p_control;
float i_control;
float q_control;

float Ta,Tb,Tc;
float F;
//SVpwm_duty tranformation
Uint16 switch_1;
int sector,N;
float va,vb,i,j,k;
float u[2];
float ccc;
float co_in;
float32_t rad_f;
float32_t rad_t;
float32_t rad = 0;
float input_d, input_q;
float angle, angle2;
float input_fun;

// ABC to dq variable
float theta;
float theta_in;
float f;
float co;
float si;

float Fa;
float Fb;

int s[6];
float y[2];  // duty output
float y2[2]; // duty - abc transform output
float y3[2]; // alpha - beta to dq transform output

// abc to dq transform
float32_t inv_alpha, alpha;
float32_t inv_beta, beta;
float32_t out_d, out_q;
float32_t d_a,d_b,d_c;
float32_t dout, qout, in_d, in_q;
float32_t in_d1, in_q1;
float32_t out_alpha, out_beta;
float32_t out_a, out_b, out_c;
float32_t i_alpha, i_beta;
float32_t i_dd, i_qq;
Uint16 count;
float vdc;
float tts;

void main(void)
{

    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //




    InitSysCtrl();

    //
    // Step 2. Initalize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to its default state.
    //
    // InitGpio(); Skipped for this example
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();

    EALLOW;
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO52 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO52 = 0;
    EDIS;

    DINT;

    //
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    //
    InitPieVectTable();

    EALLOW;  // This is needed to write to EALLOW protected register
    PieVectTable.ADCINT1 = &adc1_isr;
    PieVectTable.EPWM4_INT = &epwm4_isr;
    PieVectTable.EPWM5_INT = &epwm5_isr;
    PieVectTable.EPWM6_INT = &epwm6_isr;
    PieVectTable.EPWM7_INT = &epwm7_isr;
    PieVectTable.EPWM8_INT = &epwm8_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    initEPWM4();
    initEPWM5();
    initEPWM6();
    initEPWM7();
    initEPWM8();

    configureADC();

    configureEPWM();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    //
    // Use write-only instruction to set TSS bit = 1
    //

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1;                     // Enable CPU Interrupt 1
    IER |= M_INT3;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
    PieCtrlRegs.PIEIER3.bit.INTx4 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx5 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx6 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx8 = 1;
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    //
    // Configure the ADC: Initialize the ADC
    //

    // Main program loop - continually sample temperature
    //
    T = 0;
    F = 0.00001;
    u[0] = 0;  // vd_ref
    u[1] = 0.01;  // vq_ref

    // pwm switching parameter
    f = 10;
    rad_f = 0.1;
    rad_t = 0.00005;
    rad = 0;
    co_in = 60;
    theta = 0;
    switch_1 = 0;
    angle = 0;
    count = 0;
    in_d = 0; in_q = 0; dout = 0; qout = 0;
    adcout_ia = 0;
    adcout_ib = 0;
    adcout_ic = 0;
    Kp = 0.194;
    Ki = 0.0127;
    input_d = u[0];
    input_q = u[1];
    error_sum = 0;
    for(;;)
    {
    }

}
__interrupt void
adc1_isr(void)
{
    rad = 2*PI*4*rad_f;      // 2*Pi*f(freq)
    theta_in = rad*rad_t;  // 적분 된 theta 값
    if(angle >180)
    {
        input_q = -u[1];
    }
    else if(angle <180)
    {
        input_q = u[1];
    }
    input_d = u[0];

    error_d = u[0] - out_d;
    error_q = input_q - out_q; // (desired)iq - (measure) iq

    // Kp, Ki는 time constant = L/R로 구해서 대입

    i_q = 0.067*(Kp*error_q+Ki*error_sum);

    if(i_q > Umax)
    {
        i_q = Umax;
    }
    else if(i_q <Umin)
    {
        i_q = Umin;
    }
    error_sum += error_q;
    ipark_svgen(input_d,i_q);

    i_a = 0.067*(2020.0-(float32_t)AdcResult.ADCRESULT1)*10.0/2048.0;    // IA
    i_b = 0.067*(2020.0-(float32_t)AdcResult.ADCRESULT2)*10.0/2048.0;    // IB
    i_c = 0.067*(2020.0-(float32_t)AdcResult.ADCRESULT3)*10.0/2048.0;    // IC
    if(input_q>0)
    {
        EPwm7Regs.CMPA.half.CMPA = 2250*3*input_q+2250;
        EPwm8Regs.CMPA.half.CMPA = 2250*3*out_q+2250;

        EPwm4Regs.CMPA.half.CMPA = Ta;
        EPwm5Regs.CMPA.half.CMPA = Tb;
        EPwm6Regs.CMPA.half.CMPA = Tc;
        clarke_park(i_a, i_b, i_c);
    }
    else if(input_q < 0)
    {
        EPwm7Regs.CMPA.half.CMPA = 2250*3*input_q+2250;
        EPwm8Regs.CMPA.half.CMPA = 2250*3*out_q+2250;

        EPwm4Regs.CMPA.half.CMPA = Tc;
        EPwm5Regs.CMPA.half.CMPA = Tb;
        EPwm6Regs.CMPA.half.CMPA = Ta;
        clarke_park(i_c, i_b, i_a);
    }
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    return;
}
__interrupt void
epwm4_isr(void)
{
    EPwm4Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void
epwm5_isr(void)
{
    EPwm5Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void
epwm6_isr(void)
{
    EPwm6Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void
epwm7_isr(void)
{
    EPwm7Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void
epwm8_isr(void)
{
    EPwm8Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
void clarke_park(float32_t in_a, float32_t in_b, float32_t in_c)
{
    clarke.As = in_a;
    clarke.Bs = in_b;
    clarke.Cs = in_c;
    runClarke(&clarke);

    alpha = clarke.Alpha;
    beta = clarke.Beta;

    park.Alpha = clarke.Alpha;
    park.Beta = clarke.Beta;
    park.Sine = sin(theta);
    park.Cosine = cos(theta);
    park.Angle = theta;
    runPark(&park);

    out_d = park.Ds;
    out_q = park.Qs;
}
void ipark_svgen(float32_t vd, float32_t vq)
{
    theta += theta_in;
    angle = theta*180/PI;
    if(angle > 360)
    {
        theta = 0;
    }
    ipark.Ds = vd;
    ipark.Qs = vq;
    ipark.Sine = sin(theta);
    ipark.Cosine = cos(theta);
    ipark.Angle = theta;
    runIPark(&ipark);

    inv_alpha = ipark.Alpha;
    inv_beta = ipark.Beta;

    svgen.Ualpha = ipark.Alpha;
    svgen.Ubeta = ipark.Beta;

    runSVGenDQ(&svgen);
    d_a = svgen.Ta;
    d_b = svgen.Tb;
    d_c = svgen.Tc;
    Ta = (Uint16)(2250.0*d_a+2250.0);
    Tb = (Uint16)(2250.0*d_b+2250.0);
    Tc = (Uint16)(2250.0*d_c+2250.0);
}
void configureADC(void)
{
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
    AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
    EDIS;


    EALLOW;
    AdcRegs.ADCCTL2.bit.CLKDIV2EN = 0;    //Clock prescaler
    EDIS;


    EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // enable non-overlap mode
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcRegs.INTSEL1N2.bit.INT1E = 1;  // enable ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0; // disable ADCINT1 Continuous mode
    AdcRegs.INTSEL1N2.bit.INT1SEL = 1;

    AdcRegs.ADCSOC1CTL.bit.CHSEL = 3;   // ADCINA3 - IA
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 9; // ADCSOCA / EPWM3
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;

    AdcRegs.ADCSOC2CTL.bit.CHSEL = 11;  // ADCINB3 - IB
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 9; // ADCSOCA / EPWM3
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;

    AdcRegs.ADCSOC3CTL.bit.CHSEL = 4;   // ADCINA4 - IC
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 9; // ADCSOCA / EPWM3
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 6;

    AdcRegs.ADCSOC4CTL.bit.CHSEL = 1;   // ADCINA1
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 9; // ADCSOCA / EPWM3
    AdcRegs.ADCSOC4CTL.bit.ACQPS = 6;

    AdcRegs.ADCSOC5CTL.bit.CHSEL = 8;   // ADCINA0
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 9; // ADCSOCA / EPWM3
    AdcRegs.ADCSOC5CTL.bit.ACQPS = 6;

    AdcRegs.ADCSOC6CTL.bit.CHSEL = 0;   // ADCINB0
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL = 9; // ADCSOCA / EPWM3
    AdcRegs.ADCSOC6CTL.bit.ACQPS = 6;
    EDIS;
}

void configureEPWM(void)
{
    EPwm3Regs.ETSEL.bit.SOCAEN  = 1;        // Enable SOC on A group
    EPwm3Regs.ETSEL.bit.SOCASEL = 4;        // Select SOC from CMPA on upcount
    EPwm3Regs.ETPS.bit.SOCAPRD  = 1;        // Generate pulse on 1st event
    EPwm3Regs.CMPA.half.CMPA    = 2250;   // Set compare A value
    EPwm3Regs.TBPRD             = 4499;   // Set period for ePWM1
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm3Regs.TBCTL.bit.CLKDIV = 0;
    EPwm3Regs.TBCTL.bit.CTRMODE = 0;        // count up and start
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;      // Set PWM1A on Zero
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;    // Clear PWM1A on event A, up count
}
void initEPWM4()
{
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm4Regs.TBPRD = 4499;       // Set timer period
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm4Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;   // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = 0;

    //
    // Setup shadow register load on ZERO
    //
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm4Regs.CMPA.half.CMPA = 2250;    // Set compare A value

    //
    // Set actions
    //
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;      // Set PWM1A on Zero
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;    // Clear PWM1A on event A, up count

    EPwm4Regs.AQCTLB.bit.CAU = AQ_SET;      // Set PWM1B on Zero
    EPwm4Regs.AQCTLB.bit.ZRO = AQ_CLEAR;    // Clear PWM1B on event B, up count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm4Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm4Regs.ETPS.bit.INTPRD = 1;           // Generate INT on 3rd event
}
void initEPWM5()
{
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm5Regs.TBPRD = 4499;       // Set timer period
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm5Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;   // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = 0;

    //
    // Setup shadow register load on ZERO
    //
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm5Regs.CMPA.half.CMPA = 2250;    // Set compare A value

    //
    // Set actions
    //
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;      // Set PWM1A on Zero
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;    // Clear PWM1A on event A, up count

    EPwm5Regs.AQCTLB.bit.CAU = AQ_SET;      // Set PWM1B on Zero
    EPwm5Regs.AQCTLB.bit.ZRO = AQ_CLEAR;    // Clear PWM1B on event B, up count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm5Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm5Regs.ETPS.bit.INTPRD = 1;           // Generate INT on 3rd event
}
void initEPWM6()
{
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm6Regs.TBPRD = 4499;       // Set timer period
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm6Regs.TBPHS.half.TBPHS = 0;       // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = 0;   // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = 0;

    //
    // Setup shadow register load on ZERO
    //
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm6Regs.CMPA.half.CMPA = 2250;    // Set compare A value

    //
    // Set actions
    //
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;      // Set PWM1A on Zero
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;    // Clear PWM1A on event A, up count

    EPwm6Regs.AQCTLB.bit.CAU = AQ_SET;      // Set PWM1B on Zero
    EPwm6Regs.AQCTLB.bit.ZRO = AQ_CLEAR;    // Clear PWM1B on event B, up count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm6Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm6Regs.ETPS.bit.INTPRD = 1;           // Generate INT on 3rd event
}
void initEPWM7()
{
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm7Regs.TBPRD = 4499;       // Set timer period
    EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm7Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = 0;   // Clock ratio to SYSCLKOUT
    EPwm7Regs.TBCTL.bit.CLKDIV = 0;

    //
    // Setup shadow register load on ZERO
    //
    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm7Regs.CMPA.half.CMPA = 450;    // Set compare A value

    //
    // Set actions
    //
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;      // Set PWM1A on Zero
    EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;    // Clear PWM1A on event A, up count

    EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;      // Set PWM1B on Zero
    EPwm7Regs.AQCTLB.bit.ZRO = AQ_CLEAR;    // Clear PWM1B on event B, up count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm7Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm7Regs.ETPS.bit.INTPRD = 1;           // Generate INT on 3rd event
}

void initEPWM8()
{
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm8Regs.TBPRD = 4499;       // Set timer period
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm8Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm8Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = 0;   // Clock ratio to SYSCLKOUT
    EPwm8Regs.TBCTL.bit.CLKDIV = 0;

    //
    // Setup shadow register load on ZERO
    //
    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm8Regs.CMPA.half.CMPA = 450;    // Set compare A value

    //
    // Set actions
    //
    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;      // Set PWM1A on Zero
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;    // Clear PWM1A on event A, up count

    EPwm8Regs.AQCTLB.bit.CBU = AQ_SET;      // Set PWM1B on Zero
    EPwm8Regs.AQCTLB.bit.ZRO = AQ_CLEAR;    // Clear PWM1B on event B, up count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm8Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm8Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm8Regs.ETPS.bit.INTPRD = 1;           // Generate INT on 3rd event
}
//
// End of File
//

//
// End of File
//
