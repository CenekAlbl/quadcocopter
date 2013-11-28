/*
 * pwm.c
 *
 *  Created on: 12.5.2013
 *      Author: Vincek
 */
#include "includes.h"

void set_timer_pwm(int timer, long freq, float duty_cycle){
	uint8_t prescale;
	uint8_t extender1,extender2;
	long load,match;
	load = SysCtlClockGet()/freq;
	match = duty_cycle*load;

	//use prescaler?
	if(load>2^16-1){
		prescale=1;
		extender1 = load >> 16;
		extender2 = match >> 16;
		load &= 0xFFFF;
		match &= 0xFFFF;
	}else{
		prescale = 0;
	}

	switch(timer){
	case 0:

		GPIOPinConfigure(GPIO_PB6_T0CCP0); // Configure pin PB6 as output of Timer 0_A
		GPIOPinConfigure(GPIO_PB7_T0CCP1); // Configure pin PB7 as output of Timer 0_B
		GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6 ); // Enable pin PB6 as output of timer addressed to it
		GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_7 ); // Enable pin PB7 as output of timer addressed to it
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Enable Timer 0
		TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM));
		TimerControlLevel(TIMER0_BASE, TIMER_BOTH, 0);
		if(prescale){
			 TimerPrescaleSet(TIMER0_BASE, TIMER_A, extender1);
			 TimerPrescaleSet(TIMER0_BASE, TIMER_B, extender1);
			 TimerPrescaleMatchSet(TIMER0_BASE, TIMER_A, extender2);
			 TimerPrescaleMatchSet(TIMER0_BASE, TIMER_B, extender2);
		}
		TimerLoadSet(TIMER0_BASE, TIMER_A, load);
		TimerLoadSet(TIMER0_BASE, TIMER_B, load);
	    TimerMatchSet(TIMER0_BASE, TIMER_A, match); // Timer 0 Match set
	    TimerMatchSet(TIMER0_BASE, TIMER_B, match);
	    TimerEnable(TIMER0_BASE, TIMER_BOTH);
		break;
	case 1:

		GPIOPinConfigure(GPIO_PF2_T1CCP0); // Configure pin PB4 as output of Timer 0_A
		GPIOPinConfigure(GPIO_PB5_T1CCP1); // Configure pin PB5 as output of Timer 0_B
		GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_2 ); // Enable pin PB4 as output of timer addressed to it
		GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_5 ); // Enable pin PB5 as output of timer addressed to it
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1
		TimerConfigure(TIMER1_BASE, (TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM));
		TimerControlLevel(TIMER1_BASE, TIMER_BOTH, 0);
		if(prescale){
			 TimerPrescaleSet(TIMER1_BASE, TIMER_A, extender1);
			 TimerPrescaleSet(TIMER1_BASE, TIMER_B, extender1);
			 TimerPrescaleMatchSet(TIMER1_BASE, TIMER_A, extender2);
			 TimerPrescaleMatchSet(TIMER1_BASE, TIMER_B, extender2);
		}
		TimerLoadSet(TIMER1_BASE, TIMER_A, load);
		TimerLoadSet(TIMER1_BASE, TIMER_B, load);
		TimerMatchSet(TIMER1_BASE, TIMER_A, match); // Timer 0 Match set
		TimerMatchSet(TIMER1_BASE, TIMER_B, match);
		TimerEnable(TIMER1_BASE, TIMER_BOTH);
		break;
	case 2:

		break;
	case 3:

		break;
	case 4:

		break;
	default:

		break;


	}

}

void set_pwm_duty_cycle(int channel, long freq, float duty_cycle){
	uint8_t prescale;
	uint8_t extender;
	long match;
	match = duty_cycle*SysCtlClockGet()/freq;

	//use prescaler?
	if(match>2^16-1){
		prescale=1;
		extender = match >> 16;
		match &= 0xFFFF;
	}else{
		prescale = 0;
	}

	switch(channel){
	case 0:
		if(prescale)TimerPrescaleMatchSet(TIMER0_BASE, TIMER_A, extender);
	    TimerMatchSet(TIMER0_BASE, TIMER_A, match);
	    break;
	case 1:
		if(prescale)TimerPrescaleMatchSet(TIMER0_BASE, TIMER_B, extender);
		TimerMatchSet(TIMER0_BASE, TIMER_B, match);
		break;
	case 2:
		if(prescale)TimerPrescaleMatchSet(TIMER1_BASE, TIMER_A, extender);
		TimerMatchSet(TIMER1_BASE, TIMER_A, match);
		break;
	case 3:
		if(prescale)TimerPrescaleMatchSet(TIMER1_BASE, TIMER_B, extender);
		TimerMatchSet(TIMER1_BASE, TIMER_B, match);
		break;
	case 4:

		break;
	default:

		break;

	}
}




