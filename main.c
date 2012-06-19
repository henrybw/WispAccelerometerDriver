//
// Simple driver program to test the accelerometer API.
//
// BIT0 - red LED
// BIT6 - green LED
//
#include <msp430g2553.h>
#include "accel.h"

void main(void)
{
	// Stop watchdog timer
	WDTCTL = WDTPW + WDTHOLD;
	
	// Turn on SMCLK monitoring
	P1SEL |= BIT4;
	P1SEL2 &= ~BIT4;
	P1DIR |= BIT4;
	
	Accel_Init();
	
	// Enter LPM4 w/interrupt
	_BIS_SR(LPM4_bits + GIE);
}
