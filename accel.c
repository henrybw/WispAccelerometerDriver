/************************************************************************************************************************************/
/** @file		accel.c
 * 	@brief		TODO: fill this in
 * 	@details	TODO: fill this in
 *
 * 	@author		Henry Baba-Weiss, UW Sensors and Systems Lab
 * 	@created	5.22.12
 * 	@last rev	see source repository
 */
/************************************************************************************************************************************/
#include "accel.h"

// Accelerometer data
static int16_t accel_x, accel_y, accel_z;

// Private function prototypes
void __Accel_InitInterrupts(void);
void __Accel_InitSerial(void);
uint8_t __Accel_ReadRegister(uint8_t address);
void __Accel_WriteRegister(uint8_t address, uint8_t data);
// TODO: write some sort of common I2C communication routine

/************************************************************************************************************************************/
//															PUBLIC API															 	*
//***********************************************************************************************************************************/

// TODO: document this
void Accel_Init(void)
{
	__Accel_InitInterrupts();
	__Accel_InitSerial();
	
	// Temporary LED debugging
	P1DIR |= (BIT0 | BIT6);
}

// TODO: document this
void Accel_Read(int16_t *x, int16_t *y, int16_t *z)
{
	if (x != NULL)
		*x = accel_x;
		
	if (y != NULL)
		*y = accel_y;
		
	if (z != NULL)
		*z = accel_z;
}

// TODO: document this
void Accel_ReadVector(int16_t outData[])
{
	if (outData != NULL)
	{
		outData[0] = accel_x;
		outData[1] = accel_y;
		outData[2] = accel_z;
	}
}

/************************************************************************************************************************************/
//														PRIVATE FUNCTIONS														 	*
//***********************************************************************************************************************************/

// TODO: document this
// TODO: make better
void __Accel_InitInterrupts(void)
{
	// Configure the incoming interrupt ports as inputs
	P1DIR &= ~(BIT3 | BIT4);
	
	// TODO: explain
	P1IE |= (BIT3 | BIT4);
	P1IES |= (BIT3 | BIT4);
	P1IFG &= ~(BIT3 | BIT4);
}

// TODO: document this
// TODO: make better
void __Accel_InitSerial(void)
{
	// 
	P1SEL |= BIT6 + BIT7;
	P1SEL2|= BIT6 + BIT7;
	
	// Initial I2C configuration
	UCB0CTL1 |= UCSWRST;                         // Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;        // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;               // Use SMCLK, keep SW reset
	
	// TODO: configure baud/data rate settings
	UCB0BR0 = 12;
	UCB0BR1 = 0;
	
	// Specify I2C slave address (when SDO/ALT ADDRESS is tied to ground)
	UCB0I2CSA = 0x53;
	
	// Finish I2C configuration
	UCB0CTL1 &= ~UCSWRST;                        // Clear software reset and resume operation
	IE2 |= UCB0RXIE;                             // Enable RX interrupt
	TACTL = TASSEL_2 + MC_2;                     // Enable SMCLK in continuous mode
}

// TODO: document this
uint8_t __Accel_ReadRegister(uint8_t address)
{
	// stub
	return 0;
}

// TODO: document this
void __Accel_WriteRegister(uint8_t address, uint8_t data)
{
	// stub
}

/************************************************************************************************************************************/
//														INTERRUPT VECTORS														 	*
//***********************************************************************************************************************************/

// Port 1 interrupt service routine, triggered when the accelerometer fires an interrupt
// TODO: document better?
// TODO: should we expose this interface? or should we just use this to update accelerometer data?
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	// TODO: this needs to actually do something
	P1OUT ^= 0x01;
	P1IFG &= ~(BIT3 | BIT4);
}

// I2C is using SMCLK, so we don't care about any interrupts from the timer itself
// TODO: document better?
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TA0_ISR(void)
{
	__bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
}
