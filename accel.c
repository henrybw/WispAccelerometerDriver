/************************************************************************************************************************************/
/** @file		accel.c
 * 	@brief		WISP driver/interface for the ADXL346 accelerometer module.
 * 	@details	Reads/writes to accelerometer registers over I2C, handles interrupts triggered by the accelerometer.
 *
 * 	@author		Henry Baba-Weiss, UW Sensors and Systems Lab
 * 	@created	5.22.12
 * 	@last rev	see source repository
 * 
 * @section Overview
 * 			Abstracts all the device-specific details and sets up an interface between the WISP and the accelerometer, exposing the
 * 			accelerometer data through a simple public-facing API.
 *
 * @section Usage
 * 			On startup, call Accel_Init to initialize the device. Accel_Read (or alternatively Accel_ReadVector) will read data from
 *			the accelerometer, and then put the device into low-power standby mode.
 * 
 * @section Details
 * 			The ADXL346 is controlled by reading and writing to a bank of registers (see accel_registers.h for a register map).
 * 			Read/write commands are sent serially using either SPI or I2C (we use I2C, operating at 12.5 Hz). Additionally, the
 * 			ADXL346 also has 8 interrupts available for various events (e.g. data is ready, free fall detected), each of which can be
 * 			mapped to one of two interrupt pins (INT1 and INT2) that the ADXL346 will turn on when an interrupt-enabled event occurs.
 *			For our purposes, we only care about the data ready interrupt, since the WISP isn't powered for long enough to feasibly
 *			take multiple measurements. (INT1 and INT2 are still connected and ready to use if, in the future, we want more than just
 *			the data ready interrupt.)
 * 
 * @section Dependencies
 * 			- Uses port interrupts on P1.3 and P1.4 to catch interrupts from the accelerometer.
 * 			- Uses UCB0 (P3.0 and P3.1, along with associated control registers) for I2C communication.
 * 			- Uses SMCLK to drive I2C communication.
 * 
 * @section	TODOs
 *	@todo	Figure out how to do multiple-byte reads of the data registers
 *	@todo	Figure out error handling
 *	@todo	Finish implementing I2C communication
 * 	@todo	Change I2C pins from 1.6/1.7 to 3.0/3.1 for the 5310
 * 	@todo	Integrate with WispGuts
 * 	@todo	Look into FIFO functionality
 * 	@todo	Maybe add hooks for custom interrupt handlers?
 */
/************************************************************************************************************************************/
#include "accel.h"
#include "accel_registers.h"

// PRIVATE GLOBAL DECLARATIONS -----------------------------------------------------------------------------------------------------//

// State variables for the ugly state machine we need to get I2C working with the MSP430
#define STATE_WRITING			0
#define STATE_READING			1
#define STATE_SENDING_RESTART	2

static uint8_t gState;
static uint8_t gDataBuffer[MAX_TRANSMISSION_SIZE];
static uint8_t gBufferSize;
static uint8_t gCurrentByte;

// Private helper functions
void __Accel_InitInterrupts(void);
void __Accel_InitDevice(void);
void __Accel_EnterStandby(void);

void __Accel_ReadAccelData(int16_t *x, int16_t *y, int16_t *z);
uint8_t __Accel_ReadRegister(uint8_t address);
void __Accel_WriteRegister(uint8_t address, uint8_t data);

void __Accel_SetupSerial(void);
void __Accel_ReadSequential(uint8_t startAddress, uint8_t *destBuffer, uint8_t size);
void __Accel_WriteSequential(uint8_t startAddress, uint8_t *srcBuffer, uint8_t size);

// PUBLIC API ----------------------------------------------------------------------------------------------------------------------//

/************************************************************************************************************************************/
/**	@fcn		void Accel_Init(void)
 *  @brief		Initializes the accelerometer and the MSP430 interfaces needed to interact with it.
 */
/************************************************************************************************************************************/
void Accel_Init(void)
{
	// Enable I2C pins 
	P1SEL  |= BIT6 + BIT7;
	P1SEL2 |= BIT6 + BIT7;
	
	// TODO: use these pins for the 5310 instead
	//P2SEL  |= BIT0 + BIT1;
	//P2SEL2 |= BIT0 + BIT1;
	
	// Temporary LED debugging (TODO: remove in production)
	//P1DIR |= BIT0;
	//P1OUT &= ~BIT0;
	
	//__Accel_InitInterrupts();
	__Accel_InitDevice();
}

/************************************************************************************************************************************/
/**	@fcn		void Accel_Read(int16_t *x, int16_t *y, int16_t *z)
 *  @brief		Reads the x, y, and z-axis data and puts the accelerometer into standby mode.
 *  @defails	Pass in NULL for any parameters you do not care about reading.
 *
 *  @param		[out]	x		The x component
 *
 *  @param		[out]	y		The y component
 * 
 *  @param		[out]	z		The z component
 */
/************************************************************************************************************************************/
void Accel_Read(int16_t *x, int16_t *y, int16_t *z)
{
	__Accel_ReadAccelData(x, y, z);
}

/************************************************************************************************************************************/
/**	@fcn		void Accel_ReadVector(int16_t data[])
 *  @brief		Reads the x, y, and z-axis data into an array and puts the accelerometer into standby mode.
 *
 *  @param		[out]	data	Expects an array of length 3 to write into (data[0] = x, data[1] = y, data[2] = z)
 */
/************************************************************************************************************************************/
void Accel_ReadVector(int16_t data[])
{
	if (data != NULL)
		__Accel_ReadAccelData(data, data + 1, data + 2);
}

// PRIVATE FUNCTIONS ---------------------------------------------------------------------------------------------------------------//

/************************************************************************************************************************************/
/**	@fcn		void __Accel_InitInterrupts(void)
 *  @brief		Sets up port interrupts to trigger on INT1 and INT2.
 *  @details	Uses P1.3 and P1.4.
 */
/************************************************************************************************************************************/
void __Accel_InitInterrupts(void)
{
	// Enable port interrupts on P1.3 and P1.4
	P1IE |= (BIT3 | BIT4);
	P1IES |= (BIT3 | BIT4);
	P1IFG &= ~(BIT3 | BIT4);
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_InitDevice(void)
 *  @brief		Sends a series of device-specific commands via register writes to initialize the accelerometer.
 */
/************************************************************************************************************************************/
void __Accel_InitDevice(void)
{
	__Accel_WriteRegister(REG_DATA_FORMAT, BIT3 | BIT1 | BIT0);		// Full resolution, +/- 16g range
	__Accel_WriteRegister(REG_BW_RATE, BIT4 | BIT2 | BIT1 | BIT0);	// Low-power mode (bit 4) with 12.5 MHz data rate
	__Accel_WriteRegister(REG_POWER_CTL, BIT3);						// Enter measurement mode
	__Accel_WriteRegister(REG_INT_ENABLE, BIT7);					// Enable the data ready interrupt

	// Sanity check the device by ensuring the DEVID register contains 0xe6
	if (__Accel_ReadRegister(REG_DEVID) != 0xe6)
	{
		// ERROR! TODO: how do we handle errors?
	}
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_EnterStandby(void)
 *  @brief		Turns off measurements and puts the accelerometer into standby mode.
 *  @details	Current consumption is reduced to 0.2 uA.
 */
/************************************************************************************************************************************/
void __Accel_EnterStandby(void)
{
	__Accel_WriteRegister(REG_POWER_CTL, 0);
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_ReadAccelData(int16_t *x, int16_t *y, int16_t *z)
 *  @brief		Private helper for reading accelerometer data. Puts the accelerometer into standby mode when finished.
 *  @defails	Pass in NULL for any parameters you do not care about reading.
 *
 *  @param		[out]	x		The x component
 *
 *  @param		[out]	y		The y component
 * 
 *  @param		[out]	z		The z component
 */
/************************************************************************************************************************************/
void __Accel_ReadAccelData(int16_t *x, int16_t *y, int16_t *z)
{
	if (x != NULL && y != NULL && z != NULL)
	{
		uint8_t buffer[6];
		
		// X, Y, and Z are each split into two registers (DATAX0, DATAX1, etc.) containing the least and most significant bytes,
		// respectively. We read all six registers in one multiple byte read to ensure atomicity (in case the accelerometer reading
		// would have changed in-between register reads).
		__Accel_ReadSequential(REG_DATAX0, buffer, 6);

		(*x) = buffer[0] | (buffer[1] << 8);
		(*y) = buffer[2] | (buffer[3] << 8);
		(*z) = buffer[4] | (buffer[5] << 8);
	}

	__Accel_EnterStandby();
}

/************************************************************************************************************************************/
/**	@fcn		uint8_t __Accel_ReadRegister(uint8_t address)
 *  @brief		Reads from the specified register.
 *
 *  @param		[in]	address		The address of the register to read from (see accel_registers.h)
 *
 *  @return		(uint8_t) The data returned from the register
 */
/************************************************************************************************************************************/
uint8_t __Accel_ReadRegister(uint8_t address)
{
	uint8_t result;

	__Accel_ReadSequential(address, &result, 1);
	return result;
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_WriteRegister(uint8_t address, uint8_t data)
 *  @brief		Writes to the specified register.
 *
 *  @param		[in]	address		The address of the register to write to (see accel_registers.h)
 *
 *  @param		[in]	data		The data to write into the register
 */
/************************************************************************************************************************************/
void __Accel_WriteRegister(uint8_t address, uint8_t data)
{
	__Accel_WriteSequential(address, &data, 1);
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_SetupSerial(void)
 *  @brief		Sets up the MSP430's I2C module to communicate with the accelerometer.
 *  @details	Uses a transmission rate of 100 kHz.
 */
/************************************************************************************************************************************/
void __Accel_SetupSerial(void)
{	
	// Disable all I2C interrupts
	IE2 &= ~(UCB0RXIE | UCB0TXIE);

	// Initial I2C configuration
	UCB0CTL1 |= UCSWRST;                         // Enable software reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;        // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;               // Use SMCLK, keep software reset
	
	// fSCL = SMCLK/12 = ~100kHz
	UCB0BR0 = 12;
	UCB0BR1 = 0;
	
	// Specify I2C slave address (SDO/ALT ADDRESS is tied to ground)
	UCB0I2CSA = 0x53;
	
	// Finish I2C configuration
	UCB0CTL1 &= ~UCSWRST;                        // Clear software reset and resume operation
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_ReadSequential(uint8_t startAddress, uint8_t *destBuffer, uint8_t size)
 *  @brief		TODO: write documentation
 *
 *  @param		[in]	startAddress	TODO: write documentation
 *
 *  @param		[in]	destBuffer		TODO: write documentation
 *
 *  @param		[in]	size			TODO: write documentation
 */
/************************************************************************************************************************************/
void __Accel_ReadSequential(uint8_t startAddress, uint8_t *destBuffer, uint8_t size)
{
	/*TI_USCI_I2C_transmitinit(0x53, 12);
	while ( TI_USCI_I2C_notready() );
	
	gDataBuffer[0] = startAddress;
	TI_USCI_I2C_transmit(1, gDataBuffer);
	LPM0;
	
	TI_USCI_I2C_receiveinit(0x48,0x3f); // initialize USCI and DMA module
	while ( TI_USCI_I2C_notready() ); // wait for bus to be free
	TI_USCI_I2C_receive(size, gDataBuffer);
	
	// The bytes we read are sitting in gDataBuffer; now copy them into destBuffer
	uint8_t i;
	for (i = 0; i < size; i++)
		destBuffer[i] = gDataBuffer[i];*/
	
	__Accel_SetupSerial();

	gState = STATE_READING;
	gDataBuffer[0] = startAddress;
	gBufferSize = size + 1;  // Account for register address
	gCurrentByte = 0;

	// Enable TX interrupt
	IE2 |= UCB0TXIE;
	UCB0I2CIE = UCSTPIE + UCSTTIE + UCNACKIE;

	// Transmit the start sentinel and then enter LPM0 with interrupts
	UCB0CTL1 |= UCTR + UCTXSTT;
	__bis_SR_register(CPUOFF + GIE);

	//
	// *** Receiving is done in the TX ISR, and when that's done, we exit low-power mode and return to here ***
	//

	// Ensure the stop sentinel was sent
	while (UCB0CTL1 & UCTXSTP);

	// The bytes we read are sitting in gDataBuffer; now copy them into destBuffer
	uint8_t i;
	for (i = 0; i < size; i++)
		destBuffer[i] = gDataBuffer[i + 1];
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_WriteSequential(uint8_t startAddress, uint8_t *destBuffer, uint8_t size)
 *  @brief		TODO: write documentation
 *
 *  @param		[in]	startAddress	TODO: write documentation
 *
 *  @param		[in]	srcBuffer		TODO: write documentation
 *
 *  @param		[in]	size			TODO: write documentation
 */
/************************************************************************************************************************************/
void __Accel_WriteSequential(uint8_t startAddress, uint8_t *srcBuffer, uint8_t size)
{
	/*TI_USCI_I2C_transmitinit(0x53, 12);
	while ( TI_USCI_I2C_notready() );
	
	gDataBuffer[0] = startAddress;
	gBufferSize = size + 1;  // Account for register address
	
	// Copy the source buffer into the transmission buffer beforehand
	uint8_t i;
	for (i = 1; i <= size; i++)
		gDataBuffer[i] = srcBuffer[i - 1];  // First slot is reserved for starting register address
	
	TI_USCI_I2C_transmit(gBufferSize, gDataBuffer);
	LPM0;*/
	
	__Accel_SetupSerial();

	gState = STATE_WRITING;
	gDataBuffer[0] = startAddress;
	gBufferSize = size + 1;  // Account for register address
	gCurrentByte = 0;

	// Copy the source buffer into the transmission buffer beforehand
	uint8_t i;
	for (i = 0; i < size; i++)
		gDataBuffer[i + 1] = srcBuffer[i];  // First slot is reserved for starting register address

	// Enable TX interrupt
	IE2 |= UCB0TXIE;
	UCB0I2CIE = UCSTPIE + UCSTTIE + UCNACKIE;

	// Transmit the start sentinel and then enter LPM0 with interrupts
	UCB0CTL1 |= UCTR + UCTXSTT;
	UCB0TXBUF = startAddress;
	while (UCB0CTL1 & UCTXSTT);
	
	__no_operation();
	__bis_SR_register(CPUOFF + GIE);

	//
	// *** Transmission is done in the TX ISR, and when that's done, we exit low-power mode and return to here ***
	//

	// Ensure the stop sentinel was sent
	while (UCB0CTL1 & UCTXSTP);
}

// INTERRUPT VECTORS ---------------------------------------------------------------------------------------------------------------//

/************************************************************************************************************************************/
/**	@fcn		__interrupt void Port_1(void)
 *  @brief		Port 1 interrupt service routine.
 *  @details	Triggered when the accelerometer fires an interrupt on INT1 or INT2.
 */
/************************************************************************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	if (P1IFG & BIT3)
	{
		// Interrupt 1 was triggered
		// TODO: actually do something with this
		P1OUT |= BIT0;
	}
	
	if (P1IFG & BIT4)
	{
		// Interrupt 2 was triggered
		// TODO: actually do something with this
		//P1OUT |= BIT6;
	}
	
	P1IFG &= ~(BIT3 | BIT4);
}

/************************************************************************************************************************************/
/**	@fcn		__interrupt void USCIAB0TX_ISR(void)
 *  @brief		USCI_B0 data interrupt service routine.
 *  @details	TODO: document the ugly state machine
 */
/************************************************************************************************************************************/
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	if (gCurrentByte == 0)
	{
		// First byte is always the (starting) register address
		UCB0TXBUF = gDataBuffer[0];

		// The read procedure is weird, since the first step involves an I2C write (to send the register address), followed by
		// a restart, and then an I2C read for actually reading data. To handle the restart properly, we hack the state machine
		// to have an extra state for sending another start sentinel in-between sending the register address and reading data.
		if (gState == STATE_READING)
			gState = STATE_SENDING_RESTART;

		gCurrentByte++;
	}
	else if (gState == STATE_SENDING_RESTART)
	{
		UCB0CTL1 &= ~UCTR;
		UCB0CTL1 |= UCTXSTT;

		// Enable RX interrupt instead
		IE2 &= ~UCB0TXIE;
		IE2 |= UCB0RXIE;

		// End hack; go back into reading mode
		gState = STATE_READING;
	}
	else if (gCurrentByte == gBufferSize)
	{
		// We're done, so send the start sentinel and exit low-power mode
		UCB0CTL1 |= UCTXSTP;
		IFG2 &= ~UCB0TXIFG;  // Clear USCI_B0 TX interrupt flag
		__bic_SR_register_on_exit(CPUOFF);
	}
	else
	{
		if (gState == STATE_READING)
			gDataBuffer[gCurrentByte] = UCB0RXBUF;
		else
			UCB0TXBUF = gDataBuffer[gCurrentByte];

		gCurrentByte++;
	}
}
