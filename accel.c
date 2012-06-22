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
 * 			the accelerometer, and then put the device into low-power standby mode.
 * 
 * @section Details
 * 			The ADXL346 is controlled by reading and writing to a bank of registers (see accel_registers.h for a register map).
 * 			Read/write commands are sent serially using either SPI or I2C (we use I2C, operating at 12.5 Hz). Additionally, the
 * 			ADXL346 also has 8 interrupts available for various events (e.g. data is ready, free fall detected), each of which can be
 * 			mapped to one of two interrupt pins (INT1 and INT2) that the ADXL346 will turn on when an interrupt-enabled event occurs.
 * 			For our purposes, we only care about the data ready interrupt, since the WISP isn't powered for long enough to feasibly
 * 			take multiple measurements. (INT1 and INT2 are still connected and ready to use if, in the future, we want more than just
 * 			the data ready interrupt.)
 * 
 * @section Dependencies
 * 			- Uses port interrupts on P1.3 and P1.4 to catch interrupts from the accelerometer.
 * 			- Uses UCB0 (P3.0 and P3.1, along with associated control registers) for I2C communication.
 * 			- Uses SMCLK to drive I2C communication.
 * 
 * @section Future Design Considerations
 * 			Currently, based on the normal WISP usage scenario, this interface is designed to be activated once, take a measurement,
 * 			and then put the accelerometer back into standby mode. The accelerometer, however, also has a sleep mode that can generate
 * 			an interrupt when activity is detected (otherwise, the custom interrupts are useless for our purposes). Right now we ignore
 * 			this, but maybe this can be redesigned in the future to configure activity interrupts and then enter sleep mode instead of
 * 			entering standby after taking a measurement.
 * 
 * @section	TODOs
 * 	@todo	Change I2C pins from 1.6/1.7 to 3.0/3.1 for the 5310
 * 	@todo	Integrate with WispGuts
 * 	@todo	Remove everything between <strip></strip> tags.
 * 	@todo	Look into calibrating the accelerometer
 * 	@todo	Maybe look into FIFO functionality?
 */
/************************************************************************************************************************************/
#include "accel.h"
#include "accel_registers.h"

// PRIVATE GLOBAL DECLARATIONS -----------------------------------------------------------------------------------------------------//

// State variables for the ugly state machine we need to get I2C working with the MSP430
#define OPERATION_WRITE		0
#define OPERATION_READ		1

static volatile uint8_t gOperation;  // The overall register operation to be performed
static volatile bool gMustSwitchI2CMode = false;  // Flag that signifies when we need to switch to I2C read mode
static volatile uint8_t gDataBuffer[MAX_TRANSMISSION_SIZE];
static volatile uint8_t gBufferSize;
static volatile uint8_t gCurrentByte;

// Private helper functions
void __Accel_InitSerial(void);
void __Accel_InitInterrupts(void);
void __Accel_InitDevice(void);

void __Accel_ReadAccelData(int16_t *x, int16_t *y, int16_t *z);
uint8_t __Accel_ReadRegister(uint8_t address);
void __Accel_WriteRegister(uint8_t address, uint8_t data);
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
	// Calibrate DCO and SMCLK to 1 MHz
	// TODO: rewrite this to instead use the UCS module on the 5310
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;

//<strip>
	// Enable I2C pins 
	P1SEL  |= BIT6 + BIT7;
	P1SEL2 |= BIT6 + BIT7;
//</strip>

	// TODO: use these pins for the 5310 instead
	//P3SEL  |= BIT0 + BIT1;
	//P3SEL2 |= BIT0 + BIT1;
	
	__Accel_InitSerial();
	__Accel_InitInterrupts();
	__Accel_InitDevice();
}

/************************************************************************************************************************************/
/**	@fcn		void Accel_Read(int16_t *x, int16_t *y, int16_t *z)
 *  @brief		Reads the x, y, and z-axis data and puts the accelerometer into standby mode.
 *  @defails	Pass in NULL for any parameters you do not care about reading.
 *
 *  @param		[out]	x		The x component.
 *
 *  @param		[out]	y		The y component.
 * 
 *  @param		[out]	z		The z component.
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
/**	@fcn		void __Accel_InitSerial(void)
 *  @brief		Sets up the MSP430's I2C module to communicate with the accelerometer.
 *  @details	Uses a transmission clock speed of 100 kHz.
 */
/************************************************************************************************************************************/
void __Accel_InitSerial(void)
{	
	// Disable all I2C interrupts
	IE2 &= ~(UCB0RXIE | UCB0TXIE);

	// Ensure there isn't a stop condition currently being sent
	while (UCB0CTL1 & UCTXSTP);

	// Put the I2C module into software reset mode
	UCB0CTL1 |= UCSWRST;
	
	// Initial I2C configuration
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;        // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;               // Use SMCLK, keep software reset
	
	// fSCL = SMCLK/10 = 1 MHz / 10 = ~100kHz
	UCB0BR0 = 10;
	UCB0BR1 = 0;
	
	// Specify I2C slave address (SDO/ALT ADDRESS is tied to ground)
	UCB0I2CSA = 0x53;
	
	// Clear software reset and resume operation
	UCB0CTL1 &= ~UCSWRST;
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_InitInterrupts(void)
 *  @brief		Sets up port interrupts to trigger on INT1 and INT2.
 *  @details	Uses P1.3 and P1.4.
 */
/************************************************************************************************************************************/
void __Accel_InitInterrupts(void)
{
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
	// Wait 1.1 milliseconds before transmitting any commands
	__delay_cycles(1100);
	
	__Accel_WriteRegister(REG_DATA_FORMAT, BIT3 | BIT1 | BIT0);		// Full resolution, +/- 16g range
	__Accel_WriteRegister(REG_BW_RATE, BIT4 | /*BIT2 | BIT1 | BIT0*/ 0x0c);	// Low-power mode (bit 4) with 12.5 Hz data rate (bits 3-0)
	__Accel_WriteRegister(REG_INT_ENABLE, BIT7);					// Enable the data ready interrupt
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_ReadAccelData(int16_t *x, int16_t *y, int16_t *z)
 *  @brief		Private helper for reading accelerometer data. Puts the accelerometer into standby mode when finished.
 *  @defails	Pass in NULL for any parameters you do not care about reading.
 *
 *  @param		[out]	x		The x component.
 *
 *  @param		[out]	y		The y component.
 * 
 *  @param		[out]	z		The z component.
 */
/************************************************************************************************************************************/
void __Accel_ReadAccelData(int16_t *x, int16_t *y, int16_t *z)
{
	// Enter measurement mode
	__Accel_WriteRegister(REG_POWER_CTL, BIT3);
	
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
	
	// Enter standby mode
	__Accel_WriteRegister(REG_POWER_CTL, 0);
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
 *  @param		[in]	address		The address of the register to write to (see accel_registers.h).
 *
 *  @param		[in]	data		The data to write into the register.
 */
/************************************************************************************************************************************/
void __Accel_WriteRegister(uint8_t address, uint8_t data)
{
	__Accel_WriteSequential(address, &data, 1);
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_ReadSequential(uint8_t startAddress, uint8_t *destBuffer, uint8_t size)
 *  @brief		Performs a multiple-byte register read operation.
 *  @defails	This reads multiple registers in one atomic operation, reading <size> number of registers in sequential order,
 *  			beginning with the register address <startAddress> and ending at the register address (<startAddress> + (<size> - 1)).
 *
 *  @param		[in]	startAddress	The address of the first register to read from (see accel_registers.h).
 *
 *  @param		[out]	destBuffer		An array to write the data received from the accelerometer into. This is assumed to be at
 *  									least <size> bytes long.
 *
 *  @param		[in]	size			The number of registers to read into destBuffer.
 */
/************************************************************************************************************************************/
void __Accel_ReadSequential(uint8_t startAddress, uint8_t *destBuffer, uint8_t size)
{
	gOperation = OPERATION_READ;
	gDataBuffer[0] = startAddress;
	gBufferSize = size + 1;  // Account for register address
	gCurrentByte = 0;
	
	// Disable interrupts while we specify which interrupts to enable. This avoids a possible condition where the TX interrupt
	// is fired (because the transmit buffer is empty) before we are actually ready to transmit anything.
	__bic_SR_register(GIE);
	
	// Enable TX and other necessary interrupts
	IE2 |= UCB0TXIE;
	UCB0I2CIE = UCSTPIE + UCSTTIE + UCNACKIE;

	// Transmit the start sentinel and then enter LPM0 with interrupts
	UCB0CTL1 |= UCTR + UCTXSTT;
	__bis_SR_register(LPM0_bits + GIE);

	//
	// *** Receiving is done in the TX ISR, and when that's done, we exit low-power mode and return to here ***
	//

	// Ensure the stop sentinel was sent
	while (UCB0CTL1 & UCTXSTP);

	// The bytes we read are sitting in gDataBuffer; now copy them into destBuffer
	uint8_t i;
	for (i = 0; i < size; i++)
		destBuffer[i] = gDataBuffer[i + 1];  // Skip over starting register address
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_WriteSequential(uint8_t startAddress, uint8_t *srcBuffer, uint8_t size)
 *  @brief		Performs a multiple-byte register write opertation.
 *  @defails	This writes to multiple registers in one atomic operation, writing to <size> number of registers in sequential order,
 *  			beginning with the register address <startAddress> and ending at the register address (<startAddress> + (<size> - 1)).
 *
 *  @param		[in]	startAddress	The address of the first register to write to (see accel_registers.h).
 *
 *  @param		[out]	srcBuffer		An array of bytes to send to the accelerometer. This is assumed to be at least <size> bytes
 *  									long.
 *
 *  @param		[in]	size			The number of registers to write to.
 */
/************************************************************************************************************************************/
void __Accel_WriteSequential(uint8_t startAddress, uint8_t *srcBuffer, uint8_t size)
{
	gOperation = OPERATION_WRITE;
	gDataBuffer[0] = startAddress;
	gBufferSize = size + 1;  // Account for register address
	gCurrentByte = 0;
	
	// Disable interrupts while we specify which interrupts to enable. This avoids a possible condition where the TX interrupt
	// is fired (because the transmit buffer is empty) before we are actually ready to transmit anything.
	__bic_SR_register(GIE);

	// Copy the source buffer into the transmission buffer before starting the transmission process
	uint8_t i;
	for (i = 0; i < size; i++)
		gDataBuffer[i + 1] = srcBuffer[i];  // Skip over starting register address
	
	// Enable TX and other necessary interrupts
	IE2 |= UCB0TXIE;
	UCB0I2CIE = UCSTPIE + UCSTTIE + UCNACKIE;

	// Transmit the start sentinel and then enter LPM0 with interrupts
	UCB0CTL1 |= UCTR + UCTXSTT;
	__bis_SR_register(LPM0_bits + GIE);
	
	//
	// *** Transmission is done in the TX ISR, and when that's done, we exit low-power mode and return to here ***
	//

	// Ensure the stop sentinel was sent before returning
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
	// TODO: we should figure out if interrupts are necessary
	if (P1IFG & BIT3)
	{
		// Interrupt 1 was triggered
		P1IFG &= ~BIT3;
		__bic_SR_register_on_exit(LPM0_bits);
	}
	
	if (P1IFG & BIT4)
	{
		// Interrupt 2 was triggered
		P1IFG &= ~BIT4;
		__bic_SR_register_on_exit(LPM0_bits);
	}
}

/************************************************************************************************************************************/
/**	@fcn		__interrupt void USCIAB0TX_ISR(void)
 *  @brief		USCI_B0 data interrupt service routine.
 *  @details	After the MSP430 sends the start sentinel, all data transmission is handled by the state machine implemented in this
 *  			ISR. This handles both I2C reads and writes of any length.
 * 
 *  			The state machine is implemented based on the current byte being read or written from the data buffer. The states
 *  			are, in rough order of execution:
 * 
 *  			- Send starting register address
 *  				This is the start state, signified by gCurrentByte being 0. The first byte in the data buffer always contains
 *  				the starting register address of the current read/write operation, and this is always written first.
 * 
 *  			- Switch to I2C read mode
 *  				Activated when the gMustSwitchI2CMode flag is true. Since the first operation of any register read or write is
 *  				to write the starting register address to the accelerometer, if we're performing a read operation, we need to
 *  				switch into I2C read mode and then send a restart to the accelerometer, signaling that we are ready to start
 *  				reading data from it.
 * 
 *  			- Transmit data
 *  				Activated when 0 < gCurrentByte < gBufferSize, meaning that we still have data left to read/write. This is a
 *  				simple transmission state, transferring data between gDataBuffer and the appropriate MSP430 transmission register.
 * 
 *  			- Done transmitting
 *  				Activated when gCurrentByte == gBufferSize, meaning that we have transmitted all the data that was requested.
 *  				This sends a stop condition, cleans up, and exits low-power mode, returning program control back to the caller.
 */
/************************************************************************************************************************************/
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	if (gCurrentByte == 0)
	{
		// We are transmitting the first byte, which is always the starting register address
		UCB0TXBUF = gDataBuffer[0];

		// The read procedure is weird, since the first step involves an I2C write (to send the register address), followed by
		// a restart, and then an I2C read for actually reading data. To handle the restart properly, we hack the state machine
		// to have an extra state for sending another start sentinel in-between sending the register address and reading data.
		if (gOperation == OPERATION_READ)
			gMustSwitchI2CMode = true;

		gCurrentByte++;
	}
	else if (gMustSwitchI2CMode)
	{
		// We have sent the register address, and now we want to actually start reading data
		UCB0CTL1 &= ~UCTR;
		UCB0CTL1 |= UCTXSTT;  // Restart is a repeated start sentinel

		// Enable the RX interrupt instead of the TX interrupt
		IE2 &= ~UCB0TXIE;
		IE2 |= UCB0RXIE;

		// End hack; go back into normal reading mode
		gMustSwitchI2CMode = false;
	}
	else if (gCurrentByte == gBufferSize)
	{
		// We've transmitted all the bytes in the buffer, so now we're done
		UCB0CTL1 |= UCTXSTP;            // Send start sentinel
		IFG2 &= ~UCB0TXIFG;             // Clear TX interrupt flag
		IE2 &= ~(UCB0TXIE | UCB0RXIE);  // Disable TX/RX interrupts
		
		__bic_SR_register_on_exit(LPM0_bits);  // Exit low-power mode
	}
	else
	{
		// We're in the middle of receiving or transmtting data
		if (gOperation == OPERATION_READ)
			gDataBuffer[gCurrentByte] = UCB0RXBUF;
		else
			UCB0TXBUF = gDataBuffer[gCurrentByte];

		gCurrentByte++;
	}
}
