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
 * 			On startup, call Accel_Init to initialize the device. This sets the device in standby mode. To actually read from the
 *			accelerometer, call Accel_StartMeasuring. Accel_Read (or alternatively Accel_ReadVector) will return the most recent
 *			accelerometer measurements. When finished, call Accel_StopMeasuring to put the device in low-power standby mode.
 * 
 * @section Details
 * 			The ADXL346 is controlled by reading and writing to a bank of registers (see accel_registers.h for a register map).
 * 			Read/write commands are sent serially using either SPI or I2C (we use I2C, operating at 12.5 Hz). Additionally, the
 * 			ADXL346 also has 8 interrupts available for various events (e.g. data is ready, free fall detected), each of which can be
 * 			mapped to one of two interrupt pins (INT1 and INT2) that the ADXL346 will turn on when an interrupt-enabled event occurs.
 * 
 * @section Dependencies
 * 			- Uses port interrupts on P1.3 and P1.4 to catch interrupts from the accelerometer.
 * 			- Uses UCB0 (P3.0 and P3.1, along with associated control registers) for I2C communication.
 * 			- Uses ACLK to drive I2C communication.
 * 
 * @section	TODOs
 *	@todo	Finish implementing I2C communication
 *	@todo	Investigate how to best handle sleep states
 * 	@todo	Change I2C pins from 1.6/1.7 to 3.0/3.1 for the 5310
 * 	@todo	Integrate with WispGuts
 * 	@todo	Maybe add hooks for custom interrupt handlers?
 * 	@todo	Look into FIFO functionality
 */
/************************************************************************************************************************************/
#include "accel.h"
#include "accel_registers.h"

// PRIVATE GLOBAL DECLARATIONS -----------------------------------------------------------------------------------------------------//

// Accelerometer data
static int16_t gAccelX = 0, gAccelY = 0, gAccelZ = 0;
static uint8_t gRXData;
static bool gStandbyMode = true;

// Private function prototypes
void __Accel_InitInterrupts(void);
void __Accel_InitSerial(void);
void __Accel_InitDevice(void);
uint8_t __Accel_ReadRegister(uint8_t address);
void __Accel_WriteRegister(uint8_t address, uint8_t data);
// TODO: write some sort of common I2C communication routine

// PUBLIC API ----------------------------------------------------------------------------------------------------------------------//

/************************************************************************************************************************************/
/**	@fcn		void Accel_Init(void)
 *  @brief		Initializes the accelerometer and the MSP430 interfaces needed to interact with it.
 *  @details	The device starts up in standby mode; call Accel_StartMeasuring() when ready to read from the accelerometer.
 */
/************************************************************************************************************************************/
void Accel_Init(void)
{
	// Temporary LED debugging (TODO: remove in production)
	P1DIR |= (BIT0 | BIT6);
	
	__Accel_InitInterrupts();
	__Accel_InitSerial();
	__Accel_InitDevice();
}

/************************************************************************************************************************************/
/**	@fcn		void Accel_StartMeasuring(void)
 *  @brief		Tells the accelerometer to wake up and start making measurements.
 */
/************************************************************************************************************************************/
void Accel_StartMeasuring(void)
{
	__Accel_WriteRegister(REG_POWER_CTL, 0x08);
	gStandbyMode = false;

	// Enable data ready interrupts
	__Accel_WriteRegister(REG_INT_ENABLE, 0x80);
}

/************************************************************************************************************************************/
/**	@fcn		void Accel_StartMeasuring(void)
 *  @brief		Stops accelerometer measurements and puts the device into low-power standby mode.
 */
/************************************************************************************************************************************/
void Accel_StopMeasuring(void)
{
	__Accel_WriteRegister(REG_POWER_CTL, 0x00);
	gStandbyMode = true;
}

/************************************************************************************************************************************/
/**	@fcn		void Accel_Read(int16_t *x, int16_t *y, int16_t *z)
 *  @brief		Returns the most recently-read accelerometer data.
 *  @details	If the device is in standby mode, this does nothing.
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
	if (!gStandbyMode)
	{
		if (x != NULL)
			*x = gAccelX;
			
		if (y != NULL)
			*y = gAccelY;
			
		if (z != NULL)
			*z = gAccelZ;
	}
}

/************************************************************************************************************************************/
/**	@fcn		void Accel_ReadVector(int16_t data[])
 *  @brief		Returns the most recently-read accelerometer data into an array.
 *  @details	If the device is in standby mode, this does nothing.
 *
 *  @param		[out]	data	Expects an array of length 3 to write into (data[0] = x, data[1] = y, data[2] = z)
 */
/************************************************************************************************************************************/
void Accel_ReadVector(int16_t data[])
{
	if (!gStandbyMode && data != NULL)
	{
		data[0] = gAccelX;
		data[1] = gAccelY;
		data[2] = gAccelZ;
	}
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
/**	@fcn		void __Accel_InitSerial(void)
 *  @brief		Sets up the MSP430's I2C module to communicate with the accelerometer.
 *  @details	Uses a data rate of 12.5 Hz.
 */
/************************************************************************************************************************************/
void __Accel_InitSerial(void)
{
	// Enable I2C pins 
	P1SEL  |= BIT6 + BIT7;
	P1SEL2 |= BIT6 + BIT7;
	
	// TODO: use these pins for the 5310 instead
	//P2SEL  |= BIT0 + BIT1;
	//P2SEL2 |= BIT0 + BIT1;
	
	// Initial I2C configuration
	UCB0CTL1 |= UCSWRST;                         // Enable software reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;        // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_1 + UCSWRST;               // Use ACLK, keep software reset
	
	// 12.5 Hz gives us the lowest current consumption (23 uA). ACLK runs at
	// 32 kHz, so the divider is: 32000/12.5 = 2560 = 0x0a00.
	UCB0BR0 = 0x00;
	UCB0BR1 = 0x0a;
	
	// Specify I2C slave address (SDO/ALT ADDRESS is tied to ground)
	UCB0I2CSA = 0x53;
	
	// Finish I2C configuration
	UCB0CTL1 &= ~UCSWRST;                        // Clear software reset and resume operation
	IE2 |= UCB0RXIE;                             // Enable RX interrupt
}

/************************************************************************************************************************************/
/**	@fcn		void __Accel_InitDevice(void)
 *  @brief		Sends a series of device-specific commands via register writes to initialize the accelerometer.
 */
/************************************************************************************************************************************/
void __Accel_InitDevice(void)
{
	__Accel_WriteRegister(REG_DATA_FORMAT, 0x0b);	// Full resolution, +/- 16g range
	
	// TODO: write the rest of the initialization sequence
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
	// TODO: write I2C stuff
	return 0;
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
	// TODO: write I2C stuff
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
		P1OUT |= BIT6;
	}
	
	P1IFG &= ~(BIT3 | BIT4);
}

/************************************************************************************************************************************/
/**	@fcn		__interrupt void USCIAB0TX_ISR(void)
 *  @brief		USCI_B0 data interrupt service routine.
 *  @details	Triggered when the accelerometer sends data over the I2C line.
 */
/************************************************************************************************************************************/
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	// Grab received data and exit low-power mode
	gRXData = UCB0RXBUF;
	__bic_SR_register_on_exit(CPUOFF);
}
