/*
 * accel_registers.h
 *
 *  Created on: May 22, 2012
 *      Author: htw
 */

#ifndef ACCEL_REGISTERS_H_
#define ACCEL_REGISTERS_H_

//
// Created from accel_registers.txt, which was in turn copied and pasted from
// the register map in the ADXL346 data sheet (table 19, page 22).
//
#define REG_DEVID			0x00	// Device ID
#define REG_THRESH_TAP		0x1D	// Tap threshold 
#define REG_OFSX			0x1E	// X-axis offset 
#define REG_OFSY			0x1F	// Y-axis offset 
#define REG_OFSZ			0x20	// Z-axis offset 
#define REG_DUR				0x21	// Tap duration 
#define REG_Latent			0x22	// Tap latency 
#define REG_Window			0x23	// Tap window 
#define REG_THRESH_ACT		0x24	// Activity threshold 
#define REG_THRESH_INACT	0x25	// Inactivity threshold 
#define REG_TIME_INACT		0x26	// Inactivity time 
#define REG_ACT_INACT_CTL	0x27	// Axis enable control for activity and inactivity detection 
#define REG_THRESH_FF		0x28	// Free-fall threshold 
#define REG_TIME_FF			0x29	// Free-fall time 
#define REG_TAP_AXES		0x2A	// Axis control for single tap/double tap 
#define REG_ACT_TAP_STATUS	0x2B	// Source of single tap/double tap 
#define REG_BW_RATE			0x2C	// Data rate and power mode control 
#define REG_POWER_CTL		0x2D	// Power-saving features control 
#define REG_INT_ENABLE		0x2E	// Interrupt enable control 
#define REG_INT_MAP			0x2F	// Interrupt mapping control 
#define REG_INT_SOURCE		0x30	// Source of interrupts 
#define REG_DATA_FORMAT		0x31	// Data format control 
#define REG_DATAX0			0x32	// X-Axis Data 0 
#define REG_DATAX1			0x33	// X-Axis Data 1 
#define REG_DATAY0			0x34	// Y-Axis Data 0 
#define REG_DATAY1			0x35	// Y-Axis Data 1 
#define REG_DATAZ0			0x36	// Z-Axis Data 0 
#define REG_DATAZ1			0x37	// Z-Axis Data 1 
#define REG_FIFO_CTL		0x38	// FIFO control 
#define REG_FIFO_STATUS		0x39	// FIFO status 
#define REG_TAP_SIGN		0x3A	// Sign and source for single tap/double tap 
#define REG_ORIENT_CONF		0x3B	// Orientation configuration 
#define REG_Orient			0x3C	// Orientation status

#endif /* ACCEL_REGISTERS_H_ */
