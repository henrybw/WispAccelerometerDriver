/*
 * accelh
 *
 *  Created on: May 22, 2012
 *      Author: htw
 */

#ifndef ACCEL_REGISTERS_H_
#define ACCEL_REGISTERS_H_

#define 0x00 DEVID  // Device ID
#define 0x1D THRESH_TAP  // Tap threshold 
#define 0x1E OFSX  // X-axis offset 
#define 0x1F OFSY  // Y-axis offset 
#define 0x20 OFSZ  // Z-axis offset 
#define 0x21 DUR  // Tap duration 
#define 0x22 Latent  // Tap latency 
#define 0x23 Window  // Tap window 
#define 0x24 THRESH_ACT  // Activity threshold 
#define 0x25 THRESH_INACT  // Inactivity threshold 
#define 0x26 TIME_INACT  // Inactivity time 
#define 0x27 ACT_INACT_CTL  // Axis enable control for activity and inactivity detection 
#define 0x28 THRESH_FF  // Free-fall threshold 
#define 0x29 TIME_FF  // Free-fall time 
#define 0x2A TAP_AXES  // Axis control for single tap/double tap 
#define 0x2B ACT_TAP_STATUS  // Source of single tap/double tap 
#define 0x2C BW_RATE  // Data rate and power mode control 
#define 0x2D POWER_CTL  // Power-saving features control 
#define 0x2E INT_ENABLE  // Interrupt enable control 
#define 0x2F INT_MAP  // Interrupt mapping control 
#define 0x30 INT_SOURCE  // Source of interrupts 
#define 0x31 DATA_FORMAT  // Data format control 
#define 0x32 DATAX0  // X-Axis Data 0 
#define 0x33 DATAX1  // X-Axis Data 1 
#define 0x34 DATAY0  // Y-Axis Data 0 
#define 0x35 DATAY1  // Y-Axis Data 1 
#define 0x36 DATAZ0  // Z-Axis Data 0 
#define 0x37 DATAZ1  // Z-Axis Data 1 
#define 0x38 FIFO_CTL  // FIFO control 
#define 0x39 FIFO_STATUS  // FIFO status 
#define 0x3A TAP_SIGN  // Sign and source for single tap/double tap 
#define 0x3B ORIENT_CONF  // Orientation configuration 
#define 0x3C Orient  // Orientation status

#endif /* ACCEL_REGISTERS_H_ */
