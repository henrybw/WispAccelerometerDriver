/*
 * accel.h
 *
 *  Created on: May 22, 2012
 *      Author: htw
 */

#ifndef ACCEL_H_
#define ACCEL_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <msp430g2553.h>  // TODO: change this to wispGuts.h when integrating into the WISP

// We should never have to read/write more than 32 bytes of data at once
#define MAX_TRANSMISSION_SIZE 32

void Accel_Init(void);
void Accel_Read(int16_t *x, int16_t *y, int16_t *z);
void Accel_ReadVector(int16_t data[]);

#endif /* ACCEL_H_ */
