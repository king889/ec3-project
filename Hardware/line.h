#ifndef __LINE_H
#define __LINE_H

#include "stm32f10x.h"

/* Initialize PA8..PA12 as inputs (pull-up) for line sensors */
void LinePins_Init(void);

/* Read PA8..PA12 and return a 5-bit mask:
	bit4..bit0 = PA8,PA9,PA10,PA11,PA12
	Example: PA8=1,PA9=0,PA10=1,PA11=0,PA12=1 -> 0b10101 -> 0x15 */
uint8_t Read_A8_A12_mask(void);

/* Write PA8..PA12 state into provided buffer as "10101" (buf must be >=6 bytes) */
void Read_A8_A12_str(char *buf);

/* Inverted helpers (if sensor logic is active-low) */
uint8_t Read_A8_A12_mask_inverted(void);
void Read_A8_A12_str_inverted(char *buf);

#endif /* __LINE_H */

