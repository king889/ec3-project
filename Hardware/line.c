/*
 * Hardware/line.c
 * Simple GPIO wrappers to read PA8..PA12 as a 5-bit pattern for line sensors
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "line.h"

/* Initialize PA8..PA12 as input pull-up. Change to GPIO_Mode_IN_FLOATING
   if you need floating inputs. */
void LinePins_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; /* input pull-up */
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* Read PA8..PA12 and return mask with bit4..bit0 = PA8..PA12 */
uint8_t Read_A8_A12_mask(void)
{
	uint8_t mask = 0;

	mask |= (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) ? (1 << 4) : 0);
	mask |= (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9) ? (1 << 3) : 0);
	mask |= (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10) ? (1 << 2) : 0);
	mask |= (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) ? (1 << 1) : 0);
	mask |= (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) ? (1 << 0) : 0);

	return mask;
}

/* Fill buf with "10101" text (buf must be >=6 bytes including terminating '\0') */
void Read_A8_A12_str(char *buf)
{
	if (!buf) return;
	buf[0] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) ? '1' : '0';
	buf[1] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9) ? '1' : '0';
	buf[2] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10) ? '1' : '0';
	buf[3] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) ? '1' : '0';
	buf[4] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) ? '1' : '0';
	buf[5] = '\0';
}

/* Inverted versions (useful when sensors are active-low) */
uint8_t Read_A8_A12_mask_inverted(void)
{
	return (~Read_A8_A12_mask()) & 0x1F; /* keep only 5 bits */
}

void Read_A8_A12_str_inverted(char *buf)
{
	if (!buf) return;
	uint8_t m = Read_A8_A12_mask_inverted();
	for (int i = 0; i < 5; ++i) {
		buf[i] = (m & (1 << (4 - i))) ? '1' : '0';
	}
	buf[5] = '\0';
}

/* Optional small helper: format mask to string without reading pins (buf>=6) */
void MaskToStr(uint8_t mask, char *buf)
{
	if (!buf) return;
	for (int i = 0; i < 5; ++i) {
		buf[i] = (mask & (1 << (4 - i))) ? '1' : '0';
	}
	buf[5] = '\0';
}

