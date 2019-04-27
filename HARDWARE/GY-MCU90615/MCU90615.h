#ifndef __MCU90615_H__
#define __MCU90615_H__

#include "sys.h"

extern u8 TEMP_data[20], Receive_ok;

void MCU90615_Init(u32 bound);
void MCU90615_SendOut(int16_t *data,uint8_t length,uint8_t send);
void MCU90615_SendCommand(u8 data);

#endif
