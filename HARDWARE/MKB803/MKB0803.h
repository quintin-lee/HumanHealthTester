#ifndef __MKB0803_H__
#define __MKB0803_H__

#include "sys.h"

#define USART2_REC_LEN  			6  	//定义最大接收字节数 6

typedef struct data_struct{
    u8 cmd;
    u8 data1;
    u8 data2;
    u8 data3;
    u8 crc0;
    u8 crc1;
}data_t;

typedef struct UART2_Data_struct {
    union{
        u8  buf[USART2_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节
        struct data_struct data;
    }rec;
    
    union{
        u8  buf[USART2_REC_LEN]; //发送缓冲,最大USART_REC_LEN个字节
        struct data_struct data;
    }send;
    u16 rx_sta;         		//接收状态标记	
} uart2_data_t;

#define rx_buf  rec.buf
#define rx_data rec.data
#define tc_buf  send.buf
#define tc_data send.data

//extern uart3_data_t g_mkb0803_data;
extern uart2_data_t g_mkb0803_data;

void MKB0803_Init(u32 bound);
void MKB0803_SendCmd(data_t *cmd);
void MKB0803_Adjust(u8 high, u8 low, u8 pluse);
u16 MKB0803_ReadData(u8 *high, u8 *low, u8 *pluse);

#endif
