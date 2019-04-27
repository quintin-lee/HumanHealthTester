#include "MCU90615.h"
#include <string.h>

u8 TEMP_data[20]={0},Receive_ok=0;

void MCU90615_Init(u32 bound)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);      //使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);     //使能USART6时钟

 	USART_DeInit(USART6);                                     //复位串口6
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_USART6);  //GPIOC6复用为USART6
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_USART6);  //GPIOC7复用为USART6	
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;    //GPIOC6和GPIOC7初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //?复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;                                    //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
  
	USART_Init(USART6, &USART_InitStructure);                                      //初始化串口3
 
	USART_Cmd(USART6, ENABLE);                                                     //使能串口
	
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);            //开启中断  
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//根据指定的参数初始化VIC寄存器
	NVIC_Init(&NVIC_InitStructure);	

    memset(TEMP_data, 0, sizeof(TEMP_data));
    memset(TEMP_data, 0, sizeof(TEMP_data));
}

void USART6_send_byte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);//等待发送完成
	USART6->DR=byte;	
}

void USART6_Send_bytes(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USART6_send_byte(Buffer[i++]);
	}
}

void USART6_Send(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];//累加Length-1前的数据
		USART6_send_byte(Buffer[i++]);
	}
}

void MCU90615_SendOut(int16_t *data,uint8_t length,uint8_t send)
{
	uint8_t TX_DATA[30],i=0,k=0;
	memset(TX_DATA,0,(2*length+5));//清零缓存TX_DATA
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=send;//功能字节
	TX_DATA[i++]=2*length;//数据个数
	for(k=0;k<length;k++)//存入数据到缓存TX_DATA
	{
		TX_DATA[i++]=(uint16_t)data[k]>>8;
		TX_DATA[i++]=(uint16_t)data[k];
	}
	USART6_Send(TX_DATA,2*length+5);	
}

void MCU90615_SendCommand(u8 data)
{
	u8 bytes[3]={0};
	bytes[0]=0xa5;
	bytes[1]=data;//功能字节
	USART6_Send(bytes,3);//发送帧头、功能字节、校验和
}

void USART6_IRQHandler(void)
{
	static uint8_t i=0,rebuf[20]={0};
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)//判断接收标志
	{
		rebuf[i++]=USART_ReceiveData(USART6);//读取串口数据，同时清接收标志
		if (rebuf[0]!=0x5a)//帧头不对
			i=0;	
		if ((i==2)&&(rebuf[1]!=0x5a))//帧头不对
			i=0;
	
		if(i>3)//i等于4时，已经接收到数据量字节rebuf[3]
		{
			if(i!=(rebuf[3]+5))//判断是否接收一帧数据完毕
				return;	
			switch(rebuf[2])//接收完毕后处理
			{
				case 0x45:
					if(!Receive_ok)//当数据处理完成后才接收新的数据
					{
						memcpy(TEMP_data,rebuf,9);//拷贝接收到的数据
						Receive_ok=1;//接收完成标志
					}
					break;
				case 0x15:break;
				case 0x35:break;
			}
			i=0;//缓存清0
		}
	}
		
}




