<<<<<<< HEAD
#include "usart2.h"	 	 
#include "gizwits_product.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4开发板
//串口3驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/8/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   


//串口3中断服务函数		 
void USART3_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//接收到数据
	{	 
	    res =USART_ReceiveData(USART3);	
        gizPutData(&res, 1);//数据写入到缓冲区		
	}
}		

////串口3中断服务函数	 
//void USART3_IRQHandler(void)
//{
//	u8 res;	    
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//?????
//	{	 
//	    res =USART_ReceiveData(USART3);	
//        if (res == g_mkb0803_data.tc_data.cmd) g_mkb0803_data.rx_sta |= 0x4000;
//        if ((g_mkb0803_data.rx_sta & 0x4000) && (0 == (g_mkb0803_data.rx_sta & 0x8000))) {
//            g_mkb0803_data.rx_buf[g_mkb0803_data.rx_sta & 0x3FFF] = res;
//            g_mkb0803_data.rx_sta++;
//            if ((g_mkb0803_data.rx_sta&0x3FFF) == 6) g_mkb0803_data.rx_sta |= 0x8000;
//        }
//	}
//}	

//初始化IO 串口3
//bound:波特率	  
void usart3_init(u32 bound)
{  

//	NVIC_InitTypeDef NVIC_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);      //使能GPIOB时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);     //使能USART3时钟

// 	USART_DeInit(USART2);                                     //复位串口3
//	
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);  //GPIOB11复用为USART3
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);  //GPIOB10复用为USART3	
//	
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;    //GPIOB11和GPIOB10初始化
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
//	GPIO_Init(GPIOA,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
//	
//	USART_InitStructure.USART_BaudRate = bound;                                    //波特率 
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
//  
//	USART_Init(USART2, &USART_InitStructure);                                      //初始化串口3
// 
//	USART_Cmd(USART2, ENABLE);                                                     //使能串口 
//	
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);            //开启中断   
//	
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);      //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);     //使能USART3时钟

 	USART_DeInit(USART3);                                     //复位串口3
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);  //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  //GPIOB10复用为USART3	
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;    //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;                                    //波特率 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
  
	USART_Init(USART3, &USART_InitStructure);                                      //初始化串口3
 
	USART_Cmd(USART3, ENABLE);                                                     //使能串口  
	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);            //开启中断 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器

}


=======
#include "usart2.h"	 	 
#include "gizwits_product.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4开发板
//串口3驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/8/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   


//串口3中断服务函数		 
void USART3_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//接收到数据
	{	 
	    res =USART_ReceiveData(USART3);	
        gizPutData(&res, 1);//数据写入到缓冲区		
	}
}		

////串口3中断服务函数	 
//void USART3_IRQHandler(void)
//{
//	u8 res;	    
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//?????
//	{	 
//	    res =USART_ReceiveData(USART3);	
//        if (res == g_mkb0803_data.tc_data.cmd) g_mkb0803_data.rx_sta |= 0x4000;
//        if ((g_mkb0803_data.rx_sta & 0x4000) && (0 == (g_mkb0803_data.rx_sta & 0x8000))) {
//            g_mkb0803_data.rx_buf[g_mkb0803_data.rx_sta & 0x3FFF] = res;
//            g_mkb0803_data.rx_sta++;
//            if ((g_mkb0803_data.rx_sta&0x3FFF) == 6) g_mkb0803_data.rx_sta |= 0x8000;
//        }
//	}
//}	

//初始化IO 串口3
//bound:波特率	  
void usart3_init(u32 bound)
{  

//	NVIC_InitTypeDef NVIC_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);      //使能GPIOB时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);     //使能USART3时钟

// 	USART_DeInit(USART2);                                     //复位串口3
//	
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);  //GPIOB11复用为USART3
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);  //GPIOB10复用为USART3	
//	
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;    //GPIOB11和GPIOB10初始化
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
//	GPIO_Init(GPIOA,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
//	
//	USART_InitStructure.USART_BaudRate = bound;                                    //波特率 
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
//  
//	USART_Init(USART2, &USART_InitStructure);                                      //初始化串口3
// 
//	USART_Cmd(USART2, ENABLE);                                                     //使能串口 
//	
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);            //开启中断   
//	
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);      //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);     //使能USART3时钟

 	USART_DeInit(USART3);                                     //复位串口3
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);  //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  //GPIOB10复用为USART3	
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;    //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;                                    //波特率 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
  
	USART_Init(USART3, &USART_InitStructure);                                      //初始化串口3
 
	USART_Cmd(USART3, ENABLE);                                                     //使能串口  
	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);            //开启中断 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器

}


>>>>>>> 563e9ece64f247a5130dd1a36575cf777e980d97
