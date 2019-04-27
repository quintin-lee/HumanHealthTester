<<<<<<< HEAD
#include "MKB0803.h"
#include "sys.h"
#include <string.h>

//uart3_data_t g_mkb0803_data;
uart2_data_t g_mkb0803_data;

// MKB0803 命令
#define  ADJUST    0xFE    // 校准命令，根据血压计测量的真实准确值设置
#define  READ_DAT  0xFD    // 读取命令
#define  READ_PUL  0xFC    // 读取脉搏信号
#define  CLEAN     0xFA    // 擦除校准信息
#define  READ_ECG  0xF9    // 读取ECG信号
#define  READ_STA  0xF8    // 读取工作状态

void MKB0803_Init(u32 bound)
{
//	NVIC_InitTypeDef NVIC_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);      //使能GPIOB时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);     //使能USART3时钟

// 	USART_DeInit(USART3);                                     //复位串口3
//	
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);  //GPIOB11复用为USART3
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  //GPIOB10复用为USART3	
//	
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;    //GPIOB11和GPIOB10初始化
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
//	GPIO_Init(GPIOB,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
//	
//	USART_InitStructure.USART_BaudRate = bound;                                    //波特率 
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
//  
//	USART_Init(USART3, &USART_InitStructure);                                      //初始化串口3
// 
//	USART_Cmd(USART3, ENABLE);                                                     //使能串口 
//	
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);            //开启中断   
//	
//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器


	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);      //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);     //使能USART3时钟

 	USART_DeInit(USART2);                                     //复位串口3
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);  //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);  //GPIOB10复用为USART3	
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;    //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //¸复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;                                    //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
  
	USART_Init(USART2, &USART_InitStructure);                                      //初始化串口3
 
	USART_Cmd(USART2, ENABLE);                                                     //使能串口
	
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);            //开启中断  
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//根据指定的参数初始化VIC寄存器
	NVIC_Init(&NVIC_InitStructure);	

    memset(g_mkb0803_data.rx_buf, 0, USART2_REC_LEN);
    memset(g_mkb0803_data.tc_buf, 0, USART2_REC_LEN);
    g_mkb0803_data.rx_sta = 0;
    g_mkb0803_data.tc_data.cmd = 0xFD;
}

//串口3中断服务函数	 
void USART2_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
	{	 
	    res =USART_ReceiveData(USART2);	
        if (res == g_mkb0803_data.tc_data.cmd) g_mkb0803_data.rx_sta |= 0x4000;
        if ((g_mkb0803_data.rx_sta & 0x4000) && (0 == (g_mkb0803_data.rx_sta & 0x8000))) {
            g_mkb0803_data.rx_buf[g_mkb0803_data.rx_sta & 0x3FFF] = res;
            g_mkb0803_data.rx_sta++;
            if ((g_mkb0803_data.rx_sta&0x3FFF) == 6) g_mkb0803_data.rx_sta |= 0x8000;
        }
	}
}	

////串口2中断服务函数	 
//void USART2_IRQHandler(void)
//{
//	u8 res;	    
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
//	{	 
//	    res =USART_ReceiveData(USART2);	
//        gizPutData(&res, 1);//数据写入缓冲区		
//	}
//}

/**
 * @brief  : 向MKB0803模块发送命令
 * @param  : 发送的命令
 * @return : none
 */
void MKB0803_SendCmd(data_t *cmd)
{
    u8 n = 0;
    g_mkb0803_data.tc_data = *cmd;
    for (n = 0; n < sizeof(data_t); n++)
    {
        USART_SendData(USART2, g_mkb0803_data.tc_buf[n]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);
    }
}

/**
 * @brief  : 校准MKB0803模块的血压信息, 以标准值去输入校验
 * @param  : high,高压值
 *           low,低压值
 *           pluse,心率
 * @return : none
 */
void MKB0803_Adjust(u8 high, u8 low, u8 pluse)
{
    data_t cmd;
    memset(&cmd, 0, sizeof(data_t));
    cmd.cmd = ADJUST;
    cmd.data1 = high;
    cmd.data2 = low;
    cmd.data3 = pluse;
    MKB0803_SendCmd(&cmd);
}

/**
 * @brief  : 读取血压心率数据
 * @param  : high,高
 *           low,低压值
 *           pluse,心率
 * @return : 0,success 1,failed
 */
u16 MKB0803_ReadData(u8 *high, u8 *low, u8 *pluse)
{
    data_t cmd;
    
    if (0 == high || 0 == low || 0 == pluse) return 1;
    
    memset(&cmd, 0, sizeof(data_t));
    cmd.cmd = READ_DAT;

    while(!(g_mkb0803_data.rx_sta&0x8000)) MKB0803_SendCmd(&cmd);
    *high = g_mkb0803_data.rx_data.data1;
    *low = g_mkb0803_data.rx_data.data2;
    *pluse = g_mkb0803_data.rx_data.data3;
    g_mkb0803_data.rx_sta = 0;
    
    return 0;
}

/**
 * @brief  : 读取MKB0803工作状态
 * @param  : sta, 状态值
 * @return : 0,success 1,failed
 */
u16 MKB0803_ReadStatus(u8 *sta)
{
    data_t cmd;
    
    if (0 == sta ) return 1;
    
    memset(&cmd, 0, sizeof(data_t));
    cmd.cmd = READ_STA;

    while(!(g_mkb0803_data.rx_sta&0x8000)) MKB0803_SendCmd(&cmd);
    *sta = g_mkb0803_data.rx_data.data3;
    g_mkb0803_data.rx_sta = 0;
    
    return 0;
}



=======
#include "MKB0803.h"
#include "sys.h"
#include <string.h>

//uart3_data_t g_mkb0803_data;
uart2_data_t g_mkb0803_data;

// MKB0803 命令
#define  ADJUST    0xFE    // 校准命令，根据血压计测量的真实准确值设置
#define  READ_DAT  0xFD    // 读取命令
#define  READ_PUL  0xFC    // 读取脉搏信号
#define  CLEAN     0xFA    // 擦除校准信息
#define  READ_ECG  0xF9    // 读取ECG信号
#define  READ_STA  0xF8    // 读取工作状态

void MKB0803_Init(u32 bound)
{
//	NVIC_InitTypeDef NVIC_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);      //使能GPIOB时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);     //使能USART3时钟

// 	USART_DeInit(USART3);                                     //复位串口3
//	
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);  //GPIOB11复用为USART3
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  //GPIOB10复用为USART3	
//	
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;    //GPIOB11和GPIOB10初始化
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
//	GPIO_Init(GPIOB,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
//	
//	USART_InitStructure.USART_BaudRate = bound;                                    //波特率 
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
//  
//	USART_Init(USART3, &USART_InitStructure);                                      //初始化串口3
// 
//	USART_Cmd(USART3, ENABLE);                                                     //使能串口 
//	
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);            //开启中断   
//	
//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器


	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);      //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);     //使能USART3时钟

 	USART_DeInit(USART2);                                     //复位串口3
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);  //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);  //GPIOB10复用为USART3	
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;    //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //¸复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;                                    //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
  
	USART_Init(USART2, &USART_InitStructure);                                      //初始化串口3
 
	USART_Cmd(USART2, ENABLE);                                                     //使能串口
	
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);            //开启中断  
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//根据指定的参数初始化VIC寄存器
	NVIC_Init(&NVIC_InitStructure);	

    memset(g_mkb0803_data.rx_buf, 0, USART2_REC_LEN);
    memset(g_mkb0803_data.tc_buf, 0, USART2_REC_LEN);
    g_mkb0803_data.rx_sta = 0;
    g_mkb0803_data.tc_data.cmd = 0xFD;
}

//串口3中断服务函数	 
void USART2_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
	{	 
	    res =USART_ReceiveData(USART2);	
        if (res == g_mkb0803_data.tc_data.cmd) g_mkb0803_data.rx_sta |= 0x4000;
        if ((g_mkb0803_data.rx_sta & 0x4000) && (0 == (g_mkb0803_data.rx_sta & 0x8000))) {
            g_mkb0803_data.rx_buf[g_mkb0803_data.rx_sta & 0x3FFF] = res;
            g_mkb0803_data.rx_sta++;
            if ((g_mkb0803_data.rx_sta&0x3FFF) == 6) g_mkb0803_data.rx_sta |= 0x8000;
        }
	}
}	

////串口2中断服务函数	 
//void USART2_IRQHandler(void)
//{
//	u8 res;	    
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
//	{	 
//	    res =USART_ReceiveData(USART2);	
//        gizPutData(&res, 1);//数据写入缓冲区		
//	}
//}

/**
 * @brief  : 向MKB0803模块发送命令
 * @param  : 发送的命令
 * @return : none
 */
void MKB0803_SendCmd(data_t *cmd)
{
    u8 n = 0;
    g_mkb0803_data.tc_data = *cmd;
    for (n = 0; n < sizeof(data_t); n++)
    {
        USART_SendData(USART2, g_mkb0803_data.tc_buf[n]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);
    }
}

/**
 * @brief  : 校准MKB0803模块的血压信息, 以标准值去输入校验
 * @param  : high,高压值
 *           low,低压值
 *           pluse,心率
 * @return : none
 */
void MKB0803_Adjust(u8 high, u8 low, u8 pluse)
{
    data_t cmd;
    memset(&cmd, 0, sizeof(data_t));
    cmd.cmd = ADJUST;
    cmd.data1 = high;
    cmd.data2 = low;
    cmd.data3 = pluse;
    MKB0803_SendCmd(&cmd);
}

/**
 * @brief  : 读取血压心率数据
 * @param  : high,高
 *           low,低压值
 *           pluse,心率
 * @return : 0,success 1,failed
 */
u16 MKB0803_ReadData(u8 *high, u8 *low, u8 *pluse)
{
    data_t cmd;
    
    if (0 == high || 0 == low || 0 == pluse) return 1;
    
    memset(&cmd, 0, sizeof(data_t));
    cmd.cmd = READ_DAT;

    while(!(g_mkb0803_data.rx_sta&0x8000)) MKB0803_SendCmd(&cmd);
    *high = g_mkb0803_data.rx_data.data1;
    *low = g_mkb0803_data.rx_data.data2;
    *pluse = g_mkb0803_data.rx_data.data3;
    g_mkb0803_data.rx_sta = 0;
    
    return 0;
}

/**
 * @brief  : 读取MKB0803工作状态
 * @param  : sta, 状态值
 * @return : 0,success 1,failed
 */
u16 MKB0803_ReadStatus(u8 *sta)
{
    data_t cmd;
    
    if (0 == sta ) return 1;
    
    memset(&cmd, 0, sizeof(data_t));
    cmd.cmd = READ_STA;

    while(!(g_mkb0803_data.rx_sta&0x8000)) MKB0803_SendCmd(&cmd);
    *sta = g_mkb0803_data.rx_data.data3;
    g_mkb0803_data.rx_sta = 0;
    
    return 0;
}



>>>>>>> 563e9ece64f247a5130dd1a36575cf777e980d97
