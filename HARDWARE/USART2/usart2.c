<<<<<<< HEAD
#include "usart2.h"	 	 
#include "gizwits_product.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4������
//����3��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/8/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   


//����3�жϷ�����		 
void USART3_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
	{	 
	    res =USART_ReceiveData(USART3);	
        gizPutData(&res, 1);//����д�뵽������		
	}
}		

////����3�жϷ�����	 
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

//��ʼ��IO ����3
//bound:������	  
void usart3_init(u32 bound)
{  

//	NVIC_InitTypeDef NVIC_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);      //ʹ��GPIOBʱ��
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);     //ʹ��USART3ʱ��

// 	USART_DeInit(USART2);                                     //��λ����3
//	
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);  //GPIOB11����ΪUSART3
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);  //GPIOB10����ΪUSART3	
//	
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;    //GPIOB11��GPIOB10��ʼ��
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //�ٶ�50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //����
//	GPIO_Init(GPIOA,&GPIO_InitStructure);                     //��ʼ��GPIOB11����GPIOB10
//	
//	USART_InitStructure.USART_BaudRate = bound;                                    //������ 
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //�շ�ģʽ
//  
//	USART_Init(USART2, &USART_InitStructure);                                      //��ʼ������3
// 
//	USART_Cmd(USART2, ENABLE);                                                     //ʹ�ܴ��� 
//	
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);            //�����ж�   
//	
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);      //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);     //ʹ��USART3ʱ��

 	USART_DeInit(USART3);                                     //��λ����3
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);  //GPIOB11����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  //GPIOB10����ΪUSART3	
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;    //GPIOB11��GPIOB10��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);                     //��ʼ��GPIOB11����GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;                                    //������ 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //�շ�ģʽ
  
	USART_Init(USART3, &USART_InitStructure);                                      //��ʼ������3
 
	USART_Cmd(USART3, ENABLE);                                                     //ʹ�ܴ���  
	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);            //�����ж� 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��VIC�Ĵ���

}


=======
#include "usart2.h"	 	 
#include "gizwits_product.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4������
//����3��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/8/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   


//����3�жϷ�����		 
void USART3_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
	{	 
	    res =USART_ReceiveData(USART3);	
        gizPutData(&res, 1);//����д�뵽������		
	}
}		

////����3�жϷ�����	 
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

//��ʼ��IO ����3
//bound:������	  
void usart3_init(u32 bound)
{  

//	NVIC_InitTypeDef NVIC_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);      //ʹ��GPIOBʱ��
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);     //ʹ��USART3ʱ��

// 	USART_DeInit(USART2);                                     //��λ����3
//	
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);  //GPIOB11����ΪUSART3
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);  //GPIOB10����ΪUSART3	
//	
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;    //GPIOB11��GPIOB10��ʼ��
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //�ٶ�50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //����
//	GPIO_Init(GPIOA,&GPIO_InitStructure);                     //��ʼ��GPIOB11����GPIOB10
//	
//	USART_InitStructure.USART_BaudRate = bound;                                    //������ 
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //�շ�ģʽ
//  
//	USART_Init(USART2, &USART_InitStructure);                                      //��ʼ������3
// 
//	USART_Cmd(USART2, ENABLE);                                                     //ʹ�ܴ��� 
//	
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);            //�����ж�   
//	
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);      //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);     //ʹ��USART3ʱ��

 	USART_DeInit(USART3);                                     //��λ����3
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);  //GPIOB11����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  //GPIOB10����ΪUSART3	
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;    //GPIOB11��GPIOB10��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);                     //��ʼ��GPIOB11����GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;                                    //������ 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //�շ�ģʽ
  
	USART_Init(USART3, &USART_InitStructure);                                      //��ʼ������3
 
	USART_Cmd(USART3, ENABLE);                                                     //ʹ�ܴ���  
	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);            //�����ж� 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��VIC�Ĵ���

}


>>>>>>> 563e9ece64f247a5130dd1a36575cf777e980d97
