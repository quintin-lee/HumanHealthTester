#include "MCU90615.h"
#include <string.h>

u8 TEMP_data[20]={0},Receive_ok=0;

void MCU90615_Init(u32 bound)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);      //ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);     //ʹ��USART6ʱ��

 	USART_DeInit(USART6);                                     //��λ����6
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_USART6);  //GPIOC6����ΪUSART6
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_USART6);  //GPIOC7����ΪUSART6	
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;    //GPIOC6��GPIOC7��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //?���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);                     //��ʼ��GPIOB11����GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;                                    //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //�շ�ģʽ
  
	USART_Init(USART6, &USART_InitStructure);                                      //��ʼ������3
 
	USART_Cmd(USART6, ENABLE);                                                     //ʹ�ܴ���
	
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);            //�����ж�  
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	NVIC_Init(&NVIC_InitStructure);	

    memset(TEMP_data, 0, sizeof(TEMP_data));
    memset(TEMP_data, 0, sizeof(TEMP_data));
}

void USART6_send_byte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);//�ȴ��������
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
		Buffer[Length-1]+=Buffer[i];//�ۼ�Length-1ǰ������
		USART6_send_byte(Buffer[i++]);
	}
}

void MCU90615_SendOut(int16_t *data,uint8_t length,uint8_t send)
{
	uint8_t TX_DATA[30],i=0,k=0;
	memset(TX_DATA,0,(2*length+5));//���㻺��TX_DATA
	TX_DATA[i++]=0X5A;//֡ͷ
	TX_DATA[i++]=0X5A;//֡ͷ
	TX_DATA[i++]=send;//�����ֽ�
	TX_DATA[i++]=2*length;//���ݸ���
	for(k=0;k<length;k++)//�������ݵ�����TX_DATA
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
	bytes[1]=data;//�����ֽ�
	USART6_Send(bytes,3);//����֡ͷ�������ֽڡ�У���
}

void USART6_IRQHandler(void)
{
	static uint8_t i=0,rebuf[20]={0};
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)//�жϽ��ձ�־
	{
		rebuf[i++]=USART_ReceiveData(USART6);//��ȡ�������ݣ�ͬʱ����ձ�־
		if (rebuf[0]!=0x5a)//֡ͷ����
			i=0;	
		if ((i==2)&&(rebuf[1]!=0x5a))//֡ͷ����
			i=0;
	
		if(i>3)//i����4ʱ���Ѿ����յ��������ֽ�rebuf[3]
		{
			if(i!=(rebuf[3]+5))//�ж��Ƿ����һ֡�������
				return;	
			switch(rebuf[2])//������Ϻ���
			{
				case 0x45:
					if(!Receive_ok)//�����ݴ�����ɺ�Ž����µ�����
					{
						memcpy(TEMP_data,rebuf,9);//�������յ�������
						Receive_ok=1;//������ɱ�־
					}
					break;
				case 0x15:break;
				case 0x35:break;
			}
			i=0;//������0
		}
	}
		
}




