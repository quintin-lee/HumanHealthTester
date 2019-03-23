#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "MKB0803.h"
#include "lcd.h"
#include "gizwits_product.h"
#include "timer.h"
	 

//ALIENTEK ̽����STM32F407������ ʵ��4
//����ͨ��ʵ�� -�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK
extern uint8_t wifi_sta;

int main(void)
{ 
    //u8 high, low, pluse;
	//u8 x=0;
	int key;
    u8 conn_sta = 0;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 
	uart_init(115200);	//���ڳ�ʼ��������Ϊ115200
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�  
	MKB0803_Init(115200);
	TIM3_Int_Init(10-1,8400-1);
	
	LCD_Init();           //��ʼ��LCD FSMC�ӿ�
	POINT_COLOR=RED;      //������ɫ����ɫ

    userInit();
	gizwitsInit();//Э���ʼ��
//	gizwitsSetMode(WIFI_AIRLINK_MODE);
//	delay_ms(200);
	while(1)
	{
        userHandle();//�û��ɼ�
		gizwitsHandle((dataPoint_t *)&currentDataPoint);//Э�鴦��
        //delay_ms(6000);
        
        if (conn_sta != wifi_sta)
        {
            wifi_sta ? printf("connect ok\r\n") : printf("disconnect \r\n");
            conn_sta = wifi_sta;
        }
		
		key = KEY_Scan(0);
		if(key==KEY1_PRES)//KEY1����
		{
			printf("WIFI����AirLink����ģʽ\r\n");
			gizwitsSetMode(WIFI_AIRLINK_MODE);//Air-linkģʽ����
		}			
		if(key==WKUP_PRES)//KEY_UP����
		{  
			printf("WIFI��λ����������������\r\n");
			gizwitsSetMode(WIFI_RESET_MODE);//WIFI��λ
		}
		delay_ms(200);
		
		/*
        if (!MKB0803_ReadData(&high, &low, &pluse))
        {
					LED0=0;
            printf("��ѹ:%d ��ѹ:%d ����:%d \n", high, low, pluse);
        }
		delay_ms(6000);
				
				switch(x)
		{
			case 0:LCD_Clear(WHITE);break;
			case 1:LCD_Clear(BLACK);break;
			case 2:LCD_Clear(BLUE);break;
			case 3:LCD_Clear(RED);break;
			case 4:LCD_Clear(MAGENTA);break;
			case 5:LCD_Clear(GREEN);break;
			case 6:LCD_Clear(CYAN);break; 
			case 7:LCD_Clear(YELLOW);break;
			case 8:LCD_Clear(BRRED);break;
			case 9:LCD_Clear(GRAY);break;
			case 10:LCD_Clear(LGRAY);break;
			case 11:LCD_Clear(BROWN);break;
		}
		POINT_COLOR=RED;
		LCD_ShowString(30,40,210,24,24,"high:");	
		LCD_ShowString(30,70,220,24,24,"low:");
		LCD_ShowString(30,100,230,24,24,"heart:");
		LCD_ShowNum(90,40,high,3,24);	
		LCD_ShowNum(90,70,low,3,24);
		LCD_ShowNum(100,100,pluse,3,24);
        */
    }
	
		
 		
}
