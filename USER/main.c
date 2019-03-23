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
	 

//ALIENTEK 探索者STM32F407开发板 实验4
//串口通信实验 -库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK
extern uint8_t wifi_sta;

int main(void)
{ 
    //u8 high, low, pluse;
	//u8 x=0;
	int key;
    u8 conn_sta = 0;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(115200);	//串口初始化波特率为115200
	LED_Init();		  		//初始化与LED连接的硬件接口  
	MKB0803_Init(115200);
	TIM3_Int_Init(10-1,8400-1);
	
	LCD_Init();           //初始化LCD FSMC接口
	POINT_COLOR=RED;      //画笔颜色：红色

    userInit();
	gizwitsInit();//协议初始化
//	gizwitsSetMode(WIFI_AIRLINK_MODE);
//	delay_ms(200);
	while(1)
	{
        userHandle();//用户采集
		gizwitsHandle((dataPoint_t *)&currentDataPoint);//协议处理
        //delay_ms(6000);
        
        if (conn_sta != wifi_sta)
        {
            wifi_sta ? printf("connect ok\r\n") : printf("disconnect \r\n");
            conn_sta = wifi_sta;
        }
		
		key = KEY_Scan(0);
		if(key==KEY1_PRES)//KEY1按键
		{
			printf("WIFI进入AirLink连接模式\r\n");
			gizwitsSetMode(WIFI_AIRLINK_MODE);//Air-link模式接入
		}			
		if(key==WKUP_PRES)//KEY_UP按键
		{  
			printf("WIFI复位，请重新配置连接\r\n");
			gizwitsSetMode(WIFI_RESET_MODE);//WIFI复位
		}
		delay_ms(200);
		
		/*
        if (!MKB0803_ReadData(&high, &low, &pluse))
        {
					LED0=0;
            printf("高压:%d 低压:%d 心率:%d \n", high, low, pluse);
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
