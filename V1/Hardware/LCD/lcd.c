/****************************************************************************************************
//=========================================��Դ����================================================//
//     LCDģ��                STM32��Ƭ��
//      VCC          ��        DC5V/3.3V      //��Դ
//      GND          ��          GND          //��Դ��
//=======================================Һ���������߽���==========================================//
//��ģ��Ĭ��������������ΪSPI����
//     LCDģ��                STM32��Ƭ��    
//    SDI(MOSI)      ��          PB5          //Һ����SPI��������д�ź�
//    SDO(MISO)      ��          PB4          //Һ����SPI�������ݶ��źţ��������Ҫ�������Բ�����
//=======================================Һ���������߽���==========================================//
//     LCDģ�� 					      STM32��Ƭ�� 
//       SCK         ��          PB3          //Һ����SPI����ʱ���ź�
//       LED         ��          PE9         //Һ������������źţ��������Ҫ���ƣ���5V��3.3V
//      DC/RS        ��          PE10         //Һ��������/��������ź�
//       RST         ��          PE8         //Һ������λ�����ź�
//       CS          ��          PE11        //Һ����Ƭѡ�����ź�
//=========================================������������=========================================//
//���ģ�鲻���������ܻ��ߴ��д������ܣ����ǲ���Ҫ�������ܣ�����Ҫ���д���������
//	   LCDģ��                STM32��Ƭ�� 
//      T_IRQ        ��          PB12          //�����������ж��ź�
//      T_DO         ��          PB14          //������SPI���߶��ź�
//      T_DIN        ��          PB15         //������SPI����д�ź�
//      T_CS         ��          PA11          //������Ƭѡ�����ź�
//      T_CLK        ��          PB13          //������SPI����ʱ���ź�
**************************************************************************************************/	
#include "lcd.h"
#include "stdlib.h" 
#include "spi.h"


//����LCD��Ҫ����
//Ĭ��Ϊ����
_lcd_dev lcddev;

//������ɫ,������ɫ
uint16_t POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
uint16_t DeviceCode;

////KAI�Զ���
//#define	LCD_CS_SET  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET)
//#define	LCD_RS_SET		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET)
//#define	LCD_RST_SET		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET)
//#define LCD_LED_SET		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET)
//		    
//#define	LCD_CS_CLR  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)
//#define	LCD_RS_CLR		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET)
//#define	LCD_RST_CLR		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET)
//#define LCD_LED_CLR		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET)

/*****************************************************************************
 * @name       :void LCD_WR_REG(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(uint8_t data)
{ 
   LCD_CS_CLR;     
   LCD_RS_CLR;	  
   SPI1_TransmitReceive(data);
   LCD_CS_SET;	
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(uint8_t data)
{
   LCD_CS_CLR;
   LCD_RS_SET;
   SPI1_TransmitReceive(data);
   LCD_CS_SET;
}

uint8_t LCD_RD_DATA(void)
{
	 uint8_t data;
	 LCD_CS_CLR;
	 LCD_RS_SET;
//	 SPI_SetSpeed(SPI1,0);
	 data = SPI1_TransmitReceive(0xFF);
//	 SPI_SetSpeed(SPI1,1);
	 LCD_CS_SET;
	 return data;
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(u8 LCD_Reg, uint16_t LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   

uint8_t LCD_ReadReg(uint8_t LCD_Reg)
{
	LCD_WR_REG(LCD_Reg);
  return LCD_RD_DATA();
}

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.rramcmd);
}

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(uint16_t Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(uint16_t Data)
{	
   LCD_CS_CLR;
   LCD_RS_SET;  
   SPI1_TransmitReceive(Data>>8);
   SPI1_TransmitReceive(Data);
   LCD_CS_SET;
}

uint16_t Color_To_565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

uint16_t Lcd_ReadData_16Bit(void)
{
	uint8_t r,g,b;
	LCD_RS_SET;
	LCD_CS_CLR;
//	SPI_SetSpeed(SPI1,0);
	//dummy data
	SPI1_TransmitReceive(0xFF);
	//8bit:red data
	//16bit:red and green data
	r=SPI1_TransmitReceive(0xFF);
	
	//8bit:green data
	//16bit:blue data
	g=SPI1_TransmitReceive(0xFF);
	
	//blue data
	b=SPI1_TransmitReceive(0xFF);
	//r >>= 8;
	//g >>= 8;
	//b >>= 8;
//	SPI_SetSpeed(SPI1,1);
	LCD_CS_SET;
	return Color_To_565(r, g, b);
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(uint16_t x,uint16_t y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);//���ù��λ�� 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);//���ù��λ�� 
	LCD_ReadRAM_Prepare();
	return Lcd_ReadData_16Bit();
}

/*****************************************************************************
 * @name       :void LCD_Clear(uint16_t Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clear(uint16_t Color)
{
  unsigned int i,m;  
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	LCD_CS_CLR;
	LCD_RS_SET;
	for(i=0;i<lcddev.height;i++)
	{
    for(m=0;m<lcddev.width;m++)
    {	
			Lcd_WriteData_16Bit(Color);
		}
	}
	 LCD_CS_SET;
} 

/*****************************************************************************
 * @name       :void LCD_Clear(uint16_t Color)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
//void LCD_GPIOInit(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	      
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB ,ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12| GPIO_Pin_13|GPIO_Pin_14| GPIO_Pin_15;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //�������
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	LCD_RST_SET;
	delay_ms(50);
	LCD_RST_CLR;
	delay_ms(100);	
	LCD_RST_SET;
	delay_ms(50);
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{
	MX_SPI1_Init(); //Ӳ��SPI��ʼ��
//	SPI_SetSpeed(SPI1,SPI_BaudRatePrescaler_2);
//	LCD_GPIOInit();//LCD GPIO��ʼ��										 
 	LCD_RESET(); //LCD ��λ
//*************2.8inch ILI9341��ʼ��**********//	
	LCD_WR_REG(0xCF);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0xC1); 
	LCD_WR_DATA(0x30); 
 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA(0x64); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0X12); 
	LCD_WR_DATA(0X81); 
 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA(0x85); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x78); 

	LCD_WR_REG(0xCB);  
	LCD_WR_DATA(0x39); 
	LCD_WR_DATA(0x2C); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x02); 
	
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA(0x20); 
 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 

	LCD_WR_REG(0xC0);       //Power control 
	LCD_WR_DATA(0x13);     //VRH[5:0] 
 
	LCD_WR_REG(0xC1);       //Power control 
	LCD_WR_DATA(0x13);     //SAP[2:0];BT[3:0] 
 
	LCD_WR_REG(0xC5);       //VCM control 
	LCD_WR_DATA(0x22);   //22
	LCD_WR_DATA(0x35);   //35
 
	LCD_WR_REG(0xC7);       //VCM control2 
	LCD_WR_DATA(0xBD);  //AF

	LCD_WR_REG(0x21);

	LCD_WR_REG(0x36);       // Memory Access Control 
	LCD_WR_DATA(0x08); 

	LCD_WR_REG(0xB6);  
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0xA2); 

	LCD_WR_REG(0x3A);       
	LCD_WR_DATA(0x55); 

	LCD_WR_REG(0xF6);  //Interface Control
	LCD_WR_DATA(0x01); 
	LCD_WR_DATA(0x30);  //MCU

	LCD_WR_REG(0xB1);       //VCM control 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x1B); 
 
	LCD_WR_REG(0xF2);       // 3Gamma Function Disable 
	LCD_WR_DATA(0x00); 
 
	LCD_WR_REG(0x26);       //Gamma curve selected 
	LCD_WR_DATA(0x01); 
 
	LCD_WR_REG(0xE0);       //Set Gamma 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x35); 
	LCD_WR_DATA(0x31); 
	LCD_WR_DATA(0x0B); 
	LCD_WR_DATA(0x0E); 
	LCD_WR_DATA(0x06); 
	LCD_WR_DATA(0x49); 
	LCD_WR_DATA(0xA7); 
	LCD_WR_DATA(0x33); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0x0C); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x00); 
 
	LCD_WR_REG(0XE1);       //Set Gamma 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x04); 
	LCD_WR_DATA(0x11); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x36); 
	LCD_WR_DATA(0x58); 
	LCD_WR_DATA(0x4D); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x0C); 
	LCD_WR_DATA(0x32); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x0F); 

	LCD_WR_REG(0x11);       //Exit Sleep 
	delay_ms(120);
	LCD_WR_REG(0x29); //display on		

    LCD_direction(USE_HORIZONTAL+2);//����LCD��ʾ����
	LCD_LED_SET;//��������	 
	LCD_Clear(WHITE);//��ȫ����ɫ
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//��ʼд��GRAM			
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(uint8_t direction)
{ 
	lcddev.setxcmd=0x2A;
	lcddev.setycmd=0x2B;
	lcddev.wramcmd=0x2C;
	lcddev.rramcmd=0x2E;
			lcddev.dir = direction%4;
	switch(lcddev.dir){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3)|(0<<6)|(0<<7));//BGR==1,MY==0,MX==0,MV==0
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(0<<7)|(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;	
		default:break;
	}

}	 

uint16_t LCD_Read_ID(void)
{
uint8_t i,val[3] = {0};
	for(i=1;i<4;i++)
	{
		LCD_WR_REG(0xD9);
		LCD_WR_DATA(0x10+i);
		LCD_WR_REG(0xD3);
		val[i-1] = LCD_RD_DATA();
	}
	lcddev.id=val[1];
	lcddev.id<<=8;
	lcddev.id|=val[2];
	return lcddev.id;
}
