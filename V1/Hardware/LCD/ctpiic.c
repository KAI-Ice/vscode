//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32F407ZGT6,����ԭ��Explorer STM32F4������,��Ƶ168MHZ������12MHZ
//QDtech-TFTҺ������ for STM32 IOģ��
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567 
//�ֻ�:15989313508���빤�� 
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/08/09
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
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
//     CTP_INT       ��          PE2          //���ݴ������ж��ź�
//     CTP_SDA       ��          PB10         //���ݴ�����IIC�����ź�
//     CTP_RST       ��          PG15          //���ݴ�������λ�ź�
//     CTP_SCL       ��          PB11          //���ݴ�����IICʱ���ź�
**************************************************************************************************/	
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/
#include "ctpiic.h"
#include "delay.h"	 


#define CTP_IIC_SCL(n)	HAL_GPIO_WritePin(CTP_SCL_GPIO_Port, CTP_SCL_Pin, n)
#define CTP_IIC_SDA(n)	HAL_GPIO_WritePin(CTP_SDA_GPIO_Port, CTP_SDA_Pin, n)
#define CTP_READ_SDA	HAL_GPIO_ReadPin(CTP_SDA_GPIO_Port,CTP_SDA_Pin)

/*****************************************************************************
 * @name       :void CTP_Delay(void)
 * @date       :2020-05-13 
 * @function   :Delay in controlling IIC speed
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void CTP_Delay(void)
{
	delay_us(1);
} 

/*****************************************************************************
 * @name       :void CTP_IIC_Init(void)
 * @date       :2020-05-13 
 * @function   :Initialize IIC
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void CTP_IIC_Init(void)
{	
//	GPIO_InitTypeDef GPIO_Initure;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOF, ENABLE);

//	GPIO_Initure.GPIO_Pin=GPIO_Pin_0; 	//PB0
//	GPIO_Initure.GPIO_Mode = GPIO_Mode_OUT;//���
//	GPIO_Initure.GPIO_OType = GPIO_OType_PP;//����
//	GPIO_Initure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Initure.GPIO_PuPd = GPIO_PuPd_UP;//����		
//	GPIO_Init(GPIOB, &GPIO_Initure);	
//	
//	GPIO_Initure.GPIO_Pin=GPIO_Pin_11; 	//PF11
//	GPIO_Init(GPIOF, &GPIO_Initure);	
	
	CTP_IIC_SCL(1);
	CTP_IIC_SDA(1);
}

/*****************************************************************************
 * @name       :void CTP_IIC_Start(void)
 * @date       :2020-05-13 
 * @function   :Generating IIC starting signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void CTP_IIC_Start(void)
{
	CTP_SDA_OUT();     //sda�����
	CTP_IIC_SDA(1);	  	  
	CTP_IIC_SCL(1);
	CTP_Delay();
 	CTP_IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	CTP_Delay();
	CTP_IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	

/*****************************************************************************
 * @name       :void CTP_IIC_Stop(void)
 * @date       :2020-05-13 
 * @function   :Generating IIC stop signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/   
void CTP_IIC_Stop(void)
{ 
	CTP_SDA_OUT();//sda�����
	CTP_IIC_SCL(0);
	CTP_IIC_SDA(0);
	CTP_Delay();
	CTP_IIC_SCL(1);
	CTP_Delay();
	CTP_IIC_SDA(1);//STOP:when CLK is high DATA change form low to high 
}

/*****************************************************************************
 * @name       :uint8_t CTP_IIC_Wait_Ack(void)
 * @date       :2020-05-13 
 * @function   :Wait for the response signal
 * @parameters :None
 * @retvalue   :0-receive response signal successfully
								1-receive response signal unsuccessfully
******************************************************************************/ 
uint8_t CTP_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	CTP_SDA_IN();      //SDA����Ϊ����  
	CTP_IIC_SDA(1);
	delay_us(1);	   
	CTP_IIC_SCL(1);
	delay_us(1);	 
	while(CTP_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			CTP_IIC_Stop();
			return 1;
		} 
	}
	CTP_IIC_SCL(0);//ʱ�����0 	   
	return 0;  
} 

/*****************************************************************************
 * @name       :void CTP_IIC_Ack(void)
 * @date       :2020-05-13 
 * @function   :Generate ACK response signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/ 
void CTP_IIC_Ack(void)
{
	CTP_IIC_SCL(0);
	CTP_SDA_OUT();
	CTP_IIC_SDA(0);
	CTP_Delay();
	CTP_IIC_SCL(1);
	CTP_Delay();
	CTP_IIC_SCL(0);
}

/*****************************************************************************
 * @name       :void CTP_IIC_NAck(void)
 * @date       :2020-05-13 
 * @function   :Don't generate ACK response signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	    
void CTP_IIC_NAck(void)
{
	CTP_IIC_SCL(0);
	CTP_SDA_OUT();
	CTP_IIC_SDA(1);
	CTP_Delay();
	CTP_IIC_SCL(1);
	CTP_Delay();
	CTP_IIC_SCL(0);
}	

/*****************************************************************************
 * @name       :void CTP_IIC_Send_Byte(uint8_t txd)
 * @date       :2020-05-13 
 * @function   :send a byte data by IIC bus
 * @parameters :txd:Data to be sent
 * @retvalue   :None
******************************************************************************/					 				     		  
void CTP_IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;   
	CTP_SDA_OUT(); 	    
  CTP_IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
  for(t=0;t<8;t++)
  {              
    CTP_IIC_SDA((txd&0x80)>>7);
    txd<<=1; 	      
	CTP_IIC_SCL(1);
	CTP_Delay();
	CTP_IIC_SCL(0);	
	CTP_Delay();
  }	 
} 	

/*****************************************************************************
 * @name       :uint8_t CTP_IIC_Read_Byte(unsigned char ack)
 * @date       :2020-05-13 
 * @function   :read a byte data by IIC bus
 * @parameters :ack:0-send nACK
									  1-send ACK
 * @retvalue   :Data to be read
******************************************************************************/	    
uint8_t CTP_IIC_Read_Byte(unsigned char ack)
{
	uint8_t i,receive=0;
 	CTP_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        CTP_IIC_SCL(0); 	    	   
		delay_us(3);
		CTP_IIC_SCL(1);  
		receive<<=1;
		if(CTP_READ_SDA)receive++;   
	}	  				 
	if (!ack)CTP_IIC_NAck();//����nACK
	else CTP_IIC_Ack(); //����ACK   
 	return receive;
}




























