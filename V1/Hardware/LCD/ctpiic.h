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
#ifndef __CTPIIC_H
#define __CTPIIC_H
//#include "sys.h"	 
#include <stdint.h>
  	   		   
//IIC IO��������
#define CTP_SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}
#define CTP_SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;}

//IO��������	 
//#define CTP_IIC_SCL    PBout(0) 			//SCL     
//#define CTP_IIC_SDA    PFout(11) 			//SDA	 
//#define CTP_READ_SDA   PFin(11)  			//����SDA 

//IIC���в�������
void CTP_IIC_Init(void);                	//��ʼ��IIC��IO��				 
void CTP_IIC_Start(void);				//����IIC��ʼ�ź�
void CTP_IIC_Stop(void);	  				//����IICֹͣ�ź�
void CTP_IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t CTP_IIC_Read_Byte(unsigned char ack);	//IIC��ȡһ���ֽ�
uint8_t CTP_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void CTP_IIC_Ack(void);					//IIC����ACK�ź�
void CTP_IIC_NAck(void);					//IIC������ACK�ź�

#endif







