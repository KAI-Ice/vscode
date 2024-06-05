//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//测试硬件：单片机STM32F407ZGT6,正点原子Explorer STM32F4开发板,主频168MHZ，晶振12MHZ
//QDtech-TFT液晶驱动 for STM32 IO模拟
//xiao冯@ShenZhen QDtech co.,LTD
//公司网站:www.qdtft.com
//淘宝网站：http://qdtech.taobao.com
//wiki技术网站：http://www.lcdwiki.com
//我司提供技术支持，任何技术问题欢迎随时交流学习
//固话(传真) :+86 0755-23594567 
//手机:15989313508（冯工） 
//邮箱:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//技术支持QQ:3002773612  3002778157
//技术交流QQ群:324828016
//创建日期:2018/08/09
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 深圳市全动电子技术有限公司 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================电源接线================================================//
//     LCD模块                STM32单片机
//      VCC          接        DC5V/3.3V      //电源
//      GND          接          GND          //电源地
//=======================================液晶屏数据线接线==========================================//
//本模块默认数据总线类型为SPI总线
//     LCD模块                STM32单片机    
//    SDI(MOSI)      接          PB5          //液晶屏SPI总线数据写信号
//    SDO(MISO)      接          PB4          //液晶屏SPI总线数据读信号，如果不需要读，可以不接线
//=======================================液晶屏控制线接线==========================================//
//     LCD模块 					      STM32单片机 
//       SCK         接          PB3          //液晶屏SPI总线时钟信号
//       LED         接          PE9         //液晶屏背光控制信号，如果不需要控制，接5V或3.3V
//      DC/RS        接          PE10         //液晶屏数据/命令控制信号
//       RST         接          PE8         //液晶屏复位控制信号
//       CS          接          PE11        //液晶屏片选控制信号
//=========================================触摸屏触接线=========================================//
//如果模块不带触摸功能或者带有触摸功能，但是不需要触摸功能，则不需要进行触摸屏接线
//	   LCD模块                STM32单片机 
//     CTP_INT       接          PE2          //电容触摸屏中断信号
//     CTP_SDA       接          PB10         //电容触摸屏IIC数据信号
//     CTP_RST       接          PG15          //电容触摸屏复位信号
//     CTP_SCL       接          PB11          //电容触摸屏IIC时钟信号
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
#include "ft6336.h"
#include "touch.h"
#include "ctpiic.h"
#include "delay.h" 
#include "string.h" 
#include "lcd.h"

extern uint8_t touch_flag;

/*****************************************************************************
 * @name       :uint8_t FT5426_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
 * @date       :2020-05-13 
 * @function   :Write data to ft5426 once
 * @parameters :reg:Start register address for written
								buf:the buffer of data written
								len:Length of data written
 * @retvalue   :0-Write succeeded 
								1-Write failed
******************************************************************************/ 
uint8_t FT6336_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i;
	uint8_t ret=0;
	CTP_IIC_Start();	 
	CTP_IIC_Send_Byte(FT_CMD_WR);	//发送写命令 	 
	CTP_IIC_Wait_Ack(); 	 										  		   
	CTP_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CTP_IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
		CTP_IIC_Send_Byte(buf[i]);  	//发数据
		ret=CTP_IIC_Wait_Ack();
		if(ret)
		{
			break;
		}
	}
  CTP_IIC_Stop();					//产生一个停止条件	    
	return ret; 
}

/*****************************************************************************
 * @name       :void FT5426_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
 * @date       :2020-05-13 
 * @function   :Read data to ft5426 once
 * @parameters :reg:Start register address for read
								buf:the buffer of data read
								len:Length of data read
 * @retvalue   :none
******************************************************************************/			  
void FT6336_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i; 
 	CTP_IIC_Start();	
 	CTP_IIC_Send_Byte(FT_CMD_WR);   	//发送写命令 	 
	CTP_IIC_Wait_Ack(); 	 										  		   
 	CTP_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CTP_IIC_Wait_Ack();  
 	CTP_IIC_Start();  	 	   
	CTP_IIC_Send_Byte(FT_CMD_RD);   	//发送读命令		   
	CTP_IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CTP_IIC_Read_Byte(i==(len-1)?0:1); //发数据	  
	} 
  CTP_IIC_Stop();//产生一个停止条件     
} 

/*****************************************************************************
 * @name       :uint8_t FT5426_Init(void)
 * @date       :2020-05-13 
 * @function   :Initialize the ft5426 touch screen
 * @parameters :none
 * @retvalue   :0-Initialization successful
								1-initialization failed
******************************************************************************/		
uint8_t FT6336_Init(void)
{
	uint8_t temp[2]; 	
//	GPIO_InitTypeDef  GPIO_InitStructure;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);//使能GPIOB和GPIOC时钟
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //PB1
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉 
//	GPIO_Init(GPIOB, &GPIO_InitStructure);				  

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //PC5
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 	 //输出	  
//	GPIO_Init(GPIOC, &GPIO_InitStructure);					

	CTP_IIC_Init();      	//初始化电容屏的I2C总线  
	FT_RST(0);				//复位
	delay_ms(10);
 	FT_RST(1);				//释放复位		    
	delay_ms(500);  	
//	temp[0]=0;
//	FT6336_WR_Reg(FT_DEVIDE_MODE,temp,1);	//进入正常操作模式 
//	FT6336_WR_Reg(FT_ID_G_MODE,temp,1);		//查询模式 
	//temp[0]=40;								//触摸有效值，22，越小越灵敏	
	//FT6336_WR_Reg(FT_ID_G_THGROUP,temp,1);	//设置触摸有效值
	FT6336_RD_Reg(FT_ID_G_FOCALTECH_ID,&temp[0],1);
	if(temp[0]!=0x11)
	{
		return 1;
	}
	FT6336_RD_Reg(FT_ID_G_CIPHER_MID,&temp[0],2);
	if(temp[0]!=0x26)
	{
		return 1;
	}
	if((temp[1]!=0x00)&&(temp[1]!=0x01)&&(temp[1]!=0x02))
	{
		return 1;
	}
	FT6336_RD_Reg(FT_ID_G_CIPHER_HIGH,&temp[0],1);
	if(temp[0]!=0x64)
	{
		return 1;
	}
//	temp[0]=12;								//激活周期，不能小于12，最大14
//	FT6336_WR_Reg(FT_ID_G_PERIODACTIVE,temp,1); 
	//读取版本号，参考值：0x3003
//	FT6336_RD_Reg(FT_ID_G_LIB_VERSION,&temp[0],2);  
//	if(temp[0]==0X10&&temp[1]==0X01)//版本:0X3003
//	{ 
//		printf("CTP ID:%x\r\n",((uint16_t)temp[0]<<8)+temp[1]);
//		return 0;
//	} 
	return 0;
}

const uint16_t FT6336_TPX_TBL[2]={FT_TP1_REG,FT_TP2_REG};

/*****************************************************************************
 * @name       :uint8_t FT5426_Scan(void)
 * @date       :2020-05-13 
 * @function   :Scan touch screen (query mode)
 * @parameters :none
 * @retvalue   :Current touch screen status
								0-No touch
								1-With touch
******************************************************************************/	
uint8_t FT6336_Scan(void)
{
	uint8_t buf[4];
	uint8_t i=0;
	uint8_t res=0;
	uint8_t temp;
	uint8_t mode;
	static uint8_t t=0;//控制查询间隔,从而降低CPU占用率   
	t++;
	if((t%10)==0||t<10)//空闲时,每进入10次CTP_Scan函数才检测1次,从而节省CPU使用率
	{
		FT6336_RD_Reg(FT_REG_NUM_FINGER,&mode,1);//读取触摸点的状态  
		if(mode&&(mode<3))
		{
			temp=0XFF<<mode;//将点的个数转换为1的位数,匹配tp_dev.sta定义 
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			for(i=0;i<CTP_MAX_TOUCH;i++)
			{
				FT6336_RD_Reg(FT6336_TPX_TBL[i],buf,4);	//读取XY坐标值 
				if(tp_dev.sta&(1<<i))	//触摸有效?
				{
					switch(lcddev.dir)
					{
						case 0:
							tp_dev.x[i]=((uint16_t)(buf[0]&0X0F)<<8)+buf[1];
							tp_dev.y[i]=((uint16_t)(buf[2]&0X0F)<<8)+buf[3];						
							break;
						case 1:
							tp_dev.y[i]=lcddev.height-(((uint16_t)(buf[0]&0X0F)<<8)+buf[1]);
							tp_dev.x[i]=((uint16_t)(buf[2]&0X0F)<<8)+buf[3];						
							break;
						case 2:
							tp_dev.x[i]=lcddev.width-(((uint16_t)(buf[0]&0X0F)<<8)+buf[1]);
							tp_dev.y[i]=lcddev.height-(((uint16_t)(buf[2]&0X0F)<<8)+buf[3]);								
							break;
						case 3:
							tp_dev.y[i]=((uint16_t)(buf[0]&0X0F)<<8)+buf[1];
							tp_dev.x[i]=lcddev.width-(((uint16_t)(buf[2]&0X0F)<<8)+buf[3]);	
							break;
					} 
					//if((buf[0]&0XF0)!=0X80)tp_dev.x[i]=tp_dev.y[i]=0;//必须是contact事件，才认为有效
					//printf("x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]);
				}			
			} 
			res=1;
			if(tp_dev.x[0]==0 && tp_dev.y[0]==0)mode=0;	//读到的数据都是0,则忽略此次数据
			t=0;		//触发一次,则会最少连续监测10次,从而提高命中率
		}
	}
	if(mode==0)//无触摸点按下
	{ 
		if(tp_dev.sta&TP_PRES_DOWN)	//之前是被按下的
		{
			tp_dev.sta&=~(1<<7);	//标记按键松开
		}else						//之前就没有被按下
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE0;	//清除点有效标记	
		}	 
	} 
	if(t>240)t=10;//重新从10开始计数
	return res;
}
 




