#ifndef __MC1112_H
#define __MC1112_H

#include <stdint.h>

#define CLKIN 2.4 //MC11/12 内部参考时钟,单位Mhz

#define Cref      100//通道1接22pf参比电容，如此处为其他值，请修改
#define Cp      10//通道0接10pf并联电容，如此处为其他值，请修改
					

typedef enum 
{//FIN_DIV 通道震荡信号分频系数
 	FIN_DIV_0        	= 0x00,   //不分频
  FIN_DIV_2        	= 0x10,   //2 分频
  FIN_DIV_4        	= 0x20,   //4 分频
 	FIN_DIV_8        	= 0x30,   //8 分频
 	FIN_DIV_16       	= 0x40,   //16 分频
  FIN_DIV_32       	= 0x50,   //32 分频
  FIN_DIV_64       	= 0x60,   //64 分频
 	FIN_DIV_128      	= 0x70,   //128 分频	
 	FIN_DIV_256      	= 0x80,   //256 分频		
} DIV_Type;

//CFG寄存器说明：时钟选择/电容比较输出结果
#define   REF_SEL_In_CLK     0x00<<7  //选择内部时钟
#define   REF_SEL_Ex_CLK     0x01<<7  //选择外部时钟

#define   INTB_EN_DIS     		   0x00<<6  //INTB 不输出标志位
#define   INTB_EN_EN     	   0x01<<6  //INTB 输出报警标志位或者转换完成标志位
#define   INTB_MODE_ALERT     	       0x00<<5  //INTB 输出报警标志位
#define   INTB_MODE_TRANS     	       0x01<<5  //IINTB 输出转换完成标志位

#define   CR_60S             0x00  //连续转换模式的时间间隔 60s 转换一次
#define   CR_30S             0x04  //连续转换模式的时间间隔 30s 转换一次
#define   CR_10S             0x08  //连续转换模式的时间间隔 10s 转换一次
#define   CR_5S              0x0c  //连续转换模式的时间间隔 5s 转换一次
#define   CR_2S              0x10  //连续转换模式的时间间隔 2s 转换一次
#define   CR_1S              0x14  //连续转换模式的时间间隔 1s 转换一次
#define   CR_05S             0x18  //连续转换模式的时间间隔 0.5s 转换一次
#define   CR_025S            0x1c  //连续转换模式的时间间隔 0.25s 转换一次

//设置通道转换模式
#define   OS_SD_Continuous_Trans      0x00  //设置通道转换模式 连续转换
#define   OS_SD_Stop_Trans            0x01  //设置通道转换模式 停止转换
#define   OS_SD_Single_Trans          0x03  //设置通道转换模式 单次转化
#define   CH_DISABLE                  0x00  //通道 0、1 关闭
#define   CH0_ENABLE                  0x40  //通道 0 开启
#define   CH1_ENABLE                  0x80  //通道 1 开启
#define   CH_ENABLE                   0xC0  //通道 0、1 开启

//DRIVE_I 说明,地址 0x25 ,设置通道 0\1 的驱动电流
typedef enum 
{
  DRIVE_I_02mA  =  0x00,   //0.2mA
  DRIVE_I_04mA  =  0x10,   //0.4mA
  DRIVE_I_08mA  =  0x20,   //0.8mA
  DRIVE_I_16mA  =  0x30,   //1.6mA
  DRIVE_I_24mA  =  0x40,   //2.4mA
  DRIVE_I_32mA  =  0x50,   //3.2mA

}Drive_I_Type;


#define   VDD_SEL                   0xFE
//VDD 电压适配选择:1：适用于 2.0V<VDD<2.5V 的情况--低电压
#define   VDD_SEL_L                      0x01     
//VDD 电压适配选择:0：适用于 2.5V<VDD<5.5V 的情况--高电压
#define   VDD_SEL_H                      0x00   

typedef enum 
{
  DATA_CH0_MSB					= 0x04,   // 通道 0 转换数据
  DATA_CH0_LSB          = 0x05,   // 通道 0 转换数据
  DATA_CH1_MSB          = 0x06,   // 通道 1 转换数据
  DATA_CH1_LSB          = 0x07,   // 通道 1 转换数据
	
  RCNT_MSB              = 0x0C,  //默认值0x12：计数时间高字节
  RCNT_LSB	            = 0x0D,  //默认值0xC0：计数时间低字节
	
  SCNT               	  = 0x10,	  //默认值0x20:建立时间
	
  FIN_DIV               = 0x15,	  //默认值0x30:振荡信号分频
  FREF_DIV              = 0x16,   //默认值0x00:参考时钟分频

  STATUS                = 0x18,   //默认值0x00：状态位，只读

  TR_H             	    = 0x1D,	  //默认值0x40:单通道报警触发门限
  TR_L                  = 0x1E,	  //默认值0x3A:单通道报警触发门限
 
  CFG             	    = 0x1F,   //默认值0x54:通道转换与 INTB 功能配置
  CHX_EN                = 0x21,   //默认值0xC0:通道选择配置
  MC_RESET              = 0x22, 	//复位功能,只写
  DRIVE_I					      = 0x25,   //默认值0x00：通道 1 驱动电流
  GLITCH_FILTER_EN			= 0x33,   //默认值0x01：抗尖峰滤波器
  CHIP_ID_MSB					  = 0x7E,    //默认值0x01：芯片 ID，只读
  CHIP_ID_LSB      			= 0x7F,    //默认值0x20：芯片 ID，只读
} REG;
/** @defgroup  
  * I2C transaction result
  */ 
typedef enum 
{
 GPIOI2C_XFER_LASTNACK     =    ((uint8_t)0x00),   	 	/* !< No error              */
 GPIOI2C_XFER_ADDRNACK   	=		((uint8_t)0x01) ,   	  /* !< No Device             */
 GPIOI2C_XFER_ABORTNACK  	=		((uint8_t)0x02) ,   	  /* !< NACK before last byte */
 GPIOI2C_XFER_LASTACK    =			((uint8_t)0x04) ,    	/* !< ACK last byte         */
 GPIOI2C_XFER_BUSERR   =  			((uint8_t)0x10) ,    	/* !< Bus error             */
}I2C_Trans_Type;

I2C_Trans_Type MC1112_I2C_Transmit(uint8_t DeviceAddr, REG reg, uint8_t Data);
I2C_Trans_Type MC1112_I2C_Receive(uint8_t DeviceAddr, REG reg,uint8_t *pData);
int MC11_Measure(float *F0, float *F1 ,float *C,int wait_time);
int MC11_Filter_EN(uint8_t en);
int MC11_Init(int *wait_time);
void DA1_DA0(float DATA,float *Coef);
int MC11_Reset(void);
int MC11_SetAlert(float TH , float TL);
int MC11_SetChannel(uint8_t ch);
int MC11_ReadReg(uint16_t temp,uint8_t *data);
int MC11_WriteReg(uint16_t temp);
int MC11_SetDrive_I(Drive_I_Type drive_i);
int MC11_SetFreDiv(uint16_t fin_div,uint8_t fref_div);
uint8_t MC_Log2(uint32_t DATA);
int MC_Pow2(uint8_t N);
#endif



