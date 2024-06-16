#include "MC11.h"
#include "i2c.h"

#define MC11_ADDR (0x68)

/**-----------------------------------------------------------------------
 * @brief 配置通道分频系数
 * @param fin_div:外部输入频率分频;fref_div:内部参考时钟分频；
  例：MC11_SetFreDiv(1, 1)配置外部输入频率部分频，内部参考频率不分频;
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
int MC11_SetFreDiv(uint16_t fin_div, uint8_t fref_div)
{
	I2C_Trans_Type I2C_Result;
	if (fin_div == 1 || fin_div == 2 || fin_div == 4 || fin_div == 8 || fin_div == 16 || fin_div == 32 || fin_div == 64 || fin_div == 128 || fin_div == 256)
	{

		MC1112_I2C_Transmit(MC11_ADDR, FIN_DIV, MC_Log2(fin_div) << 4);
		I2C_Result = MC1112_I2C_Transmit(MC11_ADDR, FREF_DIV, (fref_div - 1));
		if (I2C_Result != GPIOI2C_XFER_LASTACK)
			return 0;
	}
	else
	{
		return 0;
	}

	return 1;
}

/**-----------------------------------------------------------------------
 * @brief 配置通道驱动电流
 * @param drive_i：需要配置的驱动电流；
  MC11_SetDrive_I(DRIVE_I_04mA), 配置驱动电流为0.4ma;
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
int MC11_SetDrive_I(Drive_I_Type drive_i)
{
	I2C_Trans_Type I2C_Result;

	I2C_Result = MC1112_I2C_Transmit(MC11_ADDR, DRIVE_I, drive_i);

	if (I2C_Result != GPIOI2C_XFER_LASTACK)
		return 0;
	return 1;
}

/**-----------------------------------------------------------------------
 * @brief 写任意寄存器
 * @param temp：高8位为寄存器地址，低8位为写入值；
  MC11_WriteReg(100f), 0x10寄存器写入值0x0F;
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
int MC11_WriteReg(uint16_t temp)
{
	uint8_t data;
	I2C_Trans_Type I2C_Result;

	if (((temp >> 8) >= 0x0c && (temp >> 8) <= 0x0d) || (temp >> 8 == 0x10) || ((temp >> 8) >= 0x15 && (temp >> 8) <= 0x16) ||
		(temp >> 8 == 0x18) || ((temp >> 8) >= 0x1D && (temp >> 8) <= 0x1F) || ((temp >> 8) >= 0x21 && (temp >> 8) <= 0x22) || (temp >> 8 == 0x25) || (temp >> 8) == 0x33)
	{
		MC1112_I2C_Transmit(MC11_ADDR, (REG)(temp >> 8), temp & 0xff);
		I2C_Result = MC1112_I2C_Receive(MC11_ADDR, (REG)(temp >> 8), &data);
		if (I2C_Result != GPIOI2C_XFER_LASTNACK)
			return 0;
	}
	else
	{
		return 0;
	}
	return 1;
}

/**-----------------------------------------------------------------------
 * @brief 读任意寄存器
 * @param temp：高8位为读首位寄存器地址，低8位为读取数量；*data:读到的寄存器值
  MC11_ReadReg(0404, data), 读取0x04-0x07寄存器的值;
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
int MC11_ReadReg(uint16_t temp, uint8_t *data)
{
	I2C_Trans_Type I2C_Result;
	uint8_t addr = temp >> 8;
	for (uint8_t i = 0; i < (temp & 0xff); i++)
	{
		I2C_Result = MC1112_I2C_Receive(MC11_ADDR, (REG)(addr + i), &data[i]);
		if (I2C_Result != GPIOI2C_XFER_LASTNACK)
			return 0;
	}
	return 1;
}

/**-----------------------------------------------------------------------
 * @brief 配置测量通道数
 * @param ch：需要开启通道的通道数，3表示开启双通道, 0表示关闭双通道；
  MC11_SetChannel(1), 开启第1通道;
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
int MC11_SetChannel(uint8_t ch)
{
	uint8_t CHX = 0;
	I2C_Trans_Type I2C_Result;
	if (ch > 3)
		ch = 3;
	CHX = ch << 6;

	I2C_Result = MC1112_I2C_Transmit(MC11_ADDR, CHX_EN, CHX);
	if (I2C_Result != GPIOI2C_XFER_LASTACK)
		return 0;
	return 1;
}

/**-----------------------------------------------------------------------
 * @brief 配置双通道比值报警/报警解除阈值
 * @param TH：报警阈值;TL:解除报警阈值，MC11_SetAlert(0.7, 0.5),
  DATA0/DATA1超过0.7触发STATU报警位，低于0.5解除STATU报警位
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
int MC11_SetAlert(float TH, float TL)
{
	I2C_Trans_Type I2C_Result;
	TH = TH * 0x40;
	TL = TL * 0x40;
	if (TH > 255)
		TH = 255;
	if (TL > 255)
		TL = 255;

	MC1112_I2C_Transmit(MC11_ADDR, TR_H, (uint8_t)TH);
	I2C_Result = MC1112_I2C_Transmit(MC11_ADDR, TR_L, (uint8_t)TL);
	if (I2C_Result != GPIOI2C_XFER_LASTACK)
		return 0;
	return 1;
}

/**-----------------------------------------------------------------------
 * @brief MC11启动软复位
 * @param None
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
int MC11_Reset(void)
{
	I2C_Trans_Type I2C_Result;
	I2C_Result = MC1112_I2C_Transmit(MC11_ADDR, MC_RESET, 0x7A);
	HAL_Delay(10); // 等待软复位
	if (I2C_Result != GPIOI2C_XFER_LASTACK)
		return 0;
	return 1;
}

float DA10[11] = {0.529, 0.623, 0.717, 0.812, 0.906, 1.000, 1.094, 1.187, 1.281, 1.373, 1.466};
float Coef_fix[11] = {0.946, 0.963, 0.976, 0.985, 0.993, 1.000, 1.005, 1.011, 1.015, 1.019, 1.023};
/**-----------------------------------------------------------------------
 * @brief 计算修正值
 * @param DATA：DATA1和DATA0的比值
 * @param Coef：修正值
 * @retval None
-------------------------------------------------------------------------*/
void DA1_DA0(float DATA, float *Coef)
{
	float k = 0, b = 0;
	for (int i = 0; i < 11; i++)
	{
		if (DATA < DA10[0])
		{
			DATA = DA10[0];
		}
		else if (DATA > DA10[10])
		{
			DATA = DA10[10];
		}

		if (DATA < DA10[i])
		{

			k = (Coef_fix[i] - Coef_fix[i - 1]) / (DA10[i] - DA10[i - 1]);
			b = Coef_fix[i] - k * DA10[i];
			*Coef = k * DATA + b;
			break;
		}
		else if (DATA == DA10[i])
		{
			*Coef = Coef_fix[i];
			break;
		}
	}
}

/**-----------------------------------------------------------------------
 * @brief MC11配置抗尖峰滤波
 * @param en：开关滤波器
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
int MC11_Filter_EN(uint8_t en)
{
	I2C_Trans_Type I2C_Result;
	I2C_Result = MC1112_I2C_Transmit(MC11_ADDR, GLITCH_FILTER_EN, en);
	if (I2C_Result != GPIOI2C_XFER_LASTACK)
		return 0;
	return 1;
}

/**-----------------------------------------------------------------------
 * @brief MC11初始化函数
 * @param *wait_time:测量等待时间
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
#define SCNT_VALUE 0x0F			   // 配置建立时间，0x0F*16/2.4Mhz = 100us
#define RCNT_VALUE 0x5CD0		   // 配置通道计数时间，0x5CD0/2.4Mhz = 9.9ms
#define Drive_I_VALUE DRIVE_I_04mA // 配置驱动电流0.4mA
#define Fin_Div_VALUE 32		   // 配置外部输入频率32分频
#define Fref_Div_VALUE 1		   // 配置内部参考时钟不分频
#define Channel_Num 3			   // 配置开启双通道, 0:关闭所有通道，1：开启第一通道，2：开启第二通道，3：开启双通道
#define Filter_en 1				   // 配置开启抗尖峰滤波器使能
int MC11_Init(int *wait_time)
{
	uint8_t data_msb, data_lsb;
	I2C_Trans_Type I2C_Result;
	MC1112_I2C_Transmit(MC11_ADDR, CFG, REF_SEL_In_CLK | INTB_EN_EN | INTB_MODE_ALERT | CR_1S | OS_SD_Stop_Trans); // 停止测量

	MC11_SetDrive_I(Drive_I_VALUE);				   // 配置驱动电流为0.4ma
	MC11_SetFreDiv(Fin_Div_VALUE, Fref_Div_VALUE); // 配置输入频率32分频，内部参考采样时钟不分频
	MC11_SetChannel(Channel_Num);				   // 配置双通道开启
	MC11_Filter_EN(Filter_en);					   // 开启滤波器

	MC1112_I2C_Transmit(MC11_ADDR, RCNT_MSB, RCNT_VALUE >> 8); // 配置通道计数时间
	MC1112_I2C_Transmit(MC11_ADDR, RCNT_LSB, RCNT_VALUE & 0XFF);
	MC1112_I2C_Transmit(MC11_ADDR, SCNT, SCNT_VALUE); // 配置建立时间

	// 清除statu标志
	MC1112_I2C_Receive(MC11_ADDR, DATA_CH0_MSB, &data_msb);
	MC1112_I2C_Receive(MC11_ADDR, DATA_CH0_LSB, &data_lsb);
	MC1112_I2C_Receive(MC11_ADDR, DATA_CH1_MSB, &data_msb);
	I2C_Result = MC1112_I2C_Receive(MC11_ADDR, DATA_CH1_LSB, &data_lsb);
	if (I2C_Result != GPIOI2C_XFER_LASTNACK)
		return 0;
	*wait_time = (int)((((float)SCNT_VALUE * 16 + (float)RCNT_VALUE + 4) / (CLKIN * 1000)) * 2 + 1); // 计算双通道等待时间，1ms裕度，单位ms，T=2*（Tset+Tcnt）=2*（SCNT*16+RCNT+4）/Fref

	return 1;
}

/**-----------------------------------------------------------------------
 * @brief 开启测量双通道频率&电容
 * @param *F0:通道0频率值；*F1：通道1频率值；*C：电容值
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
int MC11_Measure(float *F0, float *F1, float *C, int wait_time)
{
	uint8_t sta, FIN0_DIV;
	I2C_Trans_Type I2C_Result;
	uint32_t timeout;
	float Coef;

	uint8_t data_msb, data_lsb, get_ch;
	timeout = 200;
	MC1112_I2C_Receive(MC11_ADDR, CHX_EN, &get_ch);

	MC1112_I2C_Transmit(MC11_ADDR, CFG, REF_SEL_In_CLK | INTB_EN_DIS | INTB_MODE_ALERT | CR_1S | OS_SD_Single_Trans); // 开启测量
	HAL_Delay(wait_time);
__measure:
	MC1112_I2C_Receive(MC11_ADDR, STATUS, &sta);
	timeout--;
	if (!((sta & 0x30) == (get_ch >> 6) << 4) && timeout > 0)
		goto __measure; // 判断statu寄存器转换完成标志位

	MC1112_I2C_Receive(MC11_ADDR, DATA_CH0_MSB, &data_msb);
	MC1112_I2C_Receive(MC11_ADDR, DATA_CH0_LSB, &data_lsb);
	I2C_Result = MC1112_I2C_Receive(MC11_ADDR, FIN_DIV, &FIN0_DIV);
	if (I2C_Result != GPIOI2C_XFER_LASTNACK)
		return 0;
	*F0 = (float)((uint16_t)(data_msb << 8 | data_lsb) * MC_Pow2(FIN0_DIV >> 4) * CLKIN / RCNT_VALUE); // 通道0频率计算公式, F=DATA0*Fin_Div*CLKIN/RCNT;DATA为通道0计数值，Fin_Div为输入频率分频，CLKIN为参考采样时钟，REV_RCNT为通道计数时间

	MC1112_I2C_Receive(MC11_ADDR, DATA_CH1_MSB, &data_msb);
	I2C_Result = MC1112_I2C_Receive(MC11_ADDR, DATA_CH1_LSB, &data_lsb);
	if (I2C_Result != GPIOI2C_XFER_LASTNACK)
		return 0;
	*F1 = (float)((uint16_t)(data_msb << 8 | data_lsb) * MC_Pow2(FIN0_DIV >> 4) * CLKIN / RCNT_VALUE);

	DA1_DA0((*F1 / *F0), &Coef);	   // 根据双通道频率比值计算修正系数Coef
	*C = *F1 / *F0 * Cref * Coef - Cp; // Csensor = F1/F0*Cref*Coef-Cp;根据参比通道电容，计算实际测试通道电容，Cp为测量通道并联电容（根据实际电路修改），Cref为参比通道电容（根据实际电路修改）

	return 1;
}

/**-----------------------------------------------------------------------
 * @brief MC1112数据发送函数
 * @param DeviceAddr：地址 reg：寄存器地址 Data：发送数据
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
I2C_Trans_Type MC1112_I2C_Transmit(uint8_t DeviceAddr, REG reg, uint8_t Data)
{
	if (HAL_I2C_Mem_Write(&hi2c1, DeviceAddr << 1, reg, 1, &Data, 1, HAL_MAX_DELAY) == HAL_OK)
	{
		return GPIOI2C_XFER_LASTACK;
	}
	else
	{
		return GPIOI2C_XFER_ABORTNACK;
	}
}

/**-----------------------------------------------------------------------
 * @brief MC1112数据接收函数
 * @param DeviceAddr：地址 reg：寄存器地址 *pData：接收数据
 * @retval 是否传输成功
-------------------------------------------------------------------------*/
I2C_Trans_Type MC1112_I2C_Receive(uint8_t DeviceAddr, REG reg, uint8_t *pData)
{
	if (HAL_I2C_Mem_Read(&hi2c1, DeviceAddr << 1, reg, 1, pData, 1, HAL_MAX_DELAY) == HAL_OK)
	{
		return GPIOI2C_XFER_LASTNACK;
	}
	else
	{
		return GPIOI2C_XFER_ABORTNACK;
	}
}

/**-----------------------------------------------------------------------
 * @brief log2查表函数，范围1-256
 * @param DATA：2的N次幂
 * @retval 真数
-------------------------------------------------------------------------*/
uint8_t MC_Log2(uint32_t DATA)
{
	uint8_t ret = 0;
	switch (DATA)
	{
	case 1:
		ret = 0;
		break;
	case 2:
		ret = 1;
		break;
	case 4:
		ret = 2;
		break;
	case 8:
		ret = 3;
		break;
	case 16:
		ret = 4;
		break;
	case 32:
		ret = 5;
		break;
	case 64:
		ret = 6;
		break;
	case 128:
		ret = 7;
		break;
	case 256:
		ret = 8;
		break;
	default:
		ret = 3;
		break;
	}
	return ret;
}
/**-----------------------------------------------------------------------
 * @brief POW2查表函数
 * @param N：真数
 * @retval 2的N次幂
-------------------------------------------------------------------------*/
int MC_Pow2(uint8_t N)
{
	int ret = 0;
	switch (N)
	{
	case 0:
		ret = 1;
		break;
	case 1:
		ret = 2;
		break;
	case 2:
		ret = 4;
		break;
	case 3:
		ret = 8;
		break;
	case 4:
		ret = 16;
		break;
	case 5:
		ret = 32;
		break;
	case 6:
		ret = 64;
		break;
	case 7:
		ret = 128;
		break;
	case 8:
		ret = 256;
		break;
	default:
		ret = 8;
		break;
	}
	return ret;
}
