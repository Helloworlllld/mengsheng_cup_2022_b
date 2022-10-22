#include "TCS3472.h"

void TCS_SDA_IN(void) 
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
}

void TCS_SDA_OUT(void) 
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
}

void TCS34725_I2C_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

    /*Configure GPIO pins : PB8 PB9 */
    GPIO_InitStruct.Pin = SCL_PIN|SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SCL_PORT, &GPIO_InitStruct);
}

/*********************************************/
void TCS3472_Delay(uint32_t i)
{
    while(i>0) i--;
};

/*********************************************/
void TCS34725_I2C_Start()
{
	TCS_SDA_OUT();
	TCS_SDA_H;
	TCS_SCL_H;
	TCS3472_Delay(40);//delay_us(4);
	TCS_SDA_L;
	TCS3472_Delay(40);//delay_us(4);
	TCS_SCL_L;
}
/*********************************************/
void TCS34725_I2C_Stop()
{
	TCS_SDA_OUT();
	TCS_SCL_L;
	TCS_SDA_L;
	TCS3472_Delay(40);//delay_us(4);
	TCS_SCL_H;
	TCS_SDA_H;
	TCS3472_Delay(40);//delay_us(4);							   	
}
/*********************************************/
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t TCS34725_I2C_Wait_ACK()
{
	uint32_t t=0;
	
	TCS_SDA_IN();//SDA设置为输入  
	TCS_SDA_H; 
	TCS3472_Delay(10);//delay_us(1);
	TCS_SCL_H; 
	TCS3472_Delay(10);//delay_us(1);
	while(TCS_SDA_READ)
	{
		t++;
		if(t > 250)
		{
			TCS34725_I2C_Stop();
			return 1;
		}
	}
	TCS_SCL_L;
	return 0;	
}
/*********************************************/
//产生ACK应答
void TCS34725_I2C_ACK()
{
	TCS_SCL_L;
	TCS_SDA_OUT();//sda线输出
	TCS_SDA_L;
	TCS3472_Delay(20);//delay_us(2);
	TCS_SCL_H;
	TCS3472_Delay(20);//delay_us(2);
	TCS_SCL_L;
}
/*********************************************/
//不产生ACK应答		    
void TCS34725_I2C_NACK()
{
	TCS_SCL_L;
	TCS_SDA_OUT();//sda线输出
	TCS_SDA_H;
	TCS3472_Delay(20);//delay_us(2);
	TCS_SCL_H;
	TCS3472_Delay(20);//delay_us(2);
	TCS_SCL_L;
}
/*********************************************/
//I2C发送一个字节		  
void TCS34725_I2C_Send_Byte(uint8_t byte)
{
	uint8_t i;
	
	TCS_SDA_OUT();//sda线输出
	TCS_SCL_L;//拉低时钟开始数据传输
	for(i = 0; i < 8; i++)
	{
		if(((byte&0x80)>>7)==1)TCS_SDA_H;
		else
			TCS_SDA_L;
		byte <<= 1;
		
		TCS3472_Delay(20);//delay_us(2);
		TCS_SCL_H;
		TCS3472_Delay(20);//delay_us(2);
		TCS_SCL_L;
		TCS3472_Delay(20);//delay_us(2);
	} 
}
/*********************************************/
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t TCS34725_I2C_Read_Byte(uint8_t ack)
{
	uint8_t i,receive = 0;
	
	TCS_SDA_IN();
	for(i = 0; i < 8; i++)
	{
		TCS_SCL_L;
		TCS3472_Delay(20);//delay_us(2);
		TCS_SCL_H;
		receive <<= 1;
		if(TCS_SDA_READ) receive++;
		TCS3472_Delay(10);//delay_us(1);
	}
	if (!ack) TCS34725_I2C_NACK();//发送nACK
	else TCS34725_I2C_ACK(); //发送ACK 
	
	return receive;
}
/*********************************************/
/*******************************************************************************
 * @brief Writes data to a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes to write.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
*******************************************************************************/
void TCS34725_I2C_Write(uint8_t slaveAddress, uint8_t* dataBuffer,uint8_t bytesNumber, uint8_t stopBit)
{
	uint8_t i = 0;
	
	TCS34725_I2C_Start();
	TCS34725_I2C_Send_Byte((slaveAddress << 1) | 0x00);	   //发送从机地址写命令
	TCS34725_I2C_Wait_ACK();
	for(i = 0; i < bytesNumber; i++)
	{
		TCS34725_I2C_Send_Byte(*(dataBuffer + i));
		TCS34725_I2C_Wait_ACK();
	}
	if(stopBit == 1) TCS34725_I2C_Stop();
}
/*******************************************************************************
 * @brief Reads data from a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes to read.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
*******************************************************************************/
void TCS34725_I2C_Read(uint8_t slaveAddress, uint8_t* dataBuffer, uint8_t bytesNumber, uint8_t stopBit)
{
	uint8_t i = 0;
	
	TCS34725_I2C_Start();
	TCS34725_I2C_Send_Byte((slaveAddress << 1) | 0x01);	   //发送从机地址读命令
	TCS34725_I2C_Wait_ACK();
	for(i = 0; i < bytesNumber; i++)
	{
		if(i == bytesNumber - 1)
		{
			*(dataBuffer + i) = TCS34725_I2C_Read_Byte(0);//读取的最后一个字节发送NACK
		}
		else
		{
			*(dataBuffer + i) = TCS34725_I2C_Read_Byte(1);
		}
	}
	if(stopBit == 1) TCS34725_I2C_Stop();
}
/*******************************************************************************
 * @brief Writes data into TCS34725 registers, starting from the selected
 *        register address pointer.
 *
 * @param subAddr - The selected register address pointer.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes that will be sent.
 *
 * @return None.
*******************************************************************************/
void TCS34725_Write(uint8_t subAddr, uint8_t* dataBuffer, uint8_t bytesNumber)
{
    uint8_t sendBuffer[10] = {0};
    uint8_t byte = 0;
    
    sendBuffer[0] = subAddr | TCS34725_COMMAND_BIT;
    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }
	TCS34725_I2C_Write(TCS34725_ADDRESS, sendBuffer, bytesNumber + 1, 1);
}
/*******************************************************************************
 * @brief Reads data from TCS34725 registers, starting from the selected
 *        register address pointer.
 *
 * @param subAddr - The selected register address pointer.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes that will be read.
 *
 * @return None.
*******************************************************************************/
void TCS34725_Read(uint8_t subAddr, uint8_t* dataBuffer, uint8_t bytesNumber)
{
	subAddr |= TCS34725_COMMAND_BIT;
	
	TCS34725_I2C_Write(TCS34725_ADDRESS, (uint8_t*)&subAddr, 1, 0);
	TCS34725_I2C_Read(TCS34725_ADDRESS, dataBuffer, bytesNumber, 1);
}
/*******************************************************************************
 * @brief TCS34725设置积分时间
 *
 * @return None
*******************************************************************************/
void TCS34725_SetIntegrationTime(uint8_t time)
{
	TCS34725_Write(TCS34725_ATIME, &time, 1);
}
/*******************************************************************************
 * @brief TCS34725设置增益
 *
 * @return None
*******************************************************************************/
void TCS34725_SetGain(uint8_t gain)
{
	TCS34725_Write(TCS34725_CONTROL, &gain, 1);
}
/*******************************************************************************
* @brief TCS34725设置中断上限制
 *
 * @return None
*******************************************************************************/
void TCS34725_SetAIHT(uint16_t set)
{
	uint8_t AIHTH=0;
	uint8_t AIHTL=0;
	if(set>255)
	{
		AIHTL=(uint8_t)set;
		AIHTH=(uint8_t)(set>>8);
	}	
	else	AIHTL=set;
	uint8_t PERS=0X01;
	TCS34725_Write(TCS34725_PERS, &PERS, 0);
	TCS34725_Write(TCS34725_AIHTL, &AIHTL, 1);
	TCS34725_Write(TCS34725_AIHTH, &AIHTH, 1);
}

/*******************************************************************************
* @brief TCS34725设置中断下限
 *
 * @return None
*******************************************************************************/
void TCS34725_SetAILT(uint16_t set)
{
	uint8_t AILTH=0,AILTL=0;
	if(set>255)
	{
		AILTL=(uint8_t)set;
		AILTH=(uint8_t)(set>>8);
	}	
	else	AILTL=set;
	uint8_t PERS=0X01;
	TCS34725_Write(TCS34725_PERS, &PERS, 0);
	TCS34725_Write(TCS34725_AILTL, &AILTL, 1);
	TCS34725_Write(TCS34725_AILTH, &AILTH, 1);
}
/*******************************************************************************
 * @brief TCS34725使能
 *
 * @return None
*******************************************************************************/
void TCS34725_Enable(void)
{
	uint8_t cmd = TCS34725_ENABLE_PON;
	
	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
	cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN|TCS34725_ENABLE_AIEN ;
	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
	//delay_s(600000);//delay_ms(3);//延时应该放在设置AEN之后
}
/*******************************************************************************
 * @brief TCS34725失能
 *
 * @return None
*******************************************************************************/
void TCS34725_Disable(void)
{
	uint8_t cmd = 0;
	
	TCS34725_Read(TCS34725_ENABLE, &cmd, 1);
	cmd = cmd & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
}
/*******************************************************************************
 * @brief TCS34725初始化
 *
 * @return ID - ID寄存器中的值
*******************************************************************************/
uint8_t TCS34725_Init(void)
{
	uint8_t id=0;
	
	TCS34725_I2C_Init(); 
	TCS34725_Read(TCS34725_ID, &id, 1);  //TCS34725 的 ID 是 0x44 可以根据这个来判断是否成功连接,0x4D是TCS34727;
	if(id==0x4D | id==0x44)
		{
			TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_101MS);
			TCS34725_SetGain(TCS34725_GAIN_1X);
			TCS34725_Enable();
			TCS34725_SetAIHT(350);
			TCS34725_SetAILT(10);
			return 1;
		}
	return 0;
}
/*******************************************************************************
 * @brief TCS34725获取单个通道数据
 *
 * @return data - 该通道的转换值
*******************************************************************************/
uint16_t TCS34725_GetChannelData(uint8_t reg)
{
	uint8_t tmp[2] = {0,0};
	uint16_t data;
	
	TCS34725_Read(reg, tmp, 2);
	data = (tmp[1] << 8) | tmp[0];
	
	return data;
}
/*******************************************************************************
 * @brief TCS34725获取各个通道数据
 *
 * @return 1 - 转换完成，数据可用
 *   	   0 - 转换未完成，数据不可用
*******************************************************************************/
uint8_t TCS34725_GetRawData(COLOR_RGBC *rgbc)
{
	uint8_t status = TCS34725_STATUS_AVALID;
	
	TCS34725_Read(TCS34725_STATUS, &status, 1);
	
	if(status & TCS34725_STATUS_AVALID)
	{
		rgbc->c = TCS34725_GetChannelData(TCS34725_CDATAL);	
		rgbc->r = TCS34725_GetChannelData(TCS34725_RDATAL);	
		rgbc->g = TCS34725_GetChannelData(TCS34725_GDATAL);	
		rgbc->b = TCS34725_GetChannelData(TCS34725_BDATAL);
		return 1;
	}
	return 0;
}
/******************************************************************************/
//RGB转HSL
void RGBtoHSL(COLOR_RGBC *Rgb, COLOR_HSL *Hsl)
{
	uint8_t maxVal,minVal,difVal;
	uint8_t r = Rgb->r*100/Rgb->c;   //[0-100]
	uint8_t g = Rgb->g*100/Rgb->c;
	uint8_t b = Rgb->b*100/Rgb->c;
	
	maxVal = max3v(r,g,b);
	minVal = min3v(r,g,b);
	difVal = maxVal-minVal;
	
	//计算亮度
	Hsl->l = (maxVal+minVal)/2;   //[0-100]
	
	if(maxVal == minVal)//若r=g=b,灰度
	{
		Hsl->h = 0; 
		Hsl->s = 0;
	}
	else
	{
		//计算色调
		if(maxVal==r)
		{
			if(g>=b)
				Hsl->h = 60*(g-b)/difVal;
			else
				Hsl->h = 60*(g-b)/difVal+360;
		}
		else
			{
				if(maxVal==g)Hsl->h = 60*(b-r)/difVal+120;
				else
					if(maxVal==b)Hsl->h = 60*(r-g)/difVal+240;
			}
		
		//计算饱和度
		if(Hsl->l<=50)Hsl->s=difVal*100/(maxVal+minVal);  //[0-100]
		else
			Hsl->s=difVal*100/(200-(maxVal+minVal));
	}
}
/******************************************************************************/
/******************************************************************************
function:   Convert raw RGB values to RGB565 format
parameter	:
     rgb    : RGBC Numerical value
******************************************************************************/
uint16_t RGB24216(uint8_t tempr,uint8_t tempg,uint8_t tempb)
{
    float i=1;
    //Limit data range
    if(tempr >= tempg && tempr >= tempb){ 
        i = tempr / 255 + 1;
    }
    else if(tempg >= tempr && tempg >= tempb){ 
        i = tempg / 255 + 1;
    }
    else if(tempb >=  tempg && tempb >= tempr){ 
        i = tempb / 255 + 1;  
    }
    if(i!=0){
        tempr = (tempr) / i;
        tempg = (tempg) / i;
        tempb = (tempb) / i;
    }
    if(tempr > 30)
        tempr = tempr - 30;
    if(tempg > 30)
        tempg = tempg - 30;
    if(tempb > 30)
        tempb = tempb - 30;
    tempr = tempr * 255 / 225;
    tempg = tempg * 255 / 225;
    tempb = tempb * 255 / 225;
    
    if(tempr>255)
           tempr = 255; 
    if(tempg>255)
           tempg = 255; 
    if(tempb>255)
           tempb = 255; 
    return (tempr << 11) | (tempg << 5) | (tempb);
}


