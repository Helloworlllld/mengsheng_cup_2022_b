/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ST7735_096.c"
#include "TCS3472.c"
#include "math.h"
#include "pic.h"
#include "HUAJI.h"
#include "OK.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//全局变量声明
uint32_t timcount = 0;
uint8_t mem[3000];
uint16_t mem_i = 0;
uint16_t remember_i = 0;

// rgb led灯pwm输出
void rgb(uint16_t r00, uint16_t g00, uint16_t b00)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, b00 * 0.8);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, g00 * 0.9);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, r00);
}

// 24bit rgb数据转16bit
uint16_t rgb16(uint8_t r00, uint8_t g00, uint8_t b00)
{
  r00 >>= 3;
  g00 >>= 2;
  b00 >>= 3;
  return ((r00 << 11) | (g00 << 5) | b00);
}

// TCS LED亮度pwm输出
void T_LED_OUTPUT(uint8_t T_LED_LEVEL)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, T_LED_LEVEL);
}

//记录
void MEM(uint8_t r, uint8_t g, uint8_t b)
{
  mem[mem_i] = r;
  mem[mem_i + 1] = g;
  mem[mem_i + 2] = b;
  mem_i += 3;
}

//单次手动测量
void color0()
{
  uint32_t color0r = 0, color0g = 0, color0b = 0, color0c = 0; //各通道颜色初始化
  float getcount;                                              //计时终数值
  timcount = 0;                                                //计时溢出计数归零，开始计时
  for (int i = 0; i <= 3; i++)
  {
    color0r += TCS34725_GetChannelData(TCS34725_RDATAL);
    color0g += TCS34725_GetChannelData(TCS34725_GDATAL);
    color0b += TCS34725_GetChannelData(TCS34725_BDATAL);
    color0c += TCS34725_GetChannelData(TCS34725_CDATAL);
  } //读取4次各通道数据并求平均
  color0r = color0r / 3.1;
  color0g = color0g / 4.85;
  color0b = color0b / 4;
  float color1r = color0r, color1g = color0g, color1b = color0b, i = 0;
  while (color1r >= 255 || color1g >= 255 || color1b >= 255)
  {
    color1r = color0r / i;
    color1g = color0g / i;
    color1b = color0b / i;
    i += 0.01;
  }
  color0b = color1b;
  color0r = color1r;
  color0g = color1g;
  getcount = (float)timcount / 10.0;                        //计算用时，每0.1ms溢出一次
  rgb(color0r, color0g, color0b);                           // rgb灯输出
  LCD_Fill(0, 0, 60, 80, rgb16(color0r, color0g, color0b)); // lcd屏输出开始
  LCD_Fill(60, 0, 160, 80, WHITE);
  char STR0[5];
  sprintf(STR0, "R=%d", color0r);
  LCD_ShowString(61, 0, STR0, BLACK, BLACK, 16, 1);
  sprintf(STR0, "G=%d", color0g);
  LCD_ShowString(61, 16, STR0, BLACK, BLACK, 16, 1);
  sprintf(STR0, "B=%d", color0b);
  LCD_ShowString(61, 32, STR0, BLACK, BLACK, 16, 1);
  sprintf(STR0, "C=%d", color0c / 4);
  LCD_ShowString(61, 48, STR0, BLACK, BLACK, 16, 1);
  char STR1[20];
  sprintf(STR1, "COST=%.1fms", getcount);
  LCD_ShowString(61, 64, STR1, BLACK, BLACK, 16, 1); // lcd屏输出结束
  MEM(color0r, color0g, color0b);
  if (abs(color0r - mem[remember_i - 3]) < 50 && abs(color0g - mem[remember_i - 2]) < 50 && abs(color0b - mem[remember_i - 1]) < 50 && fabs(color0r / color0g - mem[remember_i - 3] / mem[remember_i - 2]) < 0.1 && fabs(color0r / color0b - mem[remember_i - 3] / mem[remember_i - 1]) < 0.1)
  {
    LCD_Fill(0, 0, 60, 40, rgb16(mem[remember_i - 3], mem[remember_i - 2], mem[remember_i - 1]));
    LCD_ShowString(0, 8, "SAME!", rgb16(255 - mem[remember_i - 3], 255 - mem[remember_i - 2], 255 - mem[remember_i - 1]), BLACK, 24, 1);
    LCD_DrawLine(0, 40, 59, 40, BLACK);
    LCD_DrawLine(60, 0, 59, 80, BLACK);
    LCD_ShowPicture(100, 20, 60, 29, gImage_HUAJI);
  }
}

//自动单次测量（高亮度）
uint8_t color1()
{
  if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(1);
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
      return 0;
    }
  }
  while (1)
  {
    if (TCS34725_GetChannelData(TCS34725_CDATAL) >= 350)
    {
      float getcount; //计时终数值
      timcount = 0;   //计时溢出计数归零，开始计时
      uint8_t i = 9;
      TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_24MS);
      HAL_Delay(25);
      while (TCS34725_GetChannelData(TCS34725_CDATAL) >= 300 && i >= 1)
      {
        T_LED_OUTPUT(i);
        HAL_Delay(25);
        i--;
      }
      TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_101MS);
      HAL_Delay(110);
      uint32_t color0r = 0, color0g = 0, color0b = 0, color0c = 0; //各通道数据
      for (int i = 0; i <= 3; i++)
      {
        color0r += TCS34725_GetChannelData(TCS34725_RDATAL);
        color0g += TCS34725_GetChannelData(TCS34725_GDATAL);
        color0b += TCS34725_GetChannelData(TCS34725_BDATAL);
        color0c += TCS34725_GetChannelData(TCS34725_CDATAL);
      } //读取4次各通道数据并求平均
      color0r = color0r / 2.4;
      color0g = color0g / 3.78;
      color0b = color0b / 3.15;                                              //调色
      float color1r = color0r, color1g = color0g, color1b = color0b, i1 = 0; // i1用于计数
      while (color1r >= 255 || color1g >= 255 || color1b >= 255)
      {
        color1r = color0r / i1;
        color1g = color0g / i1;
        color1b = color0b / i1;
        i1 += 0.01;
      } //防止溢出

      color0b = color1b;
      color0r = color1r;
      color0g = color1g;

      getcount = (float)timcount / 10.0;                        //计算用时，0.1ms溢出一次
      rgb(color0r, color0g, color0b);                           // rgb灯输出
      LCD_Fill(0, 0, 60, 80, rgb16(color0r, color0g, color0b)); // lcd屏输出开始
      LCD_Fill(60, 0, 160, 80, WHITE);
      char STR0[5];
      sprintf(STR0, "R=%d", color0r);
      LCD_ShowString(61, 0, STR0, BLACK, BLACK, 16, 1);
      sprintf(STR0, "G=%d", color0g);
      LCD_ShowString(61, 16, STR0, BLACK, BLACK, 16, 1);
      sprintf(STR0, "B=%d", color0b);
      LCD_ShowString(61, 32, STR0, BLACK, BLACK, 16, 1);
      sprintf(STR0, "C=%d", color0c / 4);
      LCD_ShowString(61, 48, STR0, BLACK, BLACK, 16, 1);
      char STR1[20];
      sprintf(STR1, "COST=%.1fms", getcount);
      LCD_ShowString(61, 64, STR1, BLACK, BLACK, 16, 1); // lcd屏输出
      MEM(color0r, color0g, color0b);
      T_LED_OUTPUT(9);
      if (abs(color0r - mem[remember_i - 3]) < 50 && abs(color0g - mem[remember_i - 2]) < 50 && abs(color0b - mem[remember_i - 1]) < 50 && fabs(color0r / color0g - mem[remember_i - 3] / mem[remember_i - 2]) < 0.1 && fabs(color0r / color0b - mem[remember_i - 3] / mem[remember_i - 1]) < 0.1)
      {
        LCD_Fill(0, 0, 60, 40, rgb16(mem[remember_i - 3], mem[remember_i - 2], mem[remember_i - 1]));
        LCD_ShowString(0, 8, "SAME!", rgb16(255 - mem[remember_i - 3], 255 - mem[remember_i - 2], 255 - mem[remember_i - 1]), BLACK, 24, 1);
        LCD_DrawLine(0, 40, 59, 40, BLACK);
        LCD_DrawLine(60, 0, 59, 80, BLACK);
        LCD_ShowPicture(100, 20, 60, 29, gImage_HUAJI);
      }
      while (TCS34725_GetChannelData(TCS34725_CDATAL) >= 300)
      {
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
        {
          HAL_Delay(1);
          if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
          {
            return 0;
          }
        }
      }
    }
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
      HAL_Delay(1);
      if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
      {
        return 0;
      }
    }
  }
}

//显示历史
void HISTORY()
{
  if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(1);
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
      return;
    }
  }
  uint16_t i = 0, pmax = ceil((double)mem_i / 18.0 - 0.01), p = 1;
  int16_t j = mem_i - 1;
  char rgbstr[13], rcdstr[15];
  sprintf(rcdstr, "%d", mem_i / 3);
  while (p <= pmax)
  {
    char pgstr[10];
    sprintf(pgstr, "%d/%d", p, pmax);
    LCD_Fill(0, 0, 160, 80, BLACK);
    LCD_ShowString(120, 5, rcdstr, WHITE, BLACK, 16, 1);
    LCD_ShowString(105, 25, "records", WHITE, BLACK, 16, 1);
    LCD_ShowString(115, 50, pgstr, WHITE, BLACK, 24, 1);
    while (i <= 17 && j >= 2)
    {
      sprintf(rgbstr, "(%3d,%3d,%3d)", mem[j - 2], mem[j - 1], mem[j]);
      LCD_ShowString(0, 16 * i / 3, rgbstr, rgb16(255 - mem[j - 2], 255 - mem[j - 1], 255 - mem[j]), rgb16(mem[j - 2], mem[j - 1], mem[j]), 16, 0);
      i += 3;
      j -= 3;
    }
    i = 0;
    p++;
    HAL_Delay(100);
    while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
      ;
  }
  HAL_Delay(500);
  while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
    HAL_Delay(1);
  while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    HAL_Delay(1);
}

//计时器计时中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  timcount++;
}

//标记一种颜色
void mark()
{
  uint32_t color0r = 0, color0g = 0, color0b = 0, color0c = 0; //各通道颜色变量初始化
  float getcount;                                              //计时终数值
  timcount = 0;                                                //计时溢出计数归零，开始计时
  for (int i = 0; i <= 3; i++)
  {
    color0r += TCS34725_GetChannelData(TCS34725_RDATAL);
    color0g += TCS34725_GetChannelData(TCS34725_GDATAL);
    color0b += TCS34725_GetChannelData(TCS34725_BDATAL);
    color0c += TCS34725_GetChannelData(TCS34725_CDATAL);
  } //读取4次各通道数据并求平均
  color0r = color0r / 3.1;
  color0g = color0g / 4.85;
  color0b = color0b / 4;
  float color1r = color0r, color1g = color0g, color1b = color0b, i = 0;
  while (color1r >= 255 || color1g >= 255 || color1b >= 255)
  {
    color1r = color0r / i;
    color1g = color0g / i;
    color1b = color0b / i;
    i += 0.01;
  }
  color0b = color1b;
  color0r = color1r;
  color0g = color1g;
  getcount = (float)timcount / 10.0;
  MEM(color0r, color0g, color0b);
  remember_i = mem_i;
  LCD_Fill(0, 0, 160, 80, WHITE);
  LCD_Fill(0, 0, 60, 80, rgb16(mem[mem_i - 3], mem[mem_i - 2], mem[mem_i - 1]));
  LCD_ShowString(61, 0, "MARKED", BLACK, BLACK, 16, 1);
  char STR0[5];
  sprintf(STR0, "R=%d", color0r);
  LCD_ShowString(61, 16, STR0, BLACK, BLACK, 16, 1);
  sprintf(STR0, "G=%d", color0g);
  LCD_ShowString(61, 32, STR0, BLACK, BLACK, 16, 1);
  sprintf(STR0, "B=%d", color0b);
  LCD_ShowString(61, 48, STR0, BLACK, BLACK, 16, 1);
  char STR1[20];
  sprintf(STR1, "%.1fms", getcount);
  LCD_ShowString(61, 64, STR1, BLACK, BLACK, 16, 1); // lcd屏输出
  LCD_ShowPicture(102, 16, 58, 58, gImage_OK);
  HAL_Delay(500);
  while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    HAL_Delay(1);
  while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
    HAL_Delay(1);
  while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    HAL_Delay(1);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // INIT_START
  HAL_TIM_Base_Start_IT(&htim2);
  //  pwm
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // b LED pwm START
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // g LED pwm START
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // r LED pwm START
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // TCS_LED pwm START

  // lcd初始化
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET);
  LCD_Init();

  // rgb测试
  rgb(0, 0, 0);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(50);
  rgb(255, 0, 0);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(50);
  rgb(0, 255, 0);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(50);
  rgb(0, 0, 255);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(50);
  if (TCS34725_Init() == 0)
  {
    rgb(255, 0, 0);
    LCD_Fill(0, 0, 160, 80, RED);
    LCD_ShowString(40, 16, "E TCS", BLACK, WHITE, 32, 1);
    while (1)
      ;
  }
  for (int i = 0; i <= 9; i++)
    T_LED_OUTPUT(i);

  rgb(0, 0, 0);
  T_LED_OUTPUT(9);

  LCD_Fill(0, 0, 160, 80, WHITE);
  LCD_ShowPicture(40, 0, 80, 80, gImage_pic); //初始化结束

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (timcount >= 10000000)
      timcount = 0;
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
      HAL_Delay(1);
      if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
      {
        color0();
        HAL_Delay(500);
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
        {
          HAL_Delay(1);
          if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
          {
            LCD_Fill(0, 0, 160, 80, LIGHTBLUE);
            LCD_ShowString(45, 24, "AUTO", WHITE, WHITE, 32, 1);
            HAL_Delay(500);
            color1();
            HAL_Delay(500);
            if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
            {
              HAL_Delay(1);
              if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
              {
                LCD_Fill(0, 0, 160, 80, LIGHTBLUE);
                LCD_ShowString(25, 24, "HISTORY", WHITE, WHITE, 32, 1);
                HAL_Delay(500);
                HISTORY();
                HAL_Delay(500);
                if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
                {
                  HAL_Delay(1);
                  if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
                  {
                    LCD_Fill(0, 0, 160, 80, LIGHTBLUE);
                    LCD_ShowString(45, 24, "MARK", WHITE, WHITE, 32, 1);
                    HAL_Delay(500);
                    mark();
                    LCD_Fill(0, 0, 160, 80, LIGHTBLUE);
                    LCD_ShowString(30, 24, "MANUAL", WHITE, WHITE, 32, 1);
                  }
                }
                LCD_Fill(0, 0, 160, 80, LIGHTBLUE);
                LCD_ShowString(30, 24, "MANUAL", WHITE, WHITE, 32, 1);
              }
            }
            LCD_Fill(0, 0, 160, 80, LIGHTBLUE);
            LCD_ShowString(30, 24, "MANUAL", WHITE, WHITE, 32, 1);
          }
        }
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
