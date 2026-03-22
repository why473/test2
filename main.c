/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <unistd.h>
#include "main.h"
#include "math.h"
#include "max31855.h"
#include "data_operate.h"
#include "DIP.h"
#include "tjc_usart_hmi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint32_t sin_data[500];
#define PI 3.1415926
MAX31855_Data_t tem_data;
MAX31855_Data_t tem_buff;
float past_temp=10.0f;
float env_temp=0.0f;
int tem_tram_flag=0;//温度转换标志位
extern int ustart_OK;//串口发送标志位
extern int temp_set_flag;
extern int temp;
int DIP_res = 0;
int DIP_res_past=0;
typedef struct
{
  float data[20];
  int head;
  int tail;
}tem_ring;
tem_ring water_temp;
int prepare_flag=0;//提前准备数据

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float smoothing(tem_ring input)
{
  float result =0;
  if ((input.tail+1)%20==input.head)
  {

    for (int i=0;i<20;i++)
    {
      result += input.data[i];

    }
    result =result/20.0;
  }
  else
  {


    for (int i=input.head;i<input.tail;i++)
    {
      result += input.data[i];
    }
    result =result/(input.tail-input.head);

  }
  return result;
}

void ring_input(tem_ring* data, float value)
{
  if ((data->tail+1)%20!=data->head)
  {
    data->data[data->tail]=value;
    data->tail=(data->tail+1)%20;
  }
  else
  {
    data->data[data->tail]=value;
    data->tail=(data->tail+1)%20;
    data->head=(data->head+1)%20;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim3)
{
    past_temp=tem_data.thermocouple_temp;
    MAX31855_ReadTemperature(&tem_data);

    // if (!prepare_flag) {
    //   ring_input(&water_temp,tem_buff.thermocouple_temp);
    // }
    // else {
    //   int a=smoothing(water_temp);
    //   if (tem_buff.data_valid&&tem_buff.thermocouple_temp-smoothing(water_temp)>-3&&tem_buff.thermocouple_temp-smoothing(water_temp)<3) {
    //     tem_tram_flag = 1;
    //     tem_data.thermocouple_temp=tem_buff.thermocouple_temp;
    //     tem_data.data_valid=tem_buff.data_valid;
    //     tem_data.faults=tem_buff.faults;
    //     tem_data.internal_temp=tem_buff.internal_temp;
    //     ring_input(&water_temp,tem_data.thermocouple_temp);
    //   }
    // }
  tem_tram_flag=1;

}
void ring_init(tem_ring* data)
{
  data->head=0;
  data->tail=0;
}


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
extern UART_HandleTypeDef huart1; // 假设使用 USART2
#define Print_UART huart1

/**
  * @brief 重写 _write 系统调用，将标准输出重定向到串口
  * @param file 文件描述符（STDOUT_FILENO=1, STDERR_FILENO=2）
  * @param ptr  要发送的数据指针
  * @param len  数据长度
  * @retval 成功发送的字节数
  */
int _write(int file, char *ptr, int len) {
  // 仅重定向标准输出和标准错误输出
  if (file == STDOUT_FILENO || file == STDERR_FILENO) {
    // 使用 HAL 库的阻塞式发送函数
    HAL_UART_Transmit(&Print_UART, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len; // 返回成功发送的字节数
  }
  // 对其他文件描述符返回错误（未处理）
  return -1;
}

/* 可选：如果你希望处理换行符 \n 自动转换为 \r\n（Windows 终端习惯） */
int _write_crlf(int file, char *ptr, int len) {
  if (file == STDOUT_FILENO || file == STDERR_FILENO) {
    for (int i = 0; i < len; i++) {
      char ch = ptr[i];
      HAL_UART_Transmit(&Print_UART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
      if (ch == '\n') {
        char cr = '\r';
        HAL_UART_Transmit(&Print_UART, (uint8_t *)&cr, 1, HAL_MAX_DELAY);
      }
    }
    return len;
  }
  return -1;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MAX31855_Init();
  ring_init(&water_temp);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_Delay(2000);
  prepare_flag=1;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);
  // printf("t3.txt=\"%.1f\"\xff\xff\xff",90.1);
  // fflush(stdout);
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    int tem_water=0,tem_env=0;
    tem_water = (int)tem_data.thermocouple_temp;

    tem_env = (int)tem_data.internal_temp;
    printf("t3.txt=\"%.1f\"\xff\xff\xff",tem_data.thermocouple_temp);
    fflush(stdout);
    HAL_Delay(10);
    printf("t4.txt=\"%.1f\"\xff\xff\xff",tem_data.internal_temp);
    fflush(stdout);
    HAL_Delay(10);
    printf("add s0.id,0,%d\xff\xff\xff",tem_water);
    fflush(stdout);
    HAL_Delay(10);
    printf("add s0.id,1,%d\xff\xff\xff",tem_env);
    fflush(stdout);
    HAL_Delay(10);
    printf("add s0.id,2,%d\xff\xff\xff",temp);
    fflush(stdout);
    HAL_Delay(10);
    ustart();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_RESET);
    if (ustart_OK)
     {
       Get_Data();//确认传输完整指令，解析指令
       ustart_OK = 0;
     }
     if (temp_set_flag)//温度传输完毕后处理
     {
       if (tem_tram_flag)
       {
         DIP_res_past = DIP_res;//记录原来的结果
         if (tem_data.thermocouple_temp - temp>2.5)//温差大于一度默认最快处理
         {
           DIP_res=-1000;
         }
         else if (tem_data.thermocouple_temp - temp<-2.5) {
           DIP_res=1000;
         }
         else
         {
           DIP_res=DIP(tem_data.thermocouple_temp,past_temp,temp);
           if (DIP_res>1000)
           {
             DIP_res=1000;
           }
           else if (DIP_res<-1000)
           {
             DIP_res=-1000;
           }
           if (DIP_res>-40&&DIP_res<40) {
             DIP_res=50;
           }
           else if (DIP_res>960&&DIP_res<-960&&DIP_res!=1000&&DIP_res!=-1000) {
             DIP_res=950;
           }
         }
         //保持
         if (tem_data.thermocouple_temp - temp<1&&tem_data.thermocouple_temp - temp>0) {
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_RESET);
           HAL_Delay(200);
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
           HAL_Delay(10);
         }
         else if (tem_data.thermocouple_temp - temp>-1&&tem_data.thermocouple_temp - temp<0) {
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_RESET);
           HAL_Delay(200);
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
           HAL_Delay(20);
         }
         //调参
         else if (DIP_res>0&&DIP_res<1000)
         {
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_RESET);
           HAL_Delay(250-DIP_res/4);
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
           HAL_Delay(DIP_res/4);
         }
         else if (DIP_res<0&&DIP_res>-1000)
         {
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_RESET);
           HAL_Delay(250+DIP_res/4);
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
           HAL_Delay(-DIP_res/4);

         }
         else if (DIP_res==1000) {
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_RESET);
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
           HAL_Delay(250);
         }
         else if (DIP_res==-1000) {
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_RESET);
           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
           HAL_Delay(250);
         }
     }
  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
