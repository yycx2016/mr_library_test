/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "mx_spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "include/mr_lib.h"
#include "mr_board.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// #define LED_PIN_NUMBER 50 //PD2
#define LED_PIN_NUMBER 8 //PA6
#define SPI_CS_PIN_NUMBER 4 //PA4
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
struct mr_spi_dev host_dev;
int host_ds = -1;

void spi_init(void)
{
    int ret = MR_EOK;

    ret = mr_spi_dev_register(&host_dev, "spi1/host", SPI_CS_PIN_NUMBER, MR_SPI_CS_ACTIVE_LOW);
    if (ret < 0)
    {
        mr_printf("host spi device register failed: %d\r\n", mr_strerror(ret));
        return;
    }

    host_ds = mr_dev_open("spi1/host", MR_O_RDWR);
    if (host_ds < 0)
    {
        mr_printf("host spi device open failed: %d\r\n", mr_strerror(ret));
        return;
    }
//    mr_dev_ioctl(host_ds, MR_IOC_SPI_SET_REG, MR_MAKE_LOCAL(int, SPI_CS_PIN_NUMBER));
}
MR_INIT_APP_EXPORT(spi_init);
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	
  mr_auto_init();

 int ds = mr_dev_open("serial1", MR_O_RDWR);
 int ds_io = mr_dev_open("pin", MR_O_WRONLY);
 mr_dev_write(ds, "Hello World\r\n", sizeof("Hello World\r\n"));
 mr_dev_write(ds, "Hello MR LIBRARY\r\n", sizeof("Hello MR LIBRARY\r\n"));
 mr_dev_ioctl(ds_io, MR_IOC_PIN_SET_NUMBER, MR_MAKE_LOCAL(int, LED_PIN_NUMBER));
 mr_dev_ioctl(ds_io, MR_IOC_PIN_SET_MODE, MR_MAKE_LOCAL(int, MR_PIN_MODE_OUTPUT));
 
 uint8_t wr_buf[] = {0x01, 0x02, 0x03, 0x04};
 mr_dev_write(host_ds, wr_buf, sizeof(wr_buf));
// uint8_t rd_buf[4] = {0};
// mr_dev_read(ds, rd_buf, sizeof(rd_buf));
// mr_dev_close(ds);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    mr_dev_write(ds_io, MR_MAKE_LOCAL(uint8_t, 1), sizeof(uint8_t));
    mr_delay_ms(500);
    mr_dev_write(ds_io, MR_MAKE_LOCAL(uint8_t, 0), sizeof(uint8_t));
    mr_delay_ms(500); 
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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
