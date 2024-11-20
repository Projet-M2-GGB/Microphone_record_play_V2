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
#include "dma.h"
#include "sai.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery_audio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define AUDIO_BUFFER_SIZE       512  // 512 bytes
#define AUDIO_DEFAULT_VOLUME    70 // 70% volume

typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF = 1,
  BUFFER_OFFSET_FULL = 2,
}BUFFER_StateTypeDef;

typedef enum {
  AUDIO_ERROR_NONE = 0,
  AUDIO_ERROR_NOTREADY,
  AUDIO_ERROR_IO,
  AUDIO_ERROR_EOF,
}AUDIO_ErrorTypeDef;

typedef enum {
  AUDIO_STATE_IDLE = 0,
  AUDIO_STATE_INIT,
  AUDIO_STATE_PLAYING,
}AUDIO_PLAYBACK_StateTypeDef;

typedef struct {
  uint8_t buff[AUDIO_BUFFER_SIZE];
  uint32_t fptr;
  BUFFER_StateTypeDef state;
}AUDIO_BufferTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define AUDIO_FILE_SIZE               524288 /* 512 KB , flash size = 1 Mb */
#define AUDIO_START_OFFSET_ADDRESS    0            /* Offset relative to audio file header size */
#define AUDIO_FILE_ADDRESS            0x08080000   /* Audio file address, internal flash memory region starting at 512 KB */

#define AUDIO_BLOCK_SIZE   ((uint32_t)0xFFFE)
#define AUDIO_NB_BLOCKS    ((uint32_t)4)

#define SDRAM_DEVICE_ADDR  ((uint32_t)0xC0000000)
#define AUDIO_REC_START_ADDR         SDRAM_DEVICE_ADDR


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ALIGN_32BYTES (static AUDIO_BufferTypeDef  buffer_ctl);
static AUDIO_PLAYBACK_StateTypeDef  audio_state;
static uint32_t  AudioStartAddress;
static uint32_t  AudioFileSize;

static uint16_t  internal_buffer[AUDIO_BLOCK_SIZE];
uint32_t  audio_rec_buffer_state;

extern SAI_HandleTypeDef haudio_in_sai;
extern DMA_HandleTypeDef hdma_sai1_a;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker-
>Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

AUDIO_ErrorTypeDef AUDIO_Start(uint32_t audio_start_address, uint32_t audio_file_size);
static uint32_t GetData(void *pdata, uint32_t offset, uint8_t *pbuf, uint32_t NbrOfData);

static void AudioRec_SetHint(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t button_pressed = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SAI1_Init();
  MX_FMC_Init();

  /* USER CODE BEGIN 2 */

  /* Initialize SDRAM */
  BSP_SDRAM_Init();

  /* Initialize Audio Recorder */
	if (BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) == AUDIO_OK)
	{
		printf("Audio input setup OK\r\n");
	}

	uint32_t  block_number;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  check_button_release();
	  printf("Waiting for input to record...\r\n");
	  HAL_Delay(1000);
	  if (button_pressed == 1)
	  {
		  check_button_release();
		  printf("recording...\r\n");
		  /* Start Recording */
		  BSP_AUDIO_IN_Record(internal_buffer, AUDIO_BLOCK_SIZE);
		  for (block_number = 0; block_number < AUDIO_NB_BLOCKS; block_number++)
		  {
		    /* Wait end of half block recording */
		    while(audio_rec_buffer_state != BUFFER_OFFSET_HALF)
		    {
		      //printf("Waiting for BUFFER_OFFSET_HALF...\r\n");
		      //HAL_Delay(1000);
		      if (button_pressed == 1)
		      {
		    	check_button_release();
		        /* Stop Player before close Test */
		        BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
		        return;
		      }
		    }

		    audio_rec_buffer_state = BUFFER_OFFSET_NONE;

		    /* Copy recorded 1st half block in SDRAM */

		    printf("DMA State: %d\r\n", HAL_DMA_GetState(&hdma_sai1_a));

		    printf("DMA Error: %lu\r\n", HAL_DMA_GetError(&hdma_sai1_a));

		    if (HAL_DMA_GetState(&haudio_in_sai) == HAL_DMA_STATE_READY) {
		        // DMA is ready, it's safe to use memcpy
			    printf("Copying data to memory...\r\n");
			    memcpy((uint32_t *)(AUDIO_REC_START_ADDR + (block_number * AUDIO_BLOCK_SIZE * 2)),
			           internal_buffer,
			           AUDIO_BLOCK_SIZE);
			    printf("Copy complete.\r\n");
		    }


		    /* Wait end of one block recording */
		    while(audio_rec_buffer_state != BUFFER_OFFSET_FULL)
		    {
		      if (button_pressed == 1)
		      {
			    check_button_release();
		        /* Stop Player before close Test */
		        BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
		        return;
		      }
		    }
		    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
		    /* Copy recorded 2nd half block in SDRAM */



		    if (HAL_DMA_GetState(&hdma_sai1_a) == HAL_DMA_STATE_READY) {
		        // DMA is ready, it's safe to use memcpy
		    	printf("Copying data to memory...\r\n");
			    memcpy((uint32_t *)(AUDIO_REC_START_ADDR + (block_number * AUDIO_BLOCK_SIZE * 2)),
			           internal_buffer,
			           AUDIO_BLOCK_SIZE);
			    printf("Copy complete.\r\n");
		    }


		  }
		  printf("stopped recording\r\n");
		  BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);

		  for (int i = 0; i < 512; i++) {
		      printf("SDRAM Data[%d]: %02X\r\n", i, ((uint8_t *)AUDIO_REC_START_ADDR)[i]);
		  }

		  printf("playing recording...\r\n");
		  /* -----------Start Playback -------------- */
		  /* Initialize audio IN at REC_FREQ*/
		  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 70, DEFAULT_AUDIO_IN_FREQ);
		  BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

		  /* Play the recorded buffer*/
		  AUDIO_Start(AUDIO_REC_START_ADDR, AUDIO_BLOCK_SIZE * AUDIO_NB_BLOCKS * 2);  /* Use Audio play demo to playback sound */
		  printf("playback end...\r\n");

		  /*
		  while (button_pressed == 0)
		  {
			  if(button_pressed == 1)
					  {

					  }
		  }
		  */

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 100;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART2 and Loop until the end
of transmission */
HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
return ch;
}


AUDIO_ErrorTypeDef AUDIO_Start(uint32_t audio_start_address, uint32_t audio_file_size)
{
  uint32_t bytesread;

  buffer_ctl.state = BUFFER_OFFSET_NONE;
  AudioStartAddress = audio_start_address;
  AudioFileSize = audio_file_size;
  bytesread = GetData( (void *)AudioStartAddress,
                      0,
                      &buffer_ctl.buff[0],
                      AUDIO_BUFFER_SIZE);
  if(bytesread > 0)
  {
    /* Clean Data Cache to update the content of the SRAM */
    SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE/2);

    BSP_AUDIO_OUT_Play((uint16_t*)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE);
    audio_state = AUDIO_STATE_PLAYING;
    buffer_ctl.fptr = bytesread;
    return AUDIO_ERROR_NONE;
  }
  return AUDIO_ERROR_IO;
}

static uint32_t GetData(void *pdata, uint32_t offset, uint8_t *pbuf, uint32_t NbrOfData)
{
  uint8_t *lptr = pdata;
  uint32_t ReadDataNbr;

  ReadDataNbr = 0;
  printf("offset: %lu, AudioFileSize: %lu, NbrOfData: %lu\r\n", offset, AudioFileSize, NbrOfData);

  printf("pdata: %p, pbuf: %p\r\n", pdata, pbuf);
  printf("buffer ctl.buff : %p\r\n", buffer_ctl.buff);

  while(((offset + ReadDataNbr) < AudioFileSize) && (ReadDataNbr < NbrOfData))
  {
    pbuf[ReadDataNbr]= lptr [offset + ReadDataNbr];
    ReadDataNbr++;
  }
  return ReadDataNbr;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  if (GPIO_Pin == Button_user_Pin && button_pressed == 0)
      {
	  	  button_pressed = 1;
	  	  printf("Button pressed...\r\n");
      }

}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  if(audio_state == AUDIO_STATE_PLAYING)
  {
    /* allows AUDIO_Process() to refill 2nd part of the buffer  */
    buffer_ctl.state = BUFFER_OFFSET_FULL;
  }
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
  if(audio_state == AUDIO_STATE_PLAYING)
  {
    /* allows AUDIO_Process() to refill 1st part of the buffer  */
    buffer_ctl.state = BUFFER_OFFSET_HALF;
  }
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  printf("Full Transfer Callback triggered.\r\n");
  audio_rec_buffer_state = BUFFER_OFFSET_FULL;
  return;
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  printf("Half Transfer Callback triggered.\r\n");
  audio_rec_buffer_state = BUFFER_OFFSET_HALF;
  return;
}

void BSP_AUDIO_IN_Error_CallBack(void)
{
  /* This function is called when an Interrupt due to transfer error on or peripheral
     error occurs. */
  /* Display message on the LCD screen */
  printf("DMA ERROR\r\n");

  /* Stop the program with an infinite loop */
  while (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    return;
  }
  /* could also generate a system reset to recover from the error */
  /* .... */
}

void check_button_release()
{
    if (HAL_GPIO_ReadPin(GPIOI, Button_user_Pin) == GPIO_PIN_RESET) button_pressed = 0;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0xC0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
