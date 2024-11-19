/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
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
#include "dma.h"

/* USER CODE BEGIN 0 */
extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    /* DMA initialization for SAI RX */
    hdma_sai1_a.Instance = DMA2_Stream1;  // Use the correct DMA stream
    hdma_sai1_a.Init.Channel = DMA_CHANNEL_0;  // Channel associated with SAI
    hdma_sai1_a.Init.Direction = DMA_PERIPH_TO_MEMORY; // Periph to Mem direction
    hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;  // No increment for peripheral address
    hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;  // Increment memory address
    hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 16-bit data width
    hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD; // 16-bit data width
    hdma_sai1_a.Init.Mode = DMA_CIRCULAR;  // Circular mode
    hdma_sai1_a.Init.Priority = DMA_PRIORITY_HIGH; // High priority
    hdma_sai1_a.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  // FIFO mode disabled

    if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK)
    {
        Error_Handler();  // Handle error if DMA initialization fails
    }

    /* Link DMA to the SAI peripheral */
    __HAL_LINKDMA(&hsai_BlockA1, hdmarx, hdma_sai1_a);
}


/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

