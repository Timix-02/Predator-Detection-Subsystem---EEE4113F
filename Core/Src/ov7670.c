/*
 * ov7670.c
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */
#include <stdio.h>
#include "main.h"
#include "stm32h7xx_hal.h"
#include "common.h"
#include "ov7670.h"
#include "ov7670Config.h"
#include "ov7670Reg.h"



/*** Internal Static Variables ***/
static DCMI_HandleTypeDef *sp_hdcmi;
static DMA_HandleTypeDef  *sp_hdma_dcmi;
static I2C_HandleTypeDef  *sp_hi2c;
static uint32_t    s_destAddressForContiuousMode;
static void (* s_cbHsync)(uint32_t h);
static void (* s_cbVsync)(uint32_t v);
static uint32_t s_currentH;
static uint32_t s_currentV;

/*** Internal Function Declarations ***/
static RET ov7670_write(uint8_t regAddr, uint8_t data);
static RET ov7670_read(uint8_t regAddr, uint8_t *data);


extern UART_HandleTypeDef huart1;
extern uint8_t pictureTaken;

/*** External Function Defines ***/
RET ov7670_init(DCMI_HandleTypeDef *p_hdcmi, DMA_HandleTypeDef *p_hdma_dcmi, I2C_HandleTypeDef *p_hi2c , uint8_t CAM_Number)
{
  sp_hdcmi     = p_hdcmi;
  sp_hdma_dcmi = p_hdma_dcmi;
  sp_hi2c      = p_hi2c;
  s_destAddressForContiuousMode = 0;

  if (CAM_Number == 1){
    // HAL_GPIO_WritePin(CAM1_EN_GPIO_Port, CAM1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAM1_RST_GPIO_Port, CAM1_RST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAM1_RST_GPIO_Port, CAM1_RST_Pin, GPIO_PIN_SET);

    
  }
  if (CAM_Number == 2){
    // HAL_GPIO_WritePin(CAM2_EN_GPIO_Port, CAM2_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAM2_RST_GPIO_Port, CAM2_RST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAM2_RST_GPIO_Port, CAM2_RST_Pin, GPIO_PIN_SET);
  }


  ov7670_write(0x12, 0x80);  // RESET

  uint8_t buffer[4];
  ov7670_read(0x0b, buffer);
  // printf("[OV7670] dev id = %02X\n", buffer[0]);


  return RET_OK;
}

RET ov7670_config(uint32_t mode)
{

  ov7670_stopCap();
  ov7670_write(0x12, 0x80);  // RESET
  for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
    ov7670_write(OV7670_reg[i][0], OV7670_reg[i][1]);
  }
  return RET_OK;
}

RET ov7670_startCap(uint32_t capMode, uint32_t destAddress)
{
 pictureTaken = 0;  
  ov7670_stopCap();
  if (capMode == OV7670_CAP_CONTINUOUS) {
    /* note: continuous mode automatically invokes DCMI, but DMA needs to be invoked manually */
    s_destAddressForContiuousMode = destAddress;
    HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, destAddress, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
  } else if (capMode == OV7670_CAP_SINGLE_FRAME) {
    s_destAddressForContiuousMode = 0;
    HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, destAddress, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
    
    // HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, destAddress, 10);
  }
  return RET_OK;
}

RET ov7670_stopCap()
{
  HAL_DCMI_Stop(sp_hdcmi);
//  HAL_Delay(30);
  return RET_OK;
}

void ov7670_registerCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v))
{
  s_cbHsync = cbHsync;
  s_cbVsync = cbVsync;
}


void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  //HAL_GPIO_TogglePin(MSIZI_TRIGGER_GPIO_Port, MSIZI_TRIGGER_Pin);
  //  printf("FRAME %d\n", HAL_GetTick());
  if(s_cbVsync)s_cbVsync(s_currentV);
  if(s_destAddressForContiuousMode != 0) {
    HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, s_destAddressForContiuousMode, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
  }
  s_currentV++;
  s_currentH = 0;
  pictureTaken = 1;
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
//  printf("VSYNC %d\n", HAL_GetTick());
 HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, s_destAddressForContiuousMode, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
}

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
//  printf("HSYNC %d\n", HAL_GetTick());
 if(s_cbHsync)s_cbHsync(s_currentH);
 s_currentH++;
}

/*** Internal Function Defines ***/
static RET ov7670_write(uint8_t regAddr, uint8_t data)
{
  HAL_StatusTypeDef ret;
  do {
    ret = HAL_I2C_Mem_Write(sp_hi2c, SLAVE_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
  } while (ret != HAL_OK && 0);
  return ret;
}

static RET ov7670_read(uint8_t regAddr, uint8_t *data)
{
  HAL_StatusTypeDef ret;
  do {
    // HAL_I2C_Mem_Read doesn't work (because of SCCB protocol(doesn't have ack))? */
  //  ret = HAL_I2C_Mem_Read(sp_hi2c, SLAVE_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
    ret = HAL_I2C_Master_Transmit(sp_hi2c, SLAVE_ADDR, &regAddr, 1, HAL_MAX_DELAY);
    ret |= HAL_I2C_Master_Receive(sp_hi2c, SLAVE_ADDR, data, 1, HAL_MAX_DELAY);
  } while (ret != HAL_OK && 0);
  return ret;
}


