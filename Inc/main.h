#ifndef MAIN_H
#define MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"

/* Structures ----------------------------------------------------------------*/
typedef enum{
	CW,
	CCW
}RotationDir;

typedef struct{
	uint16_t setPoint;
	uint16_t angle;
	uint16_t speed;
	RotationDir dir;
	uint32_t adcValue;
	uint16_t pwmValue;
}ServoStruct;

/* Private variables ---------------------------------------------------------*/
DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

SDRAM_HandleTypeDef hsdram1;

TIM_HandleTypeDef htim3;

ADC_HandleTypeDef hadc1;

char str[20];
char strADC[20];
ServoStruct servo;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
static void setPWM(TIM_HandleTypeDef, uint32_t, uint16_t, uint16_t);

#endif
