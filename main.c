/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/////////////////////////////////////////////////////////////////////////////////////////////////
// writing registers of sensor

#define AC1 (0xAA<<8) + 0xAB
#define AC2 (0xAC<<8) + 0xAD
#define AC3 (0xAE<<8) + 0xAF
#define AC4 (0xB0<<8) + 0xB1
#define AC5 (0xB2<<8) + 0xB3
#define AC6 (0xB4<<8) + 0xB5

#define B1 (0xB6<<8) + 0xB7
#define B2 (0xB8<<8) + 0xB9

#define MB (0xBA<<8) + 0xBB
#define MC (0xBC<<8) + 0xBD
#define MD (0xBE<<8) + 0xBF



#define ctrl_meas  0xF4
#define	soft_reset 0xE0
#define sen_id     0xD0
#define dev_add    0xEF


uint8_t oss0 = 0x34;					     //wait 4.5
uint8_t oss1 = 0x74;  				   	//wait 7.5
uint8_t oss2 = 0xB4;				 	   //wait 13.5
uint8_t oss3 = 0xF4; 				  	//wait 25.5
uint8_t temprature = 0x2E;		 // wait 4.5

int ac1,ac2,ac3,b1,b2,mb,mc,md,ac4,ac5,ac6;

long x1,x2,b5,b6,x3,b3,b4,b7;

long UT,UP;
long temp,pre;
////////////////////////////////////////////////////////////////////////////////////////////////

long oldmin,oldmax;
uint8_t newmin=0,newmax=255;

////////////////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void calib_presure(void);
void Read_pr_data(uint8_t oss);
void usb_tran_uint16(uint16_t num);
int new_range_long_int(long val,long oldmin,long oldmax,int newmin,int newmax);
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	
	calib_presure();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		Read_pr_data(oss3);
		
		usb_tran_uint16(pre);
		
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int new_range_long_int(long val,long oldmin,long oldmax,int newmin,int newmax)
{
	long oldrange=oldmax-oldmin;
	int newrange=newmax-newmin;
	int newval;
	newval=(((val-oldmin)*newrange)/oldrange)+newmin;
	
	return newval;

}


void usb_tran_uint16(uint16_t num)
{

uint8_t buffer[2]={0};

buffer[0]=num>>8;
buffer[1]=num-(buffer[0]<<8);


CDC_Transmit_FS(buffer,2);

}



void calib_presure(void)
{

uint8_t	buffer[2]={0};	
	
HAL_I2C_Mem_Read(&hi2c1,dev_add,AC1,2,buffer,2,100);
ac1=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,AC2,2,buffer,2,100);
ac2=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,AC3,2,buffer,2,100);
ac3=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,AC4,2,buffer,2,100);
ac4=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,AC5,2,buffer,2,100);
ac5=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,AC6,2,buffer,2,100);
ac6=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,B1,2,buffer,2,100);
b1=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,B2,2,buffer,2,100);
b2=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,MB,2,buffer,2,100);
mb=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,MC,2,buffer,2,100);
mc=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;

HAL_I2C_Mem_Read(&hi2c1,dev_add,MD,2,buffer,2,100);
md=(buffer[0]<<8)+buffer[1];
for(int i=0 ; i<=1 ; i++)
	buffer[i]=0;


}


void Read_pr_data(uint8_t oss)
{

uint8_t buffer[3]={0};
int oss_set=0;
	

HAL_I2C_Mem_Write(&hi2c1,dev_add,ctrl_meas,1,&temprature,1,100);
	HAL_Delay(5);
HAL_I2C_Mem_Read(&hi2c1,dev_add,0xF6,1,&buffer[0],1,100);	
HAL_I2C_Mem_Read(&hi2c1,dev_add,0xF7,1,&buffer[1],1,100);
UT=(buffer[0]<<8)+buffer[1];
for(int i=0;i<=2;i++)
	buffer[i]=0;



if(oss==oss0)
{
	oss_set=0;
HAL_I2C_Mem_Write(&hi2c1,dev_add,ctrl_meas,1,&oss,1,100);
	HAL_Delay(5);	
}

else if(oss==oss1)
{
	oss_set=1;
HAL_I2C_Mem_Write(&hi2c1,dev_add,ctrl_meas,1,&oss,1,100);
	HAL_Delay(8);	
}

else if(oss==oss2)
{
	oss_set=2;
HAL_I2C_Mem_Write(&hi2c1,dev_add,ctrl_meas,1,&oss,1,100);
	HAL_Delay(14);	
}

else if(oss==oss3)
{
	oss_set=3;
HAL_I2C_Mem_Write(&hi2c1,dev_add,ctrl_meas,1,&oss,1,100);
	HAL_Delay(26);
}

HAL_I2C_Mem_Read(&hi2c1,dev_add,0xF6,1,&buffer[0],1,100);	
HAL_I2C_Mem_Read(&hi2c1,dev_add,0xF7,1,&buffer[1],1,100);
HAL_I2C_Mem_Read(&hi2c1,dev_add,0xF8,1,&buffer[2],1,100);

UP=(buffer[0]<<16)+(buffer[1]<<8)+buffer[2];
UP=(UP>>(8-oss_set));

////////////////////////////////temprature
x1=((UT-ac6)*ac5)/pow(2,15);
x2=mc*pow(2,11)/(x1+md);
b5=x1+x2;
temp=(b5+8)/16;
//////////////////////////////////////////

b6=b5-4000;
x1=(b2*(b6*b6/pow(2,12)))/pow(2,11);
x2=ac2*b6/pow(2,11);
x3=x1+x2;
b3=(((ac1*4+x3)<<oss_set)+2)/4;
x1=ac3*b6/pow(2,13);
x2=(b1*(b6*b6/pow(2,12)))/pow(2,16);
x3=((x1+x2)+2)/4;
b4=ac4*(unsigned long)(x3+32768)/pow(2,15);
b7=((unsigned long)UP-b3)*(50000>>oss_set);
if(b7<0x80000000)
	pre=(b7*2)/b4;
else
	pre=(b7/b4)*2;
x1=(pre/pow(2,8))*(pre*pow(2,8));
x1=(x1*3038)/pow(2,16);
x2=(-7357*pre)/pow(2,16);
pre=pre+(x1+x2+3791)/16;
//////////////////////////////////////////

}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
