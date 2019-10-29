
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */

char END_Address = 'E';

/* END ------------------------------------------------------------------*/

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "SD.h"
#include "ff_gen_drv.h"
#include "user_diskio.h"
#include "stdio.h"
#include "math.h"
#include "stdbool.h"
#include "sd_hal_mpu6050.h"

#define CS_SD_LOW() 			HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_RESET)
#define CS_SD_HIGH() 			HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_SET)

#define XBEE_Sleep() 			  HAL_GPIO_WritePin(Xbee_Sleep_Pin_GPIO_Port, Xbee_Sleep_Pin_Pin, GPIO_PIN_SET)


// DS1037 ---------------------------------------------------------//

	// Slave address for DS1307 
	
	#define  DS1307_DEVICE_ADDRESS	0x68
	#define  DS1307_DEVICE_CONTROL	0x07
	#define	 DS1307_Seconds					0x00


	// Configurar Data e Hora	
	
	#define SET_DS1307_Segundos		30
	#define SET_DS1307_Minutos		00
	#define SET_DS1307_Hora				12
	
	#define SET_DS1307_DayOfWeek 	1
	
	#define SET_DS1307_Dia				01
	#define SET_DS1307_Mes				01
	#define SET_DS1307_Ano				19
		

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/


	// --------------------------- Inicializacao -----------------------------------
	
		bool Inicializacao = true;
		
	// DS1037 ---------------------------------------------------------//
	
	uint8_t Buffer[9];


	// ----------------------------------- MPU -------------------------------------
		
		SD_MPU6050 mpu1;
		float temper;
	  int16_t g_x; 
	  int16_t g_y;
	  int16_t g_z;
	  int16_t a_x;
	  int16_t a_y;
	  int16_t a_z;
		const float alpha = 0.5;
		const float PI = 3.1415;
		double fXg = 0;
		double fYg = 0;
		double fZg = 0;
		double roll,pitch;
		double error, erroa, I,Ia, P, D, setpoint;
		double kp,ki,kd, PID,max,min;
		int parar;
		double saida;
		int sinal;
		double erro[7],erro1[7],erroM;
		int graus;
		float pitchAcc;
		float delta_t,var;
		float Q_angle,Q_bias,R_measure,angle,bias,Pk[2][2];
		float Q_angleY,Q_biasY,R_measureY,angleY,biasY,PkY[2][2];
		float gyroXrate;
		float gyroYrate,gyroYangle;
		float compAngleY;
		float Kalmans_AngleX =0,Kalmans_AngleY =0;
		float INIT_AngleX =0,INIT_AngleY =0;
		float MPU_X =0,MPU_Y =0;
		float Kalmans_RED_CODE_X=0;
		float Kalmans_RED_CODE_Y=0;
		
	// ----------------------------------- uSD -------------------------------------

		FATFS 			SDFatFs;
		FATFS 			*fs;
		FIL 				MyFile;
		FRESULT     res;
		char 				USER_Path[4]; /* logical drive path */
		//uint8_t			sprintAUX[60] = {0};
		char				sprintAUX[60];
	// ----------------------------------- UART 1 ----------------------------------
		
		char UART_Tx_Angle[19];
		char UART_Tx_ADC[20];
		char RED_CODE_FLAG = 'N';
		bool TIM3_IT = false;
		char SOL_Placa = 'S';
		uint8_t TIM3_Counter = 0;
		
	// ----------------------------------- ADC -------------------------------------

		uint16_t adcResult =0;
		float Tensao_Placa =0;
		
	// ----------------------------------- RTC -------------------------------------		
		
		uint8_t RTC_Seconds = 0;
		
		
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

	#ifdef _GNUC_
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* _GNUC_ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE  
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */

HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
			
    return ch;
}

FRESULT open_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
	){
	
	
	    FRESULT fr;
		FRESULT aux;

    /* Opens an existing file. If not exist, creates a new file. */
		
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        aux = f_lseek(fp, f_size(fp));
        if (aux != FR_OK)
            f_close(fp);
    }
    return fr;
}

	void SD_Backup(uint8_t Buf[9], float D1, float D2){
		 			
		disk_initialize(SDFatFs.drv);
					
				if(SD_Init() != 0){
					// ERROR					
				}
				else{					
					if(f_mount(&SDFatFs,(const char*)USER_Path,0)!= FR_OK){
								// ERROR
					}
					else{
								
						if(open_append(&MyFile,"BackUP.txt") != FR_OK){						
								// ERROR																
					}
							else{														
								
								sprintf(sprintAUX,"%c%c/%c%c/%c%c %c%c:%c%c:%c%c Angulo X: %3.1f ; Angulo Y: %3.1f \n",((Buf[4]>>4)+48),((Buf[4]&0x0F)+48),((Buf[5]>>4)+48),((Buf[5]&0x0F)+48),((Buf[6]>>4)+48),((Buf[6]&0x0F)+48)
																							,((Buf[2]>>4)+48),((Buf[2]&0x0F)+48),((Buf[1]>>4)+48),((Buf[1]&0x0F)+48),((Buf[0]>>4)+48),((Buf[0]&0x0F)+48),D1,D2);
								f_printf(&MyFile,sprintAUX);
											
								f_close(&MyFile);
							 
					}					 
				}
			}							
				FATFS_UnLinkDriver(USER_Path);
				
	}

	void GET_MPU_Data(void){	
	
		SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
	  g_x = mpu1.Gyroscope_X;
	  g_y = mpu1.Gyroscope_Y;
	  g_z = mpu1.Gyroscope_Z;

	  SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
	  a_x = mpu1.Accelerometer_X;
	  a_y = mpu1.Accelerometer_Y;
	  a_z = mpu1.Accelerometer_Z;
	
	}
	void GET_Kalmans_Parameters(void){	
				
			delta_t = var/10000;    // Conta o tempo ate medir novamente (usado no filtro kalman)
			var = 0;
		
			pitch = atan2(a_y, a_z + abs(a_x)) * 180/PI;		// Angulo giroscopio
			roll = atan2(a_x, a_z + abs(a_y)) * 180/PI;
		
			gyroYrate = g_x/131.0;
			gyroXrate = g_y/131.0;
		
	
	}
	float Kalmans_Filter_X(float newAngle, float newRate, float dt) {
		
		
		float rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    Pk[0][0] += dt * (dt*Pk[1][1] - Pk[0][1] - Pk[1][0] + Q_angle);
    Pk[0][1] -= dt * Pk[1][1];
    Pk[1][0] -= dt * Pk[1][1];
    Pk[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = Pk[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = Pk[0][0] / S;
    K[1] = Pk[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = Pk[0][0];
    float P01_temp = Pk[0][1];

    Pk[0][0] -= K[0] * P00_temp;
    Pk[0][1] -= K[0] * P01_temp;
    Pk[1][0] -= K[1] * P00_temp;
    Pk[1][1] -= K[1] * P01_temp;

    return angle;	
}
	
	float Kalmans_Filter_Y(float newAngle, float newRate, float dt) {
		
		// Q_angleY,Q_biasY,R_measureY,angleY,biasY,PkY[2][2];
		
		float rate = newRate - biasY;
    angleY += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    PkY[0][0] += dt * (dt*PkY[1][1] - PkY[0][1] - PkY[1][0] + Q_angleY);
    PkY[0][1] -= dt * PkY[1][1];
    PkY[1][0] -= dt * PkY[1][1];
    PkY[1][1] += Q_biasY * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = PkY[0][0] + R_measureY; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = PkY[0][0] / S;
    K[1] = PkY[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angleY; // Angle difference
    /* Step 6 */
    angleY += K[0] * y;
    biasY += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = PkY[0][0];
    float P01_temp = PkY[0][1];

    PkY[0][0] -= K[0] * P00_temp;
    PkY[0][1] -= K[0] * P01_temp;
    PkY[1][0] -= K[1] * P00_temp;
    PkY[1][1] -= K[1] * P01_temp;

    return angleY;	
}

	
	void XBEE_Tx(void){	
		
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port,LED_Out_Pin);
		
			
		//sprintf(UART_Tx_Angle,"A%c%c X:%5.1f Y:%5.1f",SOL_Placa,RED_CODE_FLAG,(Kalmans_AngleX),(Kalmans_AngleY));
		 sprintf(UART_Tx_Angle,"%c%c%c X:%5.1f Y:%5.1f",END_Address,SOL_Placa,RED_CODE_FLAG,(Kalmans_AngleX),(Kalmans_AngleY));
			printf(UART_Tx_Angle);		
					
	}
		
	char Verificar_SOL(void){	
		
			HAL_ADC_Start(&hadc1); 
			HAL_ADC_PollForConversion(&hadc1, 200); 
			adcResult = HAL_ADC_GetValue(&hadc1);
			Tensao_Placa = adcResult * (3.3/4095);
			HAL_ADC_Stop(&hadc1);
			
			if(Tensao_Placa > 1){
				
				return 'S';				
			}
			else{
				
				return 'N';
			}
					
	}
		

	void XBEE_WakeUp(void){	
		
		HAL_GPIO_WritePin(Xbee_Sleep_Pin_GPIO_Port, Xbee_Sleep_Pin_Pin, GPIO_PIN_RESET);
		HAL_Delay(600);
	}
	
	void Init_MPU_TIM3_IT(void){	
		
		if(Inicializacao){
		int i =0;
		for(i=0;i<200;i++){
			
			GET_MPU_Data();	
			GET_Kalmans_Parameters();
			Kalmans_Filter_X(pitch,gyroYrate,delta_t);
			Kalmans_Filter_Y(roll,gyroXrate,delta_t);
		}
		
		for(i=0;i<10;i++){
			
			GET_MPU_Data();	
			GET_Kalmans_Parameters();	
			
			MPU_X = MPU_X + Kalmans_Filter_X(pitch,gyroYrate,delta_t);			
			MPU_Y = MPU_Y + Kalmans_Filter_Y(roll,gyroXrate,delta_t);
		}		
		
			INIT_AngleX = MPU_X/10;		
			INIT_AngleY = MPU_Y/10;		
		
		HAL_TIM_Base_Start_IT(&htim3);
		Inicializacao = false;
		
	}		
}
	
	void RED_CODE(float Delta_XY){	
		
		if ((fabs(Kalmans_AngleX) - fabs(Kalmans_RED_CODE_X)) > Delta_XY || (fabs(Kalmans_AngleY) - fabs(Kalmans_RED_CODE_Y)) > Delta_XY )
		{	
			HAL_TIM_Base_Stop_IT(&htim3);
			RED_CODE_FLAG = 'S';
			XBEE_WakeUp();
			XBEE_Tx();			
			XBEE_Sleep();
			RED_CODE_FLAG = 'N';
			HAL_TIM_Base_Start_IT(&htim3);
		}
						
	}
	
	uint8_t bcd2bin(uint8_t bcd){
		uint8_t bin = (bcd >> 4) * 10;
		bin += bcd & 0x0F;
		
		return bin;
	}

	uint8_t bin2bcd(uint8_t bin){
		uint8_t high = bin / 10;
		uint8_t low = bin - (high *10);
		
		return (high << 4) | low;
	}		
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
	
	
	

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	SD_MPU6050_Result Result ;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start_IT(&htim2);
	
	XBEE_WakeUp();
	
	//Kalman variables X ------------
	
	  Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Reset the angle
    bias = 0.0f;  // Reset bias

    Pk[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    Pk[0][1] = 0.0f;
    Pk[1][0] = 0.0f;
    Pk[1][1] = 0.0f;
		
		//Kalman variables Y ------------
	
	  Q_angleY = 0.001f;
    Q_biasY = 0.003f;
    R_measureY = 0.03f;

    angleY = 0.0f; // Reset the angle
    biasY = 0.0f;  // Reset bias

    PkY[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    PkY[0][1] = 0.0f;
    PkY[1][0] = 0.0f;
    PkY[1][1] = 0.0f;
		
		
		HAL_GPIO_WritePin(LED_Out_GPIO_Port,LED_Out_Pin,GPIO_PIN_RESET);
		
		// --------------------------- Inicializacao MPU--------------------------------
		
	  Result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
		HAL_Delay(200);
		
		
		// --------------------------- Inicializacao DS1307 ----------------------------
		
		Buffer[0] = DS1307_Seconds;
		Buffer[1] = bin2bcd(SET_DS1307_Segundos);
		Buffer[2] = bin2bcd(SET_DS1307_Minutos);
		Buffer[3] = bin2bcd(SET_DS1307_Hora);
		Buffer[4] = bin2bcd(SET_DS1307_DayOfWeek);
		Buffer[5] = bin2bcd(SET_DS1307_Dia);
		Buffer[6] = bin2bcd(SET_DS1307_Mes);
		Buffer[7] = bin2bcd(SET_DS1307_Ano);
		Buffer[8] = bin2bcd(0x00);			
		
	  HAL_I2C_Master_Transmit(&hi2c2,(DS1307_DEVICE_ADDRESS << 1),Buffer,9,100);					
		HAL_Delay(100);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
			
		if((Result != SD_MPU6050_Result_Ok)){
				
				HAL_GPIO_WritePin(LED_Out_GPIO_Port,LED_Out_Pin,GPIO_PIN_SET);
				HAL_Delay(600);
				NVIC_SystemReset();
			
			}
			
		Init_MPU_TIM3_IT();
		
		
		if(TIM3_IT){				
						
		  GET_MPU_Data();	
			GET_Kalmans_Parameters();		
			
			Kalmans_AngleX = INIT_AngleX - Kalmans_Filter_X(pitch,gyroYrate,delta_t);			
			Kalmans_AngleY = INIT_AngleY - Kalmans_Filter_Y(roll,gyroXrate,delta_t);					
			SOL_Placa = Verificar_SOL();
						
			RED_CODE(10);
			
			Kalmans_RED_CODE_X = Kalmans_AngleX;
			Kalmans_RED_CODE_Y = Kalmans_AngleY;
			
			TIM3_IT = false;
			TIM3_Counter++;	// incremento a cada 500 ms			
		}		
		
		if(TIM3_Counter > 6){
			
			Buffer[0] = DS1307_Seconds;
			HAL_I2C_Master_Transmit(&hi2c2,(DS1307_DEVICE_ADDRESS << 1),Buffer,1,100);
			HAL_I2C_Master_Receive(&hi2c2,(DS1307_DEVICE_ADDRESS << 1)+1,Buffer,8,100);
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port,LED_Out_Pin);
			TIM3_Counter = 0;
			XBEE_WakeUp();
			XBEE_Tx();			
			XBEE_Sleep();
			
//			HAL_TIM_Base_Stop_IT(&htim2);
//			HAL_TIM_Base_Stop_IT(&htim3);
			HAL_Delay(10);
		  SD_Backup(Buffer,Kalmans_AngleX,Kalmans_AngleY);
//			HAL_TIM_Base_Start_IT(&htim2);
//			HAL_TIM_Base_Start_IT(&htim3);			
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|Xbee_Sleep_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Out_Pin */
  GPIO_InitStruct.Pin = LED_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SD_Pin */
  GPIO_InitStruct.Pin = CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Xbee_Sleep_Pin_Pin */
  GPIO_InitStruct.Pin = Xbee_Sleep_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Xbee_Sleep_Pin_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM2)
		var++;
		
	if (htim->Instance==TIM3)		
		TIM3_IT = true;
		
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
