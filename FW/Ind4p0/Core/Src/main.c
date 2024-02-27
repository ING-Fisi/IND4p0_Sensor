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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "arm_math.h"
#include "stdbool.h"
#include "protocollo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static bool crc_tab16_init = false;
static uint16_t crc_tab16[256];
#define		CRC_START_16		0x0000
#define		CRC_POLY_16		0xA001

void send_config();
void receive_config(protocollo_uart pkt);
//void send_bump_alarm(uint8_t output_X,uint8_t output_Y,uint8_t output_Z);
void send_bump_alarm();
void send_shakes_count(uint16_t shakesCount,uint16_t XshakesCount,uint16_t YshakesCount,uint16_t ZshakesCount);
void send_raw_data();
//void send_alarm_disabled();
void send_ack();
void send_nack();
uint16_t crc_16( const unsigned char *input_str, size_t num_bytes );



void send_fft_sample();
void send_sample();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef ADC_Status;

/***********************************************
 * buffer dei parametri di configurazione
 **********************************************/
uint16_t sensor_configuration[7]={0};
/**********************************************/

/**************************************************
 * Soglie, shake count e sampling time di default
 **************************************************/
uint16_t bumpsThreshold = 200;
uint16_t shakesThreshold = 10;
uint16_t shakesSamplingTime = 5; // espresso in secondi
uint16_t shakesCount = 0;
/**************************************************/

/******************************************
 * Contatori di vibrazioni lungo gli assi
 ******************************************/
uint8_t xShakesCount = 0;
uint8_t yShakesCount = 0;
uint8_t zShakesCount = 0;
/******************************************/

/******************************************
 * Raw Data
 ******************************************/
float32_t output_X=0;
float32_t output_Y=0;
float32_t output_Z=0;
/******************************************/


/******************************************
 * UART verso PC
 ******************************************/
#define MAX_BUFFER_LENGHT 128
uint8_t UART1_rx;
uint8_t UART1_rxBuffer[MAX_BUFFER_LENGHT];
uint8_t UART1_txBuffer[MAX_BUFFER_LENGHT];
uint8_t rxbuffer_index = 0;

/******************************************/

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

//PUTCHAR_PROTOTYPE
//{
//	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//	return ch;
//}

uint32_t value[3];

//*****************************FIR ***************************//

#define SNR_THRESHOLD_F32    140.0f
#define BLOCK_SIZE            32
#define NUM_TAPS              96


arm_fir_instance_f32 S_FIR;
float32_t testInput_f32_1kHz_15kHz[SAMPLES];
//static float32_t firStateF32[SAMPLES + NUM_TAPS - 1];



const float32_t firCoeffs32[NUM_TAPS] = {
		-0.0045644380812448395,
		-0.0004239214773665623,
		-0.0003992160094902055,
		-0.0003410784893278679,
		-0.000246559170922285,
		-0.00011254431170462079,
		0.00006433750801042219,
		0.0002870933492719705,
		0.0005587009072068039,
		0.0008818359747837215,
		0.0012587683624311436,
		0.0016910582068537408,
		0.0021856908690502577,
		0.002726710900242062,
		0.0033400842043236102,
		0.0040111517348650795,
		0.004739077342511737,
		0.005524253099514465,
		0.0063660475475944564,
		0.007263122811066336,
		0.008212361291143213,
		0.009210018111155885,
		0.010251426837018239,
		0.011331469979433535,
		0.01244362364387908,
		0.013584873647048157,
		0.01474370531018102,
		0.015917964027590856,
		0.01709913746351596,
		0.01827867257835206,
		0.01944880098525925,
		0.020601194890077695,
		0.02172846305781673,
		0.022822065011098805,
		0.0238738289199096,
		0.02487547673817073,
		0.025819341620558257,
		0.026697029918866366,
		0.02750356254696736,
		0.028230666791627817,
		0.028873750993741547,
		0.029427571925871056,
		0.02988701980332222,
		0.030248790890860813,
		0.030508789150617766,
		0.030665772386255268,
		0.03071819701905401,
		0.030665772386255268,
		0.030508789150617766,
		0.030248790890860813,
		0.02988701980332222,
		0.029427571925871056,
		0.028873750993741547,
		0.028230666791627817,
		0.02750356254696736,
		0.026697029918866366,
		0.025819341620558257,
		0.02487547673817073,
		0.0238738289199096,
		0.022822065011098805,
		0.02172846305781673,
		0.020601194890077695,
		0.01944880098525925,
		0.01827867257835206,
		0.01709913746351596,
		0.015917964027590856,
		0.01474370531018102,
		0.013584873647048157,
		0.01244362364387908,
		0.011331469979433535,
		0.010251426837018239,
		0.009210018111155885,
		0.008212361291143213,
		0.007263122811066336,
		0.0063660475475944564,
		0.005524253099514465,
		0.004739077342511737,
		0.0040111517348650795,
		0.0033400842043236102,
		0.002726710900242062,
		0.0021856908690502577,
		0.0016910582068537408,
		0.0012587683624311436,
		0.0008818359747837215,
		0.0005587009072068039,
		0.0002870933492719705,
		0.00006433750801042219,
		-0.00011254431170462079,
		-0.000246559170922285,
		-0.0003410784893278679,
		-0.0003992160094902055,
		-0.0004239214773665623,
		-0.0045644380812448395


};


#define NUM_TAPS_HP              87
static float32_t firStateF32_HP[SAMPLES + NUM_TAPS_HP - 1];
const float32_t firCoeffs32_high_pass[NUM_TAPS_HP] = {
		0.00015046336434925082,
		-0.0003132108493542814,
		0.0005455831397464856,
		-0.0007655671960985814,
		0.0008479244457581014,
		-0.0006137937161025474,
		-0.00014459031918041517,
		0.001618510067585061,
		-0.003921817181404502,
		0.007026723577160095,
		-0.010714381679213151,
		0.014558817550571496,
		-0.017959185429479742,
		0.020226493978637464,
		-0.020716950964363334,
		0.018989175888080526,
		-0.014951023072293205,
		0.008956333659119901,
		-0.001815853588989312,
		-0.005298400010386114,
		0.01105255933477362,
		-0.014224672422497756,
		0.013999200142186967,
		-0.010213709039717932,
		0.003489753102984941,
		0.00480206065485691,
		-0.012759954435784393,
		0.018327027547382888,
		-0.019758735816211095,
		0.01608405227976113,
		-0.007451684217961991,
		-0.004733394280315384,
		0.01793991374342082,
		-0.02890817129034569,
		0.03425596442522116,
		-0.031179921743829084,
		0.018108140425716915,
		0.004840797750476119,
		-0.03570524556862057,
		0.07088775590886412,
		-0.10570111679267988,
		0.13515850329937282,
		-0.15485721750818826,
		0.1617808126828929,
		-0.15485721750818826,
		0.13515850329937282,
		-0.10570111679267988,
		0.07088775590886412,
		-0.03570524556862057,
		0.004840797750476119,
		0.018108140425716915,
		-0.031179921743829084,
		0.03425596442522116,
		-0.02890817129034569,
		0.01793991374342082,
		-0.004733394280315384,
		-0.007451684217961991,
		0.01608405227976113,
		-0.019758735816211095,
		0.018327027547382888,
		-0.012759954435784393,
		0.00480206065485691,
		0.003489753102984941,
		-0.010213709039717932,
		0.013999200142186967,
		-0.014224672422497756,
		0.01105255933477362,
		-0.005298400010386114,
		-0.001815853588989312,
		0.008956333659119901,
		-0.014951023072293205,
		0.018989175888080526,
		-0.020716950964363334,
		0.020226493978637464,
		-0.017959185429479742,
		0.014558817550571496,
		-0.010714381679213151,
		0.007026723577160095,
		-0.003921817181404502,
		0.001618510067585061,
		-0.00014459031918041517,
		-0.0006137937161025474,
		0.0008479244457581014,
		-0.0007655671960985814,
		0.0005455831397464856,
		-0.0003132108493542814,
		0.00015046336434925082


};


//*****************************FFT ***************************//

arm_cfft_radix4_instance_f32 S;	/* ARM CFFT module */
float32_t maxValue;				/* Max FFT value is stored here */
uint32_t maxIndex;				/* Index in Output array where max value is */
int current_index_aqsample = 0;
float32_t Input_X[SAMPLES];
float32_t Input_Y[SAMPLES];
float32_t Input_Z[SAMPLES];
//float32_t Output[FFT_SIZE];

//***************************************************************//

uint16_t ADC_RES = 0;
HAL_StatusTypeDef err_tx;

typedef struct adc_channel_struct
{
	uint32_t current_index;
	uint16_t ADC_X;
	uint16_t ADC_Y;
	uint16_t ADC_Z;
}adc_channel_struct;

adc_channel_struct adc_channel_instance =
{
		.current_index = 0,
		.ADC_X = 0,
		.ADC_Y = 0,
		.ADC_Z = 0,
};




void select_adc_channel(int channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	switch (channel)
	{
	case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = 1;

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;

	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = 1;

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;

	case 2:
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 4:
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 6:
		sConfig.Channel = ADC_CHANNEL_6;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 7:
		sConfig.Channel = ADC_CHANNEL_7;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 8:
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 9:
		sConfig.Channel = ADC_CHANNEL_9;
		sConfig.Rank = 9;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 10:
		sConfig.Channel = ADC_CHANNEL_10;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 11:
		sConfig.Channel = ADC_CHANNEL_11;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 12:
		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 13:
		sConfig.Channel = ADC_CHANNEL_13;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 14:
		sConfig.Channel = ADC_CHANNEL_14;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
	case 15:
		sConfig.Channel = ADC_CHANNEL_15;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;

	default:
		break;
	}
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//arm_fir_init_f32(&S_FIR, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], SAMPLES);
	arm_fir_init_f32(&S_FIR, NUM_TAPS_HP, (float32_t *)&firCoeffs32_high_pass[0], &firStateF32_HP[0], SAMPLES);


	/********************************************
	 * attiva l'UART in interrupt mode
	 ********************************************/
	HAL_UART_Receive_IT(&huart1,&UART1_rx, 1);

	/********************************************/
	//printf("HELLO WORLD\r\n");

	/*******************************************************
	 *Calibrate The ADC On Power-Up For Better Accuracy
	 *******************************************************/
	//HAL_ADCEx_Calibration_Start(&hadc1);
	/*******************************************************/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		select_adc_channel(0);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		value[0] = HAL_ADC_GetValue(&hadc1);

		select_adc_channel(1);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		value[1] = HAL_ADC_GetValue(&hadc1);

		select_adc_channel(2);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		value[2] = HAL_ADC_GetValue(&hadc1);
		//            HAL_ADC_Stop(&hadc1);



		/*******************************************************************************
		 * printf per SERIAL PLOT
		 *******************************************************************************/
		//printf("CH0 %d CH1 %d CH2 %d\r\n",value[0],value[1],value[2]);
		printf("%f %f %f\r\n",(float)value[0],(float)value[1],(float)value[2]);
		/*******************************************************************************/

		//************************************************************************************//

		adc_channel_instance.ADC_X = value[0];
		adc_channel_instance.ADC_Y = value[1];
		adc_channel_instance.ADC_Z = value[2];

		value[0] = 0;
		value[1] = 0;
		value[2] = 0;


		Input_X[current_index_aqsample] = adc_channel_instance.ADC_X;
		Input_Y[current_index_aqsample] = adc_channel_instance.ADC_Y;
		Input_Z[current_index_aqsample] = adc_channel_instance.ADC_Z;


		//HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

		//float32_t output_X;
		//float32_t output_Y;
		//float32_t output_Z;

		arm_fir_f32(&S_FIR,&Input_X[current_index_aqsample], &output_X, 1);
		arm_fir_f32(&S_FIR,&Input_Y[current_index_aqsample], &output_Y, 1);
		arm_fir_f32(&S_FIR,&Input_Z[current_index_aqsample], &output_Z, 1);

		/****************************************************************************************************************
		 * printf per SERIAL PLOT
		 ****************************************************************************************************************/
		//printf("%f %f %f %f %f %f\r\n",(float)value[0],(float)value[1],(float)value[2], output_X,output_Y,output_Z);
		/****************************************************************************************************************/

		/************************************************************************************************
		 * Se uno dei tre segnali supera la soglia degli urti viene trasmesso al SW l'allarme di urto
		 ************************************************************************************************/
		output_X = abs(output_X);
		output_Y = abs(output_Y);
		output_Z = abs(output_Z);

		if((output_X>bumpsThreshold)||(output_Y>bumpsThreshold)||(output_Z>bumpsThreshold)){
			//			send_bump_alarm((uint8_t)output_X, (uint8_t)output_Y, (uint8_t)output_Z);
			send_bump_alarm();
		}
		/************************************************************************************************/
		/******************************************************
		 * Se output_X supera la soglia delle vibrazioni
		 * incrementa il contatore di vibrazioni xShakesCount
		 ******************************************************/
		if(output_X>shakesThreshold){
			xShakesCount++;
		}
		/******************************************************/
		/******************************************************
		 * Se output_Y supera la soglia delle vibrazioni
		 * incrementa il contatore di vibrazioni yShakesCount
		 ******************************************************/
		if(output_Y>shakesThreshold){
			yShakesCount++;
		}
		/******************************************************/
		/******************************************************
		 * Se output_Z supera la soglia delle vibrazioni
		 * incrementa il contatore di vibrazioni zShakesCount
		 ******************************************************/
		if(output_Z>shakesThreshold){
			zShakesCount++;
		}
		/******************************************************/

		current_index_aqsample = (current_index_aqsample + 1) % SAMPLES;


		/*******************************************************
		 * Trasmette al SERIAL PLOTTER solo i tre segnali filtrati
		 *******************************************************/
			//printf("%f %f %f\r\n",Input_X,Input_Y,Input_Z);
		/*******************************************************/



		HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{

		/*****************************************************************************************/

		UART1_rxBuffer[rxbuffer_index] = UART1_rx;
		//		rxbuffer_index = (rxbuffer_index + 1) % MAX_BUFFER_LENGHT;
		rxbuffer_index = (rxbuffer_index + 1);


		// se è arrivato tutto il pacchetto entra nell'if
		if(UART1_rx == ETX_TAG)
		{

			int size_of_packet = sizeof(protocollo_uart);
			if(rxbuffer_index >= size_of_packet)
			{

				protocollo_uart dummy;
				memcpy(&dummy,UART1_rxBuffer,size_of_packet);

				if((dummy.stx == STX_TAG)&&(dummy.etx == ETX_TAG))
				{

					switch(dummy.cmd)
					{

					case 	shake_monitor_cmd:

						shakesCount = xShakesCount;
						if(yShakesCount > shakesCount) shakesCount = yShakesCount;
						if(zShakesCount > shakesCount) shakesCount = zShakesCount;

						send_shakes_count(shakesCount,xShakesCount,yShakesCount,zShakesCount);

						xShakesCount = 0;
						yShakesCount = 0;
						zShakesCount = 0;
						break;

					case 	reboot_cmd:
						break;

					case 	setConfiguration_cmd:
						receive_config(dummy);
						break;

					case 	saveConfiguration_cmd:
						receive_config(dummy);
						break;

					case 	getConfiguration_cmd:
						send_config();
						break;

					case 	getRawData_cmd:
						send_raw_data();
						break;

					//case    alarm_disabled_cmd:
					//	send_alarm_disabled();
					//	break;

					default:
						send_nack();
						break;

					}

				}

			}

			/**************************************************************
			 * azzera il buffer UART1_rxBuffer e l'indice rxbuffer_index
			 **************************************************************/
			memset(UART1_rxBuffer,0,sizeof(UART1_rxBuffer));
			rxbuffer_index = 0;
			/**************************************************************/

		}

		//uint8_t pippo = UART1_rx;
		//HAL_UART_Transmit(&huart1, &pippo, 1, 100);

		/********************************************
		 * attiva l'UART in interrupt mode
		 ********************************************/
		HAL_UART_Receive_IT(&huart1,&UART1_rx, 1);
		/********************************************/

		/*****************************************************************************************/

	}

}

/************************************************************************************************
 * Invia i parametri di configurazione all'interfaccia SW
 ************************************************************************************************/
void send_config(){

	protocollo_uart send_pkt = {0};
	send_pkt.stx = STX_TAG;
	send_pkt.cmd = getConfiguration_cmd;
	send_pkt.payload[0] = bumpsThreshold;
	send_pkt.payload[1] = shakesThreshold;
	send_pkt.payload[2] = shakesSamplingTime;
	send_pkt.payload[3] = shakesCount;
	send_pkt.payload[4] = xShakesCount;
	send_pkt.payload[5] = yShakesCount;
	send_pkt.payload[6] = zShakesCount;
	send_pkt.payload[7] = output_X;
	send_pkt.payload[8] = output_Y;
	send_pkt.payload[9] = output_Z;
	send_pkt.crc = crc_16((unsigned char *)&send_pkt.payload,sizeof(send_pkt.payload));
	send_pkt.etx = ETX_TAG;

	int size_of_packet = sizeof(protocollo_uart);
	memset(UART1_txBuffer,0,MAX_BUFFER_LENGHT);
	memcpy(UART1_txBuffer,(uint8_t*)&send_pkt,size_of_packet);
	//HAL_UART_Transmit_IT(&huart1, UART1_txBuffer, size_of_packet);
	HAL_UART_Transmit(&huart1, UART1_txBuffer, size_of_packet, 100);
}
/************************************************************************************************/

/************************************************
 * Riceve i parametri di configurazione
 * dall'interfaccia SW
 ************************************************/
void receive_config(protocollo_uart pkt){

	bumpsThreshold = pkt.payload[0];
	shakesThreshold = pkt.payload[1];
	shakesSamplingTime = pkt.payload[2];
	//shakesCount = pkt.payload[3];

}
/************************************************/
/************************************************************************************************
 * Invia all'interfaccia SW il comando di allarme per urto
 ************************************************************************************************/
//void send_bump_alarm(uint8_t output_X,uint8_t output_Y,uint8_t output_Z){
void send_bump_alarm(){
	protocollo_uart send_pkt = {0};
	send_pkt.stx = STX_TAG;
	send_pkt.cmd = bump_event_cmd;
	send_pkt.payload[0] = bumpsThreshold;
	send_pkt.payload[1] = shakesThreshold;
	send_pkt.payload[2] = shakesSamplingTime;
	send_pkt.payload[3] = shakesCount;
	send_pkt.payload[4] = xShakesCount; //xShakesCount;  output_X
	send_pkt.payload[5] = yShakesCount; //yShakesCount;  output_Y
	send_pkt.payload[6] = zShakesCount; //zShakesCount;  output_Z
	send_pkt.payload[7] = output_X;
	send_pkt.payload[8] = output_Y;
	send_pkt.payload[9] = output_Z;
	send_pkt.crc = crc_16((unsigned char *)&send_pkt.payload,sizeof(send_pkt.payload));
	send_pkt.etx = ETX_TAG;

	int size_of_packet = sizeof(protocollo_uart);
	memset(UART1_txBuffer,0,MAX_BUFFER_LENGHT);
	memcpy(UART1_txBuffer,(uint8_t*)&send_pkt,size_of_packet);
	//HAL_UART_Transmit_IT(&huart1, UART1_txBuffer, size_of_packet);
	HAL_UART_Transmit(&huart1, UART1_txBuffer, size_of_packet, 100);
}
/************************************************************************************************/

/************************************************************************************************
 * Invia all'interfaccia SW il comando di allarme per vibrazioni
 ************************************************************************************************/
void send_shakes_count(uint16_t shakesCount, uint16_t XshakesCount, uint16_t YshakesCount, uint16_t ZshakesCount){
	protocollo_uart send_pkt = {0};
	send_pkt.stx = STX_TAG;
	send_pkt.cmd = shake_event_cmd;
	send_pkt.payload[0] = bumpsThreshold;
	send_pkt.payload[1] = shakesThreshold;
	send_pkt.payload[2] = shakesSamplingTime;
	send_pkt.payload[3] = shakesCount;
	send_pkt.payload[4] = XshakesCount;
	send_pkt.payload[5] = YshakesCount;
	send_pkt.payload[6] = ZshakesCount;
	send_pkt.payload[7] = output_X;
	send_pkt.payload[8] = output_Y;
	send_pkt.payload[9] = output_Z;
	send_pkt.crc = crc_16((unsigned char *)&send_pkt.payload,sizeof(send_pkt.payload));
	send_pkt.etx = ETX_TAG;

	int size_of_packet = sizeof(protocollo_uart);
	memset(UART1_txBuffer,0,MAX_BUFFER_LENGHT);
	memcpy(UART1_txBuffer,(uint8_t*)&send_pkt,size_of_packet);
	//HAL_UART_Transmit_IT(&huart1, UART1_txBuffer, size_of_packet);
	HAL_UART_Transmit(&huart1, UART1_txBuffer, size_of_packet, 100);
}
/************************************************************************************************/

/************************************************************************************************
 * Invia all'interfaccia SW i dati grezzi
 ************************************************************************************************/
void send_raw_data(){
	protocollo_uart send_pkt = {0};
	send_pkt.stx = STX_TAG;
	send_pkt.cmd = getRawData_cmd;
	send_pkt.payload[0] = bumpsThreshold;
	send_pkt.payload[1] = shakesThreshold;
	send_pkt.payload[2] = shakesSamplingTime;
	send_pkt.payload[3] = shakesCount;
	send_pkt.payload[4] = xShakesCount; //xShakesCount;  output_X
	send_pkt.payload[5] = yShakesCount; //yShakesCount;  output_Y
	send_pkt.payload[6] = zShakesCount; //zShakesCount;  output_Z
	send_pkt.payload[7] = output_X;
	send_pkt.payload[8] = output_Y;
	send_pkt.payload[9] = output_Z;
	send_pkt.crc = crc_16((unsigned char *)&send_pkt.payload,sizeof(send_pkt.payload));
	send_pkt.etx = ETX_TAG;

	int size_of_packet = sizeof(protocollo_uart);
	memset(UART1_txBuffer,0,MAX_BUFFER_LENGHT);
	memcpy(UART1_txBuffer,(uint8_t*)&send_pkt,size_of_packet);
	//HAL_UART_Transmit_IT(&huart1, UART1_txBuffer, size_of_packet);
	HAL_UART_Transmit(&huart1, UART1_txBuffer, size_of_packet, 100);
}
/************************************************************************************************/


/************************************************************************************************
 * Invia all'interfaccia SW il comando di allarme disattivato
 ************************************************************************************************/
//void send_alarm_disabled(){
//
//}
/************************************************************************************************/


void send_nack(){

}


/************************************************************************************
 * inizializzazione crc16
 ************************************************************************************/
static void init_crc16_tab( void ) {

	uint16_t i;
	uint16_t j;
	uint16_t crc;
	uint16_t c;

	for (i=0; i<256; i++) {

		crc = 0;
		c   = i;

		for (j=0; j<8; j++) {

			if ( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ CRC_POLY_16;
			else                      crc =   crc >> 1;

			c = c >> 1;
		}

		crc_tab16[i] = crc;
	}

	crc_tab16_init = true;

}  /* init_crc16_tab */
/************************************************************************************/

/*********************************************************************************
 * Calcolo crc 16
 *********************************************************************************/
uint16_t crc_16( const unsigned char *input_str, size_t num_bytes ) {

	uint16_t crc;
	const unsigned char *ptr;
	size_t a;

	if ( ! crc_tab16_init ) init_crc16_tab();

	crc = CRC_START_16;
	ptr = input_str;

	if ( ptr != NULL ) for (a=0; a<num_bytes; a++) {

		crc = (crc >> 8) ^ crc_tab16[ (crc ^ (uint16_t) *ptr++) & 0x00FF ];
	}

	return crc;

}  /* crc_16 */
/*********************************************************************************/


void send_sample()
{


}


void send_fft_sample()
{


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
