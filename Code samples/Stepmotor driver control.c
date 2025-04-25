/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define Strelka_up_shag_on                       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
#define Strelka_up_shag_off                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
#define Strelka_up_right                         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
#define Strelka_up_left                    	     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

#define Strelka_down_shag_on                  	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
#define Strelka_down_shag_off                 	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
#define Strelka_down_right                     	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
#define Strelka_down_left                    	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);



uint32_t kolvo_shag_down		=	0;
uint8_t 	     flag_down 		= 0;
uint8_t       speed_down 		= 5;
uint8_t speed_count_down 		= 0;

uint32_t kolvo_shag_up 			=	0;
uint8_t        flag_up 			= 0;
uint8_t       speed_up 			= 5;
uint8_t speed_count_up 			= 0;

uint8_t   flag_step_init    = 0;
uint8_t   invert = 0;


uint32_t ADC_value = 0;

uint16_t i=0;
uint32_t temp_adc = 0;
uint32_t adc_summ = 0;
uint16_t count=0;

uint16_t Sterlka_up=0;
uint16_t Sterlka_down=0;
uint16_t Sterlka_up_can=0;
uint16_t Sterlka_down_can=0;
uint16_t		abs_up_steep=0;
uint16_t    abs_down_steep=0;
uint16_t Strel_up[1001];
uint16_t Strel_down[1001];
uint8_t flag_tormoz=0;
uint8_t count_podsvetka=0;
uint16_t Shim = 5;
uint32_t svitch = 0;
uint64_t counter_volt=0;
uint32_t Voltage=0;
uint8_t flag_reset=0;

uint32_t adc_result = 0;

int tim7_znach = 70;


static CanTxMsgTypeDef can1TxMessage;
static CanRxMsgTypeDef can1RxMessage;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t Svitch ( void )
 {
	uint16_t svitch_temp = 0;
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)==1) {svitch_temp = svitch_temp << 1;} else {svitch_temp = (svitch_temp + 1) << 1;}
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)==1) {svitch_temp = svitch_temp << 1;} else {svitch_temp = (svitch_temp + 1) << 1;}
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)==1) {svitch_temp = svitch_temp << 1;} else {svitch_temp = (svitch_temp + 1) << 1;}
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)==1) {svitch_temp = svitch_temp << 1;} else {svitch_temp = (svitch_temp + 1) << 1;}
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1) {} else {svitch_temp = svitch_temp + 1;}
	return svitch_temp;
 }

void Podsvetka( void )
 {
	  if (count_podsvetka >= 63){count_podsvetka=0;}
    if (Shim <= count_podsvetka){HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);}
		if (Shim > count_podsvetka) {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);}
	  count_podsvetka++;
 }


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{

	if (flag_reset == 0){
  Sterlka_up_can = (can1RxMessage.Data[0] << 8) + can1RxMessage.Data[1] ;
	if (Sterlka_up_can > 999){Sterlka_up_can = 999;}
	Sterlka_down_can = (can1RxMessage.Data[2] << 8) + can1RxMessage.Data[3];
	if (Sterlka_down_can > 999){Sterlka_down_can = 999;}
	Shim = can1RxMessage.Data[4];
  }


	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

}
  void Stepdriver (void)
	{
if (kolvo_shag_up != 0)
{
if (speed_up <= speed_count_up){
if (kolvo_shag_up >= 5){
if (flag_up == 0){Strelka_up_shag_on; flag_up = 1; speed_count_up = 0;}
   else {Strelka_up_shag_off;
	       flag_up = 0;
		     if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)==0){abs_up_steep++;kolvo_shag_up--;}else{abs_up_steep--;kolvo_shag_up--;}
      	 speed_count_up = 0;
      	 if (speed_up>0){speed_up--;}
	      }
                        }
else {
if (flag_up == 0){Strelka_up_shag_on; flag_up = 1; speed_count_up = 0;}
   else {Strelka_up_shag_off;
      	 flag_up = 0;
	       if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)==0){abs_up_steep++;kolvo_shag_up--;}else{abs_up_steep--;kolvo_shag_up--;}
	       speed_count_up = 0;
	       if (speed_up<5){speed_up++;}
	      }
     }
}else {speed_count_up++;}
}
// ---------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
if (kolvo_shag_down != 0)
{
if (speed_down <= speed_count_down){
if (kolvo_shag_down >= 5){
if (flag_down == 0){Strelka_down_shag_on; flag_down = 1; speed_count_down = 0;}
   else {Strelka_down_shag_off;
	       flag_down = 0;
		     if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0){abs_down_steep++;kolvo_shag_down--;}else{abs_down_steep--;kolvo_shag_down--;}
      	 speed_count_down = 0;
      	 if (speed_down>0){speed_down--;}
	      }
                        }
else {
if (flag_down == 0){Strelka_down_shag_on; flag_down = 1; speed_count_down = 0;}
   else {Strelka_down_shag_off;
      	 flag_down = 0;
	       if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0){abs_down_steep++;kolvo_shag_down--;}else{abs_down_steep--;kolvo_shag_down--;}
	       speed_count_down = 0;
	       if (speed_down<5){speed_down++;}
	      }
     }
}else {speed_count_down++;}
}
//---------------------------------------------------------------------------------------------------
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	svitch = (Svitch() + 32) << 5;


	hcan.pRxMsg = &can1RxMessage;
	hcan.pTxMsg = &can1TxMessage;

  CAN_FilterConfTypeDef canFilterConfig;
  canFilterConfig.FilterNumber = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  canFilterConfig.FilterIdHigh = svitch;
  canFilterConfig.FilterIdLow  = svitch;
  canFilterConfig.FilterMaskIdHigh = 0 ;
  canFilterConfig.FilterMaskIdLow =  0 ;
  canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.BankNumber = 0;
  HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);

  HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	Strelka_up_shag_off;
	Strelka_down_shag_off;

	Strelka_up_left;
  kolvo_shag_up	=1000;

	Strelka_down_left;
  kolvo_shag_down	=1000;



  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	if (flag_step_init==2 && counter_volt == 0 ){Voltage = adc_result; counter_volt = 1;}


	if ((adc_result < ((Voltage*100)/115)) && (flag_step_init==2)){

		Shim=0;
		if (flag_reset == 0){
		flag_reset=1;
		HAL_TIM_Base_Stop_IT(&htim7);
		tim7_znach = 20;
		MX_TIM7_Init();
		HAL_TIM_Base_Start_IT(&htim7);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		Sterlka_down_can=0; Sterlka_up_can=0;}
	}else{
	if (flag_reset==1){ HAL_TIM_Base_Stop_IT(&htim7); tim7_znach = 120; MX_TIM7_Init(); HAL_TIM_Base_Start_IT(&htim7);HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);Shim=5;
	}
	flag_reset=0;
	}

///----------------------------------------------------------------------

	if(flag_step_init==0 && kolvo_shag_up==0 && kolvo_shag_down==0 && flag_reset==0)
	 {
	  Strelka_up_right;
		kolvo_shag_up	= 105;   //  верхняя стрелка значение калибровки на 0

	  Strelka_down_right;
		kolvo_shag_down	= 110; //  нижняя стрелка значение на 0

		for(i=0;i<1001;i++){
		Strel_up[i]=(3250*i/1000);
		Strel_down[i]=(3250*i/1000);
	  }

		flag_step_init=1;
	 }
	 if (flag_step_init==1 && kolvo_shag_up==0 && kolvo_shag_down==0 && flag_reset==0){
	  abs_up_steep=0;
    abs_down_steep=0;
		Sterlka_up=0;
		Sterlka_down=0;
    flag_step_init=2;
    tim7_znach = 120;
    MX_TIM7_Init();
		HAL_TIM_Base_Start_IT(&htim7);
	 }
///----------------------------------------------------------------------------------------------------

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  adc_result = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);

///----------------------------------------------------------------------------------------------------

if ((Sterlka_up != Sterlka_up_can) && (flag_step_init==2))
		{  if (abs_up_steep < Strel_up[Sterlka_up_can])
						{
	            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)==1 && kolvo_shag_up != 0)
								{
									if (kolvo_shag_up>5)
									   {
											 kolvo_shag_up = 5;
									   }
								}else
								{
								Strelka_up_right;
								kolvo_shag_up =  Strel_up[Sterlka_up_can] - abs_up_steep;
								Sterlka_up = Sterlka_up_can;
								}
						}else
						{
              if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)==0 & kolvo_shag_up != 0){
									if (kolvo_shag_up>5)
									   {
											 kolvo_shag_up = 5;
									   }
								}else
								{
								Strelka_up_left;
								kolvo_shag_up =  abs_up_steep - Strel_up[Sterlka_up_can] ;
								Sterlka_up = Sterlka_up_can;
						  	}
						}
  }

if ((Sterlka_down != Sterlka_down_can) && (flag_step_init==2))
		{  if (abs_down_steep < Strel_down[Sterlka_down_can])
						{
	            if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==1 && kolvo_shag_down != 0)
								{
									if (kolvo_shag_down>5)
									   {
											 kolvo_shag_down = 5;
									   }
								}else
								{
								Strelka_down_right;
								kolvo_shag_down =  Strel_down[Sterlka_down_can] - abs_down_steep;
								Sterlka_down = Sterlka_down_can;
								}
						}else
						{
              if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0 && kolvo_shag_down != 0){
									if (kolvo_shag_down>5)
									   {
											 kolvo_shag_down = 5;
									   }
								}else
								{
								Strelka_down_left;
								kolvo_shag_down =  abs_down_steep - Strel_down[Sterlka_down_can] ;
								Sterlka_down = Sterlka_down_can;
						  	}
						}
  }







  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
