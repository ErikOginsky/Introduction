/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static CanTxMsgTypeDef can1TxMessage;
static CanRxMsgTypeDef can1RxMessage;

uint8_t flag_resive_can = 0;
uint8_t Kod1=0 ;
uint8_t Kod2=0 ;
uint8_t Kod3=0 ;


uint8_t   Pot1 = 0;
uint8_t   Pot2 = 0;
uint8_t 	Pot3 = 0;
uint8_t   Pot4 = 0;


uint32_t ADC_value[4];

uint16_t adc1[250];
uint16_t adc2[250];
uint16_t adc3[250];
uint16_t adc4[250];


uint8_t   button_1    =  0;
uint8_t   button_2    =  0;
uint8_t   button_3    =  0;
uint8_t   button_4    =  0;
uint8_t   button_5    =  0;
uint8_t   button_6    =  0;
uint8_t   button_7    =  0;
uint8_t   button_8    =  0;
uint8_t   button_9    =  0;
uint8_t   button_10   =  0;
uint8_t   button_11   =  0;
uint8_t   button_12   =  0;
uint8_t   button_13   =  0;
uint8_t   button_14   =  0;
uint8_t   button_15   =  0;
uint8_t   button_16   =  0;
uint8_t   button_17   =  0;
uint8_t   button_18   =  0;
uint8_t   button_19 	=  0;
uint8_t   button_20 	=  0;
uint8_t   button_21   =  0;


uint8_t   led_1       =  0;
uint8_t   Shim_White  =  0;
uint8_t   Shim_Green  =  0;
uint8_t   led_2       =  0;
uint8_t   led_3       =  0;
uint8_t   shim_count  =  0;
uint8_t   shim_bd  =  0;

uint16_t   count        =0;


uint16_t  can_error   =  0;
uint8_t   flag_can     = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void TransmitCAN (uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8)
{
    can1TxMessage.StdId=1;
    can1TxMessage.DLC = 8;
    can1TxMessage.Data[0] = a1;
    can1TxMessage.Data[1] = a2;
    can1TxMessage.Data[2] = a3;
    can1TxMessage.Data[3] = a4;
    can1TxMessage.Data[4] = a5;
    can1TxMessage.Data[5] = a6;
    can1TxMessage.Data[6] = a7;
    can1TxMessage.Data[7] = a8;

    hcan.pTxMsg = &can1TxMessage;
    HAL_CAN_Transmit_IT(&hcan);
}

void SHIM ( void )
{
    if (shim_count >= 100){shim_count=0;}

    if (Shim_White <= shim_count)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    }

    if (Shim_Green <= shim_count)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    }


    if (shim_bd <= shim_count)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);}
        else{
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        }


        shim_count++;
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{
    led_1 		 = CanHandle->pRxMsg->Data[0];
    led_2 		 = CanHandle->pRxMsg->Data[1];
    led_3 		 = CanHandle->pRxMsg->Data[2];
    //flag_can = 1;
    can_error = 0;
    TransmitCAN(Kod1,Kod2,Kod3,Pot4,Pot2,Pot3,Pot1,0);flag_can =0;
    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

}


void CAN (void)
{
    //	if (flag_can == 1){TransmitCAN(Kod1,Kod2,Kod3,Pot4,Pot2,Pot3,Pot1,0);flag_can =0;}
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

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();

    /* USER CODE BEGIN 2 */


    hcan.pRxMsg = &can1RxMessage;
    hcan.pTxMsg = &can1TxMessage;

    CAN_FilterConfTypeDef canFilterConfig;
    canFilterConfig.FilterNumber = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    canFilterConfig.FilterIdHigh = 2 << 5;
    canFilterConfig.FilterIdLow  = 2 << 5;
    canFilterConfig.FilterMaskIdHigh = 0 ;
    canFilterConfig.FilterMaskIdLow =  0 ;
    canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.BankNumber = 0;
    HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_value,4);

    HAL_TIM_Base_Start_IT(&htim6);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        can_error++;
        if (can_error >= 1000)
        {
            MX_CAN_Init();
            CAN_FilterConfTypeDef canFilterConfig;
            canFilterConfig.FilterNumber = 0;
            canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
            canFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
            canFilterConfig.FilterIdHigh = 2 << 5;
            canFilterConfig.FilterIdLow  = 2 << 5;
            canFilterConfig.FilterMaskIdHigh = 0 ;
            canFilterConfig.FilterMaskIdLow =  0 ;
            canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
            canFilterConfig.FilterActivation = ENABLE;
            canFilterConfig.BankNumber = 0;
            HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
            HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
            can_error = 0;
        }

        //------------------------------------------------------------------------------------
        adc1[249] = (ADC_value[0] >> 4);
        uint8_t temp = 0 ;
        uint32_t adc_summ=0;
        uint8_t i=0;
        i=0;

        while(i<249)
        {
            adc1[i] = adc1[i+1];
            adc_summ = adc_summ + adc1[i];
            i++;
        }

        temp = adc_summ / 249;
        if (Pot1 != temp){if ( Pot1 < temp ){Pot1++;} else {if (Pot1 > temp ){Pot1--;}}};
        //--------------------------------------------------------------------------------------
        adc2[249] = (ADC_value[1] >> 4);
        temp = 0 ;
        adc_summ=0;
        i=0;

        while(i<249)
        {
            adc2[i] = adc2[i+1];
            adc_summ = adc_summ + adc2[i];
            i++;
        }

        temp = adc_summ / 249;
        if (Pot2 != temp){if ( Pot2 < temp ){Pot2++;} else {if (Pot2 > temp ){Pot2--;}}};
        //------------------------------------------------------------------------------------
        adc3[249] = (ADC_value[2] >> 4);
        temp = 0 ;
        adc_summ=0;
        i=0;

        while(i<249)
        {
            adc3[i] = adc3[i+1];
            adc_summ = adc_summ + adc3[i];
            i++;
        }

        temp = adc_summ / 249;
        if (Pot3 != temp){if ( Pot3 < temp ){Pot3++;} else {if (Pot3 > temp ){Pot3--;}}};
        //-------------------------------------------------------------------------------
        adc4[249] = (ADC_value[3] >> 4);
        temp = 0 ;
        adc_summ=0;
        i=0;

        while(i<249)
        {
            adc4[i] = adc4[i+1];
            adc_summ = adc_summ + adc4[i];
            i++;
        }

        temp = adc_summ / 249;
        if (Pot4 != temp){if ( Pot4 < temp ){Pot4++;} else {if (Pot4 > temp ){Pot4--;}}};

        //---------------------------------------------------------------------------------

        uint8_t temp1 = 0;

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)==0){button_1 = 1;}else{button_1 = 0;}
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)==0){button_2 = 1;}else{button_2 = 0;}
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)==0){button_3 = 1;}else{button_3 = 0;}
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)==0){button_4 = 1;}else{button_4 = 0;}
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)==0){button_6 = 1;}else{button_6 = 0;}
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==0){button_7 = 1;}else{button_7 = 0;}
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==0){button_8 = 1;}else{button_8 = 0;}
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==0){button_9 = 1;}else{button_9 = 0;}

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)==0){button_5 = 1;}else{button_5 = 0;}
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==0){button_10 = 1;}else{button_10 = 0;}
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)==0){button_11 = 1;}else{button_11 = 0;}
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)==0){button_12 = 1;}else{button_12 = 0;}
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)==0){button_13 = 1;}else{button_13 = 0;}
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==0){button_14 = 1;}else{button_14 = 0;}
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==0){button_15 = 1;}else{button_15 = 0;}
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==0){button_16 = 1;}else{button_16 = 0;}

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==0){button_17 = 1;}else{button_17 = 0;}
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)==0){button_18 = 1;}else{button_18 = 0;}
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==0){button_19 = 1;}else{button_19 = 0;}
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)==0){button_20 = 1;}else{button_20 = 0;}
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==0){button_21 = 1;}else{button_21 = 0;}


        temp1=0;
        temp1 = temp1 + (button_1 << 7);
        temp1 = temp1 + (button_2 << 6);
        temp1 = temp1 + (button_3 << 5);
        temp1 = temp1 + (button_4 << 4);
        temp1 = temp1 + (button_6 << 3);
        temp1 = temp1 + (button_7 << 2);
        temp1 = temp1 + (button_8 << 1);
        temp1 = temp1 +  button_9;
        Kod1 = temp1;

        temp1=0;
        temp1 = temp1 + (button_5 << 7);
        temp1 = temp1 + (button_10 << 6);
        temp1 = temp1 + (button_11 << 5);
        temp1 = temp1 + (button_12 << 4);
        temp1 = temp1 + (button_13 << 3);
        temp1 = temp1 + (button_14 << 2);
        temp1 = temp1 + (button_15 << 1);
        temp1 = temp1 +  button_16;
        Kod2 = temp1;

        temp1=0;
        temp1 = temp1 + (button_17 << 7);
        temp1 = temp1 + (button_18 << 6);
        temp1 = temp1 + (button_19 << 5);
        temp1 = temp1 + (button_20 << 4);
        temp1 = temp1 + (button_21 << 3);

        Kod3 = temp1;


        if(led_2 != 0){HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);}
        if(led_3 != 0){HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);}

        if(button_11 == 1 && button_12 == 0){Shim_White = 40; Shim_Green = 0;}
        if(button_11 == 0 && button_12 == 1){Shim_White = 20; Shim_Green = 0;}
        if(button_11 == 0 && button_12 == 0){Shim_White = 0; Shim_Green = 0;}

        if(button_13 == 1){Shim_Green = 40;}
        if(button_13 == 0 && button_11 == 0 && button_12 == 0){Shim_Green = 0;}
        if(button_13 == 1 && button_11 == 1 && button_12 == 0){Shim_Green = 0;}
        if(button_13 == 1 && button_11 == 0 && button_12 == 1){Shim_Green = 0;}


        if(led_1 == 1 && button_19 == 1) {shim_bd = 0;}
        if(led_1 == 1 && button_19 == 0) {shim_bd = 70;}
        if(led_1 == 0) {shim_bd = 100;}
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
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
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
     *    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
