/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "uip.h"
#include "uip_arp.h"
#include "enc28j60.h"
#include "hello-world.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

uint8_t transmitBuffer1[10];

uint8_t transmitBuffer2[10];

uint8_t transmitBuffer3[10];



char out_string[100] = "";
uint8_t in_string[100];

uint16_t error = 0;
uint8_t data_rx;
uint8_t data_tx;
uint16_t delay_arp=0;
uint16_t i_up=0;
uint16_t temp1 = 0;
uint8_t target1 = 0x00;
uint8_t target2 = 0x00;
uint8_t target3 = 0x00;
uint8_t block = 0;
uint8_t tormoz1 = 0;
uint8_t tormoz2 = 0;
uint8_t tormoz3 = 0;
uint8_t temp_p1 =0;
uint8_t temp_p2 =1;
uint8_t temp_hal =1;
uint16_t count_usart = 0;
uint8_t count_chastotniki = 1;
uint8_t position1 = 0;
uint8_t position2 = 0;
uint8_t position3 = 0;

uint8_t Skorost1 = 0;
uint8_t Skorost2 = 0;
uint8_t Skorost3 = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t SPI1_TXRX (uint8_t data)
{
data_tx = data;
HAL_SPI_TransmitReceive(&hspi1, &data_tx, &data_rx, 1, 10);
return data_rx;
}
void Ethernet (void)
{
delay_arp++;
for(i_up = 0;i_up<UIP_CONNS; i_up++){uip_periodic(i_up);if(uip_len>0){uip_arp_out();enc28j60_send_packet((uint8_t *)uip_buf, uip_len);}}
#if UIP_UDP
for(i_up=0;i_up<UIP_UDP_CONNS;i_up++){uip_udp_periodic(i_up);if(uip_len>0){uip_arp_out();network_send();}}
#endif /* UIP_UDP */
if(delay_arp>=50){delay_arp = 0;uip_arp_timer();}
uip_len = enc28j60_recv_packet((uint8_t *) uip_buf, UIP_BUFSIZE);
if(uip_len>0){if(BUF->type==htons(UIP_ETHTYPE_IP)){uip_arp_ipin();uip_input();if(uip_len>0){uip_arp_out();enc28j60_send_packet((uint8_t *)uip_buf,uip_len);}
}else if(BUF->type == htons(UIP_ETHTYPE_ARP)){uip_arp_arpin();if(uip_len>0){enc28j60_send_packet((uint8_t *)uip_buf,uip_len);}}}
}

char ObratnoePreobrazovanie(uint8_t temp)
{
char temp2 = '0';
switch (temp)
														{ case 0: temp2='0'; break; case 1: temp2='1'; break;
															case 2: temp2='2'; break; case 3: temp2='3'; break;
															case 4: temp2='4'; break; case 5: temp2='5'; break;
												      case 6: temp2='6'; break; case 7: temp2='7'; break;
															case 8: temp2='8'; break; case 9: temp2='9'; break;
															default:temp2='0'; break;
														}
														return temp2 ;
}


char Preobrazovanie_int_to_char_1_znak(uint16_t temp)
{
char temp1='0';


if(temp>10000){temp=10000;}
if(temp>=10000){temp=temp-10000;}
if(temp>=9000){temp=temp-9000;}
if(temp>=8000){temp=temp-8000;}
if(temp>=7000){temp=temp-7000;}
if(temp>=6000){temp=temp-6000;}
if(temp>=5000){temp=temp-5000;}
if(temp>=4000){temp=temp-4000;}
if(temp>=3000){temp=temp-3000;}
if(temp>=2000){temp=temp-2000;}
if(temp>=1000){temp=temp-1000;}
if(temp>=900){temp=temp-900;}
if(temp>=800){temp=temp-800;}
if(temp>=700){temp=temp-700;}
if(temp>=600){temp=temp-600;}
if(temp>=500){temp=temp-500;}
if(temp>=400){temp=temp-400;}
if(temp>=300){temp=temp-300;}
if(temp>=200){temp=temp-200;}
if(temp>=100){temp=temp-100;}

if (temp>=90 && temp<100){temp1 = ObratnoePreobrazovanie(temp-90);}
if (temp>=80 && temp<90) {temp1 = ObratnoePreobrazovanie(temp-80);}
if (temp>=70 && temp<80) {temp1 = ObratnoePreobrazovanie(temp-70);}
if (temp>=60 && temp<70) {temp1 = ObratnoePreobrazovanie(temp-60);}
if (temp>=50 && temp<60) {temp1 = ObratnoePreobrazovanie(temp-50);}
if (temp>=40 && temp<50) {temp1 = ObratnoePreobrazovanie(temp-40);}
if (temp>=30 && temp<40) {temp1 = ObratnoePreobrazovanie(temp-30);}
if (temp>=20 && temp<30) {temp1 = ObratnoePreobrazovanie(temp-20);}
if (temp>=10 && temp<20) {temp1 = ObratnoePreobrazovanie(temp-10);}
if (            temp<10) {temp1 = ObratnoePreobrazovanie(temp);   }
if (temp>99)             {temp1 = '0';}
return temp1;
}

char Preobrazovanie_int_to_char_2_znak(uint16_t temp)
{
char temp1='0';
if(temp>10000){temp=10000;}
if(temp>=10000){temp=temp-10000;}
if(temp>=9000){temp=temp-9000;}
if(temp>=8000){temp=temp-8000;}
if(temp>=7000){temp=temp-7000;}
if(temp>=6000){temp=temp-6000;}
if(temp>=5000){temp=temp-5000;}
if(temp>=4000){temp=temp-4000;}
if(temp>=3000){temp=temp-3000;}
if(temp>=2000){temp=temp-2000;}
if(temp>=1000){temp=temp-1000;}
if(temp>=900){temp=temp-900;}
if(temp>=800){temp=temp-800;}
if(temp>=700){temp=temp-700;}
if(temp>=600){temp=temp-600;}
if(temp>=500){temp=temp-500;}
if(temp>=400){temp=temp-400;}
if(temp>=300){temp=temp-300;}
if(temp>=200){temp=temp-200;}
if(temp>=100){temp=temp-100;}

if (temp>=90 && temp<100){temp1 = '9';}
if (temp>=80 && temp<90) {temp1 = '8';}
if (temp>=70 && temp<80) {temp1 = '7';}
if (temp>=60 && temp<70) {temp1 = '6';}
if (temp>=50 && temp<60) {temp1 = '5';}
if (temp>=40 && temp<50) {temp1 = '4';}
if (temp>=30 && temp<40) {temp1 = '3';}
if (temp>=20 && temp<30) {temp1 = '2';}
if (temp>=10 && temp<20) {temp1 = '1';}
if (            temp<10) {temp1 = '0';}
if (temp>99)             {temp1 = '0';}
return temp1;
}

char Preobrazovanie_int_to_char_3_znak(uint16_t temp)
{
char temp1='0';

if(temp>10000){temp=10000;}
if(temp>=10000){temp=temp-10000;}
if(temp>=9000){temp=temp-9000;}
if(temp>=8000){temp=temp-8000;}
if(temp>=7000){temp=temp-7000;}
if(temp>=6000){temp=temp-6000;}
if(temp>=5000){temp=temp-5000;}
if(temp>=4000){temp=temp-4000;}
if(temp>=3000){temp=temp-3000;}
if(temp>=2000){temp=temp-2000;}
if(temp>=1000){temp=temp-1000;}


if (temp>=900 && temp<1000) {temp1 = '9';}
if (temp>=800 && temp<900)  {temp1 = '8';}
if (temp>=700 && temp<800)  {temp1 = '7';}
if (temp>=600 && temp<700)  {temp1 = '6';}
if (temp>=500 && temp<600)  {temp1 = '5';}
if (temp>=400 && temp<500)  {temp1 = '4';}
if (temp>=300 && temp<400)  {temp1 = '3';}
if (temp>=200 && temp<300)  {temp1 = '2';}
if (temp>=100 && temp<200)  {temp1 = '1';}
if (              temp<100) {temp1 = '0';}
if (temp>999)               {temp1 = '0';}
return temp1;
}

char Preobrazovanie_int_to_char_4_znak(uint16_t temp)
{
char temp1='0';

if (temp>=9000 && temp<10000){temp1 = '9';}
if (temp>=8000 && temp<9000) {temp1 = '8';}
if (temp>=7000 && temp<8000) {temp1 = '7';}
if (temp>=6000 && temp<7000) {temp1 = '6';}
if (temp>=5000 && temp<6000) {temp1 = '5';}
if (temp>=4000 && temp<5000) {temp1 = '4';}
if (temp>=3000 && temp<4000) {temp1 = '3';}
if (temp>=2000 && temp<3000) {temp1 = '2';}
if (temp>=1000 && temp<2000) {temp1 = '1';}
if (              temp<1000) {temp1 = '0';}
if (temp>9999)               {temp1 = '0';}
return temp1;
}


void usart (void)
{
	if(count_usart >= 100){

	MX_USART1_UART_Init();

	if (block == 0){
		if (count_chastotniki==1){HAL_UART_Transmit_IT(&huart1, transmitBuffer1, 10);}
		if (count_chastotniki==2){HAL_UART_Transmit_IT(&huart1, transmitBuffer2, 10);}
		if (count_chastotniki==3){HAL_UART_Transmit_IT(&huart1, transmitBuffer3, 10);}
		if (count_chastotniki>=3){count_chastotniki=1;}else{count_chastotniki++;}

   }
	count_usart=0;
	}else{count_usart++;}
}



uint16_t  V_converchion (uint8_t temp)
{
	uint16_t a1 = 0;
	if (temp == 0){a1 = 0;}
	  if (temp == 1){a1 = 0x1388;}
		  if (temp == 2){a1 = 0x2710;}
			  if (temp == 3){a1 = 0x3A98;}
				  if (temp == 4){a1 = 0x4E20;}
					  if (temp == 5){a1 = 0x61A8;}
						  if (temp == 6){a1 = 0x7530;}
	return a1;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim7);


  struct uip_eth_addr mac = { {0x54,0x55,0x58,0x17,0x11,0x29} };
	enc28j60_init(mac.addr);

	uip_init();
	uip_arp_init();
	hello_world_init();
	uip_setethaddr(mac);

	uip_ipaddr_t ipaddr;
	uip_ipaddr(ipaddr, 192, 168, 0, 171);
	uip_sethostaddr(ipaddr);
	uip_ipaddr(ipaddr, 192, 168, 0, 1);
	uip_setdraddr(ipaddr);
	uip_ipaddr(ipaddr, 255, 255, 255, 0);
	uip_setnetmask(ipaddr);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */

   block = in_string[0];
   target1 = (in_string[1]*100 + in_string[2]*10 + in_string[3]); if (target1>180){target1=180;}
	 target2 = (in_string[4]*100 + in_string[5]*10 + in_string[6]); if (target2>180){target2=180;}
	 target3 = (in_string[7]*100 + in_string[8]*10 + in_string[9]); if (target3>180){target3=180;}
	 Skorost1 = in_string[10];if (Skorost1>6){Skorost1=6;}
	 Skorost2 = in_string[11];if (Skorost2>6){Skorost2=6;}
	 Skorost3 = in_string[12];if (Skorost3>6){Skorost3=6;}


   out_string[0] = '0';
   out_string[1] = '7';
   out_string[2] = '0';
   out_string[3] = '7';

  transmitBuffer1[0]=0x02; //признак сообщения как передача
	transmitBuffer1[1]=0x01; // адрес привода
	transmitBuffer1[2]=0x05; // тип сообщения
	transmitBuffer1[3]=0x1D;
	if (block == 1)	{transmitBuffer1[4]=0x06;} else {transmitBuffer1[4]=0x01;}
	transmitBuffer1[5]=(V_converchion(Skorost1)) >> 8; // скорость
	transmitBuffer1[6]=(V_converchion(Skorost1)) & 255; // скорость
	transmitBuffer1[7]=(target1>>4); // положение
	transmitBuffer1[8]=(target1<<4); // положение
	transmitBuffer1[9]=transmitBuffer1[0] ^ transmitBuffer1[1] ^ transmitBuffer1[2] ^ transmitBuffer1[3] ^ transmitBuffer1[4] ^ transmitBuffer1[5] ^ transmitBuffer1[6] ^ transmitBuffer1[7] ^ transmitBuffer1[8];

	transmitBuffer2[0]=0x02; //признак сообщения как передача
	transmitBuffer2[1]=0x02; // адрес привода
	transmitBuffer2[2]=0x05; // тип сообщения - не меняем
	transmitBuffer2[3]=0x1D; // не меняем
 	if (block == 1)	{transmitBuffer2[4]=0x06;} else {transmitBuffer2[4]=0x01;}
	transmitBuffer2[5]=(V_converchion(Skorost2)) >> 8; // скорость
	transmitBuffer2[6]=(V_converchion(Skorost2)) & 255; // скорость
	transmitBuffer2[7]=(target2>>4); // положение
	transmitBuffer2[8]=(target2<<4); // положение
	transmitBuffer2[9]=transmitBuffer2[0] ^ transmitBuffer2[1] ^ transmitBuffer2[2] ^ transmitBuffer2[3] ^ transmitBuffer2[4] ^ transmitBuffer2[5] ^ transmitBuffer2[6] ^ transmitBuffer2[7] ^ transmitBuffer2[8];

  transmitBuffer3[0]=0x02; //признак сообщения как передача
	transmitBuffer3[1]=0x03; // адрес привода
	transmitBuffer3[2]=0x05; // тип сообщения
	transmitBuffer3[3]=0x1D;
  if (block == 1)	{transmitBuffer3[4]=0x06;} else {transmitBuffer3[4]=0x01;}
	transmitBuffer3[5]=(V_converchion(Skorost3)) >> 8; // скорость
	transmitBuffer3[6]=(V_converchion(Skorost3)) & 255; // скорость
	transmitBuffer3[7]=(target3>>4); // положение
	transmitBuffer3[8]=(target3<<4); // положение
	transmitBuffer3[9]=transmitBuffer3[0] ^ transmitBuffer3[1] ^ transmitBuffer3[2] ^ transmitBuffer3[3] ^ transmitBuffer3[4] ^ transmitBuffer3[5] ^ transmitBuffer3[6] ^ transmitBuffer3[7] ^ transmitBuffer3[8];

	if (block == 1){
	MX_USART1_UART_Init();
  HAL_UART_Transmit_IT(&huart1, transmitBuffer1, 10);
	HAL_Delay(35);

	MX_USART1_UART_Init();
	HAL_UART_Transmit_IT(&huart1, transmitBuffer2, 10);
 	HAL_Delay(35);

	MX_USART1_UART_Init();
	HAL_UART_Transmit_IT(&huart1, transmitBuffer3, 10);
  HAL_Delay(35);
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

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
