
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE BEGIN Includes */
#include "socket.h"								 						      // Include one header for WIZCHIP
#include <bit.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//------------------------------------------------------------------------------
#define DATA_BUF_SIZE   		2048
#define SOCK_TCPS        		0
#define SOCK_UDPS        		1
extern uint8_t gDATABUF			[DATA_BUF_SIZE];
int32_t ret 					= 0;
uint8_t i          				= 0;
//------------------------------------------------------------------------------
uint8_t In_StrLen  				= 127;
uint16_t InString   			 [128];

uint8_t Out_SetLen 				= 127;
uint16_t OutString  			 [128];
//------------------------------------------------------------------------------

//------------------ CAN Variables ---------------------------------------------
    static CanTxMsgTypeDef can1TxMessage;
    static CanRxMsgTypeDef can1RxMessage;
//------------------ CAN Variables ---------------------------------------------

//-----------------Defined_variables--------------------------------------------
		uint8_t receiveBuffer     [2];
		uint8_t Last_Address_CAN  = 0;
		uint8_t can_count         = 0;
		uint8_t can_address       = 2;
//-----------------Defined_variables--------------------------------------------

//------------- переменные кнопок головного контроллера А1 ---------------------
		uint8_t   napravl_kontr    = 0;
		uint8_t   polojenie_kontr  = 0;
		uint8_t   kalorifer  	   = 0;
//----------------- Переменные контроллера машиниста----------------------------
		uint8_t temp 			    = 0;
        uint8_t temp_last		  	= 0;
		uint16_t key          		= 0;
		
		uint8_t temp_1 				= 0;
		uint8_t temp_last_1	  		= 0;
		uint16_t key_1        		= 0;
//------------------------- Определение переменных -----------------------------
		uint8_t receiveBuffer     	[2];
        uint8_t  len_rx          	= 0;
        uint8_t  schetchik_RX    	= 0;   																				// Счетчик входящего сообщения
		uint8_t  last_schetchik_RX  = 0; 																				// Счетчик предыдущего входящего сообщения
        uint8_t  buffer_TX			[256];       																		// Буфер исходящих сообщений
        uint8_t  rx_complite     	= 0;   																				// Создаем флаг принятого и обработанного сообщения /0 - нет обработанных сообщений / 1 - сообщение обработано
        uint8_t  tx_ready        	= 0;   																				// Создаем флаг готовности исходящего сообщения /0 - нет сообщения /1 - сообщение подготовлено
//------===----- переменные кнопок головного контроллера А1 --------------------
        uint8_t main_button_1 		= 0;  																				// РБ
		uint8_t main_button_2 		= 0;  																				// педаль 1
		uint8_t main_button_3 		= 0;
        uint8_t A1_button1 			= 0;       																		    // управление общее
		uint8_t A1_button2 			= 0;        																	    // топливный насос. Выкл при Авар пит дизеля
		uint8_t A1_button3 			= 0;        																	    // калорифер больше
		uint8_t A1_button4 			= 0;        																	    // калорифер меньше
        uint8_t A1_button5 			= 0; 	     																		// реверсивка вперёд
		uint8_t A1_button6 			= 0;        																	    // реверсивка назад
        uint8_t A1_button7 			= 0;        																	    // контактор1
        uint8_t A1_button8 			= 0;        																	    // контактор2
        uint8_t A1_button9 			= 0;        																	    // контактор3
        uint8_t A1_button10 		= 0;       																		    // контактор4
        uint8_t A1_button11 		= 0;       																		    // контактор5
//--------------- переменные переферийного контроллера А3 ----------------------
        uint8_t A3_button_1 		= 0;   																				// Буферные фонари передние левые — Белый
		uint8_t A3_button_2 		= 0;   																				// Буферные фонари передние левые — Красный
		uint8_t A3_button_3 		= 0;   																				// Буферные фонари передние правые — Белый
		uint8_t A3_button_4 		= 0;   																				// Буферные фонари передние правые — Красный
		uint8_t A3_button_5 		= 0;   																				// Буферные фонари задние левые — Белый
		uint8_t A3_button_6 	    = 0;   																				// Буферные фонари задние левые — Красный
		uint8_t A3_button_7 		= 0;   																				// Буферные фонари задние правые — Белый
		uint8_t A3_button_8 		= 0;   																				// Буферные фонари задние правые — Красный
		uint8_t A3_button_9 		= 0;   																				// радиостанция
		uint8_t A3_button_10 		= 0;  																				// зелёным светом
		uint8_t A3_button_11 		= 0;  																				// автоматическое управлением холодильника
		uint8_t A3_button_12 		= 0;  																				// подрамное
		uint8_t A3_button_13 		= 0;  																				// освещение пульта
		uint8_t A3_button_14 		= 0;  																				// мегометр 1
        uint8_t A3_button_15 		= 0;  																				// мегометр 2
        uint8_t A3_button_16 		= 0;  																				// питание приборов
        uint8_t A3_button_17 		= 0;  																				// масляный насос
        uint8_t A3_button_18 		= 0;  																				// стоп дизеля2
        uint8_t A3_button_19 		= 0;  																				//
        uint8_t A3_button_20 		= 0;  																				// управление машинами
        uint8_t A3_button_21 		= 0;  																				// управление переходами
		uint8_t A3_button_22 		= 0;  																				//	топливный насос
		uint8_t A3_button_23 		= 0;	 																			// пуск-остонов дизеля
		uint8_t A3_button_24 		= 0;	 																			// проворот вала дизеля
		uint8_t A3_button_25 		= 0;  																				//
//--------------- переменные переферийного контроллера А4 ----------------------
        uint8_t A4_button_1 		= 0;   																			    // жалюзи воды воздухаохладителя
		uint8_t A4_button_2 		= 0;   																		        // жалюзи верхние
		uint8_t A4_button_3 		= 0;   																				// жалюзи масла
		uint8_t A4_button_4 		= 0;   																				// жалюзи воды
		uint8_t A4_button_5 		= 0;   																				// автосцепка передняя
		uint8_t A4_button_6 		= 0;   																				// муфта вентилятора
		uint8_t A4_button_7 		= 0;   																				// белый огонь
		uint8_t A4_button_8 		= 0;   																				// прожектор ярко
		uint8_t A4_button_9 		= 0;   																				// прожектор тускло
		uint8_t A4_button_10 		= 0;  																				// автосцепка задняя
        uint8_t A4_button_11 		= 0;  																				// вызов помощника
		uint8_t A4_button_12 		= 0;  																				// питание АЛСН
		uint8_t A4_button_13 		= 0;  																				// бдительность без АЛСН
		uint8_t A4_button_14 		= 0;  																				// проверка АЛСН
		uint8_t A4_button_15 		= 0;  																				// выключитель реле РУ14
		uint8_t A4_button_16 		= 0;  																				// переключатель частоты 75Гц
		uint8_t A4_button_17 		= 0;  																				// переключатель частоты 25Гц
		uint8_t A4_button_18 		= 0;  																				//	световой номер
        uint8_t A4_button_19 		= 0;  																				//	возврат реле заземления
        uint8_t A4_button_20 		= 0;  																				// антиоблединитель
		uint8_t A4_button_21 		= 0;  																				// проверка пожарной сигнализации
		uint8_t A4_button_22 		= 0;  																				// включение бдительности
		uint8_t A4_button_23 		= 0;  																				// вентиль 395 2х ход
		uint8_t A4_button_24 		= 0;  																				// вентиль 395 3х ход
		uint8_t A4_button_25 		= 0;  																				//
//-------------переменные переферийного контроллера А5--------------------------
        uint8_t A5_button_1 		= 0;                                                                                // сигнальные контрольные приборы
		uint8_t A5_button_2 		= 0;                                                                                // световые приборы
		uint8_t A5_button_3 		= 0;                                                                                // дежурное освещение 1
		uint8_t A5_button_4 		= 0;                                                                                // дежурное освещение 2
		uint8_t A5_button_5 		= 0;                                                                                // вентиляторы кабины
		uint8_t A5_button_6 		= 0;                                                                                // прожектор
		uint8_t A5_button_7 		= 0;                                                                                // масляный насос
		uint8_t A5_button_8 		= 0;                                                                                // откл. Моторв I
		uint8_t A5_button_9 		= 0;                                                                                // откл. Моторв II
        uint8_t A5_button_10 		= 0;                                                                                // откл. Моторв откл
		uint8_t A5_button_11 		= 0;                                                                                // откл. Моторв I-II
		uint8_t A5_button_12 		= 0;                                                                                // откл. Моторв реостат
		uint8_t A5_button_13		= 0;                                                                                // стеклоочиститель 1
		uint8_t A5_button_14 		= 0;                                                                                // стеклоочиститель 2
		uint8_t A5_button_15 		= 0;                                                                                // стеклоочиститель 3
		uint8_t A5_button_16 		= 0;                                                                                //
		uint8_t A5_button_17 		= 0;                                                                                //
        uint8_t A5_button_18 		= 0;                                                                                //
        uint8_t A5_button_19 	    = 0;                                                                                // ЭПК
		uint8_t A5_button_20 		= 0;                                                                                // РБС
		uint8_t A5_button_21 		= 0;                                                                                // тифон
		uint8_t A5_button_22 		= 0;                                                                                // свисток
		uint8_t A5_button_23 		= 0;                                                                                //
		uint8_t A5_button_24 		= 0;                                                                                //
		uint8_t A5_button_25 		= 0;                                                                                //
////------------------- вывод допольнительный устройств ------------------------
    uint8_t cran_395                = 0;		                                                                        // 395 кран (0-XX)
    uint8_t cran_254                = 0;		                                                                        // 254 кран (0-XX)
    uint8_t control_naprav          = 0;                                                                                // Направление тяги на контроллере машиниста
    uint8_t skorostimer             = 0;		                                                                        // skorostimer
//------------------------------------------------------------------------------
    uint16_t  count      = 0;
    uint16_t  can_error  = 0;                                                                                // Ошибки CAN-шины
    uint8_t   flag_can   = 0;
		
    uint16_t  pribor_1   = 0;  // Ток зарядки батареи 150-0-150А М42300
    uint16_t  pribor_2   = 0;  // напряжение вспом генератора 0-1kV М42300
    uint16_t  pribor_3   = 0;  // Нагрузка генератора 0-6kA М42300
    uint16_t  pribor_4   = 0;  // напряжение генератора
//-------------------------------------------------------------------------------
    uint8_t led_1		 = 0;   // ПСС левая
    uint8_t led_2		 = 0;	 // ПСС правая
    uint8_t led_3		 = 0;	 // температура масло 84 градус
    uint8_t led_4		 = 0;	 // СН1
    uint8_t led_5		 = 0;	 // СН2
    uint8_t led_6		 = 0;	 // Д2
    uint8_t led_7		 = 0;	 // вентилятор
    uint8_t led_8		 = 0;	 // пожарная станция
		
    uint8_t led_11 		  = 0;	 // зелёный
    uint8_t led_12		  = 0;	 // жёлтый
    uint8_t led_13		  = 0;	 // жёлтый-красный
    uint8_t led_14		  =	0;	 // красный
		uint8_t led_15	  = 0;	 // белый
//    uint8_t svetofor		=	0;   //ALSN
//--------------------------- приборы шаговые -----------------------------------
    uint16_t step1       = 0;	 //  воздух элекроаппаратов      (УД800)
    uint16_t step2       = 0;	 //  топливо                     (УД800)
    uint16_t step3       = 0;	 //  Масло                       (УД800)
    uint16_t step4       = 0;	 //  t* Масло до холодильника    (ТУЭ8)
    uint16_t step5       = 0;	 //  t* Вода                     (ТУЭ8)
    uint16_t step6       = 0;	 //  t* Вода   воздухаохладителя (ТУЭ8)
    uint16_t step7       = 0;	 //  воздух питательной сети     (манометр)
    uint16_t step8       = 0;	 //  Тормозные цилиндры          (манометр)
    uint16_t step9       = 0;	 //  Уравнительный резервуары    (манометр)
    uint16_t step10      = 0;	 //  Питательная магистраль      (манометр) 
    uint16_t step11      = 0;	 //  скорость                    (скоростимер) 
// -------------------------------------------------------------------------------	
		uint8_t  shim_pribor = 20;			   // переменная подсветки шаговых приборов
//    uint8_t  shim_svet   = 64;			 // переменная освещения кабины
//    uint8_t controller   = 0;
    uint8_t EPT = 0;  
    uint8_t A2_kod_1 = 0;
    uint8_t A2_kod_2 = 0;
    uint8_t A2_kod_3 = 0;  
//    uint8_t controller_naprav = 0;    
    
    uint8_t cran_254_average = 0;
//_____________________________________    
    uint8_t buff_left_front = 0;                 // Буферные фонари передние левые — Белый/Красный
    uint8_t buff_right_front = 0;                // Буферные фонари передние правые — Белый/Красный
    uint8_t buff_left_rear = 0;                  // Буферные фонари задние левые — Белый/Красный
    uint8_t buff_right_rear = 0;	             	// Буферные фонари задние праве — Белый/Красный
//______________________________________  
    uint8_t megometr = 0;     // мегометр
    uint8_t chastota = 0;		  // частота 75/25 Гц
		uint8_t motors = 0;       // моторы
    uint8_t stekloochist = 0;	// стеклоочиститель
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

//---------------Provide capabilities to W5500 Library--------------------------
void  wizchip_select(void)			//Enable W5500 by SCNn=0 signal
{
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}
void  wizchip_deselect(void)		//Disable W5500 by SCNn=1 signal
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}
void  wizchip_write(uint8_t b)	//Transmit byte via SPI1
{
 HAL_SPI_Transmit(&hspi1, &b, 1, 0xFFFFFFFF);
}
uint8_t wizchip_read()					//Receive byte via SPI1
{
    uint8_t rbuf;
    HAL_SPI_Receive(&hspi1, &rbuf, 1, 0xFFFFFFFF);
    return rbuf;
}
//---------------Provide capabilities to W5500 Library--------------------------

//--------------------------- W5500 Settings -----------------------------------
uint8_t gDATABUF[DATA_BUF_SIZE];
wiz_NetInfo gWIZNETINFO = { .mac  = {0x00, 0x03, 0xdc,0x00, 0xab, 0xcf},
                            .ip   = {192, 168, 0, 176},
                            .sn   = {255,255,255,0},
                            .gw   = {192, 168, 0, 1},
                            .dns  = {0,0,0,0},
                            .dhcp = NETINFO_STATIC};
uint8_t tmp;
// set buffers sizes W5500 (2 Kb per socket)
uint8_t memsize[2][8] = { {2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
//--------------------------- W5500 Settings -----------------------------------
	
//-----------------------W5500 Callback function--------------------------------
	void Chip_selection_call_back(void)
{
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
}
//-----------------------W5500 Callback function--------------------------------

//------------------------- W5500 Initialize -----------------------------------
void wizchip_initialize(void)
{
    if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1)
    {
       while(1);
    }

    do	 /* PHY link status check */
    {
     if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1)
     HAL_Delay(1);
    }while(tmp == PHY_LINK_OFF);
}
//------------------------- W5500 Initialize -----------------------------------

//------------------------ Network initialize ----------------------------------
void network_init(void)
{
	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
	ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO);
}
//------------------------ Network initialize ----------------------------------

//--------------------------- UDP Function  ------------------------------------
int32_t udp_server(uint8_t sn, uint8_t buf [In_StrLen], uint16_t port)
{
  int32_t  ret;
  uint16_t size		  = In_StrLen;		// Size of RECIEVED array (in bytes)
	uint16_t sentsize = Out_SetLen;  	// Size of SEND array (in bytes)
  uint8_t  destip     [4]; 					// Computer (client) IP Adress
  uint16_t destport;  							// Computer (client) Port
	uint8_t  buf2       [Out_SetLen];	//

  switch(getSn_SR(sn)) 					// Getting socket state (sn - socket number)
   {
     case SOCK_UDP :

		 recvfrom(sn, buf, size, destip, (uint16_t*)&destport);
			i = 0;
        while (i <= In_StrLen)
        {
				 InString[i] = buf[i];i++;
				}

			i = 0;
				while (Out_SetLen > i)
				{
				buf2[i] = OutString[i];i++;
				}
		 sendto(sn,buf2, sentsize, destip, destport);

       break;

      case SOCK_CLOSED:

  if((ret = socket(sn, Sn_MR_UDP, port, 0x00)) != sn)// Opening UDP socket

            return ret;
						break;
   }
						return 1;
}
//--------------------------- UDP Function  ------------------------------------

//----------------------------- UDP pool ---------------------------------------
void UDP_Up(void)
{
	ret = udp_server(SOCK_UDPS, gDATABUF, 2000) < 0;	//Set SOCKET(2000)
}
//----------------------------- UDP pool ---------------------------------------

//////////////////////////////////////////////////////// CAN BUS ///////////////////////////////////////////////////////////////

//------------------------------------------Функция передачи данных по шине CAN-------------------------------------------------
void TransmitCAN (uint8_t a9, uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8)
		{
			can1TxMessage.StdId   = a9;
			can1TxMessage.DLC     =  8;
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
//------------------------------------------------------------------------------------------------------------------------------
		
//--------------- Раскодирование данныйх с чипа 48pin головного контроллера -------	
void raskod_main(uint8_t a, uint8_t b)  //
		{	
     	A1_button1   = ((a & 128) >> 7)^1;
			A1_button2	 = ((a & 64)  >> 6)^1;
			A1_button3   = ((a & 32)  >> 5)^1;
			A1_button4   = ((a & 16)  >> 4)^1;
			
	 if (A1_button3 == 1 || A1_button4 == 1)
      {
        if (A1_button3 == 1){kalorifer = 1;}
        if (A1_button4 == 1){kalorifer = 2;}
      }
      else
      {
       kalorifer = 0;
      }		
      
			A1_button5   = ((a &  8)  >> 3)^1;
			A1_button6   = ((a & 4)   >> 2)^1;
      
      if (A1_button5 == 1 || A1_button6 == 1)
      {
        if (A1_button5 == 1){napravl_kontr = 0;}
        if (A1_button6 == 1){napravl_kontr = 1;}
      }
      else
      {
        napravl_kontr = 2;
      }
       
			A1_button7   = ((a & 2)   >> 1)^1;
			A1_button8   =  (a & 1)        ^1;
			A1_button9   = ((b & 4)   >> 2)^1;
			A1_button10  = ((b & 2)   >> 1)^1;
			A1_button11  =  (b & 1)        ^1;
      
  
      uint8_t temp = 0;
      temp = (A1_button7 << 4) + (A1_button8 << 3) + (A1_button9 << 2) + (A1_button10 << 1) + A1_button11; 
      
      if (temp == 24){polojenie_kontr = 0;}
      if (temp == 16){polojenie_kontr = 1;}
      if (temp == 18){polojenie_kontr = 2;}
      if (temp == 2){polojenie_kontr = 3;}
      if (temp == 17){polojenie_kontr = 4;}
      if (temp == 1){polojenie_kontr = 5;}
      if (temp == 19){polojenie_kontr = 6;}
      if (temp == 3){polojenie_kontr = 7;}
      if (temp == 20){polojenie_kontr = 8;}
      if (temp == 4){polojenie_kontr = 8;}
      if (temp == 22){polojenie_kontr = 8;}
      if (temp == 6){polojenie_kontr = 8;}
      if (temp == 21){polojenie_kontr = 8;}
      if (temp == 5){polojenie_kontr = 8;}
      if (temp == 23){polojenie_kontr = 8;}
      if (temp == 7){polojenie_kontr = 8;}
		}																				
//---------------------------------------------------------------------------------


    void Raskodirovanie3 (uint8_t a1, uint8_t a2, uint8_t a3)
		{							
		  uint8_t temp = a1;		

      A3_button_1	= (temp & 128) >> 7;													
			A3_button_2  = (temp & 64)  >> 6;													
			A3_button_3  = (temp & 32)  >> 5;													
			A3_button_4  = (temp & 16)  >> 4;													
			A3_button_5  = (temp & 8)   >> 3;													
			A3_button_6  = (temp & 4)   >> 2;													
			A3_button_7  = (temp & 2)   >> 1;
      A3_button_8  = (temp & 1);      
      
		temp = a2;																									
			
			A3_button_9	= (temp & 128) >> 7;													
			A3_button_10  = (temp & 64)  >> 6;													
			A3_button_11  = (temp & 32)  >> 5;													
			A3_button_12  = (temp & 16)  >> 4;													
			A3_button_13  = (temp & 8)   >> 3;													
			A3_button_14  = (temp & 4)   >> 2;													
			A3_button_15  = (temp & 2)   >> 1;
      A3_button_16  = (temp & 1);                                
			
		temp = a3;																									
		
			A3_button_17	= (temp & 128) >> 7;													
			A3_button_18  = (temp & 64)  >> 6;													
//			A3_button_19  = (temp & 32)  >> 5;													
			A3_button_20  = (temp & 16)  >> 4;													
			A3_button_21  = (temp & 8)   >> 3;													
			A3_button_22  = (temp & 4)   >> 2;													
			A3_button_23  = (temp & 2)   >> 1;
      A3_button_24  = (temp & 1);   

    if (A3_button_1 != 0 || A3_button_2 !=0)
    {
      if (A3_button_1 == 1){buff_left_front = 1;}
      if (A3_button_2 == 1){buff_left_front = 2;} 
    }
    else
    {
      buff_left_front = 0;
    }
    
    
    
     if (A3_button_3 != 0 || A3_button_4 !=0)
    {
      if (A3_button_3 == 1){buff_right_front = 1;}
      if (A3_button_4 == 1){buff_right_front = 2;} 
    }
    else
    {
      buff_right_front = 0;
    }
    
    
    
    if (A3_button_5 != 0 || A3_button_6 !=0)
    {
      if (A3_button_5 == 1){buff_left_rear = 1;}
      if (A3_button_6 == 1){buff_left_rear = 2;} 
    }
    else
    {
      buff_left_rear = 0;
    }
    
     
    if (A3_button_7 != 0 || A3_button_8 !=0)
    {
      if (A3_button_7 == 1){buff_right_rear = 1;}
      if (A3_button_8 == 1){buff_right_rear = 2;} 
    }
    else
    {
      buff_right_rear = 0;
    } 

     if (A3_button_14 != 0 || A3_button_15 !=0)
    {
      if (A3_button_14 == 1){megometr = 1;}
      if (A3_button_15 == 1){megometr = 2;} 
    }
    else
    {
      megometr = 0;
    }
}																												  	 

////----------- Раскодирование данныйх с контроллера A4 ------------

void Raskodirovanie4 (uint8_t a1, uint8_t a2, uint8_t a3)
		{																														
	    uint8_t temp  = a1;
      
      A4_button_1	  = (temp & 128) >> 7;													
			A4_button_2   = (temp & 64)  >> 6;													
			A4_button_3   = (temp & 32)  >> 5;													
			A4_button_4   = (temp & 16)  >> 4;													
			A4_button_5  = (temp & 8)   >> 3;													
			A4_button_6  = (temp & 4)   >> 2;													
			A4_button_7  = (temp & 2)   >> 1;
      A4_button_8  = (temp & 1);
                        
			 temp = a2;																									
		
      A4_button_9	   = (temp & 128) >> 7;													
			A4_button_10   = (temp & 64)  >> 6;													
			A4_button_11   = (temp & 32)  >> 5;													
			A4_button_12   = (temp & 16)  >> 4;													
			A4_button_13   = (temp & 8)   >> 3;													
			A4_button_14   = (temp & 4)   >> 2;													
			A4_button_15   = (temp & 2)   >> 1;
      A4_button_16   = (temp & 1); 
      
      temp = a3;
      
      A4_button_17	  = (temp & 128) >> 7;													
			A4_button_18   = (temp & 64)  >> 6;													
			A4_button_19 = (temp & 32)  >> 5;													
			A4_button_20   = (temp & 16)  >> 4;													
			A4_button_21  = (temp & 8)   >> 3;													
			A4_button_22  = (temp & 4)   >> 2;													
			A4_button_23  = (temp & 2)   >> 1;
      A4_button_24  = (temp & 1);
      
        
        
      if (A4_button_16 != 0 || A3_button_17 !=0)
    {
      if (A4_button_16 == 1){chastota = 1;}
      if (A4_button_17 == 1){chastota = 2;} 
     
    }
    else
      {
        chastota = 0;
      }
}
		
		////----------- Раскодирование данныйх с контроллера A5 ------------

void Raskodirovanie5 (uint8_t a1, uint8_t a2, uint8_t a3)
		{																														
	    uint8_t temp  = a1;
      
      A5_button_1	  = (temp & 128) >> 7;													
			A5_button_2   = (temp & 64)  >> 6;													
			A5_button_3   = (temp & 32)  >> 5;													
			A5_button_4   = (temp & 16)  >> 4;													
			A5_button_5   = (temp & 8)   >> 3;													
			A5_button_6   = (temp & 4)   >> 2;													
			A5_button_7   = (temp & 2)   >> 1;
      A5_button_8   = (temp & 1);
                        
			 temp = a2;																									
		
      A5_button_9	   = (temp & 128) >> 7;													
			A5_button_10   = (temp & 64)  >> 6;													
			A5_button_11   = (temp & 32)  >> 5;													
			A5_button_12   = (temp & 16)  >> 4;													
			A5_button_13   = (temp & 8)   >> 3;													
			A5_button_14   = (temp & 4)   >> 2;													
			A5_button_15   = (temp & 2)   >> 1;
      A5_button_16   = (temp & 1); 
      
      temp = a3;
      
      A5_button_17	 = (temp & 128) >> 7;													
			A5_button_18   = (temp & 64)  >> 6;													
			A5_button_19   = (temp & 32)  >> 5;													
			A5_button_20   = (temp & 16)  >> 4;													
			A5_button_21   = (temp & 8)   >> 3;													
			A5_button_22   = (temp & 4)   >> 2;													
			A5_button_23   = (temp & 2)   >> 1;
      A5_button_24   = (temp & 1);
			
		  if (A5_button_8 != 0 || A5_button_9 !=0 || A5_button_10 != 0 || A5_button_11 !=0 || A5_button_12 !=0)
    {
      if (A5_button_8 == 1){motors = 1;}
      if (A5_button_9 == 1){motors = 2;} 
      if (A5_button_10 == 1){motors = 0;}
      if (A5_button_11 == 1){motors = 3;} 
      if (A5_button_12 == 1){motors = 4;} 
     
    }
    else
    {
      motors = 0;
    }

//		  if (A5_button_13 != 0 || A5_button_14 !=0 || A5_button_15 !=0)
//    {
      stekloochist = 1;
      if (A5_button_13 == 1){stekloochist = 0;}
//      if (A5_button_14 == 1){stekloochist = 0;} 
      if (A5_button_15 == 1){stekloochist = 2;}
     
//    }
//    else
//    {
//      stekloochist = 0;
//    }
    }  
////----------------------------------------------------------------


	
//----------------Раскодирование при приёме по CAN---------------

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{
	if	(Last_Address_CAN == 3 )
		{
			Raskodirovanie3(can1RxMessage.Data[0], can1RxMessage.Data[1], can1RxMessage.Data[2]);
		  
      goto M2;      
		}
		 
	if	(Last_Address_CAN == 4 )
		{
			Raskodirovanie4(can1RxMessage.Data[0], can1RxMessage.Data[1], can1RxMessage.Data[2]);		  
      goto M2;			
		}		

  if	(Last_Address_CAN == 5 )
		{
			Raskodirovanie5(can1RxMessage.Data[0], can1RxMessage.Data[1], can1RxMessage.Data[2]);		  
      goto M2;			
		}			
		
	if (Last_Address_CAN == 9)
		{			
      cran_395 = can1RxMessage.Data[0];
			cran_254 = can1RxMessage.Data[1];
			cran_254_average = can1RxMessage.Data[2];
      goto M2;
		}	
    
  M2:;
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
}

	
//------------------------------- Функция управления и передачи данных по CAN ---------------------------

  void CAN_TX (void)																																																							
{		

	if (can_address == 2)                    {Last_Address_CAN = 2; 		//	A2
																					 TransmitCAN(2,(pribor_1 >> 8),(pribor_1 & b1111_1111),(pribor_2 >> 8),(pribor_2 & b1111_1111),A2_kod_1,0,0,1); 							 	
																					 can_address = 20; 
																					 goto M1;}
  
  if (can_address == 20)                   {Last_Address_CAN = 2; 		//	A2
																					 TransmitCAN(2,(pribor_3 >> 8),(pribor_3 & b1111_1111),(pribor_4 >> 8),(pribor_4 & b1111_1111),A2_kod_2,0,0,2); 							 	
																					 can_address = 3; 
																					 goto M1;} 
  
  if (can_address == 3)                    {Last_Address_CAN = 3; 		//	A3																			     
                                           TransmitCAN(3,0,0,0,0,0,0,0,0);
																					 can_address = 4;
                                           goto M1;}
  	
	if (can_address == 4)                    {Last_Address_CAN = 4; 		//	A4
																			     TransmitCAN(4,0,0,0,0,0,0,0,0);
																					 can_address = 5;
																					 goto M1;}
	
	if (can_address == 5)                    {Last_Address_CAN = 5;   	//  A5																			     
                                           TransmitCAN(5,0,0,0,0,0,0,0,0);
																					 can_address = 9;   
                                           goto M1;}
	//__________________________________________________________________________________________________ 
  if (can_address == 9)                  {Last_Address_CAN = 9;		//	cran_395
                                           TransmitCAN(9,0,0,0,0,0,0,0,0);
																					 can_address = 32;
	                                         goto M1;} 	
//-------------------------------//-------------------------------//-------------------------------		
 if (can_address == 32)                  {TransmitCAN(32,(step1 >> 8),(step1 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 33;
	                                         goto M1;}		
																					 																					 
 if (can_address == 33)                  {TransmitCAN(33,(step2 >> 8),(step2 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 34;
	                                         goto M1;}

if (can_address == 34)                  {TransmitCAN(34,(step3 >> 8),(step3 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 35;
	                                         goto M1;}

if (can_address == 35)                  {TransmitCAN(35,(step4 >> 8),(step4 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 36;
	                                         goto M1;}

if (can_address == 36)                  {TransmitCAN(36,(step5 >> 8),(step5 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 37;
	                                         goto M1;}																					 
  
	if (can_address == 37)                  {TransmitCAN(37,(step6 >> 8),(step6 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 38;
	                                         goto M1;}																				 
																					 
																					 
	if (can_address == 38)                  {TransmitCAN(38,(step7 >> 8),(step7 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 39;
	                                         goto M1;}																				 
																					 
	if (can_address == 39)                  {TransmitCAN(39,(step8 >> 8),(step8 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 40;
	                                         goto M1;}																				 
																					 
	if (can_address == 40)                  {TransmitCAN(40,(step9 >> 8),(step9 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 41;
	                                         goto M1;}																				 
																					 
	if (can_address == 41)                  {TransmitCAN(41,(step10 >> 8),(step10 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 42;
	                                         goto M1;}																				 
																					 
	if (can_address == 42)                  {TransmitCAN(42,(step11 >> 8),(step11 & b1111_1111),shim_pribor,0, 0,0,0,0);   
	                                         can_address = 100;
	                                         goto M1;}																				 
																					                                        	
	if (can_address == 100){								 // Опрос по UART головной микросхемы 48pin
                                           HAL_UART_Transmit_IT(&huart2, (uint8_t*) 1, 1);
	                                         can_address = 2; } 
  																																																											
  M1:;																																																								
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);																																													
}		

//-------------------------------------------------------------------------------------------------------
												

//----------Decoding_recived_message_from_UART-------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart2)
{
	raskod_main(receiveBuffer[0],receiveBuffer[1]);
  HAL_UART_Receive_IT(huart2, receiveBuffer, 2);
}
//----------Decoding_recived_message_from_UART-------------

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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(1000);
//------------------------------------- Network functions ------------------------------------
	Chip_selection_call_back();																// Chip selection call back 
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);			                        // SPI Read & Write callback function
	wizchip_initialize();  																	// wizchip initialize
	network_init(); 																	    // Network initialization
//------------------------------------- Network functions ------------------------------------

//////////////////////////////////////////////////////////////////////////////////////////////////

//---------------------CAN_Filter---------------------------
	hcan.pTxMsg = &can1TxMessage;
  hcan.pRxMsg = &can1RxMessage;
	CAN_FilterConfTypeDef canFilterConfig;
  canFilterConfig.FilterNumber = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  canFilterConfig.FilterIdHigh = 1 << 5;				// CAN Adress = 1
  canFilterConfig.FilterIdLow  = 1 << 5;
  canFilterConfig.FilterMaskIdHigh = 0 ;
  canFilterConfig.FilterMaskIdLow =  0 ;
  canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.BankNumber = 0;
  HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
  HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	//---------------------CAN_Filter-------------------------
	
	//--------------------------------------------------------
	HAL_Delay(1000); 											// Delay before starting uart, can, tim6
  HAL_UART_Receive_IT  (&huart2, receiveBuffer, 2);				// Initialize UART
	HAL_CAN_Receive_IT   (&hcan, CAN_FIFO0);					// Initialize CAN BUS
	HAL_TIM_Base_Start_IT(&htim6);							    // Initialize Timer6
	//--------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
//--------------- Формирование строк Ввода/Вывода --------------------------------------
		
//------ вывод значений кнопок контроллера А1 -------------------
		
		OutString[0]  = main_button_1;        // РБ (0 - 1)
		OutString[1]  = main_button_2;        // Педаль1
		OutString[2]  = main_button_3;        // 
    OutString[3]  = A1_button1;           // управление общее 
		OutString[4]  = A1_button2;           // топливный насос. Выкл при Авар пит дизеля
		OutString[5]  = kalorifer;            // скорость калорифера
    OutString[6]  = napravl_kontr; 	      // Соответствие реверса
		OutString[7]  = polojenie_kontr;      // Контроллер машиниста
	    
//------ вывод значений кнопок контроллера А3 -------------------
  
		OutString[8]  = buff_left_front;           // Буферные фонари передние левые — Белый/Красный (1 - 0 - 2)
		OutString[9]  = buff_right_front;          // Буферные фонари передние правый — Белый/Красный (1 - 0 - 2)
		OutString[10] = buff_left_rear;            // Буферные фонари задние левые — Белый/Красный (1 - 0 - 2)
		OutString[11] = buff_right_rear;           // Буферные фонари задние правые — Белый/Красный (1 - 0 - 2)
		OutString[12] = A3_button_9;               // радиостанция
		OutString[13] = A3_button_10;              // зелёным светом
    OutString[14] = A3_button_11;              // аккамуляторной батареи
    OutString[15] = A3_button_12;              // автоматическое управлением холодильника
    OutString[16] = A3_button_13;              //освещение пульта
		OutString[17] = megometr;                  // мегометр(1 - 0 - 2)
    OutString[18] = A3_button_16;              // питание приборов
		OutString[19] = A3_button_17;              // масляный насос
    OutString[20] = A3_button_18;              // стоп дизеля2
		OutString[21] = A3_button_19;              // 
		OutString[22] = A3_button_20;              // управление машинами
    OutString[23] = A3_button_21;              // управление переходами
    OutString[24] = A3_button_22;              // топливный насос
    OutString[25] = A3_button_23;              // пуск-остонов дизеля
		OutString[26] = A3_button_24;              // проворот вала дизеля
    
//------ вывод значений кнопок контроллера А4 -------------------

    OutString[27] = A4_button_1;   // жалюзи воды воздухаохладителя
		OutString[28] = A4_button_2;   // жалюзи верхние
		OutString[29] = A4_button_3;   // жалюзи масла
    OutString[30] = A4_button_4;   // жалюзи воды
		OutString[31] = A4_button_5;   // автосцепка передняя
		OutString[32] = A4_button_6;   // муфта вентилятора
		OutString[33] = A4_button_7;   // белый огонь
		OutString[34] = A4_button_8;   // прожектор ярко
		OutString[35] = A4_button_9;   // прожектор тускло
    OutString[36] = A4_button_10;  // автосцепка задняя
    OutString[37] = A4_button_11;  // вызов помощника
    OutString[38] = A4_button_12;  //питание АЛСН
    OutString[39] = A4_button_13;  //бдительность без АЛСН
    OutString[40] = A4_button_14;  // проверка АЛСН
		OutString[41] = A4_button_15;  // выключитель реле РУ14
    OutString[42] = chastota;      // переключатель частоты 75Гц\ 25Гц
		OutString[43] = A4_button_18;  // световой номер
    OutString[44] = A4_button_19;  // возврат реле заземления
		OutString[45] = A4_button_20;  // антиоблединитель
    OutString[46] = A4_button_21;  // проверка пожарной сигнализации
    OutString[47] = A4_button_22;  // включение бдительности
		OutString[48] = A4_button_23;  // вентиль 395 2х ход
		OutString[49] = A4_button_24;  // вентиль 395 3х ход
   		
//------ вывод значений кнопок контроллера А5 -------------------

    OutString[50] = A5_button_1;   // сигнальные контрольные приборы
		OutString[51] = A5_button_2;   // световые приборы
		OutString[52] = A5_button_3;   // дежурное освещение 1
		OutString[53] = A5_button_4;   // дежурное освещение 2
		OutString[54] = A5_button_5;   // вентиляторы кабины
		OutString[55] = A5_button_6;   // прожектор
		OutString[56] = A5_button_7;   // масляный насос
    
    OutString[57] = A5_button_19;   // ЭПК
		OutString[58] = A5_button_20;   // РБС
		OutString[59] = A5_button_21;   // тифон 
		OutString[60] = A5_button_22;   // свисток
    
    OutString[61] = motors;          //
    OutString[62] = stekloochist;    // 

////------ вывод допольнительный устройств -------------------   
    OutString[63] = cran_395;			    // 395 кран (0-XX)
    OutString[64] = cran_254;			    // 254 кран (0-XX)
    OutString[65] = cran_254_average;

/////////////////////////////////////////////////////////////////////////////////
////--------------- Обработка входящих сигналов шаговых приборов ----------------
/////////////////////////////////////////////////////////////////////////////////
    step1       = (InString[0]  << 8) + InString[1];	 //  давление воздух элекроаппаратов      (УД800)
    step2       = (InString[2]  << 8) + InString[3];	 //  давление топливо                     (УД800)
    step3       = (InString[4]  << 8) + InString[5];	 //  давление Масло                       (УД800)
    step4       = (InString[6]  << 8) + InString[7];	 //  t* Масло до холодильника    (ТУЭ8)
    step5       = (InString[8]  << 8) + InString[9];	 //  t* Вода                     (ТУЭ8)
    step6       = (InString[10] << 8) + InString[11];	 //  t* Вода   воздухаохладителя (ТУЭ8)
    step7       = (InString[12] << 8) + InString[13];	 //  воздух питательной сети     (манометр)
    step8       = (InString[14] << 8) + InString[15];	 //  Тормозные цилиндры          (манометр)
    step9       = (InString[16] << 8) + InString[17];	 //  Уравнительный резервуары    (манометр)
    step10      = (InString[18] << 8) + InString[19];	 //  Питательная магистраль      (манометр)
    step11      = (InString[20] << 8) + InString[21];	 //  скорость                    (скоростимер)
  
    pribor_1    = (InString[22]  << 8) + InString[23];	 //  Ток зарядки батареи 150-0-150А М42300
    pribor_2    = (InString[24]  << 8) + InString[25];	 //  напряжение вспом генератора 0-1kV М42300
    pribor_3    = (InString[26]  << 8) + InString[27];	 //  Нагрузка генератора 0-6kA М42300
    pribor_4    = (InString[28]  << 8) + InString[29];	 //  напряжение генератора М42300
   
   
//--------------------Команды для вывода A2 Controller-------------------------

 
    led_1			  =	InString[30];   // ПСС левая
    led_2			  =	InString[31];		// температура масло 84 градус
    led_3			  =	InString[32];		// СН1
    led_4			  =	InString[33];		// СН2
    led_5			  =	InString[34];		// Д2
    led_6			  =	InString[35];		// вентилятор
    led_7			  =	InString[36];		// пожарная станция
    led_8		 	=	InString[38]==1;		// зелёный
    led_11		  =	InString[38]==5;		// жёлтый
    led_12 			=	InString[38]==6;		// жёлтый-красный
		led_13 			=	InString[38]==7;		// красный
    led_14		 	=	InString[38]==8;		// белый

    shim_pribor	=	InString[45];   //  Подсветка приборов (0-64)
    
//------------------------- Опрос тумблеров головного контроллера A1 ------------------------------		

	 if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))   {main_button_1 = 1;} else {main_button_1 = 0;}  
   if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))   {main_button_2 = 1;} else {main_button_2 = 0;}  
	 if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))  {main_button_3 = 1;} else {main_button_3 = 0;} 
	
//------------------ Опрос контроллера машиниста головного контроллера A1 -------------------------
		
//------------------ Объеденение положений направления тяги котроллера машиниста ------------------


		if (key_1 >= 5000)
			{
//				if ( temp_1 == 8)	 { napravl_kontr = 1  ;}      // вперед 0
//				if ( temp_1 == 16) { napravl_kontr = 0  ;}			// нейтраль 0
//				if ( temp_1 == 24) { napravl_kontr = 2  ;}			// Назад - 2
//        key_1 = 0;
			}
		
//------------------ Объеденение положений с котроллера машиниста ---------------------------------

    if (temp_last != temp){key = 0;}
				
		temp_last = temp;
		
//		if (key >= 5000)
//			{
//				if ( temp == 27) { polojenie_kontr = 0  ;}			// Положение контроллера машиниста - 0        
//				if ( temp == 31) { polojenie_kontr = 1  ;}			// Положение контроллера машиниста - 1
//				if ( temp == 21) { polojenie_kontr = 2  ;}			// Положение контроллера машиниста - 2
//				if ( temp == 29) { polojenie_kontr = 3  ;}			// Положение контроллера машиниста - 3
//        if ( temp == 22) { polojenie_kontr = 4  ;}			// Положение контроллера машиниста - 4
//				if ( temp == 30) { polojenie_kontr = 5  ;}			// Положение контроллера машиниста - 5  
//				if ( temp == 20) { polojenie_kontr = 6  ;}			// Положение контроллера машиниста - 6
//				if ( temp == 28) { polojenie_kontr = 7  ;}			// Положение контроллера машиниста - 7        
//				if ( temp == 7)  { polojenie_kontr = 8  ;}			// Положение контроллера машиниста - 8        
//				if ( temp == 15) { polojenie_kontr = 9  ;}			// Положение контроллера машиниста - 9
//				if ( temp == 5)  { polojenie_kontr = 10 ;}			// Положение контроллера машиниста - 10
//				if ( temp == 13) { polojenie_kontr = 11 ;}			// Положение контроллера машиниста - 11				
//				if ( temp == 6)  { polojenie_kontr = 12 ;}			// Положение контроллера машиниста - 12
//				if ( temp == 14) { polojenie_kontr = 13 ;}			// Положение контроллера машиниста - 13
//				if ( temp == 4)  { polojenie_kontr = 14 ;}			// Положение контроллера машиниста - 14
//				if ( temp == 12) { polojenie_kontr = 15 ;}			// Положение контроллера машиниста - 15
//        key = 0;
//			}
		
		if (key < 5000){key ++;}
		
//---------------------------------- kode for A2 ---------------------------------------------------
  
	uint8_t	temp1 = 0;  
					temp1 = temp1 + (led_1 << 7);// ПСС левая
					temp1 = temp1 + (led_2 << 6);// ПСС правая
					temp1 = temp1 + (led_3 << 5);// температура масло 84 градус
					temp1 = temp1 + (led_4 << 4);//СН1
					temp1 = temp1 + (led_5 << 3);//СН2
					temp1 = temp1 + (led_6 << 2);//Д2
					temp1 = temp1 + (led_7 << 1);// вентилятор
					temp1 = temp1 +  led_8;      // пожарная станция        
			
	A2_kod_1 = temp1;	 
	 
	temp1 = 0;  
					temp1 = temp1 + (led_11  << 7);// зелёный
					temp1 = temp1 + (led_12 << 6);// жёлтый
					temp1 = temp1 + (led_13 << 5);// жёлтый-красный
					temp1 = temp1 + (led_14 << 4); // красный
					temp1 = temp1 + (led_15 << 3);// белый	
	
	A2_kod_2 = temp1;	
  
  temp1 = 0;  
//					temp1 = temp1 + (led_17  << 7);
//					temp1 = temp1 + (led_18 << 6);
//					temp1 = temp1 + (led_19 << 5);
//          temp1 = temp1 + (led_17 << 4);
//							
	A2_kod_3 = temp1;	

// -------------------------------------------------------- 
///////////////////// ALSN ////////////////////////////////
//---------------------------------------------------------
//    
//    	  if (led_11 != 0 || led_12 !=0 || led_13 != 0 || led_14 !=0 || led_15 !=0)
//    {
//      if (led_11 == 1){svetofor = 1;}
//      if (led_12 == 1){svetofor = 2;} 
//      if (led_13 == 1){svetofor = 3;}
//      if (led_14 == 1){svetofor = 4;} 
//      if (led_15 == 1){svetofor = 5;} 
//     
//    }
//    else
//    {
//      svetofor = 0;
//    }
//-------------------------------------------------------------------------------------------------

UDP_Up();
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
