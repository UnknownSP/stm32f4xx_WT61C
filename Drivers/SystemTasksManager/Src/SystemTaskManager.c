/* ===Kisarazu RBKN Library===
 *
 * autor          : Oishi
 * version        : v0.10
 * last update    : 20160703
 *
 * **overview***
 * システムのタスクを記述
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "SystemTaskManager.h"
#include "MW_USART.h"
#include "MW_I2C.h"
#include "MW_GPIO.h"
#include "DD_Gene.h"
#include "message.h"
#include "app.h"
#include "DD_RC.h" 
#include "MW_IWDG.h"

volatile uint32_t g_SY_system_counter;
volatile uint8_t g_rc_data[RC_DATA_NUM] = {};
static uint8_t rc_rcv[RC_DATA_NUM] = {};
volatile led_mode_t g_led_mode = lmode_1;
static volatile unsigned int count_for_rc = 0;

volatile uint8_t raspi_control_rcv[8] = {0,0,0,0,0,0,0,0};
static uint8_t raspi_control_rcv_data[8] = {0,0,0,0,0,0,0,0};
static bool raspi_control_flag = true;

static int test = 0;
static bool test_flag = true;
static int test_count = 0;

static bool uart_rcv_cplt = true;

static
int SY_init(void);
static
int SY_I2CConnTest(int timeout);
static
int SY_doAppTasks(void);
static
int SY_clockInit(void);
static
void SY_GPIOInit(void);
static
void SY_DMAInit(void);


#if !_NO_DEVICE
static
int SY_doDevDriverTasks(void);
#endif

int main(void){
  int ret,i,j,k,ret_uart;
  static uint8_t test_data[8] = {};
  //static char test_data[8] = {'A','B','C','d','f','g','G','\n'};

  static unsigned int count = 0;
  static uint8_t receive_data[33*5] = {};
  static uint8_t cal_data[33] = {};
  static bool a_flag = true;
  static bool b_flag = true;
  static bool c_flag = true;

  double a[3],w[3],angle[3],T;
  int16_t temp_data = 0;

  unsigned int chk_sum = 0;

  //システムを初期化します
  ret = SY_init();
  if( ret ){
    message("err", "initialize Faild%d", ret);
    MW_waitForMessageTransitionComplete(100);
    MW_GPIOWrite(GPIOAID, GPIO_PIN_5 , GPIO_PIN_SET);
    return EXIT_FAILURE;
  }

  //(未実装)I2Cデバイスのチェックをします
  ret = SY_I2CConnTest(10);
  if( ret ){
    message("err", "I2CConnectionTest Faild%d", ret);
    MW_waitForMessageTransitionComplete(100);
    return EXIT_FAILURE;
  }

  //ここから開始します。
  g_SY_system_counter = 0;

  message("msg", "start!!\n");
  MW_printf("\033[2J\033[1;1H");
  flush();

  while( 1 ){
    receive_data[0] = MW_USART3Receive();
    MW_IWDGClr();
    if(receive_data[0] == 0x55){
      receive_data[1] = MW_USART3Receive();
      MW_IWDGClr();
      if(receive_data[1] == 0x51){
        for(i=0;i<31;i++){
          receive_data[i+2] = MW_USART3Receive();
          MW_IWDGClr();
        }
        break;
      }
    }
  }

  //アプリケーションを開始するためのループです。
  while( 1 ){
    //ウォッチドックタイマーをリセットします
    MW_IWDGClr();//reset counter of watch dog  

    if(uart_rcv_cplt){
      ret_uart = MW_USART3ReceiveMult(66, receive_data);
      uart_rcv_cplt = false;
      if(ret_uart){
        MW_printf("UART Error\n");
      }
    }
    
    for(i=0;i<66;i++){
      if(receive_data[i] == 0x55 && receive_data[i+1] == 0x51 && a_flag){
        chk_sum = 0;
        for(k=0;k<10;k++){
          chk_sum += receive_data[i+k];
        }
        if(receive_data[i+10] == (uint8_t)chk_sum){
          for(j=0;j<11;j++){
            cal_data[j] = receive_data[i+j];
          }
          a_flag = false;
        }
      }
      if(receive_data[i] == 0x55 && receive_data[i+1] == 0x52 && b_flag){
        chk_sum = 0;
        for(k=0;k<10;k++){
          chk_sum += receive_data[i+k];
        }
        if(receive_data[i+10] == (uint8_t)chk_sum){
          for(j=0;j<11;j++){
            cal_data[j+11] = receive_data[i+j];
          }
          b_flag = false;
        }
      }
      if(receive_data[i] == 0x55 && receive_data[i+1] == 0x53 && c_flag){
        chk_sum = 0;
        for(k=0;k<10;k++){
          chk_sum += receive_data[i+k];
        }
        if(receive_data[i+10] == (uint8_t)chk_sum){
          for(j=0;j<11;j++){
            cal_data[j+22] = receive_data[i+j];
          }
          c_flag = false;
        }
      }
      if(!a_flag && !b_flag && !c_flag){
        break;
      }
    }
    a_flag = true;
    b_flag = true;
    c_flag = true;

    temp_data = (cal_data[3]<<8) | cal_data[2];
    a[0] = (double)temp_data/32768.0 * 16.0*9.8;
    temp_data = (cal_data[5]<<8) | cal_data[4];
    a[1] = (double)temp_data/32768.0 * 16.0*9.8;
    temp_data = (cal_data[7]<<8) | cal_data[6];
    a[2] = (double)temp_data/32768.0 * 16.0*9.8;
    temp_data = (cal_data[9]<<8) | cal_data[8];
    T = (double)temp_data/340.0 + 36.25;

    temp_data = (cal_data[14]<<8) | cal_data[13];
    w[0] = (double)temp_data/32768.0 * 2000;
    temp_data = (cal_data[16]<<8) | cal_data[15];
    w[1] = (double)temp_data/32768.0 * 2000;
    temp_data = (cal_data[18]<<8) | cal_data[17];
    w[2] = (double)temp_data/32768.0 * 2000;
    temp_data = (cal_data[20]<<8) | cal_data[19];
    T = (double)temp_data/340.0 + 36.25;

    temp_data = (cal_data[25]<<8) | cal_data[24];
    angle[0] = (double)temp_data/32768.0 * 180;
    temp_data = (cal_data[27]<<8) | cal_data[26];
    angle[1] = (double)temp_data/32768.0 * 180;
    temp_data = (cal_data[29]<<8) | cal_data[28];
    angle[2] = (double)temp_data/32768.0 * 180;
    temp_data = (cal_data[31]<<8) | cal_data[30];
    T = (double)temp_data/340.0 + 36.25;

    if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
      if( g_SY_system_counter % 1000 == 0){
	      MW_printf("\033[2J");
      }
      MW_printf("\033[1;1H");//カーソルを(1,1)にセットして
      //DD_RCPrint((uint8_t*)g_rc_data);//RCのハンドラを表示します
      for(i=0;i<33;i++){ 
        MW_printf("[%4d]",cal_data[i]);
        test_count++; 
        if(test_count%11 == 0){
          test_count = 0;
          MW_printf("\n"); 
        }
      }
      test_count = 0;
      MW_printf("\n"); 
      MW_printf("%5d %5d %5d %5d\n",(int)(a[0]*100.0),(int)(a[1]*100.0),(int)(a[2]*100.0),(int)(T*10.0));
      MW_printf("%5d %5d %5d %5d\n",(int)(w[0]*100.0),(int)(w[1]*100.0),(int)(w[2]*100.0),(int)(T*10.0));
      MW_printf("%5d %5d %5d %5d\n",(int)(angle[0]*100.0),(int)(angle[1]*100.0),(int)(angle[2]*100.0),(int)(T*10.0));
      DD_print();//各デバイスハンドラを表示します
      flush(); /* out message. */
    }
    //タイミング待ちを行います
    while( g_SY_system_counter % _INTERVAL_MS != _INTERVAL_MS / 2 - 1 ){
    }
    //エラー処理です
    if( ret ){
      message("err", "Device Driver Tasks Faild%d", ret);
      return EXIT_FAILURE;
    }
    //タイミング待ちを行います  
    while( g_SY_system_counter % _INTERVAL_MS != 0 ){  
    }
  }
} /* main */

void SY_wait(int ms){
  volatile uint32_t time;
  time = g_SY_system_counter;
  while(time + ms > g_SY_system_counter)
    MW_IWDGClr();//reset counter of watch dog
  MW_IWDGClr();//reset counter of watch dog
}

static
int SY_doAppTasks(void){
  return appTask();
}

#if !_NO_DEVICE
static
int SY_doDevDriverTasks(void){
  return DD_doTasks();
}
#endif

static
int SY_I2CConnTest(int timeout){
  UNUSED(timeout);
  return EXIT_SUCCESS;
}

static
int SY_init(void){
  int ret,i,j;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
  **/
  if( HAL_Init()){
    return EXIT_FAILURE;
  }

  ret = SY_clockInit();
  /* Configure the system clock */
  if( ret ){
    return EXIT_FAILURE;
  }

  SY_DMAInit();
  /*Initialize GPIO*/
  SY_GPIOInit();

  /*UART initialize*/
  ret = MW_USARTInit(USART2ID);
  if( ret ){
    return EXIT_FAILURE;
  }

  ret = MW_USARTInit(USART3ID);
  if( ret ){
    return ret;
  }

  /*Initialize printf null transit*/
  flush();

  appInit();
  
  /*initialize IWDG*/
  message("msg", "IWDG initialize");
  MW_SetIWDGPrescaler(IWDG_PRESCALER_4);//clock 32kHz --> 1/4 -->8000Hz
  MW_SetIWDGReload(800);//Reload volue is 800. reset time(100ms)
  ret = MW_IWDGInit(); 
  if(ret){
    message("err", "IWDG initialize failed!\n");
    return ret;
  }
  
  appInit();

  return EXIT_SUCCESS;
} /* SY_init */

/**
   oscの設定を行います。
 */
static
int SY_clockInit(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  return EXIT_SUCCESS;
} /* SY_clockInit */

static
void SY_GPIOInit(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA10 PA11 
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC8 PC9 
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12 
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void SY_DMAInit(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
  int ret = 0;
  UNUSED(UartHandle);
  if(UartHandle->Instance == USART6){
    uart_rcv_cplt = true;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){
  UNUSED(UartHandle);
  MW_messageTransitionCompletedCallBack();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
  UNUSED(hi2c);
  MW_I2C2TransitionCompletedCallBack();
  //DD_receive2SS();
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
  UNUSED(hi2c);
  MW_I2C2ReceptionCompletedCallBack();
  //DD_receive2SS();
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  MW_GPIOWrite(GPIOAID, GPIO_PIN_5 , GPIO_PIN_SET);
  /* USER CODE END Error_Handler_Debug */
}
