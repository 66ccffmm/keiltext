/**
  ******************************************************************************
  * @file    PWR/STOP/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup PWR_STOP
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t TimingDelay;
ErrorStatus HSEStartUpStatus;
CanTxMsg TxMessage;
/* Private function prototypes -----------------------------------------------*/
void SYSCLKConfig_STOP(void);
void EXTI_Configuration(void);
void RTC_Configuration(void);
void NVIC_Configuration(void);
void SysTick_Configuration(void);
void Delay(__IO uint32_t nTime);
void CAN2_Config(void);
	int MakeCanBag22(void);
	int a;
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

  /* Initialize LEDs and Key Button mounted on STM3210X-EVAL board */       
  STM_EVAL_LEDInit(LED1);
  STM_EVAL_LEDInit(LED2);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Enable PWR and BKP clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ê1?üCAN1ê±?ó	
  CAN2_Config();
	//MakeCanBag22();
//  /* Configure EXTI Line to generate an interrupt on falling edge */
//  EXTI_Configuration();

//  /* Configure RTC clock source and prescaler */
//  RTC_Configuration();

//  /* NVIC configuration */
//  NVIC_Configuration();

//  /* Configure the SysTick to generate an interrupt each 1 millisecond */
//  //SysTick_Configuration();
//   
//     RTC_ClearFlag(RTC_FLAG_SEC);
//    while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);
//	
//	
// RTC_SetAlarm(RTC_GetCounter()+ 1);
//  /* Turn on LED1 */
// STM_EVAL_LEDOn(LED3);
  
  while (1)
  {
		

///这个被过滤掉了
TxMessage.ExtId = 0x600E852C;
TxMessage.IDE = CAN_ID_EXT;
TxMessage.RTR = CAN_RTR_DATA;
TxMessage.DLC = 0x08;
TxMessage.Data[0] = 0x01;
TxMessage.Data[1] = 0x02;
TxMessage.Data[2] = 0x03;
TxMessage.Data[3] = 0x04;
TxMessage.Data[4] = 0x05;
TxMessage.Data[5] = 0x06;
TxMessage.Data[6] = 0x07;
TxMessage.Data[7] = 0x08;

CAN_Transmit(CAN2, &TxMessage);
//		for(a=0;a<0xff;a++);
///这个可以接收到		
//TxMessage.ExtId = 0x600E852C;
//TxMessage.IDE = CAN_ID_STD;
//TxMessage.RTR = CAN_RTR_DATA;
//TxMessage.DLC = 0x08;
//TxMessage.Data[0] = 0x01;
//TxMessage.Data[1] = 0x02;
//TxMessage.Data[2] = 0x03;
//TxMessage.Data[3] = 0x04;
//TxMessage.Data[4] = 0x05;
//TxMessage.Data[5] = 0x06;
//TxMessage.Data[6] = 0x07;
//TxMessage.Data[7] = 0x08;

//CAN_Transmit(CAN2, &TxMessage);	
		


  }
}

/**
  * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
void SYSCLKConfig_STOP(void)
{
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {

#ifdef STM32F10X_CL
    /* Enable PLL2 */ 
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {
    }

#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

/**
  * @brief  Configures EXTI Lines.
  * @param  None
  * @retval None
  */
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Configures RTC clock source and prescaler.
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
  /* RTC clock source configuration ------------------------------------------*/
  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Reset Backup Domain */
  BKP_DeInit();
  
  /* Enable the LSE OSC */
  RCC_LSEConfig(RCC_LSE_ON);
  /* Wait till LSE is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* RTC configuration -------------------------------------------------------*/
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Set the RTC time base to 1s */
  RTC_SetPrescaler(32767);  
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Enable the RTC Alarm interrupt */
  RTC_ITConfig(RTC_IT_ALR, ENABLE);
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
}


void CAN2_Config(void)
{
	GPIO_InitTypeDef 			 GPIO_InitStructure;      
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ê1?üCAN1ê±?ó	


  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStructure);
	 
  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;	//CAN_Mode_LoopBack;//CAN_Mode_LoopBack;//
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;			//??D?í?2?ì????í?è(SJW) 
  CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;			//CAN_BS1_9tq:50k;CAN_BS1_8tq:250k;  ?¨ò?2é?ùμ?μ?????
  CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;			//CAN_BS2_8tq:50k;CAN_BS2_7tq:250k;  ?¨ò?·￠?íμ?μ?????
  CAN_InitStructure.CAN_Prescaler = 12;					//40:50k;9:250k;		·??μ?ê
  CAN_Init(CAN2, &CAN_InitStructure);						//2¨ì??ê=PLCK(36M)/((CAN_SJW_1tq+CAN_BS1_11tq+CAN_BS2_4tq)*·??μ?μ)
																								//2¨ì??ê=36000000/((1+8+7)*9)=250000
																								//sample = ( 1 +  CAN_BS1) /  (1 +  CAN_BS1 +  CAN_BS2)  2é?ùμ?éè???ú80%~80.75%
  




//CAN_FilterInitStructure.CAN_FilterNumber = 0;
//CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
//CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
//CAN_FilterInitStructure.CAN_FilterIdHigh = 0x600E0000;
//CAN_FilterInitStructure.CAN_FilterIdLow = 0x852C0000;
//CAN_FilterInitStructure.CAN_FilterMaskIdHigh =  0xFFFF0000;
//CAN_FilterInitStructure.CAN_FilterMaskIdLow =  0xFFFF0000;
//CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
//CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;


CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
CAN_FilterInitStructure.CAN_FilterNumber = 1;
CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
CAN_FilterInitStructure.CAN_FilterIdHigh = 0x600E;
CAN_FilterInitStructure.CAN_FilterIdLow = 0x852C;
CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
//CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
CAN2_FilterInit(&CAN_FilterInitStructure);



// CAN_FilterInitStructure.CAN_FilterNumber=0;	  //1y???÷0
//  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; 
//  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32?? 
//  CAN_FilterInitStructure.CAN_FilterIdHigh=0x600E;////32??ID   0C01D0A5*8+4=0x600E852C
//  CAN_FilterInitStructure.CAN_FilterIdLow=0x852C;
//  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//0xFFFF;//32??MASK
//  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;//0xFFFF;
//  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//1y???÷01?áaμ?FIFO0
//  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //?¤??1y???÷0
//  CAN2_FilterInit(&CAN_FilterInitStructure);//??2¨?÷3?ê??ˉ

	
  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB2_LP_CAN2_RX0_IRQn;                                  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // ?÷ó??è???a1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // ′?ó??è???a0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


int MakeCanBag22(void)
{
	u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
	
	   /* Transmit 1 message */
    TxMessage.StdId=0x0;
    TxMessage.ExtId = 0x1234;
    TxMessage.RTR=0;
    TxMessage.IDE=0x00000004;
    TxMessage.DLC=2;
    TxMessage.Data[0]=0xDE;
    TxMessage.Data[1]=0xCA;
	
	
  mbox= CAN_Transmit(CAN2, &TxMessage);   
//  i=0;
//  while((CAN_TransmitStatus(CAN2, mbox)!=CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//μè′y·￠?í?áê?
//  if(i>=0XFFF)return 1;
//  return 0;		
}






 


/**
  * @brief  Configures NVIC and Vector Table base location.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bits for Preemption Priority and 2 bits for Sub Priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures the SysTick to generate an interrupt each 1 millisecond.
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void)
{
  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
  /* Set SysTick Priority to 3 */
  NVIC_SetPriority(SysTick_IRQn, 0x0C);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
//void Delay(__IO uint32_t nTime)
//{
//  TimingDelay = nTime;

//  while(TimingDelay != 0);

//}

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
