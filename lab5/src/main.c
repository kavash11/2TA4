/****** 

1. For GPIO pins, Both OD mode and PP mode can drive the motor! However, some pins cannot output  high in OD mode!!! 
   
2. The signals do not need to be inverted before being fed in H-bridge.   
*/


#include "main.h"


#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)


void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);


static void SystemClock_Config(void);
static void Error_Handler(void);

TIM_HandleTypeDef    Tim3_Handle,Tim4_Handle; //kavya start
TIM_OC_InitTypeDef Tim4_OCInitStructure;
uint16_t Tim3_PrescalerValue,Tim4_PrescalerValue;

__IO uint16_t Tim4_CCR;

void TIM3_Config(void);
void TIM4_Config(void);
void TIM4_OC_Config(void);

void ExtBtn1_Config(void); 
void ExtBtn2_Config(void);
void ExtBtn3_Config(void);
void ExtBtn4_Config(void);
void Pin_Config(void); //need to config pins later kavya

void FullStep(void);
void HalfStep(void);

uint32_t orientation = 0; //0 = cw, 1 = ccw
uint32_t step_type = 0; //0 = full, 1 = half

uint32_t tim3_ctr; //Noor lol
uint32_t step = 0;
uint32_t count = 0;
uint32_t prescaler; // kavya end

//Noor start
uint32_t state;
uint32_t state_temp;

uint8_t redCWfull[8] = {1,1,1,1,0,0,0,0}; //SW1
uint8_t grayCWfull[8] = {0,0,0,0,1,1,1,1}; //SW2
uint8_t blackCWfull[8] = {0,0,1,1,1,1,0,0}; //SW3
uint8_t yellowCWfull[8] = {1,1,0,0,0,0,1,1}; //SW4

uint8_t redCCWfull[8] = {0,0,0,0,1,1,1,1}; //SW1
uint8_t grayCCWfull[8] = {1,1,1,1,0,0,0,0}; //SW2
uint8_t blackCCWfull[8] = {0,0,1,1,1,1,0,0}; //SW3
uint8_t yellowCCWfull[8] = {1,1,0,0,0,0,1,1}; //SW4

uint8_t redCWhalf[8] = {1,1,1,0,0,0,0,0}; //SW1
uint8_t grayCWhalf[8] = {0,0,0,0,1,1,1,0}; //SW2
uint8_t blackCWhalf[8] = {0,0,1,1,1,0,0,0}; //SW3
uint8_t yellowCWhalf[8] = {1,0,0,0,0,0,1,1}; //SW4

uint8_t redCCWhalf[8] = {0,0,0,0,0,1,1,1}; //SW1
uint8_t grayCCWhalf[8] = {0,1,1,1,0,0,0,0}; //SW2
uint8_t blackCCWhalf[8] = {0,0,0,1,1,1,0,0}; //SW3
uint8_t yellowCCWhalf[8] = {1,1,0,0,0,0,0,1}; //SW4

int getNewIndex(uint8_t orient, uint8_t steptype, uint8_t SW1, uint8_t SW2, uint8_t SW3, uint8_t SW4){ //For smooth transition between modes, it finds the index in the arrays of the NEW mode so that it starts from a similar state
	if(orient==0){
		if(steptype==0){
			for(int i=0; i<8; i++){
				if(redCWfull[i]==SW1 && grayCWfull[i]==SW2 && blackCWfull[i]==SW3 && yellowCWfull[i]==SW4){
					return i;
				}
			}
		}
		
		if(steptype==1){
			for(int i=0; i<8; i++){
				if(redCWhalf[i]==SW1 && grayCWhalf[i]==SW2 && blackCWhalf[i]==SW3 && yellowCWhalf[i]==SW4){
					return i;
				}
			}
		}
	}
	
	if(orient==1){
		if(steptype==0){
			for(int i=0; i<8; i++){
				if(redCCWfull[i]==SW1 && grayCCWfull[i]==SW2 && blackCCWfull[i]==SW3 && yellowCCWfull[i]==SW4){
					return i;
				}
			}
		}
		
		if(steptype==1){
			for(int i=0; i<8; i++){
				if(redCCWhalf[i]==SW1 && grayCCWhalf[i]==SW2 && blackCCWhalf[i]==SW3 && yellowCCWhalf[i]==SW4){
					return i;
				}
			}
		}
	}

	return 0; // if it can't find the index, just start from beginning
}

void FullStep(void){
	state+=1;
	if(state>=8){state=0;}
	
	if(orientation==0){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, redCWfull[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, grayCWfull[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, blackCWfull[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, yellowCWfull[state]);
	}
	
	if(orientation==1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, redCCWfull[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, grayCCWfull[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, blackCCWfull[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, yellowCCWfull[state]);
	}
	

}

void HalfStep(void){
	state+=1;
	if(state>=8){state=0;}
	
	if(orientation==0){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, redCWhalf[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, grayCWhalf[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, blackCWhalf[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, yellowCWhalf[state]);
	}
	
	if(orientation==1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, redCCWhalf[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, grayCCWhalf[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, blackCCWhalf[state]);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, yellowCCWhalf[state]);
	}
	

}

void GPIO_Clock_Enable(){RCC->AHB2ENR |= 0x00010000;}
void GPIO_Pin_Init(){
	GPIOE->MODER &=~(3UL<<4);
	GPIOE->MODER |=1UL<<4;
	GPIOE->OTYPER &= ~(1<<2);
	GPIOE->OSPEEDR &= ~(3UL<<4);
	GPIOE->PUPDR &= ~(3UL<<4);
}
//Noor end


int main(void){
	
		state=0; //Noor
		tim3_ctr=0; //Noor
	
		/* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
		HAL_Init();
		
	
		 /* Configure the system clock to 72 MHz */
		SystemClock_Config();
		
		HAL_InitTick(0x0000); // set systick's priority to the highest.
	
	//kavya tim3 &tim4 config
		Tim3_Handle.Init.Period = 36*(10000/48)-1; //full step
		prescaler = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
		TIM3_Config();
		HAL_TIM_Base_Start_IT(&Tim3_Handle);
		TIM4_Config();
	
		Tim4_CCR=500;       //  with clock counter freq as 500,000, this will make OC Freq as 1ms.
		TIM4_OC_Config();	
	
		BSP_LCD_Init();
		//BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address);
		BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
															// the LayerIndex may be 0 and 1. if is 2, then the LCD is dark.
		//BSP_LCD_SelectLayer(uint32_t LayerIndex);
		BSP_LCD_SelectLayer(0);
		BSP_LCD_SetLayerVisible(0, ENABLE); 
		
		BSP_LCD_Clear(LCD_COLOR_WHITE);  //need this line, otherwise, the screen is dark	
		BSP_LCD_DisplayOn();
	 
		BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	
		LCD_DisplayString(2, 3, (uint8_t *)"Lab");
	
		LCD_DisplayInt(2, 8, 5);
		
		//kavya button config
		ExtBtn1_Config(); 
		ExtBtn2_Config();
		ExtBtn3_Config();
		ExtBtn4_Config();
		//Pin_Config();
		
		/*Configure GPIO pins : PE2 PE3 PE4 PE5 */ //Noor CubeMX start
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		__HAL_RCC_GPIOE_CLK_ENABLE();
		GPIO_Clock_Enable();
		GPIO_Pin_Init();
		
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); //Noor CubeMX end
		
			
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);	
		GPIOE->ODR|=1UL<<4;
		//*((volatile uint32_t*)GPIOE)|=1UL<<5;
		BSP_LED_Init(LED3);
		BSP_LED_Init(LED4);
		
		//HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5);

		
		while(1) {	

			
			
		} // end of while loop
	
}  //end of main


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 

* 					Oscillator											=HSE
	*    				HSE frequencey 										=8,000,000   (8MHz)
	*      ----However, if the project is created by uVision, the default HSE_VALUE is 25MHz. thereore, need to define HSE_VALUE
	*						PLL Source											=HSE
  *            PLL_M                          = 4
  *            PLL_N                          = 72
  *            PLL_P                          = 2
  *            PLL_Q                          = 3
  *        --->therefore, PLLCLK =8MHz X N/M/P=72MHz   
	*            System Clock source            = PLL (HSE)
  *        --> SYSCLK(Hz)                     = 72,000,000
  *            AHB Prescaler                  = 1
	*        --> HCLK(Hz)                       = 72 MHz
  *            APB1 Prescaler                 = 2
	*        --> PCLK1=36MHz,  -->since TIM2, TIM3, TIM4 TIM5...are on APB1, thiese TIMs CLK is 36X2=72MHz
							 	
  *            APB2 Prescaler                 = 1
	*        --> PCLK1=72MHz 

  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				BSP_LCD_DisplayChar(COLUMN(ColumnNumber),LINE(LineNumber), *ptr); //new version of this function need Xpos first. so COLUMN() is the first para.
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if ((ColumnNumber+1)*(((sFONT *)BSP_LCD_GetFont())->Width)>=BSP_LCD_GetXSize() ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}


//kavya button configs

void UserBtn_Config(void)     //user btn //Noor
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /* Configure user btn pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);   
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void ExtBtn1_Config(void)     //PC1
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PC1 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void ExtBtn2_Config(void){  //PD2
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOD clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Configure PD2 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);  
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	
	
}

void ExtBtn3_Config(void){  //PC3
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PC3 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_3;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1); 
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_3);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	
	
}

void ExtBtn4_Config(void){  //PD4
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Configure PC3 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_4;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1); 
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_4);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	
	
}


void TIM3_Config(void)
{

	Tim3_PrescalerValue = (uint32_t) ((SystemCoreClock /2) / (10000*(15*1000/40))) - 1;
	Tim3_Handle.Instance = TIM3;
	Tim3_Handle.Init.Period = 36*(10000/48)-1;
	Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
	Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	
	
	if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    Error_Handler();
  }
	
	if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
		//BSP_LED_Toggle(LED3);
    Error_Handler();
  }
	
	//BSP_LED_Toggle(LED4);
}


// configure Timer4 base.
void  TIM4_Config(void)
{

		/* -----------------------------------------------------------------------
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM4CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2
      => TIM4CLK = HCLK = SystemCoreClock
    To get TIM4 counter clock at 500 KHz, the Prescaler is computed as following:
    Prescaler = (TIM4CLK / TIM4 counter clock) - 1
    Prescaler = (SystemCoreClock  /500 KHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  
  /* -----------------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
                                               
    (if CCR1_Val=500, then every 1 ms second will have an interrupt. If count 500 times of interrupt, thta is  0.5 seconds.
		 ----------------------------------------------------------------------- */ 	


  Tim4_PrescalerValue = (uint32_t) ((SystemCoreClock /2) / 500000) - 1;
  
  /* Set TIM3 instance */
  Tim4_Handle.Instance = TIM4; //TIM3 is defined in stm32f429xx.h
  
  /* Initialize TIM4 peripheral as follows:
       + Period = 65535
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
	
	Tim4_Handle.Init.Period = 65535;  //does no matter, so set it to max.
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  //if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //} 
}



void  TIM4_OC_Config(void)
{
		Tim4_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim4_OCInitStructure.Pulse=Tim4_CCR;    // 500, this means every 1/1000 second generate an inerrupt
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 	HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
				
		
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{		
	
		if(GPIO_Pin == KEY_BUTTON_PIN)  //GPIO_PIN_0
		{
					
		}
		
		
		if(GPIO_Pin == GPIO_PIN_1) //orientation switch
		{		
				//BSP_LED_Toggle(LED4);
				//kavya
				if (orientation ==0) { 
					if(step_type==0){state_temp = getNewIndex(1,step_type,redCWfull[state],grayCWfull[state],blackCWfull[state],yellowCWfull[state]);}//Noor
					else{state_temp = getNewIndex(1,step_type,redCWhalf[state],grayCWhalf[state],blackCWhalf[state],yellowCWhalf[state]);}//Noor
					state=state_temp;//Noor - please see explanation on getNewIndex func
					
					orientation =1;
				}
				else if (orientation ==1) {
					if(step_type==0){state_temp = getNewIndex(1,step_type,redCCWfull[state],grayCCWfull[state],blackCCWfull[state],yellowCCWfull[state]);}//Noor
					else{state_temp = getNewIndex(1,step_type,redCCWhalf[state],grayCCWhalf[state],blackCCWhalf[state],yellowCCWhalf[state]);}//Noor
					state=state_temp;//Noor - please see explanation on getNewIndex func
					
					orientation =0;
				}
			
		}  //end of PIN_1

		if(GPIO_Pin == GPIO_PIN_2) { //step type switch
		
				//BSP_LED_Toggle(LED4);
		} //end of if PIN_2	
		
		if(GPIO_Pin == GPIO_PIN_3) //increase speed
		{
						
					//BSP_LED_Toggle(LED4);
				
		} //end of if PIN_3
		
		if(GPIO_Pin == GPIO_PIN_4) //decrease speed
		{
						
					
				//BSP_LED_Toggle(LED4);
		} //end of if PIN_3
}



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
				//clear the timer counter!  in stm32f4xx_hal_tim.c, the counter is not cleared after  OC interrupt
				__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h
				
			
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32fxx_hal_tim.c for different callback function names. 
																															//for timer 3 , Timer 3 use update event initerrupt
{
		if ((*htim).Instance==TIM3){    //since only one timer use this interrupt, this line is actually not needed	
			tim3_ctr+=1;
			if(tim3_ctr>=1000){
				LCD_DisplayInt(4, 0, state);
				BSP_LED_Toggle(LED3);
				tim3_ctr=0;
				if(step_type==0){
					FullStep(); //Noor
				}
				else{HalfStep();}
			}
		}
	
}
 
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
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



