/***********************************************************
	File:     main.c
	Purpose:  STM32F030F4 based Cell Monitor.
	Date:     14 July, 2015

	Changes:
				3 Jul 2017, Dave:
				added a test to ensure both pairs of bytes match in a header message.
				30 Jun 2017, Dave:
				changed 'wake from sleep' from interrupt to event.
				added separate defines for alarm voltages.
				changed check for chunk of bytes so it copes with extras.
				corrected the mask bits for the high byte of u16_mean_cell_mv from master.
			v2	12 Jul 2016, Dave:
				made system tick active when load is on so that load can be cycled much faster
				to give better control. Experimented with PWM for load but had no advantage.

 **********************************************************/

/***********************************************************
	Operation:
	Every 2s the system wakes from stop and samples the ADC for voltage and temperature.
	If the voltage is above the load threshold the load output is activated.
	While the load is active the system samples every 20ms.
	When the load is inactive, the system returns to stop for 2s.

	If a byte is received the system is woken from stop,
	when a block of 6 bytes has been received they are added to the tx buffer.
	if bytes stop arriving then the cell data is appended to the tx buffer and sent.
***********************************************************/


/************** Includes **********************************/
#include <stm32f0xx.h>
#include <stm32f0xx_adc.h>
#include <stm32f0xx_dma.h>
#include <stm32f0xx_exti.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_iwdg.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_pwr.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_rtc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_usart.h>

#include "hardware.h"
#include "main.h"
#include "fec.h"
#include "temperature.h"


/************** Defines ***********************************/


/************** Prototypes ********************************/
static void			fu8_set_GPIO(uint8_t u8_type);
static void			fv_set_ADC(uint8_t u8_type);
static void			fv_set_timers(uint8_t u8_type);
static void			fv_set_USART1(uint8_t u8_type);
static void			fv_set_systick(uint8_t u8_type, uint8_t u8_enable);
static void			fv_set_IWDG_Init(void);
static void			fv_set_RTC(uint8_t u8_type);

static void			fv_read_ADC(uint8_t u8_type);
static uint16_t		fu16_scale_reading(uint16_t u16_adc_val, uint16_t u16_max_val);
static void			fv_process_comms(void);
static void			fv_append_data(void);
static void			fv_show_LED(void);


/************** Variables *********************************/
volatile uint16_t	u16a_adc_buf[MAX_ADC_CHANS];
uint16_t			u16_watchdog_word;
uint16_t			u16_tick_cntr;
uint8_t				u8a_USART1_tx_buf[TX_BUFFER_SIZE];
volatile uint16_t	u16_UART1_tx_wp;
volatile uint8_t	u8a_USART1_rx_buf[RX_BUFFER_SIZE];
volatile uint8_t	u8_UART1_rx_wp;
volatile uint8_t	u8_UART1_rx_rp;
volatile uint8_t	u8_comms_mode;
uint8_t				u8_hardware_type;
uint8_t				u8_ms_per_tick;							// default to 1ms
volatile uint8_t	u8_sys_tick_f;
volatile uint8_t	u8_comms_tick_f;
volatile uint8_t	u8_sleep_not_stop;

uint16_t			u16_cell_voltage;						// should be around 3.650v when charged
int16_t				s16_cell_temperature;
uint16_t			u16_mean_cell_mv;
volatile uint16_t	u16_comms_intrvl_tmr;					// incremented every sys_tick, zeroed when we receive a chunk.
uint8_t				u8_load_state;
uint8_t				u8_alarm_state;
uint8_t				u8_cell_number;
uint8_t				u8_cell_counter;
uint8_t				u8_valid_header;						// set when a master message received, cleared when we on-send


/***********************************************************
	Main.
***********************************************************/
int main(void)
{
	uint16_t	u16_i;

	// read hardware type
	u8_hardware_type			= HW_TYPE_CELL_MONITOR;

	// set IO pins
	fu8_set_GPIO(u8_hardware_type);

	// set the watchdog
	fv_set_IWDG_Init();

	// initialise ADC
	fv_set_ADC(u8_hardware_type);

	// set timers
	fv_set_timers(u8_hardware_type);

	// initialise comms
	fv_set_USART1(u8_hardware_type);

	// set systick
	fv_set_systick(u8_hardware_type, 1);

	// set the RTC if we use STOP mode
	fv_set_RTC(u8_hardware_type);

	// initialise variables
	u8_sleep_not_stop				= 0;
	u8_load_state					= 0;
	// BattMan should set this
	u16_mean_cell_mv				= 0;
	u16_comms_intrvl_tmr			= 0;
	u8_cell_number					= CELL_NUM_NOT_SET;			// default to a number that cannot be achieved
	u8_cell_counter					= 0;
	u8_valid_header					= 0;

	// flush the serial buffers.
	for (u16_i = 0; u16_i < RX_BUFFER_SIZE; ++u16_i)
	{
		u8a_USART1_rx_buf[u16_i]	= 0;
	}
	for (u16_i = 0; u16_i < TX_BUFFER_SIZE; ++u16_i)
	{
		u8a_USART1_tx_buf[u16_i]	= 0;
	}
	u8_UART1_rx_rp					= 0;
	u8_UART1_rx_wp					= 0;
	u16_UART1_tx_wp					= 0;
	u16_i							= 0;
	u16_tick_cntr					= 0;

    while(1)
    {
		if (0 != u8_sys_tick_f)								// takes ~700us for cell monitor @ 8MHz
		{
			// reads and scales all ADC inputs, which DMA copied into memory
			// and checks for error conditions
			fv_read_ADC(u8_hardware_type);

			if (0 != u8_sleep_not_stop)
			{
				++u16_tick_cntr;
			}
			else
			{
				u16_tick_cntr			= 0;
			}

			if (u16_comms_intrvl_tmr >= AUTOSEND_SECONDS)	// u16_comms_intrvl_tmr is reset by fv_append_data()
			{
				u8_sleep_not_stop		= 1;
//				u8_comms_tick_f			= 1;				// TODO: don't think we need this to be set here
				u16_mean_cell_mv		= 0;				// no message from master so erase mean_mv

				fv_append_data();							// this will also zero u16_comms_intrvl_tmr

				// set up DMA transmit
				fv_USART1_send(u16_UART1_tx_wp, &u8a_USART1_tx_buf[0]);

				TIM_Cmd(TIM14, DISABLE);
				TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
			}
			else if ( (u16_tick_cntr > ((TX_BUFFER_SIZE + RX_BUFFER_SIZE) / u8_ms_per_tick))
					&& (u8_comms_mode != COMMS_IDLE) )
			{
				u8_comms_mode			= COMMS_IDLE;
			}

			u8_sys_tick_f				= 0;
		} // end of if systick

		if (u8_comms_tick_f != 0)
		{
			// process message
			fv_process_comms();

			u8_comms_tick_f				= 0;
		} // end of if comms tick

		// turn off both LEDs
		// leave the ALARM LED on if overvoltage
		if ((u8_alarm_state & ALRM_OVERVOLT) == 0)
		{
			GPIO_WriteBit(ALARM_LED_PORT, ALARM_LED_PIN, 0);
		}
		GPIO_WriteBit(HEART_LED_PORT, HEART_LED_PIN, 0);	// Heartbeat LED

		// if load on or comms active, SLEEP instead of STOP.
		if (u8_sleep_not_stop != 0)
		{
			++u16_i;										// incremented every 20ms
			if (u16_i > (2400 / u8_ms_per_tick))
			{
				u16_i					= 0;
				IWDG_ReloadCounter();						// commenting this resets the PWM every 2.5s!
			}

			// save power by going to sleep
//			PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
			// 30 Jun 2017, Dave - changed wake from sleep from interrupt to event.
			PWR_EnterSleepMode(PWR_SLEEPEntry_WFE);			// wait for event seems more reliable than wait for interrupt
		}
		else
		{
			u16_i					= 0;
			// reload the watchdog before going to sleep.
			IWDG_ReloadCounter();

			PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

			fv_set_RTC(u8_hardware_type);					// reset the RTC to alarm again
		}
    } // end of while 1
} // end of main -------------------------------------------


/***********************************************************
  * @brief  copies to the DMA tx buffer for the USART and initiates transmission.
  * @param  The number of bytes to be sent, truncates if more than buffer size.
  * 		pointer to the source buffer
  * @retval None
  *********************************************************/
void			fv_USART1_send(uint16_t u16_num_bytes, uint8_t u8a_source[])
{
    DMA_InitTypeDef     DMA_InitStruct;
	NVIC_InitTypeDef	NVIC_InitStructure;
	uint16_t			u16_i;

	/* ensure data does not overrun buffer. */
	if (TX_BUFFER_SIZE < u16_num_bytes)
	{
		u16_num_bytes						= TX_BUFFER_SIZE;
	}

	/* copy the bytes into the tx buffer. */
	for (u16_i = 0; u16_i < u16_num_bytes; u16_i++)
	{
		u8a_USART1_tx_buf[u16_i]			= u8a_source[u16_i];
	}

	/* DMA1 Channel2 Config */
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStruct.DMA_PeripheralBaseAddr	= (uint32_t)&USART1->TDR;
	DMA_InitStruct.DMA_MemoryBaseAddr		= (uint32_t)u8a_USART1_tx_buf;
	DMA_InitStruct.DMA_DIR					= DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_BufferSize			= u16_num_bytes;
	DMA_InitStruct.DMA_PeripheralInc		= DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc			= DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize		= DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode					= DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority				= DMA_Priority_Medium;
	DMA_InitStruct.DMA_M2M					= DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStruct);

	/* Enable USART_DMA */
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	DMA_ClearFlag(DMA1_FLAG_TC2);
	USART_ClearFlag(USART1, USART_FLAG_TC);

	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

	/* Enable DMA1 channel2 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel			= DMA1_Channel2_3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd		= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* DMA1 Channel2 enable */
	DMA_Cmd(DMA1_Channel2, ENABLE);
} // end of fv_USART1_send function ------------------------


/***********************************************************
 * This sends debug numbers over the debug uart.
 * only accepts unsigned 16bit values.
 * applies leading zero suppression.
 **********************************************************/
void			fv_print_number(uint16_t u16_num)
{
	uint8_t		u8a_data_buf[TX_BUFFER_SIZE];
	uint8_t		u8_suppress	= 1;							// TRUE to suppress leading zeros

	if (u16_num > 9999)
	{
		u8a_data_buf[0]		= (u16_num / 10000) + '0';		// convert 5th digit to ascii
		u16_num				= (u16_num % 10000);			// remove 5th digit.
		u8_suppress			= 0;							// turn off suppression as we've found a digit
	} //end of if 5 digit value
	else
	{
		u8a_data_buf[0]		= ' ';							// pad with a leading space
	}

	if (u16_num > 999)
	{
		u8a_data_buf[1]		= (u16_num / 1000) + '0';		// convert 4th digit to ascii
		u16_num				= (u16_num % 1000);				// remove 4th digit.
		u8_suppress			= 0;							// turn off suppression as we've found a digit
	} //end of if 4 digit value
	else
	{
		if (0 == u8_suppress)
		{
			u8a_data_buf[1]	= '0';							// pad with a zero
		}
		else
		{
			u8a_data_buf[1]	= ' ';							// pad with a leading space
		}
	}

	if (u16_num > 99)
	{
		u8a_data_buf[2]		= (u16_num / 100) + '0';		// convert 3rd digit to ascii
		u16_num				= (u16_num % 100);				// remove 3rd digit.
		u8_suppress			= 0;							// turn off suppression as we've found a digit
	} //end of if 3 digit value
	else
	{
		if (0 == u8_suppress)
		{
			u8a_data_buf[2]	= '0';							// pad with a zero
		}
		else
		{
			u8a_data_buf[2]	= ' ';							// pad with a leading space
		}
	}

	if (u16_num > 9)
	{
		u8a_data_buf[3]		= (u16_num / 10) + '0';			// convert 2nd digit to ascii
		u16_num				= (u16_num % 10);				// remove 2nd digit.
	} //end of if 2 digit value
	else
	{
		if (0 == u8_suppress)
		{
			u8a_data_buf[3]	= '0';							// pad with a zero
		}
		else
		{
			u8a_data_buf[3]	= ' ';							// pad with a leading space
		}
	}

	u8a_data_buf[4]			= u16_num + '0';				// convert 1st digit to ascii
	u8a_data_buf[5]			= '\r';
	u8a_data_buf[6]			= '\n';
	u8a_data_buf[7]			= 0;

	fv_USART1_send(7, &u8a_data_buf[0]);
} // end of vPrintNumber -----------------------------------


/***********************************************************
	@brief	Sets the GPIO pins to suit the hardware type.
	@param	the hardware type identifier.
	@retval	None.
***********************************************************/
static void			fu8_set_GPIO(uint8_t u8_type)
{
	GPIO_InitTypeDef 	GPIO_InitStruct;

	// Enable the clocks for GPIO
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

	// Ensure LSE is OFF so it doesn't claim the OSC32 pins
	PWR_BackupAccessCmd(ENABLE);
	RCC_LSEConfig(RCC_LSE_OFF);
	// Ensure HSE is OFF so it doesn't claim the OSC pins
	RCC_HSEConfig(RCC_HSE_OFF);

#if (1 == BOARD_VERSION)
	// GPIOA Outputs.	PA0 PA6
	GPIO_InitStruct.GPIO_Pin		= GPIO_Pin_0 | GPIO_Pin_6;
#else
	// GPIOA Outputs.	PA0 PA5
	GPIO_InitStruct.GPIO_Pin		= GPIO_Pin_0 | GPIO_Pin_5;
#endif
	GPIO_InitStruct.GPIO_Mode		= GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType		= GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd		= GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed		= GPIO_Speed_Level_1;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// GPIOF Outputs	PF0 PF1
	GPIO_InitStruct.GPIO_Pin		= GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode		= GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType		= GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd		= GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed		= GPIO_Speed_2MHz;
	GPIO_Init(GPIOF, &GPIO_InitStruct);

	// set initial state of LEDs
	GPIO_WriteBit(HEART_LED_PORT, HEART_LED_PIN, 1);		// Heartbeat LED
	GPIO_WriteBit(ALARM_LED_PORT, ALARM_LED_PIN, 1);		// Alarm LED
	GPIO_WriteBit(CELL_LOAD_PORT, CELL_LOAD_PIN, 0);		// load output
} // end of fu8_read_hardware_type -------------------------


/***********************************************************
	@brief	Sets the ADC to suit the hardware type.
	@param	the hardware type identifier.
	@retval	None.
***********************************************************/
static void			fv_set_ADC(uint8_t u8_type)
{
    GPIO_InitTypeDef	GPIO_InitStruct;
    ADC_InitTypeDef		ADC_InitStruct;
	DMA_InitTypeDef		DMA_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	uint8_t				u8_num_adc_chans		= 2;

	/* ADC1 DeInit */
	ADC_DeInit(ADC1);

	/* Peripheral clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

	/**ADC GPIO Configuration

	Cell Monitor
	PA4     ------> ADC_IN4		Tcell
	PA7     ------> ADC_IN7		Vcell
	*/

	GPIO_InitStruct.GPIO_Pin					= GPIO_Pin_4 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode					= GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd					= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_Resolution				= ADC_Resolution_12b;
	ADC_InitStruct.ADC_DataAlign				= ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ScanDirection			= ADC_ScanDirection_Upward;
	ADC_InitStruct.ADC_ContinuousConvMode		= DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge	= ADC_ExternalTrigConvEdge_None;
	ADC_Init(ADC1, &ADC_InitStruct);

	// Configure the selected ADC regular channel to be converted.
	ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_71_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_7, ADC_SampleTime_71_5Cycles);
	u8_num_adc_chans							= 2;

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_StructInit(&DMA_InitStructure);

	DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr		= (uint32_t)&u16a_adc_buf[0];
	DMA_InitStructure.DMA_DIR					= DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize			= u8_num_adc_chans;
	DMA_InitStructure.DMA_PeripheralInc			= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc				= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize		= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode					= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority				= DMA_Priority_High;
	DMA_InitStructure.DMA_M2M					= DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

	/* Enable DMA1 channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel			= DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd		= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* ADC DMA request in circular mode */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait for the Ready flag */
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY) == 0);

	/* ADC1 regular Software Start Conv */
	ADC_StartOfConversion(ADC1);
} // end of fv_set_ADC -------------------------------------


/***********************************************************
	@brief	Sets the Timers to suit the hardware type.
	@param	the hardware type identifier.
	@retval	None.
***********************************************************/
static void			fv_set_timers(uint8_t u8_type)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef		TIM_OCInitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;
	uint16_t				u16_period, u16_prescaler, u16_pulse, u16_OC_Mode;
	uint8_t					u8_enable				= 0;

	/** set up TIM14 to provide a timeout for comms. **/
	u16_period				= 65000;						// set this out of the way
	u16_prescaler			= (SystemCoreClock / 1000) - 1;	// ~1kHz
	u16_OC_Mode				= TIM_OCMode_Timing;			// don't affect the pin
	u16_pulse				= 0;							// initialise to off
	// Don't enable the counter yet, we'll do it when we receive data
	u8_enable				= 0;

	if (u16_period != 0)
	{
		// TIMx clock enable
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);

		// TIMx Configuration
		TIM_DeInit(TIM14);

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

		TIM_OCStructInit(&TIM_OCInitStructure);

		// Time base configuration
		TIM_TimeBaseStructure.TIM_Period			= u16_period;
		TIM_TimeBaseStructure.TIM_Prescaler			= u16_prescaler;
		TIM_TimeBaseStructure.TIM_ClockDivision		= 0x0;
		TIM_TimeBaseStructure.TIM_CounterMode		= TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode				= u16_OC_Mode;
		TIM_OCInitStructure.TIM_OCIdleState			= TIM_OCIdleState_Reset;
		TIM_OCInitStructure.TIM_OCPolarity			= TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_Pulse				= u16_pulse;
		if (u16_OC_Mode == TIM_OCMode_Timing)
		{
			TIM_OCInitStructure.TIM_OutputState		= TIM_OutputState_Disable;
		}
		TIM_OC1Init(TIM14, &TIM_OCInitStructure);

		// Enable the TIMx global Interrupt
		NVIC_InitStructure.NVIC_IRQChannel			= TIM14_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority	= 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd		= ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);

		if (u8_enable != 0)
		{
			TIM_Cmd(TIM14, ENABLE);
		}
	} // end of if valid
} // end of fv_set_timers ----------------------------------


/***********************************************************
	@brief	Sets the USART1 to suit the hardware type.
	@param	the hardware type identifier.
	@retval	None.

	setting a non-zero baud rate will enable the UART.
***********************************************************/
static void			fv_set_USART1(uint8_t u8_type)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	USART_InitTypeDef	USART_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	EXTI_InitTypeDef	EXTI_InitStructure;
    uint32_t			u32_baud						= 0;

	u32_baud											= BAUDRATE;

	/** Configure EXTI3 to wake from STOP. **/

	// EXTI configuration
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_InitStructure.EXTI_Line						= EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode						= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger						= EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd						= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// NVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel					= EXTI2_3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd				= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	if (0 != u32_baud)
	{
	    // GPIO clock already enabled.

		// Enable USART clock.
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_USARTCLKConfig(RCC_USART1CLK_SYSCLK);

		// PA2	---->	Tx
		// PA3	---->	Rx
    	GPIO_InitStruct.GPIO_Pin						= GPIO_Pin_2 | GPIO_Pin_3;
		GPIO_InitStruct.GPIO_Mode						= GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType						= GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd						= GPIO_PuPd_NOPULL;
		GPIO_InitStruct.GPIO_Speed						= GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &GPIO_InitStruct);

    	// Connect PA2 to USART1_Tx.
    	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    	// Connect PA3 to USART1_Rx.
    	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

        USART_InitStructure.USART_BaudRate				= u32_baud;
        USART_InitStructure.USART_WordLength			= USART_WordLength_8b;
        USART_InitStructure.USART_StopBits				= USART_StopBits_1;
        USART_InitStructure.USART_Parity				= USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode					= USART_Mode_Rx | USART_Mode_Tx;
    	USART_Init(USART1, &USART_InitStructure);

    	// Invert the TX pin as Q2 will invert it as well.
    	USART_InvPinCmd(USART1, USART_InvPin_Tx, ENABLE);

        // Enable USART.
        USART_Cmd(USART1, ENABLE);

        /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel				= USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPriority		= 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd			= ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        // we use DMA for TX so we only need the RX interrupt
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	} // end of if UART required
} // end of fv_set_USART1 ----------------------------------


/***********************************************************
	@brief	Sets the systick to generate an interrupt every ms.
	@param	the hardware type identifier.
	@retval	None.
***********************************************************/
static void			fv_set_systick(uint8_t u8_type, uint8_t u8_enable)
{
	uint16_t		u16_tick_freq;

	u8_ms_per_tick					= 20;

	if ( (u8_ms_per_tick != 0)
			&& (u8_enable != 0) )
	{
		u16_tick_freq				= (1000 / u8_ms_per_tick);
		SysTick_Config((SystemCoreClock / u16_tick_freq));

		/* System interrupt init*/
		NVIC_SetPriority(SysTick_IRQn, 0);
	}
} // end of fv_set_systick ---------------------------------


/***********************************************************
	@brief	IWDG init function
	@param	None.
	@retval	None.
***********************************************************/
static void			fv_set_IWDG_Init(void)
{
	// Enable the LSI OSC
	RCC_LSICmd(ENABLE);

	// Wait till LSI is ready
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{
	}

	// Enable write access to IWDG_PR and IWDG_RLR registers
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	// IWDG counter clock: LSI/32
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/*******************************************************
	Set counter reload value to obtain 250ms IWDG TimeOut.
		Counter Reload Value	= 250ms / IWDG counter clock period
								= 250ms / (LSI / 32)
								= 0.25s / (LsiFreq / 32)
								= LsiFreq / (32 * 4)
								= LsiFreq / 128

	Set counter reload value to obtain 2500ms IWDG TimeOut.
		Counter Reload Value	= 2500ms / IWDG counter clock period
								= 2500ms / (LSI / 32)
								= 2.5s / (LsiFreq / 32)
								= LsiFreq / (32 / 2.5)
								= LsiFreq / 13

	*******************************************************/

	IWDG_SetReload(LSI_FREQ / 13);			// TODO: need to adjust this if INTERVAL_SECONDS is changed

	// Reload IWDG counter
	IWDG_ReloadCounter();

	// Enable IWDG (the LSI oscillator will be enabled by hardware)
	IWDG_Enable();

	u16_watchdog_word		= 0;
} // end of fv_set_IWDG_Init -------------------------------


/***********************************************************
  * @brief  Configures the RTC clock source.
	@param	the hardware type identifier.
  * @retval None
***********************************************************/
static void			fv_set_RTC(uint8_t u8_type)
{
	RTC_InitTypeDef		RTC_InitStructure;
	RTC_TimeTypeDef		RTC_TimeStructure;
	RTC_AlarmTypeDef	RTC_AlarmStructure;
	EXTI_InitTypeDef	EXTI_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	// Enable the PWR clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	// Allow access to RTC
	PWR_BackupAccessCmd(ENABLE);

	// Enable the LSI OSC
	RCC_LSICmd(ENABLE);

	// Wait till LSI is ready
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{
	}

	// Select the RTC Clock Source
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	// Enable the RTC Clock
	RCC_RTCCLKCmd(ENABLE);

	// Wait for RTC APB registers synchronisation
	RTC_WaitForSynchro();

	RTC_InitStructure.RTC_HourFormat				= RTC_HourFormat_24;
	RTC_InitStructure.RTC_AsynchPrediv				= 0x7F;
	RTC_InitStructure.RTC_SynchPrediv				= 0x0138;

	if (RTC_Init(&RTC_InitStructure) == ERROR)
	{
		while(1);
	}

	// EXTI configuration
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line					= EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode					= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger					= EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd					= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// NVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel				= RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority		= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Set the alarm X + 2s
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12		= RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours		= 0x01;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes	= 0x00;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds	= INTERVAL_SECONDS;	// alarm in 2s
	RTC_AlarmStructure.RTC_AlarmDateWeekDay			= 0x31;
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel		= RTC_AlarmDateWeekDaySel_Date;
	RTC_AlarmStructure.RTC_AlarmMask				= RTC_AlarmMask_DateWeekDay;
	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);

	// Enable the alarm
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

	// Enable the RTC Alarm A interrupt
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	// Set the time to 01h 00mn 00s AM
	RTC_TimeStructure.RTC_H12     					= RTC_H12_AM;
	RTC_TimeStructure.RTC_Hours   					= 0x01;
	RTC_TimeStructure.RTC_Minutes 					= 0x00;
	RTC_TimeStructure.RTC_Seconds 					= 0x00;

	RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);

	// Clear the Alarm A Pending Bit
	RTC_ClearITPendingBit(RTC_IT_ALRA);
} // end of fv_set_RTC -------------------------------------


/***********************************************************
	Converts the ADC results into meaningful data.
***********************************************************/
static void			fv_read_ADC(uint8_t u8_type)
{
	u16_cell_voltage			= fu16_scale_reading(u16a_adc_buf[CELL_VCELL_INDEX], CELL_VOLT_MAX_INPUT);

	/** The high byte of u16_cell_voltage appears to be always zero regardless of the value it's been set to! **/

	s16_cell_temperature		= fs16_temperature_convert(u16a_adc_buf[CELL_TCELL_INDEX]);

	// set the load state
	if (u16a_adc_buf[CELL_VCELL_INDEX] > (CELL_MAX_MV * ADC_MAX) / CELL_VOLT_MAX_INPUT)
	{
		// turn on the load
		u8_load_state			= 1;					// can also be set and cleared by Battery Manager
	}
	else if (u16a_adc_buf[CELL_VCELL_INDEX] < ((CELL_MAX_MV - CELL_LOAD_DELTA_MV) * ADC_MAX) / CELL_VOLT_MAX_INPUT)
	{
		u8_load_state			= 0;
	}
	// 4 Jul 2017, Dave: added a compiler directive to disable dynamic balancing.
#if (0 != DYNAMIC_BALANCING)
		// are we > 50mv above a valid mean
	if ( (u16_mean_cell_mv > CELL_LOAD_MIN_MV)
				&& (u16a_adc_buf[CELL_VCELL_INDEX] > ((u16_mean_cell_mv + CELL_LOAD_DELTA_MV) * ADC_MAX) / CELL_VOLT_MAX_INPUT) )
	{
		u8_load_state			= 1;
	}
#endif	// DYNAMIC_BALANCING

	// don't save power if we have the load on.
	if (0 != u8_load_state)
	{
		u8_sleep_not_stop		= 1;
	}
	else if (u8_comms_mode == COMMS_IDLE)
	{
		u8_sleep_not_stop		= 0;
	}

	// set the load output
	GPIO_WriteBit(CELL_LOAD_PORT, CELL_LOAD_PIN, u8_load_state);

	// set the alarm state
	u8_alarm_state				= 0;
	if (u16a_adc_buf[CELL_VCELL_INDEX] < (CELL_ALARM_LO_MV * ADC_MAX) / CELL_VOLT_MAX_INPUT)
	{
		u8_alarm_state			|= ALRM_UNDERVOLT;
	}
	if (u16a_adc_buf[CELL_VCELL_INDEX] > (CELL_ALARM_HI_MV * ADC_MAX) / CELL_VOLT_MAX_INPUT)
	{
		u8_alarm_state			|= ALRM_OVERVOLT;
	}
	if (s16_cell_temperature > CELL_MAX_TEMP)
	{
		u8_alarm_state			|= ALRM_OVERTEMP;
	}
} // end of fv_read_ADC ------------------------------------


/***********************************************************
	@brief	scales the ADC value linearly.
	will return u16_max_val when u16_adc_val = ADC_MAX
***********************************************************/
static uint16_t		fu16_scale_reading(uint16_t u16_adc_val, uint16_t u16_max_val)
{
	uint16_t		u16_ret_val;

	u16_ret_val			= (( (uint32_t) u16_adc_val * u16_max_val) / ADC_MAX);

	return u16_ret_val;
} // end of fu16_scale_reading -----------------------------


/***********************************************************
	@brief	looks after the communications.
	triggered by a new byte arriving or a timeout.
***********************************************************/
static void			fv_process_comms(void)
{
	uint8_t		u8_num_bytes, u8_i;
	uint8_t		u8a_buf[6];									// used to store the decoded bytes

	// check for new data in the receiver
	if (u8_UART1_rx_wp != u8_UART1_rx_rp)
	{
		if (u8_UART1_rx_wp > u8_UART1_rx_rp)
		{
			u8_num_bytes							= (u8_UART1_rx_wp - u8_UART1_rx_rp);
		}
		else
		{
			u8_num_bytes							= (RX_BUFFER_SIZE - u8_UART1_rx_rp) + u8_UART1_rx_wp;
		}

		// check if a chunk of bytes received
//		if ((u8_num_bytes % 6) == 0)
		// 30 Jun 2017, Dave - changed check for chunk of bytes so it copes with extras.
		if (u8_num_bytes >= 6)
		{
			// check if chunk is from master
			fu8_fec_decode(&u8a_USART1_rx_buf[u8_UART1_rx_rp], &u8a_buf[0], 6);

			if ( ((u8a_buf[2] & ALRM_ALL_BITS) == ALRM_ALL_BITS)	// no cell can have all 3 alarm bits set at once
					// 3 Jul 2017, Dave -  added a test to ensure both pairs of bytes match
					&& (u8a_buf[0] == u8a_buf[2])
					&& (u8a_buf[1] == u8a_buf[3]) )
//					&& ((u8a_buf[0] & 0x60) == 0) )			// the cell voltage can never get high enough to set these bits; (>8v)
			{
				u8_valid_header						= 1;

				/*	master sends mean cell voltage so that cells can be balanced during charge. */
				// 30 Jun 2017, Dave - corrected the mask bits for the high byte of u16_mean_cell_mv from master.
				u16_mean_cell_mv					= (((u8a_buf[0] & 0x1F) << 8) + u8a_buf[1]);
			}
			else		// if it's not from the master, it must be from a previous cell.
			{
				// increment cell counter, used to determine what cell number we are.
				++u8_cell_counter;
			}

			// copy the chunk to the tx buffer
			for (u8_i = 0; u8_i < 6; ++u8_i)
			{
				u8a_USART1_tx_buf[u16_UART1_tx_wp]	= u8a_USART1_rx_buf[u8_UART1_rx_rp];
				if (u16_UART1_tx_wp < (TX_BUFFER_SIZE - 1) )
				{
					++u16_UART1_tx_wp;
				}
				if (u8_UART1_rx_rp < (RX_BUFFER_SIZE - 1) )
				{
					++u8_UART1_rx_rp;
				}
			} // end of for each byte of chunk

			u8_UART1_rx_rp							= 0;
			u8_UART1_rx_wp							= 0;

			u16_comms_intrvl_tmr					= 0;
		} // end of if a chunk received
	} // end of if bytes to process
} // end of fv_process_comms -------------------------------


/***********************************************************
	@brief	appends data to the end of the tx buffer.
***********************************************************/
static void			fv_append_data(void)
{
	uint8_t		u8a_buf[6];
	uint8_t		u8_i;

	u8_comms_mode							= COMMS_TRANSMITTING;

	// add the cell data
	if (u16_cell_voltage > CELL_VOLTAGE_MASK)
	{
		u16_cell_voltage					= CELL_VOLTAGE_MASK;
	}
	u8a_buf[0]								= (u16_cell_voltage >> 8);
	u8a_buf[1]								= (uint8_t) u16_cell_voltage;

	// limit cell temperature to the available bits
	if (s16_cell_temperature < 0)
	{
		s16_cell_temperature				= 0;
	}
	if (s16_cell_temperature > CELL_TEMP_MASK)
	{
		s16_cell_temperature				= CELL_TEMP_MASK;			// 51.1c
	}
	u8a_buf[2]								= (s16_cell_temperature >> 8);
	u8a_buf[3]								= (uint8_t) s16_cell_temperature;

	// set the flag bits
	if (u8_load_state != 0)
	{
		u8a_buf[0]							|= 0x80;		// set the msb if the load is on
	}
	u8a_buf[2]								|= u8_alarm_state;

	// add the fec bytes
	fu8_fec_encode(&u8a_buf[0], 4);

	// copy to the tx buffer
	for (u8_i = 0; u8_i < 6; ++u8_i)
	{
		u8a_USART1_tx_buf[u16_UART1_tx_wp]	= u8a_buf[u8_i];
		++u16_UART1_tx_wp;
		if (u16_UART1_tx_wp > TX_BUFFER_SIZE)
		{
			u16_UART1_tx_wp					= 0;
		}
	}

	// clear the interval timer because we're sending data now.
	u16_comms_intrvl_tmr					= 0;

	// if cell number has not yet been set then do it now if the message stream was initiated by the master.
	if ( (u8_cell_number == CELL_NUM_NOT_SET)
			&& (0 != u8_valid_header) )
	{
		u8_cell_number						= (u8_cell_counter + 1);
	}
	u8_cell_counter							= 0;			// reset the cell counter ready for the next message stream.
	u8_valid_header							= 0;
} // end of fv_append_data ---------------------------------


/***********************************************************
	@brief	sets the appropriate led to on.
***********************************************************/
static void			fv_show_LED(void)
{
	// test if alarm condition active
	// and turn on appropriate LED.
	if (u8_alarm_state != 0)
	{
		GPIO_WriteBit(ALARM_LED_PORT, ALARM_LED_PIN, 1);
	}
	else
	{
		GPIO_WriteBit(HEART_LED_PORT, HEART_LED_PIN, 1);	// Heartbeat LED
	}
} // end of fv_show_LED ------------------------------------



/*********** Interrupt Handlers ***************************/


/**********************************************************/
/*        STM32F0xx Peripherals Interrupt Handlers        */
/*  The Interrupt Handler for the used peripherals (PPP)  */
/*  for the available peripheral interrupt handler's name */
/*  refer to the startup file (startup_stm32f0xx.s).      */
/**********************************************************/


/***********************************************************
	@brief	This function handles System tick timer.
***********************************************************/
void			SysTick_Handler(void)
{
	NVIC_ClearPendingIRQ(SysTick_IRQn);

	fv_show_LED();

	fv_set_ADC(u8_hardware_type);
} // end of SysTick_IRQHandler -----------------------------


/***********************************************************
  * @brief  This function handles USART1 global interrupt request.
***********************************************************/
void			USART1_IRQHandler(void)
{
	//flash the appropriate light to show activity.
	fv_show_LED();

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		u8a_USART1_rx_buf[u8_UART1_rx_wp]	= USART_ReceiveData(USART1);
		++u8_UART1_rx_wp;
		if (u8_UART1_rx_wp >= RX_BUFFER_SIZE)
		{
			u8_UART1_rx_wp					= 0;
		}

		u8_comms_mode						= COMMS_RECEIVING;

		// reset the timer to zero
		TIM_SetCounter(TIM14, 0);

		u8_comms_tick_f						= 1;
	} // end of if byte received

#if 0	// don't need the tx interrupt as we'll use DMA to handle the transfer.
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
	} // end of if byte sent
#endif

	if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
	{
	    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
	    USART_ClearITPendingBit(USART1, USART_IT_TC);

		u8_comms_mode						= COMMS_IDLE;

		// end of transmitting so return to minimum power drain
		TIM_Cmd(TIM14, DISABLE);
		u16_UART1_tx_wp						= 0;
		if (0 == u8_load_state)
		{
			u8_sleep_not_stop				= 0;			// we're done sleeping, can stop now
		}
		u8_comms_tick_f						= 1;			// trigger a tick so main will set stop mode	TODO: may not need this
	} // end of if last byte sent

	// clear the overflow flag as it can stop RXNE being set.
	USART_ClearFlag(USART1, USART_FLAG_ORE);
} // end of USART1_IRQHandler ------------------------------


/***********************************************************
  * @brief  This function handles RTC Auto wake-up interrupt request.
  *
  * The RTC is the only way to periodically wake-up from STOP.
  * Starts the ADC to take a set of readings.
  *********************************************************/
void			RTC_IRQHandler(void)
{
	if (RTC_GetITStatus(RTC_IT_ALRA) != RESET)
	{
		// Clear the Alarm A Pending Bit
		RTC_ClearITPendingBit(RTC_IT_ALRA);

		fv_show_LED();

		fv_set_ADC(u8_hardware_type);

		u16_comms_intrvl_tmr			+= INTERVAL_SECONDS;

		// Clear EXTI line17 pending bit
		EXTI_ClearITPendingBit(EXTI_Line17);
	} // end of if RTC Alarm
} // end of RTC_IRQHandler ---------------------------------


/***********************************************************
 * @brief	DMA Transfer Complete interrupt for ADC channel
 *
 * when the DMA has finished transferring all ADC channels
 * it sets the systick flag so that we process the new data.
 **********************************************************/
void			DMA1_Channel1_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_IT_TC1))
	{
		ADC_Cmd(ADC1, DISABLE);

		u8_sys_tick_f			= 1;						// signal main to process readings

		DMA_ClearITPendingBit(DMA1_IT_GL1);
	}
} // end of DMA1_Channel1_IRQHandler -----------------------


/***********************************************************
  * @brief  This function handles External lines 0 interrupt request.
  *********************************************************/
void			EXTI0_1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
} // end of EXTI0_1_IRQHandler -----------------------------


/***********************************************************
  * @brief  This function handles External lines 3 interrupt request.
  *
  * this detects activity on the RX line to wake from STOP.
  * This is the preferred solution triggered off the RX pin.
  *
  * As each byte arrives it will reset the timer so we'll only
  * get a timer interrupt when bytes have stopped arriving.
  *********************************************************/
void			EXTI2_3_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line3);

		if (0 == u8_sleep_not_stop)
		{
			// set the timeout.
			TIM_SetCounter(TIM14, 0);
			TIM_SetCompare1(TIM14, UART_SLEEP_MS);			// generate an interrupt in x ms
			TIM_Cmd(TIM14, ENABLE);
			u8_sleep_not_stop		= 1;
			u8_comms_tick_f			= 1;
		} // end of if we were stopped when this triggered
	} // end of if line 3
} // end of EXTI2_3_IRQHandler -----------------------------


/***********************************************************
 *	@brief	This sets the system to return to STOP instead of SLEEP
 *
 *	When we've finished talking over the UART we can power down
 *	all the peripherals to minimise power use, STOP mode.
 *
 *	During Receiving this interrupt will fire when bytes stop arriving.
 *	This then triggers the on-sending of the data packets.
 **********************************************************/
void			TIM14_IRQHandler(void)
{
//	if (u8_comms_mode == COMMS_RECEIVING)
	{
		// check if bytes to send.
		if (0 != u16_UART1_tx_wp)
		{
			// timed out after last byte received so transmit now.
			fv_append_data();

			// set up DMA transmit
			fv_USART1_send(u16_UART1_tx_wp, &u8a_USART1_tx_buf[0]);
		}

		// reset the receive and transmit buffer pointers.
		u8_UART1_rx_rp				= 0;
		u8_UART1_rx_wp				= 0;
		u16_UART1_tx_wp				= 0;

		TIM_Cmd(TIM14, DISABLE);
		TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
	} // end of if receiving
} // end of TIM14_IRQHandler -------------------------------


/***********************************************************
 * @brief	DMA Transfer Complete interrupt for UART channel
 *
 * when the DMA has finished transferring all UART bytes
 * it clears the u8_sleep_not_stop flag to save power.
 **********************************************************/
void			DMA1_Channel2_3_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_IT_TC2))
	{
		// This interrupt is flagged when the DMA has finished sending bytes to the UART.
		// The final byte is now in the UART buffer ready to place into the shift register.
		// Now we need to enable the Tx shift register empty interrupt to be sure all the bytes are gone.
        USART_ITConfig(USART1, USART_IT_TC, ENABLE);

		DMA_ClearITPendingBit(DMA1_IT_GL2);
	}
} // end of DMA1_Channel2_IRQHandler -----------------------



