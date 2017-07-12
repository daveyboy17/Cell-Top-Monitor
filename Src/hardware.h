/***********************************************************
	@file		hardware.h
    @author		David Burkitt
    @date		2 April, 2015
 **********************************************************/


#define BOARD_VERSION			(2)		// Rev 2.x
#define SW_VERSION				(3)		// Rev 3

// ADC ref is 2.5v, cell volt measure has a 2:1 divider so max should be very close to 5v.
#define CELL_VOLT_MAX_INPUT		(5000)			// *CALIBRATION VALUE* set what ADC_MAX means as a cell voltage in mV

#define CELL_ALARM_HI_MV		(3600)						// 3.6v - turn on alarm led above this
#define CELL_MAX_MV				(3550)						// 3.55v - turn on load and alarm led above this
#define CELL_LOAD_DELTA_MV		(50)						// 50mv
#define CELL_LOAD_MIN_MV		(2900)						// 2.9v - any lower than this and load will not turn on.
#define CELL_ALARM_LO_MV		(2600)						// 2.6v - turn on alarm led below this
#define CELL_MAX_TEMP			(450)						// 45.0c

#define HW_TYPE_UNKNOWN			(0)
#define HW_TYPE_CELL_MONITOR	(1)

#define HEART_LED_PORT			GPIOF
#define HEART_LED_PIN			GPIO_Pin_0
#define ALARM_LED_PORT			GPIOF
#define ALARM_LED_PIN			GPIO_Pin_1
#define CELL_LOAD_PORT			GPIOA
#if (1 == BOARD_VERSION)
#define CELL_LOAD_PIN			GPIO_Pin_6
#else
#define CELL_LOAD_PIN			GPIO_Pin_5
#endif

#define TX_BUFFER_SIZE			(200)						// 102 for 48v, 198 for 96v = ((number of cells + 1) x 6), 16 cells for 48v, 32 for 96v
#define RX_BUFFER_SIZE			(10)						// only needs to hold 1 6byte block
#define COMMS_IDLE				(0)
#define COMMS_RECEIVING			(1)
#define COMMS_TRANSMITTING		(2)
#define	CELL_VOLTAGE_MASK		(0x1FFF)
#define CELL_TEMP_MASK			(511)	// 0x1FF
#define CELL_NUM_NOT_SET		(0xFF)						// default to a number that cannot be achieved
// alarm constants - use the upper 3 bits of the temperature high byte
#define ALRM_UNDERVOLT			(0x20)
#define ALRM_OVERVOLT			(0x40)
#define ALRM_OVERTEMP			(0x80)
#define ALRM_ALL_BITS			(ALRM_UNDERVOLT | ALRM_OVERVOLT | ALRM_OVERTEMP)

#define WATCHDOG_MAIN			(0x0001)
#define WATCHDOG_SYSTICK		(0x0002)
#define WATCHDOG_ALL			(WATCHDOG_MAIN | WATCHDOG_SYSTICK)
#define LSI_FREQ				(40000)			// 40kHz

#define MAX_ADC_CHANS			(8)
#define ADC_MAX					(4095)			// max value that the ADC can return
#define ADC_FAULT				(0x3FFF)		// value to return for overtemp condition, top 2 bits are reserved
#define TEMP_DP					(1)				// resolve temperature to this many d.p.

#define CELL_TCELL_INDEX		(1)
#define CELL_VCELL_INDEX		(0)		// DMA places the highest channel in the zero index
#define CELL_NUM_ADC			(2)


