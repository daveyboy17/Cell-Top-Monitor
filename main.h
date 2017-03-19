/***********************************************************
    Filename:	main.h
    Author:		David Burkitt
    Date:		2 April, 2015

    Changes:
***********************************************************/


#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED


//#define BAUDRATE			(19200)
#define BAUDRATE			(9600)							// TODO: hardware on early proto's couldn't hack it at 19200
#define UART_SLEEP_MS		(20)
#define INTERVAL_SECONDS	(2)								// 2-59 - sample the cell every x seconds. NOTE: need to adjust the Watchdog interval as well.
#define AUTOSEND_SECONDS	(60)							// time to wait after last message before sending unheeded.
#define PROPORTIONAL_LOAD	(0)								// set this to 1 to enable PWM of the load FET if connected to PA6


uint16_t		fu16_get_hw_type(void);
void			fv_USART1_send(uint16_t u16_num_bytes, uint8_t u8a_source[]);
void			fv_print_number(uint16_t u16_num);


#endif // MAIN_H_INCLUDED
