/***********************************************************
    Filename:	main.h
    Author:		David Burkitt
    Date:		2 April, 2015

    Changes:
***********************************************************/


#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED


#define BAUDRATE			(19200)
//#define BAUDRATE			(9600)							// hardware on early proto's couldn't hack it at 19200

#define DYNAMIC_BALANCING	(1)								// 0 = Top balancing only, 1 = balance if > mean + 50mV.

#define INTERVAL_SECONDS	(2)								// 2-59 - sample the cell every x seconds. NOTE: need to adjust the Watchdog interval as well.
#define AUTOSEND_SECONDS	(60)							// time to wait after last message before sending unheeded.
#define UART_SLEEP_MS		(20)


void			fv_USART1_send(uint16_t u16_num_bytes, uint8_t u8a_source[]);
void			fv_print_number(uint16_t u16_num);


#endif // MAIN_H_INCLUDED
