/***********************************************************
    Filename:	main.h
    Author:		David Burkitt
    Date:		2 April, 2015

    Changes:
    			2 Aug 2017, Dave
    			removed sleeping during balancing as it interferes with comms.
    			shortened UART_SLEEP_MS to 10ms for 19200 baud option.
    			moved resetting the comms timer to after a block of 6 received.
    			27 Jul 2017, Dave
    			shortened the autosend timeout to 30s.
***********************************************************/


#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED


#define BAUDRATE			(19200)
//#define BAUDRATE			(9600)							// hardware on early proto's couldn't hack it at 19200

#define DYNAMIC_BALANCING	(0)								// 0 = Top balancing only, 1 = balance if > mean + 50mV.

#define INTERVAL_SECONDS	(2)								// 2-59 - sample the cell every x seconds. NOTE: need to adjust the Watchdog interval as well.
#define AUTOSEND_SECONDS	(30)							// time to wait after last message before sending unheeded.
#if (BAUDRATE == 19200)
#define UART_SLEEP_MS		(5)								// in theory 4ms should be long enough @ 19200
#else
#define UART_SLEEP_MS		(20)							// in theory 7ms should be long enough @ 9600
#endif


void			fv_USART1_send(uint16_t u16_num_bytes, uint8_t u8a_source[]);
void			fv_print_number(uint16_t u16_num);


#endif // MAIN_H_INCLUDED
