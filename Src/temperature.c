/***********************************************************
	File:		temperature.c
    Author:		David Burkitt
	Date:		10 June, 2015

	Changes:

 **********************************************************/


/************** Includes **********************************/
#include		"stm32f0xx.h"
#include		"hardware.h"
#include		"temperature.h"


/************** Defines ***********************************/
#define		ZERO_DEGREES_ADC		2469


/************** Prototypes ********************************/


/************** Variables *********************************/
// lookup table for 100k pull-up resistor with 47k NTC and 12bit ADC
int16_t	s16a_temp_lookup[][2] = {
		{	4096,				-500,	},		// reasonable down to -40
		{	3509,				-250,	},
		{	2945,				-100,	},
//		{	ZERO_DEGREES_ADC,	0,		},
		{	1513,				200,	},
		{	1126,				300,	},
		{	823,				400,	},
		{	596,				500,	},		// LiFePO4 should not be charged above 45deg
		{	432,				600,	},
		{	314,				700,	},
		{	230,				800,	},
		{	171,				900,	},
		{	0,					1290,	},		// this gives us the right slope out to ~103deg
};


/***********************************************************
	@brief	Converts the ADC value into a temperature.
***********************************************************/
int16_t		fs16_temperature_convert(uint16_t u16_adc)
{
	int16_t		s16_upper_adc, s16_lower_adc;
	int16_t		s16_temp_1, s16_temp_2;
	int16_t		s16_ret_val;
	uint8_t		u8_index	= 0;

	// check for error values first
	if ( (u16_adc > 4050)
			|| (u16_adc < 50) )
	{
		s16_ret_val			= ADC_FAULT;					// return the error value
	}
	else
	{
		// find the lower adc value to set the index
		while (s16a_temp_lookup[u8_index][0] > u16_adc)
		{
			++u8_index;
		}

		// check for an exact match
		if (s16a_temp_lookup[u8_index][0] == u16_adc)
		{
			s16_ret_val		= s16a_temp_lookup[u8_index][1];
		}
		else
		{
			// set the above and below values
			s16_lower_adc	= s16a_temp_lookup[u8_index][0];		// 1513
			s16_upper_adc	= s16a_temp_lookup[u8_index - 1][0];	// 2945
			s16_temp_1		= s16a_temp_lookup[u8_index][1];		// 200
			s16_temp_2		= s16a_temp_lookup[u8_index - 1][1];	// -100

			// then linear interpolate
			s16_ret_val		= s16_temp_1 - ( ( (s16_temp_1 - s16_temp_2) * (u16_adc - s16_lower_adc) ) / (s16_upper_adc - s16_lower_adc) );
		} // end of else not an exact match
	} // end of else valid adc value

	return s16_ret_val;
} // end of fs16_temperature_convert -----------------------


