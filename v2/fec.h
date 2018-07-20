/*******************************************************************************
 * FileName		: fec.h
 * Title		: house automation
 * Author		: Robert Stuart
 * Created		: 2013 February 27
 * Last Edit	: 2013 May 03, Robert Stuart
 * Company		: DARC Technologies Ltd.
 * System		:
 * Description	: Error detection, correction and interleaving
 *******************************************************************************/

/*******************************************************************************
 * fu8_fec_encode(volatile uint8_t p_msg[], uint8_t u8_msg_len);
 *      p_msg[]			the array/pointer that is to be encoded with ldpc and interleaving
 *      u8_msg_len		the length of *p_msg
 *      Returns the length of the encoded message; *p_msg
 *
 * fu8_fec_decode(volatile uint8_t en_msg[], uint8_t de_msg[], uint8_t u8_msg_len);
 *      en_msg[]		the encoded array/pointer, this is the raw string of bytes
 *      de_msg[]		the decoded array/pointer
 *      u8_msg_len		the length of en_msg[]
 *      Returns the length of the encoded message; *de_msg
 *
 *******************************************************************************/


#ifndef FILE_FEC_H                          /* sentinel */
#define FILE_FEC_H


#define LDPC_BUF_SIZE		(6)


// prototypes call from outside
extern uint8_t		fu8_fec_encode(uint8_t p_msg[], uint8_t u8_msg_len);
extern uint8_t		fu8_fec_decode(volatile uint8_t en_msg[], uint8_t de_msg[], uint8_t u8_msg_len);
extern void			fv_fec_byte_decode(uint16_t *u16_code_word);


#endif                                      /* sentinel */


/* END OF FILE */
