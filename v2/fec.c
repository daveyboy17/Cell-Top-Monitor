/*******************************************************************************
 * FileName 	: fec.c
 * Title		: house automation
 * Author		: Robert Stuart
 * Created		: 2013 February 27
 * Last Edit	: 2014 August 19, David Burkitt
 * Company		: DARC Technologies Ltd.
 * System		: see main.c
 * History		:
 * Description	: Error detection and correction
 *******************************************************************************/


#include	"stm32f0xx.h"
#include	"hardware.h"
#include	"fec.h"


uint8_t		fu8_fec_encode(uint8_t p_msg[], uint8_t u8_msg_len);
uint8_t		fu8_fec_decode(volatile uint8_t en_msg[], uint8_t de_msg[], uint8_t u8_msg_len);
void		fv_fec_byte_decode(uint16_t *u16_code_word);


//#define INTERLACING		1		// comment this to disable bitwise interlacing		// TODO:


// used to generator the 12-bit code-words from the 8-bit data-bytes
const uint8_t		u8a_generator_matrix[4]				= { 0xD5, 0xB3, 0x0F, 0xE8 };

// used to determine which bits have been corrupted
const uint16_t		u16a_parity_check_matrix[4]			= { 0x8E8, 0x40F, 0x2B3, 0x1D5 };

// used to determine which bits are to be flipped
const uint16_t		u16a_coset_leader_table[16]			= { 0x000, 0x800, 0x400, 0x008,
														    0x200, 0x020, 0x002, 0x208,
														    0x100, 0x040, 0x004, 0x084,
														    0x010, 0x080, 0x001, 0x801 };


/*******************************************************************************
 * Encodes with low-density parity-check codes
 * The first byte (header) is not encoded, thereafter the 8-bits are encoded
 * into 12-bits. The bytes are sent in the order;
 *   B1   B2   LDPC   B3   B4   LDPC   ...
 * Hence every 3rd-byte has the error correction bits for the previous 2-bytes.
 * The interleaving is done over blocks of 6-bytes.
 *
 * Returns the length of the encoded message
 *******************************************************************************/
uint8_t		fu8_fec_encode(uint8_t p_msg[], uint8_t u8_msg_len)
{
	uint16_t	u16a_code_word[LDPC_BUF_SIZE];
	uint8_t		u8_i, u8_encode, u8_g_matrix, u8_j, u8_g, u8_h;
#ifdef INTERLACING
	uint8_t		u8_k;
#endif
	uint8_t		u8a_tmp[LDPC_BUF_SIZE];

	// ensure that packet size is modulus 6
	u8_i	= 3 - ((u8_msg_len + 3) & 0x03);
	while (u8_i) {
		// pad-out with 0xFF
		p_msg[u8_msg_len]	= 0xFF;
		u8_msg_len++;
		u8_i--;
	} // end of while padding bytes needed.

	for (u8_i = 0; u8_i < u8_msg_len; u8_i++) {
		// initialise code-word with orginial byte
		u16a_code_word[u8_i]	= p_msg[u8_i];

		// encode using the generator matrix
		for (u8_g_matrix = 0; u8_g_matrix < 4; u8_g_matrix++) {
			// bit-mask byte with generator matrix
			u8_encode	= u8a_generator_matrix[u8_g_matrix] & p_msg[u8_i];

			// calculate odd-parity
			u8_encode	^= u8_encode >> 8;
			u8_encode	^= u8_encode >> 4;
			u8_encode	^= u8_encode >> 2;
			u8_encode	^= u8_encode >> 1;

			// modulus 2
			u8_encode	&= 0x01;

			// calculate code-word
			u16a_code_word[u8_i]	+= u8_encode << (u8_g_matrix + 8);
		}
	} // end of for each byte in message

	// increase message length for parity bits
	u8_msg_len	= (3 * u8_msg_len) >> 1;

	u8_encode	= 0;
	// new message with added ldpc
	for (u8_i = 0; u8_i < u8_msg_len; u8_i += 3) {		// create information bit array

		// convert code-words into data-bytes
		u8a_tmp[u8_i+0]	= u16a_code_word[u8_encode];
		u8a_tmp[u8_i+1]	= u16a_code_word[u8_encode + 1];
		u8a_tmp[u8_i+2]	= (u16a_code_word[u8_encode] >> 8) + ((u16a_code_word[u8_encode + 1] & 0x0F00) >> 4);
		u8_encode	+= 2;
	} // end of for each 3byte block in message

	// interleaving bits across the message in block sizes of six to provide burst-noise immunity, step across the 8-bits in each byte
	for (u8_i = 0; u8_i < u8_msg_len; u8_i+= 6) {
		// reset indexes
		u8_g	= 0;
		u8_h	= 0;

		for (u8_j = 0; u8_j < 6; u8_j++) {
#ifdef INTERLACING
			// reset bytes
			p_msg[u8_i + u8_j]	= 0;

			for (u8_k = 0; u8_k < 8; u8_k++) {
				// interleave bits
				p_msg[u8_i + u8_j]	+= ( (u8a_tmp[u8_i + u8_g] >> u8_h) & 0x01 ) << u8_k;

				// increment indexex
				if (++u8_g > 5) {
					u8_g	= 0;
					u8_h++;
				}
			} // end of for each bit in a byte
#else
			p_msg[u8_i + u8_j]	= u8a_tmp[u8_i + u8_j];
#endif
		} // end of for each byte in a block
	} // end of for each block of 6 bytes

	// return length of encoded message
	return u8_msg_len;
} // end of fu8_fec_encode function --------------------


/*******************************************************************************
 * Decodes the low-density parity-check codes via lookup table
 * Applies the parity check matrix to calculate the syndrome which is used, via
 * a lookup table for error correction.
 *
 * Returns the length of the decoded message
 *******************************************************************************/
uint8_t		fu8_fec_decode(volatile uint8_t en_msg[], uint8_t de_msg[], uint8_t u8_msg_len)
{
	static uint8_t	u8a_decode[LDPC_BUF_SIZE];
//	static uint8_t	u8a_save[LDPC_BUF_SIZE];
	uint16_t		u16a_code_word[4];
	uint8_t			u8_i, u8_j, u8_len;
#ifdef INTERLACING
	uint8_t			u8_k, u8_g, u8_h;
#endif
	uint8_t			u8a_tmp[LDPC_BUF_SIZE];

	// only decode in blocks of six
	u8_msg_len	= ((u8_msg_len / 6) * 6);
	u8_len		= 0;

	if (u8_msg_len > 6) {
		u8_i	= u8_msg_len - 6;
	}
	else {
		u8_i	= 0;
	}

#if 0
	// check for previous decoding
	for (u8_j = 0; u8_j < u8_msg_len; u8_j += 6) {
		if ( (u8a_save[u8_j+0] == en_msg[u8_j+0]) && (u8a_save[u8_j+1] == en_msg[u8_j+1]) && (u8a_save[u8_j+2] == en_msg[u8_j+2]) &&
			 (u8a_save[u8_j+3] == en_msg[u8_j+3]) && (u8a_save[u8_j+4] == en_msg[u8_j+4]) && (u8a_save[u8_j+5] == en_msg[u8_j+5]) ) {
			// indexing translations, used to calculate the decoded message length
			u8_len	= ((u8_i << 1) + 1) / 3;
			u8_i	+= 6;
		}
	}

	// save previous message
	for (u8_j = 0; u8_j < u8_msg_len; u8_j++) {
		u8a_save[u8_j]	= en_msg[u8_j];
	}
#endif

	// re-distribute the bits into their correct byte
//	for (; u8_i < u8_msg_len; u8_i += 6) {
	for (u8_i = 0; u8_i < u8_msg_len; u8_i += 6) {
		for (u8_j = 0; u8_j < 6; u8_j++) {
#ifdef INTERLACING
			// reset bytes
			u8a_tmp[u8_i + u8_j]	= 0;

			// reset indexes
			u8_g		= 0;
			u8_h		= u8_j;

			for (u8_k = 0; u8_k < 8; u8_k++) {
				// remove interleaving
				u8a_tmp[u8_i + u8_j]	+= ( (en_msg[u8_i + u8_g] >> u8_h) & 0x01 ) << u8_k;

				// increment indexes
				u8_h		+= 6;
				if (u8_h > 7) {
					u8_g++;
					u8_h	-= 8;
				}
			} // end of for each bit of a byte
#else
			u8a_tmp[u8_i + u8_j]	= en_msg[u8_i + u8_j];
#endif
		} // end of for each byte in a block

		// code-words
		u16a_code_word[0]	= u8a_tmp[u8_i + 0] + ( (u8a_tmp[u8_i + 2] & 0x0F) << 8);
		u16a_code_word[1]	= u8a_tmp[u8_i + 1] + ( (u8a_tmp[u8_i + 2] & 0xF0) << 4);
		u16a_code_word[2]	= u8a_tmp[u8_i + 3] + ( (u8a_tmp[u8_i + 5] & 0x0F) << 8);
		u16a_code_word[3]	= u8a_tmp[u8_i + 4] + ( (u8a_tmp[u8_i + 5] & 0xF0) << 4);

		for (u8_j = 0; u8_j < 4; u8_j++) {
			// decode byte in packet
			fv_fec_byte_decode(&u16a_code_word[u8_j]);

			// reduce to 8-bit
			u16a_code_word[u8_j] &= 0xFF;
		} // end of for each block of 4 bytes

		// decode
		u8a_decode[u8_len + 0]	= u16a_code_word[0];
		u8a_decode[u8_len + 1]	= u16a_code_word[1];
		u8a_decode[u8_len + 2]	= u16a_code_word[2];
		u8a_decode[u8_len + 3]	= u16a_code_word[3];

		// since we've decoded another 4 bytes add 4 to the length.
		u8_len	+= 4;
	} // end of for each block of 6 bytes

	// decoded message length
	u8_msg_len	= u8_len;

	for (u8_i = 0; u8_i < u8_msg_len; u8_i++) {
		de_msg[u8_i]		= u8a_decode[u8_i];
	}

	// return decoded message length
	return u8_msg_len;
} // end of fu8_fec_decode function --------------------


/*******************************************************************************
 * Decodes a 12-bits into a 8-bits using the fec
 *******************************************************************************/
void		fv_fec_byte_decode(uint16_t *u16_code_word)
{
	uint8_t			u8_p_matrix, u8_syndrome = 0;
	uint16_t		u16_decode;

	for (u8_p_matrix = 0; u8_p_matrix < 4; u8_p_matrix++) {
		// bit-mask byte with generator matrix
		u16_decode		= u16a_parity_check_matrix[u8_p_matrix] & (*u16_code_word);

		// calculate odd-parity
		u16_decode		^= u16_decode >> 8;
		u16_decode		^= u16_decode >> 4;
		u16_decode		^= u16_decode >> 2;
		u16_decode		^= u16_decode >> 1;

		// modulo 2
		u16_decode		&= 1;

		// sydrome
		u8_syndrome		+= u16_decode << u8_p_matrix;
	}

	// error correction to the codeword, XOR the codeword with the coset leader
	(*u16_code_word)	^= u16a_coset_leader_table[u8_syndrome];
} // end of fv_fec_byte_decode function ----------------




/* END OF FILE */
