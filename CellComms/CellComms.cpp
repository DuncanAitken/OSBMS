/**********************************************************
  CellComms.cpp - Library for reading the OSBMS Cell Monitors.
  Created by David Burkitt, March 8, 2017.
  Released into the public domain.
**********************************************************/

#include "Arduino.h"
#include "CellComms.h"


#define  LDPC_BUF_SIZE 		50
//#define _MAX_RX_BUFF 	102				// override the default RX buffer size - (NUM_CELLS + 1) x 6


// used to determine which bits have been corrupted
const unsigned int    u16a_parity_check_matrix[4]     = { 0x08E8, 0x040F, 0x02B3, 0x01D5 };

// used to determine which bits are to be flipped
const unsigned int    u16a_coset_leader_table[16]     = { 0x000, 0x800, 0x400, 0x008,
													  0x200, 0x020, 0x002, 0x208,
													  0x100, 0x040, 0x004, 0x084,
													  0x010, 0x080, 0x001, 0x801 };
													  
// used to generator the 12-bit code-words from the 8-bit data-bytes
const byte		u8a_generator_matrix[4]				= { 0xD5, 0xB3, 0x0F, 0xE8 };


byte _tx_packet[] = {0x9f, 0xff, 0xff, 0xff, 0xff, 0xff};

struct _cellData
{
	unsigned int millivolts;
	unsigned int temperature;
	byte balancing;
	byte overVoltage;
	byte underVoltage;
	byte overTemperature;
} _cellDataArray[NUM_CELLS];

byte cellsRead;
	

// Constructor
CellComms::CellComms(void)
{
  Serial2.begin(CELL_BAUD);
  
  for (int i = 0; i < NUM_CELLS; ++i)
  {
	// TODO: populate the array with some test data
	_cellDataArray[i].millivolts		= (3250 + i);
	_cellDataArray[i].temperature		= (210 + i);
	_cellDataArray[i].balancing			= 0;
	_cellDataArray[i].overVoltage		= 0;
	_cellDataArray[i].underVoltage		= 0;
	_cellDataArray[i].overTemperature	= 0;
  }
} // end of constructor -----------------------------------


/***********************************************************
	@brief	sends the initiator message containing the mean
			cell voltage.
***********************************************************/
void CellComms::sendMillivolts(int millivolts)
{
	// mask off the top 3 bits.
	millivolts					&= 0x1FFF;
	// set the 2 marker bits to identify this as a seed message.
	millivolts					!= 0x6000;
	
	// form into a 4byte buffer and encode
	_tx_packet[0]				= (millivolts >> 8);
	_tx_packet[0]				|= 0x60;		// set the marker bits for a master message
	_tx_packet[1]				= (millivolts);
	_tx_packet[2]				= (millivolts >> 8);
	_tx_packet[2]				|= 0x60;		// set the marker bits for a master message
	_tx_packet[3]				= (millivolts);
	
	fecEncode(_tx_packet, 4);
	
  int bytesSent					= Serial2.write(_tx_packet, 6);

  Serial2.flush();
} // end of sendMillivolts ---------------------------------


/***********************************************************
	readCells
	reads blocks of 6 bytes from serial and decodes them
	into cell data.
***********************************************************/
int CellComms::readCells(void)
{
  const int expected_bytes    					= 6;
  int bytes_available         					= Serial2.available();
  int cellNum                 					= 0;
  int tempMillivolts;
  
  while (bytes_available >= expected_bytes ) {
    int i                     					= 0;
    unsigned char payload[expected_bytes];
    while (i < expected_bytes) {
      byte currByte           					= Serial2.read();
      --bytes_available;
      payload[i]              					= currByte;
      ++i;
    }
    
    // decode the packet
    unsigned char decoded[4];
    fecDecode(&payload[0], &decoded[0], sizeof(payload));
    
	if ((decoded[0] & 0x60) == 0) {
    // Interpret decoded values.
    // Bytes 0 and 1 make up the voltage.
    // Bytes 2 and 3 make up the temperature.
	tempMillivolts									= ((decoded[0] & 0x1F) << 8) + decoded[1];
	if ( (tempMillivolts > 1900)				// The regulator colapses at 2v
			&& (tempMillivolts < 5100) ) {		// the adc will max out at 5v
		_cellDataArray[cellNum].millivolts			= ((decoded[0] & 0x1F) << 8) + decoded[1];
		_cellDataArray[cellNum].temperature			= ((decoded[2] & 0x0F) << 8) + decoded[3];
		_cellDataArray[cellNum].balancing			= ((decoded[0] & 0x80) >> 7);
		_cellDataArray[cellNum].overTemperature		= ((decoded[2] & 0x80) >> 7);
		_cellDataArray[cellNum].overVoltage			= ((decoded[2] & 0x40) >> 6);
		_cellDataArray[cellNum].underVoltage		= ((decoded[2] & 0x20) >> 5);
		} // end of if mV makes sense
	} // end of if valid cell data

	if ((decoded[0] & 0xE0) == 0x60) {
//	if ((decoded[2] & 0xE0) == 0xE0) {
		// reset the cell number as this was the seeding message
		cellNum										= 0;
	}
	else {
		++cellNum;
	}
  } // end of while packet available
  
  cellsRead											= cellNum;
  
  // Read remaining bytes...
  bytes_available             						= Serial2.available(); 
  if (bytes_available > 0) {
    for (int i = 0; i < bytes_available; ++i) {
      byte discard            						= Serial2.read();
    } // end of for each leftover byte
  } // end of if leftover bytes
  
  return cellNum;
} // end of readCells --------------------------------------


/***********************************************************
	@brief	returns the average millivolts of all cells.
***********************************************************/
int CellComms::getCellsAveV(void)
{
	int aveVal			= 0;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		aveVal			+= _cellDataArray[i].millivolts;
	}
	
	return (aveVal / cellsRead);
} // end of getCellsAveV -----------------------------------


/***********************************************************
	@brief	returns the millivolts of the lowest cell.
***********************************************************/
int CellComms::getCellsMinV(void)
{
	int minVal			= _cellDataArray[0].millivolts;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (minVal > _cellDataArray[i].millivolts)
		{
			minVal		= _cellDataArray[i].millivolts;
		}
	}
	
	return minVal;
} // end of getCellsMinV -----------------------------------


/***********************************************************
	@brief	returns the millivolts of the highest cell.
***********************************************************/
int CellComms::getCellsMaxV(void)
{
	int maxVal			= _cellDataArray[0].millivolts;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (maxVal < _cellDataArray[i].millivolts)
		{
			maxVal		= _cellDataArray[i].millivolts;
		}
	}
	
	return maxVal;
} // end of getCellsMaxV -----------------------------------


/***********************************************************
	@brief
***********************************************************/
int CellComms::getCellsAveT(void)
{
	int aveVal			= 0;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		aveVal			+= _cellDataArray[i].temperature;
	}
	
	return (aveVal / cellsRead);
} // end of getCellsAveT -----------------------------------


/***********************************************************
	@brief
***********************************************************/
int CellComms::getCellsMinT(void)
{
	int minVal			= _cellDataArray[0].temperature;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (minVal > _cellDataArray[i].temperature)
		{
			minVal		= _cellDataArray[i].temperature;
		}
	}
	
	return minVal;
} // end of getCellsMinT -----------------------------------


/***********************************************************
	@brief
***********************************************************/
int CellComms::getCellsMaxT(void)
{
	int maxVal			= _cellDataArray[0].temperature;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (maxVal < _cellDataArray[i].temperature)
		{
			maxVal		= _cellDataArray[i].temperature;
		}
	}
	
	return maxVal;
} // end of getCellsMaxT -----------------------------------


/***********************************************************
	@brief	returns the number of Cells with balancing active.
***********************************************************/
int CellComms::getCellsBalancing(void)
{
	int count		= 0;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (0 != _cellDataArray[i].balancing)
		{
			++count;
		} // end of if this cell is balancing
	} // end of for each cell
	
	return count;
} // end of getCellsBalancing ------------------------------


/***********************************************************
	@brief	returns the number of Cells with over voltage flag set.
***********************************************************/
int CellComms::getCellsOverVolt(void)
{
	int count		= 0;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (0 != _cellDataArray[i].overVoltage)
		{
			++count;
		} // end of if this cell is overVoltage
	} // end of for each cell
	
	return count;
} // end of getCellsOverVolt ------------------------------


/***********************************************************
	@brief	returns the number of Cells with under voltage flag set.
***********************************************************/
int CellComms::getCellsUnderVolt(void)
{
	int count		= 0;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (0 != _cellDataArray[i].underVoltage)
		{
			++count;
		} // end of if this cell is underVoltage
	} // end of for each cell
	
	return count;
} // end of getCellsUnderVolt ------------------------------


/***********************************************************
	@brief	returns the number of Cells with over temperature flag set.
***********************************************************/
int CellComms::getCellsOverTemp(void)
{
	int count		= 0;
	
	if (cellsRead > NUM_CELLS) {
		cellsRead		= NUM_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (0 != _cellDataArray[i].overTemperature)
		{
			++count;
		} // end of if this cell is overTemp
	} // end of for each cell
	
	return count;
} // end of getCellsOverTemp -------------------------------


// Private Functions ---------------------------------------


/***********************************************************
 * Encodes with low-density parity-check codes
 * The first byte (header) is not encoded, thereafter the 8-bits are encoded
 * into 12-bits. The bytes are sent in the order;
 *   B1   B2   LDPC   B3   B4   LDPC   ...
 * Hence every 3rd-byte has the error correction bits for the previous 2-bytes.
 * The interleaving is done over blocks of 6-bytes.
 *
 * Returns the length of the encoded message
 **********************************************************/
byte		CellComms::fecEncode(byte p_msg[], byte u8_msg_len)
{
	unsigned int	u16a_code_word[LDPC_BUF_SIZE];
	byte			u8_i, u8_encode, u8_g_matrix, u8_j, u8_g, u8_h;
#ifdef INTERLACING
	byte			u8_k;
#endif
	byte			u8a_tmp[LDPC_BUF_SIZE];

	// ensure that packet size is modulus 6
	u8_i							= 3 - ((u8_msg_len + 3) & 0x03);
	while (u8_i) {
		// pad-out with 0xFF
		p_msg[u8_msg_len]			= 0xFF;
		u8_msg_len++;
		u8_i--;
	} // end of while padding bytes needed.

	for (u8_i = 0; u8_i < u8_msg_len; u8_i++) {
		// initialise code-word with orginial byte
		u16a_code_word[u8_i]		= p_msg[u8_i];

		// encode using the generator matrix
		for (u8_g_matrix = 0; u8_g_matrix < 4; u8_g_matrix++) {
			// bit-mask byte with generator matrix
			u8_encode				= u8a_generator_matrix[u8_g_matrix] & p_msg[u8_i];

			// calculate odd-parity
			u8_encode	^= u8_encode >> 8;
			u8_encode	^= u8_encode >> 4;
			u8_encode	^= u8_encode >> 2;
			u8_encode	^= u8_encode >> 1;

			// modulus 2
			u8_encode				&= 0x01;

			// calculate code-word
			u16a_code_word[u8_i]	+= u8_encode << (u8_g_matrix + 8);
		}
	} // end of for each byte in message

	// increase message length for parity bits
	u8_msg_len						= (3 * u8_msg_len) >> 1;

	u8_encode						= 0;
	// new message with added ldpc
	for (u8_i = 0; u8_i < u8_msg_len; u8_i += 3) {		// create information bit array

		// convert code-words into data-bytes
		u8a_tmp[u8_i + 0]			= u16a_code_word[u8_encode];
		u8a_tmp[u8_i + 1]			= u16a_code_word[u8_encode + 1];
		u8a_tmp[u8_i + 2]			= (u16a_code_word[u8_encode] >> 8) + ((u16a_code_word[u8_encode + 1] & 0x0F00) >> 4);
		u8_encode					+= 2;
	} // end of for each 3byte block in message

	// interleaving bits across the message in block sizes of six to provide burst-noise immunity, step across the 8-bits in each byte
	for (u8_i = 0; u8_i < u8_msg_len; u8_i+= 6) {
		// reset indexes
		u8_g						= 0;
		u8_h						= 0;

		for (u8_j = 0; u8_j < 6; u8_j++) {
#ifdef INTERLACING
			// reset bytes
			p_msg[u8_i + u8_j]		= 0;

			for (u8_k = 0; u8_k < 8; u8_k++) {
				// interleave bits
				p_msg[u8_i + u8_j]	+= ( (u8a_tmp[u8_i + u8_g] >> u8_h) & 0x01 ) << u8_k;

				// increment indexex
				if (++u8_g > 5) {
					u8_g			= 0;
					u8_h++;
				}
			} // end of for each bit in a byte
#else
			p_msg[u8_i + u8_j]		= u8a_tmp[u8_i + u8_j];
#endif
		} // end of for each byte in a block
	} // end of for each block of 6 bytes

	// return length of encoded message
	return u8_msg_len;
} // end of fu8_fec_encode function ------------------------


/***********************************************************
    decodes a message using the fec
***********************************************************/
byte    CellComms::fecDecode(volatile byte en_msg[], byte de_msg[], byte u8_msg_len)
{
  static byte  u8a_decode[LDPC_BUF_SIZE];
  unsigned int    u16a_code_word[4];
  byte     u8_i, u8_j, u8_len;
#ifdef INTERLACING
	byte			u8_k, u8_g, u8_h;
#endif
  byte     u8a_tmp[LDPC_BUF_SIZE];

  // only decode in blocks of six
  u8_msg_len				= ((u8_msg_len / 6) * 6);
  u8_len    				= 0;

  if (u8_msg_len > 6) {
    u8_i  = u8_msg_len - 6;
  }
  else {
    u8_i  = 0;
  }

  // re-distribute the bits into their correct byte
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
      u8a_tmp[u8_i + u8_j]  = en_msg[u8_i + u8_j];
#endif
    } // end of for each byte in a block

    // code-words
    u16a_code_word[0] = u8a_tmp[u8_i + 0] + ( (u8a_tmp[u8_i + 2] & 0x0F) << 8);
    u16a_code_word[1] = u8a_tmp[u8_i + 1] + ( (u8a_tmp[u8_i + 2] & 0xF0) << 4);
    u16a_code_word[2] = u8a_tmp[u8_i + 3] + ( (u8a_tmp[u8_i + 5] & 0x0F) << 8);
    u16a_code_word[3] = u8a_tmp[u8_i + 4] + ( (u8a_tmp[u8_i + 5] & 0xF0) << 4);

    for (u8_j = 0; u8_j < 4; u8_j++) {
      // decode byte in packet
      fecByteDecode(&u16a_code_word[u8_j]);

      // reduce to 8-bit
      u16a_code_word[u8_j] &= 0xFF;
    } // end of for each block of 4 bytes

    // decode
    u8a_decode[u8_len + 0]  = u16a_code_word[0];
    u8a_decode[u8_len + 1]  = u16a_code_word[1];
    u8a_decode[u8_len + 2]  = u16a_code_word[2];
    u8a_decode[u8_len + 3]  = u16a_code_word[3];

    // since we've decoded another 4 bytes add 4 to the length.
    u8_len  += 4;
  } // end of for each block of 6 bytes

  // decoded message length
  u8_msg_len  = u8_len;

  for (u8_i = 0; u8_i < u8_msg_len; u8_i++) {
    de_msg[u8_i]    = u8a_decode[u8_i];
  }

  // return decoded message length
  return u8_msg_len;
} // end of fu8_fec_decode ---------------------------------


/***********************************************************
 * Decodes a 12-bits into a 8-bits using the fec
 **********************************************************/
void    CellComms::fecByteDecode(unsigned int *u16_code_word)
{
  byte     u8_p_matrix, u8_syndrome = 0;
  unsigned int    u16_decode;

  for (u8_p_matrix = 0; u8_p_matrix < 4; u8_p_matrix++) {
    // bit-mask byte with generator matrix
    u16_decode    = u16a_parity_check_matrix[u8_p_matrix] & (*u16_code_word);

    // calculate odd-parity
    u16_decode    ^= u16_decode >> 8;
    u16_decode    ^= u16_decode >> 4;
    u16_decode    ^= u16_decode >> 2;
    u16_decode    ^= u16_decode >> 1;

    // modulo 2
    u16_decode    &= 1;

    // sydrome
    u8_syndrome   += u16_decode << u8_p_matrix;
  }

  // error correction to the codeword, XOR the codeword with the coset leader
  (*u16_code_word)  ^= u16a_coset_leader_table[u8_syndrome];
} // end of fv_fec_byte_decode function --------------------

