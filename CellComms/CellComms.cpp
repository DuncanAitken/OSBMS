/**********************************************************
  CellComms.cpp - Library for reading the OSBMS Cell Monitors.
  Created by David Burkitt, March 8, 2017.
  Released into the public domain.
**********************************************************/

#include "Arduino.h"
#include "CellComms.h"


#define  LDPC_BUF_SIZE 		50


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
} _cellDataArray[MAX_CELLS];

byte cellsRead;
byte minVIndex;
byte maxVIndex;
byte minTIndex;
byte maxTIndex;
byte commsVersion;
byte numberOfCells;				// the actual number of cells in this battery
	

// Constructor
CellComms::CellComms(byte numCells, unsigned long baud, byte commVer)
{
	if (numCells > MAX_CELLS) {
		numCells						= MAX_CELLS;
	}
  Serial2.begin(baud);
  commsVersion							= commVer;
  numberOfCells							= numCells;
  
  for (int i = 0; i < MAX_CELLS; ++i)
  {
	_cellDataArray[i].millivolts		= 0;
	_cellDataArray[i].temperature		= 0;
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
	// mask off the top 4 bits.
	millivolts					&= 0x0FFF;
	
	// form into a 4byte buffer and encode
	_tx_packet[0]				= (millivolts >> 8);
	_tx_packet[0]				|= 0xE0;		// set the marker bits for a master message
	_tx_packet[1]				= (millivolts);
#if (CELL_COMMS_VER < 2)
	_tx_packet[2]				= (millivolts >> 8);
	_tx_packet[2]				|= 0xE0;		// set the marker bits for a master message
	_tx_packet[3]				= (millivolts);
	
	fecEncode(_tx_packet, 4);
	
  int bytesSent					= Serial2.write(_tx_packet, 6);
#else
	fecEncode(_tx_packet, 2);
	
  int bytesSent					= Serial2.write(_tx_packet, 3);
#endif
  
  cellsRead						= 0;

  Serial2.flush();
} // end of sendMillivolts ---------------------------------


/***********************************************************
	@brief	sends the initiator message requesting temperature.
***********************************************************/
void CellComms::sendTemperature(void)
{
	// form into a 2byte buffer and encode
	_tx_packet[0]				= 0xF0;			// set the marker bits for a master message
	_tx_packet[1]				= 0;
	
	fecEncode(_tx_packet, 2);
	
	int bytesSent					= Serial2.write(_tx_packet, 3);
	cellsRead						= 0;
	Serial2.flush();
} // end of sendTemperature --------------------------------


/***********************************************************
	readCells
	reads blocks of 6 bytes from serial and decodes them
	into cell data.
***********************************************************/
int CellComms::readCells(void)
{
#if (CELL_COMMS_VER > 1)
  const int expected_bytes    					= 3;
#else
  const int expected_bytes    					= 6;
#endif
  int bytes_available         					= Serial2.available();
  int cellNum                 					= 0;
//  int tempMillivolts;
  struct _cellData _cellDataBuffer[MAX_CELLS];
  static unsigned char lastOvertemp[MAX_CELLS];
  static unsigned char lastOvervolt[MAX_CELLS];
  static unsigned char lastUndervolt[MAX_CELLS];
  
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
    
	if ((decoded[0] & 0x70) == 0) {
		// Interpret decoded values.
		// Bytes 0 and 1 make up the voltage.
		// Bytes 2 and 3 make up the temperature.
//		tempMillivolts								= ((decoded[0] & 0x1F) << 8) + decoded[1];
//		unsigned int temperature					= ((decoded[2] & 0x0F) << 8) + decoded[3];
		  _cellDataBuffer[cellNum].millivolts		= ((decoded[0] & 0x1F) << 8) + decoded[1];
		  _cellDataBuffer[cellNum].balancing		= ((decoded[0] & 0x80) >> 7);
#if (CELL_COMMS_VER < 2)
		  _cellDataBuffer[cellNum].temperature		= ((decoded[2] & 0x0F) << 8) + decoded[3];
		  _cellDataBuffer[cellNum].overTemperature	= ((decoded[2] & 0x80) >> 7);
		  _cellDataBuffer[cellNum].overVoltage		= ((decoded[2] & 0x40) >> 6);
		  _cellDataBuffer[cellNum].underVoltage		= ((decoded[2] & 0x20) >> 5);
#endif
		// } // end of if mV makes sense
	} // end of if valid cell data
#if (CELL_COMMS_VER > 1)
	if ((decoded[0] & 0x70) == 1) {
		  _cellDataBuffer[cellNum].temperature		= ((decoded[0] & 0x0F) << 8) + decoded[1];
		  _cellDataBuffer[cellNum].overTemperature	= ((decoded[0] & 0x80) >> 7);
		  _cellDataBuffer[cellNum].overVoltage		= ((decoded[0] & 0x40) >> 6);
		  _cellDataBuffer[cellNum].underVoltage		= ((decoded[0] & 0x20) >> 5);
	}
#endif

//	if ((decoded[2] & 0xE0) == 0xE0) {
	if ((decoded[0] & 0xE0) == 0xE0) {
		// reset the cell number as this was the seeding message
		cellNum										= 0;
		cellsRead									= 0;
	}
	else {
		// don't allow cellNum to exceed array size.
		if (cellNum < (MAX_CELLS)) {
			++cellNum;
		}
		++cellsRead;
	}
  } // end of while packet available
  
  // copy temp buffer starting at (numberOfCells - cellNum)
  if (cellNum <= numberOfCells) {
	for (int i = 0; i < cellNum; ++i) {
		// validate the data before we copy it to the array.
		if ( (_cellDataBuffer[i].millivolts > 1900)						// The regulator colapses at 2v
			  && (_cellDataBuffer[i].millivolts < 5100)		    		// the adc will max out at 5v
			  && (_cellDataBuffer[i].temperature < 1100) ) {		    // the adc will max out at 105c
			_cellDataArray[i + (numberOfCells - cellNum)].millivolts		= _cellDataBuffer[i].millivolts;
			_cellDataArray[i + (numberOfCells - cellNum)].temperature		= _cellDataBuffer[i].temperature;
			_cellDataArray[i + (numberOfCells - cellNum)].balancing			= _cellDataBuffer[i].balancing;
			unsigned char temp											= _cellDataBuffer[i].overTemperature;
			if (lastOvertemp[i + (numberOfCells - cellNum)] == temp) {
				_cellDataArray[i + (numberOfCells - cellNum)].overTemperature	= temp;
			}
			lastOvertemp[i + (numberOfCells - cellNum)]						= temp;
			temp														= _cellDataBuffer[i].overVoltage;
			if (lastOvervolt[i + (numberOfCells - cellNum)] == temp) {
				_cellDataArray[i + (numberOfCells - cellNum)].overVoltage	= temp;
			}
			lastOvervolt[i + (numberOfCells - cellNum)]						= temp;
			temp														= _cellDataBuffer[i].underVoltage;
			if (lastUndervolt[i + (numberOfCells - cellNum)] == temp) {
				_cellDataArray[i + (numberOfCells - cellNum)].underVoltage	= temp;
			}
			lastUndervolt[i + (numberOfCells - cellNum)]					= temp;
		} // end of if valid data.
	} // end of for each cell read
  } // end of if cellNum valid
  
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
	@brief	returns the millivolts of the cell.
	@param	cell: the cell number 1 - numberOfCells.
***********************************************************/
int CellComms::getCellV(uint8_t cell)
{
	int retVal			= 0;
	
	if ( (cell != 0) && (cell <= numberOfCells) ) {
		retVal			= _cellDataArray[cell - 1].millivolts;
	}
	
	return retVal;
} // end of getCellV ---------------------------------------


/***********************************************************
	@brief	returns the temperature of the cell.
	@param	cell: the cell number 1 - numberOfCells.
***********************************************************/
int CellComms::getCellT(uint8_t cell)
{
	int retVal			= 0;
	
	if ( (cell != 0) && (cell <= numberOfCells) ) {
		retVal			= _cellDataArray[cell - 1].temperature;
	}
	
	return retVal;
} // end of getCellT ---------------------------------------


/***********************************************************
	@brief	returns the balancing load of the cell.
	@param	cell: the cell number 1 - numberOfCells.
***********************************************************/
int CellComms::getCellLoad(uint8_t cell)
{
	int retVal			= 0;
	
	if ( (cell != 0) && (cell <= numberOfCells) ) {
		retVal			= _cellDataArray[cell - 1].balancing;
	}
	
	return retVal;
} // end of getCellLoad ------------------------------------


/***********************************************************
	@brief	returns the average millivolts of all cells.
***********************************************************/
int CellComms::getCellsAveV(void)
{
	long aveVal			= 0;
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		aveVal			+= _cellDataArray[i].millivolts;
	}
	
	return (int) (aveVal / cellsRead);
} // end of getCellsAveV -----------------------------------


/***********************************************************
	@brief	returns the millivolts of the lowest cell.
***********************************************************/
int CellComms::getCellsMinV(void)
{
	int minVal			= _cellDataArray[0].millivolts;
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (minVal > _cellDataArray[i].millivolts)
		{
			minVal		= _cellDataArray[i].millivolts;
			minVIndex	= i;
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
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (maxVal < _cellDataArray[i].millivolts)
		{
			maxVal		= _cellDataArray[i].millivolts;
			maxVIndex	= i;
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
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
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
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (minVal > _cellDataArray[i].temperature)
		{
			minVal		= _cellDataArray[i].temperature;
			minTIndex	= i;
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
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
	}
	
	for (int i = 0; i < cellsRead; ++i)
	{
		if (maxVal < _cellDataArray[i].temperature)
		{
			maxVal		= _cellDataArray[i].temperature;
			maxTIndex	= i;
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
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
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
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
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
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
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
	
	if (cellsRead > MAX_CELLS) {
		cellsRead		= MAX_CELLS;
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


/***********************************************************
	@brief	returns the index of the minimum Voltage Cell.
***********************************************************/
int CellComms::getMinVCell(void)
{
	return (minVIndex + 1);
} // end of getMinVCell ------------------------------------


/***********************************************************
	@brief	returns the index of the maximum Voltage Cell.
***********************************************************/
int CellComms::getMaxVCell(void)
{
	return (maxVIndex + 1);
} // end of getMaxVCell ------------------------------------


/***********************************************************
	@brief	returns the index of the minimum temperature Cell.
***********************************************************/
int CellComms::getMinTCell(void)
{
	return (minTIndex + 1);
} // end of getMinTCell ------------------------------------


/***********************************************************
	@brief	returns the index of the maximum temperature Cell.
***********************************************************/
int CellComms::getMaxTCell(void)
{
	return (maxTIndex + 1);
} // end of getMaxTCell ------------------------------------


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

