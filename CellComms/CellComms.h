/***********************************************************
  CellComms.h - Library for reading the OSBMS Cell Monitors.
  Created by David Burkitt, March 8, 2017.
  Released into the public domain.
  
  The DARC interface board fits onto an Arduino Mega2560
  and uses Serial2 to communicate with the Cell Monitors.
  
  NOTE: You will need to edit
  C:\Users\<username>\AppData\Local\Arduino15\packages\arduino\hardware\avr\<version>\cores\arduino\HardwareSerial.h
  if you need to read more than 9 cells.
  
  User needs to select the appropriate CELL_BAUD value to match
  their Cell Monitor boards and set NUM_CELLS to match the
  number of cell monitors in their battery stack.
  
  sendMillivolts transmits the initiator packet
  readCells should be called after sendMillivolts allowing time
	for the message to propogate through the battery.
***********************************************************/


#ifndef CellComms_h
#define CellComms_h


#include "Arduino.h"


#define MAX_CELLS			84
// interlacing is not possible with comms version 2 as it requires 6 byte data blocks.
//#define INTERLACING		1		// comment this to disable bitwise interlacing


class CellComms
{
  public:
	CellComms(byte numCells, unsigned long baud, byte commVer);
	void sendMillivolts(int millivolts);
	void sendTemperature(void);
	int readCells(void);
	int getCellV(uint8_t cell);
	int getCellT(uint8_t cell);
	int getCellLoad(uint8_t cell);
	int getCellsAveV(void);
	int getCellsMinV(void);
	int getCellsMaxV(void);
	int getCellsAveT(void);
	int getCellsMinT(void);
	int getCellsMaxT(void);
	int getCellsBalancing(void);
	int getCellsOverVolt(void);
	int getCellsUnderVolt(void);
	int getCellsOverTemp(void);
	int getMinVCell(void);
	int getMaxVCell(void);
	int getMinTCell(void);
	int getMaxTCell(void);
  private:
	byte		fecEncode(byte p_msg[], byte u8_msg_len);
	byte 		fecDecode(volatile byte en_msg[], byte de_msg[], byte u8_msg_len);
	void 		fecByteDecode(unsigned int *u16_code_word);
	
	byte 		_tx_packet[];
}; // ------------------------------------------------------


#endif