/***********************************************************
  BatteryLookup.h - Support Library for the OSBMS Cell Monitors.
  Created by David Burkitt, May 28, 2017.
  Released into the public domain.
  
  The DARC Cell Monitors mount on LiFePO4 cells.
  
  Calculates the State-of-Charge (SoC) of a cell from the
  open circuit voltage.
  Actually calculates 3 values,
	1 for the millivolts given and 
	2 for the upper and lower tolerances of the cell voltage reading.
	
	The upper and lower values give a margin of error,
	LiFePO4 cells exhibit a very flat voltage curve so the
	cell voltage measurement needs to be VERY accurate.
	The resistors should be at least 1% 50ppm.
	The voltage reference should be very accurate and temperature stable.
***********************************************************/

#ifndef BatteryLookup_h
#define BatteryLookup_h


#include "Arduino.h"


// Three voltage maps are available to choose from.
#define LFP_A123	(1)
#define LFP_TI		(2)
#define LFP_SP		(3)
// I have SinoPoly cells so this is the default
#define DATASET		(LFP_SP)
#define MV_MARGIN	(50)			// +- mv either side of reading for component tolerances. 50mv = 1%

#define SOC_100PCT				(10000)


class BatteryLookup
{
  public:
	BatteryLookup(void);
	void getSoC(int millivolts, int* socArray);
  private:
	int lookupOcvSoc(int cellMv);
}; // ------------------------------------------------------


#endif