/**********************************************************
  BatteryLookup.cpp - Library for reading the OSBMS Cell Monitors.
  Created by David Burkitt, March 8, 2017.
  Released into the public domain.
**********************************************************/

#include "Arduino.h"
#include "BatteryLookup.h"


/***********************************************************
	When the battery is not being used (e.g. overnight)
	the open circuit voltage can be measured.
	This open circuit voltage, when compensated for temperature,
	can be used to determine the State of Charge (SoC)
	and therefore improve the coulomb counting capacity estimate.
***********************************************************/

#define OCV_NUM_ENTRIES			(9)
#define MV_INDEX				(0)
#define SOC_INDEX				(1)


/***********************************************************
	A lookup table where the first column is the cell oc mv,
	and the second column is the SoC; 0 - 10,000
	These are not evenly spaced but located at inflexion points
	where the slope of the graph changes.
	This gives a better fit with fewer data points.
***********************************************************/
const uint16_t	u16a_lifepo4_soc[OCV_NUM_ENTRIES][2]	= {
#if (DATASET == LFP_A123)				// Based on data from A123
		//	mV,		SoC
		{	2900,	0	 	},
		{	3205,	1000 	},
		{	3268,	2500	},
		{	3300,	4000	},	// start of flat region 1
		{	3315,	6500	},	// end of flat region 1
		{	3340,	7500	},	// start of flat region 2
		{	3349,	8900	},	// end of flat region 2
		{	3390,	9800	},
		{	3400,	10000	}
#elif (DATASET == LFP_TI)			// based on some TI data but unsure of cell manufacturer
		//	mV,		SoC
		{	2650,	0	 	},
		{	3130,	600 	},
		{	3270,	2600	},
		{	3290,	4000	},	// start of flat region 1
		{	3300,	6000	},	// end of flat region 1
		{	3325,	7500	},	// start of flat region 2
		{	3345,	9400	},	// end of flat region 2
		{	3390,	9800	},
		{	3500,	10000	}
#elif (DATASET == LFP_SP)			// Based on Thundersky data which became SinoPoly
		//	mV,		SoC
		{	2800,	0	 	},
		{	3184,	1000 	},
		{	3235,	2500	},
		{	3271,	4000	},	// start of flat region 1
		{	3294,	6500	},	// end of flat region 1
		{	3307,	7500	},	// start of flat region 2
		{	3324,	8900	},	// end of flat region 2
		{	3385,	9800	},
		{	3400,	10000	}
#else
#error Dataset not defined
#endif
};


// Constructor
BatteryLookup::BatteryLookup(void)
{
} // end of constructor -----------------------------------


/***********************************************************
	@brief	returns the SoC value for the givien mV.
			Also populates the struct with upper and lower SoC
			to provide a 'margin-of-error' indication.
***********************************************************/
void BatteryLookup::getSoC(int millivolts, int* socArray)
{
	socArray[0]		= lookupOcvSoc(millivolts);
	socArray[1]		= lookupOcvSoc(millivolts + MV_MARGIN);
	socArray[2]		= lookupOcvSoc(millivolts - MV_MARGIN);
} // end of getSoC -----------------------------------------


// Private Functions ---------------------------------------


/***********************************************************
	returns SoC for the given cell mV.
***********************************************************/
int BatteryLookup::lookupOcvSoc(int cellMv)
{
	uint16_t		u16_lower_v, u16_upper_v;
	uint16_t		u16_lower_soc, u16_upper_soc;
	uint16_t		u16_temp_soc;
	uint8_t			u8_i;

	// establish the soc estimate
	if (cellMv <= u16a_lifepo4_soc[0][MV_INDEX])
	{
		u16_temp_soc				= u16a_lifepo4_soc[0][SOC_INDEX];
	}
	else if (cellMv >= u16a_lifepo4_soc[OCV_NUM_ENTRIES - 1][MV_INDEX])
	{
		u16_temp_soc				= SOC_100PCT;
	}
	else
	{
		u8_i						= 0;
		u16_temp_soc				= 0;
		u16_lower_v					= 0;
		u16_lower_soc				= 0;

		while ( (u8_i < OCV_NUM_ENTRIES)
				&& (u16_temp_soc == 0) )
		{
			if (cellMv > u16a_lifepo4_soc[u8_i][MV_INDEX])
			{
				u16_lower_v			= u16a_lifepo4_soc[u8_i][MV_INDEX];
				u16_lower_soc		= u16a_lifepo4_soc[u8_i][SOC_INDEX];
			}
			else if (cellMv < u16a_lifepo4_soc[u8_i][MV_INDEX])
			{
				u16_upper_v			= u16a_lifepo4_soc[u8_i][MV_INDEX];
				u16_upper_soc		= u16a_lifepo4_soc[u8_i][SOC_INDEX];

				// interpolate
				u16_temp_soc		= ((cellMv - u16_lower_v) * (u16_upper_soc - u16_lower_soc)) / (u16_upper_v - u16_lower_v);
				u16_temp_soc		+= u16_lower_soc;
			}
			else		// cell voltage must = table entry
			{
				u16_temp_soc		= u16a_lifepo4_soc[u8_i][SOC_INDEX];
			}

			++u8_i;
		} // end of for each entry
	} // end of else valid regions

	return u16_temp_soc;
} // end of lookupOcvSoc -----------------------------------
