/*****************************************************
 * Test setup for the CellComms library.
 * 
 * The CellComms library needs two calls to be made periodically
 * 1. A call to sendMillivolts every sample period (5s or so)
 * 2. A call to readCells shortly afterwards.
*****************************************************/


// Options
#define debug             (1)       // enable this to have debug messages sent on serial0
#define LCD_DISPLAY       (1)       // 0 = No LCD display, 1 = LCD display fitted
#define DETAILED_INFO     (0)       // 0 = easy to read display, 1 = lots of info on screen
#define READ_CURRENT      (1)       // enable this to sample a current sensor on an ADC pin. The pin in set in the assignments section
#define ACTIVE_BALANCING  (1)       // enable this to balance the cells all the time, otherwise top balancing only.
#define SOC_ESTIMATOR     (1)       // 1 = estimate State-of-Charge, 0 = disable.
#define SHOW_BARGRAPH     (1)       // shows a bargraph of the SoC on the bottom row of the display.


/******************************************************
        LCD Display
  1) On the MEGA I2C is not pins A4 and A5, it's the two pins closest to the USB jack - check out the diagram here:
https://arduino-info.wikispaces.com/file/view/Mega2560_R3_Label-small-v2%20%282%29.png/471429496/Mega2560_R3_Label-small-v2%20%282%29.png
  2) Check the address of the screen using the very useful code here will tell you:
http://henrysbench.capnfatz.com/henrys-bench/arduino-projects-tips-and-more/arduino-quick-tip-find-your-i2c-address/
  3) NB you must install the correct library into the arduino/libraries folder on your computer. AND you MUST remove all other LCD libraries from the libraries folder, or it will not work !!!
    I used version 1.3.4 of:
https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/
******************************************************/


#include <CellComms.h>
#if (0 != LCD_DISPLAY)
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#endif
#if (0 != SOC_ESTIMATOR)
#include <BatteryLookup.h>
#endif  // SOC_ESTIMATOR



// Settings
#define BATT_MAH          (35000)   // 35Ah or 35,000mAh. yours may be larger!
#define DEBUG_BAUD        (9600)    // baudrate for debug comms to the console
#define CELL_LOOP_MS      (9000)    // interval between cell data reads
#define CELL_READ_MS      (600)     // delay after send for the data to shift in
#define AMPS_READ_LOOP_MS (100)     // interval in ms between reading the current sensor.
#define SOC_UPDATE_MS     (1000)    // interval in ms between updating the State-of-Charge (SoC).
#define SCREEN_REFRESH_MS (1000)    // how often the display is updated
#define AMPS_OFFSET       (512)     // ~1/2 of the max ADC count as the current sensor is biased so it can read charge and dis-charge currents.
#define AMPS_SCALAR       (97)      // scalar to convert the ADC count into the actual milliamps.
#if (0 != SOC_ESTIMATOR)
#define SOC_MAX           (SOC_100PCT)   // 100.00, 100% with 2 d.p.
#endif  // SOC_ESTIMATOR

// Pin Assignments
#define ALARM_LED_PIN     (4)
#define FAULT_LED_PIN     (5)
#define RELAY_PIN         (6)
#define HEATER_PIN        (7)
#define HEART_LED_PIN     (13)
#define AMPS_AN_PIN       (0)       // the ADC pin connected to the current sensor

#if (0 != LCD_DISPLAY)
#define I2C_ADDR          0x3F      // Define I2C Address where the PCF8574A is
// These pin assignments are internal to the I2C interface NOT pins on the MEGA.
#define Rs_pin            0
#define Rw_pin            1
#define En_pin            2
#define BACKLIGHT_PIN     3
#define D4_pin            4
#define D5_pin            5
#define D6_pin            6
#define D7_pin            7
#endif // LCD_DISPLAY


unsigned int cellMeanMillivolt  = 3600;   // initiate with a high mean value so the loads don't turn on.
unsigned int sentVoltage        = 3600;
unsigned int lowestVoltage;
unsigned int beepTime;
unsigned int beepGap;
unsigned long currentMillis;
unsigned long sendMillis;
unsigned long readMillis;
unsigned long readAmpsMillis;
unsigned long updateSoCMillis;
unsigned long ScreenRefreshMillis;
byte beeps;                               // the number of times to beep the buzzer
#if (0 != READ_CURRENT)
long milliAmps;
long aveMilliAmps;
long milliAmpHours;
#endif  // READ_CURRENT
#if (0 != SOC_ESTIMATOR)
unsigned int SoC;                         // State of Charge
int socArray[3];
#endif  // (0 != SOC_ESTIMATOR)

CellComms cells;                          // constructor for the CellComms library
BatteryLookup socs;

#if (0 != LCD_DISPLAY)
LiquidCrystal_I2C       lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);
#endif  // LCD_DISPLAY

void setup() {
  Serial.begin(DEBUG_BAUD);

#ifdef debug
  Serial.println("Starting...");
#endif
  
  pinMode(HEART_LED_PIN, OUTPUT);       // Enable Heart LED
  pinMode(FAULT_LED_PIN, OUTPUT);       // Enable Fault LED
  pinMode(ALARM_LED_PIN, OUTPUT);       // Enable Alarm LED
  pinMode(RELAY_PIN, OUTPUT);           // Enable Relay
  pinMode(HEATER_PIN, OUTPUT);          // Enable Heater

  // set serial 1 for the json streaming
  Serial1.begin(115200);

#if (0 != LCD_DISPLAY)
  // NEW LCD code using I2C
  lcd.begin (20, 4, LCD_5x8DOTS);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE); // init the backlight
#endif  // LCD_DISPLAY
  
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(HEART_LED_PIN, HIGH);
  digitalWrite(FAULT_LED_PIN, HIGH);
  delay(200); 
  digitalWrite(ALARM_LED_PIN, HIGH);
  delay(20); 
  digitalWrite(FAULT_LED_PIN, LOW);
  digitalWrite(ALARM_LED_PIN, LOW);
  
#if (0 != LCD_DISPLAY)
  // Print start-up message to the LCD.
  lcd.setBacklight(HIGH);     // Backlight on
  lcd.clear(); // clear display, set cursor position to zero

  lcd.print("   Eco-Ants OSBMS   ");   // NB must be 20 characters or it garbles
  lcd.setCursor(0, 1);
  lcd.print("     BMS master     ");   // NB must be 20 characters or it garbles
  lcd.setCursor(0, 2);
  lcd.print("    21 Aug 2017     ");   // NB must be 20 characters or it garbles
  lcd.setCursor(0, 3);
  lcd.print("Ants, Duncan & Dave ");   // NB must be 20 characters or it garbles

  // delay so we can read initial banner
  delay(2000);
#endif // LCD_DISPLAY

  beeps               = 0;
  beepTime            = 0;
  beepGap             = 0;
  lowestVoltage       = 3600;
  SoC                 = (SOC_MAX / 2);  // 50%
#if (0 != READ_CURRENT)
  aveMilliAmps        = 0;
  milliAmpHours       = BATT_MAH;       // Initialise to a full Battery
#endif  // (0 != READ_CURRENT)
  currentMillis       = millis();
  sendMillis          = (currentMillis + 100);
  readAmpsMillis      = (currentMillis + 50);
  updateSoCMillis     = (currentMillis + 1025);
//  ScreenRefreshMillis = (currentMillis + 1033);
}

void loop() {
  unsigned long lastMillis      = 0;
  
  // this is essentially just a scheduler to call functions at different intervals.
  
  currentMillis       = millis();

  if (currentMillis >= sendMillis) {
    startCellComms(CELL_READ_MS);
  } // end of sendMillis
  
  if (currentMillis >= readMillis) {
    readCellComms();
  } // end of readMillis
  
#if (0 != READ_CURRENT)
  if (currentMillis >= readAmpsMillis) {
    readAmps();

    readAmpsMillis        += AMPS_READ_LOOP_MS;
  } // end of readMillis
#endif  // READ_CURRENT

  if (currentMillis >= updateSoCMillis) {
    updateSoC();

    updateSoCMillis       += SOC_UPDATE_MS;
  }

  if (currentMillis != lastMillis) {
    beeper();
  }

  lastMillis          = currentMillis;
} // end of loop -------------------------------------


/******************************************
  initiate cell comms by sending the mean cell voltage.
  interval between sends can be varied.
******************************************/
void startCellComms(uint16_t interval) {
    digitalWrite(HEART_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(FAULT_LED_PIN, LOW);

    // use a very long time constant for falling voltage so high current draw doesn't cause balancing.
    if (sentVoltage > cellMeanMillivolt) {
      --sentVoltage;
    }
    else if (sentVoltage < cellMeanMillivolt) {
      sentVoltage       = (sentVoltage + cellMeanMillivolt) / 2;
    }
    // apply limits to the target value we tell the cells
    if (sentVoltage < 3000) {
      sentVoltage       = 3000;
    }
    else if (sentVoltage > 3600) {
      sentVoltage       = 3600;
    }

    // write 6 bytes to the BMS to initiate data transfer.
#if (0 != ACTIVE_BALANCING)
    cells.sendMillivolts(sentVoltage);
#else
    cells.sendMillivolts(0);
#endif

    // wait for the cell data to arrive
    readMillis        = (sendMillis + interval);
    sendMillis        += CELL_LOOP_MS;    // TODO: make the interval a function of the battery current
 } // end of startCellComms --------------------------


/*****************************************************
 * read the cell data once it has arrived.
 * 
 * if all cells read then do stuff with the data.
******************************************************/
 void readCellComms(void) {
  int cellsRead         = cells.readCells();
  
#ifdef debug
  Serial.print("\rRead ");
  Serial.println(cellsRead);
#endif
    
//  if (cellsRead == NUM_CELLS) {
  if (cellsRead > 0) {
    cellMeanMillivolt     = cells.getCellsAveV();
    int cellMinMillivolt  = cells.getCellsMinV();
    int cellMaxMillivolt  = cells.getCellsMaxV();
    int cellMinTemp       = cells.getCellsMinT();
    int cellMaxTemp       = cells.getCellsMaxT();
    int cellAveTemp       = cells.getCellsAveT();
    int cellsUndervolt    = cells.getCellsUnderVolt();
    int cellsOvervolt     = cells.getCellsOverVolt();
    int cellsOvertemp     = cells.getCellsOverTemp();
    int cellsBalancing    = cells.getCellsBalancing();
    int cellMinVnum       = cells.getMinVCell();
    int cellMaxVnum       = cells.getMaxVCell();
    int cellMinTnum       = cells.getMinTCell();
    int cellMaxTnum       = cells.getMaxTCell();
  
#ifdef debug
    Serial.print("mean ");
    Serial.print(cellMeanMillivolt);
    Serial.print(", min ");
    Serial.print(cellMinMillivolt);
    Serial.print(", max ");
    Serial.print(cellMaxMillivolt);
    Serial.print(", delta ");
    Serial.println(cellMaxMillivolt - cellMinMillivolt);
    
    Serial.print("UV ");
    Serial.print(cellsUndervolt);
    Serial.print(", OV ");
    Serial.print(cellsOvervolt);
    Serial.print(", HT ");
    Serial.print(cellsOvertemp);
    Serial.print(", Load ");
    Serial.println(cellsBalancing);
    
    Serial.print(SoC / 100);
    Serial.print(".");
    Serial.print(SoC % 100);
    Serial.println("%");
#endif

    // Check conditions to turn the charger ON
    // if any cells are under-voltage then turn the charger on
    if (cellsUndervolt > 0) {
      digitalWrite(RELAY_PIN, HIGH);   // turn charger on

      beeps               = 4;
    }
    // enable charger if SoC < 80% ?
    if (cellMaxMillivolt < 3500)
    {
      digitalWrite(RELAY_PIN, HIGH);
    }
    // TODO: may have a manual CHARGE button as well.


    // Check conditions to turn charger OFF
    // if all the cells are balancing then turn the charger off
    if (cellsBalancing == NUM_CELLS) {
      digitalWrite(RELAY_PIN, LOW);   // turn charger off
    }
    // if any cells are overvoltage then turn the charger off
    if (cellsOvervolt > 0) {
      digitalWrite(RELAY_PIN, LOW);   // turn charger off

      beeps               = 2;
    }
    // if any cells are over-temperature then turn the charger off
    if (cellsOvertemp > 0) {
      digitalWrite(RELAY_PIN, LOW);   // turn charger off

      beeps               = 3;
    }

    // Heater control
    if (cellAveTemp < 150) {
      digitalWrite(HEATER_PIN, HIGH);
    }
    else {
      digitalWrite(HEATER_PIN, LOW);
    }

    if (lowestVoltage < cellMinMillivolt) {
      lowestVoltage       = cellMinMillivolt;
    }


    // Prepare a JSON payload string
    String payload = "{\"battery\": {\r";   // 14 chars
    payload += "\t\"num cells\":";          // 13 chars - 27 total
    payload += NUM_CELLS;                   // 2 chars - 29 total
    payload += ",\r";                       // 2 char - 31 total
    payload += "\t\"voltage\":";            // 11 chars - 42 total
    payload += (cellMeanMillivolt * NUM_CELLS); 
    payload += ",\r";                       // 2 chars
    payload += "\t\"current\":";            // 11 chars
    payload += aveMilliAmps;
    payload += ",\r";                       // 2 chars
    payload += "\t\"temperature\":";        // 15 chars
    payload += cellAveTemp; 
    payload += ",\r";                       // 2 chars
    payload += "\t\"SoC\":";                // 7 chars
    payload += SoC;
    // TODO:
    payload += "\r}}";                      // 3 chars

    // Send payload
    char attributes[200];
    payload.toCharArray( attributes, 200 );
//    client.publish( "v1/devices/me/telemetry", attributes );
    Serial1.println( attributes );

  
  } // end of all cells read successfully
  else {
    digitalWrite(FAULT_LED_PIN, HIGH);

    beeps               = 1;
  } // not all cells read

#if (0 != LCD_DISPLAY)
  showLCD(cellsRead);
#endif // LCD_DISPLAY

  readMillis            += CELL_LOOP_MS;  // move the trigger value on so it doesn't run 100 times

  // backstop in case send is in the past
  if (sendMillis < millis()) {
    sendMillis          = (readMillis - CELL_READ_MS);
  }
  
  digitalWrite(HEART_LED_PIN, LOW);   // turn the LED off to show we're done
 } // end of readCellComms ---------------------------
 

/*********************************************
  manages the beeping
  called every millisecond
*********************************************/
void    beeper(void) {
  // loop to beep x times
  if (beeps > 0) {
    if ( (beepTime == 0) && (beepGap == 0) ) {
      digitalWrite(ALARM_LED_PIN, HIGH);
      beepTime        = 500;
    }
    else if (beepTime > 0) {
      --beepTime;
      // check if we're at the end of the beep
      if (beepTime == 0) {
        digitalWrite(ALARM_LED_PIN, LOW);
        --beeps;
        // are there more beeps to do?
        if (beeps > 0) {
          beepGap     = 15000;
        } // end of if more beeps to be done
      } // end of end of beep
    } // end of else if beeping
    else if (beepGap > 0) {
      --beepGap;
    }
  } // end of if beeps to do
} // end of beeper ----------------------------------


#if (0 != READ_CURRENT)
/*****************************************************
 * reads the current sensor
 ****************************************************/
 void   readAmps(void)
 {
  // sample the adc channel connected to the current sensor
  int val               = analogRead(AMPS_AN_PIN);
  
  // scale the adc reading into milliamps
  milliAmps             = ( (long) val - AMPS_OFFSET) * AMPS_SCALAR;

  // update a filtered average current as well
  aveMilliAmps          = ((aveMilliAmps * 9) + milliAmps) / 10;
 } // end of readAmps --------------------------------
#endif  // READ_CURRENT


#if (0 != SOC_ESTIMATOR)
 /****************************************************
  * updates the State of Charge estimate
  * called every second.
  ***************************************************/
  void  updateSoC(void)
  {
    static unsigned int relaxedTime    = 0;
    static long tmpMAs    = 0;
    long        SoCMinimum;
    
#if (0 != READ_CURRENT)
    // if ave current is above noise floor, 1% of full scale
//    if ( (aveMilliAmps > 1000) || (aveMilliAmps < -1000) )
//    {
//      // count coulombs or rather totalise the mA seconds
//      if (aveMilliAmps > 0)
//      {
//        // assume 5% loss during charging
//        tmpMAs            += (aveMilliAmps - (aveMilliAmps / 20));
//      }
//      else
//      {
//        tmpMAs            += (aveMilliAmps);
//      }
//
//      // establish the threshold for a change in the SoC
//      SoCMinimum          = (milliAmpHours / SOC_MAX);
//
//      // if we have more than 0.01% SoC
//      while (tmpMAs > SoCMinimum)
//      {
//        tmpMAs            -= SoCMinimum;
//        if (SoC < SOC_MAX)
//        {
//          ++SoC;
//        } // end of if SoC < 100%
//      }
//      // if we have more than -0.01% SoC
//      while (tmpMAs < (0 - SoCMinimum))
//      {
//        tmpMAs            += SoCMinimum;
//        if (SoC > 0)
//        {
//          --SoC;
//        } // end of if SoC > 0
//      }
//
//      relaxedTime         = 0;
//    } // end of if enough current to use
//    else  if ( (aveMilliAmps < 100) && (aveMilliAmps > -100) )
    {
      ++relaxedTime;
      if (relaxedTime > (60 * 5))   // 5 minutes
      {
        // read Voc
        
        // TODO: need to add a compensation for temperature
  
        // TODO: need an external library to convert open-circuit voltage (compensated for current and temperature into SoC
        socs.getSoC(cellMeanMillivolt, socArray);

        if (SoC > socArray[1]) {
          SoC             = socArray[1];
        }
        else if (SoC < socArray[2]) {
          SoC             = socArray[2];
        }
        else if (SoC < socArray[0]) {
          if (SoC < SOC_MAX) {
            ++SoC;
          }
        }
        else if (SoC > socArray[0]) {
          if (SoC > 0) {
            --SoC;
          }
        }

        relaxedTime       = 0;
      }
    } // end of else low current
#endif  // (0 != READ_CURRENT)
  } // end of updateSoC ------------------------------
#endif  // (0 != SOC_ESTIMATOR)


#if (0 != LCD_DISPLAY)
/*****************************************************
 * displays the cell data on 20x4 display.
******************************************************/
 void showLCD(int cellsRead) {
  if (cellsRead > 0) {
    cellMeanMillivolt     = cells.getCellsAveV();
    int cellMinMillivolt  = cells.getCellsMinV();
    int cellMaxMillivolt  = cells.getCellsMaxV();
    int cellMinTemp       = cells.getCellsMinT();
    int cellMaxTemp       = cells.getCellsMaxT();
    int cellAveTemp       = cells.getCellsAveT();
    int cellsUndervolt    = cells.getCellsUnderVolt();
    int cellsOvervolt     = cells.getCellsOverVolt();
    int cellsOvertemp     = cells.getCellsOverTemp();
    int cellsBalancing    = cells.getCellsBalancing();
    int cellMinVnum       = cells.getMinVCell();
    int cellMaxVnum       = cells.getMaxVCell();
    int cellMinTnum       = cells.getMinTCell();
    int cellMaxTnum       = cells.getMaxTCell();
    
    lcd.clear(); // clear display, set cursor position to 0,0

    // Line 1
    lcd.print(cellsRead);
#if (0 == DETAILED_INFO)
    lcd.print("/");
    lcd.print(NUM_CELLS);
    if (cellsUndervolt |= 0) {
      lcd.setCursor(13, 0);
      lcd.print(cellsUndervolt);
      lcd.print(" LOW");
    }
    else if (cellsOvertemp |= 0) {
      lcd.setCursor(13, 0);
      lcd.print(cellsOvertemp);
      lcd.print(" HOT");
    }
    else if (cellsOvervolt |= 0) {
      lcd.setCursor(13, 0);
      lcd.print(cellsOvervolt);
      lcd.print(" HIGH");
    }
    else if (cellsRead != NUM_CELLS) {
      lcd.setCursor(17, 0);
      lcd.print(":(");
    }
    else {
      lcd.setCursor(17, 0);
      lcd.print(":)");
    }
#else // DETAILED_INFO
    lcd.print(" read, u");
    lcd.print(cellsUndervolt);
    lcd.print(" o");
    lcd.print(cellsOvervolt);
    lcd.print(" t");
    lcd.print(cellsOvertemp);
#endif  // DETAILED_INFO
    
    // Line 2
    lcd.setCursor(0, 1);
#if (0 == DETAILED_INFO)
    lcd.print(cellMinMillivolt);
    lcd.print(" - ");
    lcd.print(cellMaxMillivolt);
    lcd.print("mV");
    lcd.setCursor(15, 1);
    lcd.print(cellAveTemp / 10);
    lcd.print(".");
    lcd.print(cellAveTemp % 10);
    lcd.print("c");
#else // DETAILED_INFO
    lcd.print("Max ");
    lcd.print(cellMaxMillivolt);
    lcd.print(":");
    if (cellMaxVnum < 10) {
      lcd.print(" ");
    }
    lcd.print(cellMaxVnum);
    lcd.print(", ");
    lcd.print(cellMaxTemp / 10);
    lcd.print(".");
    lcd.print(cellMaxTemp % 10);
    lcd.print(":");
    if (cellMaxTnum < 10) {
      lcd.print(" ");
    }
    lcd.print(cellMaxTnum);
#endif  // DETAILED_INFO
    
    // Line 3
    lcd.setCursor(0, 2);
#if (0 == DETAILED_INFO)
#if (0 != READ_CURRENT)
    lcd.print(aveMilliAmps / 1000);
    lcd.print(".");
    lcd.print((abs(aveMilliAmps % 1000) / 100));
    lcd.print("A  ");
#endif  // READ_CURRENT
    int pos = 16;
    if (SoC < SOC_MAX) {    // <100% 
      ++pos;
    }
    if (SoC < (SOC_MAX / 10)) {
      ++pos;
    }
    lcd.setCursor(pos, 2);
    lcd.print(SoC / 100);   // only show whole percentages
    lcd.print("%");
#else // DETAILED_INFO
    lcd.print("Min ");
    lcd.print(cellMinMillivolt);
    lcd.print(":");
    if (cellMinVnum < 10) {
      lcd.print(" ");
    }
    lcd.print(cellMinVnum);
    lcd.print(", ");
    lcd.print(cellMinTemp / 10);
    lcd.print(".");
    lcd.print(cellMinTemp % 10);
    lcd.print(":");
    if (cellMinTnum < 10) {
      lcd.print(" ");
    }
    lcd.print(cellMinTnum);
#endif  // DETAILED_INFO
    
    // Line 4
    lcd.setCursor(0, 3);
#if (0 != SHOW_BARGRAPH)
    int bar   = (SOC_MAX / 20);
    while (bar < SOC_MAX) {
      if (bar <= SoC) {
        lcd.print("B");
      }
      else {
        lcd.print(" ");
      }
      bar     += (SOC_MAX / 20);
    } // end of while bar < 100%
//    lcd.print(SoC / 100);
//    lcd.print(".");
//    lcd.print(SoC % 100);
#else // NO BARGRAPH
#if (0 == DETAILED_INFO)
    lcd.print("Chrgr ");
    if (digitalRead(RELAY_PIN) == 0) {
      lcd.print("Off");
    }
    else {
      lcd.print("ON");
    }
    lcd.setCursor(12, 3);
    lcd.print("Heat ");
    if (digitalRead(HEATER_PIN) == 0) {
      lcd.print("Off");
    }
    else {
      lcd.print("ON");
    }
#else // DETAILED_INFO
    if (cellsBalancing != 0) {
      lcd.print(cellsBalancing);
      lcd.print(" cells balancing");
    }
    else {
      lcd.print("Mean ");
      lcd.print(cellMeanMillivolt);
      lcd.print(", Delta ");
      lcd.print(cellMaxMillivolt - cellMinMillivolt);
    }
#endif  // DETAILED_INFO
#endif  // NO BARGRAPH
  } // end of all cells read successfully
  else {
    lcd.clear();
    lcd.print("My arms are flailing");
    lcd.setCursor(0, 1);
    lcd.print("  Error  !! ");
    lcd.setCursor(0, 2);
    lcd.print(cellsRead); lcd.print(" out of "); lcd.print(NUM_CELLS);
    lcd.setCursor(0, 3);
    lcd.print("Break before # "); lcd.print((NUM_CELLS - cellsRead));
  } // not all cells read
 } // end of showLCD -------------------------------
#endif // LCD_DISPLAY


/*****************************************************
 * Decodes a value, using fec to determine validity.
 * Returns value as a string repesentation of a float, or a string containing 'NULL' if invalid.
 * 
 * the 'balancing' parameter is an output.
 ****************************************************/
String  decodeVoltage(byte payload0, byte payload1, byte decoded0, byte decoded1) {

  float devisor = 1000.0;
  int precision = 3;
  int value =     ((decoded0 & 0x7f) << 8) | decoded1;//Mask off MSB
  float ret_val = 0.00f;
    
  if (payload0 != decoded0 || payload1 != decoded1) {
    return "NULL";
  } else {
    return String(( (float) value / devisor), precision);   
  }  
}


/*******************************************************
 * Decodes Temperature, using fec to determine validity.
 * Returns value as a string repesentation of a float, or a string containing 'NULL' if invalid.
 *******************************************************/
String  decodeTemperature(byte payload0, byte payload1, byte decoded0, byte decoded1) {

  float devisor = 10.0;
  int precision = 2;
  int value =     ((decoded0 & 0x3f) << 8) | decoded1;
  float ret_val = 0.00f;

  if (payload0 != decoded0 || payload1 != decoded1) {
    return "NULL";
  } else {
    return String(( (float) value / devisor),precision);   
  }
}


String int2String(int number)
{
  char buf[6];
  byte bufIndex   = 0;

  if (number > 10000)
  {
    buf[bufIndex] = (number / 10000) + '0';
    number        = (number % 10000);
    ++bufIndex;
  }
  if (number > 1000)
  {
    buf[bufIndex] = (number / 1000) + '0';
    number        = (number % 1000);
    ++bufIndex;
  }
  if (number > 100)
  {
    buf[bufIndex] = (number / 100) + '0';
    number        = (number % 100);
    ++bufIndex;
  }
  if (number > 10)
  {
    buf[bufIndex] = (number / 10) + '0';
    number        = (number % 10);
    ++bufIndex;
  }
  if (number > 0)
  {
    buf[bufIndex] = (number) + '0';
    ++bufIndex;
  }
  else
  {
    buf[bufIndex] = '0';
    ++bufIndex;
  }

  // append a string terminator
    buf[bufIndex] = 0;
  
  return String(buf);
}




