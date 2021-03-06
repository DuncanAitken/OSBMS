/*****************************************************
 * Test setup for the CellComms library.
 * 
 * The CellComms library needs two calls to be made periodically
 * 1. A call to sendMillivolts every sample period (9s or so)
 * 2. A call to readCells shortly afterwards.
*****************************************************/


#include "mySettings.h"


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



#if (0 != SOC_ESTIMATOR)
#define SOC_MAX             (SOC_100PCT)   // 100.00, 100% with 2 d.p.
#endif  // SOC_ESTIMATOR


unsigned int cellMeanMillivolt  = 3600;
unsigned int sentVoltage        = 3600;   // initiate with a high mean value so the loads don't turn on.
unsigned int lowestVoltage;
unsigned int lowestCell;                  // cell number of the lowest ever cell
unsigned int hottestTemp;
unsigned int hottestCell;                 // cell number of the hottest ever cell
unsigned int beepTime;
unsigned int beepGap;
unsigned long currentMillis;
unsigned long sendMillis;
unsigned long readMillis;
unsigned long readAmpsMillis;
unsigned long updateSoCMillis;
byte beeps;                               // the number of times to beep the buzzer
byte message_cntr;                        // counter for voltage messages sent before temperature requested.
#if (0 != READ_CURRENT)
long milliAmps;                           // signed current reading in mA
long aveMilliAmps;
long milliAmpHours;
#endif  // READ_CURRENT
#if (0 != SOC_ESTIMATOR)
unsigned int SoC;                         // State of Charge
int socArray[3];
#endif  // (0 != SOC_ESTIMATOR)


// Constructors --------------------------
CellComms cells(NUM_CELLS, CELL_BAUD, CELL_COMMS_VER);    // constructor for the CellComms library.

#if (0 != SOC_ESTIMATOR)
BatteryLookup socs;
#endif  // (0 != SOC_ESTIMATOR)

#if (0 != LCD_DISPLAY)
LiquidCrystal_I2C       lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);
#endif  // LCD_DISPLAY


// Setup ---------------------------------
void setup() {
  
  pinMode(HEART_LED_PIN, OUTPUT);       // Enable Heart LED
  pinMode(FAULT_LED_PIN, OUTPUT);       // Enable Fault LED
  pinMode(ALARM_LED_PIN, OUTPUT);       // Enable Alarm LED
  pinMode(CHARGER_PIN, OUTPUT);         // Enable Charger
  pinMode(HEATER_PIN, OUTPUT);          // Enable Heater
  pinMode(CONNECT_PIN, OUTPUT);         // Enable connect
  
#ifdef debug
  Serial.begin(DEBUG_BAUD);
  Serial.println("Starting...");
#endif

  // set serial 1 for the json streaming
  Serial1.begin(115200);

#if (0 != LCD_DISPLAY)
  // NEW LCD code using I2C
  lcd.begin (20, 4, LCD_5x8DOTS);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE); // init the backlight
#endif  // LCD_DISPLAY
  
  digitalWrite(CHARGER_PIN, LOW);
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(CONNECT_PIN, LOW);
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
  lcd.print("    18 Apr 2018     ");   // NB must be 20 characters or it garbles
  lcd.setCursor(0, 3);
  lcd.print("Ants, Duncan & Dave ");   // NB must be 20 characters or it garbles

  // delay so we can read initial banner
  delay(2000);
#endif // LCD_DISPLAY

  beeps               = 0;
  beepTime            = 0;
  beepGap             = 0;
  lowestVoltage       = 3600;
  hottestTemp         = 1;              // 0.1c
  currentMillis       = millis();
  sendMillis          = (currentMillis + 100);
  readMillis          = (sendMillis + CELL_READ_MS);
#if (0 != READ_CURRENT)
  aveMilliAmps        = 0;
  milliAmpHours       = BATT_MAH;       // Initialise to a full Battery
  readAmpsMillis      = (currentMillis + 50);
#endif  // (0 != READ_CURRENT)
#if (0 != SOC_ESTIMATOR)
  SoC                 = (SOC_MAX / 2);  // 50%
  updateSoCMillis     = (currentMillis + 1025);
#endif  // (0 != SOC_ESTIMATOR)
} // end of setup ----------------------


// Loop --------------------------------
void loop() {
  unsigned long lastMillis      = 0;
  
  // this is essentially just a scheduler to call functions at different intervals.
  
  currentMillis       = millis();

  if (currentMillis >= sendMillis) {
    startCellComms(CELL_READ_MS);

    // wait for the cell data to arrive
    readMillis        = (sendMillis + CELL_READ_MS);
    sendMillis        += CELL_LOOP_MS;    // TODO: could make the interval a function of the battery current
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

#if (0 != SOC_ESTIMATOR)
  if (currentMillis >= updateSoCMillis) {
    updateSoC();

    updateSoCMillis       += SOC_UPDATE_MS;
  }
#endif  // (0 != SOC_ESTIMATOR)

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
    if (sentVoltage < SENT_MEAN_MIN) {
      sentVoltage       = SENT_MEAN_MIN;
    }
    else if (sentVoltage > SENT_MEAN_MAX) {
      sentVoltage       = SENT_MEAN_MAX;
    }

    if (message_cntr > TEMP_MSG_PERIOD) {
      message_cntr      = 0;
      cells.sendTemperature();
    }
    else {
      // write 6 bytes to the BMS to initiate data transfer.
#if (0 != ACTIVE_BALANCING)
      cells.sendMillivolts(sentVoltage);
#else
      cells.sendMillivolts(0);
#endif
    }

    ++message_cntr;
 } // end of startCellComms --------------------------


/*****************************************************
 * read the cell data once it has arrived.
 * 
 * if all cells read then do stuff with the data.
******************************************************/
 void readCellComms(void) {
  static uint8_t  u8_disconnect_cntr  = 0;
  int cellsRead         = cells.readCells();
  
#ifdef debug
  Serial.print("Read ");
  Serial.print(cellsRead);
  Serial.print("/");
  Serial.println(NUM_CELLS);
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

    // Check conditions to turn the charger ON
    // if any cells are under-voltage then turn the charger on
    if (cellsUndervolt > 0) {
      digitalWrite(CHARGER_PIN, HIGH);   // turn charger on

      beeps               = 4;
    }
    // enable charger if SoC < x% ?
    if (cellMaxMillivolt < CHRG_ENABLE_MV) {
      digitalWrite(CHARGER_PIN, HIGH);
    }
    
    // TODO: may have a manual CHARGE button as well.

    // Check conditions to turn charger OFF
    // if all the cells are balancing then turn the charger off
    if (cellsBalancing == NUM_CELLS) {
      digitalWrite(CHARGER_PIN, LOW);   // turn charger off
    }
    // if any cells are overvoltage then turn the charger off
    if (cellsOvervolt > 0) {
      digitalWrite(CHARGER_PIN, LOW);   // turn charger off

      beeps               = 2;
    }
    // if any cells are over-temperature then turn the charger off
    if (cellsOvertemp > 0) {
      digitalWrite(CHARGER_PIN, LOW);   // turn charger off

      beeps               = 3;
    }

    // Heater control ------------------
    if (cellAveTemp < HTR_ENABLE_TEMP) {
      digitalWrite(HEATER_PIN, HIGH);
    }
    else {
      digitalWrite(HEATER_PIN, LOW);
    }

    // Dis-connect control -----------------
    if ( (cellMaxTemp > DISCON_OVERTEMP)
        || (cellMaxMillivolt > DISCON_OVERVOLT)
        || (cellMinMillivolt < DISCON_UNDERVOLT) ) {
      if (u8_disconnect_cntr < 3) {
        ++u8_disconnect_cntr;
      }
      else {
        digitalWrite(CONNECT_PIN, LOW);
      }
      beeps               = 6;          // sound warnings before and during disconnect
    }
    else {
      if (u8_disconnect_cntr > 0) {
        --u8_disconnect_cntr;
      }
      else {
        digitalWrite(CONNECT_PIN, HIGH);
      }
    }

    // Metrics -----------------------------
    if (lowestVoltage > cellMinMillivolt) {
      lowestVoltage       = cellMinMillivolt;
      lowestCell          = cellMinVnum;
    }

    if (hottestTemp < cellMaxTemp) {
      hottestTemp         = cellMaxTemp;
      hottestCell         = cellMaxTnum;
    }
  
#ifdef debug
    Serial.print("mean ");
    Serial.print(cellMeanMillivolt);
    Serial.print(", min ");
    Serial.print(cellMinMillivolt);
    Serial.print(", max ");
    Serial.print(cellMaxMillivolt);
    Serial.print(", delta ");
    Serial.println(cellMaxMillivolt - cellMinMillivolt);
    
    Serial.print("sent mean ");
    Serial.print(sentVoltage);
    Serial.print(", lowest ");
    Serial.print(lowestCell);
    Serial.print(": ");
    Serial.print(lowestVoltage);
    Serial.print(", hottest ");
    Serial.print(hottestCell);
    Serial.print(": ");
    Serial.println(hottestTemp);
    
    Serial.print("UV ");
    Serial.print(cellsUndervolt);
    Serial.print(", OV ");
    Serial.print(cellsOvervolt);
    Serial.print(", HT ");
    Serial.print(cellsOvertemp);
    Serial.print(", Load ");
    Serial.print(cellsBalancing);
    Serial.print(", Charger ");
    Serial.print(digitalRead(CHARGER_PIN));
    Serial.print(", Heater ");
    Serial.print(digitalRead(HEATER_PIN));
    Serial.print(", Connect ");
    Serial.println(digitalRead(CONNECT_PIN));
    
#if (0 != SOC_ESTIMATOR)
    Serial.print(SoC / 100);
    Serial.print(".");
    if ((SoC % 100) < 10) {
      Serial.print("0");
    }
    Serial.print(SoC % 100);
    Serial.println("%");
#endif  // (0 != SOC_ESTIMATOR)
#endif
    


    // Prepare a JSON payload string
    String payload = "{\"battery\": {\r";     // 14 chars
    payload += "\t\"num cells\": \"";         // 15 chars - 29 total
    payload += NUM_CELLS;                     // 2 chars - 31 total
    payload += "\",\r";                       // 3 char - 34 total
    payload += "\t\"voltage\": \"";           // 13 chars - 47 total
    unsigned int totalV = 0;
    for (int i = 0; i < NUM_CELLS; ++i) {
      totalV += cells.getCellV(i + 1);
    }
    payload += (totalV / 1000);               // 2 chars - 49 total
    payload += ".";                           // 1 chars - 50 total
    if ((totalV % 1000) < 100) {
      payload += "0";     // insert leading zero
    }
    payload += ((totalV % 1000) / 10);        // 2 chars - 52 total
    payload += "\",\r";                       // 3 chars - 55 total
    payload += "\t\"current\": \"";           // 13 chars - 68 total
#if (0 != READ_CURRENT)
    payload += aveMilliAmps;                  // 4 chars - 72 total
#else // (0 != READ_CURRENT)
    payload += "0";
#endif  // (0 != READ_CURRENT)
    payload += "\",\r";                       // 3 chars - 75 total
    payload += "\t\"temperature\": \"";       // 17 chars - 92 total
    payload += (cellAveTemp / 10);            // 2 chars - 94 total
    payload += ".";                           // 1 chars - 95 total
    payload += (cellAveTemp % 10);            // 1 chars - 96 total
    payload += "\",\r";                       // 3 chars - 99 total
    payload += "\t\"SoC\": \"";               // 9 chars - 108 total
#if (0 != SOC_ESTIMATOR)
    payload += (SoC / 100);                   // 3 chars - 111 total
    payload += ".";                           // 1 chars - 112 total
    if ((SoC % 100) < 10) {
      payload += "0";     // add a leading zero
    }
    payload += (SoC % 100);                   // 2 chars - 114 total
#else
    payload += "0.00";
#endif  // (0 != SOC_ESTIMATOR)
    payload += "\",\r";                       // 3 chars - 117 total
    payload += "\t\"charger\": \"";           // 13 chars - 130 total
    payload += digitalRead(CHARGER_PIN);      // 1 chars - 131 total
    payload += "\",\r";                       // 3 chars - 134 total
    payload += "\t\"heater\": \"";            // 12 chars - 146 total
    payload += digitalRead(HEATER_PIN);       // 1 chars - 147 total
    payload += "\",\r";                       // 3 chars - 150 total
    payload += "\t\"disconnect\": \"";        // 16 chars - 166 total   TODO: need to rename 'connect'
    payload += digitalRead(CONNECT_PIN);      // 1 chars - 167 total
    payload += "\",\r";                       // 2 chars - 169 total
    payload += "\t\"sent mean\": \"";         // 16 chars - 185 total
    payload += sentVoltage;   // what we send as the mean to the cells
    payload += "\",\r";                       // 3 chars
    payload += "\t\"cells\": [\r";            // 12 chars
    for (int i = 0; i < NUM_CELLS; ++i) {
      payload += "\t\t{\"id\": \"";           // 10 chars
      payload += (i + 1);                     // 2 chars
      payload += "\", \"mv\": \"";            // 8 chars
      payload += cells.getCellV(i + 1);
      payload += "\", \"temp\": \"";          // 10 chars
      unsigned int cellT = cells.getCellT(i + 1);
      payload += (cellT / 10);
      payload += ".";
      payload += (cellT % 10);
      payload += "\", \"load\": \"";          // 10 chars
      payload += cells.getCellLoad(i + 1);
      payload += "\"}";                       // 1 chars
      if (i < NUM_CELLS - 1) {
        payload += ",\r";                     // 2 chars
      }
      else {
        payload += "\r\t]\r";                 // 4 chars
      }
    }
    payload += "}}";                          // 3 chars

    // Send payload
    char attributes[1150];                    // actually 1140 chars
    payload.toCharArray( attributes, 1150 );
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
//  milliAmps             += AMPS_TRIM;

  // TODO: may need to add temperature compensation

  // update a filtered average current as well
  aveMilliAmps          = ((aveMilliAmps * 9) + milliAmps) / 10;
 } // end of readAmps --------------------------------
#endif  // READ_CURRENT


#if (0 != SOC_ESTIMATOR)
 /****************************************************
  * updates the State of Charge estimate
  * +ve current is charging, -ve dis-charging.
  * called every second.
  ***************************************************/
  void  updateSoC(void)
  {
    static unsigned int relaxedTime   = 0;
    static long tmpMAs                = 0;
    static uint8_t  socInitialised    = 0;
    long        SoCMinimum;
    
#if (0 != READ_CURRENT)
    // if ave current is above noise floor, 1% of full scale
    if ( (aveMilliAmps > 1000) || (aveMilliAmps < -1000) )
    {
      // count coulombs or rather totalise the mA seconds
//      if (aveMilliAmps > 0)
//      {
//        // assume 5% loss during charging
//        tmpMAs            += (aveMilliAmps - (aveMilliAmps / 20));
//      }
//      else
//      {
//        tmpMAs            += (aveMilliAmps);
//      }

      // establish the threshold for a change in the SoC
//      SoCMinimum          = (milliAmpHours / SOC_MAX);

      // if we have more than 0.01% SoC
//      while (tmpMAs > SoCMinimum)
//      {
//        tmpMAs            -= SoCMinimum;
//        if (SoC < SOC_MAX)
//        {
//          ++SoC;
//        } // end of if SoC < 100%
//      }
      // if we have more than -0.01% SoC
//      while (tmpMAs < (0 - SoCMinimum))
//      {
//        tmpMAs            += SoCMinimum;
//        if (SoC > 0)
//        {
//          --SoC;
//        } // end of if SoC > 0
//      }

      relaxedTime         = 0;
    } // end of if enough current to use
//    else  if ( (aveMilliAmps < 100) && (aveMilliAmps > -100) )
#endif  // (0 != READ_CURRENT)
    {
      ++relaxedTime;
      if ( (relaxedTime > (60 * 10))   // 10 minutes
          || (0 == socInitialised) ) {
        // read Voc
        unsigned int Voc = cellMeanMillivolt;
//        unsigned int Voc = cells.getCellsMinV();
#if (READ_CURRENT != 0)
        Voc   -= (aveMilliAmps / 50);
#endif
        
        // TODO: need to add a compensation for temperature
  
        socs.getSoC(Voc, socArray);

        if (0 == socInitialised) {
          SoC             = socArray[0];
        }
        else {
          if (SoC > socArray[1]) {
            SoC           -= ((SoC - socArray[1]) / 2);
          }
          else if (SoC < socArray[2]) {
            SoC           += ((socArray[2] - SoC) / 2);
          }
          else if (SoC < socArray[0]) {
            if ((SoC + 10) < socArray[0]) {
              SoC         += ((socArray[0] - SoC) / 5);
            }
            else {
              ++SoC;
            }
          }
          else if (SoC > socArray[0]) {
            if (SoC > (socArray[0] + 10)) {
              SoC         -= ((SoC - socArray[0]) / 5);
            }
            else if (SoC > 0) {
              --SoC;
            }
          }
        } // end of else socInitialised

        relaxedTime       = 0;
        socInitialised    = 1;
      }
    } // end of else low current
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
    lcd.print("/");
    lcd.print(NUM_CELLS);
#if (0 == DETAILED_INFO)
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
//    lcd.print(" read, u");
//    lcd.print(cellsUndervolt);
//    lcd.print(" o");
//    lcd.print(cellsOvervolt);
//    lcd.print(" t");
//    lcd.print(cellsOvertemp);
  
    lcd.setCursor(7, 0);
    if (cellsRead != NUM_CELLS) {
      lcd.print(":(");
    }
    else {
      lcd.print(":)");
    }

    lcd.setCursor(14, 0);       // TODO: some refinement needed to right align and show - for < -1A
    lcd.print(aveMilliAmps / 1000);
    lcd.print(".");
    lcd.print((abs(aveMilliAmps % 1000) / 100));
    lcd.print("A");
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
#if (0 != SOC_ESTIMATOR)
    int pos       = 16;
    if (SoC < SOC_MAX) {    // <100% 
      ++pos;
    }
    if (SoC < (SOC_MAX / 10)) {
      ++pos;
    }
    lcd.setCursor(pos, 2);
    unsigned int tempSoC   = (SoC + (SoC / 200)) / 100;   // only show whole percentages
    lcd.print(tempSoC);
    lcd.print("%");
#endif  // (0 != SOC_ESTIMATOR)
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
#if (0 != SOC_ESTIMATOR)
    int bar   = (SOC_MAX / 20);
    while (bar <= SoC) {
      lcd.print((char)255);
      bar     += (SOC_MAX / 20);
    } // end of while bar < 100%

    // Ants Mod to give SoC reading over the top of Bar Display
    if (SoC > (SOC_MAX / 2)) {
      lcd.setCursor(0, 3);    // print SoC on LHS of graph
    }
    else {
      lcd.setCursor(16, 3);   // SoC is less than 50% so print SoC on RHS
    }
//    lcd.print(SoC / 100);
//    lcd.print(tempSoC);   // TODO:
//    lcd.print("%");
#endif  // (0 != SOC_ESTIMATOR)
#else // NO BARGRAPH
#if (0 == DETAILED_INFO)
    lcd.print("Chrgr ");
    if (digitalRead(CHARGER_PIN) == 0) {
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




