/*****************************************************
* Test setup for the CellComms library.
*
* The CellComms library needs two calls to be made periodically
* 1. A call to sendMillivolts every sample period (5s or so)
* 2. A call to readCells shortly afterwards.
*****************************************************/


#include <CellComms.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR    0x27  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

LiquidCrystal_I2C       lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);



#define debug             (1)
#define DEBUG_BAUD        (9600)    // baudrate for debug comms to the console
#define CELL_LOOP_MS      (5000)    // interval between cell data reads
#define CELL_READ_MS      (400)     // delay after send for the data to shift in
#define AMPS_READ_LOOP_MS (100)
#define AMPS_OFFSET       (512)
#define AMPS_SCALAR       (97)

#define ALARM_LED_PIN     (4)
#define FAULT_LED_PIN     (5)
#define RELAY_PIN         (6)
#define HEART_LED_PIN     (13)
#define AMPS_AN_PIN       (0)

#define NUM_CELLS 6

int lowestCellEverV = 3500;
unsigned int cellMeanMillivolt  = 3600;   // initiate with a high mean value so the loads don't turn on.
unsigned long currentMillis;
unsigned long sendMillis;
unsigned long readMillis;
unsigned long readAmpsMillis;
unsigned long updateSoCMillis;
long milliAmps;
long aveMilliAmps;

CellComms cells;

void setup() {
  Serial.begin(DEBUG_BAUD);

#ifdef debug
  Serial.println("Starting...");
  // for debugging, press reset on PCB to see via serial what code is running :)
  Serial.println("FILE =->Ants_MEGA_shield_I2C_Library");
  Serial.print("Num cells:");
  Serial.println(NUM_CELLS);
#endif

  // NEW LCD code using I2C
  lcd.begin (20, 4, LCD_5x8DOTS);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE); // init the backlight


  // Print start-up message to the LCD.
  lcd.setBacklight(HIGH);     // Backlight on
  lcd.clear(); // clear display, set cursor position to zero

  lcd.print("      Eco-Ants      ");    // NB must be 20 characters or it garbles
  lcd.setCursor(0, 1);
  lcd.print("     BMS master     ");   // NB must be 20 characters or it garbles
  lcd.setCursor(0, 2);
  lcd.print("Ants_MEGA_shield_I2C");   // NB must be 20 characters or it garbles
  lcd.setCursor(0, 3);
  lcd.print("   Electric  BMW    ");   // NB must be 20 characters or it garbles
  // delay so we can read initial banner
  delay(2000);

  lcd.setCursor(0, 2);
  lcd.print("                    ");   // NB must be 20 characters or it garbles
  delay(2000);



  pinMode(HEART_LED_PIN, OUTPUT);       // Enable Heart LED
  pinMode(FAULT_LED_PIN, OUTPUT);       // Enable Fault LED
  pinMode(ALARM_LED_PIN, OUTPUT);       // Enable Alarm LED
  pinMode(RELAY_PIN, OUTPUT);           // Enable Relay

  digitalWrite(HEART_LED_PIN, HIGH);
  digitalWrite(FAULT_LED_PIN, HIGH);
  delay(200);
  digitalWrite(ALARM_LED_PIN, HIGH);
  delay(20);
  digitalWrite(FAULT_LED_PIN, LOW);
  digitalWrite(ALARM_LED_PIN, LOW);

  aveMilliAmps        = 0;
  currentMillis       = millis();
  sendMillis          = (currentMillis + 100);
  readAmpsMillis      = (currentMillis + 50);
  updateSoCMillis     = (currentMillis + 1025);
}

void loop() {
  // this is essentially just a scheduler to call functions at different intervals.

  currentMillis       = millis();

  if (currentMillis >= sendMillis) {
    startCellComms(CELL_READ_MS);
  } // end of sendMillis

  if (currentMillis >= readMillis) {
    readCellComms();
  } // end of readMillis

  if (currentMillis >= readAmpsMillis) {
    readAmps();
  } // end of readMillis

  if (currentMillis >= updateSoCMillis)
  {
    updateSoC();
  }
} // end of loop


/******************************************
  initiate cell comms by sending the mean cell voltage.
  interval between sends can be varied.
******************************************/
void startCellComms(uint16_t interval) {
  digitalWrite(HEART_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(FAULT_LED_PIN, LOW);

  // write 6 bytes to the BMS to initiate data transfer.
  cells.sendMillivolts(cellMeanMillivolt);

#ifdef debug
  //    Serial.println("Sent cell mean");
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
  int lowestCellV = cells.getCellsMinV();
  int highestCellV = cells.getCellsMaxV();

  if (cellsRead == NUM_CELLS) {
    cellMeanMillivolt   = cells.getCellsAveV();

    lcd.clear(); // clear display, set cursor position to zero
    lcd.print(cellsRead);
    lcd.print(" cells read :) ");


#ifdef debug
    Serial.print("cell mean ");
    Serial.println(cellMeanMillivolt);
#endif



    lcd.setCursor(0, 1);
    lcd.print("Max voltage = ");
    lcd.print(highestCellV);
    lcd.setCursor(0, 2);
    lcd.print("Min voltage = ");
    lcd.print(lowestCellV);

    if (lowestCellV < lowestCellEverV) {
      lowestCellEverV = lowestCellV;
    }
    lcd.setCursor(0, 3);
    lcd.print("Extreme Low = ");
    lcd.print(lowestCellEverV);


    if (lowestCellV < 2900) {                      // not good - sound brief alarm
      digitalWrite(ALARM_LED_PIN, HIGH);
      delay(20);
      digitalWrite(ALARM_LED_PIN, LOW);
    }


    if (lowestCellV < 2800) {                         // even worse - sound brief alarm again
      digitalWrite(ALARM_LED_PIN, HIGH);
      delay(20);
      digitalWrite(ALARM_LED_PIN, LOW);
      delay(20);
      digitalWrite(ALARM_LED_PIN, HIGH);
      delay(20);
      digitalWrite(ALARM_LED_PIN, LOW);
    }

    if (lowestCellV < 2600) {                          // OH SHIT - serious alarm -
      digitalWrite(ALARM_LED_PIN, HIGH);
      delay(500);
      digitalWrite(ALARM_LED_PIN, LOW);
      delay(200);
      digitalWrite(ALARM_LED_PIN, HIGH);
      delay(500);
      digitalWrite(ALARM_LED_PIN, LOW);
    }






    // Check conditions to turn the charger ON
    // if any cells are under-voltage then turn the charger on
    if (cells.getCellsUnderVolt() > 0) {
      digitalWrite(RELAY_PIN, HIGH);   // turn charger on
    }
    // enable charger if SoC < 80% ?
    if (cells.getCellsMaxV() < 3500)
    {
      digitalWrite(RELAY_PIN, HIGH);
    }

    // TODO: may have a manual CHARGE button as well.


    // Check conditions to turn charger OFF
    // if all the cells are balancing then turn the charger off
    if (cells.getCellsBalancing() == NUM_CELLS) {
      digitalWrite(RELAY_PIN, LOW);   // turn charger off
    }
    // if any cells are overvoltage then turn the charger off
    if (cells.getCellsOverVolt() > 0) {
      digitalWrite(RELAY_PIN, LOW);   // turn charger off
    }
    // if any cells are over-temperature then turn the charger off
    if (cells.getCellsOverTemp() > 0) {
      digitalWrite(RELAY_PIN, LOW);   // turn charger off
    }


    // TODO: do other stuff because the read was good.
  } // end of all cells read successfully
  else {

    digitalWrite(FAULT_LED_PIN, HIGH);
    lcd.clear();
    lcd.print("My arms are flailing");
    lcd.setCursor(0, 1);
    lcd.print("  Error  !! ");
    lcd.setCursor(0, 2);
    lcd.print(cellsRead); lcd.print(" out of "); lcd.print(NUM_CELLS);
    lcd.setCursor(0, 3);
    lcd.print("Break before # "); lcd.print((NUM_CELLS - cellsRead));

#ifdef debug
  Serial.print("Not all cells read! Cell mean:");
  Serial.println(cellMeanMillivolt);
  Serial.print("Lowest:");
  Serial.println(lowestCellV);
  Serial.print("Highest:");
  Serial.println(lowestCellV);
  Serial.println("__________");
  
  
#endif

  } // end of not all cells read



#ifdef debug
  Serial.print("cells read ");
  Serial.println(cellsRead);
#endif


  readMillis            += CELL_LOOP_MS;  // move the trigger value on so it doesn't run 100 times

  digitalWrite(HEART_LED_PIN, LOW);   // turn the LED off to show we're done
} // end of readCellComms ---------------------------


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

  readAmpsMillis        += AMPS_READ_LOOP_MS;
} // end of readAmps --------------------------------


/****************************************************
 * updates the State of Charge estimate
 * called every 1000ms
 ***************************************************/
void  updateSoC(void)
{
  static unsigned int relaxedTime    = 0;

  // if ave current is above noise floor
  if ( (aveMilliAmps > 500) || (aveMilliAmps < -500) )
  {
    // count coulombs

    relaxedTime         = 0;
  }
  else  if ( (aveMilliAmps < 100) && (aveMilliAmps > -100) )
  {
    ++relaxedTime;
    if (relaxedTime > (60 * 5))   // 5 minutes
    {
      // read Voc

      relaxedTime       = 0;
    }
  }

  updateSoCMillis       += 1000;
} // end of updateSoC ------------------------------


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
    return String(( (float) value / devisor), precision);
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




