/* 
* This code uses multiple serial outputs, it MUST be compiled for MEGA,
* and based on MEGA_Master_1-2 with comms code from Duncan and Dave
*
** July 9th -> Now adding I2C LCD for less wires**
** July 10th -> tidying code
*
*
* This version displays onto an LCD using the I2C piggy back board on the back of a 20 x 4 LCD
* 
* 
*
*
* TX(0) inverted by putting output pin to -ve of Cell to module, and positive of cell top module straight to 
*          positive rail of MEGA.
* RX(0) via Alan Chapmans Comms sub board from Cell top modules at 9600 baud
*
*/


// **** OLD LCD code ***
//#include <LiquidCrystal.h>

// initialize the LCD with the numbers of the interface pins
//LiquidCrystal lcd(48, 46, 44, 42, 40, 38);



// ** NEW LCD CODE **
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
 
LiquidCrystal_I2C       lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
 

#define debug 1
int step = 0;


char initiator_packet[] = {0x9f, 0xff, 0xff, 0xff, 0xff, 0xff}; //TODO: Verify this is in the correct format.

//TODO: Re-factor this into its own class/file...
// used to determine which bits have been corrupted
const uint16_t    u16a_parity_check_matrix[4]     = { 0x8E8, 0x40F, 0x2B3, 0x1D5 };

// used to determine which bits are to be flipped
const uint16_t    u16a_coset_leader_table[16]     = { 0x000, 0x800, 0x400, 0x008,
                                                      0x200, 0x020, 0x002, 0x208,
                                                      0x100, 0x040, 0x004, 0x084,
                                                      0x010, 0x080, 0x001, 0x801
                                                    };
uint8_t    fu8_fec_decode(volatile uint8_t en_msg[], uint8_t de_msg[], uint8_t u8_msg_len);
void    fv_fec_byte_decode(uint16_t *u16_code_word);


String decodeValue(byte payload0, byte payload1, byte decoded0, byte decoded1, float devisor, int precision);


void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);   // NB Ants tried this code with 19200 and no response from cell top modules.
                         // yet with the comms set at 19600 it works sweet :)
                         // TODO - fix this !
                         // I think my modules aren't ready for high speed for some reason.

  // for debugging, press reset on PCB to see via serial what code is running :)
  Serial.println("FILE =->Ants_MEGA_breadboard_I2C_LCD_v2017_07_11th");

 

// NEW LCD code using I2C
  lcd.begin (20,4,LCD_5x8DOTS);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE); // init the backlight


  // Print start-up message to the LCD.
  lcd.setBacklight(HIGH);     // Backlight on
  lcd.clear(); // clear display, set cursor position to zero
  
  lcd.print("      Eco-Ants      ");    // NB must be 20 characters or it garbles rest of the line
  lcd.setCursor(0, 1);
  lcd.print("     BMS master     ");   // NB must be 20 characters or it garbles rest of the line
  lcd.setCursor(0, 2);
  lcd.print("Ants...v2017_07_10th");   // NB must be 20 characters or it garbles rest of the line
  lcd.setCursor(0, 3);
  lcd.print("   Electric  BMW    ");   // NB must be 20 characters or it garbles rest of the line
  // delay so we can read initial banner
  delay(2000);

  lcd.setCursor(0, 2);
  lcd.print("                    ");   // NB must be 20 characters or it garbles rest of the line
  delay(2000);


#ifdef debug
  Serial.println("Starting...");
#endif


  pinMode(13, OUTPUT);//Enable LED

// end of setup
} 


void loop() { 
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  step++;
  //Serial.print(written);
  //Serial.print("#");
  // digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)

  readSerial();
  delay(2000);

  //write 6 bytes to the BMS to initiate data transfer.
  byte written = Serial1.write(initiator_packet, sizeof(initiator_packet));
  Serial1.flush();
  Serial.println("Wrote initiator packet");
// end of loop
}


void readSerial()
{
  const int expected_bytes = 6;
  int bytes_available = Serial1.available();
  int cellNum =  0;

  boolean underV,overV,overT,bal;
  underV = overV = overT = bal = false;
  
  float highVolt = 0.0f;
  float lowVolt = 4.0f;
  int highCellV = -1;
  int lowCellV = -1;

  float highTemp = 0.0f;
  float lowTemp = 200.0f;
  int highCellT = -1;
  int lowCellT = -1;

  

  String outLine = "[";
  while (bytes_available >= expected_bytes ) {
    if (cellNum > 0) {
      outLine += ",\n";
    }
    outLine += cellNum;
    outLine += ":";
    cellNum++;
    String content = String();
    int i = 0;
    
    
    
    unsigned char payload[expected_bytes];
    while (i < expected_bytes) {
      byte currByte = Serial1.read();
      bytes_available --;
      payload[i] = currByte;
      i++;

    }
    //Now lets print the orig and decoded messages....
    int num_dec = 4;
    unsigned char decoded[num_dec];
    fu8_fec_decode(&payload[0], &decoded[0], sizeof(payload));

    //Interpret values.
    //Bytes 0 and 1 make up the voltage.
    //Bytes 2 and 3 make up the temperature.

    //***** Balancing flag *****
    byte balancing = 0;
    String voltage = decodeVoltage(payload[0], payload[1], decoded[0], decoded[1], &balancing);


    //***** uvertemp, uverVolt and underVolt  flags *****
    byte overTemp, overVolt, underVolt;
    String temperature = decodeTemperature(payload[3], payload[4], decoded[2], decoded[3], &overTemp, &overVolt, &underVolt);

    //Can only be under or over voltage per cell
    
    //Ignore the initiator packet
    if(!voltage.equals("NULL")){
      if(overVolt){
        Serial.print(" *** OverVolt:");
        Serial.println(cellNum);
        overV = true;
      }else if(underVolt){
        underV = true;
      }
      if(overTemp){
        overT = true;
      } 
      if(balancing){
        bal = true;     
      }
      float v = voltage.toFloat();
      float t = temperature.toFloat();
      //Serial.print("Testing voltage "+voltage+":");
      //Serial.print(v,3);
      if(highVolt < v){
        highVolt = v;
        highCellV = cellNum-1;
        //Serial.print(" H");
      }
      if(lowVolt > v){
        lowVolt = v;
        lowCellV = cellNum-1;
        //Serial.print(" L");
      }
      if(lowTemp > t){
        lowTemp = t;
        lowCellT = cellNum-1;
        //Serial.print(" L");
      }
      if(highTemp < t){
        highTemp = t;
        highCellT = cellNum-1;
        //Serial.print(" L");
      }
      //Serial.print("\n");
    }

    outLine += balancing; outLine += ";";
    outLine += overVolt; outLine += ";";
    outLine += underVolt; outLine += ";";
    outLine += voltage; outLine += ";";

    outLine += overTemp; outLine += ";";
    outLine += temperature;
  }
  outLine += "]";
  if (cellNum > 0) {
    Serial.println(outLine);

    lcd.clear();                  // start with a blank screen & place cursor at 0,0
    if(!(underV || overV || overT || bal)){
      lcd.print("Status = Good :)");   // just for testing -> need an error flag here !
    }else 
        {
        lcd.print("ALERT ");  
        if(underV){
        lcd.print("uV!");
        if(overV){
        lcd.print(",");
        }
      }if(overV){
        lcd.print("oV");
        if(overT){
          lcd.print(",");
        }
      }if(overT){
        lcd.print("oT");
        if(bal) {
          lcd.print(",");
        }
      }
      if(bal){
        lcd.print("bal");
      }
    }
    lcd.setCursor(0, 1);              // go to start of 2nd line
    lcd.print("v[");lcd.print(lowCellV);lcd.print(",");lcd.print(highCellV);lcd.print("]:");
    lcd.print(String(lowVolt,3));//Need to construct a new string to get more than 3 decimal places of precision.
    lcd.print(", ");
    lcd.print(String(highVolt,3));//Need to construct a new string to get more than 3 decimal places of precision.

    lcd.setCursor(0, 2);              // go to start of 3nd line
    lcd.print("t[");lcd.print(lowCellT);lcd.print(",");lcd.print(highCellT);lcd.print("]:");
    lcd.print(lowTemp);
    lcd.print(", ");
    lcd.print(highTemp);

    lcd.setCursor(0, 3);              // go to start of 4th line
    lcd.print("vD:");
    lcd.print(String(highVolt - lowVolt, 3));
    lcd.print("   tD:");
    lcd.print(String(highTemp - lowTemp, 2));
  }

  
  //Print high and low voltages.
#define PRINT_HIGH_LOW_SERIAL 1
#ifdef PRINT_HIGH_LOW_SERIAL  
  if(cellNum > 0){
    //Only print out if we've read at least 1 cell...
    outLine = "vHigh (c.";
    outLine += highCellV;
    outLine += "):";
    outLine += String(highVolt,3);//Need to construct a new string to get more than 3 decimal places of precision.
    outLine += " vLow (c.";
    outLine += lowCellV;
    outLine += "):";
    outLine += String(lowVolt,3);
    Serial.println(outLine);
  }
#endif

  //  //Read remaining bytes...
  //  bytes_available = Serial1.available();
  //  if(bytes_available > 0){
  //    Serial.print("!! Extra bytes available:");
  //    Serial.print(bytes_available);
  //    Serial.println(". Discarding...");
  //    for(int i = 0; i < bytes_available; ++i){
  //      byte discard = Serial1.read();
  //      String content = " ";
  //      content += i;
  //      content += ":0x" + String(discard,HEX);
  //      content += " ";
  //      Serial.println(content);
  //    }
}


/*
 * Decodes a value, using fec to determine validity.
 * Returns value as a string repesentation of a float, or a string containing 'NULL' if invalid.
 *
 * the 'balancing' parameter is an output.
 */

String  decodeVoltage(byte payload0, byte payload1, byte decoded0, byte decoded1, byte *balancing) {

  float devisor = 1000.0;
  int precision = 3;
  int value =     ((decoded0 & 0x7f) << 8) | decoded1;//Mask off MSB
  //int value =     (decoded0 << 8) | decoded1;//Mask off MSB
  float ret_val = 0.00f;
  *balancing = decoded0 >> 7;

  if (payload0 != decoded0 || payload1 != decoded1) {
    return "NULL";
  } else {
    return String(((float)value / devisor), precision);
  }

}

/*
 * Decodes Temperature, using fec to determine validity.
 * Returns value as a string repesentation of a float, or a string containing 'NULL' if invalid.
 */

String  decodeTemperature(byte payload0, byte payload1, byte decoded0, byte decoded1, byte *overTemp, byte *overVolt, byte *underVolt) {

  float devisor = 10.0;
  int precision = 2;
  int value =     ((decoded0 & 0x1f) << 8) | decoded1;
  float ret_val = 0.00f;
  *overTemp = decoded0 >> 7;
  *overVolt = ((decoded0  & 0x40 ) >> 6);
  *underVolt = ((decoded0 & 0x20) >> 5);

  if (payload0 != decoded0 || payload1 != decoded1) {
    return "NULL";
  } else {

    return String(((float)value / devisor), precision);
  }

}
/*
 * Decodes a value, using fec to determine validity.
 * Returns value as a string repesentation of a float, or a string containing 'NULL' if invalid.
 */
String decodeValue(byte payload0, byte payload1, byte decoded0, byte decoded1, float devisor, int precision) {
  //TODO: fec part...

  int value =     (decoded0 << 8) | decoded1;
  float ret_val = 0.00f;
  //char ret_val[6];//
  if (payload0 != decoded0 || payload1 != decoded1) {
    return "NULL";
  } else {
    //dtostrf(((float)value / devisor),5,3,ret_val);
    return String(((float)value / devisor), precision);
    //ret_val_float /= devisor;
    //ret_val += ret_val_float;//(((float)value) / devisor);
  }

}


uint8_t    fu8_fec_decode(volatile uint8_t en_msg[], uint8_t de_msg[], uint8_t u8_msg_len)
{
  static const uint8_t LDPC_BUF_SIZE = 50;
  static uint8_t  u8a_decode[LDPC_BUF_SIZE];
  //  static uint8_t  u8a_save[LDPC_BUF_SIZE];
  uint16_t    u16a_code_word[4];
  uint8_t     u8_i, u8_j, u8_len;

  uint8_t     u8a_tmp[LDPC_BUF_SIZE];

  // only decode in blocks of six
  u8_msg_len  = ((u8_msg_len / 6) * 6);
  u8_len    = 0;

  if (u8_msg_len > 6) {
    u8_i  = u8_msg_len - 6;
  }
  else {
    u8_i  = 0;
  }


  // re-distribute the bits into their correct byte
  //  for (; u8_i < u8_msg_len; u8_i += 6) {
  for (u8_i = 0; u8_i < u8_msg_len; u8_i += 6) {
    for (u8_j = 0; u8_j < 6; u8_j++) {

      u8a_tmp[u8_i + u8_j]  = en_msg[u8_i + u8_j];
    } // end of for each byte in a block

    // code-words
    u16a_code_word[0] = u8a_tmp[u8_i + 0] + ( (u8a_tmp[u8_i + 2] & 0x0F) << 8);
    u16a_code_word[1] = u8a_tmp[u8_i + 1] + ( (u8a_tmp[u8_i + 2] & 0xF0) << 4);
    u16a_code_word[2] = u8a_tmp[u8_i + 3] + ( (u8a_tmp[u8_i + 5] & 0x0F) << 8);
    u16a_code_word[3] = u8a_tmp[u8_i + 4] + ( (u8a_tmp[u8_i + 5] & 0xF0) << 4);

    for (u8_j = 0; u8_j < 4; u8_j++) {
      // decode byte in packet
      fv_fec_byte_decode(&u16a_code_word[u8_j]);

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
}

/*******************************************************************************
 * Decodes a 12-bits into a 8-bits using the fec
 *******************************************************************************/
void    fv_fec_byte_decode(uint16_t *u16_code_word)
{
  uint8_t     u8_p_matrix, u8_syndrome = 0;
  uint16_t    u16_decode;

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
} // end of fv_fec_byte_decode function ----------------



