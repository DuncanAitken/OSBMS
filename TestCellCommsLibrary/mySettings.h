
// Options
#define debug             (1)       // enable this to have debug messages sent on serial0
#define LCD_DISPLAY       (1)       // 0 = No LCD display, 1 = LCD display fitted
#define DETAILED_INFO     (0)       // 0 = easy to read display, 1 = lots of info on screen
#define READ_CURRENT      (1)       // enable this to sample a current sensor on an ADC pin. The pin in set in the assignments section
#define ACTIVE_BALANCING  (1)       // enable this to balance the cells all the time, otherwise top balancing only.
#define SOC_ESTIMATOR     (1)       // 1 = estimate State-of-Charge, 0 = disable.
#define SHOW_BARGRAPH     (1)       // shows a bargraph of the SoC on the bottom row of the display.

// Settings
#define BATT_MAH          (35000)   // 35Ah or 35,000mAh. yours may be larger!
#define SENT_MEAN_MIN     (3000)    // minimum balancing millivolts sent to cells.
#define SENT_MEAN_MAX     (3600)    // maximum balancing millivolts sent to cells.
#define CHRG_ENABLE_MV    (3400)    // ~80% SoC - enable the charger when cells below this millivolts.
#define HTR_ENABLE_TEMP   (150)     // 15.0c - enable the heater below this temperature.
#define DISCON_OVERTEMP   (450)     // 45.0c - disconnect battery over this temperature.
#define DISCON_OVERVOLT   (3650)    // disconnect battery over this millivolts.
#define DISCON_UNDERVOLT  (2900)    // disconnect battery below this millivolts.
#define DEBUG_BAUD        (9600)    // baudrate for debug comms to the console
#define CELL_LOOP_MS      (9000)    // interval between cell data reads
#define CELL_READ_MS      (600)     // delay after send for the data to shift in
#define AMPS_READ_LOOP_MS (100)     // interval in ms between reading the current sensor.
#define AMPS_OFFSET       (514)     // ~1/2 of the max ADC count as the current sensor is biased so it can read charge and dis-charge currents.
#define AMPS_SCALAR       (97)      // scalar to convert the ADC count into the actual milliamps.
#define AMPS_TRIM         (50)      // mA value to add to scaled result to trim it.
#if (0 != SOC_ESTIMATOR)
#define SOC_UPDATE_MS     (1000)    // interval in ms between updating the State-of-Charge (SoC).
#endif  // SOC_ESTIMATOR

// Pin Assignments
#define ALARM_LED_PIN     (4)       // the buzzer on the shield board
#define FAULT_LED_PIN     (5)       // the yellow LED on the shield board
#define RELAY_PIN         (6)       // the relay on the shield board
#define CONNECT_PIN       (RELAY_PIN)
#define HEATER_PIN        (7)       // enable the cell heater
#define CHARGER_PIN       (8)       // enable/disable the charger
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