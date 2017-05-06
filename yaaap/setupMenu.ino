/*
   YAAAP Yet Another Arduino AutoPilot
   setupMenu.ino
   2016 Philippe Leclercq

*/
#include <string.h>
#include <avr/pgmspace.h>

#include "I2Cdev.h"
#include "CalLib.h"
#define BACKLIGHTPIN 10

int line = 0;

//Flash based string table. Necessary because otherwise the strings will take up all of our ram.
//Use either PrintLCDAt_P or PrintLCD_P to print these strings to the LCD.
const char string_0[] PROGMEM = "1. Exit setup";
const char string_1[] PROGMEM = "2. Update Kp";
const char string_2[] PROGMEM = "3. Update Kd";
const char string_3[] PROGMEM = "4. Upd deadband";
const char string_4[] PROGMEM = "5. Calibration";
const char string_5[] PROGMEM = "6. Save";
const char string_6[] PROGMEM = "7. Reset params";
#define LINES 7
const char* const StringTable[] PROGMEM = {
  string_0, string_1, string_2, string_3, string_4, string_5, string_6
};

//this is the gatekeeper function. The only one to really touch the strings in program memory.
// This keeps the need for a buffer string to only one.

void printLCD_P(int which) {
  char buffer[17];
  strcpy_P(buffer, (char*)pgm_read_word(&(StringTable[which])));
  lcd.print(buffer);
  delay(40);
}

/***********************************************************
   Buttons for setup : 1 down/starboard, 2 select, 3 up/port
 ***********************************************************
*/
void downClick() {
  keyPressed = 'd';
  beep(true);
}
void selClick() {
  keyPressed = 's';
  beep(true);
}
void upClick() {
  keyPressed = 'u';
  beep(true);
}
void doNothing() {
}

void menuButtonsInit() {
  button1.attachClick(downClick);
  button1.attachLongPressStop(doNothing);
  button1.attachDuringLongPress(doNothing);

  // link the button 2 functions.
  button2.attachClick(selClick);
  button2.attachDoubleClick(doNothing);
  button2.attachLongPressStop(doNothing);

  // link the button 3 functions.
  button3.attachClick(upClick);
  button3.attachLongPressStop(doNothing);
  button3.attachDuringLongPress(doNothing);
}

/***********************************************************
   update a parameter
 ***********************************************************
*/
int updateValue(int value, const char name[]) {
  while (true) {
    lcd.setCursor(0, 1);
    lcd.print(name);
    lcd.print(" ");
    printJustified(value);
    lcd.print("    ");
    keyPressed = ' ';
    while (keyPressed == ' ')
      buttonsTick();

    switch (keyPressed) {
      case 'u':
        value++;
        break;
      case 'd':
        value--;
        break;
      case 's':
        return value;
    }
  }
}

/***********************************************************
   Menu
 ***********************************************************
*/
void setupMenu() {
  bool exitMenu = false;
  menuButtonsInit();

  // At first, show current values
  // until a key is pressed
  lcd.clear();
  //	lcd.setCursor(0, 0);
  //	lcd.print("O ");
  //	printJustified(offX);
  //  printJustified(offY);
  //  printJustified(offZ);
  lcd.setCursor(0, 1);
  lcd.print("K ");
  printJustified(Kp);
  printJustified(Kd);
//  printJustified(Kdd);
  // special functions - First level - lightOff / Menu / lightOn
  keyPressed = ' ';
  while (keyPressed != 's') {
    buttonsTick();
    switch (keyPressed) {
      case 'u':
        params.backlight = 255;
        analogWrite(BACKLIGHTPIN, params.backlight);
        break;
      case 'd':
        params.backlight = 10;
        analogWrite(BACKLIGHTPIN, params.backlight);
        //       lcd.setBacklight(0); // Off
        break;
    }
  }

  RTVector3 gyroBias;
  // special functions - 2nd level
  while (!exitMenu) {
    lcd.clear();
    lcd.setCursor(0, 0);
    printLCD_P(line);
    //		lcd.print(StringTable[line]);
    keyPressed = ' ';
    while (keyPressed == ' ')
      buttonsTick();

    switch (keyPressed) {
      case 'd':
        if (line > 0) line--;
        break;
      case 'u':
        if (line < LINES) line++;
        break;
      case 's':
        switch (line) {
          case 0:
            exitMenu = true;
            break;
          case 1:
            Kp = updateValue(Kp, "Kp");
            params.Kp = Kp;
            break;
          case 2:
            Kd = updateValue(Kd, "Kd");
            params.Kd = Kd;
            break;
          case 3:
            deadband = (byte)updateValue(deadband, "deadband");
            params.deadband = deadband;
            break;
          case 4: //Calibration
  //          compassCalibration();
            break;
          case 5:
            if (imu->IMUGyroBiasValid()) { // save gyroBias data if valid
              gyroBias = imu->getGyroBias();
              params.gyroBiasValid = 0x1;
              params.gyroBias[0] = gyroBias.x();
              params.gyroBias[1] = gyroBias.y();
              params.gyroBias[2] = gyroBias.z();
            }
            calLibWrite(0, &params);
            lcd.setCursor(0, 1);
            lcd.print(F("Saved to EEPROM"));
            delay(1000);
            exitMenu = true;
            break;
          case 6: //reset data on EEPROM
            params.gyroBiasValid = 0;
            params.gyroBias[0] = 0;
            params.gyroBias[1] = 0;
            params.gyroBias[2] = 0;
            params.Kp = 3;
            params.Kd = 30;
            params.deadband = 4;
            params.reserved = 0;
            params.backlight = 255;
            calLibWrite(0, &params);
            exitMenu = true;
            break;
        }
        line = 0; //Default to Exit after any selection
    }
  }
  lcd.clear();
  buttonsInit();
}


