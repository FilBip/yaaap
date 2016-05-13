/*
   YAAAP Yet Another Arduino AutoPilot
   yaaap.ino
   2016 Philippe Leclercq

   Heading is given by the 9dof IMU MPU9250 using the RTIMUlib for Arduino by Richards-Tech.
   RTIMUlib filter is especially effective to keep a steady course when the heel angle varies.
   I reduced the impact of magnetometer instability with a reduced SLERP value: fusion.setSlerpPower(0.001)
   The tiller actuator is the old cylinder from the original Autohelm 2000 tillerPilot (with Omron motor 2332 12V)
   Typical current for this motor is 1.6A. The cheap TB6612FNG can drive this motor with the 2 channels in parrallel (2A typical to 6A peak).
   It can be replaced by a more powerfull driver like IBT_2 with BTN7970B. 
   I dropped the usage of PWM with my old small motor. Minimal pulse is 10ms, increasing to continuous depending
   on the computed command(PID).
   There is no device to read the tiller position. Current to the motor is controlled to detect end of course or overload (ACS712).
   The interface is minimal, made of 3 buttons using OneButtons library https://github.com/mathertel/OneButton
   The LCD display is an I2C version.
   
   Instructions
	The device is fixed behind the tiller facing the route.
 	Button 1 on the left: sarboard/less fonctions
	button 2 on center: , select/run-standby fonctions
	Button 3 on the right: port/plus fonctions
	
	On startup, after initialization, the device is on standby, bearing to the current direction.
	The display shows the status(Sby/Run), baering, heading and error (difference to baering).
   
	To run, click once on button 2.

	During run, click on button 1 or 3 increases (resp. decrease) 1 deg of bearing.
				long press on button 1 or 3 increases (resp. decreases) 20 deg of bearing
				long press on button 1 and 2 (starboard and select) to tack to starboard
				long press on button 3 and 2 (port and select) to tack to port
				double click on button 2 to stop motor and standby
	
	During standby,	long press button 1 push the tiller
				long press button 3 pull the tiller
				long press button 1 and 3 to enter setup

				On setup first level,	click 1(-) for dimmed backlight
							click 3(+) for full backlight
							click 2(sel) to enter setup menu
				On setup menu,		click 1(-) to go to previous menu item
							click 3(+) to go to next menu item
							click 2(sel) to select menu item
				On setup item,		click 1(-) to decrease value if applicable
						        click 3(+) to increase value if applicable
							click 2(sel) to validate entry/value

	To calibrate the compass, chose the menu item, draw 8 with the device. The display shows the offsets.
	Click button 2 when OK.
	Important: go to the saving menu item and validate to write calibration and other parameters to EEPROM.
   
   */

#define DEBUG 1
#define SERIALDEBUG 1
//#define SERVO 1
#define TACKANGLE 100 // In degrees
#define OVERLOADCURRENT 3000 // milliAmps
#define OVERLOADDELAY 5000 // milliseconds
#define DEADBAND 4 // Deadband in degrees
// motor definitions
//#define MOTORDRIVER 1 // BTN7970B
#define MOTORDRIVER 2 // TB6612FNG

#if MOTORDRIVER == 1 // BTN7970B
#define RPWM 3
#define LPWM 4
#define R_EN 5
#define L_EN 6
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
#define A_PWM 3
#define A_IN2 4
#define A_IN1 5
#define STBY 6
#define B_IN1 7
#define B_IN2 8
#define B_PWM 9
#endif
#define ACS712PIN A0
#define BACKLIGHTPIN 10

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "OneButton.h"
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"


#define LED_PIN 13
unsigned long ledStart = 0;

// Setup OneButtons on pins A1, A2, A3
OneButton button1(A1, true);
OneButton button2(A2, true);
OneButton button3(A3, true);

long refVoltage = 0;

float cmd = 0;
bool standby = true;
bool launchTack = false;
float heading = 0, bearing = 0, headingError = 0;
float previousError = 0, deltaError = 0, deltaErrorDeriv = 0, previousDeltaError = 0;
//int prevLoop = 0;
unsigned long previousTime = 0, deltaTime;
bool setupFonctions = false;
char keyPressed = ' ';
unsigned long prevDisplay = 0;
#define DISPLAY_INTERVAL  500                         // interval between pose displays


CALLIB_DATA calData;                                  // the calibration data and other EEPROM params

int Kp = 10, Kd = 15, Kdd = 0; // proportional, derivative1 and derivative2 coefficients (tiller position do the integral/sum term)

// LCD display interface pins
//LiquidCrystal lcd(12, 11, 10, 9, 8, 7);
LiquidCrystal_I2C lcd(0x27, 16, 2);
#ifdef SERVO // For development and code test only
#include <Servo.h>
Servo myservo;
int servoAngle = 90;
#endif

/***********************************************************
   Subroutines
 ***********************************************************
*/
int sign(int val) { // declare the missing sign() fn
    return (val>0) - (val<0);
}

void printJustified(int value) {
  char buffer[7];
  sprintf(buffer, "%4d", value);
  lcd.print(buffer);
}
float map360(float deg) {
  if (deg < 0) deg += 360;
  else if (deg > 360) deg -= 360;
  return deg;
}
void beep(bool on) { // No sound, only a flashing led
  if (on) {
    digitalWrite(LED_PIN, true);
    ledStart = millis();
  }
  else if ((millis() - ledStart) > 100 ) {
    ledStart = 0; // End of beeb/light. Reset delay
    //   digitalWrite(LED_PIN, false);
  }
}

/***********************************************************
   Tiller/motor management
***********************************************************

*/
byte pulseState = LOW;
unsigned long pulseCurrentMillis;
unsigned long pulseStartMillis;
int tillerCurrent = 0;
int overloadDirection = 0;
unsigned long overloadStart = 0;

void tillerStandby(boolean state) {
  if (state == true) { // stop mode
#if MOTORDRIVER == 1 // BTN7970B
    analogWrite(RPWM, LOW);
    analogWrite(LPWM, LOW);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
    digitalWrite(A_PWM, LOW);
    digitalWrite(B_PWM, LOW);
#endif
    pulseState = LOW;
    digitalWrite(LED_PIN, false);
  }
  //else // Standby mode not used
  //  digitalWrite(R_EN,HIGH);
}

void tillerInit() {
  // Init over current protection device (ACS712PIN)
  //  Serial.print("estimating avg. quiscent voltage:");
  //read X samples to stabilize value
  static const int nbSamples = 200;
  for (int i = 0; i < nbSamples; i++) {
    refVoltage += analogRead(ACS712PIN);
    delay(1);//depends on sampling (on filter capacitor), can be 1/80000 (80kHz) max.
  }
  refVoltage /= nbSamples;
  Serial.print(map(refVoltage, 0, 1023, 0, 5000)); Serial.println(" mV");
#if MOTORDRIVER == 1 // BTN7970B
  analogWrite(RPWM, LOW);
  analogWrite(LPWM, LOW);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
  pinMode(STBY, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(B_PWM, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);
  digitalWrite(STBY, HIGH);
#endif
  pulseStartMillis = millis();

  tillerStandby(true);
}

bool tillerOverload(int commandDirection) {
  const int sensitivity = 185;//change this to 100 for ACS712PIN-20A or to 66 for ACS712PIN-30A
  bool overload = false;
  if (overloadStart) {
   if (overloadDirection == commandDirection) {       // If same direction than last detected overload
    if ((millis() - overloadStart) < OVERLOADDELAY )  // wait delay
      overload = true;
    else
      overloadStart = 0; // End of pause. Reset delay
    return overload;
   }
   else
      overloadStart = 0; // Direction has changed. Reset delay
  }
  int current = 0;
  //read some samples to stabilize value
  for (int i = 0; i < 5; i++) {
    current = current + analogRead(ACS712PIN);
  }
  current /= 5;
   current -=  refVoltage;
  tillerCurrent = current * 5000000 / 1023 / sensitivity; // to mA
  if (abs(tillerCurrent) > OVERLOADCURRENT) {
    overload = true;
    overloadStart = millis();
    overloadDirection=commandDirection;
    tillerStandby(true);
    lcd.setCursor(0, 1);
    lcd.print(F(" OVERLOAD "));
  }
  else
    overloadDirection=0;

  return overload;
}

void tillerPush() {
#if MOTORDRIVER == 1 // BTN7970B
  analogWrite(LPWM, HIGH);
  analogWrite(RPWM, LOW);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, HIGH);
  digitalWrite(B_IN1, LOW);
  digitalWrite(B_IN2, HIGH);
#endif
#ifdef SERVO
  servoAngle += 1;
  if (servoAngle > 180) servoAngle = 180;
  myservo.write(servoAngle);
#endif
}

void tillerPull() {
#if MOTORDRIVER == 1 // BTN7970B
  analogWrite(LPWM, LOW);
  analogWrite(RPWM, HIGH);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
  digitalWrite(A_IN1, HIGH);
  digitalWrite(A_IN2, LOW);
  digitalWrite(B_IN1, HIGH);
  digitalWrite(B_IN2, LOW);
#endif
#ifdef SERVO // tests
  servoAngle -= 1;
  if (servoAngle < 0) servoAngle = 0;
  myservo.write(servoAngle);
#endif
}

void tillerCommand(int tillerCmd) {
  int pulsePeriod;

  if (tillerCmd == 0) {
    tillerStandby(true);
    return;
  }
  // Current / motor overload / end of course control
  if (tillerOverload(sign(tillerCmd)))
    return;

  pulseCurrentMillis = millis();
  int pulseWidth = 20 + abs(tillerCmd) * 18 / 10; // Higher command, longer pulse
  if (pulseCurrentMillis - pulseStartMillis <= pulseWidth) {
    pulseState = HIGH;
    digitalWrite(LED_PIN, true);
  }
  else {
    if (abs(tillerCmd) < 100) {
      pulseState = LOW;
      digitalWrite(LED_PIN, false);
      //     Serial.print("pulseW="); Serial.print(pulseWidth); Serial.print(" "); Serial.print(pulseCurrentMillis); Serial.print(" - "); Serial.print(pulseStartMillis); Serial.println(" ");
    }
    pulseStartMillis = millis();
  }

  if (tillerCmd > 0)
    tillerPull();
  else
    tillerPush();

#if MOTORDRIVER == 1 // BTN7970B
  digitalWrite(R_EN, pulseState);
  digitalWrite(L_EN, pulseState);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
  digitalWrite(A_PWM, pulseState);
  digitalWrite(B_PWM, pulseState);
#endif
}

/***********************************************************
   Compute tiller command
 ***********************************************************
*/
int computeCmd() {
  headingError = heading - bearing;
  if (abs(headingError) < DEADBAND / 2) {
    headingError = 0;
    cmd = 0;
  }
  else {
    if (abs(headingError) > 180) {
      if (bearing > heading)
        headingError += 360;
      else
        headingError -= 360;
    }
  }
  unsigned long now = millis();
  deltaTime = now - previousTime;
  previousTime = now;
  deltaError = headingError - previousError;
  previousError = headingError;
  deltaErrorDeriv = deltaError - previousDeltaError;
  previousDeltaError = deltaError;

  float tmp = (Kp * headingError + (Kd * deltaError + Kdd * deltaErrorDeriv) * 1000 / deltaTime) / 10; // by time in sec
  cmd = (cmd + tmp) / 2; // Average with last cmd for smoother cmd changements - test
   if (cmd > 100) cmd = 100;
  if (cmd < -100) cmd = -100;
#ifdef DEBUG
  // lcd.clear();
  lcd.setCursor(0, 1);
  //printJustified((int)(Kp*headingError));
  printJustified((int)(Kd * deltaError * 1000 / deltaTime));
  printJustified((int)(Kdd * deltaErrorDeriv * 1000 / deltaTime));
  printJustified((int)cmd);
  lcd.print("  ");
#endif
#if SERIALDEBUG==2
  Serial.print(deltaTime);
  Serial.print(" ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(headingError);
  Serial.print(" ");
  Serial.print(deltaError);
  Serial.print(" ");
  Serial.println(cmd);
#endif
  return (int)cmd;
}
/***********************************************************
   Buttons management
 ***********************************************************
*/
// This function will be called when the button1 was pressed 1 time
void b1Click() { // One degree to starboard
  bearing = map360(bearing + 1);
  beep(true);
}
void b1LongPress() { // During button pressed
  if (standby)  {
    // During Standby, if buttons 1 and 3 pressed, setup/calibration
    // else move tiller
    if (button3.isLongPressed())
      setupFonctions = true;
    else
      tillerCommand(-100);
  }
  else {
    beep(true);
    if (!launchTack && button2.isLongPressed()) { // Tack if two buttons pressed
      launchTack = true;
      bearing = map360(bearing + TACKANGLE);
    }
  }
}
void b1LongPressStop() { // Move to starboard
  if (!launchTack) bearing = map360(bearing + 20);
  if (standby) tillerStandby(true); // Stop the tiller at the end of press during standby
}
void b2Click() { // standby off
  if (standby) beep(true);
  standby = false;
}
void b2DoubleClick() { // standby on
  if (!standby) beep(true);
  standby = true;
  tillerStandby(standby);
}
void b2LongPressStop() { // reset steering or launch tack
  beep(true);
  if (launchTack)
    launchTack = false; // steering update already done
  else {
    lcd.clear();
    lcd.print("Reset steering");
    bearing = heading;
    //    delay (100);
  }
}
void b3Click() { // One degree to badboard
  beep(true);
  bearing = map360(bearing - 1);
}
void b3LongPress() { // During button pressed
  if (standby)  {
    // During Standby, if buttons 1 and 3 pressed, calibration
    // else move tiller
    if (button1.isLongPressed())
      setupFonctions = true;
    else
      tillerCommand(+100);
  }
  else {
    beep(true);
    if (!launchTack && button2.isLongPressed()) { // Tack if two buttons pressed
      launchTack = true;
      bearing = map360(bearing - TACKANGLE);
    }
  }
}

void b3LongPressStop() { // Move to badboard
  if (!launchTack) bearing = map360(bearing - 20);
  if (standby) tillerStandby(true); // Stop the tiller at the end of press during standby
}

void buttonsInit() {
  static const int clickTicks = 500, pressTicks = 900;
  // link the button 1 functions.
  button1.setClickTicks(clickTicks);
  button1.setPressTicks(pressTicks);
  button1.attachClick(b1Click);
  // button2.attachDoubleClick(b1DoubleClick);
  //button1.attachLongPressStart(b1LongPressStart);
  button1.attachLongPressStop(b1LongPressStop);
  button1.attachDuringLongPress(b1LongPress);

  // link the button 2 functions.
  button2.setClickTicks(clickTicks);
  button2.setPressTicks(pressTicks);
  button2.attachClick(b2Click);
  button2.attachDoubleClick(b2DoubleClick);
  //button2.attachLongPressStart(b2LongPressStart);
  button2.attachLongPressStop(b2LongPressStop);
  //button2.attachDuringLongPress(b2LongPress);

  // link the button 3 functions.
  button3.setClickTicks(clickTicks);
  button3.setPressTicks(pressTicks);
  button3.attachClick(b3Click);
  // button3.attachDoubleClick(b3DoubleClick);
  //button3.attachLongPressStart(b3LongPressStart);
  button3.attachLongPressStop(b3LongPressStop);
  button3.attachDuringLongPress(b3LongPress);
}

void buttonsTick() {
  beep(false);
  button1.tick();
  button2.tick();
  button3.tick();
}

/***********************************************************
   Initial setup
 ***********************************************************
*/
void setup() {

  Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
#if SERIALDEBUG
  Serial.begin(115200);
  Serial.println("Initializing...");
#endif
  pinMode(LED_PIN, OUTPUT);
  pinMode(BACKLIGHTPIN, OUTPUT); 

  lcd.setBacklight(1);
  analogWrite(BACKLIGHTPIN, 255);
  lcd.begin();
  lcd.setCursor(0, 0);
  buttonsInit();
  lcd.setCursor(0, 0);
  lcd.print("Init tiller ");
  tillerInit();
  lcd.setCursor(0, 0);
  lcd.print("Init compass");
  compassInit();
  Kp = calData.Kp;
  Kd = calData.Kd;
  Kdd = calData.Kdd;
  bearing = compassHeading();

  //prevLoop = millis();
  lcd.setCursor(0, 0);
  lcd.print("Setup done  ");

}

/***********************************************************
   Main loop
 ***********************************************************
*/

void loop() {
  /*
     Main loop
   	read keypad/buttons
   	read heading
   	compute steering (PID controller)
   	steer
  */
  int ctrl;
  buttonsTick();
  if (setupFonctions) {
    setupMenu();
    setupFonctions = false;
  }
  heading = compassHeading();
  ctrl = computeCmd();
  if (!standby)
    tillerCommand(ctrl);

  unsigned long now = millis();
  if ((now - prevDisplay) >= DISPLAY_INTERVAL) {
    prevDisplay = now;

    lcd.setCursor(0, 0);
    if (standby)
      lcd.print("Sby");
    else
      lcd.print("Run");

    printJustified((int)bearing);
    printJustified((int)heading);
    printJustified((int)headingError);
    //printJustified(tillerCurrent);
    //lcd.print("mA  ");

    lcd.print("   ");
  }

  // int waitDelay = 1000 / COMPASSFREQ - millis() + prevLoop;
  // if (waitDelay > 0) delay(waitDelay);
  //  prevLoop = millis();
}
