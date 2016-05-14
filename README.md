# YAAAP Yet Another Arduino AutoPilot

Yaaap  is a simple autopilot for a sailboat using Arduino pro mini card and breakouts
The interface is reduced to 3 buttons and an LCD display.
The project was intended to replace the CPU of an old analog pilot Autohelm 2000 retaining the original electric actuator.


## Features

Heading is given by the 9dof IMU MPU9250 using the RTIMUlib for Arduino by Richards-Tech.
RTIMUlib filter is especially effective to keep a steady course when the heel angle varies.
I reduced the impact of magnetometer instability with a reduced SLERP value: fusion.setSlerpPower(0.001)
The tiller actuator is the old cylinder from the original Autohelm 2000 tillerPilot (with Omron motor 2332 12V)
Typical current for this motor is 1.6A. The cheap TB6612FNG can drive this motor with the 2 channels in parrallel (typical 2A to peak 6A).
It can be replaced by a more powerfull driver like IBT_2 with BTN7970B. 
I dropped the usage of PWM with my old small motor. Minimal pulse is 10ms, increasing to continuous depending on the computed command(PID).
There is no device to read the tiller position. Current to the motor is controlled to detect end of course or overload (ACS712).
The interface is minimal, made of 3 buttons using OneButtons library https://github.com/mathertel/OneButton
The LCD display is an I2C version.
   
## Instructions
The device is fixed behind the tiller facing the route.
Button 1 on the left: sarboard/less fonctions
button 2 on center: select/run-standby fonctions
Button 3 on the right: port/plus fonctions
	
### Startup
On startup, after initialization, the device is on standby, bearing to the current direction.
The display shows the status(Sby/Run), baering, heading and error (difference to baering).
   
### Run
click once on button 2.

### During run
click on button 1 or 3 increases (resp. decrease) 1 deg of bearing.
long press on button 1 or 3 increases (resp. decreases) 20 deg of bearing
long press on button 2 resets bearing to current heading
long press on button 1 and 2 (starboard and select) to tack to starboard
long press on button 3 and 2 (port and select) to tack to port
double click on button 2 to stop motor and standby
	
### During standby
long press button 1 push the tiller
long press button 3 pull the tiller
long press on button 2 resets bearing to current heading
long press button 1 and 3 to enter setup

### On setup first level
click 1(-) for dimmed backlight
click 3(+) for full backlight
click 2(sel) to enter setup menu
### On setup menu
click 1(-) to go to previous menu item
click 3(+) to go to next menu item
click 2(sel) to select menu item
### On setup item
click 1(-) to decrease value if applicable
click 3(+) to increase value if applicable
click 2(sel) to validate entry/value

### Calibration
To calibrate the compass, chose the menu item, draw 8 with the device. The display shows the offsets.
Click button 2 when OK.

### Configuration saving
Go to the saving menu item and validate to write calibration and other parameters to EEPROM
