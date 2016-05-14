/*
   YAAAP Yet Another Arduino AutoPilot
   compass.ino
   2016 Philippe Leclercq

   9dof IMU breakout : MPU9250

   Uses RTIMUlib library https://github.com/richards-tech/RTIMULib-Arduino
	4 lines modified in RTIMUlib/RTIMULibDefs.h :
	//#define MPU9150_68                      // MPU9150 at address 0x68
	#define MPU9250_68                      // MPU9250 at address 0x68
	//#define RTIMU_XNORTH_YEAST              0                   // this is the default identity matrix
	#define RTIMU_XNORTH_YWEST              4


 */


//#define SERIALDEBUG 0
#define SERIALDEBUG 3 // Value to send yaw/pitch/roll to Processing sketch on host
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"
#include <EEPROM.h>

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object
unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;
#define DISPLAY_INTERVAL  500                         // interval between pose displays

/***********************************************************
   Compass management
 ***********************************************************
*/
float compassHeading() {
  float yaw;
  //  int loopCount = 1;
#if SERIALDEBUG == 1
  unsigned long now = millis();
  unsigned long delta;
#endif
  int loopCount = 1;

  while (imu->IMURead()) {                                // get the latest data if ready yet
    // this flushes remaining data in case we are falling behind
    if (++loopCount >= 10)           continue;
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
#if SERIALDEBUG == 1

    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      Serial.print("Sample rate: "); Serial.print(sampleCount);
      if (imu->IMUGyroBiasValid())
        Serial.println(", gyro bias valid");
      else
        Serial.println(", calculating gyro bias");

      sampleCount = 0;
      lastRate = now;
    }

    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      RTMath::display(" Gy", (RTVector3&)imu->getGyro());                // gyro data
      RTMath::display(" Ac", (RTVector3&)imu->getAccel());              // accel data
      RTMath::display(" Ma", (RTVector3&)imu->getCompass());              // compass data
      RTMath::displayRollPitchYaw(" Pos", (RTVector3&)fusion.getFusionPose()); // fused output
      Serial.println();
    }
#endif
  }
#if SERIALDEBUG == 3 // Sends to Processing view
  if (imu->IMUGyroBiasValid()) {
    Serial.print(fusion.getFusionPose().z()); // yaw
    Serial.print(","); // print comma so values can be parsed
    Serial.print(fusion.getFusionPose().y()); // pitch
    Serial.print(","); // print comma so values can be parsed
    Serial.println(fusion.getFusionPose().x()); // roll
  }
#endif
  yaw = map360(fusion.getFusionPose().z() * RTMATH_RAD_TO_DEGREE);
  //  yaw = fusion.getFusionPose().z() * RTMATH_RAD_TO_DEGREE;
#if SERIALDEBUG==2
  Serial.print("yaw=");
  Serial.println(yaw);
#endif
  return yaw;
}

void compassInit() {
  RTVector3 gyroBias;
  calLibRead(0, &calData);                           // pick up existing mag data (and other params) if there
#if SERIALDEBUG==1
  for (int i = 0; i < 3; i++) {
    Serial.print(calData.magMin[i]); Serial.print(" "); Serial.println(calData.magMax[i]);
  }
  Serial.print(calData.Kp); Serial.print(" "); Serial.print(calData.Kd); Serial.print(" "); Serial.println(calData.Kdd);
#endif
  imu = RTIMU::createIMU(&settings);                        // create the imu object
#if SERIALDEBUG==1
  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
#endif
  int errcode = imu->IMUInit();
#if SERIALDEBUG==1
  if (errcode < 0) {
    Serial.print("Init IMU failed: "); Serial.println(errcode);
  }
#endif

  if (calData.gyroBiasValid == 0x1) { // load gyroBias data if found on EEPROM
    gyroBias.setX(calData.gyroBias[0]);
    gyroBias.setY(calData.gyroBias[1]);
    gyroBias.setZ(calData.gyroBias[2]);
    imu->setGyroBias(gyroBias);
  }

  bool cal = imu->getCalibrationValid();
#if SERIALDEBUG==1
  if (cal)
    Serial.println("Using calibration");
  else
    Serial.println("No valid calibration data");
#endif

#if SERIALDEBUG==2
  lastDisplay = lastRate = millis();
  sampleCount = 0;
#endif

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.

  fusion.setSlerpPower(0.001);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor

  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);

  int loopCount = 0;
  do {
    //    if (++loopCount > 10000)           continue;
    compassHeading();
  } while ( ! imu->IMUGyroBiasValid() || ++loopCount<10); // at least a few loop for a good initial baering
}

void compassCalibration() {
#if SERIALDEBUG==3 // Only 32k or memory in pro mini's ATMega328 chip. No calibration if debug...
}
#else
  int tmpOffX = 0, tmpOffY = 0, tmpOffZ = 0;
  RTVector3 mag;


  // calLibRead(0, &calData);                           // pick up existing mag data if there

  calData.magValid = false;
  for (int i = 0; i < 3; i++) {
    calData.magMin[i] = 10000000;                    // init mag cal data
    calData.magMax[i] = -10000000;
  }

  imu->setCalibrationMode(true);                     // make sure we get raw data

  keyPressed = ' ';
  while (keyPressed == ' ') { // Waiting any key

    if (imu->IMURead()) {                                 // get the latest data
      mag = imu->getCompass();
      for (int i = 0; i < 3; i++) {
        if (mag.data(i) < calData.magMin[i]) {
          calData.magMin[i] = mag.data(i);
        }
        if (mag.data(i) > calData.magMax[i]) {
          calData.magMax[i] = mag.data(i);
        }
      }
    }

    // Calculate offsets
    tmpOffX = (calData.magMax[0] + calData.magMin[0]) / 2;
    tmpOffY = (calData.magMax[1] + calData.magMin[1]) / 2;
    tmpOffZ = (calData.magMax[2] + calData.magMin[2]) / 2;

    lcd.setCursor(0, 1);
    lcd.print(tmpOffX);
    lcd.print(" ");
    lcd.print(tmpOffY);
    lcd.print(" ");
    lcd.print(tmpOffZ);
    lcd.print(" ");
    delay(20);
    buttonsTick();
  }
  // Save data if select key pressed, else exit
  if (keyPressed == 's') {
    calData.magValid = true;
    calLibWrite(0, &calData);
    compassInit();
  }
  else {
    lcd.setCursor(0, 1);
    lcd.print(F("*** Aborted ***"));
  }
  delay(1000);
}
#endif
