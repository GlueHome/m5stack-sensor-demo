#include <M5Stack.h>

#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#define LCD

MPU9250 IMU;

float cax, cay, caz;
float cgx, cgy, cgz;
float cmx, cmy, cmz;
float cyaw, cpitch, croll;

bool simpleDisplay = true;

void setup() {
  M5.begin(true, false);
  Wire.begin();

  Serial.println("Starting demo....");
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
  Serial.print(" I should be "); Serial.println(0x48, HEX);
  
  // Startup Sequence...
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE ,BLACK); // Set pixel color; 1 on the monochrome screen
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(0,0); M5.Lcd.print("Sensor Demo");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0,  48); M5.Lcd.print("Using MPU9250");
  M5.Lcd.setCursor(0,  68); M5.Lcd.print("9-DOF 16-bit");
  M5.Lcd.setCursor(0,  88); M5.Lcd.print("motion sensor");
  M5.Lcd.setCursor(20, 108); M5.Lcd.print("60 ug LSB");

  delay(3000);

  Serial.println("Performing Self Test on MPU9250....");
  IMU.MPU9250SelfTest(IMU.SelfTest);
  Serial.print("x-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[0],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[1],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[2],1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[3],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[4],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[5],1); Serial.println("% of factory value");

  Serial.println("Calibrating MPU9250....");
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  Serial.println("Initialising MPU9250....");
  IMU.initMPU9250();
  Serial.println("Initialising AK8963....");
  IMU.initAK8963(IMU.magCalibration);
  Serial.println("AK8963 initialized for active data mode....");
  Serial.print("X-Axis sensitivity adjustment value ");
  Serial.println(IMU.magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value ");
  Serial.println(IMU.magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value ");
  Serial.println(IMU.magCalibration[2], 2);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0); M5.Lcd.print("MPU9250 bias / AK8963");
  M5.Lcd.setCursor(0, 32); M5.Lcd.print(" x   y   z  ");

  M5.Lcd.setCursor(0,  64); M5.Lcd.print((int)(1000*IMU.accelBias[0]));
  M5.Lcd.setCursor(64, 64); M5.Lcd.print((int)(1000*IMU.accelBias[1]));
  M5.Lcd.setCursor(128, 64); M5.Lcd.print((int)(1000*IMU.accelBias[2]));
  M5.Lcd.setCursor(192, 64); M5.Lcd.print("mg");

  M5.Lcd.setCursor(0,  96); M5.Lcd.print(IMU.gyroBias[0], 1);
  M5.Lcd.setCursor(64, 96); M5.Lcd.print(IMU.gyroBias[1], 1);
  M5.Lcd.setCursor(128, 96); M5.Lcd.print(IMU.gyroBias[2], 1);
  M5.Lcd.setCursor(192, 96); M5.Lcd.print("o/s");

  M5.Lcd.setCursor(0,128); M5.Lcd.print(IMU.magCalibration[0], 2);
  M5.Lcd.setCursor(64,128); M5.Lcd.print(IMU.magCalibration[1], 2);
  M5.Lcd.setCursor(128,128); M5.Lcd.print(IMU.magCalibration[2], 2);
  delay(5000);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(GREEN ,BLACK);
  M5.Lcd.fillScreen(BLACK);
}

void loop() {
  if(M5.BtnB.isPressed()) {
    simpleDisplay = true;
  }
  if(M5.BtnC.isPressed()) {
    simpleDisplay = false;
  }

  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU.ax = (float)IMU.accelCount[0]*IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1]*IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2]*IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;

    IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
    IMU.getMres();
//    // User environmental x-axis correction in milliGauss, should be
//    // automatically calculated
//    IMU.magbias[0] = +470.;
//    // User environmental x-axis correction in milliGauss TODO axis??
//    IMU.magbias[1] = +120.;
//    // User environmental x-axis correction in milliGauss
//    IMU.magbias[2] = +125.;
    IMU.magbias[0] = +0.;
    IMU.magbias[1] = +0.;
    IMU.magbias[2] = +0.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU.mx = (float)IMU.magCount[0]*IMU.mRes*IMU.magCalibration[0] -
               IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1]*IMU.mRes*IMU.magCalibration[1] -
               IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2]*IMU.mRes*IMU.magCalibration[2] -
               IMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  IMU.updateTime();
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  //  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MadgwickQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx*DEG_TO_RAD,
                         IMU.gy*DEG_TO_RAD, IMU.gz*DEG_TO_RAD, IMU.my,
                         IMU.mx, IMU.mz, IMU.deltat);

  IMU.delt_t = millis() - IMU.count;
  if (IMU.delt_t > 500)
  {
    IMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                  *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    IMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                  *(getQ()+2)));
    IMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                  *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    IMU.yaw   *= RAD_TO_DEG;
//    IMU.yaw   -= 8.5;
    IMU.pitch *= RAD_TO_DEG;
    IMU.roll  *= RAD_TO_DEG;

    if(M5.BtnA.isPressed()) {
      cax = IMU.ax;
      cay = IMU.ay;
      caz = IMU.az;
      cgx = IMU.gx;
      cgx = IMU.gy;
      cgx = IMU.gz;
      cmx = IMU.mx;
      cmx = IMU.my;
      cmx = IMU.mz;
      cyaw = IMU.yaw;
      cpitch = IMU.pitch;
      croll = IMU.roll;
    }


    // Print acceleration values in milligs!
    Serial.print("X-acceleration: "); Serial.print(1000*IMU.ax);
    Serial.print(" mg ");
    Serial.print("Y-acceleration: "); Serial.print(1000*IMU.ay);
    Serial.print(" mg ");
    Serial.print("Z-acceleration: "); Serial.print(1000*IMU.az);
    Serial.println(" mg ");

    // Print gyro values in degree/sec
    Serial.print("X-gyro rate: "); Serial.print(IMU.gx, 3);
    Serial.print(" degrees/sec ");
    Serial.print("Y-gyro rate: "); Serial.print(IMU.gy, 3);
    Serial.print(" degrees/sec ");
    Serial.print("Z-gyro rate: "); Serial.print(IMU.gz, 3);
    Serial.println(" degrees/sec");

    // Print mag values in degree/sec
    Serial.print("X-mag field: "); Serial.print(IMU.mx);
    Serial.print(" mG ");
    Serial.print("Y-mag field: "); Serial.print(IMU.my);
    Serial.print(" mG ");
    Serial.print("Z-mag field: "); Serial.print(IMU.mz);
    Serial.println(" mG");

    Serial.print("quaternion Output: ");
    Serial.print("q0 = "); Serial.print(*getQ());
    Serial.print(" qx = "); Serial.print(*(getQ() + 1));
    Serial.print(" qy = "); Serial.print(*(getQ() + 2));
    Serial.print(" qz = "); Serial.println(*(getQ() + 3));

    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(IMU.yaw, 2);
    Serial.print(", ");
    Serial.print(IMU.pitch, 2);
    Serial.print(", ");
    Serial.println(IMU.roll, 2);

//    Serial.print("rate = ");
//    Serial.print((float)IMU.sumCount/IMU.sum, 2);
//    Serial.println(" Hz");

//    IMU.tempCount = IMU.readTempData();  // Read the adc values
//    // Temperature in degrees Centigrade
//    IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;
//    // Print temperature in degrees Centigrade
//    Serial.print("Temperature is ");  Serial.print(IMU.temperature, 1);
//    Serial.println(" degrees C");
//    Serial.println("");

    if (simpleDisplay) {
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(GREEN ,BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0, 0); M5.Lcd.print("  x");
      M5.Lcd.setCursor(80, 0); M5.Lcd.print("  y");
      M5.Lcd.setCursor(160, 0); M5.Lcd.print("  z");
  
      M5.Lcd.setCursor(0,  48); M5.Lcd.print((int)(1000*IMU.ax));
      M5.Lcd.setCursor(80, 48); M5.Lcd.print((int)(1000*IMU.ay));
      M5.Lcd.setCursor(160, 48); M5.Lcd.print((int)(1000*IMU.az));
      M5.Lcd.setCursor(240, 48); M5.Lcd.print("mg");
  
      M5.Lcd.setCursor(0,  80); M5.Lcd.print((int)(IMU.gx));
      M5.Lcd.setCursor(80, 80); M5.Lcd.print((int)(IMU.gy));
      M5.Lcd.setCursor(160, 80); M5.Lcd.print((int)(IMU.gz));
      M5.Lcd.setCursor(240, 80); M5.Lcd.print("o/s");
  
      M5.Lcd.setCursor(0,  112); M5.Lcd.print((int)(IMU.mx));
      M5.Lcd.setCursor(80, 112); M5.Lcd.print((int)(IMU.my));
      M5.Lcd.setCursor(160, 112); M5.Lcd.print((int)(IMU.mz));
      M5.Lcd.setCursor(240, 112); M5.Lcd.print("mG");
  
      M5.Lcd.setCursor(0, 160); M5.Lcd.print(" yaw");
      M5.Lcd.setCursor(80, 160); M5.Lcd.print(" pitch");
      M5.Lcd.setCursor(160, 160); M5.Lcd.print(" roll");
  
      M5.Lcd.setCursor(0,  192); M5.Lcd.print((int)(IMU.yaw));
      M5.Lcd.setCursor(80, 192); M5.Lcd.print((int)(IMU.pitch));
      M5.Lcd.setCursor(160, 192); M5.Lcd.print((int)(IMU.roll));
      
  //    M5.Lcd.setCursor(0,  128); M5.Lcd.print("Gyro T ");
  //    M5.Lcd.setCursor(50,  128); M5.Lcd.print(IMU.temperature, 1);
  //    M5.Lcd.print(" C");
  //
  //    M5.Lcd.setCursor(12, 144);
  //    M5.Lcd.print("rt: ");
  //    M5.Lcd.print((float) IMU.sumCount / IMU.sum, 2);
  //    M5.Lcd.print(" Hz");
    } else {
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(RED ,BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0, 0); M5.Lcd.print("  x");
      M5.Lcd.setCursor(80, 0); M5.Lcd.print("  y");
      M5.Lcd.setCursor(160, 0); M5.Lcd.print("  z");
  
      M5.Lcd.setCursor(0,  48); M5.Lcd.print((int)(1000*IMU.ax));
      M5.Lcd.setCursor(80, 48); M5.Lcd.print((int)(1000*IMU.ay));
      M5.Lcd.setCursor(160, 48); M5.Lcd.print((int)(1000*IMU.az));
      M5.Lcd.setCursor(240, 48); M5.Lcd.print("mg");
  
      M5.Lcd.setCursor(0,  80); M5.Lcd.print((int)(IMU.gx));
      M5.Lcd.setCursor(80, 80); M5.Lcd.print((int)(IMU.gy));
      M5.Lcd.setCursor(160, 80); M5.Lcd.print((int)(IMU.gz));
      M5.Lcd.setCursor(240, 80); M5.Lcd.print("o/s");
  
      M5.Lcd.setCursor(0,  112); M5.Lcd.print((int)(IMU.mx));
      M5.Lcd.setCursor(80, 112); M5.Lcd.print((int)(IMU.my));
      M5.Lcd.setCursor(160, 112); M5.Lcd.print((int)(IMU.mz));
      M5.Lcd.setCursor(240, 112); M5.Lcd.print("mG");
  
      M5.Lcd.setCursor(0, 160); M5.Lcd.print(" yaw");
      M5.Lcd.setCursor(80, 160); M5.Lcd.print(" pitch");
      M5.Lcd.setCursor(160, 160); M5.Lcd.print(" roll");
  
      M5.Lcd.setCursor(0,  192); M5.Lcd.print((int)(IMU.yaw));
      M5.Lcd.setCursor(80, 192); M5.Lcd.print((int)(IMU.pitch));
      M5.Lcd.setCursor(160, 192); M5.Lcd.print((int)(IMU.roll));
    }

    IMU.count = millis();
    IMU.sumCount = 0;
    IMU.sum = 0;
  }
  M5.update();
}

