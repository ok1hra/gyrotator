
// compile for M5STACK core esp32

/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016
 */
const int REV = 20190810;
/* Changelog
20190810 - HMC5883L compass support
*/
int AntRadiationWidth=30;

#include <M5Stack.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"
// HMC5883L
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
bool EnableAZ=false;

MPU9250 IMU;
// Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances
int Target=0;
int TargetOld=0;
int AzimuthOld=0;
int Azimuth=0;
bool LcdNeedRefresh=true;
long TargetTimeout[2]={0,2000};
long GetAzTimeout[2]={0,100};
long GetBattTimeout[2]={0,5000};
bool HidenTarget=true;
bool FAstMove=true;
bool LowBattery=false;
long LongPressButtTimeout[2]={0,1000};
bool EnableMap=false;


void setup(){
  M5.begin();
  Wire.begin();
  // Start device display with ID of sensor
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE ,BLACK); // Set pixel color; 1 on the monochrome screen
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(80,100); M5.Lcd.print("GYROTATOR");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(125,133);
  M5.Lcd.print("rev.");
  M5.Lcd.print(REV);
  // delay(3000);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    // Start by performing self test and reporting values
    IMU.MPU9250SelfTest(IMU.SelfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

    IMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    // Get magnetometer calibration from AK8963 ROM
    IMU.initAK8963(IMU.magCalibration);
    // Initialize device for active mode read of magnetometer

  } // if (c == 0x71)
  // else
  // {
  //   Serial.print("Could not connect to MPU9250: 0x");
  //   Serial.println(c, HEX);
  //   while(1) ; // Loop forever if communication doesn't happen
  // }
  M5.Lcd.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  M5.Lcd.fillScreen(BLACK);   // clears the screen and buffer
  DirectionalRosette(120, 120, 110);

  // HMC5883L
  Serial.begin(115200);
  if(!mag.begin()){
      Serial.println("HMC5883L not found");
      // while(1);
  }

}
//------------------------------------------------------------------------------

void loop(){
  Watchdog();
  LcdShow();
  Buttons();
}
//------------------------------------------------------------------------------

void Buttons(){
  if(M5.BtnA.wasPressed()){
    LongPressButtTimeout[0]=millis();
  }
  if(M5.BtnA.wasReleased()){
    if(millis()-LongPressButtTimeout[0]<LongPressButtTimeout[1]){
      FAstMove=!FAstMove;
    }else{
      EnableMap=!EnableMap;
      if(EnableMap==false){
        M5.Lcd.fillScreen(BLACK);
        DirectionalRosette(120, 120, 110);
      }
    }
    LcdNeedRefresh=true;
  }

  if(M5.BtnB.wasPressed()) {
    if(EnableMap==false){
      Arrow(Azimuth,120,120,100, 0x000000);
    }
    Azimuth=Target;
    if(EnableMap==false){
      Arrow(Azimuth,120,120,100, GREEN);
    }
    LcdNeedRefresh=true;
  }

  if(M5.BtnC.wasPressed()){
    LongPressButtTimeout[0]=millis();
  }
  if(M5.BtnC.wasReleased()){
    if(millis()-LongPressButtTimeout[0]<LongPressButtTimeout[1]){
      EnableAZ=!EnableAZ;
      if(EnableAZ==true){
        if(!mag.begin()){
            Serial.println("HMC5883L not found");
            EnableAZ=false;
        }
      }
    }else{
      M5.Power.powerOFF();
    }
    LcdNeedRefresh=true;
  }


  M5.update();
}
//------------------------------------------------------------------------------

void Watchdog(){
  if(millis()-TargetTimeout[0]>TargetTimeout[1] && HidenTarget==false){
    TargetTimeout[0]=millis();
    // Target=Azimuth;
    LcdNeedRefresh=true;
    HidenTarget=true;
    if(LowBattery==true){
      M5.Lcd.fillScreen(RED);
      M5.Lcd.setTextColor(BLACK, RED); // Set pixel color; 1 on the monochrome screen
      M5.Lcd.setTextSize(3);
      M5.Lcd.setCursor(60,90); M5.Lcd.print("BATTERY LOW");
      M5.Lcd.setCursor(60,130); M5.Lcd.print("SHUTDOWN...");
      delay(5000);
      M5.Power.powerOFF();
      M5.update();
    }
  }
  if(millis()-GetAzTimeout[0]>GetAzTimeout[1]){
    HMC5883L();
    GetGyroData();
    if(FAstMove==true){
      Target=Target+(int)(IMU.gz)+(int)(IMU.gy);
    }else{
      Target=Target+((int)(IMU.gz)+(int)(IMU.gy))/8;
    }
    if(Target!=TargetOld){
      if(Target>360){
        // Target=Target-360;
        Target=map(Target, 361, 720, 1, 360);
      }
      if(Target<0){
        // Target=360-Target;
        Target=map(Target, -360, -1, 0, 359);
      }
      TargetTimeout[0]=millis();
      LcdNeedRefresh=true;
      HidenTarget=false;
    }
    GetAzTimeout[0]=millis();
  }
  if(millis()-GetBattTimeout[0]>GetBattTimeout[1]){
    GetBattTimeout[0]=millis();
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(245, 170);
    getBatteryLevel();
    // LcdNeedRefresh=true;
  }
}
//------------------------------------------------------------------------------
void HMC5883L(){
  if(EnableAZ==true){
    // vytvoření a následné načtení balíku dat ze senzoru
   sensors_event_t magData;
   mag.getEvent(&magData);
   // vytištění informací o velikostech magnetických sil v osách x,y,z
   // hodnoty jsou v jednotce uT (mikroTesla)
   Serial.print("X: "); Serial.print(magData.magnetic.x); Serial.print(" ");
   Serial.print("Y: "); Serial.print(magData.magnetic.y); Serial.print(" ");
   Serial.print("Z: "); Serial.print(magData.magnetic.z); Serial.print(" ");
   Serial.println("uT");
   // výpočet velikosti úhlu natočení ze sil v osách x a y
   float uhelNatoceni = atan2(magData.magnetic.y, magData.magnetic.x);
   // pro přesnější získání úhlu natočení je nutné navštívit tento odkaz:
   // http://www.magnetic-declination.com/
   // na zmíněné stránce zadáte svoji lokaci a z ní zjistíte údaj
   // "magnetic declinaction" - magnetický sklon, ten je ještě nutné převést z úhlů na radiány
   // tedy např. v Brně je tento úhel cca 4 stupně = 0.07 radiánů
   // pokud se vám nepodaří najít mag. sklon, stačí 2 řádky pod komentářem vymazat
   float magDeclinRad = 0.07;
   uhelNatoceni += magDeclinRad;

   // korekce výpočtu úhlu natočení
   if(uhelNatoceni < 0)
     uhelNatoceni += 2*PI;
   if(uhelNatoceni > 2*PI)
     uhelNatoceni -= 2*PI;
   // konečný přepočet úhlu natočení z radiánů na stupně
   float uhelNatoceniSt = uhelNatoceni * 180/M_PI;
   Azimuth=uhelNatoceniSt;
   if(Azimuth!=AzimuthOld){
      LcdNeedRefresh=true;
   }
   // vytištění údajů o úhlu natočení, 0 stupňů znamená sever
   Serial.print("Uhel natoceni: "); Serial.print(uhelNatoceniSt);
   Serial.println(" stupnu");
   Serial.println();
 }
}

//------------------------------------------------------------------------------
void BattIcon(int x, int y, int COLOR, int LEVEL){
  for (int i=0; i<4; i++) {
    if(i>LEVEL-1){
      M5.Lcd.fillRect(x, y-i*8, 15, 5, BLACK);
    }else{
      M5.Lcd.fillRect(x, y-i*8, 15, 5, COLOR);
    }
  }
  M5.Lcd.drawRect(x-4, y-33+5, 15+8, 37, COLOR);
  M5.Lcd.drawRect(x-4-1, y-33+5-1, 15+8+2, 37+2, COLOR);
  M5.Lcd.fillRect(x+3, y-33, 9, 5, COLOR);
}
//------------------------------------------------------------------------------

void LcdShow(){
  // /usr/bin/xplanet -window -longitude 13.7917 -latitude 50.3542 -geometry 240x240 -projection azimuthal -num_times 1 -output map.bmp
  // 320x240px
  //  IMU.delt_t = millis() - IMU.count;
  //  if (IMU.delt_t > 100){
  if(LcdNeedRefresh==true){
    // M5.Lcd.setBrightness(200);
    if(EnableMap==true){
      M5.Lcd.drawJpgFile(SD, "/map.jpg");
      DirectionalRosette(120, 120, 110);
      int deg=Target+AntRadiationWidth/2;
      int deg2=Target-AntRadiationWidth/2;
      if(HidenTarget==false){
        // M5.Lcd.drawTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), WHITE);
        deg=Target+AntRadiationWidth/2-1;
        deg2=Target+AntRadiationWidth/2+1;
        M5.Lcd.fillTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), LIGHTGREY);
        deg=Target-AntRadiationWidth/2-1;
        deg2=Target-AntRadiationWidth/2+1;
        M5.Lcd.fillTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), LIGHTGREY);
      }
      TargetOld=Target;
      deg=Azimuth+AntRadiationWidth/2-1;
      deg2=Azimuth+AntRadiationWidth/2+1;
      M5.Lcd.fillTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), RED);
      deg=Azimuth-AntRadiationWidth/2-1;
      deg2=Azimuth-AntRadiationWidth/2+1;
      M5.Lcd.fillTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), RED);
    }else{
      //  M5.Lcd.fillScreen(BLACK);
      // M5.Lcd.setCursor(0, 32); M5.Lcd.print(" x   y   z  ");

      // M5.Lcd.setCursor(0,  48); M5.Lcd.print((int)(1000*IMU.ax));
      // M5.Lcd.setCursor(32, 48); M5.Lcd.print((int)(1000*IMU.ay));
      // M5.Lcd.setCursor(64, 48); M5.Lcd.print((int)(1000*IMU.az));
      // M5.Lcd.setCursor(96, 48); M5.Lcd.print("mg");

      // M5.Lcd.setCursor(0,  64); M5.Lcd.print((int)(IMU.gx));
      // M5.Lcd.setCursor(32, 64); M5.Lcd.print((int)(IMU.gy));
      Arrow(TargetOld,120,120,100, 0x000000);
      if(HidenTarget!=true){
        Arrow(Target,120,120,100, LIGHTGREY);
      }
      TargetOld=Target;
      if(Azimuth!=AzimuthOld){
        Arrow(AzimuthOld,120,120,100, BLACK);
        AzimuthOld=Azimuth;
      }
      Arrow(Azimuth,120,120,100, GREEN);
      //  }
      // M5.Lcd.setCursor(96, 64); M5.Lcd.print("o/s");

      // M5.Lcd.setCursor(0,  96); M5.Lcd.print((int)(IMU.mx));
      // M5.Lcd.setCursor(32, 96); M5.Lcd.print((int)(IMU.my));
      // M5.Lcd.setCursor(64, 96); M5.Lcd.print((int)(IMU.mz));
      // M5.Lcd.setCursor(96, 96); M5.Lcd.print("mG");

      // M5.Lcd.setCursor(0,  128); M5.Lcd.print("Gyro T ");
      // M5.Lcd.setCursor(50,  128); M5.Lcd.print(IMU.temperature, 1);
      // M5.Lcd.print(" C");

      // digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    }

    M5.Lcd.setTextSize(5);
    M5.Lcd.setTextColor(GREEN ,BLACK);
    // M5.Lcd.setCursor(240, 10);
    M5.Lcd.setCursor(210, 10);
    if(Azimuth<10){
      M5.Lcd.print(" ");
    }
    if(Azimuth<100){
      M5.Lcd.print(" ");
    }
    M5.Lcd.print( Azimuth );
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(300, 5);
    M5.Lcd.print("o");
    M5.Lcd.setTextSize(3);
    if(HidenTarget!=true){
      M5.Lcd.setTextColor(WHITE ,BLACK);
    }else{
      M5.Lcd.setTextColor(BLACK ,BLACK);
    }
    M5.Lcd.setCursor(240, 210);
    if(Target<10){
      M5.Lcd.print(" ");
    }
    if(Target<100){
      M5.Lcd.print(" ");
    }
    M5.Lcd.print( Target );
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(297, 205);
    M5.Lcd.print("o");
    M5.Lcd.setTextSize(3);

    M5.Lcd.setCursor(10, 210);
    M5.Lcd.setTextColor(LIGHTGREY ,BLACK);
    if(FAstMove==true){
      M5.Lcd.print( "F" );
    }else{
      M5.Lcd.print( "S" );
    }
    // if(EnableMap==true){
    //   M5.Lcd.print( "M" );
    // }else{
    //   M5.Lcd.print( " " );
    // }
     LcdNeedRefresh=false;
   }

}
//------------------------------------------------------------------------------

float Xcoordinate(int dir, int Center, int r){
  float x = Center + sin(dir/RAD_TO_DEG) * r;
  return x;
}
float Ycoordinate(int dir, int Center, int r){
  float y = Center - cos(dir/RAD_TO_DEG) * r;
  return y;
}
//------------------------------------------------------------------------------

void Arrow(int deg, int X, int Y, int r, int Color){
  int deg2 = deg+130;
  int deg3 = deg+230;
  M5.Lcd.fillTriangle(Xcoordinate(deg,X,r), Ycoordinate(deg,Y,r), Xcoordinate(deg2,X,r/2), Ycoordinate(deg2,Y,r/2), Xcoordinate(deg+180,X,0), Ycoordinate(deg+180,Y,0), Color);
  M5.Lcd.fillTriangle(Xcoordinate(deg,X,r), Ycoordinate(deg,Y,r), Xcoordinate(deg3,X,r/2), Ycoordinate(deg3,Y,r/2), Xcoordinate(deg+180,X,0), Ycoordinate(deg+180,Y,0), Color);
  M5.Lcd.fillTriangle(Xcoordinate(deg+180,X,r), Ycoordinate(deg+180,Y,r), Xcoordinate(deg3,X,r/10), Ycoordinate(deg3,Y,r/10), Xcoordinate(deg+180,X,0), Ycoordinate(deg+180,Y,0), Color);
  M5.Lcd.fillTriangle(Xcoordinate(deg+180,X,r), Ycoordinate(deg+180,Y,r), Xcoordinate(deg2,X,r/10), Ycoordinate(deg2,Y,r/10), Xcoordinate(deg+180,X,0), Ycoordinate(deg+180,Y,0), Color);
}
//------------------------------------------------------------------------------

void DirectionalRosette(int X, int Y, int R){
  int dot1;
  int dot2;
  if(R>70){
    dot1=2;
    dot2=5;
  }else{
    dot1=1;
    dot2=3;
  }
  M5.Lcd.setTextColor(WHITE ,BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(Xcoordinate(0,X-5,R), Ycoordinate(0,Y-5,R)); M5.Lcd.print("N");
  M5.Lcd.setCursor(Xcoordinate(90,X-5,R), Ycoordinate(90,Y-6,R)); M5.Lcd.print("E");
  M5.Lcd.setCursor(Xcoordinate(180,X-5,R), Ycoordinate(180,Y-5,R)); M5.Lcd.print("S");
  M5.Lcd.setCursor(Xcoordinate(270,X-5,R), Ycoordinate(270,Y-6,R)); M5.Lcd.print("W");
  for (int j=0; j<36; j++) {
    if(j % 9 == 0){
      // u8g2.drawDisc(Xcoordinate(j*10,X,R), Ycoordinate(j*10,Y,R), dot2, U8G2_DRAW_ALL);
    }else{
      M5.Lcd.fillCircle(Xcoordinate(j*10,X,R), Ycoordinate(j*10,Y,R), 2, LIGHTGREY);;
    }
  }
}
//------------------------------------------------------------------------------
int8_t getBatteryLevel()
{
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  if (Wire.endTransmission(false) == 0
   && Wire.requestFrom(0x75, 1)) {
    switch (Wire.read() & 0xF0) {
    case 0xE0: BattIcon(290, 130, RED, 1);
    LowBattery=true;
    break;
    case 0xC0: BattIcon(290, 130, DARKGREY, 2);
    LowBattery=false;
    break;
    case 0x80: BattIcon(290, 130, BLACK, 3);
    LowBattery=false;
    break;
    case 0x00: BattIcon(290, 130, BLACK, 4);
    LowBattery=false;
    break;
    default: return 0;
    break;
    }
  }
  return -1;
}
//------------------------------------------------------------------------------

void GetGyroData(){
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
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
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    IMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    IMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    IMU.magbias[2] = +125.;

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

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx*DEG_TO_RAD,
                         IMU.gy*DEG_TO_RAD, IMU.gz*DEG_TO_RAD, IMU.my,
                         IMU.mx, IMU.mz, IMU.deltat);

}