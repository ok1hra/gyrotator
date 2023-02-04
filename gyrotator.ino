
// compile for M5STACK Fire esp32

/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016
 */
const int REV = 20230204;
/* Changelog
20190810 - HMC5883L compass support

TODO
- integrovat WX obrazovku na prepinani pravym butt, pokud je wx, disablovat start Rotate
- stahnout po mqtt jmeno rotatoru a zobrazit na lcd

*/
// #define MPU6886   // gyroscope
// #define MPU9250   // gyroscope old
// #define HMC5883L   // compass old

String YOUR_CALL = "BD:2F";
int AntRadiationWidth=30;

// wifi/mqtt
#include <PubSubClient.h>
#include <WiFi.h>
WiFiClient espClient;
PubSubClient client(espClient);
const char* ssid        = "SSID";
const char* password    = "pass";
const char* mqtt_server = "54.38.157.134";
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
void setupWifi();
void callback(char* topic, byte* payload, unsigned int length);
void reConnect();

#include <M5Stack.h>
// #if defined(MPU6886)
  float accX = 0.0F;  // Define variables for storing inertial sensor data
  float accY = 0.0F;
  float accZ = 0.0F;
  float gyroX = 0.0F;
  float gyroY = 0.0F;
  float gyroZ = 0.0F;
  float pitch = 0.0F;
  float roll  = 0.0F;
  float yaw   = 0.0F;
  float temp = 0.0F;
// #endif
#if defined(MPU9250)
  #include "utility/MPU9250.h"
  #include "utility/quaternionFilters.h"
  MPU9250 IMU;
#endif
#if defined(HMC5883L)
  // HMC5883L
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_HMC5883_U.h>
  Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#endif
bool EnableAZ=false;

// Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances
int Target=0;
int TargetOld=0;
int AzimuthOld=0;
int Azimuth=0;
int StartAzimuth = 0;
int MaxRotateDegree = 0;
bool LcdNeedRefresh=true;
long TargetTimeout[2]={0,4000};
bool HidenTarget=true;
bool FAstMove=true;
bool LowBattery=false;
long LongPressButtTimeout[2]={0,1000};
bool EnableMap=false;
int Status = 4;
bool OnlineStatus=false;
bool CheckOnlineInProgress=false;
long CheckOnlineTimer=0;
bool GyroCalibrate = false;

void setup(){
  M5.begin();
  // #if defined(MPU6886)
    M5.Power.begin();  // Init Power module.
    M5.IMU.Init();  // Init IMU sensor.
    M5.Lcd.setBrightness(200);
  // #endif

  // wifi/mqtt
  setupWifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  #if defined(HMC5883L)
    Wire.begin();
  #endif
  // Start device display with ID of sensor
  // M5.Lcd.fillScreen(BLACK);
  // M5.Lcd.setTextColor(WHITE ,BLACK); // Set pixel color; 1 on the monochrome screen
  // M5.Lcd.setTextSize(3);
  // M5.Lcd.setCursor(80,100); M5.Lcd.print("GYROTATOR");
  // M5.Lcd.setTextSize(1);
  // M5.Lcd.setCursor(125,133);
  // M5.Lcd.print("rev.");
  // M5.Lcd.print(REV);
  // delay(3000);

  #if defined(MPU9250)
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
  #endif

  // M5.Lcd.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  // M5.Lcd.fillScreen(BLACK);   // clears the screen and buffer
  // DirectionalRosette(120, 120, 110);

  #if defined(HMC5883L)
    // HMC5883L
    Serial.begin(115200);
    if(!mag.begin()){
        Serial.println("HMC5883L not found");
        // while(1);
    }
  #endif

}
//------------------------------------------------------------------------------

void loop(){
  Watchdog();
  LcdShow(StartAzimuth);
  Buttons();

  // wifi/mqtt
    if (!client.connected()) {
        reConnect();
    }
    client.loop();  // This function is called periodically to allow clients to
                    // process incoming messages and maintain connections to the
                    // server.

    // unsigned long now =
    //     millis();  // Obtain the host startup duration.  获取主机开机时长
    // if (now - lastMsg > 2000) {
    //     lastMsg = now;
    //     ++value;
    //     snprintf(msg, MSG_BUFFER_SIZE, "hello world #%ld",
    //              value);  // Format to the specified string and store it in MSG.
    //                       // 格式化成指定字符串并存入msg中
    //     M5.Lcd.print("Publish message: ");
    //     M5.Lcd.println(msg);
    //     client.publish("M5Stack", msg);  // Publishes a message to the specified
    //                                      // topic.  发送一条消息至指定话题
    //     if (value % 12 == 0) {
    //         M5.Lcd.clear();
    //         M5.Lcd.setCursor(0, 0);
    //     }
    // }
}
//------------------------------------------------------------------------------
void setupWifi() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE ,BLACK); // Set pixel color; 1 on the monochrome screen
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(80,100); M5.Lcd.print("GYROTATOR");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(125,133);
  M5.Lcd.print("rev.");
  M5.Lcd.print(REV);
  // M5.Lcd.setCursor(125,170);
  // M5.Lcd.print("Connecting to ");
  M5.Lcd.setCursor(0,200);
  M5.Lcd.printf("Connecting to %s", ssid);
  delay(10);
  Serial.print("Connecting to %s");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);  // Set the mode to WiFi station mode.
  WiFi.begin(ssid, password);  // Start Wifi connection.

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      M5.Lcd.print(".");
      Serial.print(".");
  }
  // M5.Lcd.setCursor(125,210);
  M5.Lcd.print("Success");
  Serial.println("\nSuccess\n");
  String topic = String(YOUR_CALL) + "/ROT/";
  M5.Lcd.setCursor(0,220);
  M5.Lcd.print("MQTT ");
  M5.Lcd.print(topic);
  delay(2000);
}

void callback(char* topic, byte* payload, unsigned int length) {
    // M5.Lcd.print("Message arrived [");
    // M5.Lcd.print(topic);
    // M5.Lcd.print("] ");
    // for (int i = 0; i < length; i++) {
    //     M5.Lcd.print((char)payload[i]);
    // }
    // M5.Lcd.println();
    String CheckTopicBase;
    CheckTopicBase.reserve(100);
    byte* p = (byte*)malloc(length);
    memcpy(p,payload,length);
    // static bool HeardBeatStatus;
    // Serial.print("RX MQTT ");
    // Serial.println(String(topic));

      // Azimuth
      CheckTopicBase = String(YOUR_CALL) + "/ROT/Azimuth";
      if ( CheckTopicBase.equals( String(topic) ) ){
        Azimuth = 0;
        unsigned long exp = 1;
        for (int i = length-1; i >=0 ; i--) {
          // Numbers only
          if(p[i]>=48 && p[i]<=58){
            Azimuth = Azimuth + (p[i]-48)*exp;
            exp = exp*10;
          }
        }
        Serial.print("Azimuth =  ");
        Serial.println(Azimuth);
        LcdNeedRefresh=true;
        OnlineStatus=true;
        CheckOnlineInProgress=false;
      }

      CheckTopicBase = String(YOUR_CALL) + "/ROT/StartAzimuth";
      if ( CheckTopicBase.equals( String(topic) ) ){
        StartAzimuth = 0;
        unsigned long exp = 1;
        for (int i = length-1; i >=0 ; i--) {
          // Numbers only
          if(p[i]>=48 && p[i]<=58){
            StartAzimuth = StartAzimuth + (p[i]-48)*exp;
            exp = exp*10;
          }
        }
        Serial.print("StartAzimuth =  ");
        Serial.println(StartAzimuth);
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setTextColor(WHITE ,BLACK); // Set pixel color; 1 on the monochrome screen
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(40,100); M5.Lcd.print("CALIBRATE...");
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(40,130); M5.Lcd.print("do not move");
        // DirectionalRosette(120, 120, 110);
        // LcdNeedRefresh=true;
      }

      CheckTopicBase = String(YOUR_CALL) + "/ROT/MaxRotateDegree";
      if ( CheckTopicBase.equals( String(topic) ) ){
        MaxRotateDegree = 0;
        unsigned long exp = 1;
        for (int i = length-1; i >=0 ; i--) {
          // Numbers only
          if(p[i]>=48 && p[i]<=58){
            MaxRotateDegree = MaxRotateDegree + (p[i]-48)*exp;
            exp = exp*10;
          }
        }
        Serial.print("MaxRotateDegree =  ");
        Serial.println(MaxRotateDegree);
        // M5.Lcd.fillScreen(BLACK);
        // M5.Lcd.setTextColor(WHITE ,BLACK); // Set pixel color; 1 on the monochrome screen
        // M5.Lcd.setTextSize(3);
        // M5.Lcd.setCursor(80,100); M5.Lcd.print("CALIBRATE\ndo not move");
        // DirectionalRosette(120, 120, 110);
        // LcdNeedRefresh=true;
      }

      // Status
      // 1 PwmDwnCCW|2 CCW|3 PwmUpCCW|4 off|5 PwmUpCW|6 CW|7 PwmDwnCW
      CheckTopicBase = String(YOUR_CALL) + "/ROT/Status";
      if ( CheckTopicBase.equals( String(topic) ) ){
        Status = 0;
        unsigned long exp = 1;
        for (int i = length-1; i >=0 ; i--) {
          // Numbers only
          if(p[i]>=48 && p[i]<=58){
            Status = Status + (p[i]-48)*exp;
            exp = exp*10;
          }
        }
        Serial.print("Status = ");
        Serial.println(Status);
        if(Status==4){
          HidenTarget=true;
        }
        OnlineStatus=true;
        CheckOnlineInProgress=false;
        LcdNeedRefresh=true;
      }
}

void reConnect() {
    while (!client.connected()) {
        // M5.Lcd.print("Attempting MQTT connection...");
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID.  创建一个随机的客户端ID
        String clientId = "M5Stack-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect.  尝试重新连接
        if (client.connect(clientId.c_str())) {
            // M5.Lcd.printf("\nSuccess\n");
            Serial.println("Success");
            // Once connected, publish an announcement to the topic.
            // 一旦连接，发送一条消息至指定话题
            client.publish("BD:2F/ROT/M5StackClient", "connected");
            // ... and resubscribe.  重新订阅话题
            client.subscribe("BD:2F/ROT/Azimuth");
            client.subscribe("BD:2F/ROT/StartAzimuth");
            client.subscribe("BD:2F/ROT/MaxRotateDegree");
            // client.subscribe("BD:2F/ROT/Target");
            client.subscribe("BD:2F/ROT/Status");

            MqttPubString("get", "0", 0);
            CheckOnlineInProgress=true;
            CheckOnlineTimer=millis();
            M5.Lcd.fillScreen(BLACK);   // clears the screen and buffer
            DirectionalRosette(120, 120, 110);
            LcdNeedRefresh=true;
        } else {
            // M5.Lcd.print("failed, rc=");
            // M5.Lcd.print(client.state());
            // M5.Lcd.println("try again in 5 seconds");
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println("try again in 5 seconds");
            delay(5000);
        }
    }
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
    if(EnableMap==false && HidenTarget==false){
      Arrow(Azimuth,120,120,100, 0x000000);
      // Arrow(Azimuth,120,120,100, GREEN);
      // client.publish("BD:2F/ROT/Target", char(Target) );
      MqttPubString("Target", String(Target), 0);
      CheckOnlineInProgress=true;
      CheckOnlineTimer=millis();
      // Azimuth=Target;
      HidenTarget=true;
      LcdNeedRefresh=true;
    }
    if(Status!=4){
      MqttPubString("stop", "0", 0);
    }
  }

  if(M5.BtnC.wasPressed()){
    LongPressButtTimeout[0]=millis();
  }
  if(M5.BtnC.wasReleased()){
    if(millis()-LongPressButtTimeout[0]<LongPressButtTimeout[1]){
      // #if defined(HMC5883L)
        EnableAZ=!EnableAZ;
        // if(EnableAZ==true){
        //   if(!mag.begin()){
        //       Serial.println("HMC5883L not found");
        //       EnableAZ=false;
        //   }
        // }
      // #endif
    }else{
      M5.Power.powerOFF();
    }
    LcdNeedRefresh=true;
  }
  M5.update();
}
//-----------------------------------------------------------------------------------
void MqttPubString(String TOPIC, String DATA, bool RETAIN){
  char charbuf[50];
  const int MqttBuferSize = 1000; // 1000
  static char mqttTX[MqttBuferSize];
  static char mqttPath[MqttBuferSize];

   // // memcpy( charbuf, mac, 6);
   // ETH.macAddress().toCharArray(charbuf, 10);
   // charbuf[6] = 0;
  // Interrupts(false);
  // if(EnableEthernet==1 && MQTT_ENABLE==1 && EthLinkStatus==1 && mqttClient.connected()==true){
  // if(mqttClient.connected()==true){
  //   if (mqttClient.connect(MACchar)) {
      String topic = String(YOUR_CALL) + "/ROT/"+TOPIC;
      topic.toCharArray( mqttPath, 50 );
      DATA.toCharArray( mqttTX, 50 );
      // mqttClient.publish(mqttPath, mqttTX, RETAIN);
      client.publish(mqttPath, mqttTX);

  //   }
  // }
  // Interrupts(true);
}
//------------------------------------------------------------------------------

void Watchdog(){

  if(millis()-CheckOnlineTimer > 5000 && CheckOnlineInProgress==true){
    OnlineStatus=false;
    CheckOnlineInProgress=false;
  }

  static long PingTimer = 0;
  if(millis()-PingTimer > 120000){
    MqttPubString("get", "0", 0);
    CheckOnlineInProgress=true;
    CheckOnlineTimer=millis();
    PingTimer=millis();
  }

  static long BattTimer = 0;
  if(millis()-BattTimer > 5000){
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(245, 170);
    getBatteryLevel();
    // LcdNeedRefresh=true;

    // if(LowBattery==true){
    //   M5.Lcd.fillScreen(RED);
    //   M5.Lcd.setTextColor(BLACK, RED); // Set pixel color; 1 on the monochrome screen
    //   M5.Lcd.setTextSize(3);
    //   M5.Lcd.setCursor(60,90); M5.Lcd.print("BATTERY LOW");
    //   M5.Lcd.setCursor(60,130); M5.Lcd.print("SHUTDOWN...");
    //   delay(5000);
    //   M5.Power.powerOFF();
    //   M5.update();
    // }
    BattTimer=millis();
  }

  static long ArrowTimer = 0;
  if(millis()-ArrowTimer > 100){
    if( abs(Target-Azimuth)<20 && HidenTarget==false){
      LcdNeedRefresh=true;
      HidenTarget=true;
    }
    ArrowTimer=millis();
  }

  if(millis()-TargetTimeout[0]>TargetTimeout[1] && HidenTarget==false && Status==4){
    TargetTimeout[0]=millis();
    // Target=Azimuth;
    LcdNeedRefresh=true;
    HidenTarget=true;
  }

  static long GetAzTimeout=0;
  if(millis()-GetAzTimeout >100 && Status==4){
    // #if defined(MPU6886)
      M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
      M5.IMU.getAccelData(
          &accX, &accY,
          &accZ);  // Stores the triaxial accelerometer.
      // M5.IMU.getAhrsData(
      //     &pitch, &roll,
      //     &yaw);  // Stores the inertial sensor attitude.
      M5.IMU.getTempData(&temp);  // Stores the inertial sensor temperature to
                                  // temp.

      static float ShiftGyroZ = 0;
      static float GyroZbuffer[100];
      static int ShiftGyroZCounter = -100;

      if(ShiftGyroZCounter>=0 && GyroCalibrate==false){
        GyroCalibrate=true;
        M5.Lcd.fillScreen(BLACK);
        DirectionalRosette(120, 120, 110);
        LcdNeedRefresh=true;
      }
      if( (ShiftGyroZCounter < 0) || (abs(gyroZ-ShiftGyroZ) < 3 && ShiftGyroZCounter >= 0) ){
        if(ShiftGyroZCounter<0){
          GyroZbuffer[ShiftGyroZCounter+100] = gyroZ;
          ShiftGyroZCounter++;
        }else{
          GyroZbuffer[ShiftGyroZCounter] = gyroZ;
          ShiftGyroZCounter++;
          if(ShiftGyroZCounter >99){
            ShiftGyroZCounter = 0;
          }
        }
        for (int i=0; i<100; i++) {
          ShiftGyroZ = ShiftGyroZ + GyroZbuffer[i];
        }
        ShiftGyroZ = ShiftGyroZ/100;
      }

      if(FAstMove==true){
        if(abs(gyroZ-ShiftGyroZ)>3 && ShiftGyroZCounter >= 0){
          Target=Target+(int)(gyroZ-ShiftGyroZ);
        }
      }else{
        Target=Target+(int)( (gyroZ-ShiftGyroZ)/6);
      }

      // Serial.print("X Y Z acc ");
      // Serial.print(accX);
      // Serial.print(" ");
      // Serial.print(accY);
      // Serial.print(" ");
      // Serial.print(accZ);
      // Serial.print("gyroZ ");
      // Serial.print(gyroX);
      // Serial.print(" ");
      // Serial.print(gyroY);
    // Serial.print(gyroZ);
    // Serial.print(" ");
    // Serial.print(ShiftGyroZ);
    // Serial.print(" ");
    // Serial.println(gyroZ-ShiftGyroZ);
      // Serial.print(" ");
      // Serial.println(ShiftGyroZCounter);
      // Serial.print(" pitch/roll/yaw/temp ");
      // Serial.print(pitch);
      // Serial.print(" ");
      // Serial.print(roll);
      // Serial.print(" ");
      // Serial.print(yaw);
      // Serial.print(" ");
      // Serial.print(temp);
      // Serial.print("|");
      // Serial.println(Target);
    // #endif

    #if defined(HMC5883L)
      HMC5883L();
    #endif

    #if defined(MPU9250)
      GetGyroData();
      if(FAstMove==true){
        Target=Target+(int)(IMU.gz)+(int)(IMU.gy);
      }else{
        Target=Target+((int)(IMU.gz)+(int)(IMU.gy))/8;
      }
    #endif

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
    GetAzTimeout=millis();
  }

}
//------------------------------------------------------------------------------
#if defined(HMC5883L)
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
#endif

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
int Shifted(int DEG){
  DEG=DEG+StartAzimuth;
  if(DEG>359){
    DEG=DEG-360;
  }
  return DEG;
}
//------------------------------------------------------------------------------

void LcdShow(int SHIFT){
  // /usr/bin/xplanet -window -longitude 13.7917 -latitude 50.3542 -geometry 240x240 -projection azimuthal -num_times 1 -output map.bmp
  // 320x240px
  //  IMU.delt_t = millis() - IMU.count;
  //  if (IMU.delt_t > 100){
  if(LcdNeedRefresh==true){
    // M5.Lcd.setBrightness(200);
    if(EnableMap==true){
      M5.Lcd.drawJpgFile(SD, "/map.jpg");
      DirectionalRosette(120, 120, 110);
      int deg=Shifted(Target+AntRadiationWidth/2);
      int deg2=Shifted(Target-AntRadiationWidth/2);
      if(HidenTarget==false){
        // M5.Lcd.drawTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), WHITE);
        deg=Shifted(Target+AntRadiationWidth/2-1);
        deg2=Shifted(Target+AntRadiationWidth/2+1);
        M5.Lcd.fillTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), LIGHTGREY);
        deg=Shifted(Target-AntRadiationWidth/2-1);
        deg2=Shifted(Target-AntRadiationWidth/2+1);
        M5.Lcd.fillTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), LIGHTGREY);
      }
      TargetOld=Target;
      deg=Shifted(Azimuth+AntRadiationWidth/2-1);
      deg2=Shifted(Azimuth+AntRadiationWidth/2+1);
      M5.Lcd.fillTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), RED);
      deg=Shifted(Azimuth-AntRadiationWidth/2-1);
      deg2=Shifted(Azimuth-AntRadiationWidth/2+1);
      M5.Lcd.fillTriangle(120, 120, Xcoordinate(deg,120,100), Ycoordinate(deg,120,100), Xcoordinate(deg2,120,100), Ycoordinate(deg2,120,100), RED);
    }else if(GyroCalibrate==true){
      //  M5.Lcd.fillScreen(BLACK);
      // M5.Lcd.setCursor(0, 32); M5.Lcd.print(" x   y   z  ");

      // M5.Lcd.setCursor(0,  48); M5.Lcd.print((int)(1000*IMU.ax));
      // M5.Lcd.setCursor(32, 48); M5.Lcd.print((int)(1000*IMU.ay));
      // M5.Lcd.setCursor(64, 48); M5.Lcd.print((int)(1000*IMU.az));
      // M5.Lcd.setCursor(96, 48); M5.Lcd.print("mg");

      // M5.Lcd.setCursor(0,  64); M5.Lcd.print((int)(IMU.gx));
      // M5.Lcd.setCursor(32, 64); M5.Lcd.print((int)(IMU.gy));
      Arrow(Shifted(TargetOld),120,120,100, 0x000000);
      if(HidenTarget!=true){
        if(OnlineStatus==true){
          Arrow(Shifted(Target),120,120,100, DARKGREY);
        }else{
          Arrow(Shifted(Target),120,120,100, BLACK);
        }
      }
      TargetOld=Target;
      if(Azimuth!=AzimuthOld){
        Arrow(Shifted(AzimuthOld),120,120,100, BLACK);
        AzimuthOld=Azimuth;
      }
      if(OnlineStatus==true){
        if(Status==4){
          if(Azimuth>359){
            Arrow(Shifted(Azimuth),120,120,100, ORANGE);
          }else{
            Arrow(Shifted(Azimuth),120,120,100, DARKGREEN);
          }
        }else{
          Arrow(Shifted(Azimuth),120,120,100, RED);
        }
      }else{
        Arrow(Shifted(Azimuth),120,120,100, BLACK);
      }
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
    if(OnlineStatus==true){
      M5.Lcd.setTextColor(DARKGREEN ,BLACK);
    }else{
      M5.Lcd.setTextColor(BLACK ,BLACK);
    }
    M5.Lcd.setCursor(210, 10);
    if(Shifted(Azimuth)<10){
      M5.Lcd.print(" ");
    }
    if(Shifted(Azimuth)<100){
      M5.Lcd.print(" ");
    }
    M5.Lcd.print( Shifted(Azimuth) );
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(300, 5);
    M5.Lcd.print("o");
    M5.Lcd.setTextSize(3);
    // if(HidenTarget!=true){
    if( abs(Target-Azimuth)>10 && OnlineStatus==true){
      M5.Lcd.setTextColor(DARKGREY ,BLACK);
    }else{
      M5.Lcd.setTextColor(BLACK ,BLACK);
    }
    M5.Lcd.setCursor(240, 210);
    if(Shifted(Target)<10){
      M5.Lcd.print(" ");
    }
    if(Shifted(Target)<100){
      M5.Lcd.print(" ");
    }
    M5.Lcd.print( Shifted(Target) );
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(297, 205);
    M5.Lcd.print("o");
    M5.Lcd.setTextSize(3);

    M5.Lcd.setCursor(10, 210);
    M5.Lcd.setTextColor(DARKGREY ,BLACK);
    if(FAstMove==true){
      M5.Lcd.print( "F" );
      M5.Lcd.setCursor(26, 217);
      M5.Lcd.setTextSize(2);
      M5.Lcd.print( "ast" );
    }else{
      M5.Lcd.print( "S" );
      M5.Lcd.setCursor(27, 217);
      M5.Lcd.setTextSize(2);
      M5.Lcd.print( "low" );
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
  for (int j=StartAzimuth; j<MaxRotateDegree-360+StartAzimuth; j++) {
    M5.Lcd.fillCircle(Xcoordinate(j,X,R), Ycoordinate(j,Y,R), 3, 0X528A);
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
      // Serial.print("j= ");
      // Serial.print(j);
      // Serial.print("|StartAzimuth= ");
      // Serial.print(StartAzimuth);
      // Serial.print("|MAX= ");
      // Serial.println(MaxRotateDegree-360+StartAzimuth);
      if( j >= StartAzimuth/10 && j <= (MaxRotateDegree-360+StartAzimuth)/10 ){
        M5.Lcd.fillCircle(Xcoordinate(j*10,X,R), Ycoordinate(j*10,Y,R), 2, BLACK);
      }else{
        M5.Lcd.fillCircle(Xcoordinate(j*10,X,R), Ycoordinate(j*10,Y,R), 2, DARKGREY);
      }
    }
  }
}
//------------------------------------------------------------------------------
int8_t getBatteryLevel()
{
  static byte BattStatusPrev = 0x00;
  static byte BattStatus = 0x00;
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  if (Wire.endTransmission(false) == 0
   && Wire.requestFrom(0x75, 1)) {
    BattStatus = Wire.read() & 0xF0;
    switch (BattStatus) {
    case 0xE0: BattIcon(290, 130, RED, 1);
    LowBattery=true;
    break;
    case 0xC0: BattIcon(290, 130, LIGHTGREY, 2);
    LowBattery=false;
    break;
    case 0x80: BattIcon(290, 130, DARKGREY, 3);
    LowBattery=false;
    break;
    case 0x00: BattIcon(290, 130, DARKGREY, 4);
    LowBattery=false;
    break;
    default: return 0;
    break;
    }
  }
  if(BattStatus != BattStatusPrev){
    BattStatusPrev = BattStatus;
    LcdNeedRefresh = true;
  }
  return -1;
}
//------------------------------------------------------------------------------
#if defined(MPU9250)
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
#endif
