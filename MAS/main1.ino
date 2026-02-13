#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <time.h>

// ================== PIN CONFIG ==================
#define SDA 21
#define SCL 22
#define RX 16
#define TX 17
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2
#define button 32

// ================== HEART RATE SENSOR ==================
#define HR_PIN 34
#define HR_MIN_BPM 45
#define HR_MAX_BPM 120

// ================== WIFI / THINGSPEAK ==================
const char* ssid = "Pixel 8a";
const char* password = "Justin2022";
const char* apiKey = "JKON2VLDXG10Z0LV";
String tsServer = "http://api.thingspeak.com/update?";

// ================== OBJECTS ==================
Adafruit_MPU6050 mpu;
HardwareSerial GPS(2);
TinyGPSPlus gps;

// ================== PARAMETERS ==================
#define N 25
#define FS_HZ 50
#define ALIM_TH 1.8
#define ANGLE_DELTA_TH 60.0
#define ANGLE_ROLL_TH 100.0
#define ANGLE_PITCH_TH 50.0
#define ANGLE_BETA_TH 100
#define MODULE_TH 10
#define alphaLPF 0.98

// ================== VARIABLES ==================
float gravX=0, gravY=0, gravZ=0;
float axBuf[N], ayBuf[N], azBuf[N];
float gxBuf[N], gyBuf[N], gzBuf[N];
sensors_event_t a, g, temp;

float ax,ay,az,gx,gy,gz,lax,lay,laz;
float pitch,roll,alim,mag1,mag2,dot,angle_deg;
double lat=0.0,lng=0.0;

bool f_roll,f_pitch,f_alim,fa_angle,fg_angle,f_module;
bool f_hr_abnormal=false;

unsigned long lastMillis=0;
int votes=0;
int oldest=0, newest=N-1;

// ================== HEART RATE VARIABLES ==================
int hrRaw=0;
float hrFiltered=0, hrBaseline=0, bpm=0;
float hrThreshold=40;
bool beatArmed=true;
unsigned long lastBeatMs=0;

// ================== PROTOTYPES ==================
bool Post_Confirmation();
void updateHeartRate();
bool isPatientCriticalAfterFall();
void sendLoRa(bool fall);
void sendToThingSpeak(bool fall);

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  pinMode(button, INPUT_PULLUP);

  GPS.begin(9600, SERIAL_8N1, RX, TX);
  Wire.begin(SDA, SCL);

  analogReadResolution(12);
  pinMode(HR_PIN, INPUT);

  while (!mpu.begin()) delay(10);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  for(int i=0;i<100;i++){
    mpu.getEvent(&a,&g,&temp);
    gravX+=a.acceleration.x;
    gravY+=a.acceleration.y;
    gravZ+=a.acceleration.z;
    delay(10);
  }
  gravX/=100; gravY/=100; gravZ/=100;

  LoRa.setPins(LORA_SS,LORA_RST,LORA_DIO0);
  while(!LoRa.begin(433E6));
  Serial.println("✅ LoRa Ready");

  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED) delay(500);
  Serial.println("✅ WiFi Connected");

  configTime(0,0,"pool.ntp.org");
  lastMillis=millis();
}

// ================== LOOP ==================
void loop() {
  if(millis()-lastMillis<1000/FS_HZ) return;
  lastMillis=millis();

  mpu.getEvent(&a,&g,&temp);
  ax=a.acceleration.x; ay=a.acceleration.y; az=a.acceleration.z;
  gx=g.gyro.x; gy=g.gyro.y; gz=g.gyro.z;

  lax=ax-gravX; lay=ay-gravY; laz=az-gravZ;
  gravX=alphaLPF*ax; gravY=alphaLPF*ay; gravZ=alphaLPF*az;

  updateHeartRate();

  for(int i=N-1;i>0;i--){
    axBuf[i]=axBuf[i-1]; ayBuf[i]=ayBuf[i-1]; azBuf[i]=azBuf[i-1];
    gxBuf[i]=gxBuf[i-1]; gyBuf[i]=gyBuf[i-1]; gzBuf[i]=gzBuf[i-1];
  }
  axBuf[0]=ax; ayBuf[0]=ay; azBuf[0]=az;
  gxBuf[0]=gx; gyBuf[0]=gy; gzBuf[0]=gz;

  pitch=atan2(-ax,sqrt(ay*ay+az*az))*180/PI;
  roll=atan2(ay,az)*180/PI;
  alim=sqrt(lax*lax+lay*lay+laz*laz);

  f_pitch=fabs(pitch)>ANGLE_PITCH_TH;
  f_roll=fabs(roll)>ANGLE_ROLL_TH;
  f_alim=alim>ALIM_TH;

  dot=axBuf[newest]*axBuf[oldest]+ayBuf[newest]*ayBuf[oldest]+azBuf[newest]*azBuf[oldest];
  mag1=sqrt(axBuf[newest]*axBuf[newest]+ayBuf[newest]*ayBuf[newest]+azBuf[newest]*azBuf[newest]);
  mag2=sqrt(axBuf[oldest]*axBuf[oldest]+ayBuf[oldest]*ayBuf[oldest]+azBuf[oldest]*azBuf[oldest]);
  angle_deg=(mag1*mag2>0)?acos(dot/(mag1*mag2))*180/PI:0;
  fa_angle=angle_deg>ANGLE_DELTA_TH;

  f_hr_abnormal=(bpm>0&&(bpm<HR_MIN_BPM||bpm>HR_MAX_BPM));

  votes=f_pitch+f_roll+f_alim+fa_angle+f_hr_abnormal;

  Serial.print("BPM: "); Serial.print(bpm,1);
  Serial.print(" | votes: "); Serial.println(votes);

  if(votes>=4 && Post_Confirmation()){
    sendLoRa(true);
    sendToThingSpeak(true);

    Serial.println("⚠️ CHUTE CONFIRMÉE");

    if(isPatientCriticalAfterFall()){
      Serial.println("🔴 ÉTAT CRITIQUE APRÈS LA CHUTE");
    }else{
      Serial.println("🟢 ÉTAT STABLE APRÈS LA CHUTE");
    }
  }
}

// ================== HEART RATE ==================
void updateHeartRate(){
  hrRaw=analogRead(HR_PIN);
  hrFiltered=0.9*hrFiltered+0.1*hrRaw;
  hrBaseline=0.995*hrBaseline+0.005*hrFiltered;
  float signal=hrFiltered-hrBaseline;
  unsigned long now=millis();

  if(beatArmed && signal>hrThreshold){
    if(now-lastBeatMs>250){
      bpm=60000.0/(now-lastBeatMs);
      lastBeatMs=now;
    }
    beatArmed=false;
  }
  if(!beatArmed && signal<hrThreshold*0.3) beatArmed=true;
}

// ================== POST FALL HEALTH ==================
bool isPatientCriticalAfterFall(){
  int abnormal=0;
  for(int i=0;i<5;i++){
    updateHeartRate();
    if(bpm>0&&(bpm<HR_MIN_BPM||bpm>HR_MAX_BPM)) abnormal++;
    delay(1000);
  }
  return abnormal>=3;
}

// ================== POST CONFIRM ==================
bool Post_Confirmation(){
  delay(3000);
  return true;
}

// ================== COMM ==================
void sendLoRa(bool fall){
  LoRa.beginPacket();
  LoRa.print(fall?"CHUTE:1":"FAUSSE_ALERTE:1");
  //LoRa.print(",BPM:");
  //LoRa.print(bpm,1);
  LoRa.endPacket();
}

void sendToThingSpeak(bool fall){
  if(WiFi.status()!=WL_CONNECTED) return;
  HTTPClient http;
  String url=tsServer+"api_key="+apiKey+"&field1="+String(fall?1:0)+"&field5="+String(bpm,1);
  http.begin(url);
  http.GET();
  http.end();
}
