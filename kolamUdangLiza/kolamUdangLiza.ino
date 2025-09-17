/*------------------------------------------------------------------
  ===================  Seksyen 1 - HEADER        ===================
  ------------------------------------------------------------------*/
//-------------------------------------------------------------------
//=  A. - Library  include and define  yang diperlukan              =
//-------------------------------------------------------------------

#include <WiFi.h>
#include <MQTT.h>
#include <Wire.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>;
#include "DFRobot_PH.h"
#include <EEPROM.h>

//--------------------------------------------------------------------
//=  B. - ISTIHAR Sambungan nama Pin dengan Nombor Fizikal PIN ----
//--------------------------------------------------------------------

//---Actuator and Relay  pin connection---
#define relay01     18
#define relay02     19
#define buzzer      2
//#define SensorSuhu  15 // pin sambungan ke DS18B20 (ONEWIRE)
#define SenAM2301   16
#define sensorLDR   34
#define tiltSensor  4
#define PIN         5
#define oneWireBus  4
#define phSensor    35


// ~~~~~~~~~~~~~~~~~~~~ PENTING ~~~~~~~~~~~~~~~~~~~~~~~~
// ------ Sila edit nama atau ID ikut keperluan --------
#define Client_Id   "2025fishfeederPTSB03indoorMain"
//#define NamaBroker  "broker.hivemq.com"
#define NamaBroker  "broker.emqx.io"
//#define NamaBroker  "103.186.117.135"
//#define namaSSID    "HaMa iPhone 13";
//#define SSIDpwd     "1234556790";
#define namaSSID    "Haza@unifi";
#define SSIDpwd     "0135335045";
//#define namaSSID    "IoT";
//#define SSIDpwd     "iot@kksj2023";
// ~~~~~~~~~~~~~~~~~~~  TAMMAT   ~~~~~~~~~~~~~~~~~~~~~~~

//=   C.  - ISTIHAR  constant dan pembolehubah------------------------------
//---Penetapan nama Pembolehubah yg diumpukkan kepada satu nilai awal  --
const char ssid[] = namaSSID;
const char pass[] = SSIDpwd;

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

LiquidCrystal_I2C lcd(0x27,20,4);

//------ ISTIHAR Pemboleh ubah dengan jenis Data yang digunakan---
unsigned long lastMillis = 0;
//float dataSuhuC; // suhu dalam Celsius
int suisKon;
int tiltState = 0;
float duration_us, distance_cm;
int hum; //Stores humidity value
float temp; //Stores temperature value

const int sampleSize = 30; // Number of samples for the moving average
int readings[sampleSize]; // Array to store the readings
int readIndex = 0;        // Index of the current reading
long total = 0;    
//-=   D. - Cipta Objek dari Librari berkaitan------------------ ----
//--------------------------------------------------------------------------

WiFiClient net;
MQTTClient client;

// Istihar objek bagi Module OLED Display - SSD1306 
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// Istihar objek bagi Module Sersor Suhu Dallas 18B20
//OneWire oneWire(SensorSuhu);
//DallasTemperature sensors(&oneWire);
DHT dht(SenAM2301, DHT21);

//##################  Seksyen 1 - TAMAT #############################

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect(Client_Id,"husaini","aza")) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected!");
  client.subscribe("fishfeederPTSBmain/buzzer");

  // -- tambah subcribe disini ---
 
}

void messageReceived(String &topic, String &payload) {
  //Serial.println("incoming: " + topic + " - " + payload);
   Serial.println(payload);
  
  
   //==========================Buzzer Control ==========
  if(String(topic) == "fishfeederPTSBmain/buzzer") 
  {
      if(payload =="on")
      {
        Serial.println(payload);
        digitalWrite(buzzer,HIGH);
        Serial.println("Buzzer ON");
        delay(500);
      }
      
      else if(payload =="off")
      {
        Serial.println(payload);
        digitalWrite(buzzer,LOW);
        Serial.println("Buzzer OFF");
      }
  } 

//==============================================

//==========================Device Control ===========
   
//   ----Tulis Kod Kawalan ( subsribe here ) -------

//--------------------------- END --------------------

}

//==================  Seksyen 2 - Fungsi Setup ======================
//-------------------------------------------------------------------

void setup() {
  pinMode(relay01,OUTPUT);
  pinMode(relay02,OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(tiltSensor, INPUT);
 
  Serial.begin(115200); // initialize serial

  WiFi.begin(ssid, pass);
  client.begin(NamaBroker, net);
  client.onMessage(messageReceived);
 connect();
  //sensors.begin();    // initialize the DS18B20 sensor
  dht.begin();
  sensors.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("---  KOLAM UDANG ---");
  delay(3000);
 
}
//##################  Seksyen 2 - TAMAT #############################



//==============  Seksyen 3 - Fungsi Utama (LOOP) ===================
//-------------------------------------------------------------------
void loop() {

 client.loop();
 delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

// publish a message roughly every second.
  if (millis() - lastMillis > 3000) {
    lastMillis = millis();

    //------SENSOR 1-------------------------------------------------------------
  
    //------SENSOR seterusnya-------------------------------------------------------------
   
   
    //-----------end ----------------------------------

   //sensorTilt();
   sensorCahaya();
   suhudht();
   dallasSensor();
   sensorPH();

  }

  
}
//##################  Seksyen 3 - TAMAT #############################



//-------------sensor cahaya -------------------------------------------
void sensorCahaya() {

float dataLDR = analogRead(sensorLDR); //Read light level
float square_ratio = dataLDR / 4095.0; //Get percent of maximum value (4095)
square_ratio = pow(square_ratio, 2.0);
Serial.print("bacaan Cahaya: ");
Serial.println(dataLDR);

float VoltCahaya= dataLDR * 3.3 / 4095.0;
Serial.print("bacaan Voltan cahaya: ");
Serial.println(VoltCahaya);

float amps = VoltCahaya / 10000.0;  // em 10,000 Ohms
float microamps = amps * 1000000; // Convert to Microamps
float lux = (microamps * 3.0) +10; // Convert to Lux */

Serial.print("bacaan LUX cahaya: ");
Serial.println(lux);

Serial.print("bacaan ration cahaya: ");
Serial.println(square_ratio);
Serial.println(microamps);

client.publish("fishfeederPTSBmain/lux", String(lux)); 

}

//-------------sensor shu AMS (DHT22) ------------------------------------
void suhudht() {

    hum = dht.readHumidity();
    temp= dht.readTemperature();
 
 //Print temp and humidity values to serial monitor
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.print("%, Temperature: ");
    Serial.print(temp);
    Serial.println(" Celsius");
    client.publish("fishfeederPTSBmain/suhu", String(temp)); 
    client.publish("fishfeederPTSBmain/humiditi", String(hum)); 

    lcd.setCursor(0,1);
    lcd.print("TMP:");
    lcd.print(temp,1);
    lcd.setCursor(10,1);
    lcd.print("HUM:");
    lcd.print(hum,0);
  
}

void dallasSensor(void)
{
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
 
  float tempC = sensors.getTempCByIndex(0);

  Serial.print("Water Temperature: ");
  Serial.println(tempC);

  client.publish("fishfeederPTSBmain/tankTemp", String(tempC)); 

  
}

void sensorPH() {
    
    int newReading = analogRead(phSensor);
    total -= readings[readIndex];
    readings[readIndex] = newReading;
    total += newReading;
    readIndex = (readIndex + 1) % sampleSize;
    int smoothedValue = total / sampleSize;
    Serial.print("Raw value: ");
    Serial.print(newReading);
    Serial.print("\tSmoothed value: ");
    Serial.println(smoothedValue);

    //Serial.print(dataSensorPH);
    //Serial.print(" | ");
    float voltage=smoothedValue*(3.3/4095.0);
    float ph=(3.3*voltage)-2;
    Serial.println(ph);
    client.publish("fishfeederPTSBmain/ph", String(ph));
    
    lcd.setCursor(0,2);
    lcd.print("bacaan pH :");
    lcd.setCursor(14,2);
    lcd.print(ph,1);

  //  https://www.youtube.com/watch?v=Wdu5w0rZfvE&t=235s
}