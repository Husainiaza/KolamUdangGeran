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
#include <OneWire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <DallasTemperature.h>
#include <DHT.h>;

//--------------------------------------------------------------------------
//-----b. - ISTIHAR Sambungan nama Pin dengan Nombor Fizikal PIN ----
//--------------------------------------------------------------------------

//---Actuator and Relay  pin connection---
#define relay01     27 
#define relay02     26
#define buzzer      25
#define SensorSuhu  15 // pin sambungan ke DS18B20 (ONEWIRE)
#define sensorLDR   36
#define DHTPIN      5
#define distanceSensor  34
#define tiltSensor  4

//---Penentuan nama Pembolehubah nama yg diumpukkan kepada satu nilai awal yg ditetapkan --
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define DHTTYPE DHT21   // DHT 21 (AM2301)

// ~~~~~~~~~~~~~~~~~~~~ PENTING ~~~~~~~~~~~~~~~~~~~~~~~~
// ------ Sila edit nama atau ID ikut keperluan --------
#define Client_Id   "2025fishfeederPTSB03indoor000003"
#define NamaBroker  "broker.emqx.io"
#define namaSSID    "Haza@unifi";
#define SSIDpwd     "0135335045";
// ~~~~~~~~~~~~~~~~~~~  TAMMAT   ~~~~~~~~~~~~~~~~~~~~~~~

//-----c.  - ISTIHAR  constant dan pembolehubah------------------------------
//---Penetapan nama Pembolehubah yg diumpukkan kepada satu nilai awal  --
const char ssid[] = namaSSID;
const char pass[] = SSIDpwd;

//------ ISTIHAR Pemboleh ubah dengan jenis Data yang digunakan---
unsigned long lastMillis = 0;
float dataSuhuC; // suhu dalam Celsius
int tiltState = 0;

//-----d. - Cipta Objek dari Librari berkaitan------------------ ----
//--------------------------------------------------------------------------

WiFiClient net;
MQTTClient client;

// Istihar objek bagi Module OLED Display - SSD1306 
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Istihar objek bagi Module Sersor Suhu Dallas 18B20
OneWire oneWire(SensorSuhu);
DallasTemperature sensors(&oneWire);

DHT dht(DHTPIN, DHTTYPE);

//##################  Seksyen 1 - TAMAT #############################
//--------------FUNCTION----------------------------

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect(Client_Id)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected!");
  client.subscribe("perantifishfeederPTSB03Indoor/motor");
  client.subscribe("fishfeederPTSB03/relay1");
  client.subscribe("fishfeederPTSB03/motorfeeder");
  client.subscribe("fishfeederPTSB03/buzzer");
  // -- tambah subcribe disini ---
 
}

void messageReceived(String &topic, String &payload) {
  //Serial.println("incoming: " + topic + " - " + payload);
   Serial.println(payload);
  
  //==============================================================

  //==========================Relay Control ========================
  if (String(topic) == "fishfeederPTSB03/relay1") {
    Serial.print("Changing output to ");
    if(payload == "on"){
      Serial.println("on");
      digitalWrite(relay01,HIGH);
      client.publish("fishfeederPTSB03/relay1/status", "RELAY 1 ON");
      
      
    }
    else if(payload == "off"){
      Serial.println("off");
      digitalWrite(relay01,LOW);
      client.publish("fishfeederPTSB03/relay1", "RELAY 1 OFF");
      
    }
  }

 if (String(topic) == "fishfeederPTSB03/motorfeeder") {
    Serial.print("Changing output to ");
    if(payload == "on"){
      Serial.println("on");
      digitalWrite(relay02,HIGH);
      client.publish("fishfeederPTSB03/motorfeeder/status", "RELAY 2 ON");
    }
    else if(payload == "off"){
      Serial.println("off");
      digitalWrite(relay02,LOW);
       client.publish("fishfeederPTSB03/motorfeeder/status", "RELAY 2 OFF");
    }
  }

  if (String(topic) == "fishfeederPTSB03/buzzer") {
    Serial.print("Changing output to ");
    if(payload == "on"){
      Serial.println("on");
      digitalWrite(buzzer,HIGH);
      client.publish("fishfeederPTSB03/buzzer/status", "RELAY 2 ON");
    }
    else if(payload == "off"){
      Serial.println("off");
      digitalWrite(buzzer,LOW);
       client.publish("fishfeederPTSB03PTSB/buzzer/status", "RELAY 2 OFF");
    }
  }

  
  
  //   ----Tulis Kod Kawalan ( subsribe here ) -------

  //--------------------------- Relay Control --------------------


}


//###################################################
//==================  Seksyen 2 - Fungsi Setup ======================
//-------------------------------------------------------------------

void setup() {
  pinMode(relay01,OUTPUT);
  pinMode(relay02,OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(tiltSensor, INPUT);

  Serial.begin(115200); // initialize serial
/*
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  } */

  WiFi.begin(ssid, pass);

  client.begin(NamaBroker, net);
  client.onMessage(messageReceived);


  sensors.begin();    // initialize the DS18B20 sensor
   dht.begin();

  connect();

/*
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("---  ESI-DEVKIT  ---");
  display.setCursor(0,10);
  display.println("K. Komuniti Sbg Jaya");
  display.setCursor(10,20);
  display.println("--------------------");
  display.display();
  delay(5000); */
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
  if (millis() - lastMillis > 5000) {
  lastMillis = millis();

    //------SENSOR 1-------------------------------------------------------------
    sensors.requestTemperatures(); 
    dataSuhuC = sensors.getTempCByIndex(0);
    Serial.print(dataSuhuC);
    Serial.println(" ºC");
    client.publish("fishfeederPTSB03/suhu", String(dataSuhuC,1));
    //----------------------------------------------------------------------------
    //------SENSOR 2--------------------------------------------------------------
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float f = dht.readTemperature(true);

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
   //-----------------------------------------------------------------------------
     
   //----------- Sensor Food Level ----------------------------------
  Serial.println("-----------------Makanan Ikan ------------------");
  float volts = analogRead(distanceSensor) * 0.0008056640625; // value from sensor * (3.3/4096)
  int distance = 29.988 * pow( volts, -1.173);

  Serial.print("FOOD LEVEL: ");
  Serial.println(distance);

 // client.publish("fishfeederPTSB03/foodtankLevel", String(distance));

  if (distance <= 80) {
    Serial.println(distance);
    client.publish("fishfeederPTSB03/foodtank", String(distance));
  }

  int isipadu = distance - 10;
  if (isipadu <10) {
      client.publish("fishfeederPTSB03/foodtank/status", "100");}

  else if ((isipadu >10) && (isipadu <15)){
     client.publish("fishfeederPTSB03/foodtank/status", "75");}

  else if ((isipadu >15) && (isipadu <20)){
    client.publish("fishfeederPTSB03/foodtank/status", "50");}

  else if ((isipadu >20) && (isipadu <25)){
      client.publish("fishfeederPTSB03/foodtank/status", "25");}

  else if (isipadu > 25){
      client.publish("fishfeederPTSB03/foodtank/status", "10");}


  //-----------------------------

    tiltState = digitalRead(tiltSensor);
    Serial.println(tiltState);
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (tiltState == HIGH) {
    // turn LED on:
    digitalWrite(buzzer, HIGH);
    client.publish("fishfeederPTSB03/tanklid", "Food Lid is OPEN");
  } 
    
  else {
    // turn LED off:
    digitalWrite(buzzer, LOW);
    client.publish("fishfeederPTSB03/tanklid", "Food Lid is CLOSE");
  }
 //-----------end ----------------------------------
  }
}

//##################  Seksyen 3 - TAMAT #############################
