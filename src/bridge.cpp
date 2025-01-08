// bridge_polite
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, routed reliable messaging client
// with the RHMesh class.
// It is designed to work with the other examples rf95_mesh_server*
// Hint: you can simulate other network topologies by setting the
// RH_TEST_NETWORK define in RHRouter.h

// Mesh has much greater memory requirements, and you may need to limit the
// max message length to prevent wierd crashes
#define RH_MESH_MAX_MESSAGE_LEN 70

#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>
#include <HT_SSD1306Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// #include <SSD1306Wire.h>
#include <SD.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Adafruit_Sensor.h>
#include "Arduino.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Update.h>

#include <otadrive_esp.h>

// In this small artifical network of 4 nodes,
#define BRIDGE_ADDRESS 1 // address of the bridge ( we send our data to, hopefully the bridge knows what to do with our data )

// lilygo T3 v2.1.6
// lora SX1276/8
#define LLG_SCK 5
#define LLG_MISO 19
#define LLG_MOSI 27
#define LLG_CS 18
#define LLG_RST 23
#define LLG_DI0 26
#define LLG_DI1 33
#define LLG_DI2 32

#define LLG_LED_GRN 25

// oled
#define LLG_OLED_SDA 21
#define LLG_OLED_SCL 22

// tfcard
#define LLG_SD_CS 13
#define LLG_SD_MISO 2
#define LLG_SD_MOSI 15
#define LLG_SD_SCK 14

#define RXTIMEOUT 5000 // it is roughly the delay between successive transmissions
//================================================== VARIAVEIS ======================================================================

uint8_t data[] = "Hello back from bridge";
// Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
uint8_t res;

// Singleton instance of the radio driver
RH_RF95 rf95(LLG_CS, LLG_DI0); // slave select pin and interrupt pin, [heltec|ttgo] ESP32 Lora OLED with sx1276/8

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(rf95, BRIDGE_ADDRESS);

// WiFi and MQTT configuration
const char *ssid = "Gigo 2.4G";
//const char *ssid = "iphone bia(2)";
const char *password = "18253122Ro";
//const char* mqtt_server = "192.168.70.103";
// const char* mqtt_server = "192.168.15.10";
const char *mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char *mqtt_user = "espmeshguard123";
const char *mqtt_password = "your_MQTT_PASSWORD";

WiFiClient espClient;
PubSubClient client(espClient);

HTTPClient http;



//===============================================  FUNÇÕES  ======================================================================

//===============================================  OTAdrive  ======================================================================

// OTAdrive APIkey for this product
#define APIKEY "bd43d323-1857-4125-9e4b-0a32bd8d0883"
// this app version
#define FW_VER "v@1.0.0"

void onUpdateProgress(int progress, int totalt)
{
  static int last = 0;
  int progressPercent = (100 * progress) / totalt;
  Serial.print("*");
  if (last != progressPercent && progressPercent % 10 == 0)
  {
    // print every 10%
    Serial.printf("%d", progressPercent);
  }
  last = progressPercent;
}

void checkForUpdates()
{
  OTADRIVE.setInfo(APIKEY, FW_VER);
  OTADRIVE.onUpdateFirmwareProgress(onUpdateProgress);

  // retrive firmware info from OTAdrive server
  auto inf = OTADRIVE.updateFirmwareInfo();
  if (WiFi.status() == WL_CONNECTED)
    // retrive firmware info from OTAdrive server
    auto inf = OTADRIVE.updateFirmwareInfo();

  // update firmware if newer available
  if (inf.available)
  {
    log_d("\nNew version available, %dBytes, %s\n", inf.size, inf.version.c_str());
    OTADRIVE.updateFirmware();
  }
  else
  {
    log_d("\nNo newer version\n");
  }
}
//================================================== LoRa  ======================================================================
void receiveHandle()
{
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAck(buf, &len, &from))
  {
    Serial.print("request from node n.");
    Serial.print(from);
    Serial.print(": ");
    Serial.print((char *)buf);
    Serial.print(" rssi: ");
    Serial.println(rf95.lastRssi());

    // Send a reply back to the originator client
    res = manager.sendtoWait(data, sizeof(data), from);
    if (res != RH_ROUTER_ERROR_NONE)
    {
      Serial.print("sendtoWait failed:");
      Serial.println(res);
    }
  }
}
//===============================================  OLED Definições  ======================================================================
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
void meshGuard_Logo()
{
  display.init();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  // Mantém o título por mais 2 segundos
  display.drawString(0, 22, "MeshGuard");
  display.drawString(0, 40, "Roteador");
  display.display();
  delay(4000);

  // Limpa a tela
  display.clear();
  display.display();
}

void testeOLED()
{

  display.init();
  display.clear();
  display.display();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  // Data to display
  String lines[] = {
      "WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"),
      "IP: " + WiFi.localIP().toString(),
      "MQTT: " + String(client.connected() ? "Connected" : "Disconnected"),
      "MQTT Serv: " + String(mqtt_server),
      "SSID: " + String(ssid),
      "LoRa Msg: " + String(manager.available() ? "Received: " + String((char *)buf) : "None")};

  int totalLines = sizeof(lines) / sizeof(lines[0]);
  int displayLines = 4; // Number of lines to display at once
  int startLine = 0;

  // Scroll effect
  for (int i = 0; i < totalLines + displayLines; i++)
  {
    display.clear();
    for (int j = 0; j < displayLines; j++)
    {
      int lineIndex = (startLine + j) % totalLines;
      display.drawString(0, j * 12, lines[lineIndex]);
    }
    display.display();
    startLine++;
    delay(1000); // Adjust delay for scroll speed
  }
}
//===============================================  WiFi e MQTT  ======================================================================
void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  static unsigned long lastAttemptTime = 0;
  static int attemptCount = 0;
  const int maxAttempts = 3;
  unsigned long now = millis();

  if (!client.connected() && attemptCount < maxAttempts)
  {
    if (now - lastAttemptTime > 5000)
    {
      //Serial.print("Attempting MQTT connection...");
      if (client.connect("ESP32Clientmeshguard1234"))
      {
        //Serial.println("connected");
        attemptCount = 0; // Reset attempt count on successful connection
      }
      else
      {
        //Serial.print("failed, rc=");
        //Serial.print(client.state());
        //Serial.println(" try again in 5 seconds");
        attemptCount++;
        lastAttemptTime = now;
      }
    }
  }

  if (attemptCount == maxAttempts)
  {
    //Serial.println("Max reconnection attempts reached. Will try again later.");
  }
}

void publishToMQTT(const char *topic, const char *payload)
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  client.publish(topic, payload);
}

void setup_mqtt()
{
  client.setServer(mqtt_server, mqtt_port);
  reconnect();
}

void handleLoRaData()
{
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAck(buf, &len, &from))
  {
    //Serial.print("request from node n.");
    //Serial.print(from);
    //Serial.print(": ");
    Serial.print((char *)buf);
    //Serial.print(" rssi: ");
    //Serial.println(rf95.lastRssi());

    // Publish to MQTT
    publishToMQTT("meshguard/id3/data", (char *)buf);

    // Send a reply back to the originator client
    res = manager.sendtoWait(data, sizeof(data), from);
    if (res != RH_ROUTER_ERROR_NONE)
    {
      Serial.print("sendtoWait failed:");
      Serial.println(res);
    }
  }
}

void enviarMensagem(String valorLeitura) {
  Serial.print("#");  // Início da mensagem
  Serial.print(valorLeitura);  // Valor da leitura
}

//================================================== LoRa  ======================================================================
void receiveConvertData()
{
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (manager.recvfromAck(buf, &len, &from))
  {
    //Serial.print("request from node n.");
    //Serial.print(from);
    //Serial.print(": ");
    Serial.print((char *)buf);
    //Serial.print(" rssi: ");
    //Serial.println(rf95.lastRssi());
    enviarMensagem((char *)buf);


    

    // Send a reply back to the originator client
    res = manager.sendtoWait(data, sizeof(data), from);
    if (res != RH_ROUTER_ERROR_NONE)
    {
      //Serial.print("sendtoWait failed:");
      //Serial.println(res);

      // Transform the received uint8_t data into char
      char sensorsData[RH_MESH_MAX_MESSAGE_LEN];
      snprintf(sensorsData, sizeof(sensorsData), "%s", (char *)buf);
      Serial.println(sensorsData);

      // Store received LoRa data into the FIFO queue
      static const int fifoSize = 1;
      static String fifoPublishTopic[fifoSize];
      static int fifoIndex = 0;

      if (fifoIndex < fifoSize)
      {
        fifoPublishTopic[fifoIndex] = sensorsData;
        fifoIndex++;
      }
      else
      {
        // If the FIFO queue is full, reset the index to overwrite the oldest data
        fifoIndex = 0;
        fifoPublishTopic[fifoIndex] = sensorsData;
      }

      // Publish all messages in the FIFO queue to the MQTT topic
      for (int i = 0; i < fifoSize; i++)
      {
        if (fifoPublishTopic[i].length() > 0)
        {
          publishToMQTT("meshguard/id3/data", fifoPublishTopic[i].c_str());
        }
      }

      // Send a reply back to the originator client
      res = manager.sendtoWait(buf, len, from);
      if (res != RH_ROUTER_ERROR_NONE)
      {
        //Serial.print("sendtoWait failed:");
        //Serial.println(res);
      }
    }
  }
}
//==================================================  SETUP  ======================================================================
void setup()
{
  Serial.begin(115200);
  Wire.begin(4, 15);
  // Wire1.begin(21, 22);
  display.init();
  display.setFont(ArialMT_Plain_10);
  meshGuard_Logo();
  setup_wifi();
  checkForUpdates();

  setup_mqtt();
  //Serial.print(F("initializing node "));
  //Serial.print(BRIDGE_ADDRESS);
  SPI.begin(LLG_SCK, LLG_MISO, LLG_MOSI, LLG_CS);
  if (!manager.init())
  {
    //Serial.println(" init failed");
  }
  else
  {
    //Serial.println(" done");
  } // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36

  rf95.setTxPower(2, false); // with false output is on PA_BOOST, power from 2 to 20 dBm, use this setting for high power demos/real usage
  // rf95.setTxPower(1, true); // true output is on RFO, power from 0 to 15 dBm, use this setting for low power demos ( does not work on lilygo lora32 )
  rf95.setFrequency(868.0);
  rf95.setCADTimeout(500);

  // long range configuration requires for on-air time
  boolean longRange = false;
  if (longRange)
  {
    // custom configuration
    RH_RF95::ModemConfig modem_config = {
        0x78, // Reg 0x1D: BW=125kHz, Coding=4/8, Header=explicit
        0xC4, // Reg 0x1E: Spread=4096chips/symbol, CRC=enable
        0x08  // Reg 0x26: LowDataRate=On, Agc=Off.  0x0C is LowDataRate=ON, ACG=ON
    };
    rf95.setModemRegisters(&modem_config);
  }
  else
  {
    // Predefined configurations( bandwidth, coding rate, spread factor ):
    // Bw125Cr45Sf128     Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
    // Bw500Cr45Sf128     Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
    // Bw31_25Cr48Sf512   Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
    // Bw125Cr48Sf4096    Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range
    // Bw125Cr45Sf2048    Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range
    if (!rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128))
    {
      //Serial.println(F("set config failed"));
    }
  }
  //Serial.println("RF95 ready");
}
//==================================================  LOOP  ======================================================================
void loop()
{
  // receiveHandle();
  // handleLoRaData();
  //receiveConvertData();
  static unsigned long lastReceiveTime = 0;
  const unsigned long receiveInterval = 1000; // Interval in milliseconds

  unsigned long currentMillis = millis();
  if (currentMillis - lastReceiveTime >= receiveInterval) {
    lastReceiveTime = currentMillis;
    receiveConvertData();
  }
  testeOLED();
}
