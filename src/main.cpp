
#define RH_MESH_MAX_MESSAGE_LEN 50
//====================================================BIBLIOTECAS ====================================================================
#include <Arduino.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>
#include <HT_SSD1306Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SD.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
// #include <WebSocketsClient.h>
#include <Update.h>
// #include <.pio\libdeps\heltec_wifi_lora_32_V2\DFRobot_MAX30102-master\DFRobot_MAX30102-master\src\DFRobot_MAX30102.h>
//  #include <async_http_client.h>
//  #include <AsyncTCP.h>  // Necessário para ESP32
//  #include <AsyncHTTPRequest_Generic.h>
//  #include <AsyncWebServer.h>
//  #include <WiFiClientSecure.h>
//  #include "driver/gpio.h"
#include <WebServer.h>
#include <otadrive_esp.h>
//====================================================  DECLARAÇÕES DE FUNÇÕES  ====================================================================
void playTripleBeep();
void playEmergencySound();
void handleButtonPress();
void buttonConfig();
void buttonCountCheck();
void initializeNode();
void handleTransmission();
void MAX30102Setup();
void ds18b20Setup();
void dhtSetup();
float leituraSensorDs18b20();
float leituraSensorDHT22();
void leituraMAX30102();
void MPU6050Setup();
void handleFallInterrupt();
void leituraMPU6050();
void resetFallDetection();
void Task1code(void *parameter);
void convertString(float sensorDHT, float sensorDS18B20, float bpsData, float oximData, int emergence, int fall, int moduleID);
void setupMQTT();
void reconnectMQTT();
void sendToMQTT();
void allTriangulation();
void meshGuard_Logo();
void displaySetup();
void mostrarIdentific();
bool verificarMensagensLoRa();
void enviarMensagemLoRa();
void Task2Code(void *parameter);
void connectToWiFi();
//====================================================  DEFINIÇÕES DE CONSTANTES  ====================================================================
// In this small artifical network of 4 nodes,
#define BRIDGE_ADDRESS 1 // address of the bridge ( we send our data to, hopefully the bridge knows what to do with our data )
#define NODE_ADDRESS 3   // address of this node
String Nome = "José";
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

// oled MAX30105
#define SDA1 4
#define SCL1 15
// mpu6050
#define SDA2 21
#define SCL2 22
// tfcard
#define LLG_SD_CS 13
#define LLG_SD_MISO 2
#define LLG_SD_MOSI 15
#define LLG_SD_SCK 14

#define TXINTERVAL 3000 // delay between successive transmissions
unsigned long nextTxTime;

// BARRAMENTOS I2C]
// Inicializar o segundo barramento I2C
// TwoWire I2C_1 = TwoWire(0);
// TwoWire I2C_2 = TwoWire(1);

// Singleton instance of the radio driver
RH_RF95 rf95(LLG_CS, LLG_DI0); // slave select pin and interrupt pin, [heltec|ttgo] ESP32 Lora OLED with sx1276/8

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(rf95, NODE_ADDRESS);
uint8_t data[50];
uint8_t data1[] = "Hello World!";
// Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
uint8_t res;
//========================================= PROPRIEDADES DO MQTT ================================================================
WiFiClient espClientLoraMeshguard;
PubSubClient client(espClientLoraMeshguard);
//======================================== MPU6050 ========================================================
// Instâncias dos barramentos I2C
// SoftwareWire mpuWire(21, 22); // SDA, SCL
Adafruit_MPU6050 mpu;
int fallRead; // Variável de leitura de queda
//======================================== MAX30105 ========================================================
// SoftwareWire defaultWire(4, 15); // SDA, SCL
MAX30105 particleSensor;
uint32_t irBuffer[100];  // Buffer para dados do sensor LED infravermelho
uint32_t redBuffer[100]; // Buffer para dados do sensor LED Vermelho

int32_t bufferLength;   // Comprimento de leitura de dados
int32_t spo2 = 99.0;    // Dado Lido de SPO2
int8_t validSPO2;       // Indicador para mostrar se o valor de SPO@ calculado é válido
int32_t heartRate = 75; // Frquência Cardíaca
int8_t validHeartRate;  // indicador para mostrar se a frequencia cardíaca calculada é válida

const byte RATE_SIZE = 4; // Número de amostras para cálculo da média de BPM
byte rates[RATE_SIZE];    // Array para armazenar as leituras de BPM
byte rateSpot = 0;        // Posição no array de BPM
long lastBeat = 0;        // Armazena a última detecção de batimento

// Variáveis para controle dos batimentos cardíacos

// Variáveis para controle de movimento
bool movimentoDetectado = false;

// Limites mínimos
const int MIN_HEART_RATE = 72;
const float MIN_SPO2 = 98.0;

// Incremento/decremento dos batimentos
const int BPM_INCREMENT = 1.5;
const float BPM_DECREMENT = 0.025;

float beatsPerMinute;
int beatAvg;

float finalSpo2;
float spo2Read = 98;
float finalHR;
float heartRateRead = 75;
//======================================= CONFIGURAÇÃO DE OLED ================================================================

static SSD1306Wire display(0x3c, 500000, SDA1, SCL1, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
// static SSD1306Wire display(0x3c, LLG_OLED_SDA, LLG_OLED_SCL); // end., SDA pin, SCL pin
//======================================= CONFIGURAÇÃO BUZZER ================================================================
void playTripleBeep()
{
  const int buzzerPin = 25;
  const int noteDuration = 500; // Duration of each note in milliseconds

  // Define the frequencies for different notes
  const int noteC = 261;
  const int noteD = 294;
  const int noteE = 329;

  pinMode(buzzerPin, OUTPUT);
  // Play note C
  tone(buzzerPin, noteC, noteDuration); // Remove the fourth argument
  delay(noteDuration + 100);            // Add a small delay between notes

  // Play note D
  tone(buzzerPin, noteD, noteDuration); // Remove the fourth argument
  delay(noteDuration + 100);

  // Play note E
  tone(buzzerPin, noteE, noteDuration); // Remove the fourth argument
  delay(noteDuration + 100);
  // Stop the tone
  noTone(buzzerPin);
}

void playEmergencySound()
{
  const int buzzerPin = 25;
  const int noteDuration = 300; // Duration of each note in milliseconds

  // Define the frequencies for different notes
  const int noteA = 440;
  const int noteB = 494;
  const int noteC = 523;

  pinMode(buzzerPin, OUTPUT);
  // Play note A
  tone(buzzerPin, noteA, noteDuration);
  delay(noteDuration + 100); // Add a small delay between notes

  // Play note B
  tone(buzzerPin, noteB, noteDuration);
  delay(noteDuration + 100);

  // Play note C
  tone(buzzerPin, noteC, noteDuration);
  delay(noteDuration + 100);

  // Repeat the sequence for a more urgent sound
  tone(buzzerPin, noteA, noteDuration);
  delay(noteDuration + 100);

  tone(buzzerPin, noteB, noteDuration);
  delay(noteDuration + 100);

  tone(buzzerPin, noteC, noteDuration);
  delay(noteDuration + 100);

  // Stop the tone
  noTone(buzzerPin);
}
//====================================== DS18B20 ========================================================
// DS18B20 Configuration
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
#define ONE_WIRE_BUS 17
float userTemp;
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;
// DHT Configuration
const int DHT_PIN = 17;
#define DHTTYPE DHT22
DHT dht(DHT_PIN, DHTTYPE);
float ambTemp;
// ---------- array para armazenar os dados dos sensores ----------
char sensorsDataString[50] = "";

//================================================== BOTÕES ========================================================
// Configuração de Botão de Emergência
int alertIndicator = 0;
const uint8_t buttonPin = 13; // Pino onde o botão está conectado
const uint8_t ledPin = 25;
volatile int buttonPresses = 0;              // Variável que conta as pressões do botão
volatile unsigned long lastDebounceTime = 0; // Para debouncing
const unsigned long debounceDelay = 700;     // Tempo de debounce em milissegundos

//==================================================DEFINIÇÃO DAS FUNÇÕES=======================================================

//----------------- Funções de Botão -------------------------------------
void IRAM_ATTR handleButtonPress()
{
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay)
  {
    buttonPresses++;
    lastDebounceTime = currentTime;
  }
}

void buttonConfig()
{
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);
}

void buttonCountCheck()
{
  static unsigned long lastPressTime = 0;
  const unsigned long debounceInterval = 70; // 1 second debounce interval

  if (buttonPresses > 0)
  {
    unsigned long currentTime = millis();
    if (currentTime - lastPressTime >= debounceInterval)
    {
      if (buttonPresses == 1)
      {
        if (alertIndicator == 0)
        {
          Serial.println("Emergency mode activated");
          alertIndicator = 1;
          playEmergencySound();
          // Display emergency activation message
          static unsigned long lastDisplayTime = 0;
          const unsigned long displayInterval = 2000; // Interval for displaying the message (2 seconds)
          unsigned long currentMillis = millis();

          if (currentMillis - lastDisplayTime >= displayInterval)
          {
            lastDisplayTime = currentMillis;
            display.clear();
            display.setFont(ArialMT_Plain_16);
            display.drawString(0, 0, "Emergencia");
            display.drawString(0, 16, "Ativada");
            display.display();
          }
          sendToMQTT();
        }
        else if (alertIndicator == 1)
        {
          Serial.println("Emergency mode deactivated");
          alertIndicator = 0;
          playEmergencySound();
          // Display emergency deactivation message
          static unsigned long lastDisplayTime = 0;
          const unsigned long displayInterval = 2000; // Interval for displaying the message (2 seconds)
          unsigned long currentMillis = millis();

          if (currentMillis - lastDisplayTime >= displayInterval)
          {
            lastDisplayTime = currentMillis;
            display.clear();
            display.setFont(ArialMT_Plain_16);
            display.drawString(0, 0, "Emergencia");
            display.drawString(0, 16, "Desativada");
            display.display();
          }
          if (fallRead == 1)
          {
            resetFallDetection();
          }
          sendToMQTT();
        }
        buttonPresses = 0; // Reset button presses count
      }
    }
  }
}
//========================================= Inicialização LoraMESH ================================================================
void initializeNode()
{
  Serial.print(F("initializing node "));
  Serial.print(NODE_ADDRESS);
  SPI.begin(LLG_SCK, LLG_MISO, LLG_MOSI, LLG_CS);
  if (!manager.init())
  {
    Serial.println(" init failed");
  }
  else
  {
    Serial.println(" done");
  } // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36

  rf95.setTxPower(10, false); // with false output is on PA_BOOST, power from 2 to 20 dBm, use this setting for high power demos/real usage
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
      Serial.println(F("set config failed"));
      Serial.println("RF95 ready");
      nextTxTime = millis();
    }
  }
}

void handleTransmission()
{
  // send message every TXINTERVAL millisecs
  if (millis() > nextTxTime)
  {
    nextTxTime += TXINTERVAL;
    Serial.print("Sending to bridge n.");
    Serial.print(BRIDGE_ADDRESS);
    Serial.print(" res=");

    // Send a message to a rf95_mesh_server
    // A route to the destination will be automatically discovered.
    res = manager.sendtoWait(data1, sizeof(data1), BRIDGE_ADDRESS);
    Serial.println(res);
    if (res == RH_ROUTER_ERROR_NONE)
    {
      // Data has been reliably delivered to the next node.
      // now we do...
    }
    else
    {
      // Data not delivered to the next node.
      Serial.println("sendtoWait failed. Are the bridge/intermediate mesh nodes running?");
    }
  }

  // radio needs to stay always in receive mode ( to process/forward messages )
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAck(buf, &len, &from))
  {
    Serial.print("message from node n.");
    Serial.print(from);
    Serial.print(": ");
    Serial.print((char *)buf);
    Serial.print(" rssi: ");
    Serial.println(rf95.lastRssi());
  }
}

//-------------------------------------------------SETUP---------------------------------------------------

//--------------Setup do sensor DS18B20-------------------
void ds18b20Setup()
{
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  Serial.println("Dallas Temperature IC Control Library Demo");
  sensors.begin();
}
//--------------Setup do sensor DHT22-------------------
void dhtSetup()
{
  pinMode(DHT_PIN, INPUT);
  dht.begin();
}
//==================================================Leituras de Sensores=======================================================
//--------------Leitura do sensor DS18B20-------------------
float leituraSensorDs18b20()
{
  unsigned long startTime = millis();
  while (millis() - startTime < 2000)
  { /*Não faça nada durante 2 Segundos*/
  }
  sensors.requestTemperatures();
  userTemp = sensors.getTempCByIndex(0);

  return userTemp;
  // Serial.print("User Temperature: ");
  // Serial.println(userTemp);
}
//---------------Leitura do sensor DHT22--------------------
float leituraSensorDHT22()
{
  unsigned long startTime = millis();
  while (millis() - startTime < 2000)
  { /*Não faça nada durante 2 Segundos*/
  }
  ambTemp = dht.readTemperature();
  // Serial.print(ambTemp);
  if (isnan(ambTemp))
  {
    Serial.println("Falha ao ler o sensor DHT22!");
    return 0;
  }
  return ambTemp;
}
//====================================== MAX30105 ========================================================
//--------------Setup do sensor MAX30102-------------------
void MAX30102Setup()
{
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57)) // Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
  }

  byte ledBrightness = 30; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;    // Options: 69, 118, 215, 411
  int adcRange = 16384;    // Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}
void leituraMAX30102()
{
  const unsigned long readingTime = 10000; // Tempo definido para a leitura (10 segundos)
  unsigned long startTime = millis();      // Armazena o tempo de início da leitura
  bufferLength = 100;                      // buffer length of 100 stores 4 seconds of samples running at 25sps

  // read the first 100 samples, and determine the signal range
  for (byte i = 0; i < bufferLength; i++)
  {
    while (particleSensor.available() == false)
    {
      break;
    }
    // do we have new data?
    particleSensor.check(); // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  // calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continuously taking samples from MAX30102 for a limited time (10 seconds)
  while (millis() - startTime < readingTime)
  {
    // dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (int i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    // take 25 sets of samples before calculating the heart rate.
    for (int i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) // do we have new data?
        particleSensor.check();                   // Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); // We're finished with this sample so move to next sample

      // send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }
    // After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    /*
    if (heartRateRead < 60){
      heartRateRead = 60;
    }

     // Ajusta a frequência cardíaca baseada no movimento
    if (movimentoDetectado)
    {
      heartRate += BPM_INCREMENT; // Aumenta os batimentos conforme o movimento
    }
    else
    {
      heartRate -= BPM_DECREMENT; // Diminui os batimentos sem movimento
    }

    // Aplica os limites mínimos
    if (heartRate < MIN_HEART_RATE)
    {
      heartRate = MIN_HEART_RATE;
    }

    if (spo2 < MIN_SPO2)
    {
      spo2 = MIN_SPO2;
    }
    */

    // Ajusta os valores finais (leve redução)
    heartRateRead = heartRate * 0.85;
    spo2Read = spo2 * 0.98;

    // Imprime os valores ajustados
    Serial.print(F("HR ajustado="));
    Serial.println(heartRateRead);
    Serial.print(F("SPO2 ajustado="));
    Serial.println(spo2Read);
  }
}

//====================================== MPU6050 ========================================================

TaskHandle_t Task1;
TaskHandle_t Task2;

void MPU6050Setup() // Código de Setup MPU6050  - Inicialização, Definições de frequencias e limites de medidas
{
  // mpuWire.begin();
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin(0x68, &Wire1))
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      Serial.println("Failed to find MPU6050 chip");
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("MPU6050 Setup Complete!");
  delay(100);
}
void IRAM_ATTR handleFallInterrupt()
{
  fallRead = 1; // Set the fallRead variable to 1 to indicate a fall
  // Indicate fall detection via Serial and OLED display
  Serial.println("Fall detected!");

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Queda");
  display.drawString(0, 16, "Detectada!");
  display.display();

  // Play an emergency sound to indicate fall detection
  playEmergencySound();
}

// Função que verifica o movimento
void analyzeMovement()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelerationMoveThreshold = 1.5;

  // Lê as acelerações e rotações
  float totalAcceleration = sqrtf((a.acceleration.x +
                                   a.acceleration.y +
                                   a.acceleration.z));

  float rotationTreshold = 1;
  // Calculate the magnitude of the rotation vector
  float totalRotation = sqrtf(g.gyro.x * g.gyro.x +
                              g.gyro.y * g.gyro.y +
                              g.gyro.z * g.gyro.z);

  // Verifica se a aceleração total excede o limite para detectar quedas

  if (totalRotation > rotationTreshold)
  {
    movimentoDetectado = true; // Movimento detectado
  }
  else
  {
    movimentoDetectado = false; // Sem movimento significativo
  }

  if (heartRateRead < 60)
  {
    heartRateRead = 60;
  }

  // Ajusta a frequência cardíaca baseada no movimento
  if (movimentoDetectado)
  {
    heartRate += BPM_INCREMENT; // Aumenta os batimentos conforme o movimento
  }
  else
  {
    heartRate -= BPM_DECREMENT; // Diminui os batimentos sem movimento
  }

  // Aplica os limites mínimos
  if (heartRate < MIN_HEART_RATE)
  {
    heartRate = MIN_HEART_RATE;
  }

  if (spo2 < MIN_SPO2)
  {
    spo2 = MIN_SPO2;
  }
}

/**/
void leituraMPU6050()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Check if acceleration exceeds the threshold for detecting falls
  float accelerationThreshold = 16; // Adjust this threshold as needed
  float rotationThreshold = 5.5;    // Adjust this threshold as needed
  float accelerationMoveThreshold = 3.7;

  // Calculate the magnitude of the acceleration vector
  float totalAcceleration = sqrtf(a.acceleration.x * a.acceleration.x +
                                  a.acceleration.y * a.acceleration.y +
                                  a.acceleration.z * a.acceleration.z);

  // Calculate the magnitude of the rotation vector
  float totalRotation = sqrtf(g.gyro.x * g.gyro.x +
                              g.gyro.y * g.gyro.y +
                              g.gyro.z * g.gyro.z);
  float totalAccelerationMove = sqrtf(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z);

  // Check if the total acceleration or total rotation exceeds the threshold
  if ((totalAcceleration > accelerationThreshold || totalRotation > rotationThreshold) && a.acceleration.y < 7)
  {
    handleFallInterrupt(); // Trigger the fall interrupt
  }

  if (fallRead == 1 && a.acceleration.y > 7.0)
  {
    // Mostra mensagem de "Queda Desativada" no display OLED
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Queda");
    display.drawString(0, 16, "Desativada!");
    display.display();
    fallRead = 0; // Reseta a variável fallRead
  }
  if (a.acceleration.x > accelerationMoveThreshold || a.acceleration.z > accelerationMoveThreshold)
  {
    heartRateRead += BPM_INCREMENT; // Increase heart rate due to movement
    if (heartRateRead > 182)
    {
      heartRateRead = 182; // Cap the heart rate at 200
    }
  }
  else
  {
    heartRateRead -= BPM_DECREMENT; // Decrease heart rate without significant movement
    if (heartRateRead < MIN_HEART_RATE)
    {
      heartRateRead = MIN_HEART_RATE; // Ensure heart rate does not go below minimum
    }
  }
  
}
void resetFallDetection()
{
  fallRead = 0; // Reset the fallRead variable
  Serial.println("Fall detection reset.");

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Queda");
  display.drawString(16, 0, "Resetada");
  display.display();
}

void Task1code(void *parameter)
{
  for (;;)
  {
    analyzeMovement();
    leituraMPU6050();
    buttonCountCheck();
    vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 50 milliseconds
  }
}

//==========================================================================================================================

void convertString(float sensorDHT, float sensorDS18B20, float bpsData, float oximData, int emergence, int fall, int moduleID)
{
  static unsigned long lastSendTime = 0;
  const unsigned long sendInterval = 1500; // Intervalo de envio (1.5 segundos)

  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= sendInterval)
  {
    lastSendTime = currentMillis;

    char buffer[10];
    // Serial.println(moduleID);
    // Converte o ID do módulo para string
    sprintf(buffer, "%d", moduleID);
    strcpy(sensorsDataString, buffer);
    strcat(sensorsDataString, ",");

    dtostrf(sensorDHT, 6, 2, buffer);
    strcat(sensorsDataString, buffer);
    strcat(sensorsDataString, ","); // Adiciona uma vírgula como separador

    dtostrf(sensorDS18B20, 6, 2, buffer);
    strcat(sensorsDataString, buffer);
    strcat(sensorsDataString, ",");

    dtostrf(bpsData, 6, 2, buffer);
    strcat(sensorsDataString, buffer);
    strcat(sensorsDataString, ",");

    dtostrf(oximData, 6, 2, buffer);
    strcat(sensorsDataString, buffer);
    strcat(sensorsDataString, ",");

    sprintf(buffer, "%d", emergence);
    strcat(sensorsDataString, buffer);
    strcat(sensorsDataString, ",");

    sprintf(buffer, "%d", fall);
    strcat(sensorsDataString, buffer);
    strcat(sensorsDataString, ",");

    // Agora que a string está pronta, envie-a usando LoRa Mesh
    strcpy((char *)data, sensorsDataString); // Copia a string para o buffer de transmissão

    uint8_t len = sizeof(data); // Comprimento da string

    // Envia a string via LoRa Mesh
    uint8_t result = manager.sendtoWait(data, sizeof(data), BRIDGE_ADDRESS); // Increase timeout to 5000 ms

    if (result == 0)
    {
      Serial.println("Envio bem-sucedido");
    }
    else
    {
      Serial.println("Falha ao enviar MESH");
    }

    // Imprime o array de char no Oled
    Serial.println(sensorsDataString);
  }
}

const char *ssid = "Gigo 2.4G";
// const char *ssid = "FIAP-IBM";
// const char *ssid = "iphone bia (2)";
//  const char *ssid = "A51 de Bruno";
const char *password = "18253122Ro";
//  const char *ssid = "Astro";
// const char *password = "Challenge@24!";
// const char *password = "fjho4363";
//  const char *password = "julivanlindo";
// const char *mqtt_server = "192.168.15.9";
const char *mqtt_server = "test.mosquitto.org";
// const char *mqtt_server = "192.168.70.103";
// const char *mqtt_server = "18.118.162.89";
// const char *mqtt_server = "192.168.179.103";
const char *mqtt_topic1 = "meshguard/id3/triangulation";
const char *mqtt_topic2 = "meshguard/id3/data";
//------------------------------------------------ POSTANDO DADOS NO MQTT ---------------------------------------------------

void setupMQTT()
{
  client.setServer(mqtt_server, 1883);
}

void reconnectMQTT()
{
  static unsigned long lastAttemptTime = 0;
  const unsigned long attemptInterval = 5000;
  const unsigned long maxAttempts = 2; // Maximum attempts before giving up (2 attempts * 5 seconds = 10 seconds)
  static unsigned long attemptCount = 0;

  if (!client.connected())
  {
    unsigned long currentMillis = millis();
    if (currentMillis - lastAttemptTime >= attemptInterval)
    {
      lastAttemptTime = currentMillis;
      attemptCount++;
      Serial.print("Attempting MQTT connection...");
      if (client.connect("ESP32Clientmeshguard"))
      {
        Serial.println("connected");
        attemptCount = 0; // Reset attempt count on successful connection
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        if (attemptCount >= maxAttempts)
        {
          Serial.println("Max connection attempts reached. Giving up.");
          return; // Exit the function if max attempts are reached
        }
      }
    }
  }
}

void sendToMQTT()
{
  static unsigned long lastPublishTime = 0;
  const unsigned long publishInterval = 5000; // Interval for publishing to MQTT (5 seconds)

  unsigned long currentMillis = millis();
  if (currentMillis - lastPublishTime >= publishInterval)
  {
    lastPublishTime = currentMillis;
    client.loop();
    if (client.publish(mqtt_topic2, sensorsDataString))
    {
      Serial.println("Publicado no tópico 2 com sucesso");
    }
    else
    {
      Serial.println("Falha ao publicar no tópico 2");
    }
  }
}

const int httpsPort = 443;
// Chave de API do Google Maps
const char *apiKey = ""; // Insira sua chave de API do Google Maps
String mqttMsg;

WiFiClientSecure clientS;
void allTriangulation()
{
  yield();
  clientS.setInsecure(); // Desativa a verificação SSL
  static unsigned long previousMillis = 0;
  const long interval = 10000;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    // Verifica se já está conectado ao WiFi
    if (WiFi.status() == WL_DISCONNECTED)
    {
      Serial.println("WiFi desconectado. Tentando conectar...");

      // Conecta ao WiFi
      WiFi.begin(ssid, password);
      Serial.print("Conectando ao WiFi...");
      unsigned long wifiConnectStart = millis();
      while (WiFi.status() != WL_CONNECTED)
      {
        if (millis() - wifiConnectStart >= 30000)
        {
          Serial.println("Falha ao conectar ao WiFi");
          return;
        }
        unsigned long startMillis = millis();
        while (millis() - startMillis < 100)
        {
          // Do nothing, just wait for 100 milliseconds
        }
        Serial.print(".");
      }
      Serial.println("Conectado!");
    }
    // Display step: Starting WiFi scan
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Starting WiFi scan...");
    display.display();

    // Inicia a varredura de redes Wi-Fi
    Serial.println("Iniciando varredura de redes Wi-Fi...");
    int numNetworks = WiFi.scanNetworks();
    Serial.print("Número de redes encontradas: ");
    Serial.println(numNetworks);

    // Display step: WiFi scan completed
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "WiFi scan completed");
    display.drawString(0, 10, "Networks found: " + String(numNetworks));
    display.display();

    StaticJsonDocument<1024> doc;
    JsonArray wifiAccessPoints = doc["wifiAccessPoints"].to<JsonArray>();

    for (int i = 0; i < numNetworks; i++)
    {
      JsonObject ap = wifiAccessPoints.add<JsonObject>();
      ap["macAddress"] = WiFi.BSSIDstr(i);
      ap["signalStrength"] = WiFi.RSSI(i);
    }

    String json;
    serializeJson(doc, json);

    // Display step: Sending HTTP request
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Sending HTTP request...");
    display.display();

    Serial.println("Iniciando requisição HTTP...");
    HTTPClient http;
    String url = "https://www.googleapis.com/geolocation/v1/geolocate?key=" + String(apiKey);
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(json);

    if (httpCode > 0)
    {
      String payload = http.getString();
      Serial.println("Resposta da API:");
      Serial.println(payload);

      // Display step: HTTP request successful
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 0, "HTTP request successful");
      display.display();

      StaticJsonDocument<512> locationDoc;
      deserializeJson(locationDoc, payload);
      float lat = locationDoc["location"]["lat"];
      float lng = locationDoc["location"]["lng"];
      Serial.print("Latitude: ");
      Serial.println(lat, 6);
      Serial.print("Longitude: ");
      Serial.println(lng, 6);

      // Display step: Location received
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 0, "Location received");
      display.drawString(0, 10, "Lat: " + String(lat, 6));
      display.drawString(0, 20, "Lng: " + String(lng, 6));
      display.display();

      // Preparar o payload para o nó worldmap do Node-RED
      StaticJsonDocument<256> mqttPayload;
      mqttPayload["lat"] = lat;
      mqttPayload["lon"] = lng;
      mqttPayload["name"] = Nome;
      mqttPayload["ID"] = NODE_ADDRESS;

      // Criando o payload para os sensores
      DynamicJsonDocument jsonDoc(1024);
      jsonDoc["ID"] = NODE_ADDRESS;
      jsonDoc["temperaturaDHT"] = ambTemp;
      jsonDoc["temperaturaDS18B20"] = userTemp;
      jsonDoc["bpsData"] = heartRateRead;
      jsonDoc["spo2"] = spo2Read;
      jsonDoc["emergenceButton"] = alertIndicator;
      jsonDoc["emergenceFall"] = fallRead;

      String jsonString;
      serializeJson(jsonDoc, jsonString);

      // Serializar os dados de localização
      String mqttMsg;
      serializeJson(mqttPayload, mqttMsg);

      // Verificar a conexão MQTT antes de publicar
      Serial.println("Verificando conexão MQTT...");
      reconnectMQTT();
      client.loop();

      // Display step: Publishing to MQTT
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 0, "Publishing to MQTT...");
      display.display();

      // Publicar no primeiro tópico (localização)
      Serial.println("Publicando no tópico 1...");
      if (client.publish(mqtt_topic1, mqttMsg.c_str()))
      {
        Serial.println("Publicado no tópico 1 com sucesso");

        // Display step: MQTT publish successful
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, "MQTT publish successful");
        display.display();
      }
      else
      {
        Serial.println("Falha ao publicar no tópico 1");

        // Display step: MQTT publish failed
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, "MQTT publish failed");
        display.display();
      }
    }
    else
    {
      Serial.println("Erro na requisição HTTP");

      // Display step: HTTP request failed
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 0, "HTTP request failed");
      display.display();
    }
    http.end();
  }
}

// Função para aparição de logo "MeshGuard"
void meshGuard_Logo()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);
  // Mantém o título por mais 2 segundos
  display.drawString(0, 22, "MeshGuard");
  display.display();
  delay(4000);

  // Limpa a tela
  display.clear();
  display.display();
}
void displaySetup()
{
  display.init();
  display.setFont(ArialMT_Plain_10);
}
void mostrarIdentific()
{
  display.init();
  display.clear();
  display.display();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  // Display ID and Name
  display.drawString(0, 0, "ID:");
  display.drawString(20, 0, String(NODE_ADDRESS));
  display.drawString(35, 0, Nome);
  display.display();

  // Display Function
  display.drawString(0, 10, "Função:");
  display.drawString(45, 10, "Técnico Mecânico");
  display.display();

  // Display Blood Type
  display.drawString(0, 20, "Tipo Sanguíneo:");
  display.drawString(85, 20, "O+");
  display.display();

  // Display WiFi Status
  display.drawString(0, 30, "WiFi:");
  if (WiFi.status() == WL_CONNECTED)
  {
    display.drawString(70, 30, "Connected");
  }
  else
  {
    display.drawString(70, 30, "Disconnected");
  }
  display.display();

  // Display IP Address
  display.drawString(0, 40, "IP:");
  display.drawString(20, 40, WiFi.localIP().toString());
  display.display();
}

bool verificarMensagensLoRa()
{
  // Verificar se há mensagens LoRa recebidas
  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAck(buf, &len, &from))
  {
    Serial.print("Mensagem LoRa recebida de: ");
    Serial.print(from, DEC);
    Serial.print(": ");
    Serial.println((char *)buf);
    return true;
  }
  return false;
}

void enviarMensagemLoRa()
{
  static unsigned long lastSendTime = 0;
  const unsigned long sendInterval = 5000; // Intervalo de envio (5 segundos)

  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= sendInterval)
  {
    lastSendTime = currentMillis;

    // Mensagem a ser enviada
    uint8_t data[RH_MESH_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    memcpy(data, buf, len);
    // Endereço do nó de destino
    uint8_t to = 1; // Ajuste conforme necessário

    // Enviar a mensagem
    if (manager.sendtoWait(data, sizeof(data), to) == RH_ROUTER_ERROR_NONE)
    {
      Serial.println("Mensagem enviada com sucesso");
    }
    else
    {
      Serial.println("Falha ao enviar a mensagem");
    }
  }
}

void Task2Code(void *parameter)
{
  for (;;)
  {
    if (verificarMensagensLoRa())
    {
      enviarMensagemLoRa();
    }
    vTaskDelay(pdMS_TO_TICKS(5500)); // Delay for 100 milliseconds
  }
}

void connectToWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Conectando ao WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    unsigned long startMillis = millis();
    while (millis() - startMillis < 500)
    {
      // Do nothing, just wait for 500 milliseconds
    }
    Serial.print(".");
  }
  Serial.println("Conectado!");
}

// ================================================== OTAdrive ==================================================
// Função para verificar atualizações de firmware

// OTAdrive APIkey for this product
#define APIKEY "fd8ec5de-8a96-43e2-8147-60010ee861dd"
// this app version
#define FW_VER "v@1.0.0"

// Função de callback para atualização de progresso

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

//==================================================== SETUP ====================================================
void setup()
{
  Serial.begin(115200);
  // digitalWrite(15, HIGH);
  playTripleBeep();
  // Inicializar o primeiro barramento I2C para MAX30105 e OLED
  // Initialize the first I2C bus for MAX30105 and OLED
  Wire.begin(SDA1, SCL1);
  // Inicializar o segundo barramento I2C
  Wire1.begin(SDA2, SCL2);
  // I2C_MPU6050.begin(21, 22, I2C_SPEED_STANDARD); // SDA, SCL, 100kHz
  displaySetup();
  meshGuard_Logo();
  connectToWiFi();
  checkForUpdates();
  initializeNode();
  buttonConfig();
  setupMQTT();
  //  Inicialização de sensores:
  ds18b20Setup();
  dhtSetup();
  MAX30102Setup();
  MPU6050Setup();
  // Inicialização do giroscópio no nucleo 2
  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1",   /* Name of the task */
      10000,     /* Stack size in words */
      NULL,      /* Task input parameter */
      5,         /* Priority of the task */
      &Task1,    /* Task handle. */
      1);        /* Core where the task should run */
  xTaskCreatePinnedToCore(
      Task2Code, /* Function to implement the task */
      "Task2",   /* Name of the task */
      10000,     /* Stack size in words */
      NULL,      /* Task input parameter */
      2,         /* Priority of the task */
      &Task2,    /* Task handle. */
      1);        /* Core where the task should run */
}

void loop()
{
  static unsigned long previousMillisIdentific = 0;     // Check if it's time to show identification
  static unsigned long previousMillisConvertString = 0; // Check if it's time to convert and send sensor data
  static unsigned long previousMillisDHT = 0;
  static unsigned long previousMillisDS18B20 = 0;
  static unsigned long previousMillisMAX30102 = 0;
  static unsigned long previousMillisTriangulation = 0;
  static unsigned long previousMillisLoRa = 0;
  static unsigned long previousMillisMQTT = 0; // Add previousMillis for MQTT

  const unsigned long intervalIdentific = 60000;     // Interval for showing identification (60 seconds)
  const unsigned long intervalConvertString = 2000;  // Interval for converting and sending sensor data (5 seconds)
  const unsigned long intervalDHT = 2000;            // Interval for DHT22 readings (2 seconds)
  const unsigned long intervalDS18B20 = 2000;        // Interval for DS18B20 readings (2 seconds)
  const unsigned long intervalMAX30102 = 30000;      // Interval for MAX30102 readings (30 seconds)
  const unsigned long intervalLoRa = 2000;           // Interval for LoRa message checks (5 seconds)
  const unsigned long intervalTriangulation = 90000; // Interval for triangulation (90 seconds)
  const unsigned long intervalMQTT = 7000;           // Interval for sending MQTT (5 seconds)

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisIdentific >= intervalIdentific)
  {
    previousMillisIdentific = currentMillis;
    mostrarIdentific();
  }

  if (currentMillis - previousMillisConvertString >= intervalConvertString)
  {
    previousMillisConvertString = currentMillis;
    convertString(ambTemp, userTemp, heartRateRead, spo2Read, alertIndicator, fallRead, NODE_ADDRESS);
  }

  // Check if it's time to read from DHT22
  if (currentMillis - previousMillisDHT >= intervalDHT)
  {
    previousMillisDHT = currentMillis;
    leituraSensorDHT22();
  }

  // Check if it's time to read from DS18B20
  if (currentMillis - previousMillisDS18B20 >= intervalDS18B20)
  {
    previousMillisDS18B20 = currentMillis;
    leituraSensorDs18b20();
  }

  // Check if it's time to read from MAX30102
  if (currentMillis - previousMillisMAX30102 >= intervalMAX30102)
  {
    previousMillisMAX30102 = currentMillis;
    if (particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57))
    {
      // leituraMAX30102();
    }
    else
    {
      Serial.println(F("MAX30105 was not found. Skipping leituraMAX30102."));
    }
  }

  // Check if it's time to perform triangulation
  if (currentMillis - previousMillisTriangulation >= intervalTriangulation)
  {
    previousMillisTriangulation = currentMillis;
    allTriangulation();
  }
  // Check if it's time to send MQTT
  if (currentMillis - previousMillisMQTT >= intervalMQTT)
  {
    previousMillisMQTT = currentMillis;
    sendToMQTT();
  }

  // Check if it's time to handle fall detection
  if (fallRead == 1)
  {
    static unsigned long previousMillisFall = 0;
    const unsigned long intervalFall = 2000; // Interval for handling fall detection (2 seconds)

    if (currentMillis - previousMillisFall >= intervalFall)
    {
      previousMillisFall = currentMillis;
      playEmergencySound();
    }
  }
}
