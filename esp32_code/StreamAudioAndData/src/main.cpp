#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <driver/adc.h>

// Define Audio
#define AUDIO_BUFFER_MAX 2205
uint8_t audioBuffer[AUDIO_BUFFER_MAX];
uint8_t transmitBuffer[AUDIO_BUFFER_MAX];
uint32_t bufferPointer = 0;

// Timer Interrupt Parameters
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Wifi Credentials
const char *ssid = "Coworking-Rosales";
const char *password = "MoniPepe_4327";

// Server connection parameters
const char *server = "192.168.1.13";
const uint16_t port = 3030;

// Ldo regulator pin
const int ldoPin = 21;

// Define client object
WiFiClient client;

// Boolean variables for flags
bool transmitNow = false;
bool debug = true;

// Functions
void IRAM_ATTR onTimer(void);
void sendAudioData(bool transmitFlag);
void connectWifi(void);
void connectServer(void);
void IRAM_ATTR onTimer();
void sendAudioData(bool *transmitFlag);
void Audiofilter(float input);

void setup()
{
  // Initialize Debug interface
  if (debug)
    Serial.begin(115200);
  // Turn on second LDO regulator for microphone
  pinMode(ldoPin, OUTPUT);
  digitalWrite(ldoPin, HIGH);
  // Init Wifi
  connectWifi();
  // Iniit Server connection
  connectServer();
  // Configuring ADC Parameters
  adc1_config_width(ADC_WIDTH_BIT_13);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11); // ADC 1 channel 0 GPIO3
  // Configuring timer interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 125, true);
  timerAlarmEnable(timer);
}

void loop()
{
  sendAudioData(&transmitNow);
}

void sendAudioData(bool *transmitFlag)
{
  if (*transmitFlag)
  {
    *transmitFlag = false;
    client.write(transmitBuffer, sizeof(transmitBuffer));
    client.flush();
  }
}

float filter(float input)
{
  static float x[3] = {0.0, 0.0, 0.0};
  static float y[3] = {0.0, 0.0, 0.0};
  float b[] = {0.0194, 0.0582, 0.0582, 0.0194};
  float a[] = {1, -1.7600, 1.1829, -0.2785};

  float output = b[0] * input + b[1] * x[0] + b[2] * x[1] + b[3] * x[2] - a[1] * y[0] - a[2] * y[1] - a[3] * y[2];
  x[2] = x[1];
  x[1] = x[0];
  x[0] = input;
  y[2] = y[1];
  y[1] = y[0];
  y[0] = output;

  return output;
}

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);         // to run critical code without being interrupted.

  
  int adcVal = adc1_get_raw(ADC1_CHANNEL_2); // reads the ADC value

  //Without Filter
  // uint16_t value = map(adcVal, 0, 8192, 0, 1023); // // mapping to 10 bits

  //With Filter
  float voltage = adcVal * 3.3 / 8192.0; // convert ADC value to voltage
  float filtered_voltage = filter(voltage);
  uint16_t value = map(filtered_voltage * 1023.0 / 3.3, 0, 1023, 0, 1023); // mapping to 8 bits

  audioBuffer[bufferPointer] = value; // storing the value
  bufferPointer++;

  // Action on buffer filling
  if (bufferPointer == AUDIO_BUFFER_MAX)
  {
    bufferPointer = 0;
    memcpy(transmitBuffer, audioBuffer, AUDIO_BUFFER_MAX); // transfers the buffer
    transmitNow = true;                                    //  flag for buffer transmission
  }
  portEXIT_CRITICAL_ISR(&timerMux); // priority in critical code
}

void connectWifi(void)
{
  while (WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin(ssid, password);
    delay(1000);
    if (debug)
    {
      Serial.println("Connecting to WiFi");
    }
  }
  if (debug)
  {
    Serial.println("Connected to WiFi");
  }
}

void connectServer(void)
{
  while (!client.connect(server, port))
  {
    if (debug)
    {
      Serial.println("Connecting to Server");
    }
  }
  if (debug)
  {
    Serial.println("Connected to server");
  }
}
