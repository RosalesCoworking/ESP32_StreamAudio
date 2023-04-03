#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <driver/adc.h>
#include "MP3EncoderLAME.h"

// use dynamically precalculated log table
#define USE_FAST_LOG 0

// use precalculated log table as const -> in the ESP32 this will end up in flash memory
#define USE_FAST_LOG_CONST 1

// Avoid big memory allocations in replaygain_data
#define USE_MEMORY_HACK 1

// If you know the encoder will be used in a single threaded environment, you can use this hack to just
// recycle the memory. This will prevent memory fragmentation. Only use this if you are sure that the
// encoder will be called from a single thread.
#define USE_STACK_HACK_RECYCLE_ALLOCATION_SINGLE_THREADED 1

// If the device is ESP32 and ESP_PSRAM_ENABLE_LIMIT is > 0, then the ESP32 will
// be configured to use allocate any allocation above ESP_PSRAM_ENABLE_LIMIT using
// psram, rather than scarce main memory.
#define ESP_PSRAM_ENABLE_LIMIT 10000

// Not all microcontroller support vararg methods: alternative impelemtation of logging using the preprocessor
#define USE_LOGGING_HACK 1

// Print debug and trace messages
#define USE_DEBUG 0

// Print memory allocations and frees
#define USE_DEBUG_ALLOC 0

// The stack on microcontrollers is very limite   d - use the heap for big arrays instead of the stack! 
#define USE_STACK_HACK 1

#define AUDIO_BUFFER_MAX 512

using namespace liblame;


uint16_t audioBuffer[AUDIO_BUFFER_MAX];
uint16_t transmitBuffer[AUDIO_BUFFER_MAX];
uint32_t bufferPointer = 0;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

WiFiClient client;
const char *ssid = "Coworking-Rosales";
const char *password = "MoniPepe_4327";

const char *server = "192.168.1.13";
const uint16_t port = 3030;

// Ldo regulator pin
const int ldoPin = 21;

bool transmitNow = false; // Flag for transmit audio buffer
bool debug = true;        // Flag for debug

void IRAM_ATTR onTimer(void);
void connectWifi(void);
void connectServer(void);
void dataCallback(uint8_t *mp3_data, size_t len);
void sendAudioData(bool *transmitFlag);
void Audiofilter(float input);

MP3EncoderLAME mp3(dataCallback);
AudioInfo info;

void setup()
{
  if (debug)
    Serial.begin(115200);

  info.channels = 1;
  info.sample_rate = 16000;
  mp3.begin(info);

  // Turn on second LDO regulator for microphone
  pinMode(ldoPin, OUTPUT);
  digitalWrite(ldoPin, HIGH);

  connectWifi();
  connectServer();

  adc1_config_width(ADC_WIDTH_BIT_13);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_MAX); // ADC 1 channel 0 GPIO3

  timer = timerBegin(0, 40, true); // 40 Prescaler (for 16 kHz)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 125, true);
  timerAlarmEnable(timer);
}

void loop()
{
  sendAudioData(&transmitNow);
  // if (transmitNow)
  // {
  //   transmitNow = false;
  //   client.write((const uint8_t *)audioBuffer, sizeof(audioBuffer));
  //   client.flush();
  // }
}

void sendAudioData(bool *transmitFlag)
{
  if (transmitNow)
  {
    mp3.write(audioBuffer, AUDIO_BUFFER_MAX);
  }
  // if (*transmitFlag)
  // {
  //   *transmitFlag = false;
  //   client.write((const uint8_t *)audioBuffer, sizeof(audioBuffer));
  //   client.flush();
  // }
}

void dataCallback(uint8_t *mp3_data, size_t len)
{
  Serial.print("mp3 generated with ");
  Serial.print(len);
  Serial.println(" bytes");

  // Send AAC buffer over WiFi
  if (client.connected())
  {
    client.write(mp3_data, len);
    client.flush();
  }
  else
  {
    Serial.println("Connection lost.");
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
  portENTER_CRITICAL_ISR(&timerMux); // to run critical code without being interrupted.

  int adcVal = adc1_get_raw(ADC1_CHANNEL_2); // reads the ADC value

  // Without Filter
  // uint16_t value = map(adcVal, 0, 8192, 0, 8192); // // mapping to 10 bits

  // With Filter
  float voltage = adcVal * 3.3 / 8192.0; // convert ADC value to voltage
  float filtered_voltage = filter(voltage);
  uint16_t value = map(filtered_voltage * 8192.0 / 3.3, 0, 8192, 0, 8192); // mapping to 10 bits

  audioBuffer[bufferPointer++] = value;

  // Action on buffer filling
  if (bufferPointer == AUDIO_BUFFER_MAX)
  {
    bufferPointer = 0;
    // memcpy(transmitBuffer, audioBuffer, AUDIO_BUFFER_MAX); // transfers the buffer
    transmitNow = true; // flag for buffer transmission
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
