#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <driver/adc.h>

// Wifi Credentials
const char *ssid = "Coworking-Rosales";
const char *password = "MoniPepe_4327";

// Server connection parameters
const char *server_address = "192.168.1.13";
const uint16_t port = 3030;

// Configuración de tasa de muestreo
const int sample_rate = 16000;                                             // Tasa de muestreo en Hz
const int samples_per_buffer = 256;                                        // Número de muestras por buffer
const int buffer_duration_us = 1000000 * samples_per_buffer / sample_rate; // Duración de un buffer en microsegundos

volatile bool audio_buffer_flag = false;
bool debug = true;

uint8_t audio_buffer[samples_per_buffer];
uint8_t transmit_buffer[samples_per_buffer];

WiFiClient client; // Cliente de Wi-Fi

// Ldo regulator pin
const int ldoPin = 21;

// Functions
void connectWifi(void);
void connectServer(void);
float filter(float input);
void adc_task(void *parameter);

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
  // Configuring ADC Parameters
  adc1_config_width(ADC_WIDTH_BIT_13);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11); // ADC 1 channel 0 GPIO3

  // Crear tarea de lectura de ADC
  xTaskCreatePinnedToCore(
      adc_task,   // Función de la tarea
      "adc_task", // Nombre de la tarea
      4000,       // Tamaño de la pila de la tarea (en palabras)
      NULL,       // Parámetros de la tarea
      1,          // Prioridad de la tarea
      NULL,       // Identificador de la tarea (no es necesario en este caso)
      0           // Núcleo de la CPU en el que se ejecutará la tarea (0 o 1)
  );
  delay(1000); // wait for task to start

  // Iniit Server connection
  connectServer();
}

void loop()
{
  // // Enviar los datos al servidor cada vez que el buffer DMA esté listo
  // if (audio_buffer_flag)
  // {
  //   client.write(transmit_buffer, sizeof(transmit_buffer));
  //   client.flush();
  //   audio_buffer_flag = false;
  // }
}

// Función para el hilo de lectura de ADC
void adc_task(void *parameter)
{
  while (1)
  {
    // Leer datos del ADC
    for (int i = 0; i < samples_per_buffer; i++)
    {

      // Without filter
      // audio_buffer[i] = adc1_get_raw(ADC1_CHANNEL_2) << (16 - 13);

      // With filter
       int adcVal = adc1_get_raw(ADC1_CHANNEL_2);
       float voltage = adcVal * 3.3 / 8192.0; // convert ADC value to voltage
       float filtered_voltage = filter(voltage);
       uint16_t value = map(filtered_voltage * 1023.0 / 3.3, 0, 1023, 0, 1023); // mapping to 8 bits
       audio_buffer[i] = value; // storing the value
    }

    // Indicar que el buffer DMA está listo
    audio_buffer_flag = true;
    //  memcpy(transmit_buffer, audio_buffer, samples_per_buffer); // transfers the buffer

    if (audio_buffer_flag)
    {
      client.write(transmit_buffer, sizeof(transmit_buffer));
      client.flush();
      audio_buffer_flag = false;
    }
    // Esperar hasta que sea tiempo de leer de nuevo el ADC
    vTaskDelay(buffer_duration_us / portTICK_PERIOD_MS);
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
  while (!client.connect(server_address, port))
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
