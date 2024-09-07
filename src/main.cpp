#include <Arduino.h>

#include "BluetoothA2DPSink.h"

// DEBUG VAR FOR INTERRUPTS
#define DEBUG false

BluetoothA2DPSink a2dp_sink;

/* Audio Files */
extern const uint8_t PROGMEM bike_groove_on_wav[];
extern const uint32_t bike_groove_on_wav_len;

extern const uint8_t PROGMEM bike_groove_off_wav[];
extern const uint32_t bike_groove_off_wav_len;

extern const uint8_t PROGMEM bluetooth_connected_wav[];
extern const uint32_t bluetooth_connected_wav_len;

extern const uint8_t PROGMEM bluetooth_disconnected_wav[];
extern const uint32_t bluetooth_disconnected_wav_len;

/* Audio Tmp */
const uint32_t chunkSize = 60000;
uint8_t audio_ble_wav_ram[chunkSize];

/* Bluetooth autoconnect variable (May be useless) */
bool connected = true;

/* Volume modifier (0-14) */
int16_t current_volume = 14;

/* ESP Status */
volatile bool isAwake = false;

/* Buttons Commands Requests */
uint32_t commandRequest = 0;

/* Buttons Utilities */
volatile bool isStartPressed = false;
volatile uint32_t startPressedTime = 0;

volatile bool isDownPressed = false;
volatile uint32_t downPressedTime = 0;

volatile bool isUpPressed = false;
volatile uint32_t upPressedTime = 0;

volatile bool soundPlayed = false;
/* Buttons Requests types */
enum requests
{
  START_REQUEST = 1,
  NEXT_REQUEST = 1 << 1,
  PREVIOUS_REQUEST = 1 << 2,
  UP_REQUEST = 1 << 3,
  DOWN_REQUEST = 1 << 4,
};

/* Buttons IO Attribution */
#define START_BUTTON GPIO_NUM_33
#define DOWN_BUTTON 22
#define UP_BUTTON 23

/* Leds IO Attribution */
#define RED_LED 18
#define BLUE_LED 19

/* I2S IO Attribution */
#define I2S_ENABLE 17
#define I2S_BCLK 27
#define I2S_LRCLK 26
#define I2S_DIN 25

/* Probe IO Attribution */
#define VOLTAGE_PROBE 13
#define USB_VOLTAGE_PROBE 32

void readSound(const uint8_t PROGMEM sound[], uint32_t sound_len)
{
  a2dp_sink.set_i2s_active(true);

  unsigned int totalLength = sound_len; // La longueur totale de audio_ble_wav
  unsigned int offset = 0;              // Décalage actuel dans audio_ble_wav

  while (totalLength > 0)
  {
    unsigned int currentChunkSize = min(chunkSize, totalLength); // Taille du chunk actuel
    for (unsigned int i = 0; i < currentChunkSize; i++)
    {
      audio_ble_wav_ram[i] = pgm_read_byte(&sound[offset + i]);
    }
    a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav_ram, currentChunkSize);

    offset += currentChunkSize;      // Augmenter le décalage pour le prochain chunk
    totalLength -= currentChunkSize; // Réduire la longueur totale restante
  }
}

void onBluetoothConnect2(esp_a2d_connection_state_t state, void *)
{
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED)
  {
    if (DEBUG)
    {
      Serial.println("Bluetooth connected");
    }
    readSound(bluetooth_connected_wav, bluetooth_connected_wav_len);
  }

  if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
  {
    if (DEBUG)
    {
      Serial.println("Bluetooth disconnected");
    }
    readSound(bluetooth_disconnected_wav, bluetooth_disconnected_wav_len);

    // readSound(bluetooth_disconnected_wav, bluetooth_disconnected_wav_len);
  }
}

TaskHandle_t StartPressTaskHandle = NULL;
TaskHandle_t StartPressShortTaskHandle = NULL;
TaskHandle_t DownPressTaskHandle = NULL;
TaskHandle_t DownPressShortTaskHandle = NULL;
TaskHandle_t UpPressTaskHandle = NULL;
TaskHandle_t UpPressShortTaskHandle = NULL;

void StartPressTask(void *parameter)
{
  if (DEBUG)
  {
    Serial.println("Start Button Long Press Detected (Task)");
  }
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  if (DEBUG)
  {
    Serial.println("Bike Groove Off");
  }
  soundPlayed = true;
  readSound(bike_groove_off_wav, bike_groove_off_wav_len);

  vTaskDelay(1000 / portTICK_PERIOD_MS); // Attendre 1 seconde pour le message vocal

  soundPlayed = false;
  // Mettre en sommeil profond
  if (DEBUG)
  {
    Serial.println("Deep Sleep Actived");
  }
  digitalWrite(I2S_ENABLE, LOW);
  esp_deep_sleep_start();
  StartPressTaskHandle = NULL;
  vTaskDelete(NULL);
}

void StartPressShortTask(void *parameter)
{
  if (DEBUG)
  {
    Serial.printf("Start/Stop\n");
    Serial.printf("Audio State : %d\n", a2dp_sink.get_audio_state());
    // Serial.printf("Audio State : %d\n", a2dp_sink.get);
  }
  if (a2dp_sink.get_audio_state() == ESP_A2D_AUDIO_STATE_STARTED)
  {
    a2dp_sink.stop();
  }
  else
  {
    a2dp_sink.play();
  }

  // // set_on_audio_state_changed
  // if (a2dp_sink.is_i2s_active) // Fix ça
  //                              // a2dp_sink.pause();
  //   a2dp_sink.execute_avrc_command(0x46);
  // else
  //   a2dp_sink.play();
  vTaskDelete(NULL);
}

void IRAM_ATTR StartPress()
{
  if (DEBUG)
  {
    Serial.println("Start Button Pressed");
  }
  if (digitalRead(START_BUTTON) == LOW)
  {
    if (DEBUG)
    {
      Serial.println("Start Button Pushing");
    }
    isStartPressed = true;
    startPressedTime = millis();
    if (StartPressTaskHandle == NULL)
    {
      xTaskCreate(StartPressTask, "StartPressTask", 2048, NULL, 1, &StartPressTaskHandle);
    }
  }
  else if (isStartPressed == true)
  {
    isStartPressed = false;
    if (millis() - startPressedTime < 2000)
    {
      if (DEBUG)
      {
        Serial.println("Start Button Short Press");
      }
      if (StartPressTaskHandle != NULL)
      {
        vTaskDelete(StartPressTaskHandle);
        StartPressTaskHandle = NULL;
      }
      
      xTaskCreate(StartPressShortTask, "StartPressShortTask", 2048, NULL, 1, &StartPressShortTaskHandle);
      // commandRequest |= START_REQUEST;
      // timerAlarmDisable(startPressTimer);
      // Stop Timer
    }
    else
    {
      // Serial.println("Start Button Long Press");
      // digitalWrite(I2S_ENABLE, LOW);
      // esp_deep_sleep_start();
    }
  }
}

void DownPressTask(void *parameter)
{
  if (DEBUG)
  {
    Serial.println("Down Button Long Press (Task)");
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  // commandRequest |= PREVIOUS_REQUEST;
  if (DEBUG)
  {
    Serial.printf("Previous\n");
  }
  a2dp_sink.previous();
  DownPressTaskHandle = NULL;
  vTaskDelete(NULL);

}

void DownPressShortTask(void *parameter)
{
  int tmp_current_volume = (a2dp_sink.get_volume() - 15) / 8;
  tmp_current_volume = tmp_current_volume > 0 ? tmp_current_volume - 1 : tmp_current_volume;
  a2dp_sink.set_volume(tmp_current_volume * 8 + 15);
  if (DEBUG)
  {
    Serial.printf("(down) Current Volume : %d-%d\n", a2dp_sink.get_volume(), tmp_current_volume);
  }
  vTaskDelete(NULL);//Set handle to 0 ?
}

void IRAM_ATTR DownPress()
{
  if (DEBUG)
  {
    Serial.println("Down Button Pressed");
  }
  if (digitalRead(DOWN_BUTTON) == LOW)
  {
    isDownPressed = true;
    downPressedTime = millis();
    // Serial.printf("HANDLE V: %D\n", DownPressTaskHandle);
    if (DownPressTaskHandle == NULL)
    {
      xTaskCreate(DownPressTask, "DownPressTask", 2048, NULL, 1, &DownPressTaskHandle);
    }
  }
  else
  {
    isDownPressed = false;
    if (millis() - downPressedTime < 1000)
    {
      if (DEBUG)
      {
        Serial.println("Down Button Short Press");
      }
      if (DownPressTaskHandle != NULL)
      {
        vTaskDelete(DownPressTaskHandle);
        DownPressTaskHandle = NULL;
      }
      xTaskCreate(DownPressShortTask, "DownPressShortTask", 2048, NULL, 1, &DownPressShortTaskHandle);

      // commandRequest |= DOWN_REQUEST;
    }
    else
    {
      // Serial.println("Down Button Long Press");
      // commandRequest |= PREVIOUS_REQUEST;
    }
  }
}

void UpPressTask(void *parameter)
{
  if (DEBUG)
  {
    Serial.println("Up Button Long Press (Task)");
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  // commandRequest |= NEXT_REQUEST;
  if (DEBUG)
  {
    Serial.printf("Next\n");
  }
  a2dp_sink.next();
  UpPressTaskHandle = NULL;
  vTaskDelete(NULL);
}

void UpPressShortTask(void *parameter)
{
  int testwtf = a2dp_sink.get_volume();
  int tmp_current_volume = (a2dp_sink.get_volume() - 15) / 8;
  tmp_current_volume = tmp_current_volume < 14 ? tmp_current_volume + 1 : tmp_current_volume;
  a2dp_sink.set_volume(tmp_current_volume * 8 + 15);
  if (DEBUG)
  {
    Serial.printf("(up) Current Volume : %d-%d\n", a2dp_sink.get_volume(), tmp_current_volume);
  }
  vTaskDelete(NULL);
}

void IRAM_ATTR UpPress()
{
  if (DEBUG)
  {
    Serial.println("Up Button Pressed");
  }
  if (digitalRead(UP_BUTTON) == LOW)
  {
    isUpPressed = true;
    upPressedTime = millis();
    if (UpPressTaskHandle == NULL)
    {
      xTaskCreate(UpPressTask, "UpPressTask", 2048, NULL, 1, &UpPressTaskHandle);
    }
  }
  else
  {
    isUpPressed = false;
    if (millis() - upPressedTime < 1000)
    {
      if (DEBUG)
      {
        Serial.println("Up Button Short Press");
      }
      if (UpPressTaskHandle != NULL)
      {
        vTaskDelete(UpPressTaskHandle);
        UpPressTaskHandle = NULL;
      }
      xTaskCreate(UpPressShortTask, "UpPressShortTask", 2048, NULL, 1, &UpPressShortTaskHandle);

      // commandRequest |= UP_REQUEST;
    }
    else
    {
      // Serial.println("Up Button Long Press");
      // commandRequest |= NEXT_REQUEST;
    }
  }
}

void callbackaudio(esp_a2d_audio_state_t state, void* param)
{
  Serial.printf("Callback state:%d\n", state);
}

void setup()
{
  Serial.begin(115200);
  pinMode(BLUE_LED, OUTPUT); // Blue Led
  pinMode(RED_LED, OUTPUT);  // Red Led

  pinMode(VOLTAGE_PROBE, INPUT);     // Voltage Probe
  pinMode(USB_VOLTAGE_PROBE, INPUT); // USB Voltage Probe
  pinMode(START_BUTTON, INPUT);      // Power
  esp_sleep_enable_ext0_wakeup(START_BUTTON, 0);
  pinMode(DOWN_BUTTON, INPUT); // Down
  pinMode(UP_BUTTON, INPUT);   // Up

  pinMode(I2S_ENABLE, OUTPUT); // Enable I2S
  float batteryLevel = (float)analogRead(VOLTAGE_PROBE) * 7.1 / 4096;
  digitalWrite(I2S_ENABLE, batteryLevel > 3.2 ? HIGH : LOW);

  if (DEBUG)
  {
    a2dp_sink.set_on_audio_state_changed_post(&callbackaudio);
    a2dp_sink.set_on_audio_state_changed(&callbackaudio);
  }


  if (batteryLevel < 3.2)
  {
    if (DEBUG)
    {
      Serial.println("test\n");
    }
    digitalWrite(I2S_ENABLE, LOW);
    esp_deep_sleep_start();

    // for(;;);
  }
  // Vérifie si l'ESP32 a été réveillé par le bouton
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
  {
    unsigned long timePressed = millis();
    // Boucle tant que le bouton est pressé
    while (digitalRead(START_BUTTON) == LOW)
    {
      // Si le bouton est pressé pendant plus de 2 secondes
      if (millis() - timePressed > 2000)
      {
        // Sort de la boucle au bout de 2s
        break;
      }
    }

    // Si le bouton est relâché avant 2 secondes, remettre en deep sleep
    if (millis() - timePressed <= 2000)
    {
      if (DEBUG)
      {
        Serial.println("Start Button Pressed too quickly to wake up");
      }
      digitalWrite(I2S_ENABLE, LOW);
      esp_deep_sleep_start();
    }
  }

  attachInterrupt(digitalPinToInterrupt(START_BUTTON), StartPress, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DOWN_BUTTON), DownPress, CHANGE);
  attachInterrupt(digitalPinToInterrupt(UP_BUTTON), UpPress, CHANGE);

  i2s_pin_config_t my_pin_config = {
      .bck_io_num = I2S_BCLK,
      .ws_io_num = I2S_LRCLK,
      .data_out_num = I2S_DIN,
      .data_in_num = I2S_PIN_NO_CHANGE};
  a2dp_sink.set_pin_config(my_pin_config);
  a2dp_sink.set_auto_reconnect(true);
  a2dp_sink.start("Hi-Fi Ride");
  a2dp_sink.set_on_connection_state_changed(onBluetoothConnect2);
  a2dp_sink.set_volume(current_volume * 8 + 15);
  a2dp_sink.set_i2s_active(true);
  readSound(bike_groove_on_wav, bike_groove_on_wav_len);
  a2dp_sink.set_auto_reconnect(true);
}

void loop()
{
  static bool ledState = false;
  float batteryLevel = (float)analogRead(VOLTAGE_PROBE) * 7.1 / 4096;
  if (DEBUG)
  {
    Serial.printf("Probe : %fV\n", batteryLevel);
  }
  float usbVoltageLevel = (float)analogRead(USB_VOLTAGE_PROBE) * 7.1 / 4096;
  if (DEBUG)
  {
    Serial.printf("USB : %fV\n", usbVoltageLevel);
  }

  if (usbVoltageLevel > 2.5)
  {
    digitalWrite(I2S_ENABLE, LOW);
    esp_deep_sleep_start(); 
  }

  if (ledState)
  {
    digitalWrite(batteryLevel < 3.4 ? RED_LED : BLUE_LED, HIGH);
  }
  else
  {
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RED_LED, LOW);
  }

  ledState = !ledState;
  delay(500); // do nothing
}
