#include <Arduino.h>

#include "BluetoothA2DPSink.h"

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
    Serial.println("Bluetooth connected");
    readSound(bluetooth_connected_wav, bluetooth_connected_wav_len);
  }

  if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
  {
    Serial.println("Bluetooth disconnected");
    readSound(bluetooth_disconnected_wav, bluetooth_disconnected_wav_len);

    // readSound(bluetooth_disconnected_wav, bluetooth_disconnected_wav_len);
  }
}

void IRAM_ATTR StartPress()
{
  Serial.println("Start Button Pressed");
  if (digitalRead(START_BUTTON) == LOW)
  {
    Serial.println("Start Button Pushing");
    isStartPressed = true;
    startPressedTime = millis();
  }
  else if (isStartPressed == true)
  {
    isStartPressed = false;
    if (millis() - startPressedTime < 2000)
    {
      Serial.println("Start Button Short Press");
      commandRequest |= START_REQUEST;
    }
    else
    {
      Serial.println("Start Button Long Press");
      digitalWrite(I2S_ENABLE, LOW);
      esp_deep_sleep_start();
    }
  }
}

void IRAM_ATTR DownPress()
{
  Serial.println("Down Button Pressed");
  if (digitalRead(DOWN_BUTTON) == LOW)
  {
    isDownPressed = true;
    downPressedTime = millis();
  }
  else
  {
    isDownPressed = false;
    if (millis() - downPressedTime < 500)
    {
      Serial.println("Down Button Short Press");
      commandRequest |= DOWN_REQUEST;
    }
    else
    {
      Serial.println("Down Button Long Press");
      commandRequest |= PREVIOUS_REQUEST;
    }
  }
}

void IRAM_ATTR UpPress()
{
  Serial.println("Up Button Pressed");
  if (digitalRead(UP_BUTTON) == LOW)
  {
    isUpPressed = true;
    upPressedTime = millis();
  }
  else
  {
    isUpPressed = false;
    if (millis() - upPressedTime < 500)
    {
      Serial.println("Up Button Short Press");
      commandRequest |= UP_REQUEST;
    }
    else
    {
      Serial.println("Up Button Long Press");
      commandRequest |= NEXT_REQUEST;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(BLUE_LED, OUTPUT); // Blue Led
  pinMode(RED_LED, OUTPUT); // Red Led
  pinMode(I2S_ENABLE, OUTPUT); // Enable I2S
  digitalWrite(I2S_ENABLE, HIGH);
  pinMode(VOLTAGE_PROBE, INPUT); // Voltage Probe
  pinMode(START_BUTTON, INPUT); // Power
  esp_sleep_enable_ext0_wakeup(START_BUTTON, 0);
  pinMode(DOWN_BUTTON, INPUT); // Down
  pinMode(UP_BUTTON, INPUT); // Up

  // Vérifie si l'ESP32 a été réveillé par le bouton
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    unsigned long timePressed = millis();
    // Boucle tant que le bouton est pressé
    while(digitalRead(START_BUTTON) == LOW) {
      // Si le bouton est pressé pendant plus de 2 secondes
      // if (millis() - timePressed > 2000) {
      //   // Placez ici le code à exécuter après 2 secondes de pression
      //   break;
      // }
    }

    // Si le bouton est relâché avant 2 secondes, remettre en deep sleep
    if (millis() - timePressed <= 2000) {
      Serial.println("Start Button Pressed too quickly to wake up");
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
  a2dp_sink.start("--Bike Groove 2");
  a2dp_sink.set_on_connection_state_changed(onBluetoothConnect2);
  a2dp_sink.set_i2s_active(true);
  readSound(bike_groove_on_wav, bike_groove_on_wav_len);
  a2dp_sink.set_auto_reconnect(true);
}

void loop()
{
  static bool ledState = false;
  float batteryLevel = (float)analogRead(13) * 7.1 / 4096;
  Serial.printf("Probe : %fV\n", batteryLevel);

  if (ledState)
  {
    digitalWrite(batteryLevel < 3.3 ? RED_LED : BLUE_LED, HIGH);
  }
  else
  {
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RED_LED, LOW);
  }

  ledState = !ledState;
  delay(500); // do nothing

  if (commandRequest & NEXT_REQUEST)
  {
    Serial.printf("Next\n");
    a2dp_sink.next();
    commandRequest = commandRequest & ~NEXT_REQUEST;
  }

  if (commandRequest & PREVIOUS_REQUEST)
  {
    Serial.printf("Previous\n");
    a2dp_sink.previous();
    commandRequest = commandRequest & ~PREVIOUS_REQUEST;
  }

  if (commandRequest & START_REQUEST)
  {
    Serial.printf("Start/Stop\n");
    if (a2dp_sink.is_i2s_active) // Fix ça
                                 // a2dp_sink.pause();
      a2dp_sink.execute_avrc_command(0x46);

    else
      a2dp_sink.play();
    commandRequest = commandRequest & ~START_REQUEST;
  }

  if (commandRequest & UP_REQUEST)
  {
    // a2dp_sink.set_volume(a2dp_sink.get_volume() + 1);
    current_volume = current_volume < 14 ? current_volume + 1 : current_volume;
    a2dp_sink.set_volume(current_volume * 8 + 15);

    Serial.printf("(up) Current Volume : %d-%d\n", a2dp_sink.get_volume(), current_volume);
    // a2dp_sink.execute_avrc_command(0x41);
    commandRequest = commandRequest & ~UP_REQUEST;
  }

  if (commandRequest & DOWN_REQUEST)
  {
    // current_volume -= 1;
    current_volume = current_volume > 0 ? current_volume - 1 : current_volume;
    a2dp_sink.set_volume(current_volume * 8 + 15);
    Serial.printf("(down) Current Volume : %d-%d\n", a2dp_sink.get_volume(), current_volume);

    commandRequest = commandRequest & ~DOWN_REQUEST;
  }
  // Serial.printf("Active 1 : %d\n",a2dp_sink.spp_active);
  // Serial.printf("Active 2 : %d\n",a2dp_sink.player_init);
  // Serial.printf("Active 3 : %d\n",a2dp_sink.audio_type);
  // Serial.printf("Active 4 : %d\n",a2dp_sink.bt_volumechange);
  // Serial.printf("Active 5 : %d\n",a2dp_sink.is_i2s_active);
  // Serial.printf("Active 6 : %d\n",a2dp_sink.rssi_active);
  // Serial.printf("Active 6 : %d\n",a2dp_sink.pl);
  // commandRequest = 0;
}
