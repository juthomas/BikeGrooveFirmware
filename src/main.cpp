#include <Arduino.h>

#include "BluetoothA2DPSink.h"

BluetoothA2DPSink a2dp_sink;

extern const uint8_t PROGMEM bike_groove_on_wav[];
extern const unsigned int bike_groove_on_wav_len;

extern const uint8_t PROGMEM bike_groove_off_wav[];
extern const unsigned int bike_groove_off_wav_len;

extern const uint8_t PROGMEM bluetooth_connected_wav[];
extern const unsigned int bluetooth_connected_wav_len;

extern const uint8_t PROGMEM bluetooth_disconnected_wav[];
extern const unsigned int bluetooth_disconnected_wav_len;

unsigned int commandRequest = 0;

const unsigned int chunkSize = 60000;
uint8_t audio_ble_wav_ram[chunkSize];

enum requests
{
  START_REQUEST = 1,
  NEXT_REQUEST = 1 << 1,
  PREVIOUS_REQUEST = 1 << 2,
  UP_REQUEST = 1 << 3,
  DOWN_REQUEST = 1 << 4,
};

#define START_BUTTON 33
#define DOWN_BUTTON 22
#define UP_BUTTON 23

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

volatile bool isStartPressed = false;
volatile unsigned long startPressedTime = 0;

volatile bool isDownPressed = false;
volatile unsigned long downPressedTime = 0;

volatile bool isUpPressed = false;
volatile unsigned long upPressedTime = 0;

void IRAM_ATTR StartPress()
{
  Serial.println("Start Button Pressed");
  if (digitalRead(START_BUTTON) == LOW)
  {
    isStartPressed = true;
    startPressedTime = millis();
  }
  else
  {
    isStartPressed = false;
    if (millis() - startPressedTime < 500)
    {
      Serial.println("Start Button Short Press");
      commandRequest |= START_REQUEST;
    }
    else
    {
      Serial.println("Start Button Long Press");
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
  pinMode(19, OUTPUT); // Blue Led
  pinMode(18, OUTPUT); // Red Led
  pinMode(17, OUTPUT); // Enable I2S
  digitalWrite(17, HIGH);
  pinMode(21, INPUT); // Voltage Probe
  pinMode(33, INPUT); // Power
  pinMode(22, INPUT); // Down
  pinMode(23, INPUT); // Up

  attachInterrupt(digitalPinToInterrupt(START_BUTTON), StartPress, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DOWN_BUTTON), DownPress, CHANGE);
  attachInterrupt(digitalPinToInterrupt(UP_BUTTON), UpPress, CHANGE);

  i2s_pin_config_t my_pin_config = {
      .bck_io_num = 27,
      .ws_io_num = 26,
      .data_out_num = 25,
      .data_in_num = I2S_PIN_NO_CHANGE};
  a2dp_sink.set_pin_config(my_pin_config);
  a2dp_sink.set_auto_reconnect(true);
  a2dp_sink.start("--Bike Groove");
  a2dp_sink.set_on_connection_state_changed(onBluetoothConnect2);
  a2dp_sink.set_i2s_active(true);
  readSound(bike_groove_on_wav, bike_groove_on_wav_len);
}

void loop()
{
  static bool ledState = false;
  digitalWrite(19, ledState);
  digitalWrite(18, !ledState);
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
    Serial.printf("(up) Current Volume : %d-%d\n", a2dp_sink.get_volume());
    // a2dp_sink.set_volume(a2dp_sink.get_volume() + 1);
    a2dp_sink.execute_avrc_command(0x41);
    commandRequest = commandRequest & ~UP_REQUEST;
  }

  if (commandRequest & DOWN_REQUEST)
  {
    Serial.printf("(down) Current Volume : %d\n", a2dp_sink.get_volume());
    a2dp_sink.set_volume(a2dp_sink.get_volume() - 1);
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
