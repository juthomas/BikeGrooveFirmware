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

const unsigned int chunkSize = 60000;
uint8_t audio_ble_wav_ram[chunkSize];

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

void setup()
{
  Serial.begin(115200);
  i2s_pin_config_t my_pin_config = {
      .bck_io_num = 27,
      .ws_io_num = 26,
      .data_out_num = 25,
      .data_in_num = I2S_PIN_NO_CHANGE};
  a2dp_sink.set_pin_config(my_pin_config);
  a2dp_sink.start("--Bike Groove");
  a2dp_sink.set_on_connection_state_changed(onBluetoothConnect2);
  a2dp_sink.set_i2s_active(true);
  readSound(bike_groove_on_wav, bike_groove_on_wav_len);
}

void loop()
{
  delay(1000); // do nothing
}
