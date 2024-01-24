#include <Arduino.h>

#include "BluetoothA2DPSink.h"

BluetoothA2DPSink a2dp_sink;
extern const uint8_t audio_ble_wav[];
extern const unsigned int audio_ble_wav_len;

void onBluetoothConnect(esp_a2d_connection_state_t state, void *)
{
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED)
  {
    Serial.println("Bluetooth connected");
    a2dp_sink.set_i2s_active(true);
    a2dp_sink.write_audio((uint8_t *)audio_ble_wav, audio_ble_wav_len / 2);
  }

  if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
  {
    Serial.println("Bluetooth disconnected");
    a2dp_sink.set_i2s_active(true);
    a2dp_sink.write_audio((uint8_t *)audio_ble_wav, audio_ble_wav_len / 2);
  }

  // Serial.println("Connection State Changed");
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
  a2dp_sink.start("--MyMusic");
  a2dp_sink.set_on_connection_state_changed(onBluetoothConnect);
  a2dp_sink.set_i2s_active(true);
  a2dp_sink.write_audio((uint8_t *)audio_ble_wav, audio_ble_wav_len / 2);
}

void loop()
{
  delay(1000); // do nothing
}
