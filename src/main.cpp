#include <Arduino.h>

#include "BluetoothA2DPSink.h"

BluetoothA2DPSink a2dp_sink;
extern const uint8_t PROGMEM audio_ble_wav[];
extern const uint8_t PROGMEM no_101_wav[];
extern const unsigned int audio_ble_wav_len;
extern const unsigned int no_101_wav_len;

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
  i2s_driver_uninstall((i2s_port_t)0);
  i2s_config_t tmp = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 22050,
    .bits_per_sample = (i2s_bits_per_sample_t)16,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    .tx_desc_auto_clear = true, // avoiding noise in case of data unavailability
    .fixed_mclk = 0,
    .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
    .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
#else
    .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
#endif
  };

  a2dp_sink.set_i2s_config(tmp);
  a2dp_sink.init_i2s();

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

  tmp = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = (i2s_bits_per_sample_t)16,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    .tx_desc_auto_clear = true, // avoiding noise in case of data unavailability
    .fixed_mclk = 0,
    .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
    .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
#else
    .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
#endif
  };
  i2s_driver_uninstall((i2s_port_t)0);
  a2dp_sink.set_i2s_config(tmp);
  a2dp_sink.init_i2s();
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
  }
}

void onBluetoothConnect(esp_a2d_connection_state_t state, void *)
{
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED)
  {
    Serial.println("Bluetooth connected");
    // a2dp_sink.set_i2s_active(true);
    // a2dp_sink.set_volume(60);

    i2s_driver_uninstall((i2s_port_t)0);
    i2s_config_t tmp = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 22050,
      .bits_per_sample = (i2s_bits_per_sample_t)16,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      .tx_desc_auto_clear = true, // avoiding noise in case of data unavailability
      .fixed_mclk = 0,
      .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
      .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
#else
      .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
#endif
    };

    // i2s_driver_install((i2s_port_t)0, &tmp, 0, NULL);
    a2dp_sink.set_i2s_config(tmp);
    a2dp_sink.init_i2s();

    // for (unsigned int i = 0; i < no_101_wav_len; i++)
    // {
    //   audio_ble_wav_ram[i] = pgm_read_byte(&no_101_wav[i]);
    // }
    // a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav_ram, no_101_wav_len);

    // readSound(audio_ble_wav, audio_ble_wav_len);

    // const unsigned int chunkSize = 60000;
    // uint8_t audio_ble_wav_ram[chunkSize];

    // unsigned int totalLength = audio_ble_wav_len; // La longueur totale de audio_ble_wav
    // unsigned int offset = 0;              // Décalage actuel dans audio_ble_wav

    // while (totalLength > 0)
    // {
    //   unsigned int currentChunkSize = min(chunkSize, totalLength); // Taille du chunk actuel
    //   for (unsigned int i = 0; i < currentChunkSize; i++)
    //   {
    //     audio_ble_wav_ram[i] = pgm_read_byte(&audio_ble_wav[offset + i]);
    //   }
    //   a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav_ram, currentChunkSize);

    //   offset += currentChunkSize;      // Augmenter le décalage pour le prochain chunk
    //   totalLength -= currentChunkSize; // Réduire la longueur totale restante
    // }

    for (unsigned int i = 0; i < 60000; i++)
    {
      audio_ble_wav_ram[i] = pgm_read_byte(&audio_ble_wav[i]);
    }

    a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav_ram, 60000);

    for (unsigned int i = 0; i < audio_ble_wav_len - 60000; i++)
    {
      audio_ble_wav_ram[i] = pgm_read_byte(&audio_ble_wav[60000 + i]);
    }
    a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav_ram, audio_ble_wav_len - 60000);

    // i2s_driver_uninstall((i2s_port_t)0);
    // a2dp_sink.set_i2s_active(false);
    // a2dp_sink.set_i2s_active(true);

    tmp = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 44100,
      .bits_per_sample = (i2s_bits_per_sample_t)16,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      .tx_desc_auto_clear = true, // avoiding noise in case of data unavailability
      .fixed_mclk = 0,
      .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
      .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
#else
      .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
#endif
    };
    i2s_driver_uninstall((i2s_port_t)0);
    a2dp_sink.set_i2s_config(tmp);
    a2dp_sink.init_i2s();

    // for (unsigned int i = 0; i < 60000; i++)
    // {
    //   audio_ble_wav_ram[i] = pgm_read_byte(&audio_ble_wav[i]);
    // }

    // a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav_ram, 60000);

    // for (unsigned int i = 0; i < audio_ble_wav_len - 60000; i++)
    // {
    //   audio_ble_wav_ram[i] = pgm_read_byte(&audio_ble_wav[60000 + i]);
    // }
    // a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav_ram, audio_ble_wav_len - 60000);

    // a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav, audio_ble_wav_len );
    // a2dp_sink.write_audio((uint8_t *)audio_ble_wav, audio_ble_wav_len / 2);
  }

  if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
  {
    Serial.println("Bluetooth disconnected");
    a2dp_sink.set_i2s_active(true);
    // a2dp_sink.write_audio((uint8_t *)audio_ble_wav, audio_ble_wav_len / 2);
    // a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav, audio_ble_wav_len / 2);
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
  a2dp_sink.start("--Bike Groove");
  a2dp_sink.set_on_connection_state_changed(onBluetoothConnect2);
  a2dp_sink.set_i2s_active(true);
  readSound(bike_groove_on_wav, bike_groove_on_wav_len);

  // a2dp_sink.audio_data_callback((uint8_t *)audio_ble_wav, audio_ble_wav_len / 2);
  // a2dp_sink.write_audio((uint8_t *)audio_ble_wav, audio_ble_wav_len / 2);
}

void loop()
{
  delay(1000); // do nothing
}
