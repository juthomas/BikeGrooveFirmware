## Pins Attribution
### Amplificators :
- Activate (High): `IO17`
- I2S :
  - bck (BCLK) : `IO27`
  - ws (LRCLK) : `IO26`
  - data (DIN) : `IO25`

### Switchs (Pull-up) :
 - Power SW1 : `IO33`
 - Down SW2  : `IO22`
 - Up SW4    : `IO23`

### Battery Probe :
- Voltage : `IO13` * 2

### Leds :
 - Blue : `IO19`
 - Red  : `IO18`
---

## Import Sound

### Audacity :

Project Rate (Bottom) : `88200`
Export `WAV` `Signed 16-bit PCM`

### WAV File Needed Specifications

Audio Channels : `Mono`
Sample Rate : `88,2 kHz`
Bits per sample : `16`
Audio Format : `Linear PCM, 16 bit little-endian signed integer, 88200 Hz`

### Convert Wav to C file

```shell
xxd -i file.wav file.c
```
---
## Library Adaptation

Removing protected attribute in `BluetoothA2DPSink.h` & `BluetoothA2DPSinkQueued.h`
Adding `NULL` param in `vRingbufferGetInfo`