#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_SSD1306.h> //HTV3 OLED drivers
#include <cmath>
#include <cstring> // std::memcpy

#include "FS.h"
#include <LittleFS.h>
#include <tinycbor.h>
#include <driver/i2s.h>
// #include <I2S.h>

/* - -----     ENABLE/DISABLE FEATURES ----- - */
// #define LCD_4ELECRO
// #define TOUCH_4ELECRO
#define MICROPHONE
#define SPEAKER
#define GPS
#define BATTERY
#define HT3_OLED

#ifdef SPEAKER

#define SPEAKER_I2S_PORT I2S_NUM_1
//=====SPEAKER :  AUDIO OUT
#define SPEAKER_I2S_DOUT 48
#define SPEAKER_I2S_BCLK 47
#define SPEAKER_I2S_LRC 7

const int devices_sample_rate = 8000;

i2s_pin_config_t speaker_i2spins = {
    .bck_io_num = SPEAKER_I2S_BCLK,
    .ws_io_num = SPEAKER_I2S_LRC,
    .data_out_num = SPEAKER_I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE};

i2s_config_t speaker_i2sconfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = devices_sample_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    //.intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false};

#endif

#ifdef LCD_4ELECRO
// Elecro 4.0 TFT needs adafruit_st7789 or TFT_eSPI (configured for ST7789 driver)
// the chipset is an ST7789
// resolution is 480x320
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();
#endif

#ifdef TOUCH_4ELECRO
#include <XPT2046_Touchscreen.h>
// make an SPI for the TS
#define TS_MOSI 4
#define TS_MISO 2
#define TS_SCK 3
#define TS_CS 5

SPIClass ts_SPI = SPIClass(HSPI);
XPT2046_Touchscreen ts(TS_CS);
extern XPT2046_Touchscreen ts;
#endif

#ifdef MICROPHONE //=====MICROPHONE :  AUDIO IN

#define MIC_I2S_PORT I2S_NUM_0
#define MIC_I2S_DOUT 4
#define MIC_I2S_BCLK 5
#define MIC_I2S_LRC 3
#define MIC_CS 2
#include <codec2.h>            //In the codec2 folder in the library folder
#include <ButterworthFilter.h> //In the codec2 folder in the library folder
#include <FastAudioFIFO.h>     //In the codec2 folder in the library folder

i2s_config_t mic_i2sconfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = devices_sample_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    //.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false};

i2s_pin_config_t mic_i2spins = {
    .bck_io_num = MIC_I2S_BCLK,
    .ws_io_num = MIC_I2S_LRC,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = MIC_I2S_DOUT};

#endif

#ifdef HT3_OLED
// Heltec OLED Pins
#define oled_scl 18
#define oled_sda 17
#define oled_rst 21
#define VEXT_PIN 36 // external voltage, needed to send power to the OLED display
#endif

#define LED_Orange 1 // the orange LED seems to have a mind of it's own...
#define LED_W 35
#define BOOT_BTN 0

// LoRa module (SX1262) pins
#define LoRa_MOSI 10
#define LoRa_MISO 11
#define LoRa_SCK 9
#define LoRa_nss 8
#define LoRa_dio1 14
#define LoRa_nrst 12
#define LoRa_busy 13

#define LoRa_en_Tx 15
#define LoRa_en_Rx 16

uint8_t codec_buffer[196];
const int audio_buffer_size = 128;
int32_t locally_saved_recording[audio_buffer_size * 256]; // enough for ~2 seconds of voice samples at 8KHz samplerate

#ifdef HT3_OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C // Heltec default

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    oled_rst);
#endif

enum Topic : uint8_t
{
  CONTROL = 0,
  TELEMETRY = 1,
  TRANSFER = 2,
  MESSAGE = 3,
  VOICE = 4,
  VIDEO = 5,
  IMAGE = 6,
  LOCATION = 7
};

struct LoRaPacket
{
  uint8_t senderID;     // 1 byte, unique per transmitter
  Topic topicID;        // 1 byte, or more if many topics
  uint8_t targetID;     // 1 byte, unique per target
  uint8_t payload[192]; // adjustable to stay under 200 bytes total
};

// an audio filter
ButterworthFilter hp_filter(240, 8000, ButterworthFilter::ButterworthFilter::Highpass, 1);

SX1262 radio = new Module(LoRa_nss, LoRa_dio1, LoRa_nrst, LoRa_busy);
String oled_message = "OLED Test!";

// function to generate a random string
String random_string(size_t length)
{
  static const char chars[] =
      "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

  String s;
  for (int i = 0; i < length; i++)
  {
    s += chars[random(0, sizeof(chars) - 1)];
  }

  return s;
}

void SendLoRaMessage(String msg)
{
  int result = radio.transmit(msg);

  if (result == RADIOLIB_ERR_NONE)
  { // the packet was successfully transmitted
    Serial.println(F("Success!"));
  }
  else if (result == RADIOLIB_ERR_PACKET_TOO_LONG)
  { // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));
  }
  else if (result == RADIOLIB_ERR_TX_TIMEOUT)
  { // timeout occured while transmitting packet
    Serial.println(F("timeout!"));
  }
  else
  { // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(result);
  }
}

void PutOnScreen(String msg, bool clear)
{
  if (clear)
  {
    oled_message = msg;
  }
  else
  {
    oled_message = oled_message + msg;
  }
}

void RecieveLoRaMessage()
{
  // PutOnScreen("Awaiting message...", false);

  // you can receive data as an Arduino String
  // NOTE: receive() is a blocking method!
  //       See example ReceiveInterrupt for details
  //       on non-blocking reception method.
  String str;
  int state = radio.receive(str);

  // you can also receive data as byte array
  /*
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);
  */

  if (state == RADIOLIB_ERR_NONE)
  {
    // packet was successfully received
    Serial.println(F("success!"));

    // print the data of the packet
    Serial.print(F("[SX1262] Data:\t\t"));
    Serial.println(str);
    PutOnScreen("Recieved: " + str, false);

    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[SX1262] RSSI:\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("[SX1262] SNR:\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));
  }
  else if (state == RADIOLIB_ERR_RX_TIMEOUT)
  {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH)
  {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));
  }
  else
  {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void setup()
{
  Serial.begin(115200);
  // MakeSineWave(locally_saved_recording, sizeof(locally_saved_recording));

#ifdef SPEAKER
  i2s_driver_install(SPEAKER_I2S_PORT, &speaker_i2sconfig, 0, NULL);
  i2s_set_pin(SPEAKER_I2S_PORT, &speaker_i2spins);
#endif

#ifdef MICROPHONE
  i2s_driver_install(MIC_I2S_PORT, &mic_i2sconfig, 0, NULL);
  i2s_set_pin(MIC_I2S_PORT, &mic_i2spins);
#endif

#ifdef LCD_4ELECRO
  Serial.println("Initializing TFT...");
  tft.begin();
  tft.setRotation(0);
  delay(100);
  tft.fillScreen(TFT_BLACK);
  // tft.fillRect(100,100,200,200,TFT_GREEN);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  Serial.println("TFT Initialized!");
#endif

#ifdef TOUCH_4ELECRO
  ts_SPI.begin(TS_SCK, TS_MISO, TS_MOSI, TS_CS);
  ts.begin(ts_SPI);
  ts.setRotation(2);
#endif

  pinMode(MIC_CS, OUTPUT);
  digitalWrite(MIC_CS, HIGH);
  pinMode(LED_W, OUTPUT);
  pinMode(LED_Orange, OUTPUT);
  pinMode(BOOT_BTN, INPUT_PULLUP);
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW); // LOW = ON

#ifdef HT3_OLED
  // Initialize HT3 OLED
  Wire.begin(oled_sda, oled_scl);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    while (true)
      ;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("OLED Initialized");
  display.display();
#endif

  // Initialize LoRa
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_nss);
  pinMode(LoRa_en_Rx, OUTPUT);
  pinMode(LoRa_en_Tx, OUTPUT);

  int radio_state = radio.begin(
      915.0, // frequency MHz
      125.0, // bandwidth kHz
      7,     // spreading factor
      5,     // coding rate (4/5)
      0x34,  // sync word
      22,    // TX power dBm
      8      // preamble length
  );

  delay(100);
  if (radio_state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("Initialized LoRa radio!"));
  }
  else
  {
    Serial.print(F("failed to initialize LoRa radio: " + radio_state));
  }

  // this is codec2 compression stuff
  TinyCBOR.init();
  TinyCBOR.Encoder.init(codec_buffer, sizeof(codec_buffer));

  Serial.println("setup complete!");
  display.println("Setup complete!");
}

void PlayRecording() // play the sample on the speaker
{
  Serial.print("PlayRecording: -- START --");
#ifdef SPEAKER

  // size_t bytes_to_write = sizeof(locally_saved_recording);
  size_t bytes_to_write = audio_buffer_size * 4;
  int loops = sizeof(locally_saved_recording) / audio_buffer_size;
  size_t bytes_written;
  for (int l = 0; l <= loops - 1; l++)
  {
    esp_err_t err = i2s_write(SPEAKER_I2S_PORT, &locally_saved_recording[l * audio_buffer_size], bytes_to_write, &bytes_written, portMAX_DELAY);
    Serial.print("loop: " + String(l) + ", PlayRecording() just wrote: " + String(bytes_written) + " bytes \n");
    if (err != ESP_OK)
    {
      Serial.print("An error occured while trying to send audio to speaker: " + err);
    }

    // wait the amount of time that this amount of samples should take to play?
    // delay(audio_buffer_size / 16);
  }

#endif
  Serial.print("PlayRecording: -- FINISHED --");
  return;
}

size_t recording_index = 0;

bool feedback = false;
int32_t GetAudioSamples(int32_t *buffer, size_t bytes_to_get, bool give_level = false)
{
  int32_t temp_buffer[bytes_to_get];
  size_t bytesRead;
  i2s_read(MIC_I2S_PORT, temp_buffer, bytes_to_get, &bytesRead, portMAX_DELAY);
  Serial.print("i2s_read just read " + String(bytesRead) + " bytes \n");
  std::memcpy(buffer, temp_buffer, bytesRead);

  /*
  if (feedback)
  {
    size_t bytes_written;
    esp_err_t err = i2s_write(SPEAKER_I2S_PORT, buffer, bytesRead, &bytes_written, portMAX_DELAY);
    Serial.print("i2s_write just wrote " + String(bytes_written) + " bytes \n");
    if (err != ESP_OK)
    {
      Serial.print("An error occured while trying to send audio to speaker: " + err);
    }
  }
  */

  /*
  if (give_level)
  {
    int32_t samples_average_value = 0;
    for (size_t i = 0; i < bytes_to_get * 0.25; i++)
    {
      samples_average_value += abs(temp_buffer[i] >> 14);
    }
    samples_average_value /= audio_buffer_size;

    return samples_average_value;
  }
  else
  {
    return bytesRead;
  }*/

  return bytesRead;
}

/*
void RecordAudio(size_t *start_index, int32_t *buffer, size_t buffer_size)
{
  int number_of_samples = 80;
  int32_t samples[number_of_samples]; // samples are coming at a rate of 8000 per second, so 80 is 0.01s
  size_t bytesWritten;

  i2s_read(MIC_I2S_PORT, samples, sizeof(samples), &bytesWritten, 40);

  for (size_t i = 0; i < number_of_samples; i++) // transfer the samples into a locally saved recording array
  {
    if (start_index + i >= buffer_size) // prevent overflow of the recording array
    {
      start_index = 0 - i; // don't go out of range of the array
    }
    else
    {
      buffer[start_index + i] = samples[i];
    }
  }

  start_index += number_of_samples;
}
  */

float Get_Mic_Level()
{
  int32_t samples[audio_buffer_size]; // 32bit ints which means 4x8 4bytes data per int/sample
  size_t bytesRead;

  i2s_read(MIC_I2S_PORT, samples, audio_buffer_size, &bytesRead, portMAX_DELAY);

  Serial.println(bytesRead);

  float level = 0;
  for (int i = 0; i < audio_buffer_size; i++)
  {
    level += abs(samples[i] >> 8);
  }
  level *= 0.001f;
  level = level / audio_buffer_size;
  return level;
}

void ChangeChannel(int chan)
{ // Available range: 862 – 960 MHz (true?)
  /* Channel map
  0	915.2
  1	915.4
  2	915.6
  3	915.8
  4	916.0
  5	916.2
  ...etc
  */

  // Change frequency
  radio.setFrequency(915.2 + (chan * 0.2));
}

uint8_t *encodePayload(uint8_t senderID, uint8_t topicID, uint8_t targetID, String message)
{

  // TinyCBOR.Encoder.create_map(3);            // 3 key-value pairs
  // TinyCBOR.Encoder.encode_text_stringz("sid");
  // TinyCBOR.Encoder.encode_uint(senderID);

  TinyCBOR.Encoder.create_array(4);
  TinyCBOR.Encoder.encode_uint(senderID);
  TinyCBOR.Encoder.encode_uint(topicID);
  TinyCBOR.Encoder.encode_uint(targetID);
  TinyCBOR.Encoder.encode_text_stringz(message.c_str());

  TinyCBOR.Encoder.close_container();

  return codec_buffer;
}

void decodePayload(uint8_t *payload)
{
}

void sendRandomStringLoRaMsg()
{
  String new_message = random_string(6);
  SendLoRaMessage("Message-" + new_message);
  PutOnScreen("Sent: " + new_message, false);
}

#ifdef LCD_4ELECRO
void drawRandomLine()
{
  int random_x1 = random(0, TFT_WIDTH);
  int random_y1 = random(0, TFT_HEIGHT);
  int random_x2 = random(0, TFT_WIDTH);
  int random_y2 = random(0, TFT_HEIGHT);
  tft.drawLine(random_x1, random_y1, random_x2, random_y2, TFT_GREEN);
}
#endif

#ifdef TOUCH_4ELECRO
TS_Point calibratedPoint(TS_Point p)
{
  TS_Point returnPoint;

  float x_mult = 0.088f;
  float y_mult = 0.13f;
  int x_offset = -20;
  int y_offset = -20;

  returnPoint.x = (p.x * x_mult) + x_offset;
  returnPoint.y = (p.y * y_mult) + y_offset;

  return returnPoint;
}
#endif

// -- MAIN LOOP --
bool blink = false;
bool transmitting = false;
bool draw_waveform = false;
void loop()
{
#ifdef LCD_4ELECRO
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
#endif

#ifdef TOUCH_4ELECRO
  boolean istouched = ts.touched();
  if (istouched)
  {
    // tft.print("a");
    TS_Point p = calibratedPoint(ts.getPoint());
    // tft.print( p.x );
    tft.fillCircle(p.x, p.y, 5, TFT_WHITE);
    tft.drawCircle(p.x, p.y, 50, TFT_WHITE);
  }
#endif

  // float mic_in = Get_Mic_Level();
  // oled_message = mic_in;
  display.clearDisplay();

  // int32_t level_display = GetAudioSamples(&locally_saved_recording[recording_index], 40, false);
  // oled_message = level_display;
  // display.fillCircle(SCREEN_WIDTH * 0.5, SCREEN_HEIGHT * 0.5, ((level_display - 20000) * 0.03f), SSD1306_WHITE);

  if (digitalRead(BOOT_BTN) == LOW)
  {
    // PRG button pressed
    if (!transmitting)
    {
      transmitting = true;
      recording_index = 0;
    }
  }
  else
  {
    if (transmitting) // we WERE just transmitting, so play the samples back once
    {
      oled_message = "Playing >>>";
      display.clearDisplay();
      display.drawRect(28, 4, 100, 60, SSD1306_WHITE);
      display.display();
      transmitting = false;
      recording_index = 0;
      if (!feedback)
      {
        if (draw_waveform)
        {
          // draw the recorded wave (or at least a part of the front of it anyway)

          int segment_width = audio_buffer_size * 64 / 128;
          for (int i = 0; i < 128; i++)
          {
            int32_t ave = 0;
            for (int x = 0; x < segment_width; x++)
            {
              // FIXME: we need to check that this wont go beyond the array bounds!!!
              ave += abs(locally_saved_recording[(i * segment_width) + x] >> 15);
            }
            ave /= segment_width;
            display.drawLine(i, 32 - ave, i, segment_width + ave, SSD1306_WHITE);
          }
          display.display();
        }
        // actually send the audio to the speaker
        PlayRecording();
      }

      oled_message = "Finished playing [_]";
    }

    // RecieveLoRaMessage();
  }

  if (transmitting) // record the MIC_IN adc (should be talking)
  {
    oled_message = "REC... " + (String)recording_index;
    display.drawLine(0, 32, (recording_index / audio_buffer_size * 128) / 128, 32, SSD1306_WHITE);
    display.display();

    int32_t ave_value = GetAudioSamples(&locally_saved_recording[recording_index], audio_buffer_size * 4);
    if (!feedback)
    {
      recording_index += audio_buffer_size;
      if (recording_index >= sizeof(locally_saved_recording))
      {
        recording_index -= sizeof(locally_saved_recording);
      }
    }

    // float level = Get_Mic_Level();
    // display.setCursor(0, 20);
    // display.print(ave_value);
    // display.fillCircle(64, 32, ave_value, SSD1306_WHITE);

    // compress the recorded waveform using Codec2
  }

  // display.setCursor(0, 0);
  // display.print(oled_message);
  if (blink)
  {
    display.fillCircle(124, 60, 2, SSD1306_WHITE);
  }
  display.display();

  blink = !blink;
}