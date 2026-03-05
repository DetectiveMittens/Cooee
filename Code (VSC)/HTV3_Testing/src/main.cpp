/* TODO
0. switch to 320, 8KHz samples
1. figure out how to encode our samples with codec2
2. implement 'feedback' sending the samples read each frame directly back out the speaker
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_SSD1306.h> //HTV3 OLED drivers
#include <cmath>
#include <cstring> // std::memcpy
#include "FS.h"
#include <LittleFS.h>
#include <driver/i2s.h>
#include "codec2.h"

/* - -----     ENABLE/DISABLE FEATURES ----- - */
// #define LCD_4ELECRO
// #define TOUCH_4ELECRO
#define MICROPHONE
#define SPEAKER
#define GPS
#define BATTERY
#define HT3_OLED

const int32_t devices_devices_sample_rate = 16000;
const int bytes_per_sample = 2;
const int recording_length = 64;
const int audio_buffer_size = 1024;
int16_t locally_saved_recording[audio_buffer_size * recording_length]; // enough for ~2 seconds of voice samples at 8KHz samplerate

#ifdef SPEAKER //=====SPEAKER :  AUDIO OUT

#define SPEAKER_I2S_PORT I2S_NUM_1
#define SPEAKER_I2S_DOUT 48
#define SPEAKER_I2S_BCLK 47
#define SPEAKER_I2S_LRC 7

i2s_pin_config_t speaker_i2spins = {
    .bck_io_num = SPEAKER_I2S_BCLK,
    .ws_io_num = SPEAKER_I2S_LRC,
    .data_out_num = SPEAKER_I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE};

i2s_config_t speaker_i2sconfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = devices_devices_sample_rate / 2,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    //.intr_alloc_flags = 0,
    .dma_buf_count = 2,
    .dma_buf_len = audio_buffer_size,
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
    .sample_rate = devices_devices_sample_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    //.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,               // does this just multiply the value below? whats this about...
    .dma_buf_len = audio_buffer_size, // this value is in samples!
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
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
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

//-- AUDIO ENCODING --
struct CODEC2 *codec2_state;
#define ENCODE_FRAME_BYTES 8
#define ENCODE_CODEC2_PACKET_BYTES 40 // wait for 5 x 8byte frames before sending over LoRa
unsigned char tx_encode_frame[ENCODE_CODEC2_PACKET_BYTES];
int tx_encode_frame_index = 0;

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

void SaveSamples()
{

  Serial.println("Here's all the samples: ");
  for (int16_t sample : locally_saved_recording)
  {
    Serial.print(String(sample) + ",");
  }

  /*
  File file = LittleFS.open("/hello.txt", FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }

  // write some data
  file.println("Hello ESP32");
  file.close(); // close the file – flushes the buffer

  Serial.println("Wrote /hello.txt");
  */
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

  /*
  // initialise the file‑system
  Serial.println("Attempting to mount LittleFS.");
  delay(1000);
  if (!LittleFS.begin())
  { // false = mounting failed
    Serial.println("LittleFS mount failed");
    return;
  }
  Serial.println("LittleFS mounted");
  */

  // Init codec2
  codec2_state = codec2_create(CODEC2_MODE_1600);
  codec2_set_lpc_post_filter(codec2_state, 1, 0, 0.8, 0.2);

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

  Serial.println("setup complete!");
  display.println("Setup complete!");
}

void PlayRecording() // play the sample on the speaker
{
  Serial.print("PlayRecording: -- START --");

#ifdef SPEAKER
  bool split_into_chunks = false;
  // size_t bytes_to_write = sizeof(locally_saved_recording);
  size_t bytes_to_write = audio_buffer_size * bytes_per_sample;
  int loops = sizeof(locally_saved_recording) / audio_buffer_size;
  size_t bytes_written;

  if (split_into_chunks)
  {
    for (int l = 0; l <= loops - 1; l++)
    {
      esp_err_t err = i2s_write(SPEAKER_I2S_PORT, &locally_saved_recording[l * audio_buffer_size], bytes_to_write, &bytes_written, portMAX_DELAY);
      // Serial.print("loop: " + String(l) + ", PlayRecording() just wrote: " + String(bytes_written) + " bytes \n");
      if (err != ESP_OK)
      {
        Serial.print("An error occured while trying to send audio to speaker: " + err);
      }

      // wait the amount of time that this amount of samples should take to play?
      // delay(audio_buffer_size / 16);
    }
  }
  else
  {
    // play the whole dang file
    bytes_to_write = sizeof(locally_saved_recording);

    esp_err_t err = i2s_write(SPEAKER_I2S_PORT, locally_saved_recording, bytes_to_write, &bytes_written, portMAX_DELAY);
    if (err != ESP_OK)
    {
      Serial.print("An error occured while trying to send audio to speaker: " + err);
    }
  }

#endif
  Serial.print("PlayRecording: -- FINISHED --");
  return;
}

size_t recording_index = 0;

bool feedback = false;
int16_t GetAudioSamples(int16_t *buffer, size_t samples_to_get, bool give_level = false)
{
  int16_t input_gain = 40;
  int16_t temp_buffer[samples_to_get];
  size_t bytesRead;
  i2s_read(MIC_I2S_PORT, temp_buffer, samples_to_get * bytes_per_sample, &bytesRead, portMAX_DELAY);

  int16_t samples_average_value = 0;
  for (int i = 0; i < samples_to_get; i++)
  {
    temp_buffer[i] *= input_gain;
    samples_average_value += abs(temp_buffer[i] >> 14);
  }
  samples_average_value /= samples_to_get;

  std::memcpy(buffer, temp_buffer, bytesRead); // only 'paste' in the number of bytes that were populated with samples in the buffer. temp_buffer now contains the entire buffer, populated or not, which means there will be a lot of zeroes in there!

  if (false)
  {
    // compress the recorded waveform using Codec2
    codec2_encode(codec2_state, tx_encode_frame + tx_encode_frame_index, temp_buffer);
    tx_encode_frame_index += ENCODE_FRAME_BYTES;
    if (tx_encode_frame_index >= ENCODE_CODEC2_PACKET_BYTES) // packet is full
    {
      tx_encode_frame_index = 0;
      // SendEncodedAudioOverLoRa();
    }

    // Recieve the compressed bits over LoRa
    // Decode the compressed bits
    int16_t decoded_samples[samples_to_get];
    codec2_decode(codec2_state, decoded_samples, tx_encode_frame);

    // Play Feedback
    size_t bytes_written;
    esp_err_t err = i2s_write(SPEAKER_I2S_PORT, decoded_samples, bytesRead, &bytes_written, portMAX_DELAY);
  }

  return samples_average_value;
}

float Get_Mic_Level()
{
  int16_t samples[audio_buffer_size]; // 32bit ints which means 4x8 4bytes data per int/sample
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

// ======================-- MAIN LOOP --==========================
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

  if (digitalRead(BOOT_BTN) == LOW) // PRG button pressed down
  {
    if (!transmitting)
    {
      transmitting = true;
      recording_index = 0;
    }
  }
  else // PRG button up
  {
    if (transmitting) // we WERE just transmitting, so play the samples back once
    {
      transmitting = false;
      // SaveSamples();

      oled_message = "Playing >>>";
      display.display();

      PlayRecording();

      oled_message = "Finished playing [_]";
    }

    // RecieveLoRaMessage();
  }

  if (transmitting) // record the MIC_IN adc (should be talking)
  {
    oled_message = "REC... " + (String)recording_index;
    display.drawLine(0, OLED_HEIGHT / 2, (recording_index / audio_buffer_size * recording_length) / OLED_WIDTH, OLED_HEIGHT / 2, SSD1306_WHITE);
    display.display();

    if (true)
    {
      int16_t ave_value = GetAudioSamples(&locally_saved_recording[recording_index], audio_buffer_size, true);
      recording_index += audio_buffer_size;
      if (recording_index >= (audio_buffer_size * recording_length)) // wrap around the array
      {
        recording_index -= (audio_buffer_size * recording_length);
      }
      display.setCursor(0, 20);
      display.print(ave_value);
      display.fillCircle(64, 32, ave_value * 0.001, SSD1306_WHITE);
    }
    else
    {
      Serial.println("Size of locally_saved_recording in bytes is : " + String(sizeof(locally_saved_recording)));
      Serial.println("position in recording : " + String(sizeof(locally_saved_recording)));
      Serial.println("Recording this last chunk would overflow the range of the saved sample array, skipping");
      recording_index = 0;
    }
  }

  // display.setCursor(0, 0);
  display.print(oled_message);
  if (blink)
  {
    display.fillCircle(124, 60, 2, SSD1306_WHITE);
  }
  display.display();

  blink = !blink;
}