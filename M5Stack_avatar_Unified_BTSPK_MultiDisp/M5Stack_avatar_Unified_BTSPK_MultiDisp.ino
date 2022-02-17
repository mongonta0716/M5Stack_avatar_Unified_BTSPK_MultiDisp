#if defined ( ARDUINO )

#include <Arduino.h>

//// If you use SD card, write this.
//#include <SD.h>
//
//// If you use SPIFFS, write this.
//#include <SPIFFS.h>

#endif
#include <M5Unified.h>
#include <M5UnitOLED.h>
#include <M5UnitLCD.h>

//M5UnitOLED oled;
LGFX_Device* gfx2 = nullptr;

int16_t lipsync_level = 0;

#include "Avatar.h"
#if defined( ARDUINO_M5STACK_FIRE ) || defined(ARDUINO_M5STACK_Core2)
#define USE_ATARU_FACE
#ifdef USE_ATARU_FACE  
#include "AtaruFace.h"
#endif
#endif
using namespace m5avatar;

Avatar avatar;
#ifdef USE_ATARU_FACE  
Face* face;
ColorPalette* cp;
#endif

/// need ESP32-A2DP library. ( URL : https://github.com/pschatzmann/ESP32-A2DP/ )
#include <BluetoothA2DPSink.h>

/// set M5Speaker virtual channel (0-7)
static constexpr uint8_t m5spk_virtual_channel = 0;

/// set ESP32-A2DP device name
static constexpr char bt_device_name[] = "ESP32";


class BluetoothA2DPSink_M5Speaker : public BluetoothA2DPSink
{
public:
  BluetoothA2DPSink_M5Speaker(m5::Speaker_Class* m5sound, uint8_t virtual_channel = 0)
  : BluetoothA2DPSink()
  {
    is_i2s_output = false; // I2S control by BluetoothA2DPSink is not required.
  }

  // get rawdata buffer for FFT.
  const int16_t* getBuffer(void) const { return _flip_buf[_flip_index]; }

  const char* getMetaData(size_t id) { _meta_bits &= ~(1<<id); return (id < metatext_num) ? _meta_text[id] : nullptr; }

  uint8_t getMetaUpdateInfo(void) const { return _meta_bits; }

  void clear(void)
  {
    for (int i = 0; i < 2; ++i)
    {
      if (_flip_buf[i]) { memset(_flip_buf[i], 0, _flip_buf_size[i]); }
    }
  }

  static constexpr size_t metatext_size = 80;
  static constexpr size_t metatext_num = 3;

protected:
  int16_t* _flip_buf[2] = { nullptr, nullptr };
  size_t _flip_buf_size[2] = { 0, 0 };
  bool _flip_index = 0;
  char _meta_text[metatext_num][metatext_size];
  uint8_t _meta_bits = 0;

  void clearMetaData(void)
  {
    for (int i = 0; i < metatext_num; ++i)
    {
      _meta_text[i][0] = 0;
    }
    _meta_bits = (1<<metatext_num)-1;
  }

  void av_hdl_a2d_evt(uint16_t event, void *p_param) override
  {
    esp_a2d_cb_param_t* a2d = (esp_a2d_cb_param_t *)(p_param);

    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
      if (ESP_A2D_CONNECTION_STATE_CONNECTED == a2d->conn_stat.state)
      { // 接続
//          Serial.printf("connected\n\r");  
          avatar.setExpression(Expression::Neutral);    
      }
      else
      if (ESP_A2D_CONNECTION_STATE_DISCONNECTED == a2d->conn_stat.state)
      { // 切断
//          Serial.printf("disconnected\n\r");      
          avatar.setExpression(Expression::Sleepy);    
      }
      break;

    case ESP_A2D_AUDIO_STATE_EVT:
      if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state)
      { // 再生
        avatar.setExpression(Expression::Neutral);    
      } else
      if ( ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND == a2d->audio_stat.state
        || ESP_A2D_AUDIO_STATE_STOPPED        == a2d->audio_stat.state )
      { // 停止
        clearMetaData();
        avatar.setExpression(Expression::Doubt);    
      }
      break;
    }

    BluetoothA2DPSink::av_hdl_a2d_evt(event, p_param);
  }

  void av_hdl_avrc_evt(uint16_t event, void *p_param) override
  {
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);

    switch (event)
    {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
      for (size_t i = 0; i < metatext_num; ++i)
      {
        if (0 == (rc->meta_rsp.attr_id & (1 << i))) { continue; }
        strncpy(_meta_text[i], (char*)(rc->meta_rsp.attr_text), metatext_size);
        _meta_bits |= rc->meta_rsp.attr_id;
        break;
      }
      break;

    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
      break;

    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
      break;

    default:
      break;
    }

    BluetoothA2DPSink::av_hdl_avrc_evt(event, p_param);
  }

  void audio_data_callback(const uint8_t *data, uint32_t length) override
  {
    /// When the queue is empty or full, delay processing is performed.
    if (M5.Speaker.isPlaying(m5spk_virtual_channel) != 1)
    {
      vTaskDelay(5 / portTICK_RATE_MS);
      while (M5.Speaker.isPlaying(m5spk_virtual_channel) == 2) { taskYIELD(); }
    }
    bool flip = !_flip_index;
    if (_flip_buf_size[flip] < length)
    {
      _flip_buf_size[flip] = length;
      if (_flip_buf[flip] != nullptr) { heap_caps_free(_flip_buf[flip]); }
      auto tmp = (int16_t*)heap_caps_malloc(length, MALLOC_CAP_8BIT);
      _flip_buf[flip] = tmp;
      if (tmp == nullptr)
      {
        _flip_buf_size[flip] = 0;
        return;        
      }
    }
    memcpy(_flip_buf[flip], data, length);
    _flip_index = flip;
    M5.Speaker.playRAW(_flip_buf[flip], length >> 1, this->i2s_config.sample_rate, true, 1, m5spk_virtual_channel);
  }
};


#define FFT_SIZE 256
class fft_t
{
  float _wr[FFT_SIZE + 1];
  float _wi[FFT_SIZE + 1];
  float _fr[FFT_SIZE + 1];
  float _fi[FFT_SIZE + 1];
  uint16_t _br[FFT_SIZE + 1];
  size_t _ie;

public:
  fft_t(void)
  {
#ifndef M_PI
#define M_PI 3.141592653
#endif
    _ie = logf( (float)FFT_SIZE ) / log(2.0) + 0.5;
    static constexpr float omega = 2.0f * M_PI / FFT_SIZE;
    static constexpr int s4 = FFT_SIZE / 4;
    static constexpr int s2 = FFT_SIZE / 2;
    for ( int i = 1 ; i < s4 ; ++i)
    {
    float f = cosf(omega * i);
      _wi[s4 + i] = f;
      _wi[s4 - i] = f;
      _wr[     i] = f;
      _wr[s2 - i] = -f;
    }
    _wi[s4] = _wr[0] = 1;

    size_t je = 1;
    _br[0] = 0;
    _br[1] = FFT_SIZE / 2;
    for ( size_t i = 0 ; i < _ie - 1 ; ++i )
    {
      _br[ je << 1 ] = _br[ je ] >> 1;
      je = je << 1;
      for ( size_t j = 1 ; j < je ; ++j )
      {
        _br[je + j] = _br[je] + _br[j];
      }
    }
  }

  void exec(const int16_t* in)
  {
    memset(_fi, 0, sizeof(_fi));
    for ( size_t j = 0 ; j < FFT_SIZE / 2 ; ++j )
    {
      float basej = 0.25 * (1.0-_wr[j]);
      size_t r = FFT_SIZE - j - 1;

      /// perform han window and stereo to mono convert.
      _fr[_br[j]] = basej * (in[j * 2] + in[j * 2 + 1]);
      _fr[_br[r]] = basej * (in[r * 2] + in[r * 2 + 1]);
    }

    size_t s = 1;
    size_t i = 0;
    do
    {
      size_t ke = s;
      s <<= 1;
      size_t je = FFT_SIZE / s;
      size_t j = 0;
      do
      {
        size_t k = 0;
        do
        {
          size_t l = s * j + k;
          size_t m = ke * (2 * j + 1) + k;
          size_t p = je * k;
          float Wxmr = _fr[m] * _wr[p] + _fi[m] * _wi[p];
          float Wxmi = _fi[m] * _wr[p] - _fr[m] * _wi[p];
          _fr[m] = _fr[l] - Wxmr;
          _fi[m] = _fi[l] - Wxmi;
          _fr[l] += Wxmr;
          _fi[l] += Wxmi;
        } while ( ++k < ke) ;
      } while ( ++j < je );
    } while ( ++i < _ie );
  }

  uint32_t get(size_t index)
  {
    return (index < FFT_SIZE / 2) ? (uint32_t)sqrtf(_fr[ index ] * _fr[ index ] + _fi[ index ] * _fi[ index ]) : 0u;
  }
};


static BluetoothA2DPSink_M5Speaker a2dp_sink = { &M5.Speaker, m5spk_virtual_channel };
static fft_t fft;
static bool fft_enabled = false;
static uint16_t prev_y[(FFT_SIZE/2)+1];
static uint16_t peak_y[(FFT_SIZE/2)+1];
static int header_height = 0;


void gfxSetup(LGFX_Device* gfx)
{
  if (gfx == nullptr) { return; }
  gfx->setRotation(3);
//  if (gfx->width() < gfx->height())
//  {
//    gfx->setRotation(gfx->getRotation()^1);
//  }
  gfx->setFont(&fonts::lgfxJapanGothic_12);
  gfx->setEpdMode(epd_mode_t::epd_fastest);
  gfx->setCursor(0, 8);
  gfx->startWrite(); /// Omit the paired endWrite.
  gfx->print("BT A2DP : ");
  gfx->println(bt_device_name);
  gfx->setTextWrap(false);
  gfx->fillRect(0, 6, gfx->width(), 2, TFT_BLACK);

  header_height = 45;
  fft_enabled = !gfx->isEPD();

  for (int x = 0; x < (FFT_SIZE/2)+1; ++x)
  {
    prev_y[x] = INT16_MAX;
    peak_y[x] = INT16_MAX;
  }
}

void gfxLoop(LGFX_Device* gfx)
{
  if (gfx == nullptr) { return; }
  auto bits = a2dp_sink.getMetaUpdateInfo();
  if (bits)
  {
    gfx->startWrite();
    for (int id = 0; id < 8; ++id)
    {
      if (0 == (bits & (1<<id))) { continue; }
      gfx->setCursor(0, 8 + id * 12);
      gfx->fillRect(0, 8 + id * 12, gfx->width(), 12, gfx->getBaseColor());
      gfx->print(a2dp_sink.getMetaData(id));
      gfx->print(" "); // Garbage data removal when UTF8 characters are broken in the middle.
    }
    gfx->display();
    gfx->endWrite();
  }

  if (!gfx->displayBusy())
  { // draw volume bar
    static int px;
    uint8_t v = M5.Speaker.getChannelVolume(m5spk_virtual_channel);
    int x = v * (gfx->width()) >> 8;
    if (px != x)
    {
      gfx->fillRect(x, 6, px - x, 2, px < x ? 0xAAFFAAu : 0u);
      gfx->display();
      px = x;
    }
  }

  if (fft_enabled && !gfx->displayBusy())
  {
    static int prev_x[2];
    static int peak_x[2];
    static bool prev_conn;
    bool connected = a2dp_sink.is_connected();
    if (prev_conn != connected)
    {
      prev_conn = connected;
      if (!connected)
      {
        a2dp_sink.clear();
      }
    }

    auto data = a2dp_sink.getBuffer();
    if (data)
    {
      gfx->startWrite();

      // draw stereo level meter
       uint16_t level[2] = { 0, 0 };
      for (int i = 0; i < 512; i += 16)
      {
        uint32_t lv = abs(data[i]);
        if (level[0] < lv) { level[0] = lv; }
        lv = abs(data[i+1]);
        if (level[1] < lv) { level[1] = lv; }
      }
      for (int i = 0; i < 2; ++i)
      {
        int x = (level[i] * gfx->width() - 4) / INT16_MAX;
        int px = prev_x[i];
        if (px != x)
        {
          gfx->fillRect(x, i * 3, px - x, 2, px < x ? 0xFF9900u : 0x330000u);
          prev_x[i] = x;
        }
        px = peak_x[i];
        if (px > x)
        {
          gfx->writeFastVLine(px, i * 3, 2, TFT_BLACK);
          px--;
        }
        else
        {
          px = x;
        }
        if (peak_x[i] != px)
        {
          peak_x[i] = px;
          gfx->writeFastVLine(px, i * 3, 2, TFT_WHITE);
        }
      }
      gfx->display();

      // draw FFT level meter
      fft.exec(data);
      int bw = gfx->width() / 60;
      if (bw < 3) { bw = 3; }
      int dsp_height = gfx->height();
      int fft_height = dsp_height - header_height;
      int xe = gfx->width() / bw;
      if (xe > (FFT_SIZE/2)) { xe = (FFT_SIZE/2); }
      int32_t lipsync_temp = 0;
      for (int x = 0; x <= xe; ++x)
      {
        if (((x * bw) & 7) == 0) { gfx->display(); }
        int32_t f = fft.get(x) * fft_height;
        if (x > 0 and x < 10) { // 0〜31の範囲でlipsyncでピックアップしたい音域を選びます。
          int32_t f1 = fft.get(x) * 100;
          lipsync_temp = std::max(lipsync_temp, f1 >> 19); // 指定した範囲で最も高い音量を設定。
        }
        int y = f >> 19;
        if (y > fft_height) { y = fft_height; }
        y = dsp_height - y;
//        Serial.printf("f:%d, x:%d, xe: %d, y:%d lipsync_temp:%d\n", f >> 19, x, xe, y, lipsync_temp);
        int py = prev_y[x];
        if (y != py)
        {
          if (x > 0 and x < 10) { // 0〜31の範囲でlipsyncでピックアップしたい音域を選びます。
          gfx->fillRect(x * bw, y, bw - 1, py - y, (y < py) ? 0x00F800u : 0x000033u);
          } else {
          gfx->fillRect(x * bw, y, bw - 1, py - y, (y < py) ? 0x99AAFFu : 0x000033u);
          }
          prev_y[x] = y;
        }
        py = peak_y[x] + 1;
        if (py < y)
        {
          gfx->writeFastHLine(x * bw, py - 1, bw - 1, TFT_BLACK);
        }
        else
        {
          py = y - 1;
        }
        if (peak_y[x] != py)
        {
          peak_y[x] = py;
          gfx->writeFastHLine(x * bw, py, bw - 1, TFT_WHITE);
        }
      }
      lipsync_level = lipsync_temp; // リップシンクを設定
      gfx->display();
      gfx->endWrite();
    }
  }
}

void lipSync(void *args)
{
  float gazeX, gazeY;
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
   for (;;)
  {
    //int level = a2dp_sink.audio_level;
//    printf("data=%d\n\r",lipsync_level);
    lipsync_level = abs(lipsync_level);
//    lipsync_level *= 3;
    if(lipsync_level > 80)
    {
      lipsync_level = 80;
//      avatar->setExpression(Expression::Happy);
    }
//    else
//    {
//      avatar->setExpression(Expression::Neutral);
//    }
    float open = (float)lipsync_level/80.0;
    avatar->setMouthOpenRatio(open);
    avatar->getGaze(&gazeY, &gazeX);
    avatar->setRotation(gazeX * 10);
    delay(50);
  }
}

void setup()
{
 auto cfg = M5.config();

  cfg.external_spk = true;    /// use external speaker (SPK HAT / ATOMIC SPK)
//cfg.external_spk_detail.omit_atomic_spk = true; // exclude ATOMIC SPK
//cfg.external_spk_detail.omit_spk_hat    = true; // exclude SPK HAT

  M5.begin(cfg);


  /// Increasing the sample_rate will improve the sound quality instead of increasing the CPU load.
  auto spk_cfg = M5.Speaker.config();
//  spk_cfg.sample_rate = 125000; // default:48000 (48kHz)
  M5.Speaker.config(spk_cfg);
  M5.Speaker.begin();
  M5.Display.setBrightness(128);
  
  header_height = 48;
  for (int x = 0; x < (FFT_SIZE/2)+1; ++x)
  {
    prev_y[x] = INT16_MAX;
    peak_y[x] = INT16_MAX;
  }

  a2dp_sink.start(bt_device_name, false);

  gfx2 = new M5UnitLCD();
  if(!gfx2->init()) {
    delete gfx2;
    gfx2 = new M5UnitOLED();
    gfx2->init();
  }
  gfxSetup(gfx2);
#ifdef USE_ATARU_FACE  
  face = new AtaruFace();
  cp = new ColorPalette();
  cp->set(COLOR_PRIMARY, TFT_BLACK);
  cp->set(COLOR_BACKGROUND, TFT_WHITE);
  cp->set(COLOR_SECONDARY, TFT_WHITE);
#endif
  avatar.init(); // start drawing
#ifdef USE_ATARU_FACE  
  avatar.setFace(face);
  avatar.setColorPalette(*cp);
#endif
  avatar.setExpression(Expression::Sleepy);    
  avatar.addTask(lipSync, "lipSync");
//  xTaskCreatePinnedToCore(
//                    lipSync1,     /* Function to implement the task */
//                    "lipSync1",   /* Name of the task */
//                    1024,      /* Stack size in words */
//                    NULL,      /* Task input parameter */
//                    3,         /* Priority of the task */
//                    NULL,      /* Task handle. */
//                    0);        /* Core where the task should run */
}

void loop(void)
{
  gfxLoop(gfx2);

  {
    static int prev_frame;
    int frame;
    while (prev_frame == (frame = millis() >> 3)) /// 8 msec cycle wait
    {
      vTaskDelay(1);
    }
    prev_frame = frame;
  }

  M5.update();
  if (M5.BtnA.wasClicked())
  {
    M5.Speaker.tone(1000, 100);
    a2dp_sink.next();
  }
  if (M5.BtnA.isHolding() || M5.BtnB.isPressed() || M5.BtnC.isPressed())
  {
    size_t v = M5.Speaker.getChannelVolume(m5spk_virtual_channel);
    if (M5.BtnB.isPressed()) { --v; } else { ++v; }
    if (v <= 255 || M5.BtnA.isHolding())
    {
      M5.Speaker.setChannelVolume(m5spk_virtual_channel, v);
    }
  }
}

#if !defined ( ARDUINO )
extern "C" {
  void loopTask(void*)
  {
    setup();
    for (;;) {
      loop();
    }
    vTaskDelete(NULL);
  }

  void app_main()
  {
    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, 1);
  }
}
#endif
