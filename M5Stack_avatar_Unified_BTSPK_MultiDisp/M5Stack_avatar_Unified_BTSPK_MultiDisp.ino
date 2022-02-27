#include <M5UnitLCD.h>
#include <M5UnitOLED.h>
#include <M5Unified.h>
#include <driver/adc.h>
LGFX_Device* gfx2 = nullptr;

// --------------------
// サーボピンの初期設定
#if defined(ARDUINO_M5STACK_Core2)
  // M5Stack Core2用のサーボの設定
  // Port.A X:G32, Y:G33
  // Port.C X:G13, Y:G14
  // スタックチャン基板 X:G27, Y:G19
  #define SERVO_PIN_X 13
  #define SERVO_PIN_Y 14
#elif defined( ARDUINO_M5STACK_FIRE )
  // M5Stack Fireの場合はPort.A(X:G22, Y:G21)のみです。
  // I2Cと同時利用は不可
  #define SERVO_PIN_X 22
  #define SERVO_PIN_Y 21
#elif defined( ARDUINO_M5Stack_Core_ESP32 )
  // M5Stack Basic/Gray/Go用の設定
  // Port.A X:G22, Y:G21
  // Port.C X:G16, Y:G17
  // スタックチャン基板 X:G5, Y:G2
  #define SERVO_PIN_X 16
  #define SERVO_PIN_Y 17
#endif
// サーボピンの初期設定end
// --------------------


// --------------------
// Avatar関連の初期設定
#include "Avatar.h"
using namespace m5avatar;

Avatar avatar;
static int32_t lipsync_level = 0;       // リップシンク値
static float lipsync_level_max = 20.0f; // リップシンクの上限初期値
float mouth_ratio = 0.0f;
// Avatar関連の設定 end
// --------------------

// --------------------
// サーボ関連の初期設定
#include <ServoEasing.hpp>
ServoEasing servo_x;
ServoEasing servo_y;

#define START_DEGREE_VALUE_X 90
#define START_DEGREE_VALUE_Y 90
int servo_offset_x = 0;                 // X軸サーボのオフセット（90°からの+-で設定）
int servo_offset_y = 0;                 // Y軸サーボのオフセット（90°からの+-で設定）
static long interval_min = 1000;        // 待機時インターバル最小
static long interval_max = 5000;        // 待機時インターバル最大
static long move_time_min = 500;        // 待機時のサーボ移動時間最小
static long move_time_max = 1000;       // 待機時のサーボ移動時間最大
static long sing_interval_min = 1000;   // 歌うモードのサーボ移動時間最小
static long sing_interval_max = 2000;   // 歌うモードのサーボ移動時間最大
// サーボ関連の設定 end
// --------------------


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
  bool _sing_happy = true;
  size_t _sample_rate = 48000;

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
        avatar.setExpression(Expression::Neutral);
      }
      else
      if (ESP_A2D_CONNECTION_STATE_DISCONNECTED == a2d->conn_stat.state)
      { // 切断
        avatar.setExpression(Expression::Sad);
      }
      break;

    case ESP_A2D_AUDIO_STATE_EVT:
      if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state)
      { // 再生
        if (_sing_happy) {
          avatar.setExpression(Expression::Happy);
        } else {
          avatar.setExpression(Expression::Neutral);
        }
        _sing_happy = !_sing_happy;
      } else
      if ( ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND == a2d->audio_stat.state
        || ESP_A2D_AUDIO_STATE_STOPPED        == a2d->audio_stat.state )
      { // 停止
        avatar.setExpression(Expression::Sleepy);
        clearMetaData();
      }
      break;

    case ESP_A2D_AUDIO_CFG_EVT:
      {
        esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(p_param);
        size_t tmp = a2d->audio_cfg.mcc.cie.sbc[0];
        size_t rate = 16000;
        if (     tmp & (1 << 6)) { rate = 32000; }
        else if (tmp & (1 << 5)) { rate = 44100; }
        else if (tmp & (1 << 4)) { rate = 48000; }
        _sample_rate = rate;
      }
      break;

    default:
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
      do { vTaskDelay(1); } while (M5.Speaker.isPlaying(m5spk_virtual_channel) > 1);
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
    M5.Speaker.playRAW(_flip_buf[flip], length >> 1, _sample_rate, true, 1, m5spk_virtual_channel);
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


uint32_t bgcolor(LGFX_Device* gfx, int y)
{
  auto h = gfx->height();
  auto dh = h - header_height;
  int v = ((h - y)<<5) / dh;
  if (dh > 40)
  {
    int v2 = ((h - y + 1)<<5) / dh;
    if ((v >> 2) != (v2 >> 2))
    {
      return 0x666666u;
    }
  }
  return gfx->color888(v + 2, v, v + 6);
}

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
  gfx->print("BT A2DP : ");
  gfx->println(bt_device_name);
  gfx->setTextWrap(false);
  gfx->fillRect(0, 6, gfx->width(), 2, TFT_BLACK);

  header_height = 45;
  fft_enabled = !gfx->isEPD();
  if (fft_enabled)
  {
    for (int y = header_height; y < gfx->height(); ++y)
    {
      gfx->drawFastHLine(0, y, gfx->width(), bgcolor(gfx, y));
    }
  }

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
    for (int id = 0; id < a2dp_sink.metatext_num; ++id)
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
      int fft_height = dsp_height - header_height - 1;
      int xe = gfx->width() / bw;
      if (xe > (FFT_SIZE/2)) { xe = (FFT_SIZE/2); }
      int32_t lipsync_temp = 0;
      for (int x = 0; x <= xe; ++x)
      {
        if (((x * bw) & 7) == 0) { gfx->display(); }
        int32_t f = fft.get(x);
        int y = (f * fft_height) >> 18;
        if (y > fft_height) { y = fft_height; }
        y = dsp_height - y;
        int py = prev_y[x];
        if (x > 5 and x <= xe) { // 0〜63の範囲でlipsyncでピックアップしたい音域を選びます。xeは最大
          //printf("x:%d, y:%d, f:%d, lipsync_temp:%d\n", x, y, f, lipsync_temp);
          lipsync_temp = lipsync_temp + f;
        }
        if (y != py)
        {
//          gfx->fillRect(x * bw, y, bw - 1, py - y, (y < py) ? 0x99AAFFu : 0x000033u);
            if (x > 5 and x <= xe) { // 0〜63の範囲でlipsyncでピックアップしたい音域を選びます。xeは最大
              gfx->fillRect(x * bw, y, bw - 1, py - y, (y < py) ? 0x00F800u : 0x000033u);
            } else {
              gfx->fillRect(x * bw, y, bw - 1, py - y, (y < py) ? 0x99AAFFu : 0x000033u);
            }
          prev_y[x] = y;
        }
        py = peak_y[x] + 1;
        if (py < y)
        {
          gfx->writeFastHLine(x * bw, py - 1, bw - 1, bgcolor(gfx, py - 1));
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
      // ログは下記をコメントアウトしてください。
      //printf("data_temp=%d\n",lipsync_temp);
      lipsync_level = lipsync_temp >> 16; // リップシンクを設定
      //printf("data=%d\n",lipsync_level);
      gfx->display();
      gfx->endWrite();
    }
  }
}

void moveX(int x, uint32_t millis_for_move = 0) {
  if (millis_for_move == 0) {
    servo_x.easeTo(x + servo_offset_x);
  } else {
    servo_x.easeToD(x + servo_offset_x, millis_for_move);
  }
}

void moveY(int y, uint32_t millis_for_move = 0) {
  if (millis_for_move == 0) {
    servo_y.easeTo(y + servo_offset_y);
  } else {
    servo_y.easeToD(y + servo_offset_y, millis_for_move);
  }
}

void moveXY(int x, int y, uint32_t millis_for_move = 0) {
  if (millis_for_move == 0) {
    servo_x.setEaseTo(x + servo_offset_x);
    servo_y.setEaseTo(y + servo_offset_y);
  } else {
    servo_x.setEaseToD(x + servo_offset_x, millis_for_move);
    servo_y.setEaseToD(y + servo_offset_y, millis_for_move);
  }
  // サーボが停止するまでウェイトします。
  synchronizeAllServosStartAndWaitForAllServosToStop();
}

void servoLoop(void *args) {
  long move_time = 0;
  long interval_time = 0;
  long move_x = 0;
  long move_y = 0;
  float gaze_x = 0.0f;
  float gaze_y = 0.0f;
  bool sing_mode = false;
  for (;;) {
    if (mouth_ratio == 0.0f) {
      // 待機時の動き
      interval_time = random(interval_min, interval_max);
      move_time = random(move_time_min, move_time_max);
      lipsync_level_max = 20.0f; // リップシンク上限の初期化
      sing_mode = false;

    } else {
      // 歌うモードの動き
      interval_time = 1;
      move_time = random(sing_interval_min, sing_interval_max);
      sing_mode = true;
    } 
    avatar.getGaze(&gaze_y, &gaze_x);
    
//    Serial.printf("x:%f:y:%f\n", gaze_x, gaze_y);
    // X軸は90°から+-で左右にスイング
    if (gaze_x < 0) {
      move_x = START_DEGREE_VALUE_X - mouth_ratio * 15 + (int)(15.0 * gaze_x);
    } else {
      move_x = START_DEGREE_VALUE_X + mouth_ratio * 15 + (int)(15.0 * gaze_x);
    }
    // Y軸は90°から上にスイング（最大35°）
    move_y = START_DEGREE_VALUE_Y - mouth_ratio * 15 - abs(20.0 * gaze_y);
    moveXY(move_x, move_y, move_time);
    if (sing_mode) {
      // 歌っているときはうなずく
      moveXY(move_x, move_y + 10, 400);
    }
    vTaskDelay(interval_time/portTICK_PERIOD_MS);

  }
}


void lipSync(void *args)
{
  float gazeX, gazeY;
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
   for (;;)
  {
    // スレッド内でログを出そうとすると不具合が起きる場合があります。
    //printf("data=%d\n\r",lipsync_level);
    lipsync_level = abs(lipsync_level);
    float open = (float)lipsync_level/lipsync_level_max;
    mouth_ratio = open;
    if (mouth_ratio > 1.2f) {
      if (mouth_ratio > 1.5f) {
        lipsync_level_max += 10.0f; // リップシンク上限を大幅に超えるごとに上限を上げていく。
      }
      mouth_ratio = 1.2f;
    }
    avatar->setMouthOpenRatio(mouth_ratio);
    vTaskDelay(1/portTICK_PERIOD_MS);
  }
}

void avrc_metadata_callback(uint8_t data1, const uint8_t *data2)
{
  Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
}

void setup(void)
{
  auto cfg = M5.config();

  cfg.external_spk = true;    /// use external speaker (SPK HAT / ATOMIC SPK)
//cfg.external_spk_detail.omit_atomic_spk = true; // exclude ATOMIC SPK
//cfg.external_spk_detail.omit_spk_hat    = true; // exclude SPK HAT

  M5.begin(cfg);


  { /// custom setting
    auto spk_cfg = M5.Speaker.config();
    /// Increasing the sample_rate will improve the sound quality instead of increasing the CPU load.
    spk_cfg.sample_rate = 96000; // default:48000 (48kHz)  e.g. 50000 , 80000 , 96000 , 100000 , 144000 , 192000
    // spk_cfg.task_pinned_core = 0;
    // spk_cfg.task_priority = configMAX_PRIORITIES - 2;
    M5.Speaker.config(spk_cfg);
  }

  M5.Speaker.setVolume(128);

  M5.Speaker.begin();
  gfx2 = new M5UnitLCD();
  if(!gfx2->init()) {
    delete gfx2;
    gfx2 = new M5UnitOLED();
    gfx2->init();
  }
  gfxSetup(gfx2);

  if (servo_x.attach(SERVO_PIN_X, 
                     START_DEGREE_VALUE_X + servo_offset_x,
                     DEFAULT_MICROSECONDS_FOR_0_DEGREE,
                     DEFAULT_MICROSECONDS_FOR_180_DEGREE)) {
    Serial.print("Error attaching servo x");
  }
  if (servo_y.attach(SERVO_PIN_Y,
                     START_DEGREE_VALUE_Y + servo_offset_y,
                     DEFAULT_MICROSECONDS_FOR_0_DEGREE,
                     DEFAULT_MICROSECONDS_FOR_180_DEGREE)) {
    Serial.print("Error attaching servo y");
  }
  servo_x.setEasingType(EASE_QUADRATIC_IN_OUT);
  servo_y.setEasingType(EASE_QUADRATIC_IN_OUT);
  avatar.init(); // start drawing
  avatar.addTask(lipSync, "lipSync");
  avatar.addTask(servoLoop, "servoLoop");
  avatar.setExpression(Expression::Sad);

  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.start(bt_device_name, false);

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
