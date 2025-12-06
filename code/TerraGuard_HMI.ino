#include <PCA9557.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_NeoPixel.h>

// Screen-side firmware for the ESP32-S3 HMI
#define ASU_MAROON 0x8C1D40
#define ASU_GOLD 0xFFC627

class LGFX : public lgfx::LGFX_Device
{
public:
  lgfx::Bus_RGB   _bus_instance;
  lgfx::Panel_RGB _panel_instance;

  LGFX(void)
  {
    { 
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;

      cfg.pin_d0  = GPIO_NUM_15; 
      cfg.pin_d1  = GPIO_NUM_7;  
      cfg.pin_d2  = GPIO_NUM_6;  
      cfg.pin_d3  = GPIO_NUM_5;  
      cfg.pin_d4  = GPIO_NUM_4;  

      cfg.pin_d5  = GPIO_NUM_9;  
      cfg.pin_d6  = GPIO_NUM_46; 
      cfg.pin_d7  = GPIO_NUM_3;  
      cfg.pin_d8  = GPIO_NUM_8;  
      cfg.pin_d9  = GPIO_NUM_16; 
      cfg.pin_d10 = GPIO_NUM_1;  

      cfg.pin_d11 = GPIO_NUM_14; 
      cfg.pin_d12 = GPIO_NUM_21; 
      cfg.pin_d13 = GPIO_NUM_47; 
      cfg.pin_d14 = GPIO_NUM_48; 
      cfg.pin_d15 = GPIO_NUM_45; 

      cfg.pin_henable = GPIO_NUM_41;
      cfg.pin_vsync   = GPIO_NUM_40;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_0;
      cfg.freq_write  = 15000000;

      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 40;
      cfg.hsync_pulse_width = 48;
      cfg.hsync_back_porch  = 40;

      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 1;
      cfg.vsync_pulse_width = 31;
      cfg.vsync_back_porch  = 13;

      cfg.pclk_active_neg   = 1;
      cfg.de_idle_high      = 0;
      cfg.pclk_idle_high    = 0;

      _bus_instance.config(cfg);
    }

    { 
      auto cfg = _panel_instance.config();
      cfg.memory_width  = 800;
      cfg.memory_height = 480;
      cfg.panel_width   = 800;
      cfg.panel_height  = 480;
      cfg.offset_x      = 0;
      cfg.offset_y      = 0;
      _panel_instance.config(cfg);
    }

    _panel_instance.setBus(&_bus_instance);
    setPanel(&_panel_instance);
  }
};

LGFX lcd;

#define TFT_BL 2

#define LED_PIN 44
#define LED_COUNT 60
#define LED_BRIGHTNESS 255

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long ledLastToggle = 0;
bool ledState = false;
const int BLINK_INTERVAL_MS = 500; 

#include "touch.h"   

const char *WIFI_SSID = "TerraGuard-AP";
const char *WIFI_PASS = "phoenixforce";          

// rover AP endpoints
const char *METRICS_URL = "http://192.168.4.1/metrics";
const char *STREAM_HOST = "192.168.4.1";
const int   STREAM_PORT = 80;
const char *STREAM_PATH = "/stream";

const char *CONTROL_URL  = "http://192.168.4.1/control";
const char *COORDS_URL   = "http://192.168.4.1/coords";

const uint32_t METRIC_POLL_MS     = 1000;
const uint32_t WIFI_RETRY_MS      = 30000;
const uint32_t HTTP_TIMEOUT_MS    = 2000;

const uint32_t COORDS_POLL_MS     = 1000;

bool streamActive = false;
WiFiClient streamClient;
LGFX_Sprite streamSprite(&lcd);
uint8_t *jpgBuffer = NULL;
const size_t JPG_BUF_SIZE = 40000; 
lv_img_dsc_t stream_img_dsc;       

// MJPEG frames land here before being pushed into the LVGL image
static lv_obj_t *label_stream_status;
static bool      stream_error       = false;
static uint32_t  lastStreamFrameMs  = 0;
const uint32_t   STREAM_FRAME_TIMEOUT_MS = 5000; 

static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[800 * 480 / 15];
static lv_disp_drv_t disp_drv;

static lv_style_t style_card;
static lv_style_t style_badge;
static lv_style_t style_title;
static lv_style_t style_value;
static lv_style_t style_label_small;
static lv_style_t style_button;
static lv_style_t style_button_pressed;
static lv_style_t style_button_red; 

static lv_style_t style_metric_title;
static lv_style_t style_metric_value;

static lv_style_t style_conn_card;
static lv_style_t style_status_bar;
static lv_style_t style_drive_card;
static lv_style_t style_drive_btn;
static lv_style_t style_drive_btn_stop;
static lv_style_t style_drive_status;

static lv_obj_t *label_temp_value;
static lv_obj_t *label_temp_caption;

static lv_obj_t *label_hum_value;
static lv_obj_t *label_hum_caption;

static lv_obj_t *label_gas_value;
static lv_obj_t *label_gas_caption;

static lv_obj_t *label_air_value;
static lv_obj_t *label_air_caption;

static lv_obj_t *label_status;
static lv_obj_t *label_conn;        
static lv_obj_t *label_refresh;

static lv_obj_t *btn_settings;
static lv_obj_t *panel_settings;
static lv_obj_t *switch_units;
static lv_obj_t *btn_settings_refresh;
static lv_obj_t *btn_settings_restart;

static lv_obj_t *panel_drive;
static lv_obj_t *btn_drive_open;
static lv_obj_t *label_drive_lat;
static lv_obj_t *label_drive_lon;
static lv_obj_t *label_drive_sats;
static lv_obj_t *label_drive_status;

static lv_obj_t *btn_stream_start;
static lv_obj_t *panel_stream;
static lv_obj_t *img_stream_display;
static lv_obj_t *btn_stream_stop;

static lv_obj_t *panel_welcome;
static lv_obj_t *btn_welcome_close;
static lv_obj_t *label_welcome_countdown;
static uint32_t welcome_seconds_remaining = 30;  

// cache from /metrics
struct RoverMetrics {
  float   temp_c     = NAN;
  float   humidity   = NAN;
  float   gas_kohms  = NAN;
  String  gas_type   = "";
  String  air_quality = "";
  bool    hasData    = false;
  uint32_t lastUpdateMs = 0;
};

static RoverMetrics metrics;

// cache from /coords
struct RoverCoords {
  String   lat   = "--";
  String   lon   = "--";
  String   sats  = "0";
  bool     hasData = false;
  uint32_t lastUpdateMs = 0;
};

static RoverCoords coords;

static uint32_t lastPollMs        = 0;
static uint32_t lastWiFiAttemptMs = 0;

static uint32_t lastCoordsPollMs  = 0;

static bool useFahrenheit = false;

void init_styles();
void create_main_ui();
void create_settings_panel(lv_obj_t *parent);
void create_stream_panel(lv_obj_t *parent);
void create_welcome_overlay(lv_obj_t *parent);
void ui_update_metrics();
void ui_update_status(const char *msg);
void ui_update_connection();
void handle_network();
void handle_stream();
void handle_leds();
void ensure_wifi();
bool fetch_metrics();
bool parse_json_metrics(const String &body, RoverMetrics &out);

void create_drive_panel(lv_obj_t *parent);
void ui_update_coords();
bool parse_json_coords(const String &body, RoverCoords &out);
bool fetch_coords();
void send_drive_command(const char *cmd);

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  lcd.pushImageDMA(area->x1, area->y1, w, h, (lgfx::rgb565_t*)&color_p->full);

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  LV_UNUSED(indev_driver);

  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PRESSED;
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_RELEASED;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void init_styles()
{
  
  lv_style_init(&style_card);
  lv_style_set_radius(&style_card, 22);
  lv_style_set_bg_color(&style_card, lv_color_hex(0xFFFFFF));
  lv_style_set_bg_opa(&style_card, LV_OPA_COVER);
  lv_style_set_border_color(&style_card, lv_color_hex(ASU_MAROON));
  lv_style_set_border_width(&style_card, 1);
  lv_style_set_pad_all(&style_card, 16);
  lv_style_set_shadow_width(&style_card, 18);
  lv_style_set_shadow_ofs_x(&style_card, 0);
  lv_style_set_shadow_ofs_y(&style_card, 10);
  lv_style_set_shadow_opa(&style_card, LV_OPA_20);
  lv_style_set_shadow_color(&style_card, lv_color_hex(ASU_MAROON));

  
  lv_style_init(&style_badge);
  lv_style_set_radius(&style_badge, 999);
  lv_style_set_bg_color(&style_badge, lv_color_hex(ASU_MAROON));
  lv_style_set_bg_opa(&style_badge, LV_OPA_COVER);
  lv_style_set_pad_left(&style_badge, 18);
  lv_style_set_pad_right(&style_badge, 18);
  lv_style_set_pad_top(&style_badge, 8);
  lv_style_set_pad_bottom(&style_badge, 8);
  lv_style_set_text_color(&style_badge, lv_color_hex(0xFFFFFF));
  lv_style_set_text_letter_space(&style_badge, 3);

  
  lv_style_init(&style_title);
  lv_style_set_text_color(&style_title, lv_color_hex(0x111827));

  
  lv_style_init(&style_value);
  lv_style_set_text_color(&style_value, lv_color_hex(0x111827));

  
  lv_style_init(&style_label_small);
  lv_style_set_text_color(&style_label_small, lv_color_hex(0x6B7280));

  
  lv_style_init(&style_button);
  lv_style_set_radius(&style_button, 999);
  lv_style_set_bg_color(&style_button, lv_color_hex(ASU_MAROON));
  lv_style_set_bg_opa(&style_button, LV_OPA_COVER);
  lv_style_set_text_color(&style_button, lv_color_hex(0xFFFFFF));
  lv_style_set_pad_left(&style_button, 28);
  lv_style_set_pad_right(&style_button, 28);
  lv_style_set_pad_top(&style_button, 10);
  lv_style_set_pad_bottom(&style_button, 10);

  lv_style_init(&style_button_pressed);
  lv_style_set_bg_color(&style_button_pressed, lv_color_hex(0x6A1230)); 

  
  lv_style_init(&style_button_red);
  lv_style_set_radius(&style_button_red, 999);
  lv_style_set_bg_color(&style_button_red, lv_color_hex(ASU_GOLD));
  lv_style_set_bg_opa(&style_button_red, LV_OPA_COVER);
  lv_style_set_text_color(&style_button_red, lv_color_hex(0x222222));
  lv_style_set_pad_left(&style_button_red, 28);
  lv_style_set_pad_right(&style_button_red, 28);
  lv_style_set_pad_top(&style_button_red, 12);
  lv_style_set_pad_bottom(&style_button_red, 12);

  
  lv_style_init(&style_metric_title);
  lv_style_set_text_color(&style_metric_title, lv_color_hex(0x111827));
#if LV_FONT_MONTSERRAT_20
  lv_style_set_text_font(&style_metric_title, &lv_font_montserrat_20);
#endif

  
  lv_style_init(&style_metric_value);
  lv_style_set_text_color(&style_metric_value, lv_color_hex(0x6B7280));  
#if LV_FONT_MONTSERRAT_22
  lv_style_set_text_font(&style_metric_value, &lv_font_montserrat_22);
#elif LV_FONT_MONTSERRAT_18
  lv_style_set_text_font(&style_metric_value, &lv_font_montserrat_18);
#endif

  
  lv_style_init(&style_conn_card);
  lv_style_set_radius(&style_conn_card, 999);
  lv_style_set_bg_color(&style_conn_card, lv_color_hex(0xFFF4CC)); 
  lv_style_set_bg_opa(&style_conn_card, LV_OPA_COVER);
  lv_style_set_border_color(&style_conn_card, lv_color_hex(ASU_GOLD));
  lv_style_set_border_width(&style_conn_card, 1);
  lv_style_set_pad_left(&style_conn_card, 16);
  lv_style_set_pad_right(&style_conn_card, 16);
  lv_style_set_pad_top(&style_conn_card, 6);
  lv_style_set_pad_bottom(&style_conn_card, 6);
  lv_style_set_shadow_width(&style_conn_card, 8);
  lv_style_set_shadow_opa(&style_conn_card, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&style_conn_card, 4);
  lv_style_set_shadow_color(&style_conn_card, lv_color_hex(ASU_GOLD));

  
  lv_style_init(&style_status_bar);
  lv_style_set_radius(&style_status_bar, 22);
  lv_style_set_bg_color(&style_status_bar, lv_color_hex(0xFFFFFF));
  lv_style_set_bg_opa(&style_status_bar, LV_OPA_COVER);
  lv_style_set_border_color(&style_status_bar, lv_color_hex(ASU_MAROON));
  lv_style_set_border_width(&style_status_bar, 1);
  lv_style_set_pad_left(&style_status_bar, 20);
  lv_style_set_pad_right(&style_status_bar, 20);
  lv_style_set_pad_top(&style_status_bar, 12);
  lv_style_set_pad_bottom(&style_status_bar, 12);
  lv_style_set_shadow_width(&style_status_bar, 12);
  lv_style_set_shadow_opa(&style_status_bar, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&style_status_bar, 6);
  lv_style_set_shadow_color(&style_status_bar, lv_color_hex(ASU_MAROON));

    

  
  lv_style_init(&style_drive_card);
  lv_style_set_radius(&style_drive_card, 28);
  lv_style_set_bg_color(&style_drive_card, lv_color_hex(0xF9FAFB));   
  lv_style_set_bg_opa(&style_drive_card, LV_OPA_COVER);
  lv_style_set_border_color(&style_drive_card, lv_color_hex(0xE5E7EB));
  lv_style_set_border_width(&style_drive_card, 1);
  lv_style_set_pad_left(&style_drive_card, 26);
  lv_style_set_pad_right(&style_drive_card, 26);
  lv_style_set_pad_top(&style_drive_card, 20);
  lv_style_set_pad_bottom(&style_drive_card, 20);
  lv_style_set_shadow_width(&style_drive_card, 20);
  lv_style_set_shadow_opa(&style_drive_card, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&style_drive_card, 10);
  lv_style_set_shadow_color(&style_drive_card, lv_color_hex(0x9CA3AF)); 

  
  lv_style_init(&style_drive_btn);
  lv_style_set_radius(&style_drive_btn, 18);
  lv_style_set_bg_color(&style_drive_btn, lv_color_hex(0xFFFFFF));
  lv_style_set_bg_opa(&style_drive_btn, LV_OPA_COVER);
  lv_style_set_border_color(&style_drive_btn, lv_color_hex(ASU_MAROON));
  lv_style_set_border_width(&style_drive_btn, 2);
  lv_style_set_shadow_width(&style_drive_btn, 15);
  lv_style_set_shadow_opa(&style_drive_btn, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&style_drive_btn, 5);
  lv_style_set_shadow_color(&style_drive_btn, lv_color_hex(0x9CA3AF));
  lv_style_set_pad_left(&style_drive_btn, 8);
  lv_style_set_pad_right(&style_drive_btn, 8);
  lv_style_set_pad_top(&style_drive_btn, 8);
  lv_style_set_pad_bottom(&style_drive_btn, 8);
  lv_style_set_text_color(&style_drive_btn, lv_color_hex(0x111827));

  
  lv_style_init(&style_drive_btn_stop);
  lv_style_set_radius(&style_drive_btn_stop, 18);
  lv_style_set_bg_color(&style_drive_btn_stop, lv_color_hex(ASU_MAROON));
  lv_style_set_bg_opa(&style_drive_btn_stop, LV_OPA_COVER);
  lv_style_set_border_color(&style_drive_btn_stop, lv_color_hex(ASU_MAROON));
  lv_style_set_border_width(&style_drive_btn_stop, 2);
  lv_style_set_shadow_width(&style_drive_btn_stop, 15);
  lv_style_set_shadow_opa(&style_drive_btn_stop, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&style_drive_btn_stop, 5);
  lv_style_set_shadow_color(&style_drive_btn_stop, lv_color_hex(0x9CA3AF));
  lv_style_set_pad_left(&style_drive_btn_stop, 8);
  lv_style_set_pad_right(&style_drive_btn_stop, 8);
  lv_style_set_pad_top(&style_drive_btn_stop, 8);
  lv_style_set_pad_bottom(&style_drive_btn_stop, 8);
  lv_style_set_text_color(&style_drive_btn_stop, lv_color_hex(0xFFFFFF));

  
  lv_style_init(&style_drive_status);
  lv_style_set_text_color(&style_drive_status, lv_color_hex(0x4B5563)); 

}

const char *rssiToQuality(int rssi)
{
  if (rssi >= -60) return "Strong";
  if (rssi >= -70) return "Good";
  if (rssi >= -80) return "Fair";
  return "Weak";
}

static void drive_open_event_cb(lv_event_t *e); 
static void drive_close_event_cb(lv_event_t *e); 
static void drive_forward_cb(lv_event_t *e); 
static void drive_back_cb(lv_event_t *e); 
static void drive_left_cb(lv_event_t *e); 
static void drive_right_cb(lv_event_t *e); 
static void drive_stop_cb(lv_event_t *e);

static void drive_open_event_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  if (panel_drive) {
    lv_obj_clear_flag(panel_drive, LV_OBJ_FLAG_HIDDEN);
  }
}

static void drive_close_event_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  if (panel_drive) {
    lv_obj_add_flag(panel_drive, LV_OBJ_FLAG_HIDDEN);
  }
}

static void drive_forward_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  send_drive_command("1 forward");
}

static void drive_back_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  send_drive_command("1 back");
}

static void drive_left_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  send_drive_command("left");
}

static void drive_right_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  send_drive_command("right");
}

static void drive_stop_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  send_drive_command("stop");
}

static void settings_open_event_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  if (panel_settings) {
    lv_obj_clear_flag(panel_settings, LV_OBJ_FLAG_HIDDEN);
  }
}

static void settings_close_event_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  if (panel_settings) {
    lv_obj_add_flag(panel_settings, LV_OBJ_FLAG_HIDDEN);
  }
}

static void stream_open_event_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  if (panel_stream) {
    lv_obj_clear_flag(panel_stream, LV_OBJ_FLAG_HIDDEN);
    streamActive      = true;
    stream_error      = false;
    lastStreamFrameMs = millis();
    if (streamClient.connected()) streamClient.stop();
    if (label_stream_status) {
      lv_label_set_text(label_stream_status, "Connecting to rover video...");
    }
    ui_update_status("Starting live stream...");
    streamSprite.fillSprite(TFT_BLACK);
  }
}

static void stream_close_event_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  if (panel_stream) {
    lv_obj_add_flag(panel_stream, LV_OBJ_FLAG_HIDDEN);
    streamActive = false;
    if (streamClient.connected()) streamClient.stop();
    ui_update_status("Stream stopped. Resuming metrics.");
  }
}

static void settings_units_event_cb(lv_event_t *e)
{
  lv_obj_t *sw = lv_event_get_target(e);
  useFahrenheit = lv_obj_has_state(sw, LV_STATE_CHECKED);
  ui_update_metrics();  
}

static void settings_refresh_event_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  ui_update_status("Refreshing metrics...");
  fetch_metrics();
}

static void settings_restart_event_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  ui_update_status("Restarting device...");
  delay(250);
  ESP.restart();
}

static void welcome_close_event_cb(lv_event_t *e)
{
  LV_UNUSED(e);
  if (panel_welcome) {
    lv_obj_add_flag(panel_welcome, LV_OBJ_FLAG_HIDDEN);
  }
}

static void welcome_countdown_cb(lv_timer_t *t)
{
  LV_UNUSED(t);

  if (welcome_seconds_remaining > 0) {
    welcome_seconds_remaining--;
  }

  if (label_welcome_countdown && welcome_seconds_remaining > 0) {
    char buf[64];
    snprintf(buf, sizeof(buf),
             "(You can dismiss this in %lus)",
             (unsigned long)welcome_seconds_remaining);
    lv_label_set_text(label_welcome_countdown, buf);
  }

  if (welcome_seconds_remaining == 0) {
    
    if (btn_welcome_close) {
      lv_obj_clear_flag(btn_welcome_close, LV_OBJ_FLAG_HIDDEN);
    }
    
    if (label_welcome_countdown) {
      lv_obj_add_flag(label_welcome_countdown, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

void create_main_ui()
{
  lv_obj_t *scr = lv_scr_act();

  
  lv_obj_set_style_bg_color(scr, lv_color_hex(0xE5E7EB), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  

  lv_obj_t *card_header = lv_obj_create(scr);
  lv_obj_add_style(card_header, &style_card, 0);
  lv_obj_set_size(card_header, 760, 120);
  lv_obj_align(card_header, LV_ALIGN_TOP_MID, 0, 16);
  lv_obj_set_flex_flow(card_header, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(card_header,
                        LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_START);

  
  lv_obj_t *badge = lv_label_create(card_header);
  lv_obj_add_style(badge, &style_badge, 0);
  lv_label_set_text(badge, "PHOENIX FORCE");

  
  lv_obj_t *subtitle = lv_label_create(card_header);
  lv_obj_add_style(subtitle, &style_title, 0);
  lv_label_set_text(subtitle, "Project home | EGR 314 | TerraGuard Rover");

  
  lv_obj_t *conn_card = lv_obj_create(scr);
  lv_obj_add_style(conn_card, &style_conn_card, 0);
  lv_obj_set_size(conn_card, 260, LV_SIZE_CONTENT);
  lv_obj_align_to(conn_card, card_header, LV_ALIGN_TOP_RIGHT, -8, 32);

  label_conn = lv_label_create(conn_card);
  lv_obj_add_style(label_conn, &style_title, 0);
  lv_label_set_text(label_conn, "Connection: offline");
  lv_obj_center(label_conn);

  

  lv_obj_t *metrics_row = lv_obj_create(scr);
  lv_obj_set_size(metrics_row, 760, 260);   
  lv_obj_align_to(metrics_row, card_header, LV_ALIGN_OUT_BOTTOM_MID, 0, 6);

  lv_obj_set_style_bg_opa(metrics_row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(metrics_row, 0, 0);
  lv_obj_set_style_pad_all(metrics_row, 0, 0);
  lv_obj_set_style_pad_row(metrics_row, 18, 0);    
  lv_obj_set_style_pad_column(metrics_row, 18, 0); 

  lv_obj_set_flex_flow(metrics_row, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(metrics_row,
                        LV_FLEX_ALIGN_SPACE_BETWEEN,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_clear_flag(metrics_row, LV_OBJ_FLAG_SCROLLABLE);

  
  auto create_metric_card = [&](const char *caption,
                                lv_obj_t **out_caption,
                                lv_obj_t **out_value)
  {
    
    lv_obj_t *card = lv_obj_create(metrics_row);
    lv_obj_add_style(card, &style_card, 0);
    lv_obj_set_size(card, 360, 110);

    
    lv_obj_t *inner = lv_obj_create(card);
    lv_obj_set_size(inner, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(inner, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(inner, 0, 0);
    lv_obj_set_style_pad_all(inner, 0, 0);
    lv_obj_set_flex_flow(inner, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(inner,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_center(inner);   

    
    lv_obj_t *cap = lv_label_create(inner);
    lv_obj_add_style(cap, &style_metric_title, 0);
    lv_label_set_text(cap, caption);
    lv_obj_set_style_text_align(cap, LV_TEXT_ALIGN_CENTER, 0);

    
    lv_obj_t *val = lv_label_create(inner);
    lv_obj_add_style(val, &style_metric_value, 0);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_CENTER, 0);

    if (out_caption) *out_caption = cap;
    if (out_value)   *out_value   = val;
  };

  create_metric_card("Temperature",   &label_temp_caption, &label_temp_value);
  create_metric_card("Humidity",      &label_hum_caption,  &label_hum_value);
  create_metric_card("Air Type | Air Quality",   &label_air_caption,  &label_air_value);
  create_metric_card("Gas Resistance",&label_gas_caption,  &label_gas_value);

  
  lv_label_set_long_mode(label_air_value, LV_LABEL_LONG_DOT);
  lv_obj_set_width(label_air_value, 320);
  lv_obj_set_style_text_align(label_air_value, LV_TEXT_ALIGN_CENTER, 0);

  lv_label_set_long_mode(label_gas_value, LV_LABEL_LONG_DOT);
  lv_obj_set_width(label_gas_value, 320);
  lv_obj_set_style_text_align(label_gas_value, LV_TEXT_ALIGN_CENTER, 0);

  

  lv_obj_t *footer = lv_obj_create(scr);
  lv_obj_add_style(footer, &style_status_bar, 0);  
  lv_obj_set_size(footer, 760, LV_SIZE_CONTENT);
  lv_obj_align_to(footer, metrics_row, LV_ALIGN_OUT_BOTTOM_MID, 0, 6);
  lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(footer,
                        LV_FLEX_ALIGN_SPACE_BETWEEN,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);

  
  label_status = lv_label_create(footer);
  lv_obj_add_style(label_status, &style_title, 0);
  lv_label_set_text(label_status, "Status: Connecting to rover...");

  
  lv_obj_t *btn_container = lv_obj_create(footer);
  lv_obj_set_size(btn_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(btn_container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(btn_container, 0, 0);
  lv_obj_set_style_pad_all(btn_container, 0, 0);
  lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
  lv_obj_set_style_pad_gap(btn_container, 10, 0); 

  
  btn_stream_start = lv_btn_create(btn_container);
  lv_obj_add_style(btn_stream_start, &style_button_red, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_stream_start, stream_open_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_stream = lv_label_create(btn_stream_start);
  lv_label_set_text(lbl_stream, "Stream");

  
  btn_drive_open = lv_btn_create(btn_container);
  lv_obj_add_style(btn_drive_open, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_drive_open, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_drive_open, drive_open_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_drive = lv_label_create(btn_drive_open);
  lv_label_set_text(lbl_drive, "Drive");

  
  btn_settings = lv_btn_create(btn_container);
  lv_obj_add_style(btn_settings, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_settings, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_settings, settings_open_event_cb, LV_EVENT_CLICKED, NULL);
  label_refresh = lv_label_create(btn_settings);
  lv_label_set_text(label_refresh, "Settings");

  
  create_settings_panel(scr);
  
  create_stream_panel(scr);
  
  create_drive_panel(scr);
  
  create_welcome_overlay(scr);
}

void create_settings_panel(lv_obj_t *parent)
{
  panel_settings = lv_obj_create(parent);
  lv_obj_set_size(panel_settings, 800, 480);
  lv_obj_align(panel_settings, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(panel_settings, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(panel_settings, LV_OPA_30, 0);
  lv_obj_add_flag(panel_settings, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_flag(panel_settings, LV_OBJ_FLAG_HIDDEN); 

  
  lv_obj_t *card = lv_obj_create(panel_settings);
  lv_obj_add_style(card, &style_card, 0);
  lv_obj_set_size(card, 520, 320);
  lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(card,
                        LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_START);

  
  lv_obj_t *row_top = lv_obj_create(card);
  lv_obj_set_size(row_top, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row_top, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row_top, 0, 0);
  lv_obj_set_flex_flow(row_top, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_top,
                        LV_FLEX_ALIGN_SPACE_BETWEEN,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_t *label_title = lv_label_create(row_top);
  lv_obj_add_style(label_title, &style_title, 0);
  lv_label_set_text(label_title, "Settings");

  lv_obj_t *btn_close = lv_btn_create(row_top);
  lv_obj_add_style(btn_close, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_close, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_set_width(btn_close, 60);
  lv_obj_add_event_cb(btn_close, settings_close_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *label_close = lv_label_create(btn_close);
  lv_label_set_text(label_close, "X");

  
  lv_obj_t *row_units = lv_obj_create(card);
  lv_obj_set_size(row_units, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row_units, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row_units, 0, 0);
  lv_obj_set_flex_flow(row_units, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_units,
                        LV_FLEX_ALIGN_SPACE_BETWEEN,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_t *label_units = lv_label_create(row_units);
  lv_obj_add_style(label_units, &style_label_small, 0);
  lv_label_set_text(label_units, "Temperature units (°C / °F)");

  switch_units = lv_switch_create(row_units);
  lv_obj_add_event_cb(switch_units, settings_units_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  
  lv_obj_t *row_buttons = lv_obj_create(card);
  lv_obj_set_size(row_buttons, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row_buttons, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row_buttons, 0, 0);
  lv_obj_set_flex_flow(row_buttons, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_buttons,
                        LV_FLEX_ALIGN_SPACE_AROUND,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  btn_settings_refresh = lv_btn_create(row_buttons);
  lv_obj_add_style(btn_settings_refresh, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_settings_refresh, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_settings_refresh, settings_refresh_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_sref = lv_label_create(btn_settings_refresh);
  lv_label_set_text(lbl_sref, "Refresh now");

  btn_settings_restart = lv_btn_create(row_buttons);
  lv_obj_add_style(btn_settings_restart, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_settings_restart, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_settings_restart, settings_restart_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_srst = lv_label_create(btn_settings_restart);
  lv_label_set_text(lbl_srst, "Restart HMI");
}

void create_stream_panel(lv_obj_t *parent)
{
  panel_stream = lv_obj_create(parent);
  lv_obj_set_size(panel_stream, 800, 480);
  lv_obj_set_style_bg_color(panel_stream, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(panel_stream, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(panel_stream, 0, 0);
  lv_obj_set_style_radius(panel_stream, 0, 0); 
  lv_obj_clear_flag(panel_stream, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(panel_stream, LV_OBJ_FLAG_HIDDEN);

  
  label_stream_status = lv_label_create(panel_stream);
  lv_obj_add_style(label_stream_status, &style_label_small, 0);
  lv_label_set_text(label_stream_status, "");
  lv_obj_set_width(label_stream_status, lv_pct(100));
  lv_obj_set_style_text_align(label_stream_status, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(label_stream_status, LV_ALIGN_TOP_MID, 0, 18);

  
  img_stream_display = lv_img_create(panel_stream);
  stream_img_dsc.header.always_zero = 0;
  stream_img_dsc.header.w = 320;
  stream_img_dsc.header.h = 240;
  stream_img_dsc.data_size = 320 * 240 * 2; 
  stream_img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
  stream_img_dsc.data = (const uint8_t*)streamSprite.getBuffer();

  lv_img_set_src(img_stream_display, &stream_img_dsc);
  lv_img_set_zoom(img_stream_display, 512); 
  lv_obj_set_style_radius(img_stream_display, 0, 0); 
  lv_obj_center(img_stream_display);

  
  btn_stream_stop = lv_btn_create(panel_stream);
  lv_obj_add_style(btn_stream_stop, &style_button_red, LV_PART_MAIN);
  lv_obj_align(btn_stream_stop, LV_ALIGN_BOTTOM_MID, 0, -30);
  lv_obj_add_event_cb(btn_stream_stop, stream_close_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label = lv_label_create(btn_stream_stop);
  lv_label_set_text(label, "STOP STREAM");
#if LV_FONT_MONTSERRAT_14
  lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
#endif
}

void create_drive_panel(lv_obj_t *parent)
{
  panel_drive = lv_obj_create(parent);
  lv_obj_set_size(panel_drive, 800, 480);
  lv_obj_set_style_bg_color(panel_drive, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(panel_drive, LV_OPA_40, 0);
  lv_obj_set_style_border_width(panel_drive, 0, 0);
  lv_obj_set_style_radius(panel_drive, 0, 0);
  lv_obj_clear_flag(panel_drive, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(panel_drive, LV_OBJ_FLAG_HIDDEN);   

  
  lv_obj_t *card = lv_obj_create(panel_drive);
  lv_obj_add_style(card, &style_card, 0);
  lv_obj_set_size(card, 800, 480);
  lv_obj_set_style_radius(card, 0, 0);
  lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);

  
  lv_obj_set_style_pad_all(card, 10, 0);
  lv_obj_set_style_pad_row(card, 4, 0);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
  
  lv_obj_set_flex_align(card,
                        LV_FLEX_ALIGN_START,   
                        LV_FLEX_ALIGN_CENTER,  
                        LV_FLEX_ALIGN_START);  

  
  lv_obj_t *row_top = lv_obj_create(card);
  lv_obj_set_size(row_top, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row_top, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row_top, 0, 0);
  lv_obj_set_style_pad_top(row_top, 2, 0);
  lv_obj_set_style_pad_bottom(row_top, 2, 0);
  lv_obj_set_flex_flow(row_top, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_top,
                        LV_FLEX_ALIGN_SPACE_BETWEEN,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_t *label_title = lv_label_create(row_top);
  lv_obj_add_style(label_title, &style_title, 0);
  lv_label_set_text(label_title, "Drive Control");

  lv_obj_t *btn_back = lv_btn_create(row_top);
  lv_obj_add_style(btn_back, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_back, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_set_width(btn_back, 70);
  lv_obj_add_event_cb(btn_back, drive_close_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *label_back = lv_label_create(btn_back);
  lv_label_set_text(label_back, "Back");

  
  lv_obj_t *row_coords = lv_obj_create(card);
  lv_obj_set_size(row_coords, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row_coords, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row_coords, 0, 0);
  lv_obj_set_style_pad_top(row_coords, 0, 0);
  lv_obj_set_style_pad_bottom(row_coords, 0, 0);
  lv_obj_set_flex_flow(row_coords, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_coords,
                        LV_FLEX_ALIGN_SPACE_AROUND,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_t *lbl_lat_cap = lv_label_create(row_coords);
  lv_obj_add_style(lbl_lat_cap, &style_label_small, 0);
  lv_label_set_text(lbl_lat_cap, "Lat:");

  label_drive_lat = lv_label_create(row_coords);
  lv_obj_add_style(label_drive_lat, &style_value, 0);
  lv_label_set_text(label_drive_lat, "--");

  lv_obj_t *lbl_lon_cap = lv_label_create(row_coords);
  lv_obj_add_style(lbl_lon_cap, &style_label_small, 0);
  lv_label_set_text(lbl_lon_cap, "Lon:");

  label_drive_lon = lv_label_create(row_coords);
  lv_obj_add_style(label_drive_lon, &style_value, 0);
  lv_label_set_text(label_drive_lon, "--");

  
  lv_obj_t *row_dpad = lv_obj_create(card);
  lv_obj_set_size(row_dpad, lv_pct(100), 160);       
  lv_obj_set_style_bg_opa(row_dpad, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row_dpad, 0, 0);
  lv_obj_set_style_pad_top(row_dpad, 4, 0);
  lv_obj_set_style_pad_bottom(row_dpad, 4, 0);
  lv_obj_set_flex_flow(row_dpad, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(row_dpad,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  
  lv_obj_t *btn_up = lv_btn_create(row_dpad);
  lv_obj_add_style(btn_up, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_up, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_up, drive_forward_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_up = lv_label_create(btn_up);
  lv_label_set_text(lbl_up, "FORWARD");

  
  lv_obj_t *row_mid = lv_obj_create(row_dpad);
  lv_obj_set_size(row_mid, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row_mid, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row_mid, 0, 0);
  lv_obj_set_style_pad_top(row_mid, 4, 0);
  lv_obj_set_style_pad_bottom(row_mid, 4, 0);
  lv_obj_set_flex_flow(row_mid, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_mid,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_t *btn_left = lv_btn_create(row_mid);
  lv_obj_add_style(btn_left, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_left, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_left, drive_left_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_left = lv_label_create(btn_left);
  lv_label_set_text(lbl_left, "LEFT");

  lv_obj_t *btn_stop = lv_btn_create(row_mid);
  lv_obj_add_style(btn_stop, &style_button_red, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_stop, drive_stop_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_stop = lv_label_create(btn_stop);
  lv_label_set_text(lbl_stop, "STOP");

  lv_obj_t *btn_right = lv_btn_create(row_mid);
  lv_obj_add_style(btn_right, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_right, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_right, drive_right_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_right = lv_label_create(btn_right);
  lv_label_set_text(lbl_right, "RIGHT");

  
  lv_obj_t *btn_down = lv_btn_create(row_dpad);
  lv_obj_add_style(btn_down, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_down, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_down, drive_back_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_down = lv_label_create(btn_down);
  lv_label_set_text(lbl_down, "REVERSE");

  
  label_drive_status = lv_label_create(card);
  lv_obj_add_style(label_drive_status, &style_label_small, 0);
  lv_label_set_text(label_drive_status, "Drive ready. Use D-pad to send commands.");
}

void create_welcome_overlay(lv_obj_t *parent)
{
  panel_welcome = lv_obj_create(parent);
  lv_obj_set_size(panel_welcome, 800, 480);
  lv_obj_align(panel_welcome, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(panel_welcome, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(panel_welcome, LV_OPA_60, 0);
  lv_obj_clear_flag(panel_welcome, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(panel_welcome, LV_OBJ_FLAG_CLICKABLE); 

  
  lv_obj_t *card = lv_obj_create(panel_welcome);
  lv_obj_add_style(card, &style_card, 0);
  lv_obj_set_size(card, 620, 260);
  lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
  
  lv_obj_set_flex_align(card,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  
  lv_obj_t *title = lv_label_create(card);
  lv_obj_add_style(title, &style_title, 0);
  lv_label_set_text(title, "Welcome to TerraGuard Operator Dashboard");
  lv_obj_set_width(title, lv_pct(100));
  lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);

  
  lv_obj_set_style_text_color(title, lv_color_hex(ASU_MAROON), 0);
#if LV_FONT_MONTSERRAT_24
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
#elif LV_FONT_MONTSERRAT_22
  lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
#endif

  
  lv_obj_t *msg = lv_label_create(card);
  lv_obj_add_style(msg, &style_label_small, 0);
  lv_label_set_text(msg,
    "Please note that the screen takes several minutes to\n"
    "calibrate the data and display it correctly.");
  lv_obj_set_width(msg, lv_pct(100));
  lv_obj_set_style_text_align(msg, LV_TEXT_ALIGN_CENTER, 0);
#if LV_FONT_MONTSERRAT_18
  lv_obj_set_style_text_font(msg, &lv_font_montserrat_18, 0);
#endif

  
  btn_welcome_close = lv_btn_create(card);
  lv_obj_add_style(btn_welcome_close, &style_button, LV_PART_MAIN);
  lv_obj_add_style(btn_welcome_close, &style_button_pressed, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_welcome_close, welcome_close_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_flag(btn_welcome_close, LV_OBJ_FLAG_HIDDEN);  

  lv_obj_t *lbl_close = lv_label_create(btn_welcome_close);
  lv_label_set_text(lbl_close, "Got it");

  
  label_welcome_countdown = lv_label_create(card);
  lv_obj_add_style(label_welcome_countdown, &style_label_small, 0);
  char buf[64];
  snprintf(buf, sizeof(buf),
           "(You can dismiss this in %lus)",
           (unsigned long)welcome_seconds_remaining);
  lv_label_set_text(label_welcome_countdown, buf);
  lv_obj_set_width(label_welcome_countdown, lv_pct(100));
  lv_obj_set_style_text_align(label_welcome_countdown, LV_TEXT_ALIGN_CENTER, 0);
}

void ui_update_coords()
{
  if (!label_drive_lat || !label_drive_lon) return;

  if (!coords.hasData) {
    lv_label_set_text(label_drive_lat, "--");
    lv_label_set_text(label_drive_lon, "--");
    if (label_drive_status) {
      lv_label_set_text(label_drive_status, "GPS: waiting for fix...");
    }
    return;
  }

  lv_label_set_text(label_drive_lat, coords.lat.c_str());
  lv_label_set_text(label_drive_lon, coords.lon.c_str());

  if (label_drive_status) {
    char buf[64];
    snprintf(buf, sizeof(buf), "GPS: %s sats", coords.sats.c_str());
    lv_label_set_text(label_drive_status, buf);
  }
}

void ui_update_metrics()
{
  if (!metrics.hasData) {
    lv_label_set_text(label_temp_value, "--");
    lv_label_set_text(label_hum_value,  "--");
    lv_label_set_text(label_gas_value,  "--");
    lv_label_set_text(label_air_value,  "--");
    return;
  }

  char buf[64];

  
  float displayTemp = metrics.temp_c;
  const char *unit = "C";
  if (useFahrenheit) {
    displayTemp = metrics.temp_c * 9.0f / 5.0f + 32.0f;
    unit = "F";
  }
  snprintf(buf, sizeof(buf), "%.1f °%s", displayTemp, unit);
  lv_label_set_text(label_temp_value, buf);

  snprintf(buf, sizeof(buf), "%.1f %%", metrics.humidity);
  lv_label_set_text(label_hum_value, buf);

  snprintf(buf, sizeof(buf), "%.2f kOhms", metrics.gas_kohms);
  lv_label_set_text(label_gas_value, buf);

  String aq = metrics.gas_type;
  if (aq.length() > 0 && metrics.air_quality.length() > 0) {
    aq += " | ";
  }
  aq += metrics.air_quality;
  lv_label_set_text(label_air_value, aq.c_str());
}

void ui_update_status(const char *msg)
{
  if (label_status) {
    lv_label_set_text(label_status, msg);
  }
}

void ui_update_connection()
{
  if (!label_conn) return;

  if (WiFi.status() != WL_CONNECTED) {
    lv_label_set_text(label_conn, "Connection: offline");
    return;
  }

  int rssi = WiFi.RSSI();
  const char *quality = rssiToQuality(rssi);
  char buf[64];
  snprintf(buf, sizeof(buf), "Connection: %s", quality);
  lv_label_set_text(label_conn, buf);
}

void ensure_wifi()
{
  if (WiFi.status() == WL_CONNECTED) return;

  uint32_t now = millis();
  if (now - lastWiFiAttemptMs < WIFI_RETRY_MS) return;
  lastWiFiAttemptMs = now;

  ui_update_status("Connecting to TerraGuard-AP...");

  Serial.println("[WiFi] Connecting...");
  WiFi.mode(WIFI_STA);
  if (strlen(WIFI_PASS) == 0) {
    WiFi.begin(WIFI_SSID);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

bool parse_json_coords(const String &body, RoverCoords &out)
{
  auto findString = [&](const char *key, String &val) -> bool {
    String k = String("\"") + key + "\":\"";
    int idx = body.indexOf(k);
    if (idx < 0) return false;
    idx += k.length();
    int end = body.indexOf('"', idx);
    if (end < 0) return false;
    val = body.substring(idx, end);
    return true;
  };

  bool ok = true;
  ok &= findString("lat", out.lat);
  ok &= findString("lon", out.lon);
  
  findString("sats", out.sats);

  return ok;
}

bool parse_json_metrics(const String &body, RoverMetrics &out)
{
  auto findFloat = [&](const char *key, float &val) -> bool {
    String k = String("\"") + key + "\":";
    int idx = body.indexOf(k);
    if (idx < 0) return false;
    idx += k.length();
    int end = idx;
    while (end < (int)body.length() && (isDigit(body[end]) || body[end] == '.' || body[end] == '-')) {
      end++;
    }
    if (end == idx) return false;
    val = body.substring(idx, end).toFloat();
    return true;
  };

  auto findString = [&](const char *key, String &val) -> bool {
    String k = String("\"") + key + "\":\"";
    int idx = body.indexOf(k);
    if (idx < 0) return false;
    idx += k.length();
    int end = body.indexOf('"', idx);
    if (end < 0) return false;
    val = body.substring(idx, end);
    return true;
  };

  bool ok = true;
  ok &= findFloat("temp_c", out.temp_c);
  ok &= findFloat("humidity", out.humidity);
  ok &= findFloat("gas_kohms", out.gas_kohms);
  findString("gas_type", out.gas_type);
  findString("air_quality", out.air_quality);

  return ok;
}

bool fetch_metrics()
{
  if (streamActive) return false;

  ensure_wifi();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] No WiFi yet");
    ui_update_status("WiFi not connected to TerraGuard-AP");
    ui_update_connection();
    return false;
  }

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);

  Serial.printf("[HTTP] GET %s\n", METRICS_URL);

  if (!http.begin(METRICS_URL)) {
    Serial.println("[HTTP] begin() failed (metrics)");
    ui_update_status("HTTP begin failed (/metrics)");
    http.end();
    return false;
  }

  int code = http.GET();

  if (code <= 0) {
    
    Serial.printf("[HTTP] GET failed, error = %d\n", code);
    ui_update_status("HTTP conn error (/metrics)");
    http.end();
    return false;
  }

  if (code != HTTP_CODE_OK) {
    
    Serial.printf("[HTTP] Error: %d\n", code);
    char buf[64];
    snprintf(buf, sizeof(buf), "HTTP %d from /metrics", code);
    ui_update_status(buf);
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();

  Serial.println("[HTTP] Body:");
  Serial.println(body);

  RoverMetrics newMetrics;
  if (!parse_json_metrics(body, newMetrics)) {
    Serial.println("[HTTP] JSON parse failed");
    ui_update_status("Parse error (/metrics)");
    return false;
  }

  metrics = newMetrics;
  metrics.hasData = true;
  metrics.lastUpdateMs = millis();

  ui_update_metrics();
  ui_update_status("Status: Streaming live data. Last update: 0s ago");
  ui_update_connection();

  return true;
}

bool fetch_coords()
{
  ensure_wifi();
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  Serial.println(String("[HTTP] GET ") + COORDS_URL);
  if (!http.begin(COORDS_URL)) {
    Serial.println("[HTTP] begin() failed (coords)");
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[HTTP] Coords error: %d\n", code);
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();

  Serial.println("[HTTP] Coords body:");
  Serial.println(body);

  RoverCoords newCoords;
  if (!parse_json_coords(body, newCoords)) {
    Serial.println("[HTTP] Coords JSON parse failed");
    return false;
  }

  coords = newCoords;
  coords.hasData = true;
  coords.lastUpdateMs = millis();

  ui_update_coords();
  return true;
}

void send_drive_command(const char *cmd)
{
  ensure_wifi();
  if (WiFi.status() != WL_CONNECTED) {
    ui_update_status("Drive: no WiFi");
    if (label_drive_status) {
      lv_label_set_text(label_drive_status, "Drive: no WiFi");
    }
    return;
  }

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  Serial.print("[DRIVE] POST ");
  Serial.print(CONTROL_URL);
  Serial.print(" cmd = ");
  Serial.println(cmd);

  if (!http.begin(CONTROL_URL)) {
    Serial.println("[DRIVE] http.begin failed");
    if (label_drive_status) {
      lv_label_set_text(label_drive_status, "Drive: HTTP begin failed");
    }
    ui_update_status("Drive: HTTP error");
    return;
  }

  http.addHeader("Content-Type", "text/plain");
  int code = http.POST((uint8_t*)cmd, strlen(cmd));
  http.end();

  if (code != HTTP_CODE_OK) {
    Serial.printf("[DRIVE] HTTP error: %d\n", code);
    if (label_drive_status) {
      lv_label_set_text(label_drive_status, "Drive: command failed");
    }
    ui_update_status("Drive: command failed");
  } else {
    char buf[64];
    snprintf(buf, sizeof(buf), "Sent: %s", cmd);
    if (label_drive_status) {
      lv_label_set_text(label_drive_status, buf);
    }
    ui_update_status("Drive command sent");
  }
}

void fix_stream_byteswap()
{
  uint16_t *buf = (uint16_t *)streamSprite.getBuffer();
  if (!buf) return;

  const int PIXELS = 320 * 240; 
  for (int i = 0; i < PIXELS; ++i)
  {
    uint16_t c = buf[i];
    buf[i] = (uint16_t)((c >> 8) | (c << 8));  
  }
}

void handle_stream() {
  if (!streamActive) return;

  uint32_t now = millis();

  
  if (!streamClient.connected()) {
    Serial.println("[Stream] Connecting...");
    if (!streamClient.connect(STREAM_HOST, STREAM_PORT)) {
      Serial.println("[Stream] Connection failed");
      if (!stream_error) {
        stream_error = true;
        if (label_stream_status) {
          lv_label_set_text(label_stream_status,
            "Cannot fetch stream.\n"
            "Please ensure TerraGuard rover is on and in range.");
        }
        ui_update_status("Stream error: cannot reach rover.");
      }
      return;
    }

    
    streamClient.print(String("GET ") + STREAM_PATH + " HTTP/1.1\r\n");
    streamClient.print("Host: " + String(STREAM_HOST) + "\r\n");
    streamClient.print("Connection: close\r\n\r\n");

    lastStreamFrameMs = now;
    stream_error = false;
    if (label_stream_status) {
      lv_label_set_text(label_stream_status, "Connecting to camera...");
    }
    ui_update_status("Stream: connecting to camera...");
  }

  
  if (!stream_error && (now - lastStreamFrameMs > STREAM_FRAME_TIMEOUT_MS)) {
    stream_error = true;
    if (label_stream_status) {
      lv_label_set_text(label_stream_status,
        "Cannot fetch stream.\n"
        "Camera may be disconnected or failed to initialize.\n"
        "Please check the rover and camera module.");
    }
    ui_update_status("Stream error: no video frames received.");
  }

  if (!streamClient.available()) return;

  String line = streamClient.readStringUntil('\n');

  if (line.startsWith("Content-Length: ")) {
    int len = line.substring(16).toInt();
    streamClient.readStringUntil('\n'); 

    if (len > 0 && len < (int)JPG_BUF_SIZE) {
      int bytesRead = 0;
      while (bytesRead < len && streamClient.connected()) {
        int r = streamClient.read(jpgBuffer + bytesRead, len - bytesRead);
        if (r > 0) bytesRead += r;
      }

      if (bytesRead == len) {
        streamSprite.drawJpg(jpgBuffer, len, 0, 0);
        fix_stream_byteswap();
        if (img_stream_display) {
          lv_obj_invalidate(img_stream_display);
        }

        lastStreamFrameMs = millis();
        stream_error = false;

        if (label_stream_status) {
          lv_label_set_text(label_stream_status, "Live stream active");
        }
        ui_update_status("Status: Live stream active");
      }
    }
  }
}

void handle_network()
{
  uint32_t now = millis();

  ensure_wifi();

  if (!streamActive) {
    if (now - lastPollMs >= METRIC_POLL_MS) {
      lastPollMs = now;
      fetch_metrics();
    }

    if (metrics.hasData && label_status) {
      uint32_t age = (now - metrics.lastUpdateMs) / 1000;
      char buf[120];
      snprintf(buf, sizeof(buf),
               "Status: Streaming live data. Last update: %lus ago",
               (unsigned long)age);
      lv_label_set_text(label_status, buf);
    }
  }

  
  if (now - lastCoordsPollMs >= COORDS_POLL_MS) {
    lastCoordsPollMs = now;
    fetch_coords();
  }

  ui_update_connection();
}

void handle_leds() {
  
  if (WiFi.status() != WL_CONNECTED) {
    uint32_t hotPink = strip.Color(255, 20, 147);   
    strip.fill(hotPink);
    strip.show();
    return;
  }

  
  if (!metrics.hasData && !streamActive) {
    strip.clear();
    strip.show();
    return;
  }

  
  float gas = metrics.gas_kohms;

  
  if (isnan(gas) || gas <= 0.0f) {
    strip.clear();
    strip.show();
    return;
  }

  uint32_t targetColor;
  bool solid = false;

  
  if (gas > 25.0f) {
    targetColor = strip.Color(0, 255, 0);    
    solid = true;
  }
  
  else if (gas >= 20.0f) {
    targetColor = strip.Color(255, 255, 0);  
    solid = false;
  }
  
  else {
    targetColor = strip.Color(255, 0, 0);    
    solid = false;
  }

  if (solid) {
    
    strip.fill(targetColor);
    strip.show();
  } else {
    
    unsigned long currentMillis = millis();
    if (currentMillis - ledLastToggle >= BLINK_INTERVAL_MS) {
      ledLastToggle = currentMillis;
      ledState = !ledState;

      if (ledState) {
        strip.fill(targetColor);
      } else {
        strip.clear();
      }
      strip.show();
    }
  }
}

PCA9557 Out;

void setup()
{
  Serial.begin(9600);
  delay(100);

  // frame buffer first, so MJPEG has a home
  jpgBuffer = (uint8_t*)ps_malloc(JPG_BUF_SIZE);
  if (!jpgBuffer) {
    Serial.println("PSRAM Alloc Failed! Using Heap.");
    jpgBuffer = (uint8_t*)malloc(JPG_BUF_SIZE);
  }

  // panel reset + IO expander reset
  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);

  // expander boots the panel rails
  Wire.begin(19, 20);
  Out.reset();
  Out.setMode(IO_OUTPUT);  
  Out.setState(IO0, IO_LOW);
  Out.setState(IO1, IO_LOW);
  delay(20);
  Out.setState(IO0, IO_HIGH);
  delay(100);
  Out.setMode(IO1, IO_INPUT);

  // screen + sprite for the stream tile
  lcd.begin();
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextSize(2);
  delay(200);

  streamSprite.setColorDepth(16);
  streamSprite.createSprite(320, 240);
  streamSprite.fillSprite(TFT_BLACK);

  // local LED strip for status
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.show(); 

  // LVGL + touch
  lv_init();

  touch_init();

  screenWidth  = lcd.width();
  screenHeight = lcd.height();

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL,
                        screenWidth * screenHeight / 15);

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type    = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  
  
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);   
#endif

  init_styles();
  create_main_ui();
  ui_update_status("Status: Connecting to rover...");

  
  lv_timer_create(welcome_countdown_cb, 1000, NULL);
}

void loop()
{
  // main loop keeps Wi‑Fi + UI + LEDs alive
  handle_network();     
  handle_stream();
  handle_leds();
  lv_timer_handler();
  delay(5);
}
