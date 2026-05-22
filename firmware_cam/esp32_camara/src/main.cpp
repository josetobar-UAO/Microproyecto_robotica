/**
 * ESP32-CAM AI Thinker — Servidor MJPEG
 * ══════════════════════════════════════════════════════════════
 * Stream:    http://<IP>:81/stream   ← el nodo ROS2 consume esto
 * Snapshot:  http://<IP>/capture
 * Status:    http://<IP>/status      ← devuelve IP en JSON
 * ══════════════════════════════════════════════════════════════
 */
#include "Arduino.h"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "WiFi.h"
#include "WebServer.h"

// Pines camara AI Thinker ESP-32S
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define LED_GPIO_NUM    4

WebServer server(80);
WebServer stream_server(STREAM_PORT);

static const char* STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char* STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

bool camera_init() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk  = XCLK_GPIO_NUM;  cfg.pin_pclk  = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM; cfg.pin_href  = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM; cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn  = PWDN_GPIO_NUM;  cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    cfg.frame_size   = FRAMESIZE_VGA;   // 640x480
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;
  } else {
    cfg.frame_size   = FRAMESIZE_QVGA;  // 320x240
    cfg.jpeg_quality = 20;
    cfg.fb_count     = 1;
  }

  if (esp_camera_init(&cfg) != ESP_OK) return false;

  sensor_t* s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_gain_ctrl(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  return true;
}

void handle_stream() {
  WiFiClient client = stream_server.client();
  camera_fb_t* fb  = nullptr;
  char part_buf[64];

  stream_server.sendContent(
    "HTTP/1.1 200 OK\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Content-Type: multipart/x-mixed-replace;boundary=frame\r\n\r\n");

  while (client.connected()) {
    fb = esp_camera_fb_get();
    if (!fb) continue;
    stream_server.sendContent(STREAM_BOUNDARY);
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    stream_server.sendContent(part_buf, hlen);
    stream_server.sendContent((const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    delay(33);  // ~30 fps
  }
}

void handle_capture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(500, "text/plain", "Error"); return; }
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handle_status() {
  String json = "{\"ip\":\"" + WiFi.localIP().toString() + "\","
                "\"psram\":" + String(psramFound() ? "true":"false") + "}";
  server.send(200, "application/json", json);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial.println("\n===== ESP32-CAM AI Thinker =====");

  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

  if (!camera_init()) {
    Serial.println("[ERROR] Camara fallo — reiniciando");
    delay(3000); ESP.restart();
  }
  Serial.println("[OK] Camara lista");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[INFO] Conectando WiFi");
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[ERROR] WiFi fallo — reiniciando");
    delay(2000); ESP.restart();
  }

  Serial.printf("\n[OK] IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[OK] Stream: http://%s:%d/stream\n",
                WiFi.localIP().toString().c_str(), STREAM_PORT);

  server.on("/capture", HTTP_GET, handle_capture);
  server.on("/status",  HTTP_GET, handle_status);
  server.begin();

  stream_server.on("/stream", HTTP_GET, handle_stream);
  stream_server.begin();
}

void loop() {
  server.handleClient();
  stream_server.handleClient();
}