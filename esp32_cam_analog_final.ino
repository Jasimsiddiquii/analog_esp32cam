#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "img_converters.h"
#include "esp_arduino_version.h"

#include "mechanical_ocr_types.h"

#ifndef MACHINESENS_MECH_TYPES_V4
#error "Wrong/old mechanical_ocr_types.h. Replace it with the updated one from this package."
#endif

#include "mechanical_templates_int8.h"

static_assert(
    MECH_INT8_TEMPLATE_COUNT == 38 &&
    MECH_INT8_TEMPLATE_H == 64 &&
    MECH_INT8_TEMPLATE_W == 40,
    "Wrong mechanical_templates_int8.h - expected the validated 38 x 40 x 64 header.");



// ============================================================================
// MachineSens - Mechanical Roller Meter ESP32-CAM V4.4 - RED DIGIT SHAPE FIX
// UI matched to the user's existing MachineSens digital-meter setup page.
//
// OCR path is a direct embedded port of the Python reader that passed INT8 parity:
//   - frozen 38-template int8 header (0/127)
//   - Python mask first (S<110), RGB565-safe S<125 only as guarded rescue
//   - same connected-components filters
//   - same 40x64 centering with area-weighted resize
//   - same +/-2 shifted correlation
//   - same targeted 5-vs-6 fix
//
// V2.1 adds:
//   - MachineSens dark two-column UI
//   - VGA live camera on port 81
//   - XGA still capture for OCR
//   - crop canvas + BEFORE/AFTER decimal selectors
//   - camera controls
//   - detailed per-digit result panel
// ============================================================================

const char* AP_SSID = "Mechanical_Meter_Test";
const char* AP_PASSWORD = "12345678";

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define FLASH_LED_PIN 4

constexpr framesize_t STREAM_FRAME_SIZE = FRAMESIZE_VGA;
constexpr int STREAM_JPEG_QUALITY = 11;
constexpr framesize_t CAPTURE_FRAME_SIZE = FRAMESIZE_XGA;
constexpr int CAPTURE_JPEG_QUALITY = 3;

constexpr int DECODE_W = 1024;
constexpr int DECODE_H = 768;

constexpr uint16_t MAIN_SERVER_PORT = 80;
constexpr uint16_t STREAM_SERVER_PORT = 81;

constexpr uint32_t STREAM_STOP_TIMEOUT_MS = 3500;
constexpr uint32_t CAMERA_MUTEX_TIMEOUT_MS = 10000;
constexpr float STILL_DARK_MEAN_THRESHOLD = 82.0f;

httpd_handle_t mainServer = nullptr;
httpd_handle_t streamServer = nullptr;
SemaphoreHandle_t cameraMutex = nullptr;

volatile bool streamPauseRequested = false;
volatile bool streamClientActive = false;

uint8_t* latestJpeg = nullptr;
size_t latestJpegLen = 0;
int latestJpegW = 0;
int latestJpegH = 0;

// Last black/white OCR mask shown on the webpage.
// This is the exact selected preprocessing mask used for the most recent read.
uint8_t* lastOcrMask = nullptr;
int lastOcrMaskW = 0;
int lastOcrMaskH = 0;


struct CameraUiSettings {
  int brightness = 0;
  int contrast = 0;
  int saturation = 0;
  int sharpness = 0;
  int denoise = 0;
  int specialEffect = 0;

  bool whiteBalance = true;
  bool awbGain = true;
  int wbMode = 0;

  bool exposureControl = true;
  bool aec2 = false;
  int aeLevel = 0;
  int aecValue = 300;

  bool gainControl = true;
  int agcGain = 0;
  int gainCeiling = 2;

  bool blackPixelCorrection = true;
  bool whitePixelCorrection = true;
  bool rawGamma = true;
  bool lensCorrection = true;
  bool downsize = true;

  bool horizontalMirror = false;
  bool verticalFlip = false;

  bool flashEnabled = false;
  int flashBrightness = 80;

  int captureWarmupMs = 700;
  int framesToDiscard = 4;
};

CameraUiSettings camUi;

static void clearLastOcrMask() {
  if (lastOcrMask) {
    free(lastOcrMask);
    lastOcrMask = nullptr;
  }

  lastOcrMaskW = 0;
  lastOcrMaskH = 0;
}

static void clearLatestJpeg() {
  clearLastOcrMask();
  if (latestJpeg) {
    free(latestJpeg);
    latestJpeg = nullptr;
  }

  latestJpegLen = 0;
  latestJpegW = 0;
  latestJpegH = 0;
}


static void initialiseFlashPWM() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(FLASH_LED_PIN, 5000, 8);
  ledcWrite(FLASH_LED_PIN, 0);
#else
  constexpr uint8_t FLASH_LEDC_CHANNEL = 7;
  ledcSetup(FLASH_LEDC_CHANNEL, 5000, 8);
  ledcAttachPin(FLASH_LED_PIN, FLASH_LEDC_CHANNEL);
  ledcWrite(FLASH_LEDC_CHANNEL, 0);
#endif
}

static void setFlashBrightness(uint8_t brightness) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(FLASH_LED_PIN, brightness);
#else
  constexpr uint8_t FLASH_LEDC_CHANNEL = 7;
  ledcWrite(FLASH_LEDC_CHANNEL, brightness);
#endif
}

static void applyCameraUiSettings() {
  sensor_t* s = esp_camera_sensor_get();

  if (!s) {
    return;
  }

  if (s->set_brightness) {
    s->set_brightness(s, constrain(camUi.brightness, -2, 2));
  }

  if (s->set_contrast) {
    s->set_contrast(s, constrain(camUi.contrast, -2, 2));
  }

  if (s->set_saturation) {
    s->set_saturation(s, constrain(camUi.saturation, -2, 2));
  }

  if (s->set_sharpness) {
    s->set_sharpness(s, constrain(camUi.sharpness, -2, 2));
  }

  if (s->set_denoise) {
    s->set_denoise(s, constrain(camUi.denoise, 0, 8));
  }

  if (s->set_special_effect) {
    s->set_special_effect(s, constrain(camUi.specialEffect, 0, 6));
  }

  if (s->set_whitebal) {
    s->set_whitebal(s, camUi.whiteBalance ? 1 : 0);
  }

  if (s->set_awb_gain) {
    s->set_awb_gain(s, camUi.awbGain ? 1 : 0);
  }

  if (s->set_wb_mode) {
    s->set_wb_mode(s, constrain(camUi.wbMode, 0, 4));
  }

  if (s->set_exposure_ctrl) {
    s->set_exposure_ctrl(s, camUi.exposureControl ? 1 : 0);
  }

  if (s->set_aec2) {
    s->set_aec2(s, camUi.aec2 ? 1 : 0);
  }

  if (s->set_ae_level) {
    s->set_ae_level(s, constrain(camUi.aeLevel, -2, 2));
  }

  if (s->set_aec_value) {
    s->set_aec_value(s, constrain(camUi.aecValue, 0, 1200));
  }

  if (s->set_gain_ctrl) {
    s->set_gain_ctrl(s, camUi.gainControl ? 1 : 0);
  }

  if (s->set_agc_gain) {
    s->set_agc_gain(s, constrain(camUi.agcGain, 0, 30));
  }

  if (s->set_gainceiling) {
    s->set_gainceiling(
        s,
        static_cast<gainceiling_t>(
            constrain(camUi.gainCeiling, 0, 6)));
  }

  if (s->set_bpc) {
    s->set_bpc(s, camUi.blackPixelCorrection ? 1 : 0);
  }

  if (s->set_wpc) {
    s->set_wpc(s, camUi.whitePixelCorrection ? 1 : 0);
  }

  if (s->set_raw_gma) {
    s->set_raw_gma(s, camUi.rawGamma ? 1 : 0);
  }

  if (s->set_lenc) {
    s->set_lenc(s, camUi.lensCorrection ? 1 : 0);
  }

  if (s->set_dcw) {
    s->set_dcw(s, camUi.downsize ? 1 : 0);
  }

  if (s->set_hmirror) {
    s->set_hmirror(s, camUi.horizontalMirror ? 1 : 0);
  }

  if (s->set_vflip) {
    s->set_vflip(s, camUi.verticalFlip ? 1 : 0);
  }

  // Flash is intentionally OFF during live view.
  // It is applied only during the high-quality still capture.
  setFlashBrightness(0);
}

static bool getQueryString(httpd_req_t* req, const char* key, String& value) {
  const size_t len = httpd_req_get_url_query_len(req);
  if (len == 0) return false;

  char* query = static_cast<char*>(malloc(len + 1));
  if (!query) return false;

  bool ok = false;

  if (httpd_req_get_url_query_str(req, query, len + 1) == ESP_OK) {
    char buffer[64];
    if (httpd_query_key_value(query, key, buffer, sizeof(buffer)) == ESP_OK) {
      value = String(buffer);
      ok = true;
    }
  }

  free(query);
  return ok;
}

static bool getQueryInt(httpd_req_t* req, const char* key, int& value) {
  String text;
  if (!getQueryString(req, key, text)) return false;
  value = text.toInt();
  return true;
}

static esp_err_t sendJson(httpd_req_t* req, const String& json, int status = 200) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  if (status == 400) httpd_resp_set_status(req, "400 Bad Request");
  else if (status == 404) httpd_resp_set_status(req, "404 Not Found");
  else if (status == 500) httpd_resp_set_status(req, "500 Internal Server Error");

  return httpd_resp_send(req, json.c_str(), json.length());
}


// ============================================================================
// FINAL MECHANICAL INT8 OCR
//
// Desktop parity already validated:
//   ORIGINAL PYTHON == INT8 PYTHON
//
// Embedded path:
//   retained still JPEG -> RGB565 -> crop
//   -> Python S<110 mask
//   -> guarded S<125 rescue only when RGB565 quantization needs it
//   -> Otsu V threshold, clamp 65..135
//   -> 2x2 close
//   -> 8-connected components
//   -> 40x64 area-resized normalized digits
//   -> +/-2 shifted Pearson correlation against frozen INT8 templates
//   -> exact targeted 5-vs-6 hole rule
// ============================================================================

static void mechSetError(
    MechanicalOcrResult* out,
    const char* message) {
  if (!out) {
    return;
  }

  out->success = false;

  strncpy(
      out->error,
      message ? message : "Unknown mechanical OCR error",
      sizeof(out->error) - 1);

  out->error[
      sizeof(out->error) - 1] =
      '\0';
}

static inline uint8_t mechR5To8(uint16_t pixel) {
  return static_cast<uint8_t>(
      ((((pixel >> 11) & 0x1F) * 255U) / 31U));
}

static inline uint8_t mechG6To8(uint16_t pixel) {
  return static_cast<uint8_t>(
      ((((pixel >> 5) & 0x3F) * 255U) / 63U));
}

static inline uint8_t mechB5To8(uint16_t pixel) {
  return static_cast<uint8_t>(
      (((pixel & 0x1F) * 255U) / 31U));
}

// OpenCV HSV S/V equivalent needed by the Python preprocessing.
static inline void mechSVFromRgb565(
    uint16_t pixel,
    uint8_t& saturation,
    uint8_t& value) {
  const uint8_t r = mechR5To8(pixel);
  const uint8_t g = mechG6To8(pixel);
  const uint8_t b = mechB5To8(pixel);

  const uint8_t maximum =
      max(r, max(g, b));

  const uint8_t minimum =
      min(r, min(g, b));

  value =
      maximum;

  saturation =
      (maximum == 0)
          ? 0
          : static_cast<uint8_t>(
                (static_cast<uint16_t>(
                     maximum - minimum) *
                 255U) /
                maximum);
}

static int mechOtsuThreshold(
    const uint32_t histogram[256],
    uint32_t totalPixels) {
  if (totalPixels == 0) {
    return 90;
  }

  uint64_t totalWeighted = 0;

  for (int i = 0;
       i < 256;
       ++i) {
    totalWeighted +=
        static_cast<uint64_t>(i) *
        histogram[i];
  }

  uint64_t backgroundWeighted = 0;
  uint32_t backgroundCount = 0;

  double bestVariance = -1.0;
  int bestThreshold = 90;

  for (int threshold = 0;
       threshold < 256;
       ++threshold) {
    backgroundCount +=
        histogram[threshold];

    if (backgroundCount == 0) {
      continue;
    }

    const uint32_t foregroundCount =
        totalPixels -
        backgroundCount;

    if (foregroundCount == 0) {
      break;
    }

    backgroundWeighted +=
        static_cast<uint64_t>(
            threshold) *
        histogram[threshold];

    const double meanBackground =
        static_cast<double>(
            backgroundWeighted) /
        backgroundCount;

    const double meanForeground =
        static_cast<double>(
            totalWeighted -
            backgroundWeighted) /
        foregroundCount;

    const double difference =
        meanBackground -
        meanForeground;

    const double variance =
        static_cast<double>(
            backgroundCount) *
        static_cast<double>(
            foregroundCount) *
        difference *
        difference;

    if (variance > bestVariance) {
      bestVariance =
          variance;

      bestThreshold =
          threshold;
    }
  }

  return bestThreshold;
}

static uint8_t* mechMakeMask(
    const uint16_t* frame,
    int frameWidth,
    int frameHeight,
    int roiX,
    int roiY,
    int roiWidth,
    int roiHeight,
    int saturationLimit) {
  if (!frame ||
      frameWidth <= 0 ||
      frameHeight <= 0 ||
      roiWidth <= 0 ||
      roiHeight <= 0) {
    return nullptr;
  }

  roiX =
      max(0, roiX);

  roiY =
      max(0, roiY);

  if (roiX + roiWidth >
      frameWidth) {
    roiWidth =
        frameWidth -
        roiX;
  }

  if (roiY + roiHeight >
      frameHeight) {
    roiHeight =
        frameHeight -
        roiY;
  }

  if (roiWidth <= 0 ||
      roiHeight <= 0) {
    return nullptr;
  }

  uint32_t histogram[256] = {0};
  uint32_t valueCount = 0;

  // Python:
  // values = val[sat < 110]
  for (int y = 0;
       y < roiHeight;
       ++y) {
    const uint16_t* row =
        frame +
        (roiY + y) *
            frameWidth +
        roiX;

    for (int x = 0;
         x < roiWidth;
         ++x) {
      uint8_t saturation;
      uint8_t value;

      mechSVFromRgb565(
          row[x],
          saturation,
          value);

      if (saturation <
          saturationLimit) {
        histogram[value]++;
        valueCount++;
      }
    }
  }

  int threshold =
      90;

  if (valueCount > 100) {
    threshold =
        mechOtsuThreshold(
            histogram,
            valueCount);
  }

  threshold =
      constrain(
          threshold,
          65,
          135);

  const size_t pixelCount =
      static_cast<size_t>(
          roiWidth) *
      roiHeight;

  uint8_t* mask =
      static_cast<uint8_t*>(
          heap_caps_malloc(
              pixelCount,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  uint8_t* temporary =
      static_cast<uint8_t*>(
          heap_caps_malloc(
              pixelCount,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  if (!mask ||
      !temporary) {
    free(mask);
    free(temporary);

    return nullptr;
  }

  // Python:
  // mask = ((sat < limit) & (val > threshold)) * 255
  for (int y = 0;
       y < roiHeight;
       ++y) {
    const uint16_t* row =
        frame +
        (roiY + y) *
            frameWidth +
        roiX;

    uint8_t* output =
        mask +
        y * roiWidth;

    for (int x = 0;
         x < roiWidth;
         ++x) {
      uint8_t saturation;
      uint8_t value;

      mechSVFromRgb565(
          row[x],
          saturation,
          value);

      output[x] =
          (saturation <
               saturationLimit &&
           value >
               threshold)
              ? 255
              : 0;
    }
  }

  // Python:
  // cv2.morphologyEx(mask, MORPH_CLOSE, ones((2,2)))
  //
  // 2x2 dilation, anchor=(1,1)
  for (int y = 0;
       y < roiHeight;
       ++y) {
    for (int x = 0;
         x < roiWidth;
         ++x) {
      uint8_t value = 0;

      for (int dy = -1;
           dy <= 0 &&
           value == 0;
           ++dy) {
        const int yy =
            y + dy;

        if (yy < 0 ||
            yy >= roiHeight) {
          continue;
        }

        for (int dx = -1;
             dx <= 0;
             ++dx) {
          const int xx =
              x + dx;

          if (xx < 0 ||
              xx >= roiWidth) {
            continue;
          }

          if (mask[
                  yy *
                      roiWidth +
                  xx]) {
            value =
                255;

            break;
          }
        }
      }

      temporary[
          y *
              roiWidth +
          x] =
          value;
    }
  }

  // 2x2 erosion.
  for (int y = 0;
       y < roiHeight;
       ++y) {
    for (int x = 0;
         x < roiWidth;
         ++x) {
      uint8_t value =
          255;

      for (int dy = -1;
           dy <= 0 &&
           value != 0;
           ++dy) {
        const int yy =
            y + dy;

        if (yy < 0 ||
            yy >= roiHeight) {
          continue;
        }

        for (int dx = -1;
             dx <= 0;
             ++dx) {
          const int xx =
              x + dx;

          if (xx < 0 ||
              xx >= roiWidth) {
            continue;
          }

          if (!temporary[
                  yy *
                      roiWidth +
                  xx]) {
            value =
                0;

            break;
          }
        }
      }

      mask[
          y *
              roiWidth +
          x] =
          value;
    }
  }

  free(temporary);

  return mask;
}


// ============================================================================
// COLOR-SAFE WHITE-INK MASK
//
// Designed for roller digits printed in white over RED / ORANGE / GREEN /
// BLACK backgrounds.
//
// Key idea:
//   white digit      -> R, G and B are all high
//   red background   -> R can be high, but G/B are much lower
//
// Therefore min(R,G,B) isolates the white printed number without turning the
// colored roller background into a solid white rectangle.
// ============================================================================
static uint8_t* mechMakeWhiteInkMask(
    const uint16_t* frame,
    int frameWidth,
    int frameHeight,
    int roiX,
    int roiY,
    int roiWidth,
    int roiHeight) {
  if (!frame ||
      frameWidth <= 0 ||
      frameHeight <= 0 ||
      roiWidth <= 0 ||
      roiHeight <= 0) {
    return nullptr;
  }

  roiX =
      max(0, roiX);

  roiY =
      max(0, roiY);

  if (roiX + roiWidth >
      frameWidth) {
    roiWidth =
        frameWidth -
        roiX;
  }

  if (roiY + roiHeight >
      frameHeight) {
    roiHeight =
        frameHeight -
        roiY;
  }

  if (roiWidth <= 0 ||
      roiHeight <= 0) {
    return nullptr;
  }

  uint32_t histogram[256] = {0};
  uint32_t count = 0;

  // Histogram of min(R,G,B).
  // A colored background is naturally suppressed because at least one
  // channel is low; white ink stays bright in all three channels.
  for (int y = 0;
       y < roiHeight;
       ++y) {
    const uint16_t* row =
        frame +
        (roiY + y) *
            frameWidth +
        roiX;

    for (int x = 0;
         x < roiWidth;
         ++x) {
      const uint8_t r =
          mechR5To8(
              row[x]);

      const uint8_t g =
          mechG6To8(
              row[x]);

      const uint8_t b =
          mechB5To8(
              row[x]);

      const uint8_t whiteLevel =
          min(
              r,
              min(
                  g,
                  b));

      histogram[
          whiteLevel]++;

      count++;
    }
  }

  int threshold =
      115;

  if (count > 100) {
    threshold =
        mechOtsuThreshold(
            histogram,
            count);
  }

  // Keep threshold practical for OV2640 stills:
  // too low admits colored roller paint;
  // too high can erase dim white digits.
  threshold =
      constrain(
          threshold,
          70,
          190);

  const size_t pixelCount =
      static_cast<size_t>(
          roiWidth) *
      roiHeight;

  uint8_t* mask =
      static_cast<uint8_t*>(
          heap_caps_malloc(
              pixelCount,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  uint8_t* temporary =
      static_cast<uint8_t*>(
          heap_caps_malloc(
              pixelCount,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  if (!mask ||
      !temporary) {
    free(mask);
    free(temporary);

    return nullptr;
  }

  // Binary mask from min(R,G,B).
  for (int y = 0;
       y < roiHeight;
       ++y) {
    const uint16_t* row =
        frame +
        (roiY + y) *
            frameWidth +
        roiX;

    uint8_t* output =
        mask +
        y *
            roiWidth;

    for (int x = 0;
         x < roiWidth;
         ++x) {
      const uint8_t r =
          mechR5To8(
              row[x]);

      const uint8_t g =
          mechG6To8(
              row[x]);

      const uint8_t b =
          mechB5To8(
              row[x]);

      const uint8_t whiteLevel =
          min(
              r,
              min(
                  g,
                  b));

      output[x] =
          (whiteLevel >
           threshold)
              ? 255
              : 0;
    }
  }

  // Same 2x2 MORPH_CLOSE used by the Python-style mask.
  // Dilation, anchor=(1,1).
  for (int y = 0;
       y < roiHeight;
       ++y) {
    for (int x = 0;
         x < roiWidth;
         ++x) {
      uint8_t value = 0;

      for (int dy = -1;
           dy <= 0 &&
           value == 0;
           ++dy) {
        const int yy =
            y + dy;

        if (yy < 0 ||
            yy >= roiHeight) {
          continue;
        }

        for (int dx = -1;
             dx <= 0;
             ++dx) {
          const int xx =
              x + dx;

          if (xx < 0 ||
              xx >= roiWidth) {
            continue;
          }

          if (mask[
                  yy *
                      roiWidth +
                  xx]) {
            value =
                255;

            break;
          }
        }
      }

      temporary[
          y *
              roiWidth +
          x] =
          value;
    }
  }

  // Erosion.
  for (int y = 0;
       y < roiHeight;
       ++y) {
    for (int x = 0;
         x < roiWidth;
         ++x) {
      uint8_t value =
          255;

      for (int dy = -1;
           dy <= 0 &&
           value != 0;
           ++dy) {
        const int yy =
            y + dy;

        if (yy < 0 ||
            yy >= roiHeight) {
          continue;
        }

        for (int dx = -1;
             dx <= 0;
             ++dx) {
          const int xx =
              x + dx;

          if (xx < 0 ||
              xx >= roiWidth) {
            continue;
          }

          if (!temporary[
                  yy *
                      roiWidth +
                  xx]) {
            value =
                0;

            break;
          }
        }
      }

      mask[
          y *
              roiWidth +
          x] =
          value;
    }
  }

  free(temporary);

  return mask;
}

static void mechSortComponentsByX(
    MechanicalComponent* components,
    int count) {
  for (int i = 0;
       i < count - 1;
       ++i) {
    for (int j = i + 1;
         j < count;
         ++j) {
      if (components[j].x <
          components[i].x) {
        MechanicalComponent temporary =
            components[i];

        components[i] =
            components[j];

        components[j] =
            temporary;
      }
    }
  }
}

static int mechFindDigits(
    uint8_t* mask,
    int width,
    int height,
    MechanicalComponent output[
        MECH_MAX_COMPONENTS]) {
  if (!mask ||
      width <= 0 ||
      height <= 0) {
    return 0;
  }

  // Exact Python:
  // min_area = max(50, int(H*W*0.0005))
  const int minimumArea =
      max(
          50,
          static_cast<int>(
              static_cast<float>(
                  height) *
              static_cast<float>(
                  width) *
              0.0005f));

  int32_t* stack =
      static_cast<int32_t*>(
          heap_caps_malloc(
              sizeof(int32_t) *
                  MECH_FLOOD_STACK_CAP,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  if (!stack) {
    return -1;
  }

  static const int8_t deltaX[8] = {
      -1, 0, 1,
      -1,    1,
      -1, 0, 1};

  static const int8_t deltaY[8] = {
      -1, -1, -1,
       0,      0,
       1,  1,  1};

  int outputCount = 0;
  bool overflowSeen = false;

  for (int startY = 0;
       startY < height;
       ++startY) {
    for (int startX = 0;
         startX < width;
         ++startX) {
      const int startIndex =
          startY *
              width +
          startX;

      if (mask[startIndex] !=
          255) {
        continue;
      }

      int stackTop = 0;

      stack[stackTop++] =
          startIndex;

      mask[startIndex] =
          128;

      int minimumX =
          startX;

      int maximumX =
          startX;

      int minimumY =
          startY;

      int maximumY =
          startY;

      int area = 0;
      bool overflow = false;

      while (stackTop > 0) {
        const int index =
            stack[--stackTop];

        const int x =
            index %
            width;

        const int y =
            index /
            width;

        area++;

        minimumX =
            min(
                minimumX,
                x);

        maximumX =
            max(
                maximumX,
                x);

        minimumY =
            min(
                minimumY,
                y);

        maximumY =
            max(
                maximumY,
                y);

        for (int neighbour = 0;
             neighbour < 8;
             ++neighbour) {
          const int nextX =
              x +
              deltaX[neighbour];

          const int nextY =
              y +
              deltaY[neighbour];

          if (nextX < 0 ||
              nextX >= width ||
              nextY < 0 ||
              nextY >= height) {
            continue;
          }

          const int nextIndex =
              nextY *
                  width +
              nextX;

          if (mask[nextIndex] !=
              255) {
            continue;
          }

          mask[nextIndex] =
              128;

          if (stackTop <
              MECH_FLOOD_STACK_CAP) {
            stack[stackTop++] =
                nextIndex;
          }

          else {
            overflow =
                true;
          }
        }
      }

      if (overflow) {
        overflowSeen =
            true;

        continue;
      }

      const int componentWidth =
          maximumX -
          minimumX +
          1;

      const int componentHeight =
          maximumY -
          minimumY +
          1;

      const float centreY =
          minimumY +
          componentHeight /
              2.0f;

      // Exact Python component filters.
      const bool accepted =
          componentHeight >=
              max(
                  20,
                  static_cast<int>(
                      height *
                      0.25f)) &&
          componentHeight <=
              static_cast<int>(
                  height *
                  0.85f) &&
          componentWidth >=
              5 &&
          componentWidth <=
              static_cast<int>(
                  width *
                  0.18f) &&
          area >=
              minimumArea &&
          centreY >
              height *
                  0.18f &&
          centreY <
              height *
                  0.82f;

      if (accepted &&
          outputCount <
              MECH_MAX_COMPONENTS) {
        output[outputCount++] = {
            minimumX,
            minimumY,
            componentWidth,
            componentHeight,
            area};
      }
    }
  }

  const size_t pixelCount =
      static_cast<size_t>(
          width) *
      height;

  for (size_t i = 0;
       i < pixelCount;
       ++i) {
    if (mask[i] ==
        128) {
      mask[i] =
          255;
    }
  }

  free(stack);

  if (overflowSeen &&
      outputCount == 0) {
    return -2;
  }

  mechSortComponentsByX(
      output,
      outputCount);

  return outputCount;
}

// Area-weighted sampling to match cv2.INTER_AREA much more closely than
// the old bilinear embedded approximation.
static uint8_t mechAreaResizeSample(
    const uint8_t* source,
    int sourceWidth,
    int sourceHeight,
    int destinationX,
    int destinationY,
    int destinationWidth,
    int destinationHeight) {
  const float x0 =
      static_cast<float>(
          destinationX) *
      sourceWidth /
      destinationWidth;

  const float x1 =
      static_cast<float>(
          destinationX + 1) *
      sourceWidth /
      destinationWidth;

  const float y0 =
      static_cast<float>(
          destinationY) *
      sourceHeight /
      destinationHeight;

  const float y1 =
      static_cast<float>(
          destinationY + 1) *
      sourceHeight /
      destinationHeight;

  const int sourceX0 =
      max(
          0,
          static_cast<int>(
              floorf(x0)));

  const int sourceX1 =
      min(
          sourceWidth - 1,
          static_cast<int>(
              ceilf(x1) - 1));

  const int sourceY0 =
      max(
          0,
          static_cast<int>(
              floorf(y0)));

  const int sourceY1 =
      min(
          sourceHeight - 1,
          static_cast<int>(
              ceilf(y1) - 1));

  float weightedSum =
      0.0f;

  float totalWeight =
      0.0f;

  for (int sourceY = sourceY0;
       sourceY <= sourceY1;
       ++sourceY) {
    const float weightY =
        max(
            0.0f,
            min(
                y1,
                static_cast<float>(
                    sourceY + 1)) -
            max(
                y0,
                static_cast<float>(
                    sourceY)));

    if (weightY <= 0.0f) {
      continue;
    }

    for (int sourceX = sourceX0;
         sourceX <= sourceX1;
         ++sourceX) {
      const float weightX =
          max(
              0.0f,
              min(
                  x1,
                  static_cast<float>(
                      sourceX + 1)) -
              max(
                  x0,
                  static_cast<float>(
                      sourceX)));

      if (weightX <= 0.0f) {
        continue;
      }

      const float weight =
          weightX *
          weightY;

      weightedSum +=
          source[
              sourceY *
                  sourceWidth +
              sourceX] *
          weight;

      totalWeight +=
          weight;
    }
  }

  if (totalWeight <= 0.0f) {
    return 0;
  }

  return static_cast<uint8_t>(
      roundf(
          weightedSum /
          totalWeight));
}

static bool mechNormalizeDigit(
    const uint8_t* mask,
    int maskWidth,
    int maskHeight,
    const MechanicalComponent& component,
    uint8_t output[
        MECH_INT8_TEMPLATE_PIXELS]) {
  memset(
      output,
      0,
      MECH_INT8_TEMPLATE_PIXELS);

  int left =
      component.x +
      component.w;

  int right =
      component.x - 1;

  int top =
      component.y +
      component.h;

  int bottom =
      component.y - 1;

  // Python tightens component to non-zero pixels.
  for (int y = component.y;
       y < component.y +
               component.h;
       ++y) {
    if (y < 0 ||
        y >= maskHeight) {
      continue;
    }

    for (int x = component.x;
         x < component.x +
                 component.w;
         ++x) {
      if (x < 0 ||
          x >= maskWidth) {
        continue;
      }

      if (mask[
              y *
                  maskWidth +
              x] >
          0) {
        left =
            min(
                left,
                x);

        right =
            max(
                right,
                x);

        top =
            min(
                top,
                y);

        bottom =
            max(
                bottom,
                y);
      }
    }
  }

  if (right < left ||
      bottom < top) {
    return false;
  }

  const int sourceWidth =
      right -
      left +
      1;

  const int sourceHeight =
      bottom -
      top +
      1;

  // Exact Python target:
  // scale = min(34/w, 58/h)
  const float scale =
      min(
          34.0f /
              max(
                  1,
                  sourceWidth),
          58.0f /
              max(
                  1,
                  sourceHeight));

  const int newWidth =
      max(
          1,
          static_cast<int>(
              roundf(
                  sourceWidth *
                  scale)));

  const int newHeight =
      max(
          1,
          static_cast<int>(
              roundf(
                  sourceHeight *
                  scale)));

  const int offsetX =
      (MECH_INT8_TEMPLATE_W -
       newWidth) /
      2;

  const int offsetY =
      (MECH_INT8_TEMPLATE_H -
       newHeight) /
      2;

  const size_t cropBytes =
      static_cast<size_t>(
          sourceWidth) *
      sourceHeight;

  uint8_t* crop =
      static_cast<uint8_t*>(
          heap_caps_malloc(
              cropBytes,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  if (!crop) {
    return false;
  }

  for (int y = 0;
       y < sourceHeight;
       ++y) {
    memcpy(
        crop +
            y *
                sourceWidth,
        mask +
            (top + y) *
                maskWidth +
            left,
        sourceWidth);
  }

  for (int destinationY = 0;
       destinationY < newHeight;
       ++destinationY) {
    for (int destinationX = 0;
         destinationX < newWidth;
         ++destinationX) {
      output[
          (offsetY +
           destinationY) *
              MECH_INT8_TEMPLATE_W +
          (offsetX +
           destinationX)] =
          mechAreaResizeSample(
              crop,
              sourceWidth,
              sourceHeight,
              destinationX,
              destinationY,
              newWidth,
              newHeight);
    }
  }

  free(crop);

  return true;
}

// Exact Pearson correlation math, using the validated INT8 template values.
// No float 0..1 conversion is required because correlation is invariant to
// positive scaling.
static void mechTemplateStats(
    int templateIndex,
    int64_t& sumReference,
    int64_t& sumReferenceSquared) {
  sumReference = 0;
  sumReferenceSquared = 0;

  // Calculated from the user's exact Python-generated INT8 header.
  // This keeps mechanical_templates_int8.h completely untouched.
  for (int pixelIndex = 0;
       pixelIndex < MECH_INT8_TEMPLATE_PIXELS;
       ++pixelIndex) {
    const int32_t referenceValue =
        mechInt8TemplatePixel(
            templateIndex,
            pixelIndex);

    sumReference +=
        referenceValue;

    sumReferenceSquared +=
        static_cast<int64_t>(
            referenceValue) *
        referenceValue;
  }
}

static float mechCorrelationShifted(
    const uint8_t test[
        MECH_INT8_TEMPLATE_PIXELS],
    int templateIndex,
    int shiftX,
    int shiftY,
    int64_t sumReference,
    int64_t sumReferenceSquared) {
  constexpr int64_t N =
      MECH_INT8_TEMPLATE_PIXELS;

  int64_t sumTest = 0;
  int64_t sumTestSquared = 0;
  int64_t sumProduct = 0;

  for (int y = 0;
       y < MECH_INT8_TEMPLATE_H;
       ++y) {
    const int sourceY =
        y -
        shiftY;

    for (int x = 0;
         x < MECH_INT8_TEMPLATE_W;
         ++x) {
      const int sourceX =
          x -
          shiftX;

      int32_t testValue = 0;

      if (sourceX >= 0 &&
          sourceX < MECH_INT8_TEMPLATE_W &&
          sourceY >= 0 &&
          sourceY < MECH_INT8_TEMPLATE_H) {
        testValue =
            test[
                sourceY *
                    MECH_INT8_TEMPLATE_W +
                sourceX];
      }

      const int pixelIndex =
          y *
              MECH_INT8_TEMPLATE_W +
          x;

      const int32_t referenceValue =
          mechInt8TemplatePixel(
              templateIndex,
              pixelIndex);

      sumTest +=
          testValue;

      sumTestSquared +=
          static_cast<int64_t>(
              testValue) *
          testValue;

      sumProduct +=
          static_cast<int64_t>(
              testValue) *
          referenceValue;
    }
  }

  const int64_t numerator =
      N *
          sumProduct -
      sumTest *
          sumReference;

  const int64_t varianceTest =
      N *
          sumTestSquared -
      sumTest *
          sumTest;

  const int64_t varianceReference =
      N *
          sumReferenceSquared -
      sumReference *
          sumReference;

  if (varianceTest <= 0 ||
      varianceReference <= 0) {
    return -1.0f;
  }

  const double denominator =
      sqrt(
          static_cast<double>(
              varianceTest) *
          static_cast<double>(
              varianceReference));

  if (denominator < 1e-9) {
    return -1.0f;
  }

  return static_cast<float>(
      static_cast<double>(
          numerator) /
      denominator);
}

static float mechShiftedScore(
    const uint8_t test[
        MECH_INT8_TEMPLATE_PIXELS],
    int templateIndex) {
  float bestScore =
      -1.0f;

  // Compute template mean/variance statistics once, then reuse them
  // across all 25 shifts.
  int64_t sumReference = 0;
  int64_t sumReferenceSquared = 0;

  mechTemplateStats(
      templateIndex,
      sumReference,
      sumReferenceSquared);

  // Exact Python range(-2, 3).
  for (int shiftY = -2;
       shiftY <= 2;
       ++shiftY) {
    for (int shiftX = -2;
         shiftX <= 2;
         ++shiftX) {
      bestScore =
          max(
              bestScore,
              mechCorrelationShifted(
                  test,
                  templateIndex,
                  shiftX,
                  shiftY,
                  sumReference,
                  sumReferenceSquared));
    }
  }

  return bestScore;
}

static int mechCountHoles(
    const uint8_t image[
        MECH_INT8_TEMPLATE_PIXELS]) {
  uint8_t* inverted =
      static_cast<uint8_t*>(
          heap_caps_malloc(
              MECH_INT8_TEMPLATE_PIXELS,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  int32_t* stack =
      static_cast<int32_t*>(
          heap_caps_malloc(
              sizeof(int32_t) *
                  MECH_INT8_TEMPLATE_PIXELS,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  if (!inverted ||
      !stack) {
    free(inverted);
    free(stack);

    return 0;
  }

  // Exact Python:
  // binary = img > 127
  // inverted = 1 - binary
  for (int i = 0;
       i < MECH_INT8_TEMPLATE_PIXELS;
       ++i) {
    inverted[i] =
        (image[i] >
         127)
            ? 0
            : 1;
  }

  static const int8_t deltaX[8] = {
      -1, 0, 1,
      -1,    1,
      -1, 0, 1};

  static const int8_t deltaY[8] = {
      -1, -1, -1,
       0,      0,
       1,  1,  1};

  const int minimumArea =
      max(
          5,
          static_cast<int>(
              MECH_INT8_TEMPLATE_H *
              MECH_INT8_TEMPLATE_W *
              0.01f));

  int holes = 0;

  for (int startY = 0;
       startY < MECH_INT8_TEMPLATE_H;
       ++startY) {
    for (int startX = 0;
         startX < MECH_INT8_TEMPLATE_W;
         ++startX) {
      const int startIndex =
          startY *
              MECH_INT8_TEMPLATE_W +
          startX;

      if (inverted[startIndex] !=
          1) {
        continue;
      }

      int stackTop = 0;

      stack[stackTop++] =
          startIndex;

      inverted[startIndex] =
          2;

      int minimumX =
          startX;

      int maximumX =
          startX;

      int minimumY =
          startY;

      int maximumY =
          startY;

      int area = 0;

      while (stackTop > 0) {
        const int index =
            stack[--stackTop];

        const int x =
            index %
            MECH_INT8_TEMPLATE_W;

        const int y =
            index /
            MECH_INT8_TEMPLATE_W;

        area++;

        minimumX =
            min(
                minimumX,
                x);

        maximumX =
            max(
                maximumX,
                x);

        minimumY =
            min(
                minimumY,
                y);

        maximumY =
            max(
                maximumY,
                y);

        for (int neighbour = 0;
             neighbour < 8;
             ++neighbour) {
          const int nextX =
              x +
              deltaX[neighbour];

          const int nextY =
              y +
              deltaY[neighbour];

          if (nextX < 0 ||
              nextX >=
                  MECH_INT8_TEMPLATE_W ||
              nextY < 0 ||
              nextY >=
                  MECH_INT8_TEMPLATE_H) {
            continue;
          }

          const int nextIndex =
              nextY *
                  MECH_INT8_TEMPLATE_W +
              nextX;

          if (inverted[nextIndex] ==
              1) {
            inverted[nextIndex] =
                2;

            stack[stackTop++] =
                nextIndex;
          }
        }
      }

      const bool touchesEdge =
          minimumX <= 0 ||
          minimumY <= 0 ||
          maximumX >=
              MECH_INT8_TEMPLATE_W - 1 ||
          maximumY >=
              MECH_INT8_TEMPLATE_H - 1;

      if (!touchesEdge &&
          area >=
              minimumArea) {
        holes++;
      }
    }
  }

  free(inverted);
  free(stack);

  return holes;
}


static bool mechShapeStronglyLooksLike2(
    const uint8_t image[
        MECH_INT8_TEMPLATE_PIXELS]) {
  // Tight ink bounding box first, so the test is independent of centering.
  int minX =
      MECH_INT8_TEMPLATE_W;

  int maxX =
      -1;

  int minY =
      MECH_INT8_TEMPLATE_H;

  int maxY =
      -1;

  for (int y = 0;
       y < MECH_INT8_TEMPLATE_H;
       ++y) {
    for (int x = 0;
         x < MECH_INT8_TEMPLATE_W;
         ++x) {
      if (image[
              y *
                  MECH_INT8_TEMPLATE_W +
              x] >
          127) {
        minX =
            min(
                minX,
                x);

        maxX =
            max(
                maxX,
                x);

        minY =
            min(
                minY,
                y);

        maxY =
            max(
                maxY,
                y);
      }
    }
  }

  if (maxX <= minX ||
      maxY <= minY) {
    return false;
  }

  const int width =
      maxX -
      minX +
      1;

  const int height =
      maxY -
      minY +
      1;

  auto bandCentroid =
      [&](float yStart,
          float yEnd,
          float& centroid) -> bool {
        int startY =
            minY +
            static_cast<int>(
                floorf(
                    height *
                    yStart));

        int endY =
            minY +
            static_cast<int>(
                ceilf(
                    height *
                    yEnd));

        startY =
            constrain(
                startY,
                minY,
                maxY);

        endY =
            constrain(
                endY,
                startY + 1,
                maxY + 1);

        uint32_t xSum = 0;
        uint32_t count = 0;

        for (int y = startY;
             y < endY;
             ++y) {
          for (int x = minX;
               x <= maxX;
               ++x) {
            if (image[
                    y *
                        MECH_INT8_TEMPLATE_W +
                    x] >
                127) {
              xSum +=
                  static_cast<uint32_t>(
                      x -
                      minX);

              count++;
            }
          }
        }

        if (count == 0) {
          return false;
        }

        centroid =
            static_cast<float>(
                xSum) /
            count /
            max(
                1,
                width - 1);

        return true;
      };

  float middleCentroid =
      1.0f;

  float lowerCentroid =
      1.0f;

  // These bands are intentionally away from the top/bottom horizontal bars.
  if (!bandCentroid(
          0.50f,
          0.65f,
          middleCentroid) ||
      !bandCentroid(
          0.68f,
          0.84f,
          lowerCentroid)) {
    return false;
  }

  // A "2" bends left strongly in the lower half.
  // A "3" remains right-heavy through both middle/lower bands.
  return
      middleCentroid <=
          0.61f &&
      lowerCentroid <=
          0.50f;
}

static void mechClassifyDigit(
    const uint8_t normalized[
        MECH_INT8_TEMPLATE_PIXELS],
    char& bestDigit,
    float& bestScore,
    char& secondDigit,
    float& secondScore) {
  float scores[10];

  for (int digit = 0;
       digit < 10;
       ++digit) {
    scores[digit] =
        -1.0f;
  }

  // Python:
  // for digit 0..9:
  //   max shifted_score across all templates of that label
  for (int templateIndex = 0;
       templateIndex <
           MECH_INT8_TEMPLATE_COUNT;
       ++templateIndex) {
    const char label =
        mechInt8TemplateLabel(
            templateIndex);

    if (label < '0' ||
        label > '9') {
      continue;
    }

    const int digit =
        label -
        '0';

    const float score =
        mechShiftedScore(
            normalized,
            templateIndex);

    if (score >
        scores[digit]) {
      scores[digit] =
          score;
    }
  }

  int best =
      0;

  for (int digit = 1;
       digit < 10;
       ++digit) {
    if (scores[digit] >
        scores[best]) {
      best =
          digit;
    }
  }

  bestDigit =
      static_cast<char>(
          '0' +
          best);

  bestScore =
      scores[best];

  // Exact Python targeted 5-vs-6 correction.
  if (bestDigit == '5' ||
      bestDigit == '6') {
    const int holes =
        mechCountHoles(
            normalized);

    const float score5 =
        scores[5];

    const float score6 =
        scores[6];

    if (holes == 0 &&
        score5 >=
            score6 -
            0.08f) {
      bestDigit =
          '5';

      bestScore =
          score5;
    }

    else if (
        holes >= 1 &&
        score6 >=
            score5 -
            0.08f) {
      bestDigit =
          '6';

      bestScore =
          score6;
    }
  }

  // ESP32 roller-font rescue: 2 vs 3.
  //
  // In the red-background test image, the WHITE-INK mask clearly contains
  // the shape of "2", but correlation can rank the unusual roller-font stroke
  // as "3". Only override a LOW/MEDIUM-confidence 3 when the lower-half
  // trajectory strongly matches a 2.
  if (bestDigit == '3' &&
      bestScore <
          0.78f &&
      mechShapeStronglyLooksLike2(
          normalized)) {
    Serial.printf(
        "MECH SHAPE RESCUE 3->2 | score3=%.3f score2=%.3f\n",
        bestScore,
        scores[2]);

    bestDigit =
        '2';

    bestScore =
        scores[2];
  }

  const int chosenDigit =
      bestDigit -
      '0';

  int second =
      -1;

  for (int digit = 0;
       digit < 10;
       ++digit) {
    if (digit ==
        chosenDigit) {
      continue;
    }

    if (second < 0 ||
        scores[digit] >
            scores[second]) {
      second =
          digit;
    }
  }

  if (second < 0) {
    second =
        chosenDigit;
  }

  secondDigit =
      static_cast<char>(
          '0' +
          second);

  secondScore =
      scores[second];
}

static bool mechRunSingleMask(
    const uint16_t* frame,
    int frameWidth,
    int frameHeight,
    int roiX,
    int roiY,
    int roiWidth,
    int roiHeight,
    int before,
    int after,
    int saturationLimit,
    bool useWhiteInkMask,
    MechanicalOcrResult* out) {
  if (!out) {
    return false;
  }

  memset(
      out,
      0,
      sizeof(*out));

  const int expectedDigits =
      before +
      after;

  out->expectedDigits =
      expectedDigits;

  out->saturationLimitUsed =
      saturationLimit;

  out->whiteInkMaskUsed =
      useWhiteInkMask;

  uint8_t* mask =
      useWhiteInkMask
          ? mechMakeWhiteInkMask(
                frame,
                frameWidth,
                frameHeight,
                roiX,
                roiY,
                roiWidth,
                roiHeight)
          : mechMakeMask(
                frame,
                frameWidth,
                frameHeight,
                roiX,
                roiY,
                roiWidth,
                roiHeight,
                saturationLimit);

  if (!mask) {
    mechSetError(
        out,
        "Could not allocate/build mechanical digit mask");

    return false;
  }

  MechanicalComponent components[
      MECH_MAX_COMPONENTS];

  const int componentCount =
      mechFindDigits(
          mask,
          roiWidth,
          roiHeight,
          components);

  if (componentCount == -1) {
    free(mask);

    mechSetError(
        out,
        "PSRAM allocation failed during connected components");

    return false;
  }

  if (componentCount == -2) {
    free(mask);

    mechSetError(
        out,
        "Connected component exceeded flood buffer");

    return false;
  }

  if (componentCount <= 0) {
    free(mask);

    mechSetError(
        out,
        "No roller digits detected");

    return false;
  }

  if (componentCount >
      MECH_MAX_DIGITS) {
    free(mask);

    mechSetError(
        out,
        "More than 16 digit-like components detected");

    return false;
  }

  out->detectedDigits =
      componentCount;

  uint8_t* normalized =
      static_cast<uint8_t*>(
          heap_caps_malloc(
              MECH_INT8_TEMPLATE_PIXELS,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  if (!normalized) {
    free(mask);

    mechSetError(
        out,
        "PSRAM allocation failed for normalized digit");

    return false;
  }

  float totalScore =
      0.0f;

  float totalMargin =
      0.0f;

  for (int i = 0;
       i < componentCount;
       ++i) {
    if (!mechNormalizeDigit(
            mask,
            roiWidth,
            roiHeight,
            components[i],
            normalized)) {
      free(normalized);
      free(mask);

      mechSetError(
          out,
          "Digit normalization failed");

      return false;
    }

    char digit;
    float score;
    char secondDigit;
    float secondScore;

    mechClassifyDigit(
        normalized,
        digit,
        score,
        secondDigit,
        secondScore);

    out->digits[i].digit =
        digit;

    out->digits[i].score =
        score;

    out->digits[i].secondDigit =
        secondDigit;

    out->digits[i].secondScore =
        secondScore;

    out->digits[i].component =
        components[i];

    out->raw[i] =
        digit;

    totalScore +=
        score;

    totalMargin +=
        max(
            0.0f,
            score -
                secondScore);
  }

  free(normalized);
  free(mask);

  out->raw[
      componentCount] =
      '\0';

  out->averageScore =
      totalScore /
      componentCount;

  out->averageMargin =
      totalMargin /
      componentCount;

  if (after > 0 &&
      componentCount >
          before) {
    int outputPosition = 0;

    for (int i = 0;
         i < componentCount &&
         outputPosition <
             static_cast<int>(
                 sizeof(
                     out->reading)) -
                 2;
         ++i) {
      if (i ==
          before) {
        out->reading[
            outputPosition++] =
            '.';
      }

      out->reading[
          outputPosition++] =
          out->raw[i];
    }

    out->reading[
        outputPosition] =
        '\0';
  }

  else {
    strncpy(
        out->reading,
        out->raw,
        sizeof(
            out->reading) -
            1);

    out->reading[
        sizeof(
            out->reading) -
        1] =
        '\0';
  }

  out->success =
      true;

  out->error[0] =
      '\0';

  return true;
}


static int mechProbeDigitCount(
    const uint16_t* frame,
    int frameWidth,
    int frameHeight,
    int roiX,
    int roiY,
    int roiWidth,
    int roiHeight,
    int saturationLimit) {
  uint8_t* mask =
      mechMakeMask(
          frame,
          frameWidth,
          frameHeight,
          roiX,
          roiY,
          roiWidth,
          roiHeight,
          saturationLimit);

  if (!mask) {
    return -1;
  }

  MechanicalComponent components[
      MECH_MAX_COMPONENTS];

  const int count =
      mechFindDigits(
          mask,
          roiWidth,
          roiHeight,
          components);

  free(mask);

  return count;
}


static int mechProbeWhiteInkDigitCount(
    const uint16_t* frame,
    int frameWidth,
    int frameHeight,
    int roiX,
    int roiY,
    int roiWidth,
    int roiHeight) {
  uint8_t* mask =
      mechMakeWhiteInkMask(
          frame,
          frameWidth,
          frameHeight,
          roiX,
          roiY,
          roiWidth,
          roiHeight);

  if (!mask) {
    return -1;
  }

  MechanicalComponent components[
      MECH_MAX_COMPONENTS];

  const int count =
      mechFindDigits(
          mask,
          roiWidth,
          roiHeight,
          components);

  free(mask);

  return count;
}

static float mechResultEvidence(
    const MechanicalOcrResult& result) {
  if (!result.success ||
      result.detectedDigits <= 0) {
    return -999.0f;
  }

  float solidBlobPenalty =
      0.0f;

  for (int i = 0;
       i < result.detectedDigits;
       ++i) {
    const MechanicalComponent& c =
        result.digits[i].component;

    const int boxArea =
        max(
            1,
            c.w *
            c.h);

    const float fill =
        static_cast<float>(
            c.area) /
        boxArea;

    // A real printed digit normally contains substantial black space.
    // A colored roller panel accidentally converted to white is close to
    // a completely filled rectangle.
    if (fill >
        0.82f) {
      solidBlobPenalty +=
          0.35f +
          (fill -
           0.82f);
    }
  }

  solidBlobPenalty /=
      max(
          1,
          result.detectedDigits);

  return
      result.averageScore +
      0.65f *
          result.averageMargin -
      solidBlobPenalty;
}

// Python S<110 remains the baseline.
// ESP32 RGB565 can increase apparent saturation compared with the desktop
// BGR image, so we probe a small set of saturation limits WITHOUT changing
// the frozen INT8 classifier. Expected digit count (BEFORE+AFTER) is used
// only to choose the best segmentation candidate.
static bool mechanicalReadRgb565(
    const uint16_t* frame,
    int frameWidth,
    int frameHeight,
    int roiX,
    int roiY,
    int roiWidth,
    int roiHeight,
    int before,
    int after,
    MechanicalOcrResult* out) {
  if (!out) {
    return false;
  }

  memset(
      out,
      0,
      sizeof(*out));

  const int expectedDigits =
      before +
      after;

  if (!frame) {
    mechSetError(
        out,
        "No decoded camera image");

    return false;
  }

  if (before < 0 ||
      after < 0 ||
      expectedDigits <= 0 ||
      expectedDigits >
          MECH_MAX_DIGITS) {
    mechSetError(
        out,
        "Invalid BEFORE/AFTER digit layout");

    return false;
  }

  // IMPORTANT:
  // This changes preprocessing selection only.
  // Your validated mechanical_templates_int8.h is untouched.
  static const int saturationLimits[] = {
      110,
      125,
      140,
      150,
      160};

  constexpr int LIMIT_COUNT =
      sizeof(saturationLimits) /
      sizeof(saturationLimits[0]);

  int counts[LIMIT_COUNT];

  Serial.printf(
      "MECH SEGMENT PROBE expected=%d: ",
      expectedDigits);

  for (int i = 0;
       i < LIMIT_COUNT;
       ++i) {
    counts[i] =
        mechProbeDigitCount(
            frame,
            frameWidth,
            frameHeight,
            roiX,
            roiY,
            roiWidth,
            roiHeight,
            saturationLimits[i]);

    Serial.printf(
        "S<%d=%d%s",
        saturationLimits[i],
        counts[i],
        (i + 1 <
         LIMIT_COUNT)
            ? " "
            : "\n");
  }

  const int whiteInkCount =
      mechProbeWhiteInkDigitCount(
          frame,
          frameWidth,
          frameHeight,
          roiX,
          roiY,
          roiWidth,
          roiHeight);

  Serial.printf(
      "WHITE_INK=%d\n",
      whiteInkCount);

  bool haveCandidate =
      false;

  MechanicalOcrResult bestResult = {};

  float bestEvidence =
      -999.0f;

  int bestDistance =
      999;

  int bestLimit =
      110;

  bool bestWhiteInk =
      false;

  // First evaluate the color-safe WHITE-INK mask when it finds the
  // configured number of digits. This is the important rescue for a white
  // digit printed over a red roller background.
  if (whiteInkCount ==
      expectedDigits) {
    MechanicalOcrResult whiteCandidate = {};

    if (mechRunSingleMask(
            frame,
            frameWidth,
            frameHeight,
            roiX,
            roiY,
            roiWidth,
            roiHeight,
            before,
            after,
            110,
            true,
            &whiteCandidate)) {
      const float evidence =
          mechResultEvidence(
              whiteCandidate);

      bestResult =
          whiteCandidate;

      bestEvidence =
          evidence;

      bestDistance =
          0;

      bestLimit =
          110;

      bestWhiteInk =
          true;

      haveCandidate =
          true;
    }
  }

  // Pass 1:
  // classify exact-count saturation masks too.
  for (int i = 0;
       i < LIMIT_COUNT;
       ++i) {
    if (counts[i] !=
        expectedDigits) {
      continue;
    }

    MechanicalOcrResult candidate = {};

    if (!mechRunSingleMask(
            frame,
            frameWidth,
            frameHeight,
            roiX,
            roiY,
            roiWidth,
            roiHeight,
            before,
            after,
            saturationLimits[i],
            false,
            &candidate)) {
      continue;
    }

    const float evidence =
        mechResultEvidence(
            candidate);

    // Prefer stronger classifier evidence.
    // If effectively tied, stay closer to the Python baseline S<110.
    if (!haveCandidate ||
        evidence >
            bestEvidence +
                0.003f ||
        (fabsf(
             evidence -
             bestEvidence) <=
             0.003f &&
         abs(
             saturationLimits[i] -
             110) <
             abs(
                 bestLimit -
                 110))) {
      bestResult =
          candidate;

      bestEvidence =
          evidence;

      bestDistance =
          0;

      bestLimit =
          saturationLimits[i];

      bestWhiteInk =
          false;

      haveCandidate =
          true;
    }
  }

  // Pass 2:
  // If no exact-count candidate exists, also consider the WHITE-INK mask
  // alongside the closest saturation mask.
  if (!haveCandidate) {
    if (whiteInkCount > 0) {
      MechanicalOcrResult whiteCandidate = {};

      if (mechRunSingleMask(
              frame,
              frameWidth,
              frameHeight,
              roiX,
              roiY,
              roiWidth,
              roiHeight,
              before,
              after,
              110,
              true,
              &whiteCandidate)) {
        const int distance =
            abs(
                whiteCandidate.detectedDigits -
                expectedDigits);

        bestResult =
            whiteCandidate;

        bestEvidence =
            mechResultEvidence(
                whiteCandidate);

        bestDistance =
            distance;

        bestLimit =
            110;

        bestWhiteInk =
            true;

        haveCandidate =
            true;
      }
    }

    for (int i = 0;
         i < LIMIT_COUNT;
         ++i) {
      if (counts[i] <= 0) {
        continue;
      }

      const int distance =
          abs(
              counts[i] -
              expectedDigits);

      if (distance >
          bestDistance) {
        continue;
      }

      MechanicalOcrResult candidate = {};

      if (!mechRunSingleMask(
              frame,
              frameWidth,
              frameHeight,
              roiX,
              roiY,
              roiWidth,
              roiHeight,
              before,
              after,
              saturationLimits[i],
              false,
            &candidate)) {
        continue;
      }

      const float evidence =
          mechResultEvidence(
              candidate);

      if (!haveCandidate ||
          distance <
              bestDistance ||
          (distance ==
               bestDistance &&
           evidence >
               bestEvidence +
                   0.003f)) {
        bestResult =
            candidate;

        bestEvidence =
            evidence;

        bestDistance =
            distance;

        bestLimit =
            saturationLimits[i];

        haveCandidate =
            true;
      }
    }
  }

  if (!haveCandidate) {
    mechSetError(
        out,
        "No usable mechanical segmentation candidate");

    return false;
  }

  *out =
      bestResult;

  out->whiteInkMaskUsed =
      bestWhiteInk;

  out->rgb565RescueUsed =
      bestWhiteInk ||
      bestLimit !=
          110;

  if (bestWhiteInk) {
    Serial.printf(
        "MECH SEGMENT SELECT WHITE_INK minRGB | %d/%d | reading=%s | score=%.3f margin=%.3f\n",
        out->detectedDigits,
        out->expectedDigits,
        out->reading,
        out->averageScore,
        out->averageMargin);
  }

  else {
    Serial.printf(
        "MECH SEGMENT SELECT S<%d | %d/%d | reading=%s | score=%.3f margin=%.3f\n",
        bestLimit,
        out->detectedDigits,
        out->expectedDigits,
        out->reading,
        out->averageScore,
        out->averageMargin);
  }

  return true;
}

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>MachineSens Mechanical Meter Setup</title>

<style>
:root{
  --bg:#0d1117;
  --panel:#161b22;
  --panel2:#0d1117;
  --border:#30363d;
  --text:#f0f6fc;
  --muted:#8b949e;
  --green:#238636;
  --green2:#2ea043;
  --blue:#1f6feb;
  --orange:#d29922;
  --red:#da3633;
  --gray:#30363d;
  --accent:#58a6ff;
}

*{box-sizing:border-box}

body{
  margin:0;
  background:var(--bg);
  color:var(--text);
  font-family:Arial,sans-serif;
}

header{
  padding:14px 18px;
  background:var(--panel);
  border-bottom:1px solid var(--border);
  position:sticky;
  top:0;
  z-index:20;
}

h1{
  margin:0;
  font-size:20px;
}

.subtitle{
  color:var(--muted);
  font-size:12px;
  margin-top:5px;
}

.layout{
  display:grid;
  grid-template-columns:minmax(350px,1.15fr) minmax(330px,.85fr);
  gap:14px;
  max-width:1500px;
  margin:auto;
  padding:14px;
}

.panel{
  background:var(--panel);
  border:1px solid var(--border);
  border-radius:12px;
  overflow:hidden;
}

.panel-title{
  padding:12px 14px;
  border-bottom:1px solid var(--border);
  font-weight:bold;
}

.preview-wrap{
  padding:12px;
}

#liveStage,#captureStage{
  position:relative;
  width:100%;
  background:#000;
  border-radius:8px;
  overflow:hidden;
}

#liveStage{
  min-height:220px;
  display:flex;
  align-items:center;
  justify-content:center;
}

#captureStage{
  display:block;
  min-height:0;
}

#stream,#captured{
  display:block;
  width:100%;
  height:auto;
  max-height:72vh;
  object-fit:contain;
}

#cropCanvas{
  position:absolute;
  left:0;
  top:0;
  z-index:5;
  touch-action:none;
  cursor:crosshair;
  pointer-events:auto;
}

.hidden{display:none!important}

.capture-actions{
  display:flex;
  flex-wrap:wrap;
  gap:8px;
  margin-top:12px;
}

button{
  border:0;
  border-radius:7px;
  padding:11px 14px;
  font-weight:bold;
  cursor:pointer;
  font-size:13px;
  color:#fff;
}

button:disabled{
  opacity:.55;
  cursor:not-allowed;
}

.primary{background:var(--green)}
.primary:hover{background:var(--green2)}
.blue{background:var(--blue)}
.orange{background:var(--orange)}
.gray{background:var(--gray)}
.red{background:var(--red)}

.section{
  border:1px solid var(--border);
  border-radius:10px;
  padding:12px;
  background:var(--panel2);
}

.section h3{
  margin:0 0 10px;
  font-size:14px;
}

select{
  width:100%;
  background:var(--panel);
  color:var(--text);
  border:1px solid var(--border);
  border-radius:6px;
  padding:8px;
}

.reading-label{
  color:var(--muted);
  font-size:12px;
  margin-bottom:5px;
}

#captureStatus{
  margin:12px 0 0;
  padding:10px 12px;
  border:1px solid var(--border);
  background:var(--panel2);
  border-radius:8px;
  color:#c9d1d9;
  font-size:13px;
}

#readingBox{
  margin-top:12px;
  padding:14px;
  border:1px solid var(--border);
  border-radius:10px;
  background:var(--panel2);
}

.reading-value{
  font-size:36px;
  line-height:1.15;
  font-weight:800;
  letter-spacing:1px;
  color:#3fb950;
  word-break:break-word;
}

.info{
  margin-top:8px;
  color:var(--muted);
  font-size:12px;
  line-height:1.45;
}


.controls{
  padding:12px;
}

.control{
  margin-bottom:14px;
}

.control:last-child{
  margin-bottom:0;
}

.control-header{
  display:flex;
  justify-content:space-between;
  gap:10px;
  margin-bottom:6px;
  font-size:13px;
}

.value{
  color:var(--accent);
  font-weight:bold;
}

input[type="range"]{
  width:100%;
  accent-color:#2ea043;
  cursor:pointer;
  height:24px;
}

input[type="range"]::-webkit-slider-runnable-track{
  height:6px;
  border-radius:999px;
  background:#30363d;
}

input[type="range"]::-webkit-slider-thumb{
  margin-top:-5px;
}

select{
  width:100%;
  background:#161b22;
  color:#f0f6fc;
  border:1px solid #30363d;
  border-radius:6px;
  padding:8px;
}

.switch-row{
  display:flex;
  justify-content:space-between;
  align-items:center;
  gap:15px;
  margin:10px 0;
  font-size:13px;
}

.switch-row input{
  width:22px;
  height:22px;
}

.mask-panel{
  margin-top:12px;
  padding:12px;
  border:1px solid var(--border);
  border-radius:10px;
  background:#05070a;
}

.mask-title{
  font-size:13px;
  font-weight:bold;
  margin-bottom:8px;
}

.mask-subtitle{
  color:var(--muted);
  font-size:11px;
  margin-bottom:9px;
}

#maskImage{
  display:block;
  width:100%;
  height:auto;
  max-height:280px;
  object-fit:contain;
  background:#000;
  border:1px solid #30363d;
  border-radius:7px;
  image-rendering:pixelated;
}

.details{
  margin-top:12px;
  padding:12px;
  border:1px solid var(--border);
  border-radius:10px;
  background:#0b0f14;
  color:#a5d6ff;
  white-space:pre-wrap;
  font-family:Consolas,monospace;
  font-size:12px;
  max-height:340px;
  overflow:auto;
}

.status-grid{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:8px;
  margin-top:12px;
}

.status-card{
  border:1px solid var(--border);
  background:var(--panel2);
  border-radius:8px;
  padding:10px;
}

.status-card .k{
  color:var(--muted);
  font-size:11px;
}

.status-card .v{
  margin-top:4px;
  font-weight:bold;
  font-size:13px;
}

#message{
  position:fixed;
  right:16px;
  bottom:16px;
  z-index:100;
  display:none;
  max-width:360px;
  padding:12px 14px;
  border-radius:8px;
  background:var(--green);
  color:#fff;
  box-shadow:0 5px 25px #000;
}

@media(max-width:900px){
  .layout{grid-template-columns:1fr}
  header{position:relative}
  .status-grid{grid-template-columns:1fr}
}
</style>
</head>

<body>

<header>
  <h1>MachineSens Mechanical Meter Camera</h1>
  <div class="subtitle">
    VGA Live → XGA Still → Frozen INT8 OCR · White-Ink Red-Background Rescue · Mask Preview
  </div>
</header>

<div class="layout">

  <!-- LEFT -->
  <div class="panel">
    <div class="panel-title">Camera & Roller Meter Crop</div>

    <div class="preview-wrap">

      <div id="liveMode">
        <div id="liveStage">
          <img id="stream" alt="Live camera">
        </div>

        <div class="capture-actions">
          <button class="primary" onclick="takePhoto()">📷 TAKE PHOTO</button>
          <button class="gray" onclick="restartStream(true)">🔄 Restart Livecam</button>
        </div>
      </div>

      <div id="captureMode" class="hidden">
        <div id="captureStage">
          <img id="captured" alt="Captured photo">
          <canvas id="cropCanvas"></canvas>
        </div>

        <div class="section" style="margin-top:12px">
          <h3>Meter Format</h3>

          <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px">
            <div>
              <div class="reading-label">Digits BEFORE decimal</div>
              <select id="digitsBefore" onchange="updateExpectedDigits()">
                <option>0</option>
                <option>1</option>
                <option>2</option>
                <option>3</option>
                <option>4</option>
                <option>5</option>
                <option selected>6</option>
                <option>7</option>
                <option>8</option>
                <option>9</option>
                <option>10</option>
                <option>11</option>
                <option>12</option>
                <option>13</option>
                <option>14</option>
                <option>15</option>
                <option>16</option>
              </select>
            </div>

            <div>
              <div class="reading-label">Digits AFTER decimal</div>
              <select id="digitsAfter" onchange="updateExpectedDigits()">
                <option>0</option>
                <option selected>1</option>
                <option>2</option>
                <option>3</option>
                <option>4</option>
                <option>5</option>
                <option>6</option>
              </select>
            </div>
          </div>

          <div id="expectedDigits" class="info">
            Expected numeric digits: 7
          </div>

          <div class="info">
            Crop ONLY the roller-number window. The decimal is inserted from this configuration.
          </div>
        </div>

        <div class="capture-actions">
          <button class="primary" onclick="confirmCrop()">✅ READ MECHANICAL</button>
          <button class="blue" onclick="recrop()">✂ RE-CROP</button>
          <button class="orange" onclick="retakePhoto()">📷 RETAKE PHOTO</button>
          <button class="gray" onclick="backToLive()">🎥 BACK TO LIVE</button>
        </div>
      </div>

      <div id="captureStatus">Connecting to live camera...</div>

      <div id="readingBox">
        <div class="reading-label">Mechanical meter result</div>
        <div id="readingValue" class="reading-value">—</div>
        <div id="readingInfo" class="info">
          Take a photo, crop the roller digits, choose the digit format, then press READ MECHANICAL.
        </div>
      </div>

      <div id="maskPanel" class="mask-panel hidden">
        <div class="mask-title">OCR BLACK / WHITE MASK</div>
        <div class="mask-subtitle">
          This is the exact binary crop the ESP32 uses to find the roller digits.
        </div>
        <img id="maskImage" alt="Black and white OCR mask">
      </div>

      <div id="resultDetails" class="details">No OCR result yet.</div>
    </div>
  </div>

  <!-- RIGHT -->
  <div class="panel">
    <div class="panel-title">Camera Controls</div>

    <div class="controls">

      <div class="section">
        <h3>Image</h3>

        <div class="control">
          <div class="control-header">
            <span>Brightness</span>
            <span id="brightnessValue" class="value">0</span>
          </div>
          <input id="brightness" type="range" min="-2" max="2" step="1" value="0"
                 oninput="rangeChanged('brightness',this.value)">
        </div>

        <div class="control">
          <div class="control-header">
            <span>Contrast</span>
            <span id="contrastValue" class="value">0</span>
          </div>
          <input id="contrast" type="range" min="-2" max="2" step="1" value="0"
                 oninput="rangeChanged('contrast',this.value)">
        </div>

        <div class="control">
          <div class="control-header">
            <span>Saturation</span>
            <span id="saturationValue" class="value">0</span>
          </div>
          <input id="saturation" type="range" min="-2" max="2" step="1" value="0"
                 oninput="rangeChanged('saturation',this.value)">
        </div>

        <div class="control">
          <div class="control-header">
            <span>Sharpness</span>
            <span id="sharpnessValue" class="value">0</span>
          </div>
          <input id="sharpness" type="range" min="-2" max="2" step="1" value="0"
                 oninput="rangeChanged('sharpness',this.value)">
        </div>

        <div class="control">
          <div class="control-header">
            <span>Denoise</span>
            <span id="denoiseValue" class="value">0</span>
          </div>
          <input id="denoise" type="range" min="0" max="8" step="1" value="0"
                 oninput="rangeChanged('denoise',this.value)">
        </div>

        <div class="control">
          <div class="control-header"><span>Special Effect</span></div>
          <select id="specialEffect" onchange="selectChanged('specialEffect',this.value)">
            <option value="0" selected>None</option>
            <option value="1">Negative</option>
            <option value="2">Grayscale</option>
            <option value="3">Red Tint</option>
            <option value="4">Green Tint</option>
            <option value="5">Blue Tint</option>
            <option value="6">Sepia</option>
          </select>
        </div>
      </div>

      <div class="section" style="margin-top:12px">
        <h3>White Balance</h3>

        <div class="switch-row">
          <span>White Balance</span>
          <input id="whiteBalance" type="checkbox" checked
                 onchange="toggleChanged('whiteBalance',this.checked)">
        </div>

        <div class="switch-row">
          <span>AWB Gain</span>
          <input id="awbGain" type="checkbox" checked
                 onchange="toggleChanged('awbGain',this.checked)">
        </div>

        <div class="control">
          <div class="control-header"><span>WB Mode</span></div>
          <select id="wbMode" onchange="selectChanged('wbMode',this.value)">
            <option value="0" selected>Auto</option>
            <option value="1">Sunny</option>
            <option value="2">Cloudy</option>
            <option value="3">Office</option>
            <option value="4">Home</option>
          </select>
        </div>
      </div>

      <div class="section" style="margin-top:12px">
        <h3>Exposure</h3>

        <div class="switch-row">
          <span>Automatic Exposure Control</span>
          <input id="exposureControl" type="checkbox" checked
                 onchange="toggleChanged('exposureControl',this.checked)">
        </div>

        <div class="switch-row">
          <span>AEC DSP Mode</span>
          <input id="aec2" type="checkbox"
                 onchange="toggleChanged('aec2',this.checked)">
        </div>

        <div class="control">
          <div class="control-header">
            <span>Auto Exposure Level</span>
            <span id="aeLevelValue" class="value">0</span>
          </div>
          <input id="aeLevel" type="range" min="-2" max="2" step="1" value="0"
                 oninput="rangeChanged('aeLevel',this.value)">
        </div>

        <div class="control">
          <div class="control-header">
            <span>Manual Exposure</span>
            <span id="aecValueValue" class="value">300</span>
          </div>
          <input id="aecValue" type="range" min="0" max="1200" step="10" value="300"
                 oninput="rangeChanged('aecValue',this.value)">
        </div>
      </div>

      <div class="section" style="margin-top:12px">
        <h3>Gain</h3>

        <div class="switch-row">
          <span>Automatic Gain Control</span>
          <input id="gainControl" type="checkbox" checked
                 onchange="toggleChanged('gainControl',this.checked)">
        </div>

        <div class="control">
          <div class="control-header">
            <span>Manual Gain</span>
            <span id="agcGainValue" class="value">0</span>
          </div>
          <input id="agcGain" type="range" min="0" max="30" step="1" value="0"
                 oninput="rangeChanged('agcGain',this.value)">
        </div>

        <div class="control">
          <div class="control-header"><span>Gain Ceiling</span></div>
          <select id="gainCeiling" onchange="selectChanged('gainCeiling',this.value)">
            <option value="0">2×</option>
            <option value="1">4×</option>
            <option value="2" selected>8×</option>
            <option value="3">16×</option>
            <option value="4">32×</option>
            <option value="5">64×</option>
            <option value="6">128×</option>
          </select>
        </div>
      </div>

      <div class="section" style="margin-top:12px">
        <h3>Image Corrections</h3>

        <div class="switch-row">
          <span>Black Pixel Correction</span>
          <input id="blackPixelCorrection" type="checkbox" checked
                 onchange="toggleChanged('blackPixelCorrection',this.checked)">
        </div>

        <div class="switch-row">
          <span>White Pixel Correction</span>
          <input id="whitePixelCorrection" type="checkbox" checked
                 onchange="toggleChanged('whitePixelCorrection',this.checked)">
        </div>

        <div class="switch-row">
          <span>Raw Gamma</span>
          <input id="rawGamma" type="checkbox" checked
                 onchange="toggleChanged('rawGamma',this.checked)">
        </div>

        <div class="switch-row">
          <span>Lens Correction</span>
          <input id="lensCorrection" type="checkbox" checked
                 onchange="toggleChanged('lensCorrection',this.checked)">
        </div>

        <div class="switch-row">
          <span>Downsize</span>
          <input id="downsize" type="checkbox" checked
                 onchange="toggleChanged('downsize',this.checked)">
        </div>
      </div>

      <div class="section" style="margin-top:12px">
        <h3>Orientation</h3>

        <div class="switch-row">
          <span>Horizontal Mirror</span>
          <input id="horizontalMirror" type="checkbox"
                 onchange="toggleChanged('horizontalMirror',this.checked)">
        </div>

        <div class="switch-row">
          <span>Vertical Flip</span>
          <input id="verticalFlip" type="checkbox"
                 onchange="toggleChanged('verticalFlip',this.checked)">
        </div>
      </div>

      <div class="section" style="margin-top:12px">
        <h3>Flash LED</h3>

        <div class="switch-row">
          <span>Use Flash for Still Photo</span>
          <input id="flashEnabled" type="checkbox"
                 onchange="toggleChanged('flashEnabled',this.checked)">
        </div>

        <div class="control">
          <div class="control-header">
            <span>LED Intensity</span>
            <span id="flashBrightnessValue" class="value">80</span>
          </div>
          <input id="flashBrightness" type="range" min="0" max="255" step="1" value="80"
                 oninput="rangeChanged('flashBrightness',this.value)">
        </div>
      </div>

      <div class="section" style="margin-top:12px">
        <h3>Still Capture</h3>

        <div class="control">
          <div class="control-header">
            <span>Capture Warm-up (ms)</span>
            <span id="captureWarmupMsValue" class="value">700</span>
          </div>
          <input id="captureWarmupMs" type="range" min="0" max="3000" step="50" value="700"
                 oninput="rangeChanged('captureWarmupMs',this.value)">
        </div>

        <div class="control">
          <div class="control-header">
            <span>Frames to Discard</span>
            <span id="framesToDiscardValue" class="value">4</span>
          </div>
          <input id="framesToDiscard" type="range" min="0" max="5" step="1" value="4"
                 oninput="rangeChanged('framesToDiscard',this.value)">
        </div>

        <div class="info">
          For stable bright still photos, firmware uses at least 900 ms warm-up, 5 discarded XGA transition frames, and one automatic brighter retry if the actual still is dark.
        </div>
      </div>

      <div class="section" style="margin-top:12px">
        <h3>ESP32-CAM Status</h3>

        <div class="status-grid">
          <div class="status-card">
            <div class="k">Wi-Fi</div>
            <div class="v">Mechanical_Meter_Test</div>
          </div>

          <div class="status-card">
            <div class="k">IP</div>
            <div class="v">192.168.4.1</div>
          </div>

          <div class="status-card">
            <div class="k">Live</div>
            <div class="v">640 × 480</div>
          </div>

          <div class="status-card">
            <div class="k">OCR photo</div>
            <div class="v">1024 × 768</div>
          </div>
        </div>
      </div>

      <div class="capture-actions">
        <button class="gray" onclick="resetCameraDefaults()">↩ RESET CAMERA DEFAULTS</button>
      </div>

    </div>
  </div>
</div>

<div id="message"></div>

<script>
const stream = document.getElementById('stream');
const captured = document.getElementById('captured');
const canvas = document.getElementById('cropCanvas');
const ctx = canvas.getContext('2d');

const liveMode = document.getElementById('liveMode');
const captureMode = document.getElementById('captureMode');
const captureStatus = document.getElementById('captureStatus');

const streamUrl = `http://${location.hostname}:81/stream`;

let cropRect = null;
let dragging = false;
let startX = 0;
let startY = 0;
let capturedObjectUrl = null;
let streamRetryTimer = null;
let streamGeneration = 0;

// OCR digit boxes returned by ESP32.
// Coordinates are relative to the OCR crop, not the whole photograph.
let digitBoxes = [];
let digitBoxRoiW = 1;
let digitBoxRoiH = 1;

function sleepMs(ms){
  return new Promise(resolve=>setTimeout(resolve,ms));
}

function showMessage(text,bad=false){
  const box=document.getElementById('message');
  box.textContent=text;
  box.style.background=bad?'#da3633':'#238636';
  box.style.display='block';

  setTimeout(()=>{
    box.style.display='none';
  },2600);
}

function liveVisible(){
  return !liveMode.classList.contains('hidden');
}

async function restartStream(manual=false){
  const generation=++streamGeneration;

  if(streamRetryTimer){
    clearTimeout(streamRetryTimer);
    streamRetryTimer=null;
  }

  stream.onerror=null;
  stream.src='';

  await sleepMs(manual?260:180);

  if(generation!==streamGeneration || !liveVisible()){
    return;
  }

  stream.onerror=()=>{
    if(!liveVisible() || generation!==streamGeneration){
      return;
    }

    captureStatus.textContent='Live camera dropped. Reconnecting...';

    streamRetryTimer=setTimeout(()=>{
      restartStream(false);
    },1300);
  };

  stream.src=streamUrl+'?t='+Date.now();
  captureStatus.textContent='Stable VGA live camera ready.';
}

function stopStream(){
  ++streamGeneration;

  if(streamRetryTimer){
    clearTimeout(streamRetryTimer);
    streamRetryTimer=null;
  }

  stream.onerror=null;
  stream.src='';
}

function clearCropCanvas(){
  ctx.clearRect(0,0,canvas.width,canvas.height);
}

function resizeCropCanvas(){
  const rect=captured.getBoundingClientRect();

  // captureMode must be visible before this runs.
  if(rect.width<2 || rect.height<2){
    requestAnimationFrame(resizeCropCanvas);
    return;
  }

  canvas.width=Math.max(1,Math.round(rect.width));
  canvas.height=Math.max(1,Math.round(rect.height));

  canvas.style.width=rect.width+'px';
  canvas.style.height=rect.height+'px';

  drawCrop();
}

function pointerPosition(event){
  const bounds=canvas.getBoundingClientRect();

  return{
    x:(event.clientX-bounds.left)*(canvas.width/bounds.width),
    y:(event.clientY-bounds.top)*(canvas.height/bounds.height)
  };
}

function drawCrop(){
  clearCropCanvas();

  if(!cropRect){
    return;
  }

  // Main roller crop.
  ctx.fillStyle='rgba(0,255,85,.06)';
  ctx.fillRect(cropRect.x,cropRect.y,cropRect.w,cropRect.h);

  ctx.strokeStyle='#00ff55';
  ctx.lineWidth=3;
  ctx.strokeRect(cropRect.x,cropRect.y,cropRect.w,cropRect.h);

  ctx.fillStyle='#00ff55';
  ctx.font='bold 14px Arial';
  ctx.fillText(
    'ROLLER CROP',
    cropRect.x+5,
    Math.max(17,cropRect.y-5)
  );

  // After OCR, draw one clear box around every component that the ESP32
  // actually classified as a digit.
  if(digitBoxes.length && digitBoxRoiW>0 && digitBoxRoiH>0){
    const scaleX=cropRect.w/digitBoxRoiW;
    const scaleY=cropRect.h/digitBoxRoiH;

    for(const d of digitBoxes){
      // Small visual padding so the rectangle surrounds the entire roller
      // character instead of sitting exactly on the white component pixels.
      const padX=Math.max(2,3*scaleX);
      const padY=Math.max(2,3*scaleY);

      const bx=cropRect.x+d.x*scaleX-padX;
      const by=cropRect.y+d.y*scaleY-padY;
      const bw=d.w*scaleX+padX*2;
      const bh=d.h*scaleY+padY*2;

      ctx.strokeStyle='#ffd33d';
      ctx.lineWidth=3;
      ctx.strokeRect(bx,by,bw,bh);

      const label=`${d.index}: ${d.digit}`;
      ctx.font='bold 13px Arial';
      const textW=ctx.measureText(label).width;
      const labelX=Math.max(cropRect.x,bx);
      const labelY=Math.max(cropRect.y+16,by-4);

      ctx.fillStyle='rgba(0,0,0,.82)';
      ctx.fillRect(labelX-2,labelY-14,textW+6,18);

      ctx.fillStyle='#ffd33d';
      ctx.fillText(label,labelX+1,labelY);
    }
  }
}

canvas.addEventListener('pointerdown',event=>{
  event.preventDefault();

  try{
    canvas.setPointerCapture(event.pointerId);
  }catch(e){}

  const p=pointerPosition(event);

  startX=p.x;
  startY=p.y;
  dragging=true;
  digitBoxes=[];
  document.getElementById('maskPanel').classList.add('hidden');

  cropRect={
    x:p.x,
    y:p.y,
    w:0,
    h:0
  };

  drawCrop();
});

canvas.addEventListener('pointermove',event=>{
  if(!dragging){
    return;
  }

  event.preventDefault();

  const p=pointerPosition(event);

  cropRect={
    x:Math.min(startX,p.x),
    y:Math.min(startY,p.y),
    w:Math.abs(p.x-startX),
    h:Math.abs(p.y-startY)
  };

  drawCrop();
});

function finishCrop(event){
  if(!dragging){
    return;
  }

  event.preventDefault();
  dragging=false;

  try{
    canvas.releasePointerCapture(event.pointerId);
  }catch(e){}

  if(!cropRect || cropRect.w<12 || cropRect.h<12){
    cropRect=null;
    clearCropCanvas();
    captureStatus.textContent='Crop too small. Drag again.';
    return;
  }

  captureStatus.textContent=
    'Crop selected. Press READ MECHANICAL, re-crop, or retake.';
}

canvas.addEventListener('pointerup',finishCrop);
canvas.addEventListener('pointercancel',finishCrop);

window.addEventListener('resize',()=>{
  if(!captureMode.classList.contains('hidden')){
    resizeCropCanvas();
  }
});

function updateExpectedDigits(){
  const before=parseInt(document.getElementById('digitsBefore').value||'0');
  const after=parseInt(document.getElementById('digitsAfter').value||'0');

  document.getElementById('expectedDigits').textContent=
    `Expected numeric digits: ${before+after}`;
}

async function takePhoto(){
  captureStatus.textContent='Taking high-quality XGA photo...';
  stopStream();

  await sleepMs(450);

  try{
    const response=await fetch('/capture?t='+Date.now(),{cache:'no-store'});

    if(!response.ok){
      let message='Capture failed';

      try{
        const data=await response.json();
        message=data.message||message;
      }catch(e){}

      throw new Error(message);
    }

    const blob=await response.blob();

    if(capturedObjectUrl){
      URL.revokeObjectURL(capturedObjectUrl);
    }

    capturedObjectUrl=URL.createObjectURL(blob);
    cropRect=null;
    digitBoxes=[];
    document.getElementById('maskPanel').classList.add('hidden');
    document.getElementById('maskImage').removeAttribute('src');

    captured.onload=()=>{
      // IMPORTANT:
      // show the capture view BEFORE measuring the image.
      // Measuring while captureMode is display:none makes the canvas 1x1.
      liveMode.classList.add('hidden');
      captureMode.classList.remove('hidden');

      requestAnimationFrame(()=>{
        requestAnimationFrame(()=>{
          resizeCropCanvas();
          clearCropCanvas();
          captureStatus.textContent=
            'Photo captured. Drag a crop around ONLY the roller digits.';
        });
      });
    };

    captured.src=capturedObjectUrl;
  }

  catch(error){
    captureStatus.textContent=error.message;
    showMessage(error.message,true);
    await backToLive();
  }
}

function recrop(){
  cropRect=null;
  digitBoxes=[];
  document.getElementById('maskPanel').classList.add('hidden');
  document.getElementById('maskImage').removeAttribute('src');
  clearCropCanvas();
  captureStatus.textContent='Drag a new crop around ONLY the roller digits.';
}

async function retakePhoto(){
  cropRect=null;
  await takePhoto();
}

async function backToLive(){
  captureMode.classList.add('hidden');
  liveMode.classList.remove('hidden');
  cropRect=null;
  digitBoxes=[];
  document.getElementById('maskPanel').classList.add('hidden');
  document.getElementById('maskImage').removeAttribute('src');
  clearCropCanvas();
  await restartStream(false);
}

async function confirmCrop(){
  if(!captured.naturalWidth || !cropRect || cropRect.w<5 || cropRect.h<5){
    showMessage('Draw a crop around the roller digits first.',true);
    return;
  }

  const sx=captured.naturalWidth/canvas.width;
  const sy=captured.naturalHeight/canvas.height;

  const x=Math.round(cropRect.x*sx);
  const y=Math.round(cropRect.y*sy);
  const w=Math.round(cropRect.w*sx);
  const h=Math.round(cropRect.h*sy);

  const before=parseInt(document.getElementById('digitsBefore').value||'0');
  const after=parseInt(document.getElementById('digitsAfter').value||'0');

  if(before+after<=0 || before+after>16){
    showMessage('Total digits must be between 1 and 16.',true);
    return;
  }

  captureStatus.textContent='Running mechanical template OCR...';
  document.getElementById('readingValue').textContent='...';
  document.getElementById('readingInfo').textContent='Processing on ESP32-CAM...';
  document.getElementById('resultDetails').textContent='Reading...';

  try{
    const url=
      `/read?x=${x}&y=${y}&w=${w}&h=${h}&before=${before}&after=${after}`;

    const response=await fetch(url,{cache:'no-store'});
    const data=await response.json();

    if(!response.ok || !data.success){
      throw new Error(data.message||'Mechanical OCR failed');
    }

    document.getElementById('readingValue').textContent=data.reading;

    // Display the exact digit boxes returned by the ESP32.
    digitBoxes=Array.isArray(data.digits)?data.digits:[];
    digitBoxRoiW=Math.max(1,data.roi_w||1);
    digitBoxRoiH=Math.max(1,data.roi_h||1);
    drawCrop();

    // Display the exact black/white mask used for this OCR result.
    const maskPanel=document.getElementById('maskPanel');
    const maskImage=document.getElementById('maskImage');
    maskPanel.classList.remove('hidden');
    maskImage.src='/mask?t='+Date.now();

    const countOk=data.detected===data.expected;

    document.getElementById('readingValue').style.color=
      countOk?'#3fb950':'#d29922';

    const maskName=data.white_ink
      ?'WHITE-INK color-safe'
      :(data.rgb565_rescue?'RGB565 saturation rescue':'Python mask');

    document.getElementById('readingInfo').textContent=
      `Detected ${data.detected}/${data.expected} digits · Score ${data.average_score} · ${maskName}`;

    let details=
      `RAW: ${data.raw}\n`+
      `FINAL: ${data.reading}\n`+
      `EXPECTED DIGITS: ${data.expected}\n`+
      `DETECTED DIGITS: ${data.detected}\n`+
      `AVERAGE TEMPLATE SCORE: ${data.average_score}\n`+
      `AVERAGE MARGIN: ${data.average_margin}\n`+
      `MASK: ${data.white_ink?'WHITE-INK min(R,G,B)':('S < '+data.sat_limit)}\n\n`;

    for(const d of data.digits){
      details+=
        `Digit ${d.index}: ${d.digit}`+
        ` | score=${d.score}`+
        ` | 2nd=${d.second} (${d.second_score})`+
        ` | box=[${d.x},${d.y},${d.w},${d.h}]\n`;
    }

    if(!countOk){
      details+=
        `\nCHECK: expected ${data.expected} digits but detected ${data.detected}.`;
    }

    document.getElementById('resultDetails').textContent=details;

    captureStatus.textContent=
      countOk
        ?'Mechanical reading complete.'
        :'Reading complete, but digit count needs review.';

    showMessage('Mechanical OCR complete.');
  }

  catch(error){
    document.getElementById('readingValue').textContent='FAILED';
    document.getElementById('readingValue').style.color='#f85149';
    document.getElementById('readingInfo').textContent=error.message;
    document.getElementById('resultDetails').textContent=error.message;
    captureStatus.textContent='Mechanical OCR failed.';
    showMessage(error.message,true);
  }
}

let controlTimer=null;

function rangeChanged(name,value){
  document.getElementById(name+'Value').textContent=value;

  clearTimeout(controlTimer);

  controlTimer=setTimeout(()=>{
    setControl(name,value);
  },120);
}

function toggleChanged(name,checked){
  setControl(name,checked?1:0);
}

function selectChanged(name,value){
  setControl(name,value);
}

async function setControl(name,value){
  try{
    const response=await fetch(
      `/control?var=${encodeURIComponent(name)}&val=${encodeURIComponent(value)}`,
      {cache:'no-store'}
    );

    const data=await response.json();

    if(!response.ok || !data.success){
      throw new Error(data.message||'Camera control failed');
    }
  }

  catch(error){
    showMessage(error.message,true);
  }
}

async function resetCameraDefaults(){
  const ranges={
    brightness:0,
    contrast:0,
    saturation:0,
    sharpness:0,
    denoise:0,
    aeLevel:0,
    aecValue:300,
    agcGain:0,
    flashBrightness:80,
    captureWarmupMs:700,
    framesToDiscard:4
  };

  const toggles={
    whiteBalance:true,
    awbGain:true,
    exposureControl:true,
    aec2:false,
    gainControl:true,
    blackPixelCorrection:true,
    whitePixelCorrection:true,
    rawGamma:true,
    lensCorrection:true,
    downsize:true,
    horizontalMirror:false,
    verticalFlip:false,
    flashEnabled:false
  };

  const selects={
    specialEffect:0,
    wbMode:0,
    gainCeiling:2
  };

  for(const [key,value] of Object.entries(ranges)){
    const el=document.getElementById(key);
    if(el)el.value=value;

    const valueEl=document.getElementById(key+'Value');
    if(valueEl)valueEl.textContent=value;
  }

  for(const [key,value] of Object.entries(toggles)){
    const el=document.getElementById(key);
    if(el)el.checked=value;
  }

  for(const [key,value] of Object.entries(selects)){
    const el=document.getElementById(key);
    if(el)el.value=String(value);
  }

  try{
    const response=await fetch(
      '/defaults',
      {method:'POST',cache:'no-store'}
    );

    const data=await response.json();

    if(!response.ok || !data.success){
      throw new Error(data.message||'Reset failed');
    }

    showMessage('Camera defaults applied.');
  }

  catch(error){
    showMessage(error.message,true);
  }
}

updateExpectedDigits();
restartStream(false);
</script>

</body>
</html>
)HTML";

static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t controlHandler(httpd_req_t* req) {
  String variable;
  int value = 0;

  if (!getQueryString(req, "var", variable) ||
      !getQueryInt(req, "val", value)) {
    return sendJson(
        req,
        "{\"success\":false,\"message\":\"var and val are required\"}",
        400);
  }

  bool sensorSetting = true;

  if (variable == "brightness") {
    camUi.brightness = constrain(value, -2, 2);
  }

  else if (variable == "contrast") {
    camUi.contrast = constrain(value, -2, 2);
  }

  else if (variable == "saturation") {
    camUi.saturation = constrain(value, -2, 2);
  }

  else if (variable == "sharpness") {
    camUi.sharpness = constrain(value, -2, 2);
  }

  else if (variable == "denoise") {
    camUi.denoise = constrain(value, 0, 8);
  }

  else if (variable == "specialEffect") {
    camUi.specialEffect = constrain(value, 0, 6);
  }

  else if (variable == "whiteBalance") {
    camUi.whiteBalance = value != 0;
  }

  else if (variable == "awbGain") {
    camUi.awbGain = value != 0;
  }

  else if (variable == "wbMode") {
    camUi.wbMode = constrain(value, 0, 4);
  }

  else if (variable == "exposureControl") {
    camUi.exposureControl = value != 0;
  }

  else if (variable == "aec2") {
    camUi.aec2 = value != 0;
  }

  else if (variable == "aeLevel") {
    camUi.aeLevel = constrain(value, -2, 2);
  }

  else if (variable == "aecValue") {
    camUi.aecValue = constrain(value, 0, 1200);
  }

  else if (variable == "gainControl") {
    camUi.gainControl = value != 0;
  }

  else if (variable == "agcGain") {
    camUi.agcGain = constrain(value, 0, 30);
  }

  else if (variable == "gainCeiling") {
    camUi.gainCeiling = constrain(value, 0, 6);
  }

  else if (variable == "blackPixelCorrection") {
    camUi.blackPixelCorrection = value != 0;
  }

  else if (variable == "whitePixelCorrection") {
    camUi.whitePixelCorrection = value != 0;
  }

  else if (variable == "rawGamma") {
    camUi.rawGamma = value != 0;
  }

  else if (variable == "lensCorrection") {
    camUi.lensCorrection = value != 0;
  }

  else if (variable == "downsize") {
    camUi.downsize = value != 0;
  }

  else if (variable == "horizontalMirror") {
    camUi.horizontalMirror = value != 0;
  }

  else if (variable == "verticalFlip") {
    camUi.verticalFlip = value != 0;
  }

  else if (variable == "flashEnabled") {
    camUi.flashEnabled = value != 0;
    sensorSetting = false;
  }

  else if (variable == "flashBrightness") {
    camUi.flashBrightness = constrain(value, 0, 255);
    sensorSetting = false;
  }

  else if (variable == "captureWarmupMs") {
    camUi.captureWarmupMs = constrain(value, 0, 3000);
    sensorSetting = false;
  }

  else if (variable == "framesToDiscard") {
    camUi.framesToDiscard = constrain(value, 0, 5);
    sensorSetting = false;
  }

  else {
    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Unknown camera control\"}",
        400);
  }

  if (sensorSetting) {
    if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
      applyCameraUiSettings();
      xSemaphoreGive(cameraMutex);
    }

    else {
      return sendJson(
          req,
          "{\"success\":false,\"message\":\"Camera is busy\"}",
          500);
    }
  }

  return sendJson(req, "{\"success\":true}");
}

static esp_err_t defaultsHandler(httpd_req_t* req) {
  camUi = CameraUiSettings();

  if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
    applyCameraUiSettings();
    xSemaphoreGive(cameraMutex);
  }

  else {
    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Camera is busy\"}",
        500);
  }

  return sendJson(req, "{\"success\":true}");
}

static float estimateJpegCenterBrightness(
    const uint8_t* jpeg,
    size_t jpegLength) {
  // XGA 1024x768 / 4 = 256x192.
  constexpr int WIDTH = 256;
  constexpr int HEIGHT = 192;

  const size_t bytes =
      static_cast<size_t>(
          WIDTH) *
      HEIGHT *
      sizeof(uint16_t);

  uint16_t* preview =
      static_cast<uint16_t*>(
          heap_caps_malloc(
              bytes,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  if (!preview) {
    return -1.0f;
  }

  const bool decoded =
      jpg2rgb565(
          jpeg,
          jpegLength,
          reinterpret_cast<uint8_t*>(
              preview),
          JPG_SCALE_4X);

  if (!decoded) {
    free(preview);

    return -1.0f;
  }

  uint64_t total = 0;
  uint32_t count = 0;

  // Central 80% only.
  for (int y = 19;
       y < 173;
       ++y) {
    for (int x = 26;
         x < 230;
         ++x) {
      const uint16_t pixel =
          preview[
              y *
                  WIDTH +
              x];

      const uint8_t r =
          static_cast<uint8_t>(
              ((((pixel >> 11) &
                 0x1F) *
                255U) /
               31U));

      const uint8_t g =
          static_cast<uint8_t>(
              ((((pixel >> 5) &
                 0x3F) *
                255U) /
               63U));

      const uint8_t b =
          static_cast<uint8_t>(
              (((pixel &
                 0x1F) *
                255U) /
               31U));

      const uint16_t luma =
          static_cast<uint16_t>(
              (77U * r +
               150U * g +
               29U * b) >>
              8);

      total +=
          luma;

      count++;
    }
  }

  free(preview);

  if (count == 0) {
    return -1.0f;
  }

  return
      static_cast<float>(
          total) /
      count;
}

static bool switchSensorForCapture() {
  sensor_t* sensor =
      esp_camera_sensor_get();

  if (!sensor) {
    return false;
  }

  // IMPORTANT:
  // Do NOT re-apply AEC/AGC/AWB immediately after changing resolution.
  // That can restart automatic exposure and cause a dark first still.
  const int frameResult =
      sensor->set_framesize(
          sensor,
          CAPTURE_FRAME_SIZE);

  const int qualityResult =
      sensor->set_quality(
          sensor,
          CAPTURE_JPEG_QUALITY);

  return
      frameResult == 0 &&
      qualityResult == 0;
}

static void restoreSensorForLive() {
  setFlashBrightness(0);

  sensor_t* sensor =
      esp_camera_sensor_get();

  if (!sensor) {
    return;
  }

  sensor->set_framesize(
      sensor,
      STREAM_FRAME_SIZE);

  sensor->set_quality(
      sensor,
      STREAM_JPEG_QUALITY);

  // Restore exact user camera controls only after returning to VGA.
  applyCameraUiSettings();

  delay(120);

  for (int i = 0;
       i < 2;
       ++i) {
    camera_fb_t* transition =
        esp_camera_fb_get();

    if (transition) {
      esp_camera_fb_return(
          transition);
    }

    delay(40);
  }
}

static esp_err_t captureHandler(httpd_req_t* req) {
  // Fully release the live MJPEG client before changing sensor mode.
  streamPauseRequested =
      true;

  const uint32_t waitStarted =
      millis();

  while (
      streamClientActive &&
      millis() -
              waitStarted <
          STREAM_STOP_TIMEOUT_MS) {
    delay(20);
  }

  if (streamClientActive) {
    streamPauseRequested =
        false;

    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Live stream did not release camera. Restart livecam and try again.\"}",
        503);
  }

  if (xSemaphoreTake(
          cameraMutex,
          pdMS_TO_TICKS(
              CAMERA_MUTEX_TIMEOUT_MS)) !=
      pdTRUE) {
    streamPauseRequested =
        false;

    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Camera mutex timeout\"}",
        503);
  }

  sensor_t* sensor =
      esp_camera_sensor_get();

  if (!sensor ||
      !switchSensorForCapture()) {
    restoreSensorForLive();

    xSemaphoreGive(
        cameraMutex);

    streamPauseRequested =
        false;

    return sendJson(
        req,
        "{\"success\":false,\"message\":\"OV2640 rejected XGA still mode\"}",
        500);
  }

  delay(160);

  if (camUi.flashEnabled) {
    setFlashBrightness(
        static_cast<uint8_t>(
            constrain(
                camUi.flashBrightness,
                0,
                255)));
  }

  else {
    setFlashBrightness(0);
  }

  // Accuracy-first exposure settling.
  const int warmupMs =
      max(
          camUi.captureWarmupMs,
          900);

  delay(
      warmupMs);

  const int discardedFrames =
      max(
          camUi.framesToDiscard,
          5);

  for (int i = 0;
       i < discardedFrames;
       ++i) {
    camera_fb_t* transition =
        esp_camera_fb_get();

    if (transition) {
      esp_camera_fb_return(
          transition);
    }

    delay(65);
  }

  camera_fb_t* frame =
      esp_camera_fb_get();

  if (!frame) {
    restoreSensorForLive();

    xSemaphoreGive(
        cameraMutex);

    streamPauseRequested =
        false;

    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Camera returned no XGA frame\"}",
        500);
  }

  // Measure the ACTUAL still. If it is dark, discard it and take one
  // exposure-boosted retry. Live settings are restored afterward.
  const float firstBrightness =
      estimateJpegCenterBrightness(
          frame->buf,
          frame->len);

  if (camUi.exposureControl &&
      firstBrightness >= 0.0f &&
      firstBrightness <
          STILL_DARK_MEAN_THRESHOLD) {
    esp_camera_fb_return(
        frame);

    frame =
        nullptr;

    if (sensor->set_ae_level) {
      sensor->set_ae_level(
          sensor,
          firstBrightness <
                  62.0f
              ? 2
              : 1);
    }

    if (sensor->set_brightness) {
      sensor->set_brightness(
          sensor,
          max(
              1,
              camUi.brightness));
    }

    if (sensor->set_gainceiling) {
      sensor->set_gainceiling(
          sensor,
          static_cast<gainceiling_t>(
              max(
                  3,
                  camUi.gainCeiling)));
    }

    if (sensor->set_aec2) {
      sensor->set_aec2(
          sensor,
          1);
    }

    delay(1000);

    for (int i = 0;
         i < 3;
         ++i) {
      camera_fb_t* transition =
          esp_camera_fb_get();

      if (transition) {
        esp_camera_fb_return(
            transition);
      }

      delay(80);
    }

    frame =
        esp_camera_fb_get();
  }

  if (!frame) {
    restoreSensorForLive();

    xSemaphoreGive(
        cameraMutex);

    streamPauseRequested =
        false;

    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Camera returned no final still frame\"}",
        500);
  }

  if (frame->format !=
      PIXFORMAT_JPEG) {
    esp_camera_fb_return(
        frame);

    restoreSensorForLive();

    xSemaphoreGive(
        cameraMutex);

    streamPauseRequested =
        false;

    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Camera did not return JPEG\"}",
        500);
  }

  clearLatestJpeg();

  latestJpeg =
      static_cast<uint8_t*>(
          heap_caps_malloc(
              frame->len,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  if (!latestJpeg) {
    esp_camera_fb_return(
        frame);

    restoreSensorForLive();

    xSemaphoreGive(
        cameraMutex);

    streamPauseRequested =
        false;

    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Not enough PSRAM to retain still image\"}",
        500);
  }

  memcpy(
      latestJpeg,
      frame->buf,
      frame->len);

  latestJpegLen =
      frame->len;

  latestJpegW =
      frame->width;

  latestJpegH =
      frame->height;

  httpd_resp_set_type(
      req,
      "image/jpeg");

  httpd_resp_set_hdr(
      req,
      "Cache-Control",
      "no-store");

  char contentLength[24];

  snprintf(
      contentLength,
      sizeof(
          contentLength),
      "%u",
      static_cast<unsigned int>(
          frame->len));

  httpd_resp_set_hdr(
      req,
      "Content-Length",
      contentLength);

  const esp_err_t response =
      httpd_resp_send(
          req,
          reinterpret_cast<const char*>(
              frame->buf),
          frame->len);

  esp_camera_fb_return(
      frame);

  restoreSensorForLive();

  xSemaphoreGive(
      cameraMutex);

  streamPauseRequested =
      false;

  return response;
}

static esp_err_t readHandler(httpd_req_t* req) {
  if (!latestJpeg ||
      latestJpegLen == 0) {
    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Take a photo first\"}",
        400);
  }

  int x;
  int y;
  int w;
  int h;
  int before;
  int after;

  if (!getQueryInt(
          req,
          "x",
          x) ||
      !getQueryInt(
          req,
          "y",
          y) ||
      !getQueryInt(
          req,
          "w",
          w) ||
      !getQueryInt(
          req,
          "h",
          h) ||
      !getQueryInt(
          req,
          "before",
          before) ||
      !getQueryInt(
          req,
          "after",
          after)) {
    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Missing crop or digit configuration\"}",
        400);
  }

  const float scaleX =
      static_cast<float>(
          DECODE_W) /
      latestJpegW;

  const float scaleY =
      static_cast<float>(
          DECODE_H) /
      latestJpegH;

  int decodeX =
      static_cast<int>(
          roundf(
              x *
              scaleX));

  int decodeY =
      static_cast<int>(
          roundf(
              y *
              scaleY));

  int decodeWidth =
      static_cast<int>(
          roundf(
              w *
              scaleX));

  int decodeHeight =
      static_cast<int>(
          roundf(
              h *
              scaleY));

  decodeX =
      constrain(
          decodeX,
          0,
          DECODE_W - 1);

  decodeY =
      constrain(
          decodeY,
          0,
          DECODE_H - 1);

  decodeWidth =
      constrain(
          decodeWidth,
          1,
          DECODE_W -
              decodeX);

  decodeHeight =
      constrain(
          decodeHeight,
          1,
          DECODE_H -
              decodeY);

  const size_t decodeBytes =
      static_cast<size_t>(
          DECODE_W) *
      DECODE_H *
      sizeof(uint16_t);

  uint16_t* rgb565 =
      static_cast<uint16_t*>(
          heap_caps_malloc(
              decodeBytes,
              MALLOC_CAP_SPIRAM |
                  MALLOC_CAP_8BIT));

  if (!rgb565) {
    return sendJson(
        req,
        "{\"success\":false,\"message\":\"PSRAM decode allocation failed\"}",
        500);
  }

  // XGA JPEG -> full XGA RGB565.
  // No old XGA -> 800x600 downsample.
  const bool decoded =
      jpg2rgb565(
          latestJpeg,
          latestJpegLen,
          reinterpret_cast<uint8_t*>(
              rgb565),
          JPG_SCALE_NONE);

  if (!decoded) {
    free(rgb565);

    return sendJson(
        req,
        "{\"success\":false,\"message\":\"JPEG decode failed\"}",
        500);
  }

  MechanicalOcrResult result = {};

  const bool ok =
      mechanicalReadRgb565(
          rgb565,
          DECODE_W,
          DECODE_H,
          decodeX,
          decodeY,
          decodeWidth,
          decodeHeight,
          before,
          after,
          &result);

  // Rebuild and retain the EXACT selected binary mask for webpage preview.
  // This does not change OCR or the validated INT8 templates.
  clearLastOcrMask();

  if (ok) {
    lastOcrMask =
        result.whiteInkMaskUsed
            ? mechMakeWhiteInkMask(
                  rgb565,
                  DECODE_W,
                  DECODE_H,
                  decodeX,
                  decodeY,
                  decodeWidth,
                  decodeHeight)
            : mechMakeMask(
                  rgb565,
                  DECODE_W,
                  DECODE_H,
                  decodeX,
                  decodeY,
                  decodeWidth,
                  decodeHeight,
                  result.saturationLimitUsed);

    if (lastOcrMask) {
      lastOcrMaskW =
          decodeWidth;

      lastOcrMaskH =
          decodeHeight;
    }
  }

  free(rgb565);

  if (!ok) {
    String json =
        "{\"success\":false,\"message\":\"";

    json +=
        result.error;

    json +=
        "\"}";

    return sendJson(
        req,
        json,
        500);
  }

  String json;
  json.reserve(2200);

  json +=
      "{\"success\":true";

  json +=
      ",\"reading\":\"" +
      String(
          result.reading) +
      "\"";

  json +=
      ",\"raw\":\"" +
      String(
          result.raw) +
      "\"";

  json +=
      ",\"expected\":" +
      String(
          result.expectedDigits);

  json +=
      ",\"detected\":" +
      String(
          result.detectedDigits);

  json +=
      ",\"average_score\":" +
      String(
          result.averageScore,
          4);

  json +=
      ",\"average_margin\":" +
      String(
          result.averageMargin,
          4);

  json +=
      ",\"sat_limit\":" +
      String(
          result.saturationLimitUsed);

  json +=
      ",\"rgb565_rescue\":" +
      String(
          result.rgb565RescueUsed
              ? "true"
              : "false");

  json +=
      ",\"white_ink\":" +
      String(
          result.whiteInkMaskUsed
              ? "true"
              : "false");

  json +=
      ",\"roi_w\":" +
      String(
          decodeWidth);

  json +=
      ",\"roi_h\":" +
      String(
          decodeHeight);

  json +=
      ",\"digits\":[";

  for (int i = 0;
       i <
           result.detectedDigits;
       ++i) {
    if (i) {
      json += ",";
    }

    const MechanicalDigitResult& digit =
        result.digits[i];

    json += "{";

    json +=
        "\"index\":" +
        String(i + 1);

    json +=
        ",\"digit\":\"" +
        String(
            digit.digit) +
        "\"";

    json +=
        ",\"score\":" +
        String(
            digit.score,
            4);

    json +=
        ",\"second\":\"" +
        String(
            digit.secondDigit) +
        "\"";

    json +=
        ",\"second_score\":" +
        String(
            digit.secondScore,
            4);

    json +=
        ",\"x\":" +
        String(
            digit.component.x);

    json +=
        ",\"y\":" +
        String(
            digit.component.y);

    json +=
        ",\"w\":" +
        String(
            digit.component.w);

    json +=
        ",\"h\":" +
        String(
            digit.component.h);

    json += "}";
  }

  json += "]}";

  Serial.printf(
      "MECHANICAL FINAL: %s | %d/%d | avg=%.3f margin=%.3f | S<%d%s\n",
      result.reading,
      result.detectedDigits,
      result.expectedDigits,
      result.averageScore,
      result.averageMargin,
      result.saturationLimitUsed,
      result.rgb565RescueUsed
          ? " RGB565_RESCUE"
          : "");

  return sendJson(
      req,
      json);
}


static void bmpWrite16(
    uint8_t* destination,
    uint16_t value) {
  destination[0] =
      static_cast<uint8_t>(
          value &
          0xFF);

  destination[1] =
      static_cast<uint8_t>(
          (value >> 8) &
          0xFF);
}

static void bmpWrite32(
    uint8_t* destination,
    uint32_t value) {
  destination[0] =
      static_cast<uint8_t>(
          value &
          0xFF);

  destination[1] =
      static_cast<uint8_t>(
          (value >> 8) &
          0xFF);

  destination[2] =
      static_cast<uint8_t>(
          (value >> 16) &
          0xFF);

  destination[3] =
      static_cast<uint8_t>(
          (value >> 24) &
          0xFF);
}

static esp_err_t maskHandler(
    httpd_req_t* req) {
  if (!lastOcrMask ||
      lastOcrMaskW <= 0 ||
      lastOcrMaskH <= 0) {
    return sendJson(
        req,
        "{\"success\":false,\"message\":\"No OCR mask yet. Read the meter first.\"}",
        404);
  }

  const uint32_t width =
      static_cast<uint32_t>(
          lastOcrMaskW);

  const uint32_t height =
      static_cast<uint32_t>(
          lastOcrMaskH);

  // 8-bit grayscale BMP rows must be aligned to 4 bytes.
  const uint32_t rowStride =
      (width + 3U) &
      ~3U;

  constexpr uint32_t FILE_HEADER_SIZE =
      14;

  constexpr uint32_t DIB_HEADER_SIZE =
      40;

  constexpr uint32_t PALETTE_SIZE =
      256U *
      4U;

  constexpr uint32_t PIXEL_OFFSET =
      FILE_HEADER_SIZE +
      DIB_HEADER_SIZE +
      PALETTE_SIZE;

  const uint32_t pixelBytes =
      rowStride *
      height;

  const uint32_t fileSize =
      PIXEL_OFFSET +
      pixelBytes;

  uint8_t header[
      FILE_HEADER_SIZE +
      DIB_HEADER_SIZE] = {0};

  header[0] = 'B';
  header[1] = 'M';

  bmpWrite32(
      header + 2,
      fileSize);

  bmpWrite32(
      header + 10,
      PIXEL_OFFSET);

  bmpWrite32(
      header + 14,
      DIB_HEADER_SIZE);

  bmpWrite32(
      header + 18,
      width);

  bmpWrite32(
      header + 22,
      height);

  bmpWrite16(
      header + 26,
      1);

  bmpWrite16(
      header + 28,
      8);

  bmpWrite32(
      header + 34,
      pixelBytes);

  bmpWrite32(
      header + 46,
      256);

  bmpWrite32(
      header + 50,
      256);

  uint8_t palette[
      PALETTE_SIZE];

  for (int i = 0;
       i < 256;
       ++i) {
    palette[
        i * 4 + 0] =
        static_cast<uint8_t>(
            i);

    palette[
        i * 4 + 1] =
        static_cast<uint8_t>(
            i);

    palette[
        i * 4 + 2] =
        static_cast<uint8_t>(
            i);

    palette[
        i * 4 + 3] =
        0;
  }

  uint8_t* row =
      static_cast<uint8_t*>(
          heap_caps_malloc(
              rowStride,
              MALLOC_CAP_8BIT));

  if (!row) {
    return sendJson(
        req,
        "{\"success\":false,\"message\":\"Could not allocate BMP row buffer\"}",
        500);
  }

  httpd_resp_set_type(
      req,
      "image/bmp");

  httpd_resp_set_hdr(
      req,
      "Cache-Control",
      "no-store");

  esp_err_t status =
      httpd_resp_send_chunk(
          req,
          reinterpret_cast<const char*>(
              header),
          sizeof(header));

  if (status == ESP_OK) {
    status =
        httpd_resp_send_chunk(
            req,
            reinterpret_cast<const char*>(
                palette),
            sizeof(palette));
  }

  // BMP stores positive-height images bottom-up.
  for (int y =
           lastOcrMaskH - 1;
       y >= 0 &&
       status == ESP_OK;
       --y) {
    memset(
        row,
        0,
        rowStride);

    memcpy(
        row,
        lastOcrMask +
            static_cast<size_t>(
                y) *
                lastOcrMaskW,
        lastOcrMaskW);

    status =
        httpd_resp_send_chunk(
            req,
            reinterpret_cast<const char*>(
                row),
            rowStride);
  }

  free(row);

  if (status == ESP_OK) {
    status =
        httpd_resp_send_chunk(
            req,
            nullptr,
            0);
  }

  return status;
}

static esp_err_t streamHandler(httpd_req_t* req) {
  static const char* STREAM_CONTENT_TYPE =
      "multipart/x-mixed-replace;boundary=frame";

  static const char* STREAM_BOUNDARY =
      "\r\n--frame\r\n";

  static const char* STREAM_PART =
      "Content-Type: image/jpeg\r\n"
      "Content-Length: %u\r\n\r\n";

  esp_err_t result =
      httpd_resp_set_type(
          req,
          STREAM_CONTENT_TYPE);

  if (result != ESP_OK) {
    return result;
  }

  streamClientActive = true;

  while (!streamPauseRequested) {
    if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(1200)) != pdTRUE) {
      delay(15);
      continue;
    }

    camera_fb_t* frame =
        esp_camera_fb_get();

    if (!frame) {
      xSemaphoreGive(cameraMutex);
      delay(80);
      continue;
    }

    if (frame->format != PIXFORMAT_JPEG) {
      esp_camera_fb_return(frame);
      xSemaphoreGive(cameraMutex);
      delay(60);
      continue;
    }

    char part[64];

    const int hlen =
        snprintf(
            part,
            sizeof(part),
            STREAM_PART,
            static_cast<unsigned int>(frame->len));

    result =
        httpd_resp_send_chunk(
            req,
            STREAM_BOUNDARY,
            strlen(STREAM_BOUNDARY));

    if (result == ESP_OK) {
      result =
          httpd_resp_send_chunk(
              req,
              part,
              hlen);
    }

    if (result == ESP_OK) {
      result =
          httpd_resp_send_chunk(
              req,
              reinterpret_cast<const char*>(
                  frame->buf),
              frame->len);
    }

    esp_camera_fb_return(frame);
    xSemaphoreGive(cameraMutex);

    if (result != ESP_OK) {
      break;
    }

    delay(70);
  }

  streamClientActive = false;
  return result;
}

static bool initCamera() {
  initialiseFlashPWM();
  setFlashBrightness(0);

  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Initialize at XGA so PSRAM buffers are already large enough for stills.
  config.frame_size = CAPTURE_FRAME_SIZE;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  if (!psramFound()) {
    Serial.println("ERROR: PSRAM not found");
    return false;
  }

  const esp_err_t error =
      esp_camera_init(&config);

  if (error != ESP_OK) {
    Serial.printf(
        "ERROR: camera init 0x%X\n",
        error);

    return false;
  }

  cameraMutex = xSemaphoreCreateMutex();

  if (!cameraMutex) {
    Serial.println("ERROR: camera mutex failed");
    return false;
  }

  sensor_t* sensor =
      esp_camera_sensor_get();

  if (sensor) {
    sensor->set_framesize(
        sensor,
        STREAM_FRAME_SIZE);

    sensor->set_quality(
        sensor,
        STREAM_JPEG_QUALITY);
  }

  applyCameraUiSettings();

  for (int i = 0; i < 2; ++i) {
    camera_fb_t* frame =
        esp_camera_fb_get();

    if (frame) {
      esp_camera_fb_return(frame);
    }

    delay(100);
  }

  return true;
}

static void startServers() {
  httpd_config_t mainConfig =
      HTTPD_DEFAULT_CONFIG();

  mainConfig.server_port =
      MAIN_SERVER_PORT;

  mainConfig.max_uri_handlers =
      10;

  mainConfig.stack_size =
      8192;

  mainConfig.lru_purge_enable =
      true;

  if (httpd_start(
          &mainServer,
          &mainConfig) !=
      ESP_OK) {
    Serial.println("ERROR: main HTTP server failed");
    return;
  }

  httpd_uri_t indexUri = {};
  indexUri.uri = "/";
  indexUri.method = HTTP_GET;
  indexUri.handler = indexHandler;
  httpd_register_uri_handler(mainServer, &indexUri);

  httpd_uri_t captureUri = {};
  captureUri.uri = "/capture";
  captureUri.method = HTTP_GET;
  captureUri.handler = captureHandler;
  httpd_register_uri_handler(mainServer, &captureUri);

  httpd_uri_t readUri = {};
  readUri.uri = "/read";
  readUri.method = HTTP_GET;
  readUri.handler = readHandler;
  httpd_register_uri_handler(mainServer, &readUri);

  httpd_uri_t maskUri = {};
  maskUri.uri = "/mask";
  maskUri.method = HTTP_GET;
  maskUri.handler = maskHandler;
  httpd_register_uri_handler(mainServer, &maskUri);

  httpd_uri_t controlUri = {};
  controlUri.uri = "/control";
  controlUri.method = HTTP_GET;
  controlUri.handler = controlHandler;
  httpd_register_uri_handler(mainServer, &controlUri);

  httpd_uri_t defaultsUri = {};
  defaultsUri.uri = "/defaults";
  defaultsUri.method = HTTP_POST;
  defaultsUri.handler = defaultsHandler;
  httpd_register_uri_handler(mainServer, &defaultsUri);

  httpd_config_t streamConfig =
      HTTPD_DEFAULT_CONFIG();

  streamConfig.server_port =
      STREAM_SERVER_PORT;

  streamConfig.ctrl_port =
      mainConfig.ctrl_port + 1;

  streamConfig.max_uri_handlers =
      4;

  streamConfig.stack_size =
      8192;

  streamConfig.lru_purge_enable =
      true;

  if (httpd_start(
          &streamServer,
          &streamConfig) !=
      ESP_OK) {
    Serial.println("ERROR: stream HTTP server failed");
    return;
  }

  httpd_uri_t streamUri = {};
  streamUri.uri = "/stream";
  streamUri.method = HTTP_GET;
  streamUri.handler = streamHandler;
  httpd_register_uri_handler(streamServer, &streamUri);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!initCamera()) {
    while (true) {
      delay(1000);
    }
  }

  WiFi.mode(WIFI_AP);

  if (!WiFi.softAP(
          AP_SSID,
          AP_PASSWORD)) {
    Serial.println("ERROR: AP failed");

    while (true) {
      delay(1000);
    }
  }

  WiFi.setSleep(false);

  startServers();

  Serial.println();
  Serial.println("======================================");
  Serial.println("MECHANICAL OCR V2 READY");
  Serial.print("Wi-Fi: ");
  Serial.println(AP_SSID);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("Live stream: port 81");
  Serial.println("======================================");
}

void loop() {
  delay(250);
}
