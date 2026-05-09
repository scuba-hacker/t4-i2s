// **** CONCURRENT APP LOAD WITH Oceanic ****

// Upload Oceanic via USB to app0 partition, and then use OTA to upload t4-i2s to app1 partition.
// If t4-i2s is uploaded via USB, then there is no OTA capability in that app to upload Oceanic.
// USB upload always uploads to app0 and overwrites the otadata partition with boot_app0.bin so all 
// trace of the other OTA app are gone.
// To install the apps side-by-side, Oceanic must be uploaded by USB, and then t4-i2s by OTA.

#include <Arduino.h>
#include <FS.h>
#include <LilyGo_AMOLED.h>
#include <LV_Helper.h>
#include <SD.h>
#include <WiFi.h>

// TEST: online tone generator
// http://onlinetonegenerator.com
#include "mercator_secrets.c"

#include <driver/i2s.h>
#include <esp_err.h>
#include <esp_idf_version.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_timer.h>

#include <math.h>
#include <string.h>

#include "SerialConfig.h"

#include "fft.h"

#include <Fonts/Font16.h>
#include <Fonts/Font32rle.h>
#include <SimpleFTPServer.h>

bool enableOtaPartitionSwitch = true;  // Long-press top button switches to the other OTA app partition

FtpServer ftpServer;
bool ftpActive = false;
bool writeLogToSerial = true;

enum class FtpTransferDirection : uint8_t {
    None,
    Upload,
    Download,
};

FtpTransferDirection ftpTransferDirection = FtpTransferDirection::None;

void setupFTPServer();
void _ftpConnectCallback(FtpOperation ftpOperation, uint32_t freeSpace, uint32_t totalSpace);
void _ftpTransferCallback(FtpTransferOperation ftpOperation, const char* name, uint32_t transferredSize);

const char* scanForKnownNetwork(); // return first known network found
bool connectToWiFiAndInitOTA(const bool wifiOnly, int repeatScanAttempts);
bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly);

const String ssid_not_connected = "-";
String ssid_connected;

namespace {

constexpr i2s_port_t kI2SPort = I2S_NUM_0;

// SEL on the Adafruit Mems I2S Mic is not connected to GPIO
// SEL is pulled to Ground on the board.
constexpr int kI2SBclk_GPIO = 39;   // Bit Clock
constexpr int kI2SLrclk_GPIO= 40;   // Left-Right Clock
constexpr int kI2SData_GPIO = 41;   // Mic Data
constexpr bool kLrclkSquareWaveTestEnabled = false;
constexpr uint32_t kLrclkSquareWaveHalfPeriodMs = 250;

constexpr int kTopButton_GPIO= 48;  // was 48 temporarily 21 (TO SWAP BUTTONS)
constexpr int kSideButton_GPIO= 21; // was 21 temporarily 47 (TO DISABLE TOP BUTTON WHICH IS 48)

constexpr int kSampleRate = 16000;
constexpr int kFftSize = 1024;
constexpr int kI2SReadFrames = 128;
constexpr int kI2SWordsPerFrame = 2;
constexpr int kI2SSampleShift = 16;
constexpr int kMicSlot = 0;

constexpr int kScreenWidth = 600;
constexpr int kScreenHeight = 450;
constexpr int kFftDisplayWidth = 200;
constexpr int kFftDisplayHeight = 135;
constexpr int kFftDisplayX = (kScreenWidth - kFftDisplayWidth) / 2;
constexpr int kFftDisplayY = (kScreenHeight - kFftDisplayHeight) / 2;
constexpr int kScopeDisplayWidth = 200;
constexpr int kScopeDisplayHeight = 135;
constexpr int kScopeDisplayX = kFftDisplayX;
constexpr int kScopeDisplayY = kFftDisplayY + kFftDisplayHeight;
constexpr int kStaticPanelWidth = kScreenWidth / 3;
constexpr int kStaticPanelHeight = kScreenHeight / 3;
constexpr int kStaticPanelLeftX = 0;
constexpr int kStaticPanelRightX = kStaticPanelWidth * 2;
constexpr int kStaticPanelTopY = 0;
constexpr int kStaticPanelBottomY = kStaticPanelHeight * 2;
constexpr int kWifiLabelX = kScopeDisplayX + kScopeDisplayWidth + 8;
constexpr int kWifiLabelWidth = kScreenWidth - kWifiLabelX - 12;
constexpr int kWifiLabelHeight = 62;
constexpr int kWifiLabelY =
    kScreenHeight - kWifiLabelHeight - 12;
constexpr int kCalibrationLabelWidth = kWifiLabelWidth;
constexpr int kCalibrationLabelHeight = 30;
constexpr int kCalibrationLabelX =
    kWifiLabelX + kWifiLabelWidth - kCalibrationLabelWidth;
constexpr int kCalibrationLabelY =
    kWifiLabelY - kCalibrationLabelHeight - 8;
constexpr int kAnalyzerDisplayWidth = 200;
constexpr int kAnalyzerDisplayHeight = 135;
constexpr int kAnalyzerDisplayX = kFftDisplayX;
constexpr int kAnalyzerDisplayY = kFftDisplayY - kAnalyzerDisplayHeight;
constexpr int kVuDisplayWidth = kAnalyzerDisplayWidth - 16;
constexpr int kVuDisplayHeight = kAnalyzerDisplayHeight;
constexpr int kVuDisplayX = kFftDisplayX - kVuDisplayWidth;
constexpr int kVuDisplayY = kFftDisplayY;
constexpr int kAnalogVuDisplayWidth = kVuDisplayWidth;
constexpr int kAnalogVuDisplayHeight = kVuDisplayHeight;
constexpr int kAnalogVuDisplayX = kFftDisplayX + kFftDisplayWidth;
constexpr int kAnalogVuDisplayY = kFftDisplayY;

constexpr int kDisplayLowBin = 4;     // 62.5 Hz at 16 kHz; avoids DC/rumble bins.
constexpr int kDisplayHighBin = 256;  // 4 kHz at 16 kHz sample rate.
constexpr float kNoiseMarginDb = 8.0f;
constexpr float kDisplayRangeDb = 52.0f;
constexpr uint32_t kDisplayNoiseCalibrationMs = 5000;
constexpr uint32_t kDisplayNoiseRecalibrationDelayMs = 500;

constexpr int kAnalyzerBandCount = 32;
constexpr int kAnalyzerBandWidth = 5;
constexpr int kAnalyzerBandGap = 1;
constexpr float kAnalyzerLowHz = 50.0f;
constexpr float kAnalyzerHighHz = 4000.0f;
constexpr float kAnalyzerSmoothing = 0.65f;
constexpr float kAnalyzerNoiseMarginDb = 10.0f;
constexpr float kAnalyzerRangeDb = 58.0f;
constexpr bool kAnalyzerDirtyRectEnabled = true;
constexpr float kVuMinDb = -60.0f;
constexpr float kVuMaxDb = 0.0f;
constexpr int16_t kVuClipThreshold = 32600;
constexpr uint8_t kVuClipHoldFrames = 8;
constexpr uint32_t kDevSerialStartupDelayMs = 5000;
constexpr uint32_t kLoopHousekeepingIntervalMs = 10;
constexpr uint32_t kDiagnosticsFrames = 120;
constexpr int64_t kDropReportWindowUs = 1000000;
constexpr size_t kRecordBufferFrameCount = 256;
constexpr size_t kRecordFrameBytes = kFftSize * sizeof(int16_t);
constexpr uint32_t kWavBitsPerSample = 16;
constexpr uint32_t kWavChannels = 1;
constexpr uint32_t kWavByteRate =
    kSampleRate * kWavChannels * (kWavBitsPerSample / 8);
constexpr uint16_t kWavBlockAlign = kWavChannels * (kWavBitsPerSample / 8);
constexpr uint32_t kSdFlushEveryChunks = 32;
constexpr int kAnalyzerActiveWidth =
    kAnalyzerBandCount * kAnalyzerBandWidth +
    (kAnalyzerBandCount - 1) * kAnalyzerBandGap;
static_assert(kAnalyzerActiveWidth <= kAnalyzerDisplayWidth,
              "Analyzer bands are too wide for the display");
static_assert(kWifiLabelWidth > 0 && kWifiLabelHeight > 0,
              "WiFi label must fit on the right of the scope display");
static_assert(kVuDisplayX >= 0, "VU display must fit left of the analyzer");
static_assert(kAnalogVuDisplayX + kAnalogVuDisplayWidth <= kScreenWidth,
              "Analogue VU display must fit right of the analyzer");
static_assert(kCalibrationLabelWidth > 0 && kCalibrationLabelHeight > 0,
              "Calibration label must fit on the display");
static_assert(kAnalyzerDisplayWidth == kScopeDisplayWidth &&
                  kAnalyzerDisplayHeight == kScopeDisplayHeight,
              "Shared graph buffer requires matching graph dimensions");
static_assert(kRecordBufferFrameCount <= UINT16_MAX,
              "Record buffer index storage assumes uint16_t indices");

constexpr uint16_t kColorBlack = 0x0000;
constexpr uint16_t kColorFrameActive    = 0x0005; // mid-green  (0x0500 byte-swapped)
constexpr uint16_t kColorFramePaused    = 0x20FD; // orange (0xFD20 byte-swapped)
constexpr uint16_t kColorFrameError     = 0x00A0; // mid-red    (0xA000 byte-swapped)
constexpr uint16_t kColorFrameRecording = 0x00A0; // mid-red    (0xA000 byte-swapped)
constexpr uint16_t kColorFrameStopping  = 0xE0FF; // yellow (0xFFE0 byte-swapped)
constexpr uint16_t kColorAnalyzerGrid   = 0x1082; // dark grey (empirically tuned, already compensates for byte swap)
constexpr uint16_t kColorScopeGrid      = 0x1082; // dark grey (empirically tuned, already compensates for byte swap)
constexpr uint16_t kColorScopeAxis      = 0xE739; //        (0x39E7 byte-swapped)
constexpr uint16_t kColorScopeTrace     = 0xFF07; // cyan   (0x07FF byte-swapped)

LilyGo_AMOLED amoled;

lv_obj_t *startupTitleLabel = nullptr;
lv_obj_t *startupStatusLabel = nullptr;
lv_obj_t *startupDetailLabel = nullptr;
lv_obj_t *startupFooterLabel = nullptr;
lv_obj_t *startupProgressBar = nullptr;
bool startupUiReady = false;
bool startupUiActive = false;
volatile bool wifiGotIpEvent = false;

SemaphoreHandle_t spectrogramMutex = nullptr;
SemaphoreHandle_t serialMutex = nullptr;
TaskHandle_t displayTaskHandle = nullptr;
TaskHandle_t captureTaskHandle = nullptr;
TaskHandle_t sdWriterTaskHandle = nullptr;
TaskHandle_t buttonTaskHandle = nullptr;

fft_config_t *fftPlan = nullptr;

int32_t i2sRaw[kI2SReadFrames * kI2SWordsPerFrame];
int16_t pcmFrame[kFftSize];
float fftInput[kFftSize];
float fftOutput[kFftSize];
float hannWindow[kFftSize];

uint8_t spectrogram[kFftDisplayWidth][kFftDisplayHeight];
uint8_t analyzerBarHeights[kAnalyzerBandCount];
uint8_t scopeTraceTop[kScopeDisplayWidth];
uint8_t scopeTraceBottom[kScopeDisplayWidth];
float vuLevel = 0.0f;
float vuPeakLevel = 0.0f;
bool vuClipActive = false;
uint16_t fftPixels[kFftDisplayWidth * kFftDisplayHeight];
uint16_t graphPixels[kScopeDisplayWidth * kScopeDisplayHeight];
uint16_t *analyzerBasePixels = nullptr;
uint16_t *vuBasePixels = nullptr;
uint16_t *analogVuBasePixels = nullptr;
uint16_t *vuDrawPixels = nullptr;
uint16_t *analogVuDrawPixels = nullptr;
int vuDrawWidth = kVuDisplayWidth;
int vuDrawHeight = kVuDisplayHeight;
int vuDrawOriginX = 0;
int vuDrawOriginY = 0;
int analogVuDrawWidth = kAnalogVuDisplayWidth;
int analogVuDrawHeight = kAnalogVuDisplayHeight;
int analogVuDrawOriginX = 0;
int analogVuDrawOriginY = 0;
uint16_t wifiLabelPixels[kWifiLabelWidth * kWifiLabelHeight];
uint16_t calibrationLabelPixels[kCalibrationLabelWidth * kCalibrationLabelHeight];
uint16_t palette[256];
uint16_t solidLine[kScreenWidth];
uint16_t spectrogramWriteColumn = 0;
uint8_t lastSpectrogramRenderMax = 0;
uint32_t lastSpectrogramRenderNonzero = 0;
bool analyzerReady = false;
bool analyzerBaseReady = false;
bool analyzerScreenReady = false;
uint8_t analyzerLastBarHeights[kAnalyzerBandCount];
bool scopeTraceReady = false;
bool vuReady = false;
bool vuBaseReady = false;
bool vuScreenReady = false;
int vuLastFillWidth = -1;
int vuLastPeakX = -1;
bool vuLastClipActive = false;
bool vuLastReady = false;
bool analogVuBaseReady = false;
bool analogVuScreenReady = false;
int analogVuLastNeedleDeg = -1;
bool analogVuLastClipActive = false;
bool analogVuLastReady = false;

volatile bool captureEnabled = true;
volatile bool frameDirty = true;
volatile bool i2sReady = false;
volatile bool fftReady = false;

float noiseFloorDb = 70.0f;
volatile bool displayNoiseCalibrated = true;
uint32_t displayNoiseCalibrationFrames = 0;
int64_t displayNoiseCalibrationStartUs = 0;
float displayNoiseCalibrationSumDb = 0.0f;
float displayNoiseCalibrationMinDb = 0.0f;
float lastFramePeakDb = 0.0f;
float lastFrameAvgDb = 0.0f;
float lastFrameMean = 0.0f;
float lastFrameRms = 0.0f;
int16_t lastFrameMin = 0;
int16_t lastFrameMax = 0;
uint16_t lastFrameClipped = 0;
float analyzerLevels[kAnalyzerBandCount];
float scopeScale = 4096.0f;
float vuSmoothedLevel = 0.0f;
uint8_t vuClipHold = 0;

enum class SdCommand : uint8_t {
    Start,
    Stop,
};

enum class SdStatus : uint8_t {
    NotReady,
    Idle,
    Starting,
    Recording,
    Stopping,
    Error,
};

struct RecordFrame {
    int16_t samples[kFftSize];
};

struct __attribute__((packed)) WavHeader {
    char riff[4];
    uint32_t chunkSize;
    char wave[4];
    char fmt[4];
    uint32_t subchunk1Size;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    char data[4];
    uint32_t dataSize;
};

static_assert(sizeof(WavHeader) == 44, "WAV header must be 44 bytes");

RecordFrame *recordFrames = nullptr;
QueueHandle_t recordFreeQueue = nullptr;
QueueHandle_t recordFilledQueue = nullptr;
QueueHandle_t sdCommandQueue = nullptr;
SemaphoreHandle_t sdStateMutex = nullptr;

volatile bool sdCardReady = false;
volatile bool sdRecordingActive = false;
volatile bool sdStopping = false;
volatile SdStatus sdStatus = SdStatus::NotReady;
char sdCurrentFileName[32] = "-";
uint32_t sdChunksWritten = 0;
uint64_t sdBytesWritten = 0;
uint32_t sdErrors = 0;
uint32_t sdQueueDrops = 0;
uint32_t sdQueueDropsTotal = 0;
uint32_t sdQueueHighWater = 0;
int64_t sdTotalWriteUs = 0;
int64_t sdMaxWriteUs = 0;

struct DebouncedButton {
    int pin = -1;
    bool stablePressed = false;
    bool lastReading = false;
    uint32_t lastChangeMs = 0;

    void begin(int gpio) {
        pin = gpio;
        pinMode(pin, INPUT_PULLUP);
        stablePressed = digitalRead(pin) == LOW;
        lastReading = stablePressed;
        lastChangeMs = millis();
    }

    static constexpr int kPressed  =  1;
    static constexpr int kReleased = -1;
    static constexpr int kNoChange =  0;

    // Call once per loop iteration. Returns kPressed, kReleased, or kNoChange.
    // Must not be called twice per loop for the same button — a single call
    // advances the debounce state, so a second call in the same iteration would
    // consume the event before the caller can act on it.
    int update() {
        const bool reading = digitalRead(pin) == LOW;
        const uint32_t now = millis();

        if (reading != lastReading) {
            lastReading = reading;
            lastChangeMs = now;
        }

        if (reading != stablePressed && (now - lastChangeMs) >= 35) {
            stablePressed = reading;
            return stablePressed ? kPressed : kReleased;
        }

        return kNoChange;
    }
};

DebouncedButton topButton;
DebouncedButton sideButton;

void notifyDisplay();

uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t c = ((uint16_t)(r & 0xF8) << 8) |
                 ((uint16_t)(g & 0xFC) << 3) |
                 ((uint16_t)b >> 3);
    return (c >> 8) | (c << 8);
}

struct AnalogVuTheme {
    uint16_t face;
    uint16_t faceStripe;
    uint16_t faceGlow;
    uint16_t border;
    uint16_t innerArc;
    uint16_t greenArc;
    uint16_t amberArc;
    uint16_t redArc;
    uint16_t tick;
    uint16_t needle;
    uint16_t label;
    uint16_t labelShadow;
    uint16_t pivotOuter;
    uint16_t pivotInner;
    uint16_t clipOn;
    uint16_t clipOff;
    uint16_t clipHighlightOn;
    uint16_t clipHighlightOff;
    int glowCenterX;
    int glowCenterY;
    int glowRadius;
    int glowStrength;
    int arcPointHalfThickness;
    int outerArcOutwardPixels;
    int minorTickHalfThickness;
    int majorTickHalfThickness;
    int needleHalfThickness;
    int labelX;
    int labelY;
    int labelScale;
    int labelHalfThickness;
    int labelShadowOffsetX;
    int labelShadowOffsetY;
    int clipX;
    int clipY;
    int clipRadius;
    int clipHighlightOffsetX;
    int clipHighlightOffsetY;
    int clipHighlightRadius;
};

const AnalogVuTheme kAnalogVuLightTheme = {
    .face = rgb565(246, 246, 238),
    .faceStripe = rgb565(238, 238, 230),
    .faceGlow = rgb565(252, 236, 174),
    .border = rgb565(12, 14, 16),
    .innerArc = rgb565(222, 224, 216),
    .greenArc = rgb565(18, 150, 58),
    .amberArc = rgb565(220, 165, 28),
    .redArc = rgb565(210, 20, 18),
    .tick = rgb565(86, 88, 84),
    .needle = rgb565(185, 16, 18),
    .label = rgb565(18, 18, 16),
    .labelShadow = rgb565(218, 218, 210),
    .pivotOuter = rgb565(12, 14, 16),
    .pivotInner = rgb565(178, 178, 170),
    .clipOn = rgb565(210, 20, 18),
    .clipOff = rgb565(210, 210, 202),
    .clipHighlightOn = rgb565(255, 180, 160),
    .clipHighlightOff = rgb565(236, 236, 226),
    .glowCenterX = kAnalogVuDisplayWidth / 2,
    .glowCenterY = kAnalogVuDisplayHeight + 18,
    .glowRadius = 118,
    .glowStrength = 26,
    .arcPointHalfThickness = 1,
    .outerArcOutwardPixels = 3,
    .minorTickHalfThickness = 1,
    .majorTickHalfThickness = 2,
    .needleHalfThickness = 2,
    .labelX = 70,
    .labelY = 74,
    .labelScale = 3,
    .labelHalfThickness = 1,
    .labelShadowOffsetX = 1,
    .labelShadowOffsetY = 1,
    .clipX = kAnalogVuDisplayWidth - 19,
    .clipY = 20,
    .clipRadius = 6,
    .clipHighlightOffsetX = 2,
    .clipHighlightOffsetY = -2,
    .clipHighlightRadius = 2,
};

const AnalogVuTheme kAnalogVuDarkTheme = {
    .face = rgb565(15, 18, 22),
    .faceStripe = rgb565(21, 25, 30),
    .faceGlow = rgb565(48, 42, 30),
    .border = rgb565(190, 204, 208),
    .innerArc = rgb565(58, 66, 72),
    .greenArc = rgb565(35, 210, 96),
    .amberArc = rgb565(238, 188, 48),
    .redArc = rgb565(245, 55, 45),
    .tick = rgb565(150, 160, 164),
    .needle = rgb565(255, 82, 76),
    .label = rgb565(215, 224, 226),
    .labelShadow = rgb565(0, 0, 0),
    .pivotOuter = rgb565(190, 204, 208),
    .pivotInner = rgb565(72, 82, 88),
    .clipOn = rgb565(255, 28, 22),
    .clipOff = rgb565(52, 10, 12),
    .clipHighlightOn = rgb565(255, 190, 168),
    .clipHighlightOff = rgb565(94, 28, 26),
    .glowCenterX = kAnalogVuDisplayWidth / 2,
    .glowCenterY = kAnalogVuDisplayHeight + 24,
    .glowRadius = 120,
    .glowStrength = 34,
    .arcPointHalfThickness = 1,
    .outerArcOutwardPixels = 4,
    .minorTickHalfThickness = 1,
    .majorTickHalfThickness = 2,
    .needleHalfThickness = 2,
    .labelX = 70,
    .labelY = 74,
    .labelScale = 3,
    .labelHalfThickness = 1,
    .labelShadowOffsetX = 1,
    .labelShadowOffsetY = 1,
    .clipX = kAnalogVuDisplayWidth - 19,
    .clipY = 20,
    .clipRadius = 6,
    .clipHighlightOffsetX = 2,
    .clipHighlightOffsetY = -2,
    .clipHighlightRadius = 2,
};

const AnalogVuTheme kAnalogVuIncandescentTheme = {
    .face = rgb565(76, 47, 18),
    .faceStripe = rgb565(92, 56, 20),
    .faceGlow = rgb565(255, 202, 82),
    .border = rgb565(34, 18, 8),
    .innerArc = rgb565(112, 70, 26),
    .greenArc = rgb565(96, 90, 30),
    .amberArc = rgb565(206, 125, 22),
    .redArc = rgb565(184, 38, 28),
    .tick = rgb565(62, 34, 14),
    .needle = rgb565(36, 18, 8),
    .label = rgb565(42, 22, 8),
    .labelShadow = rgb565(170, 98, 28),
    .pivotOuter = rgb565(36, 18, 8),
    .pivotInner = rgb565(130, 75, 26),
    .clipOn = rgb565(255, 44, 24),
    .clipOff = rgb565(84, 30, 10),
    .clipHighlightOn = rgb565(255, 176, 96),
    .clipHighlightOff = rgb565(130, 62, 24),
    .glowCenterX = kAnalogVuDisplayWidth / 2,
    .glowCenterY = kAnalogVuDisplayHeight + 18,
    .glowRadius = 160,
    .glowStrength = 230,
    .arcPointHalfThickness = 1,
    .outerArcOutwardPixels = 4,
    .minorTickHalfThickness = 1,
    .majorTickHalfThickness = 2,
    .needleHalfThickness = 2,
    .labelX = 70,
    .labelY = 74,
    .labelScale = 3,
    .labelHalfThickness = 1,
    .labelShadowOffsetX = 1,
    .labelShadowOffsetY = 1,
    .clipX = kAnalogVuDisplayWidth - 19,
    .clipY = 20,
    .clipRadius = 6,
    .clipHighlightOffsetX = 2,
    .clipHighlightOffsetY = -2,
    .clipHighlightRadius = 2,
};

const AnalogVuTheme &activeAnalogVuTheme = kAnalogVuIncandescentTheme;

uint8_t clampToByte(float value) {
    if (value <= 0.0f) {
        return 0;
    }
    if (value >= 255.0f) {
        return 255;
    }
    return (uint8_t)(value + 0.5f);
}

float percentOf(int64_t partUs, int64_t wholeUs) {
    if (wholeUs <= 0) {
        return 0.0f;
    }
    return ((float)partUs * 100.0f) / (float)wholeUs;
}

bool wifiHasUsableIp() {
    const IPAddress ip = WiFi.localIP();
    return ip[0] != 0 || ip[1] != 0 || ip[2] != 0 || ip[3] != 0;
}

bool wifiReadyForServices() {
    return wifiGotIpEvent || WiFi.status() == WL_CONNECTED ||
           (wifiHasUsableIp() && WiFi.SSID().length() > 0);
}

void onWiFiEvent(arduino_event_id_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            wifiGotIpEvent = true;
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        case ARDUINO_EVENT_WIFI_STA_STOP:
            wifiGotIpEvent = false;
            break;
        default:
            break;
    }
}

void ensureWiFiEventHandler() {
    static bool registered = false;
    if (!registered) {
        WiFi.onEvent(onWiFiEvent);
        registered = true;
    }
}

void lockSerial() {
    if (serialMutex != nullptr) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);
    }
}

void unlockSerial() {
    if (serialMutex != nullptr) {
        xSemaphoreGive(serialMutex);
    }
}

void lockSdState() {
    if (sdStateMutex != nullptr) {
        xSemaphoreTake(sdStateMutex, portMAX_DELAY);
    }
}

void unlockSdState() {
    if (sdStateMutex != nullptr) {
        xSemaphoreGive(sdStateMutex);
    }
}

const char *sdStatusName(SdStatus status) {
    switch (status) {
        case SdStatus::NotReady:
            return "not_ready";
        case SdStatus::Idle:
            return "idle";
        case SdStatus::Starting:
            return "starting";
        case SdStatus::Recording:
            return "recording";
        case SdStatus::Stopping:
            return "stopping";
        case SdStatus::Error:
            return "error";
    }
    return "unknown";
}

void setSdStatus(SdStatus status, const char *fileName = nullptr) {
    lockSdState();
    sdStatus = status;
    if (fileName != nullptr) {
        strlcpy(sdCurrentFileName, fileName, sizeof(sdCurrentFileName));
    }
    unlockSdState();

    sdRecordingActive = status == SdStatus::Recording;
    sdStopping = status == SdStatus::Starting || status == SdStatus::Stopping;
    frameDirty = true;
    notifyDisplay();
}

SdStatus getSdStatusSnapshot(char *fileName, size_t fileNameSize) {
    lockSdState();
    const SdStatus status = sdStatus;
    if (fileName != nullptr && fileNameSize > 0) {
        strlcpy(fileName, sdCurrentFileName, fileNameSize);
    }
    unlockSdState();
    return status;
}

void buildPalette() {
    struct Stop {
        uint8_t at;
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    constexpr Stop stops[] = {
        {0, 0, 0, 0},
        {32, 0, 0, 42},
        {88, 0, 34, 160},
        {140, 0, 210, 220},
        {190, 245, 220, 36},
        {230, 255, 72, 20},
        {255, 255, 255, 255},
    };

    size_t stopIndex = 0;
    for (int i = 0; i < 256; ++i) {
        while (stopIndex + 1 < sizeof(stops) / sizeof(stops[0]) &&
               i > stops[stopIndex + 1].at) {
            ++stopIndex;
        }

        const Stop &a = stops[stopIndex];
        const Stop &b = stops[stopIndex + 1];
        const float t = (float)(i - a.at) / (float)(b.at - a.at);
        const uint8_t r = (uint8_t)(a.r + (b.r - a.r) * t);
        const uint8_t g = (uint8_t)(a.g + (b.g - a.g) * t);
        const uint8_t bl = (uint8_t)(a.b + (b.b - a.b) * t);
        palette[i] = rgb565(r, g, bl);
    }
}

void notifyDisplay() {
    if (displayTaskHandle != nullptr) {
        xTaskNotifyGive(displayTaskHandle);
    }
}

void pumpStartupUi(uint32_t durationMs = 0) {
    if (!startupUiReady) {
        if (durationMs > 0) {
            delay(durationMs);
        }
        return;
    }

    const uint32_t startMs = millis();
    do {
        lv_timer_handler();
        if (durationMs == 0) {
            lv_refr_now(nullptr);
        }
        if (durationMs == 0) {
            break;
        }
        delay(10);
    } while ((uint32_t)(millis() - startMs) < durationMs);
}

void startupScreenStatus(const char *status, const char *detail = nullptr,
                         int progressPercent = -1) {
    if (!startupUiReady || !startupUiActive) {
        return;
    }

    if (startupStatusLabel != nullptr) {
        lv_label_set_text(startupStatusLabel, status != nullptr ? status : "");
    }
    if (startupDetailLabel != nullptr) {
        lv_label_set_text(startupDetailLabel, detail != nullptr ? detail : "");
    }
    if (startupProgressBar != nullptr && progressPercent >= 0) {
        if (progressPercent > 100) {
            progressPercent = 100;
        }
        lv_bar_set_value(startupProgressBar, progressPercent, LV_ANIM_OFF);
    }

    pumpStartupUi();
}

void beginStartupScreen() {
    beginLvglHelper(amoled);
    startupUiReady = true;
    startupUiActive = true;

    lv_obj_t *screen = lv_scr_act();
    lv_obj_clean(screen);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x061014), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    startupTitleLabel = lv_label_create(screen);
    lv_label_set_text(startupTitleLabel, "t4-i2s");
    lv_obj_set_style_text_color(startupTitleLabel, lv_color_hex(0xe7fbff),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(startupTitleLabel, &lv_font_montserrat_36,
                               LV_PART_MAIN);
    lv_obj_align(startupTitleLabel, LV_ALIGN_TOP_MID, 0, 54);

    lv_obj_t *subtitle = lv_label_create(screen);
    lv_label_set_text(subtitle, "I2S recorder startup");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0x7ec8d8),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_18,
                               LV_PART_MAIN);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 100);

    startupStatusLabel = lv_label_create(screen);
    lv_obj_set_width(startupStatusLabel, 500);
    lv_label_set_long_mode(startupStatusLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(startupStatusLabel, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_color(startupStatusLabel, lv_color_hex(0xffffff),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(startupStatusLabel, &lv_font_montserrat_24,
                               LV_PART_MAIN);
    lv_obj_align(startupStatusLabel, LV_ALIGN_CENTER, 0, -26);

    startupDetailLabel = lv_label_create(screen);
    lv_obj_set_width(startupDetailLabel, 520);
    lv_label_set_long_mode(startupDetailLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(startupDetailLabel, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_color(startupDetailLabel, lv_color_hex(0xa8bcc4),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(startupDetailLabel, &lv_font_montserrat_16,
                               LV_PART_MAIN);
    lv_obj_align(startupDetailLabel, LV_ALIGN_CENTER, 0, 24);

    startupProgressBar = lv_bar_create(screen);
    lv_obj_set_size(startupProgressBar, 360, 12);
    lv_bar_set_range(startupProgressBar, 0, 100);
    lv_obj_set_style_radius(startupProgressBar, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(startupProgressBar, 4, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(startupProgressBar, lv_color_hex(0x21313a),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_color(startupProgressBar, lv_color_hex(0x26d7a1),
                              LV_PART_INDICATOR);
    lv_obj_align(startupProgressBar, LV_ALIGN_BOTTOM_MID, 0, -86);

    startupFooterLabel = lv_label_create(screen);
    lv_label_set_text(startupFooterLabel, "WiFi is optional; recording can run offline");
    lv_obj_set_style_text_color(startupFooterLabel, lv_color_hex(0x6f8990),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(startupFooterLabel, &lv_font_montserrat_14,
                               LV_PART_MAIN);
    lv_obj_align(startupFooterLabel, LV_ALIGN_BOTTOM_MID, 0, -42);

    startupScreenStatus("Display ready", "Initialising hardware", 5);
}

void finishStartupScreen() {
    if (!startupUiReady || !startupUiActive) {
        return;
    }

    startupScreenStatus("Starting live displays",
                        "Switching from LVGL startup screen", 100);
    pumpStartupUi(250);
    startupUiActive = false;

    lv_obj_clean(lv_scr_act());
    pumpStartupUi();
}

int font16CharWidth(char c) {
    if (c < firstchr_f16 || c >= firstchr_f16 + nr_chrs_f16) {
        return 0;
    }
    return pgm_read_byte(widtbl_f16 + (c - firstchr_f16));
}

int font16TextWidth(const char *text) {
    int width = 0;
    if (text == nullptr) {
        return width;
    }

    while (*text != '\0') {
        const int charWidth = font16CharWidth(*text++);
        if (charWidth > 0) {
            width += charWidth + 1;
        }
    }
    return width > 0 ? width - 1 : 0;
}

int font32CharWidth(char c) {
    if (c < firstchr_f32 || c >= firstchr_f32 + nr_chrs_f32) {
        return 0;
    }
    return pgm_read_byte(widtbl_f32 + (c - firstchr_f32));
}

int font32TextWidth(const char *text) {
    int width = 0;
    if (text == nullptr) {
        return width;
    }

    while (*text != '\0') {
        const int charWidth = font32CharWidth(*text++);
        if (charWidth > 0) {
            width += charWidth + 1;
        }
    }
    return width > 0 ? width - 1 : 0;
}

void fitFont16Text(const char *source, char *dest, size_t destSize,
                   int maxWidth) {
    if (dest == nullptr || destSize == 0) {
        return;
    }

    dest[0] = '\0';
    if (source == nullptr) {
        return;
    }

    int width = 0;
    size_t out = 0;
    while (*source != '\0' && out + 1 < destSize) {
        const int charWidth = font16CharWidth(*source);
        if (charWidth <= 0) {
            ++source;
            continue;
        }

        const int nextWidth = width == 0 ? charWidth : width + 1 + charWidth;
        if (nextWidth > maxWidth) {
            break;
        }

        dest[out++] = *source++;
        width = nextWidth;
    }
    dest[out] = '\0';
}

void fitFont32Text(const char *source, char *dest, size_t destSize,
                   int maxWidth) {
    if (dest == nullptr || destSize == 0) {
        return;
    }

    dest[0] = '\0';
    if (source == nullptr) {
        return;
    }

    int width = 0;
    size_t out = 0;
    while (*source != '\0' && out + 1 < destSize) {
        const int charWidth = font32CharWidth(*source);
        if (charWidth <= 0) {
            ++source;
            continue;
        }

        const int nextWidth = width == 0 ? charWidth : width + 1 + charWidth;
        if (nextWidth > maxWidth) {
            break;
        }

        dest[out++] = *source++;
        width = nextWidth;
    }
    dest[out] = '\0';
}

bool staticPanelOriginForAbsolutePixel(int x, int y, int &originX,
                                       int &originY) {
    const bool left = x >= kStaticPanelLeftX &&
                      x < kStaticPanelLeftX + kStaticPanelWidth;
    const bool right = x >= kStaticPanelRightX &&
                       x < kStaticPanelRightX + kStaticPanelWidth;
    const bool top = y >= kStaticPanelTopY &&
                     y < kStaticPanelTopY + kStaticPanelHeight;
    const bool bottom = y >= kStaticPanelBottomY &&
                        y < kStaticPanelBottomY + kStaticPanelHeight;

    if (left && top) {
        originX = kStaticPanelLeftX;
        originY = kStaticPanelTopY;
        return true;
    }
    if (right && top) {
        originX = kStaticPanelRightX;
        originY = kStaticPanelTopY;
        return true;
    }
    if (left && bottom) {
        originX = kStaticPanelLeftX;
        originY = kStaticPanelBottomY;
        return true;
    }
    if (right && bottom) {
        originX = kStaticPanelRightX;
        originY = kStaticPanelBottomY;
        return true;
    }

    return false;
}

uint16_t staticPanelBackdropColor(int x, int y) {
    int originX = 0;
    int originY = 0;
    if (!staticPanelOriginForAbsolutePixel(x, y, originX, originY)) {
        return kColorBlack;
    }

    const int localX = x - originX;
    const int localY = y - originY;
    const bool grid = (localX % 50 == 0) || (localY % 27 == 0);
    return grid ? kColorAnalyzerGrid : kColorBlack;
}

void fillStaticPanelBackdrop(uint16_t *pixels, int x, int y, int width,
                             int height) {
    for (int yy = 0; yy < height; ++yy) {
        for (int xx = 0; xx < width; ++xx) {
            pixels[yy * width + xx] =
                staticPanelBackdropColor(x + xx, y + yy);
        }
    }
}

void plotWifiLabelPixel(int x, int y, uint16_t color) {
    if (x < 0 || x >= kWifiLabelWidth || y < 0 || y >= kWifiLabelHeight) {
        return;
    }
    wifiLabelPixels[y * kWifiLabelWidth + x] = color;
}

void drawFont16CharToWifiLabel(int x, int y, char c, uint16_t color) {
    if (c < firstchr_f16 || c >= firstchr_f16 + nr_chrs_f16) {
        return;
    }

    const int charIndex = c - firstchr_f16;
    const int charWidth = pgm_read_byte(widtbl_f16 + charIndex);
    const int rowBytes = (charWidth + 6) / 8;
    const uint8_t *glyph =
        (const uint8_t *)pgm_read_ptr(&chrtbl_f16[charIndex]);

    for (int row = 0; row < chr_hgt_f16; ++row) {
        for (int byteIndex = 0; byteIndex < rowBytes; ++byteIndex) {
            const uint8_t bits = pgm_read_byte(glyph + row * rowBytes + byteIndex);
            for (int bit = 0; bit < 8; ++bit) {
                const int px = byteIndex * 8 + bit;
                if (px >= charWidth) {
                    break;
                }
                if ((bits & (0x80 >> bit)) != 0) {
                    plotWifiLabelPixel(x + px, y + row, color);
                }
            }
        }
    }
}

void drawFont32CharToWifiLabel(int x, int y, char c, uint16_t color) {
    if (c < firstchr_f32 || c >= firstchr_f32 + nr_chrs_f32) {
        return;
    }

    const int charIndex = c - firstchr_f32;
    const int charWidth = pgm_read_byte(widtbl_f32 + charIndex);
    const uint8_t *glyph =
        (const uint8_t *)pgm_read_ptr(&chrtbl_f32[charIndex]);

    int pc = 0;
    const int pixelCount = charWidth * chr_hgt_f32;
    while (pc < pixelCount) {
        uint8_t run = pgm_read_byte(glyph++);
        const bool foreground = (run & 0x80) != 0;
        run = (run & 0x7F) + 1;

        while (run-- > 0 && pc < pixelCount) {
            if (foreground) {
                plotWifiLabelPixel(x + (pc % charWidth),
                                   y + (pc / charWidth), color);
            }
            ++pc;
        }
    }
}

void drawFont16StringToWifiLabel(const char *text, int rightX, int y,
                                 uint16_t color) {
    if (text == nullptr || text[0] == '\0') {
        return;
    }

    int x = rightX - font16TextWidth(text);
    while (*text != '\0') {
        const int charWidth = font16CharWidth(*text);
        if (charWidth > 0) {
            drawFont16CharToWifiLabel(x, y, *text, color);
            x += charWidth + 1;
        }
        ++text;
    }
}

void drawFont32StringToWifiLabel(const char *text, int rightX, int y,
                                 uint16_t color) {
    if (text == nullptr || text[0] == '\0') {
        return;
    }

    int x = rightX - font32TextWidth(text);
    while (*text != '\0') {
        const int charWidth = font32CharWidth(*text);
        if (charWidth > 0) {
            drawFont32CharToWifiLabel(x, y, *text, color);
            x += charWidth + 1;
        }
        ++text;
    }
}

void drawWifiStatusLabelOnce() {
    if (!wifiReadyForServices()) {
        return;
    }

    fillStaticPanelBackdrop(wifiLabelPixels, kWifiLabelX, kWifiLabelY,
                            kWifiLabelWidth, kWifiLabelHeight);

    char ssid[48];
    char ip[24];
    fitFont16Text(WiFi.SSID().c_str(), ssid, sizeof(ssid), kWifiLabelWidth - 4);
    fitFont32Text(WiFi.localIP().toString().c_str(), ip, sizeof(ip),
                  kWifiLabelWidth - 4);

  //  const uint16_t ssidColor = rgb565(210, 238, 245);
//    const uint16_t ipColor = rgb565(125, 198, 214);

    const uint16_t ssidColor = rgb565(255, 255, 255);
    const uint16_t ipColor = rgb565(255, 255, 255);

    const int rightX = kWifiLabelWidth - 2;
    drawFont16StringToWifiLabel(ssid, rightX, 0, ssidColor);
    drawFont32StringToWifiLabel(ip, rightX, chr_hgt_f32 + 4, ipColor);

    amoled.pushColors(kWifiLabelX, kWifiLabelY, kWifiLabelWidth,
                      kWifiLabelHeight, wifiLabelPixels);
}

void plotCalibrationLabelPixel(int x, int y, uint16_t color) {
    if (x < 0 || x >= kCalibrationLabelWidth ||
        y < 0 || y >= kCalibrationLabelHeight) {
        return;
    }
    calibrationLabelPixels[y * kCalibrationLabelWidth + x] = color;
}

void drawFont16CharToCalibrationLabel(int x, int y, char c, uint16_t color) {
    if (c < firstchr_f16 || c >= firstchr_f16 + nr_chrs_f16) {
        return;
    }

    const int charIndex = c - firstchr_f16;
    const int charWidth = pgm_read_byte(widtbl_f16 + charIndex);
    const int rowBytes = (charWidth + 6) / 8;
    const uint8_t *glyph =
        (const uint8_t *)pgm_read_ptr(&chrtbl_f16[charIndex]);

    for (int row = 0; row < chr_hgt_f16; ++row) {
        for (int byteIndex = 0; byteIndex < rowBytes; ++byteIndex) {
            const uint8_t bits =
                pgm_read_byte(glyph + row * rowBytes + byteIndex);
            for (int bit = 0; bit < 8; ++bit) {
                const int px = byteIndex * 8 + bit;
                if (px >= charWidth) {
                    break;
                }
                if ((bits & (0x80 >> bit)) != 0) {
                    plotCalibrationLabelPixel(x + px, y + row, color);
                }
            }
        }
    }
}

void drawFont32CharToCalibrationLabel(int x, int y, char c, uint16_t color) {
    if (c < firstchr_f32 || c >= firstchr_f32 + nr_chrs_f32) {
        return;
    }

    const int charIndex = c - firstchr_f32;
    const int charWidth = pgm_read_byte(widtbl_f32 + charIndex);
    const uint8_t *glyph =
        (const uint8_t *)pgm_read_ptr(&chrtbl_f32[charIndex]);

    int pc = 0;
    const int pixelCount = charWidth * chr_hgt_f32;
    while (pc < pixelCount) {
        uint8_t run = pgm_read_byte(glyph++);
        const bool foreground = (run & 0x80) != 0;
        run = (run & 0x7F) + 1;

        while (run-- > 0 && pc < pixelCount) {
            if (foreground) {
                plotCalibrationLabelPixel(x + (pc % charWidth),
                                          y + (pc / charWidth), color);
            }
            ++pc;
        }
    }
}

void drawFont16StringToCalibrationLabel(const char *text, int x, int y,
                                        uint16_t color) {
    if (text == nullptr) {
        return;
    }

    while (*text != '\0') {
        const int charWidth = font16CharWidth(*text);
        if (charWidth > 0) {
            drawFont16CharToCalibrationLabel(x, y, *text, color);
            x += charWidth + 1;
        }
        ++text;
    }
}

void drawFont32StringToCalibrationLabel(const char *text, int x, int y,
                                        uint16_t color) {
    if (text == nullptr) {
        return;
    }

    while (*text != '\0') {
        const int charWidth = font32CharWidth(*text);
        if (charWidth > 0) {
            drawFont32CharToCalibrationLabel(x, y, *text, color);
            x += charWidth + 1;
        }
        ++text;
    }
}

void renderCalibrationStatusLabel() {
    static bool labelVisible = false;
    const bool shouldShow = !displayNoiseCalibrated;
    if (!shouldShow && !labelVisible) {
        return;
    }

    fillStaticPanelBackdrop(calibrationLabelPixels, kCalibrationLabelX,
                            kCalibrationLabelY, kCalibrationLabelWidth,
                            kCalibrationLabelHeight);

    if (shouldShow) {
        char label[24];
        fitFont32Text("Calibrating", label, sizeof(label),
                      kCalibrationLabelWidth - 4);
        const int x = kCalibrationLabelWidth - 2 - font32TextWidth(label);
        drawFont32StringToCalibrationLabel(label, x, 0,
                                           rgb565(0, 180, 0));
    }

    amoled.pushColors(kCalibrationLabelX, kCalibrationLabelY,
                      kCalibrationLabelWidth, kCalibrationLabelHeight,
                      calibrationLabelPixels);
    labelVisible = shouldShow;
}

void pushSolidRect(int x, int y, int width, int height, uint16_t color) {
    if (width <= 0 || height <= 0 || x < 0 || y < 0 ||
        x + width > amoled.width() || y + height > amoled.height()) {
        return;
    }

    const size_t bufferPixels = sizeof(graphPixels) / sizeof(graphPixels[0]);
    int rowsPerChunk = (int)(bufferPixels / (size_t)width);
    if (rowsPerChunk < 1) {
        rowsPerChunk = 1;
    }

    for (size_t i = 0; i < bufferPixels; ++i) {
        graphPixels[i] = color;
    }

    for (int row = 0; row < height; row += rowsPerChunk) {
        int chunkRows = height - row;
        if (chunkRows > rowsPerChunk) {
            chunkRows = rowsPerChunk;
        }
        amoled.pushColors(x, y + row, width, chunkRows, graphPixels);
        if (((row / rowsPerChunk) & 0x03) == 0x03) {
            yield();
        }
    }
}

void pushStaticGridRect(int x, int y, int width, int height) {
    if (width <= 0 || height <= 0 || x < 0 || y < 0 ||
        x + width > amoled.width() || y + height > amoled.height()) {
        return;
    }

    const size_t bufferPixels = sizeof(graphPixels) / sizeof(graphPixels[0]);
    int rowsPerChunk = (int)(bufferPixels / (size_t)width);
    if (rowsPerChunk < 1) {
        rowsPerChunk = 1;
    }

    for (int row = 0; row < height; row += rowsPerChunk) {
        int chunkRows = height - row;
        if (chunkRows > rowsPerChunk) {
            chunkRows = rowsPerChunk;
        }

        fillStaticPanelBackdrop(graphPixels, x, y + row, width, chunkRows);
        amoled.pushColors(x, y + row, width, chunkRows, graphPixels);
        if (((row / rowsPerChunk) & 0x03) == 0x03) {
            yield();
        }
    }
}

void drawStaticBackgroundPanels() {
    pushStaticGridRect(kStaticPanelLeftX, kStaticPanelTopY,
                       kStaticPanelWidth, kStaticPanelHeight);
    pushStaticGridRect(kStaticPanelRightX, kStaticPanelTopY,
                       kStaticPanelWidth, kStaticPanelHeight);
    pushStaticGridRect(kStaticPanelLeftX, kStaticPanelBottomY,
                       kStaticPanelWidth, kStaticPanelHeight);
    pushStaticGridRect(kStaticPanelRightX, kStaticPanelBottomY,
                       kStaticPanelWidth, kStaticPanelHeight);
}

void clearScreen() {
    pushSolidRect(0, 0, amoled.width(), amoled.height(), kColorBlack);
}

void drawFrameColor(uint16_t color) {
    // borders must be even as pushColors / display driver cannot deal with odd numbers of pixels.
    constexpr int border = 6;
    const int w = amoled.width();
    const int h = amoled.height();

    pushSolidRect(0, 0, w, border+12, color);
    pushSolidRect(0, h - border, w, border, color);
    pushSolidRect(0, 0, border, h, color);
    pushSolidRect(w - border, 0, border, h, color);
}

void drawFrame() {
    const uint16_t color = !i2sReady || !fftReady
                               ? kColorFrameError
                               : sdRecordingActive ? kColorFrameRecording
                               : sdStopping       ? kColorFrameStopping
                               : captureEnabled   ? kColorFrameActive
                                                  : kColorFramePaused;
    drawFrameColor(color);
}

struct PixelRect {
    int x;
    int y;
    int width;
    int height;
};

PixelRect emptyRect() {
    return {0, 0, 0, 0};
}

bool rectValid(const PixelRect &rect) {
    return rect.width > 0 && rect.height > 0;
}

PixelRect clampRect(PixelRect rect, int maxWidth, int maxHeight) {
    const int x0 = max(0, rect.x);
    const int y0 = max(0, rect.y);
    const int x1 = min(maxWidth, rect.x + rect.width);
    const int y1 = min(maxHeight, rect.y + rect.height);
    if (x1 <= x0 || y1 <= y0) {
        return emptyRect();
    }
    return {x0, y0, x1 - x0, y1 - y0};
}

PixelRect rectFromBounds(int x0, int y0, int x1, int y1) {
    if (x1 < x0 || y1 < y0) {
        return emptyRect();
    }
    return {x0, y0, x1 - x0 + 1, y1 - y0 + 1};
}

PixelRect unionRect(const PixelRect &a, const PixelRect &b) {
    if (!rectValid(a)) {
        return b;
    }
    if (!rectValid(b)) {
        return a;
    }

    const int x0 = min(a.x, b.x);
    const int y0 = min(a.y, b.y);
    const int x1 = max(a.x + a.width, b.x + b.width);
    const int y1 = max(a.y + a.height, b.y + b.height);
    return {x0, y0, x1 - x0, y1 - y0};
}

PixelRect expandRect(PixelRect rect, int padding, int maxWidth,
                     int maxHeight) {
    if (!rectValid(rect)) {
        return rect;
    }
    rect.x -= padding;
    rect.y -= padding;
    rect.width += padding * 2;
    rect.height += padding * 2;
    return clampRect(rect, maxWidth, maxHeight);
}

PixelRect alignRectForPushColors(PixelRect rect, int displayX, int maxWidth,
                                 int maxHeight) {
    rect = clampRect(rect, maxWidth, maxHeight);
    if (!rectValid(rect)) {
        return rect;
    }

    int left = rect.x;
    int right = rect.x + rect.width;

    if (((displayX + left) & 0x01) != 0) {
        if (left > 0) {
            --left;
        } else {
            ++right;
        }
    }

    if (((right - left) & 0x01) != 0) {
        if (right < maxWidth) {
            ++right;
        } else if (left > 0) {
            --left;
        }
    }

    return clampRect({left, rect.y, right - left, rect.height},
                     maxWidth, maxHeight);
}

void copyBaseRectToGraph(const uint16_t *base, int baseWidth,
                         const PixelRect &rect) {
    for (int row = 0; row < rect.height; ++row) {
        memcpy(&graphPixels[row * rect.width],
               &base[(rect.y + row) * baseWidth + rect.x],
               (size_t)rect.width * sizeof(uint16_t));
    }
}

bool initVuRenderBuffers() {
    if (vuBasePixels != nullptr && analogVuBasePixels != nullptr) {
        return true;
    }

    if (vuBasePixels == nullptr) {
        vuBasePixels = (uint16_t *)ps_malloc(
            (size_t)kVuDisplayWidth * kVuDisplayHeight * sizeof(uint16_t));
    }
    if (analogVuBasePixels == nullptr) {
        analogVuBasePixels = (uint16_t *)ps_malloc(
            (size_t)kAnalogVuDisplayWidth * kAnalogVuDisplayHeight *
            sizeof(uint16_t));
    }

    if (vuBasePixels == nullptr || analogVuBasePixels == nullptr) {
        lockSerial();
        Serial.println("VU render buffer allocation failed");
        unlockSerial();
        return false;
    }
    return true;
}

bool initAnalyzerRenderBuffer() {
    if (analyzerBasePixels != nullptr) {
        return true;
    }

    analyzerBasePixels = (uint16_t *)ps_malloc(
        (size_t)kAnalyzerDisplayWidth * kAnalyzerDisplayHeight *
        sizeof(uint16_t));

    if (analyzerBasePixels == nullptr) {
        lockSerial();
        Serial.println("analyzer render buffer allocation failed");
        unlockSerial();
        return false;
    }
    return true;
}

void clearSpectrogram() {
    if (spectrogramMutex != nullptr) {
        xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
        memset(spectrogram, 0, sizeof(spectrogram));
        memset(analyzerBarHeights, 0, sizeof(analyzerBarHeights));
        memset(analyzerLastBarHeights, 0, sizeof(analyzerLastBarHeights));
        memset(analyzerLevels, 0, sizeof(analyzerLevels));
        memset(scopeTraceTop, kScopeDisplayHeight / 2, sizeof(scopeTraceTop));
        memset(scopeTraceBottom, kScopeDisplayHeight / 2,
               sizeof(scopeTraceBottom));
        vuLevel = 0.0f;
        vuPeakLevel = 0.0f;
        vuSmoothedLevel = 0.0f;
        vuClipActive = false;
        vuClipHold = 0;
        spectrogramWriteColumn = 0;
        analyzerReady = false;
        analyzerScreenReady = false;
        scopeTraceReady = false;
        vuReady = false;
        vuScreenReady = false;
        analogVuScreenReady = false;
        xSemaphoreGive(spectrogramMutex);
    }
}

void renderSpectrogram() {
    if (spectrogramMutex == nullptr) {
        return;
    }

    uint8_t renderMax = 0;
    uint32_t renderNonzero = 0;

    xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
    const uint16_t nextColumn = spectrogramWriteColumn;

    for (int y = 0; y < kFftDisplayHeight; ++y) {
        for (int x = 0; x < kFftDisplayWidth; ++x) {
            const int srcColumn = (nextColumn + x) % kFftDisplayWidth;
            const uint8_t value = spectrogram[srcColumn][y];
            if (value > renderMax) {
                renderMax = value;
            }
            if (value > 0) {
                ++renderNonzero;
            }
            fftPixels[y * kFftDisplayWidth + x] = palette[value];
        }
    }
    xSemaphoreGive(spectrogramMutex);

    lastSpectrogramRenderMax = renderMax;
    lastSpectrogramRenderNonzero = renderNonzero;

    amoled.pushColors(kFftDisplayX, kFftDisplayY, kFftDisplayWidth,
                      kFftDisplayHeight, fftPixels);
}

uint16_t analyzerColorForY(int y) {
    const float t =
        (float)(kAnalyzerDisplayHeight - 1 - y) /
        (float)(kAnalyzerDisplayHeight - 1);

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    if (t < 0.55f) {
        const float local = t / 0.55f;
        r = (uint8_t)(20.0f + 215.0f * local);
        g = (uint8_t)(220.0f + 20.0f * local);
        b = (uint8_t)(70.0f - 50.0f * local);
    } else {
        const float local = (t - 0.55f) / 0.45f;
        r = 255;
        g = (uint8_t)(240.0f - 190.0f * local);
        b = (uint8_t)(20.0f - 20.0f * local);
    }

    return rgb565(r, g, b);
}

constexpr int kAnalyzerBarsLeft =
    (kAnalyzerDisplayWidth - kAnalyzerActiveWidth) / 2;
constexpr int kAnalyzerBarsBottom = kAnalyzerDisplayHeight - 4;
constexpr int kAnalyzerMaxBarHeight = kAnalyzerDisplayHeight - 8;

void fillAnalyzerBase(uint16_t *pixels) {
    for (int y = 0; y < kAnalyzerDisplayHeight; ++y) {
        for (int x = 0; x < kAnalyzerDisplayWidth; ++x) {
            const bool grid = (x % 50 == 0) || (y % 27 == 0);
            pixels[y * kAnalyzerDisplayWidth + x] =
                grid ? kColorAnalyzerGrid : kColorBlack;
        }
    }
}

bool buildAnalyzerBase() {
    if (!initAnalyzerRenderBuffer()) {
        return false;
    }
    fillAnalyzerBase(analyzerBasePixels);
    analyzerBaseReady = true;
    return true;
}

void drawAnalyzerBarsIntoRect(const uint8_t *barHeights, bool barsReady,
                              const PixelRect &rect) {
    if (barsReady) {
        for (int band = 0; band < kAnalyzerBandCount; ++band) {
            int height = barHeights[band];
            if (height > kAnalyzerMaxBarHeight) {
                height = kAnalyzerMaxBarHeight;
            }
            const int x0 =
                kAnalyzerBarsLeft + band * (kAnalyzerBandWidth + kAnalyzerBandGap);
            const int y0 = kAnalyzerBarsBottom - height + 1;
            const int drawX0 = max(x0, rect.x);
            const int drawX1 = min(x0 + kAnalyzerBandWidth,
                                   rect.x + rect.width);
            const int drawY0 = max(y0, rect.y);
            const int drawY1 = min(kAnalyzerBarsBottom + 1,
                                   rect.y + rect.height);
            if (drawX1 <= drawX0 || drawY1 <= drawY0) {
                continue;
            }
            for (int x = x0; x < x0 + kAnalyzerBandWidth; ++x) {
                if (x < drawX0 || x >= drawX1) {
                    continue;
                }
                for (int y = drawY0; y < drawY1; ++y) {
                    graphPixels[(y - rect.y) * rect.width + (x - rect.x)] =
                        analyzerColorForY(y);
                }
            }
        }
    }
}

void renderAnalyzerFull(const uint8_t *barHeights, bool barsReady) {
    fillAnalyzerBase(graphPixels);
    const PixelRect full = {0, 0, kAnalyzerDisplayWidth,
                            kAnalyzerDisplayHeight};
    drawAnalyzerBarsIntoRect(barHeights, barsReady, full);
    amoled.pushColors(kAnalyzerDisplayX, kAnalyzerDisplayY,
                      kAnalyzerDisplayWidth, kAnalyzerDisplayHeight,
                      graphPixels);
    memcpy(analyzerLastBarHeights, barHeights, sizeof(analyzerLastBarHeights));
    analyzerScreenReady = true;
}

int analyzerMaxHeight(const uint8_t *barHeights, bool barsReady) {
    if (!barsReady) {
        return 0;
    }

    int maxHeight = 0;
    for (int band = 0; band < kAnalyzerBandCount; ++band) {
        if (barHeights[band] > maxHeight) {
            maxHeight = barHeights[band];
        }
    }
    if (maxHeight > kAnalyzerMaxBarHeight) {
        maxHeight = kAnalyzerMaxBarHeight;
    }
    return maxHeight;
}

void renderAnalyzerDirty(const uint8_t *barHeights, bool barsReady) {
    if (!analyzerBaseReady && !buildAnalyzerBase()) {
        renderAnalyzerFull(barHeights, barsReady);
        return;
    }

    if (!analyzerScreenReady) {
        const PixelRect full = {0, 0, kAnalyzerDisplayWidth,
                                kAnalyzerDisplayHeight};
        copyBaseRectToGraph(analyzerBasePixels, kAnalyzerDisplayWidth, full);
        drawAnalyzerBarsIntoRect(barHeights, barsReady, full);
        amoled.pushColors(kAnalyzerDisplayX, kAnalyzerDisplayY,
                          kAnalyzerDisplayWidth, kAnalyzerDisplayHeight,
                          graphPixels);
        memcpy(analyzerLastBarHeights, barHeights,
               sizeof(analyzerLastBarHeights));
        analyzerScreenReady = true;
        return;
    }

    const int oldMaxHeight = analyzerMaxHeight(analyzerLastBarHeights, true);
    const int newMaxHeight = analyzerMaxHeight(barHeights, barsReady);
    const int dirtyHeight = max(oldMaxHeight, newMaxHeight);
    if (dirtyHeight <= 0) {
        memcpy(analyzerLastBarHeights, barHeights,
               sizeof(analyzerLastBarHeights));
        return;
    }

    const int dirtyY = kAnalyzerBarsBottom - dirtyHeight + 1;
    const PixelRect dirtyRect =
        alignRectForPushColors({0, dirtyY, kAnalyzerDisplayWidth,
                                kAnalyzerBarsBottom - dirtyY + 1},
                               kAnalyzerDisplayX, kAnalyzerDisplayWidth,
                               kAnalyzerDisplayHeight);
    if (!rectValid(dirtyRect)) {
        return;
    }

    copyBaseRectToGraph(analyzerBasePixels, kAnalyzerDisplayWidth, dirtyRect);
    drawAnalyzerBarsIntoRect(barHeights, barsReady, dirtyRect);
    amoled.pushColors(kAnalyzerDisplayX + dirtyRect.x,
                      kAnalyzerDisplayY + dirtyRect.y,
                      dirtyRect.width, dirtyRect.height, graphPixels);
    memcpy(analyzerLastBarHeights, barHeights, sizeof(analyzerLastBarHeights));
}

void renderAnalyzer() {
    if (spectrogramMutex == nullptr) {
        return;
    }

    uint8_t barHeights[kAnalyzerBandCount];
    bool barsReady = false;

    xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
    memcpy(barHeights, analyzerBarHeights, sizeof(barHeights));
    barsReady = analyzerReady;
    xSemaphoreGive(spectrogramMutex);

    if (kAnalyzerDirtyRectEnabled) {
        renderAnalyzerDirty(barHeights, barsReady);
    } else {
        renderAnalyzerFull(barHeights, barsReady);
    }
}

uint16_t vuColorForLevel(float level) {
    if (level < 0.72f) {
        return rgb565(34, 220, 78);
    }
    if (level < 0.88f) {
        return rgb565(246, 220, 52);
    }
    return rgb565(255, 64, 42);
}

constexpr int kVuMeterX = 12;
constexpr int kVuMeterWidth = kVuDisplayWidth - 24;
constexpr int kVuMeterHeight = 24;
constexpr int kVuMeterY = (kVuDisplayHeight - kVuMeterHeight) / 2;
constexpr int kVuTickTop = kVuMeterY - 16;
constexpr int kVuTickHeight = 10;
constexpr int kVuClipCenterX = kVuDisplayWidth - 23;
constexpr int kVuClipCenterY = 24;
constexpr int kVuClipRadius = 7;
constexpr int kVuPeakMarkerWidth = 2;

void setVuDrawTarget(uint16_t *pixels, int width, int height, int originX,
                     int originY) {
    vuDrawPixels = pixels;
    vuDrawWidth = width;
    vuDrawHeight = height;
    vuDrawOriginX = originX;
    vuDrawOriginY = originY;
}

void plotVuPixel(int x, int y, uint16_t color) {
    uint16_t *target = vuDrawPixels != nullptr ? vuDrawPixels : graphPixels;
    const int localX = x - vuDrawOriginX;
    const int localY = y - vuDrawOriginY;
    if (localX < 0 || localX >= vuDrawWidth ||
        localY < 0 || localY >= vuDrawHeight) {
        return;
    }
    target[localY * vuDrawWidth + localX] = color;
}

void fillVuRect(int x, int y, int width, int height, uint16_t color) {
    for (int yy = y; yy < y + height; ++yy) {
        for (int xx = x; xx < x + width; ++xx) {
            plotVuPixel(xx, yy, color);
        }
    }
}

void fillVuCircle(int centerX, int centerY, int radius, uint16_t color) {
    const int radiusSquared = radius * radius;
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            if (x * x + y * y <= radiusSquared) {
                plotVuPixel(centerX + x, centerY + y, color);
            }
        }
    }
}

void drawVuBox(int x, int y, int width, int height, uint16_t color) {
    fillVuRect(x, y, width, 1, color);
    fillVuRect(x, y + height - 1, width, 1, color);
    fillVuRect(x, y, 1, height, color);
    fillVuRect(x + width - 1, y, 1, height, color);
}

bool buildVuBase() {
    if (!initVuRenderBuffers()) {
        return false;
    }

    setVuDrawTarget(vuBasePixels, kVuDisplayWidth, kVuDisplayHeight, 0, 0);
    for (int y = 0; y < kVuDisplayHeight; ++y) {
        for (int x = 0; x < kVuDisplayWidth; ++x) {
            const bool grid = (x % 46 == 0) || (y % 27 == 0);
            vuBasePixels[y * kVuDisplayWidth + x] =
                grid ? kColorAnalyzerGrid : kColorBlack;
        }
    }

    const uint16_t dimGreen = rgb565(3, 38, 14);
    const uint16_t dimYellow = rgb565(44, 38, 4);
    const uint16_t dimRed = rgb565(50, 8, 6);
    const uint16_t border = rgb565(70, 96, 98);

    drawVuBox(kVuMeterX - 1, kVuMeterY - 1, kVuMeterWidth + 2,
              kVuMeterHeight + 2, border);

    for (int x = 0; x < kVuMeterWidth; ++x) {
        const float t = (float)x / (float)(kVuMeterWidth - 1);
        const uint16_t color = t < 0.72f ? dimGreen
                               : t < 0.88f ? dimYellow
                                            : dimRed;
        for (int y = 0; y < kVuMeterHeight; ++y) {
            plotVuPixel(kVuMeterX + x, kVuMeterY + y, color);
        }
    }

    for (int tick = 0; tick <= 4; ++tick) {
        const int x = kVuMeterX + (tick * (kVuMeterWidth - 1)) / 4;
        fillVuRect(x, kVuTickTop, 1, kVuTickHeight, border);
    }

    vuBaseReady = true;
    return true;
}

int vuFillWidthForLevel(float level, bool ready) {
    if (!ready) {
        return 0;
    }
    int fillWidth = (int)(level * (float)kVuMeterWidth + 0.5f);
    if (fillWidth < 0) {
        fillWidth = 0;
    }
    if (fillWidth > kVuMeterWidth) {
        fillWidth = kVuMeterWidth;
    }
    return fillWidth;
}

int vuPeakXForLevel(float peak, bool ready) {
    if (!ready) {
        return kVuMeterX;
    }
    int peakX = kVuMeterX + (int)(peak * (float)(kVuMeterWidth - 1) + 0.5f);
    if (peakX < kVuMeterX) {
        peakX = kVuMeterX;
    }
    if (peakX >= kVuMeterX + kVuMeterWidth) {
        peakX = kVuMeterX + kVuMeterWidth - 1;
    }
    return peakX;
}

PixelRect vuDynamicLevelRect(int fillWidth, int peakX, bool ready) {
    PixelRect rect = emptyRect();
    if (ready && fillWidth > 0) {
        rect = unionRect(rect, {kVuMeterX, kVuMeterY, fillWidth,
                                kVuMeterHeight});
    }
    if (ready) {
        rect = unionRect(rect, {peakX, kVuMeterY - 3, kVuPeakMarkerWidth,
                                kVuMeterHeight + 6});
    }
    return expandRect(rect, 1, kVuDisplayWidth, kVuDisplayHeight);
}

PixelRect vuClipRect() {
    return expandRect(rectFromBounds(kVuClipCenterX - kVuClipRadius,
                                     kVuClipCenterY - kVuClipRadius,
                                     kVuClipCenterX + kVuClipRadius,
                                     kVuClipCenterY + kVuClipRadius),
                      2, kVuDisplayWidth, kVuDisplayHeight);
}

void drawVuDynamicLevel(int fillWidth, int peakX, bool ready) {
    if (ready) {
        for (int x = 0; x < fillWidth; ++x) {
            const float t = (float)x / (float)(kVuMeterWidth - 1);
            const uint16_t color = vuColorForLevel(t);
            for (int y = 0; y < kVuMeterHeight; ++y) {
                plotVuPixel(kVuMeterX + x, kVuMeterY + y, color);
            }
        }

        fillVuRect(peakX, kVuMeterY - 3, kVuPeakMarkerWidth,
                   kVuMeterHeight + 6,
                   rgb565(230, 245, 245));
    }
}

void drawVuClipLamp(bool clip) {
    const uint16_t clipColor = clip ? rgb565(255, 24, 18)
                                    : rgb565(48, 0, 0);
    fillVuCircle(kVuClipCenterX, kVuClipCenterY, kVuClipRadius, clipColor);
    fillVuCircle(kVuClipCenterX + 2, kVuClipCenterY - 2, 2,
                 clip ? rgb565(255, 180, 160) : rgb565(70, 8, 8));
}

void pushVuDirtyRect(const PixelRect &rect) {
    const PixelRect pushRect =
        alignRectForPushColors(rect, kVuDisplayX, kVuDisplayWidth,
                               kVuDisplayHeight);
    if (!rectValid(pushRect)) {
        return;
    }
    amoled.pushColors(kVuDisplayX + pushRect.x, kVuDisplayY + pushRect.y,
                      pushRect.width, pushRect.height, graphPixels);
}

void renderVuMeter() {
    if (spectrogramMutex == nullptr) {
        return;
    }

    float level = 0.0f;
    float peak = 0.0f;
    bool clip = false;
    bool ready = false;

    xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
    level = vuLevel;
    peak = vuPeakLevel;
    clip = vuClipActive;
    ready = vuReady;
    xSemaphoreGive(spectrogramMutex);

    if (!vuBaseReady && !buildVuBase()) {
        return;
    }

    const int fillWidth = vuFillWidthForLevel(level, ready);
    const int peakX = vuPeakXForLevel(peak, ready);

    if (!vuScreenReady) {
        const PixelRect full = {0, 0, kVuDisplayWidth, kVuDisplayHeight};
        copyBaseRectToGraph(vuBasePixels, kVuDisplayWidth, full);
        setVuDrawTarget(graphPixels, full.width, full.height, full.x, full.y);
        drawVuDynamicLevel(fillWidth, peakX, ready);
        drawVuClipLamp(clip);
        pushVuDirtyRect(full);
    } else {
        const PixelRect oldLevelRect =
            vuDynamicLevelRect(vuLastFillWidth, vuLastPeakX, vuLastReady);
        const PixelRect newLevelRect =
            vuDynamicLevelRect(fillWidth, peakX, ready);
        const PixelRect levelRect = unionRect(oldLevelRect, newLevelRect);
        if (rectValid(levelRect)) {
            const PixelRect levelPushRect =
                alignRectForPushColors(levelRect, kVuDisplayX,
                                       kVuDisplayWidth, kVuDisplayHeight);
            copyBaseRectToGraph(vuBasePixels, kVuDisplayWidth,
                                levelPushRect);
            setVuDrawTarget(graphPixels, levelPushRect.width,
                            levelPushRect.height, levelPushRect.x,
                            levelPushRect.y);
            drawVuDynamicLevel(fillWidth, peakX, ready);
            pushVuDirtyRect(levelPushRect);
        }

        if (clip != vuLastClipActive) {
            const PixelRect clipDirty =
                alignRectForPushColors(vuClipRect(), kVuDisplayX,
                                       kVuDisplayWidth, kVuDisplayHeight);
            copyBaseRectToGraph(vuBasePixels, kVuDisplayWidth, clipDirty);
            setVuDrawTarget(graphPixels, clipDirty.width, clipDirty.height,
                            clipDirty.x, clipDirty.y);
            drawVuClipLamp(clip);
            pushVuDirtyRect(clipDirty);
        }
    }

    vuLastFillWidth = fillWidth;
    vuLastPeakX = peakX;
    vuLastClipActive = clip;
    vuLastReady = ready;
    vuScreenReady = true;
}

constexpr int kAnalogVuCenterX = kAnalogVuDisplayWidth / 2;
constexpr int kAnalogVuCenterY = 116;
constexpr int kAnalogVuArcRadius = 86;
constexpr int kAnalogVuTickOuter = 83;
constexpr int kAnalogVuTickInner = 68;
constexpr int kAnalogVuStartDeg = 205;
constexpr int kAnalogVuEndDeg = 335;
constexpr int kAnalogVuSweepDeg = kAnalogVuEndDeg - kAnalogVuStartDeg;
constexpr int kAnalogVuNeedleLength = 70;
constexpr int kAnalogVuPivotRadius = 6;

void setAnalogVuDrawTarget(uint16_t *pixels, int width, int height,
                           int originX, int originY) {
    analogVuDrawPixels = pixels;
    analogVuDrawWidth = width;
    analogVuDrawHeight = height;
    analogVuDrawOriginX = originX;
    analogVuDrawOriginY = originY;
}

void plotAnalogVuPixel(int x, int y, uint16_t color) {
    uint16_t *target =
        analogVuDrawPixels != nullptr ? analogVuDrawPixels : graphPixels;
    const int localX = x - analogVuDrawOriginX;
    const int localY = y - analogVuDrawOriginY;
    if (localX < 0 || localX >= analogVuDrawWidth ||
        localY < 0 || localY >= analogVuDrawHeight) {
        return;
    }
    target[localY * analogVuDrawWidth + localX] = color;
}

void drawAnalogVuLine(int x0, int y0, int x1, int y1, uint16_t color) {
    const int dx = abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        plotAnalogVuPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void fillAnalogVuCircle(int centerX, int centerY, int radius, uint16_t color) {
    const int radiusSquared = radius * radius;
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            if (x * x + y * y <= radiusSquared) {
                plotAnalogVuPixel(centerX + x, centerY + y, color);
            }
        }
    }
}

void plotAnalogVuThickPixel(int x, int y, int halfThickness,
                            uint16_t color) {
    for (int yy = y - halfThickness; yy <= y + halfThickness; ++yy) {
        for (int xx = x - halfThickness; xx <= x + halfThickness; ++xx) {
            plotAnalogVuPixel(xx, yy, color);
        }
    }
}

void drawAnalogVuThickLine(int x0, int y0, int x1, int y1,
                           int halfThickness, uint16_t color) {
    drawAnalogVuLine(x0, y0, x1, y1, color);
    for (int offset = 1; offset <= halfThickness; ++offset) {
        drawAnalogVuLine(x0 - offset, y0, x1 - offset, y1, color);
        drawAnalogVuLine(x0 + offset, y0, x1 + offset, y1, color);
        drawAnalogVuLine(x0, y0 - offset, x1, y1 - offset, color);
        drawAnalogVuLine(x0, y0 + offset, x1, y1 + offset, color);
    }
}

uint16_t blendRgb565(uint16_t fromColor, uint16_t toColor, uint8_t amount) {
    const uint16_t fromNative =
        (uint16_t)((fromColor >> 8) | (fromColor << 8));
    const uint16_t toNative = (uint16_t)((toColor >> 8) | (toColor << 8));

    const uint8_t fr = fromNative >> 11;
    const uint8_t fg = (fromNative >> 5) & 0x3F;
    const uint8_t fb = fromNative & 0x1F;
    const uint8_t tr = toNative >> 11;
    const uint8_t tg = (toNative >> 5) & 0x3F;
    const uint8_t tb = toNative & 0x1F;

    const uint8_t r = fr + (((int)tr - (int)fr) * amount) / 255;
    const uint8_t g = fg + (((int)tg - (int)fg) * amount) / 255;
    const uint8_t b = fb + (((int)tb - (int)fb) * amount) / 255;
    const uint16_t native =
        ((uint16_t)r << 11) | ((uint16_t)g << 5) | (uint16_t)b;
    return (uint16_t)((native >> 8) | (native << 8));
}

uint16_t analogVuBackgroundPixel(const AnalogVuTheme &theme, int x, int y) {
    const bool stripe = (y % 12) == 0;
    uint16_t color = stripe ? theme.faceStripe : theme.face;

    if (theme.glowRadius > 0 && theme.glowStrength > 0) {
        const int dx = x - theme.glowCenterX;
        const int dy = y - theme.glowCenterY;
        const int distanceSquared = dx * dx + dy * dy;
        const int radiusSquared = theme.glowRadius * theme.glowRadius;
        if (distanceSquared < radiusSquared) {
            const int glow =
                (theme.glowStrength * (radiusSquared - distanceSquared)) /
                radiusSquared;
            color = blendRgb565(color, theme.faceGlow,
                                (uint8_t)min(glow, 255));
        }
    }

    return color;
}

void drawAnalogVuLabelAt(const AnalogVuTheme &theme, int x, int y,
                         uint16_t color) {
    const int scale = theme.labelScale;
    const int thick = theme.labelHalfThickness;
    const int letterWidth = 6 * scale;
    const int letterHeight = 8 * scale;
    const int gap = 3 * scale;
    const int vMidX = x + letterWidth / 2;
    const int bottomY = y + letterHeight;

    drawAnalogVuThickLine(x, y, vMidX, bottomY, thick, color);
    drawAnalogVuThickLine(x + letterWidth, y, vMidX, bottomY, thick, color);

    const int uX = x + letterWidth + gap;
    const int uRight = uX + letterWidth;
    drawAnalogVuThickLine(uX, y, uX, y + letterHeight - scale, thick,
                          color);
    drawAnalogVuThickLine(uRight, y, uRight, y + letterHeight - scale,
                          thick, color);
    drawAnalogVuThickLine(uX, bottomY, uRight, bottomY, thick, color);
}

void drawAnalogVuLabel(const AnalogVuTheme &theme) {
    drawAnalogVuLabelAt(theme, theme.labelX + theme.labelShadowOffsetX,
                        theme.labelY + theme.labelShadowOffsetY,
                        theme.labelShadow);
    drawAnalogVuLabelAt(theme, theme.labelX, theme.labelY, theme.label);
}

void drawAnalogVuArc(int centerX, int centerY, int radius, int startDeg,
                     int endDeg, int halfThickness, uint16_t color) {
    for (int deg = startDeg; deg <= endDeg; ++deg) {
        const float rad = (float)deg * PI / 180.0f;
        const int x = centerX + (int)(cosf(rad) * (float)radius + 0.5f);
        const int y = centerY + (int)(sinf(rad) * (float)radius + 0.5f);
        plotAnalogVuThickPixel(x, y, halfThickness, color);
    }
}

void drawAnalogVuOuterArc(int centerX, int centerY, int radius, int startDeg,
                          int endDeg, int outwardPixels, int halfThickness,
                          uint16_t color) {
    for (int offset = 0; offset <= outwardPixels; ++offset) {
        drawAnalogVuArc(centerX, centerY, radius + offset, startDeg, endDeg,
                        halfThickness, color);
    }
}

bool buildAnalogVuBase() {
    if (!initVuRenderBuffers()) {
        return false;
    }

    const AnalogVuTheme &theme = activeAnalogVuTheme;
    setAnalogVuDrawTarget(analogVuBasePixels, kAnalogVuDisplayWidth,
                          kAnalogVuDisplayHeight, 0, 0);
    for (int y = 0; y < kAnalogVuDisplayHeight; ++y) {
        for (int x = 0; x < kAnalogVuDisplayWidth; ++x) {
            analogVuBasePixels[y * kAnalogVuDisplayWidth + x] =
                analogVuBackgroundPixel(theme, x, y);
        }
    }

    for (int x = 0; x < kAnalogVuDisplayWidth; ++x) {
        plotAnalogVuPixel(x, 0, theme.border);
        plotAnalogVuPixel(x, kAnalogVuDisplayHeight - 1, theme.border);
    }
    for (int y = 0; y < kAnalogVuDisplayHeight; ++y) {
        plotAnalogVuPixel(0, y, theme.border);
        plotAnalogVuPixel(kAnalogVuDisplayWidth - 1, y, theme.border);
    }

    drawAnalogVuOuterArc(kAnalogVuCenterX, kAnalogVuCenterY,
                         kAnalogVuArcRadius, kAnalogVuStartDeg, 278,
                         theme.outerArcOutwardPixels,
                         theme.arcPointHalfThickness, theme.greenArc);
    drawAnalogVuOuterArc(kAnalogVuCenterX, kAnalogVuCenterY,
                         kAnalogVuArcRadius, 279, 316,
                         theme.outerArcOutwardPixels,
                         theme.arcPointHalfThickness, theme.amberArc);
    drawAnalogVuOuterArc(kAnalogVuCenterX, kAnalogVuCenterY,
                         kAnalogVuArcRadius, 317, kAnalogVuEndDeg,
                         theme.outerArcOutwardPixels,
                         theme.arcPointHalfThickness, theme.redArc);

    for (int tick = 0; tick <= 10; ++tick) {
        const int deg = kAnalogVuStartDeg + (tick * kAnalogVuSweepDeg) / 10;
        const float rad = (float)deg * PI / 180.0f;
        const int inner =
            (tick % 5 == 0) ? kAnalogVuTickInner - 4 : kAnalogVuTickInner;
        const int x0 =
            kAnalogVuCenterX + (int)(cosf(rad) * (float)inner + 0.5f);
        const int y0 =
            kAnalogVuCenterY + (int)(sinf(rad) * (float)inner + 0.5f);
        const int x1 = kAnalogVuCenterX +
                       (int)(cosf(rad) * (float)kAnalogVuTickOuter + 0.5f);
        const int y1 = kAnalogVuCenterY +
                       (int)(sinf(rad) * (float)kAnalogVuTickOuter + 0.5f);
        const int halfThickness = tick % 5 == 0
                                      ? theme.majorTickHalfThickness
                                      : theme.minorTickHalfThickness;
        drawAnalogVuThickLine(x0, y0, x1, y1, halfThickness, theme.tick);
    }

    drawAnalogVuArc(kAnalogVuCenterX, kAnalogVuCenterY, 54,
                    kAnalogVuStartDeg, kAnalogVuEndDeg,
                    theme.arcPointHalfThickness, theme.innerArc);
    drawAnalogVuLabel(theme);

    analogVuBaseReady = true;
    return true;
}

int analogVuNeedleDegForLevel(float level, bool ready) {
    float shownLevel = ready ? level : 0.0f;
    if (shownLevel < 0.0f) {
        shownLevel = 0.0f;
    }
    if (shownLevel > 1.0f) {
        shownLevel = 1.0f;
    }
    return kAnalogVuStartDeg +
           (int)(shownLevel * (float)kAnalogVuSweepDeg + 0.5f);
}

void analogVuNeedleEndpoint(int needleDeg, int &x, int &y) {
    const float needleRad = (float)needleDeg * PI / 180.0f;
    x = kAnalogVuCenterX +
        (int)(cosf(needleRad) * (float)kAnalogVuNeedleLength + 0.5f);
    y = kAnalogVuCenterY +
        (int)(sinf(needleRad) * (float)kAnalogVuNeedleLength + 0.5f);
}

PixelRect analogVuNeedleRect(int needleDeg, const AnalogVuTheme &theme) {
    int needleX = 0;
    int needleY = 0;
    analogVuNeedleEndpoint(needleDeg, needleX, needleY);
    const int x0 = min(kAnalogVuCenterX, needleX);
    const int y0 = min(kAnalogVuCenterY, needleY);
    const int x1 = max(kAnalogVuCenterX, needleX);
    const int y1 = max(kAnalogVuCenterY, needleY);
    const int padding = max(kAnalogVuPivotRadius + 2,
                            theme.needleHalfThickness + 4);
    return expandRect(rectFromBounds(x0, y0, x1, y1), padding,
                      kAnalogVuDisplayWidth, kAnalogVuDisplayHeight);
}

PixelRect analogVuClipRect(const AnalogVuTheme &theme) {
    const PixelRect lamp =
        rectFromBounds(theme.clipX - theme.clipRadius,
                       theme.clipY - theme.clipRadius,
                       theme.clipX + theme.clipRadius,
                       theme.clipY + theme.clipRadius);
    const PixelRect highlight =
        rectFromBounds(theme.clipX + theme.clipHighlightOffsetX -
                           theme.clipHighlightRadius,
                       theme.clipY + theme.clipHighlightOffsetY -
                           theme.clipHighlightRadius,
                       theme.clipX + theme.clipHighlightOffsetX +
                           theme.clipHighlightRadius,
                       theme.clipY + theme.clipHighlightOffsetY +
                           theme.clipHighlightRadius);
    return expandRect(unionRect(lamp, highlight), 2,
                      kAnalogVuDisplayWidth, kAnalogVuDisplayHeight);
}

void drawAnalogVuNeedleAndPivot(const AnalogVuTheme &theme, int needleDeg) {
    int needleX = 0;
    int needleY = 0;
    analogVuNeedleEndpoint(needleDeg, needleX, needleY);
    drawAnalogVuThickLine(kAnalogVuCenterX, kAnalogVuCenterY, needleX,
                          needleY,
                          theme.needleHalfThickness, theme.needle);

    fillAnalogVuCircle(kAnalogVuCenterX, kAnalogVuCenterY,
                       kAnalogVuPivotRadius, theme.pivotOuter);
    fillAnalogVuCircle(kAnalogVuCenterX, kAnalogVuCenterY, 3,
                       theme.pivotInner);
}

void drawAnalogVuClipLamp(const AnalogVuTheme &theme, bool clip) {
    const uint16_t clipColor = clip ? theme.clipOn : theme.clipOff;
    fillAnalogVuCircle(theme.clipX, theme.clipY, theme.clipRadius,
                       clipColor);
    fillAnalogVuCircle(theme.clipX + theme.clipHighlightOffsetX,
                       theme.clipY + theme.clipHighlightOffsetY,
                       theme.clipHighlightRadius,
                       clip ? theme.clipHighlightOn
                            : theme.clipHighlightOff);
}

void pushAnalogVuDirtyRect(const PixelRect &rect) {
    const PixelRect pushRect =
        alignRectForPushColors(rect, kAnalogVuDisplayX,
                               kAnalogVuDisplayWidth,
                               kAnalogVuDisplayHeight);
    if (!rectValid(pushRect)) {
        return;
    }
    amoled.pushColors(kAnalogVuDisplayX + pushRect.x,
                      kAnalogVuDisplayY + pushRect.y,
                      pushRect.width, pushRect.height, graphPixels);
}

void renderAnalogVuMeter() {
    if (spectrogramMutex == nullptr) {
        return;
    }

    float level = 0.0f;
    bool clip = false;
    bool ready = false;

    xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
    level = vuLevel;
    clip = vuClipActive;
    ready = vuReady;
    xSemaphoreGive(spectrogramMutex);

    if (!analogVuBaseReady && !buildAnalogVuBase()) {
        return;
    }

    const AnalogVuTheme &theme = activeAnalogVuTheme;
    const int needleDeg = analogVuNeedleDegForLevel(level, ready);

    if (!analogVuScreenReady) {
        const PixelRect full = {0, 0, kAnalogVuDisplayWidth,
                                kAnalogVuDisplayHeight};
        copyBaseRectToGraph(analogVuBasePixels, kAnalogVuDisplayWidth, full);
        setAnalogVuDrawTarget(graphPixels, full.width, full.height,
                              full.x, full.y);
        drawAnalogVuNeedleAndPivot(theme, needleDeg);
        drawAnalogVuClipLamp(theme, clip);
        pushAnalogVuDirtyRect(full);
    } else {
        if (needleDeg != analogVuLastNeedleDeg ||
            ready != analogVuLastReady) {
            const PixelRect oldNeedle =
                analogVuNeedleRect(analogVuLastNeedleDeg, theme);
            const PixelRect newNeedle =
                analogVuNeedleRect(needleDeg, theme);
            const PixelRect needleDirty =
                alignRectForPushColors(unionRect(oldNeedle, newNeedle),
                                       kAnalogVuDisplayX,
                                       kAnalogVuDisplayWidth,
                                       kAnalogVuDisplayHeight);
            copyBaseRectToGraph(analogVuBasePixels, kAnalogVuDisplayWidth,
                                needleDirty);
            setAnalogVuDrawTarget(graphPixels, needleDirty.width,
                                  needleDirty.height, needleDirty.x,
                                  needleDirty.y);
            drawAnalogVuNeedleAndPivot(theme, needleDeg);
            pushAnalogVuDirtyRect(needleDirty);
        }

        if (clip != analogVuLastClipActive) {
            const PixelRect clipDirty =
                alignRectForPushColors(analogVuClipRect(theme),
                                       kAnalogVuDisplayX,
                                       kAnalogVuDisplayWidth,
                                       kAnalogVuDisplayHeight);
            copyBaseRectToGraph(analogVuBasePixels, kAnalogVuDisplayWidth,
                                clipDirty);
            setAnalogVuDrawTarget(graphPixels, clipDirty.width,
                                  clipDirty.height, clipDirty.x,
                                  clipDirty.y);
            drawAnalogVuClipLamp(theme, clip);
            pushAnalogVuDirtyRect(clipDirty);
        }
    }

    analogVuLastNeedleDeg = needleDeg;
    analogVuLastClipActive = clip;
    analogVuLastReady = ready;
    analogVuScreenReady = true;
}

uint8_t scopeYForValue(float value, float scale) {
    if (scale < 1.0f) {
        scale = 1.0f;
    }

    const float halfHeight = (float)(kScopeDisplayHeight - 1) * 0.5f;
    const float usableHalfHeight = halfHeight - 3.0f;
    int y = (int)(halfHeight - (value / scale) * usableHalfHeight);
    if (y < 0) {
        return 0;
    }
    if (y >= kScopeDisplayHeight) {
        return kScopeDisplayHeight - 1;
    }
    return (uint8_t)y;
}

void plotScopePixel(int x, int y, uint16_t color) {
    if (x < 0 || x >= kScopeDisplayWidth ||
        y < 0 || y >= kScopeDisplayHeight) {
        return;
    }
    graphPixels[y * kScopeDisplayWidth + x] = color;
}

void drawScopeLine(int x0, int y0, int x1, int y1, uint16_t color) {
    const int dx = abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        plotScopePixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void renderOscilloscope() {
    if (spectrogramMutex == nullptr) {
        return;
    }

    uint8_t traceTop[kScopeDisplayWidth];
    uint8_t traceBottom[kScopeDisplayWidth];
    bool traceReady = false;

    xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
    memcpy(traceTop, scopeTraceTop, sizeof(traceTop));
    memcpy(traceBottom, scopeTraceBottom, sizeof(traceBottom));
    traceReady = scopeTraceReady;
    xSemaphoreGive(spectrogramMutex);

    const int centerY = kScopeDisplayHeight / 2;
    for (int y = 0; y < kScopeDisplayHeight; ++y) {
        for (int x = 0; x < kScopeDisplayWidth; ++x) {
            const bool axis = y == centerY;
            const bool grid = (x % 50 == 0) || (y % 33 == 0);
            graphPixels[y * kScopeDisplayWidth + x] =
                axis ? kColorScopeAxis : grid ? kColorScopeGrid : kColorBlack;
        }
    }

    if (traceReady) {
        int lastMidY = centerY;
        for (int x = 0; x < kScopeDisplayWidth; ++x) {
            int top = traceTop[x];
            int bottom = traceBottom[x];
            if (top > bottom) {
                const int swap = top;
                top = bottom;
                bottom = swap;
            }

            for (int y = top; y <= bottom; ++y) {
                plotScopePixel(x, y, kColorScopeTrace);
            }

            const int midY = (top + bottom) / 2;
            if (x > 0) {
                drawScopeLine(x - 1, lastMidY, x, midY, kColorScopeTrace);
            }
            lastMidY = midY;
        }
    }

    amoled.pushColors(kScopeDisplayX, kScopeDisplayY, kScopeDisplayWidth,
                      kScopeDisplayHeight, graphPixels);
}

bool initI2SMicrophone() {
    i2s_config_t i2sConfig = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = kSampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = kI2SReadFrames,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
    };

    i2s_pin_config_t pinConfig = {};
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0)
    pinConfig.mck_io_num = I2S_PIN_NO_CHANGE;
#endif
    pinConfig.bck_io_num = kI2SBclk_GPIO;
    pinConfig.ws_io_num = kI2SLrclk_GPIO;
    pinConfig.data_out_num = I2S_PIN_NO_CHANGE;
    pinConfig.data_in_num = kI2SData_GPIO;

    esp_err_t err = i2s_driver_install(kI2SPort, &i2sConfig, 0, nullptr);
    if (err != ESP_OK) {
        Serial.printf("i2s_driver_install failed: %s\n", esp_err_to_name(err));
        return false;
    }

    err = i2s_set_pin(kI2SPort, &pinConfig);
    if (err != ESP_OK) {
        Serial.printf("i2s_set_pin failed: %s\n", esp_err_to_name(err));
        return false;
    }

    err = i2s_set_clk(kI2SPort, kSampleRate, I2S_BITS_PER_SAMPLE_32BIT,
                      I2S_CHANNEL_STEREO);
    if (err != ESP_OK) {
        Serial.printf("i2s_set_clk failed: %s\n", esp_err_to_name(err));
        return false;
    }

    i2s_zero_dma_buffer(kI2SPort);
    return true;
}

void buildHannWindow() {
    for (int i = 0; i < kFftSize; ++i) {
        hannWindow[i] = 0.5f - 0.5f * cosf((2.0f * PI * i) / (kFftSize - 1));
    }
}

bool readFrameForFft() {
    int sampleIndex = 0;
    int64_t sum = 0;
    int16_t frameMin = INT16_MAX;
    int16_t frameMax = INT16_MIN;
    uint16_t clipped = 0;

    while (sampleIndex < kFftSize) {
        size_t bytesRead = 0;
        const esp_err_t err =
            i2s_read(kI2SPort, i2sRaw, sizeof(i2sRaw), &bytesRead,
                     pdMS_TO_TICKS(250));

        if (err != ESP_OK) {
            Serial.printf("i2s_read failed: %s\n", esp_err_to_name(err));
            return false;
        }

        const int wordsRead = bytesRead / sizeof(i2sRaw[0]);
        const int framesRead = wordsRead / kI2SWordsPerFrame;
        if (framesRead == 0) {
            return false;
        }

        for (int i = 0; i < framesRead && sampleIndex < kFftSize; ++i) {
            const int base = i * kI2SWordsPerFrame;
            int32_t sample = i2sRaw[base + kMicSlot] >> kI2SSampleShift;
            if (sample > INT16_MAX) {
                sample = INT16_MAX;
                ++clipped;
            } else if (sample < INT16_MIN) {
                sample = INT16_MIN;
                ++clipped;
            }

            const int16_t pcm = (int16_t)sample;
            if (pcm < frameMin) {
                frameMin = pcm;
            }
            if (pcm > frameMax) {
                frameMax = pcm;
            }

            pcmFrame[sampleIndex++] = pcm;
            sum += sample;
        }
    }

    const float mean = (float)sum / (float)kFftSize;
    float sumSquares = 0.0f;
    for (int i = 0; i < kFftSize; ++i) {
        const float centered = (float)pcmFrame[i] - mean;
        fftInput[i] = centered * hannWindow[i];
        sumSquares += centered * centered;
    }

    lastFrameMean = mean;
    lastFrameRms = sqrtf(sumSquares / (float)kFftSize);
    lastFrameMin = frameMin;
    lastFrameMax = frameMax;
    lastFrameClipped = clipped;

    return true;
}

void requestDisplayNoiseCalibration(const char *reason) {
    displayNoiseCalibrated = false;
    displayNoiseCalibrationFrames = 0;
    displayNoiseCalibrationStartUs = 0;
    displayNoiseCalibrationSumDb = 0.0f;
    displayNoiseCalibrationMinDb = 0.0f;
    noiseFloorDb = lastFrameAvgDb > 1.0f ? lastFrameAvgDb : 70.0f;

    lockSerial();
    Serial.printf("display noise calibration requested: %s\n",
                  reason != nullptr ? reason : "manual");
    unlockSerial();
    notifyDisplay();
}

void updateDisplayNoiseCalibration(float frameAvg) {
    if (displayNoiseCalibrated) {
        return;
    }

    const int64_t nowUs = esp_timer_get_time();
    if (displayNoiseCalibrationFrames == 0) {
        displayNoiseCalibrationStartUs = nowUs;
        displayNoiseCalibrationSumDb = 0.0f;
        displayNoiseCalibrationMinDb = frameAvg;
    }

    if (frameAvg < displayNoiseCalibrationMinDb) {
        displayNoiseCalibrationMinDb = frameAvg;
    }
    displayNoiseCalibrationSumDb += frameAvg;
    ++displayNoiseCalibrationFrames;

    noiseFloorDb =
        displayNoiseCalibrationSumDb / (float)displayNoiseCalibrationFrames;

    const int64_t calibrationUs =
        (int64_t)kDisplayNoiseCalibrationMs * 1000LL;
    if (nowUs - displayNoiseCalibrationStartUs >= calibrationUs) {
        displayNoiseCalibrated = true;
        lockSerial();
        Serial.printf("display noise calibrated: floor=%.1fdB min=%.1fdB "
                      "frames=%lu duration=%lums\n",
                      noiseFloorDb, displayNoiseCalibrationMinDb,
                      (unsigned long)displayNoiseCalibrationFrames,
                      (unsigned long)kDisplayNoiseCalibrationMs);
        unlockSerial();
        notifyDisplay();
    }
}

void publishFftColumn() {
    float binDb[kDisplayHighBin + 1];
    float framePeak = 0.0f;
    float frameSum = 0.0f;
    const int displayBinCount = kDisplayHighBin - kDisplayLowBin + 1;

    binDb[0] = 0.0f;
    for (int bin = kDisplayLowBin; bin <= kDisplayHighBin; ++bin) {
        const float re = fftOutput[2 * bin];
        const float im = fftOutput[2 * bin + 1];
        const float mag = sqrtf(re * re + im * im);
        const float db = 20.0f * log10f(mag + 1.0f);
        binDb[bin] = db;
        if (db > framePeak) {
            framePeak = db;
        }
        frameSum += db;
    }

    const float frameAvg = frameSum / (float)displayBinCount;
    updateDisplayNoiseCalibration(frameAvg);

    lastFramePeakDb = framePeak;
    lastFrameAvgDb = frameAvg;

    const float floorDb = noiseFloorDb + kNoiseMarginDb;
    const float ceilingDb = floorDb + kDisplayRangeDb;
    uint8_t column[kFftDisplayHeight];

    for (int row = 0; row < kFftDisplayHeight; ++row) {
        const int rowFromBottom = kFftDisplayHeight - 1 - row;
        int binLo = kDisplayLowBin +
                    (rowFromBottom * (kDisplayHighBin - kDisplayLowBin + 1)) /
                        kFftDisplayHeight;
        int binHi = kDisplayLowBin +
                    ((rowFromBottom + 1) *
                     (kDisplayHighBin - kDisplayLowBin + 1)) /
                        kFftDisplayHeight -
                    1;
        if (binHi < binLo) {
            binHi = binLo;
        }
        if (binHi > kDisplayHighBin) {
            binHi = kDisplayHighBin;
        }

        float db = binDb[binLo];
        for (int bin = binLo + 1; bin <= binHi; ++bin) {
            if (binDb[bin] > db) {
                db = binDb[bin];
            }
        }

        float norm = (db - floorDb) / (ceilingDb - floorDb);
        if (norm < 0.0f) {
            norm = 0.0f;
        } else if (norm > 1.0f) {
            norm = 1.0f;
        }
        norm *= norm;
        column[row] = clampToByte(norm * 255.0f);
    }

    xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
    memcpy(spectrogram[spectrogramWriteColumn], column, sizeof(column));
    spectrogramWriteColumn = (spectrogramWriteColumn + 1) % kFftDisplayWidth;
    xSemaphoreGive(spectrogramMutex);
}

void publishAnalyzerFrame() {
    if (spectrogramMutex == nullptr) {
        return;
    }

    constexpr float binHz = (float)kSampleRate / (float)kFftSize;
    constexpr int maxFftBin = kFftSize / 2 - 1;
    constexpr int maxBarHeight = kAnalyzerDisplayHeight - 8;
    const float logStep =
        logf(kAnalyzerHighHz / kAnalyzerLowHz) / (float)kAnalyzerBandCount;
    const float floorDb = noiseFloorDb + kAnalyzerNoiseMarginDb;
    uint8_t heights[kAnalyzerBandCount];

    for (int band = 0; band < kAnalyzerBandCount; ++band) {
        const float freqLo = kAnalyzerLowHz * expf(logStep * band);
        const float freqHi = kAnalyzerLowHz * expf(logStep * (band + 1));
        int binLo = (int)ceilf(freqLo / binHz);
        int binHi = (int)floorf(freqHi / binHz);

        if (binLo < 1) {
            binLo = 1;
        }
        if (binHi > maxFftBin) {
            binHi = maxFftBin;
        }
        if (binHi < binLo) {
            binHi = binLo;
        }

        float peakDb = 0.0f;
        for (int bin = binLo; bin <= binHi; ++bin) {
            const float re = fftOutput[2 * bin];
            const float im = fftOutput[2 * bin + 1];
            const float mag = sqrtf(re * re + im * im);
            const float db = 20.0f * log10f(mag + 1.0f);
            if (db > peakDb) {
                peakDb = db;
            }
        }

        float target = (peakDb - floorDb) / kAnalyzerRangeDb;
        if (target < 0.0f) {
            target = 0.0f;
        } else if (target > 1.0f) {
            target = 1.0f;
        }

        analyzerLevels[band] =
            analyzerLevels[band] * kAnalyzerSmoothing +
            target * (1.0f - kAnalyzerSmoothing);
        heights[band] =
            (uint8_t)(analyzerLevels[band] * (float)maxBarHeight + 0.5f);
    }

    xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
    memcpy(analyzerBarHeights, heights, sizeof(analyzerBarHeights));
    analyzerReady = true;
    xSemaphoreGive(spectrogramMutex);
}

void publishVuFrame() {
    if (spectrogramMutex == nullptr) {
        return;
    }

    const float rms = lastFrameRms > 1.0f ? lastFrameRms : 1.0f;
    const float db = 20.0f * log10f(rms / 32768.0f);
    float target = (db - kVuMinDb) / (kVuMaxDb - kVuMinDb);
    if (target < 0.0f) {
        target = 0.0f;
    } else if (target > 1.0f) {
        target = 1.0f;
    }

    if (target > vuSmoothedLevel) {
        vuSmoothedLevel = vuSmoothedLevel * 0.35f + target * 0.65f;
    } else {
        vuSmoothedLevel = vuSmoothedLevel * 0.92f + target * 0.08f;
    }

    const bool clipped = lastFrameClipped > 0 ||
                         lastFrameMax >= kVuClipThreshold ||
                         lastFrameMin <= -kVuClipThreshold;
    if (clipped) {
        vuClipHold = kVuClipHoldFrames;
    } else if (vuClipHold > 0) {
        --vuClipHold;
    }

    xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
    vuLevel = vuSmoothedLevel;
    vuPeakLevel = target;
    vuClipActive = vuClipHold > 0;
    vuReady = true;
    xSemaphoreGive(spectrogramMutex);
}

void publishOscilloscopeFrame() {
    if (spectrogramMutex == nullptr) {
        return;
    }

    float peakAbs = 0.0f;
    for (int i = 0; i < kFftSize; ++i) {
        const float centered = (float)pcmFrame[i] - lastFrameMean;
        const float absSample = fabsf(centered);
        if (absSample > peakAbs) {
            peakAbs = absSample;
        }
    }

    float targetScale = peakAbs * 1.15f;
    if (targetScale < 512.0f) {
        targetScale = 512.0f;
    }
    if (targetScale > scopeScale) {
        scopeScale = scopeScale * 0.65f + targetScale * 0.35f;
    } else {
        scopeScale = scopeScale * 0.98f + targetScale * 0.02f;
    }

    uint8_t top[kScopeDisplayWidth];
    uint8_t bottom[kScopeDisplayWidth];

    for (int x = 0; x < kScopeDisplayWidth; ++x) {
        const int start = (x * kFftSize) / kScopeDisplayWidth;
        int end = ((x + 1) * kFftSize) / kScopeDisplayWidth;
        if (end <= start) {
            end = start + 1;
        }

        float minSample = 32767.0f;
        float maxSample = -32768.0f;
        for (int i = start; i < end && i < kFftSize; ++i) {
            const float centered = (float)pcmFrame[i] - lastFrameMean;
            if (centered < minSample) {
                minSample = centered;
            }
            if (centered > maxSample) {
                maxSample = centered;
            }
        }

        const uint8_t maxY = scopeYForValue(maxSample, scopeScale);
        const uint8_t minY = scopeYForValue(minSample, scopeScale);
        top[x] = maxY < minY ? maxY : minY;
        bottom[x] = maxY < minY ? minY : maxY;
    }

    xSemaphoreTake(spectrogramMutex, portMAX_DELAY);
    memcpy(scopeTraceTop, top, sizeof(scopeTraceTop));
    memcpy(scopeTraceBottom, bottom, sizeof(scopeTraceBottom));
    scopeTraceReady = true;
    xSemaphoreGive(spectrogramMutex);
}

bool writeWavHeader(File &file, uint32_t dataBytes, bool flushAfterWrite) {
    WavHeader header = {
        {'R', 'I', 'F', 'F'},
        36U + dataBytes,
        {'W', 'A', 'V', 'E'},
        {'f', 'm', 't', ' '},
        16,
        1,
        kWavChannels,
        kSampleRate,
        kWavByteRate,
        kWavBlockAlign,
        kWavBitsPerSample,
        {'d', 'a', 't', 'a'},
        dataBytes,
    };

    if (!file.seek(0)) {
        return false;
    }

    const size_t written = file.write((const uint8_t *)&header, sizeof(header));
    if (flushAfterWrite) {
        file.flush();
    }
    return written == sizeof(header);
}

bool findNextRecordingPath(char *path, size_t pathSize) {
    for (uint32_t index = 1; index < 100000; ++index) {
        snprintf(path, pathSize, "/%lu.wav", (unsigned long)index);
        if (!SD.exists(path)) {
            return true;
        }
    }
    return false;
}

void resetRecordQueues() {
    if (recordFreeQueue == nullptr || recordFilledQueue == nullptr) {
        return;
    }

    xQueueReset(recordFreeQueue);
    xQueueReset(recordFilledQueue);
    for (uint16_t index = 0; index < kRecordBufferFrameCount; ++index) {
        xQueueSend(recordFreeQueue, &index, 0);
    }
    sdQueueHighWater = 0;
}

bool initRecordBuffers() {
    recordFrames =
        (RecordFrame *)ps_malloc(sizeof(RecordFrame) * kRecordBufferFrameCount);
    if (recordFrames == nullptr) {
        recordFrames =
            (RecordFrame *)malloc(sizeof(RecordFrame) * kRecordBufferFrameCount);
    }

    if (recordFrames == nullptr) {
        Serial.println("record buffer allocation failed");
        startupScreenStatus("Recorder buffer failed",
                            "SD recording will not be available", 40);
        return false;
    }

    recordFreeQueue = xQueueCreate(kRecordBufferFrameCount, sizeof(uint16_t));
    recordFilledQueue = xQueueCreate(kRecordBufferFrameCount, sizeof(uint16_t));
    sdCommandQueue = xQueueCreate(4, sizeof(SdCommand));
    sdStateMutex = xSemaphoreCreateMutex();

    if (recordFreeQueue == nullptr || recordFilledQueue == nullptr ||
        sdCommandQueue == nullptr || sdStateMutex == nullptr) {
        Serial.println("record queue allocation failed");
        startupScreenStatus("Recorder queue failed",
                            "SD recording will not be available", 40);
        return false;
    }

    resetRecordQueues();
    Serial.printf("record buffer: frames=%u bytes=%u psram=%s\n",
                  (unsigned)kRecordBufferFrameCount,
                  (unsigned)(sizeof(RecordFrame) * kRecordBufferFrameCount),
                  psramFound() ? "yes" : "no");
    startupScreenStatus("Recorder buffer ready",
                        psramFound() ? "Using PSRAM for audio buffering"
                                     : "Using heap fallback for audio buffering",
                        42);
    return true;
}

void printDirectory(File dir, int depth) {
    for (;;) {
        File entry = dir.openNextFile();
        if (!entry) {
            break;
        }

        for (int i = 0; i < depth; ++i) {
            Serial.print("  ");
        }
        Serial.printf("SD %s %s size=%llu\n",
                      entry.isDirectory() ? "dir " : "file",
                      entry.name(),
                      (unsigned long long)entry.size());

        if (entry.isDirectory() && depth < 1) {
            printDirectory(entry, depth + 1);
        }
        entry.close();
    }
}

void printSdFileListAtBoot() {
    const uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("SD card not detected");
        startupScreenStatus("SD card not detected",
                            "Recording disabled; live capture will continue",
                            48);
        sdCardReady = false;
        setSdStatus(SdStatus::NotReady, "-");
        return;
    }

    sdCardReady = true;
    setSdStatus(SdStatus::Idle, "-");
    const unsigned long long cardSizeMb =
        (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL));
    const unsigned long long usedMb =
        (unsigned long long)(SD.usedBytes() / (1024ULL * 1024ULL));
    Serial.printf("SD card size=%llu MB used=%llu MB\n", cardSizeMb, usedMb);
    char detail[96];
    snprintf(detail, sizeof(detail), "%llu MB card, %llu MB used",
             cardSizeMb, usedMb);
    startupScreenStatus("SD card ready", detail, 50);

    File root = SD.open("/");
    if (!root) {
        Serial.println("SD root open failed");
        startupScreenStatus("SD root open failed",
                            "Recording disabled until SD is fixed", 50);
        ++sdErrors;
        setSdStatus(SdStatus::Error, "-");
        return;
    }

    Serial.println("SD file list:");
    printDirectory(root, 0);
    root.close();
}

void requestRecordingStart() {
    if (!sdCardReady || sdCommandQueue == nullptr) {
        Serial.println("record start ignored: SD not ready");
        return;
    }

    const SdCommand command = SdCommand::Start;
    if (xQueueSend(sdCommandQueue, &command, 0) == pdTRUE) {
        Serial.println("record start requested");
    } else {
        Serial.println("record start command queue full");
        ++sdErrors;
    }
}

void requestRecordingStop() {
    if (sdCommandQueue == nullptr) {
        return;
    }

    const SdCommand command = SdCommand::Stop;
    if (xQueueSend(sdCommandQueue, &command, 0) == pdTRUE) {
        Serial.println("record stop requested");
    } else {
        Serial.println("record stop command queue full");
        ++sdErrors;
    }
}

void switchToApp(esp_partition_subtype_t subtype) {
    const esp_partition_t *target = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, subtype, NULL);
    if (target == NULL) {
        Serial.println("OTA switch: target partition not found");
        return;
    }
    esp_app_desc_t desc;
    if (esp_ota_get_partition_description(target, &desc) != ESP_OK) {
        Serial.println("OTA switch: target partition empty or invalid");
        return;
    }
    Serial.printf("OTA switch: rebooting into %s\n", desc.project_name);
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
        Serial.println("OTA switch: esp_ota_set_boot_partition failed");
        return;
    }
    esp_restart();
}

void switchToOtherOtaPartition() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_subtype_t target =
        (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)
            ? ESP_PARTITION_SUBTYPE_APP_OTA_1
            : ESP_PARTITION_SUBTYPE_APP_OTA_0;
    switchToApp(target);
}

void toggleRecording() {
    char fileName[sizeof(sdCurrentFileName)];
    const SdStatus status = getSdStatusSnapshot(fileName, sizeof(fileName));
    if (status == SdStatus::Recording || status == SdStatus::Starting) {
        requestRecordingStop();
    } else if (status == SdStatus::Idle || status == SdStatus::Error) {
        requestRecordingStart();
    } else {
        Serial.printf("record toggle ignored: status=%s file=%s\n",
                      sdStatusName(status), fileName);
    }
}

void buttonTask(void *) {
    bool displayNoiseRecalibrationPending = false;
    uint32_t displayNoiseRecalibrationReleaseMs = 0;
    uint32_t topButtonPressedAtMs = 0;
    bool topButtonLongPressTriggered = false;

    for (;;) {
        const uint32_t now = millis();

        // Sample top button once; result used for both press and release checks.
        const int topEvent = topButton.update();

        // Record the moment the button went down so we can time the long press.
        if (topEvent == DebouncedButton::kPressed) {
            topButtonPressedAtMs = now;
            topButtonLongPressTriggered = false;
        }

        // Long press (3 s held): switch OTA partition and reboot.
        // Flag is set here so the subsequent release check knows not to also
        // trigger the short-press action.
        if (enableOtaPartitionSwitch &&
            topButton.stablePressed && !topButtonLongPressTriggered &&
            (uint32_t)(now - topButtonPressedAtMs) >= 3000) {
            topButtonLongPressTriggered = true;
            lockSerial();
            Serial.println("top button long press: switching OTA partition");
            unlockSerial();
            switchToOtherOtaPartition();
        }

        // Short press (released before long-press threshold): toggle SD recording.
        if (topEvent == DebouncedButton::kReleased) {
            if (!topButtonLongPressTriggered) {
                toggleRecording();
            }
            topButtonLongPressTriggered = false;
        }

        // Side button release: queue a display noise recalibration after a short
        // settle delay so any vibration from pressing the button dies down first.
        if (sideButton.update() == DebouncedButton::kReleased) {
            displayNoiseRecalibrationPending = true;
            displayNoiseRecalibrationReleaseMs = now;
            lockSerial();
            Serial.println("display noise recalibration queued");
            unlockSerial();
        }

        if (displayNoiseRecalibrationPending &&
            (uint32_t)(now - displayNoiseRecalibrationReleaseMs) >=
                kDisplayNoiseRecalibrationDelayMs) {
            displayNoiseRecalibrationPending = false;
            requestDisplayNoiseCalibration("side button");
        }

        vTaskDelay(pdMS_TO_TICKS(kLoopHousekeepingIntervalMs));
    }
}

void queueRecordingFrame() {
    if (!sdRecordingActive || recordFrames == nullptr ||
        recordFreeQueue == nullptr || recordFilledQueue == nullptr) {
        return;
    }

    uint16_t index = 0;
    if (xQueueReceive(recordFreeQueue, &index, 0) != pdTRUE) {
        ++sdQueueDrops;
        ++sdQueueDropsTotal;
        return;
    }

    memcpy(recordFrames[index].samples, pcmFrame, kRecordFrameBytes);
    if (xQueueSend(recordFilledQueue, &index, 0) != pdTRUE) {
        xQueueSend(recordFreeQueue, &index, 0);
        ++sdErrors;
        return;
    }

    const uint32_t queued = uxQueueMessagesWaiting(recordFilledQueue);
    if (queued > sdQueueHighWater) {
        sdQueueHighWater = queued;
    }
}

void resetSdWriteStatsForNewFile() {
    sdChunksWritten = 0;
    sdBytesWritten = 0;
    sdQueueDrops = 0;
    sdQueueHighWater = 0;
    sdTotalWriteUs = 0;
    sdMaxWriteUs = 0;
}

bool openRecordingFile(File &file, char *path, size_t pathSize) {
    if (!findNextRecordingPath(path, pathSize)) {
        Serial.println("record open failed: no available filename");
        ++sdErrors;
        return false;
    }

    file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.printf("record open failed: %s\n", path);
        ++sdErrors;
        return false;
    }

    if (!writeWavHeader(file, 0, true)) {
        Serial.printf("record header write failed: %s\n", path);
        file.close();
        SD.remove(path);
        ++sdErrors;
        return false;
    }

    resetSdWriteStatsForNewFile();
    setSdStatus(SdStatus::Recording, path);
    Serial.printf("recording started: %s\n", path);
    return true;
}

void closeRecordingFile(File &file, const char *path, uint32_t dataBytes,
                        SdStatus finalStatus = SdStatus::Idle) {
    setSdStatus(SdStatus::Stopping, path);
    if (!writeWavHeader(file, dataBytes, true)) {
        Serial.printf("record final header write failed: %s\n", path);
        ++sdErrors;
        finalStatus = SdStatus::Error;
    }
    file.flush();
    file.close();
    setSdStatus(finalStatus, path);
    Serial.printf("recording stopped: %s bytes=%lu chunks=%lu errors=%lu\n",
                  path, (unsigned long)dataBytes,
                  (unsigned long)sdChunksWritten,
                  (unsigned long)sdErrors);
}

void sdWriterTask(void *) {
    File recordingFile;
    char recordingPath[sizeof(sdCurrentFileName)] = "-";
    bool fileOpen = false;
    bool stopRequested = false;
    bool writeError = false;
    uint32_t dataBytes = 0;

    for (;;) {
        SdCommand command;
        const TickType_t waitTicks =
            fileOpen ? pdMS_TO_TICKS(10) : portMAX_DELAY;
        if (xQueueReceive(sdCommandQueue, &command, waitTicks) == pdTRUE) {
            if (command == SdCommand::Start && !fileOpen) {
                resetRecordQueues();
                setSdStatus(SdStatus::Starting, "-");
                stopRequested = false;
                writeError = false;
                dataBytes = 0;
                if (openRecordingFile(recordingFile, recordingPath,
                                      sizeof(recordingPath))) {
                    fileOpen = true;
                } else {
                    setSdStatus(SdStatus::Error, "-");
                }
            } else if (command == SdCommand::Stop && fileOpen) {
                stopRequested = true;
                sdRecordingActive = false;
                setSdStatus(SdStatus::Stopping, recordingPath);
            }
        }

        if (!fileOpen) {
            continue;
        }

        uint16_t index = 0;
        if (xQueueReceive(recordFilledQueue, &index, pdMS_TO_TICKS(5)) ==
            pdTRUE) {
            const int64_t writeStartUs = esp_timer_get_time();
            const size_t written = recordingFile.write(
                (const uint8_t *)recordFrames[index].samples, kRecordFrameBytes);
            const int64_t writeUs = esp_timer_get_time() - writeStartUs;

            sdTotalWriteUs += writeUs;
            if (writeUs > sdMaxWriteUs) {
                sdMaxWriteUs = writeUs;
            }

            if (written == kRecordFrameBytes) {
                ++sdChunksWritten;
                sdBytesWritten += written;
                dataBytes += written;
                if (sdChunksWritten % kSdFlushEveryChunks == 0) {
                    recordingFile.flush();
                }
            } else {
                ++sdErrors;
                Serial.printf("SD write short: wrote=%u expected=%u\n",
                              (unsigned)written, (unsigned)kRecordFrameBytes);
                stopRequested = true;
                writeError = true;
                sdRecordingActive = false;
                setSdStatus(SdStatus::Error, recordingPath);
            }

            xQueueSend(recordFreeQueue, &index, portMAX_DELAY);
        }

        if (stopRequested && uxQueueMessagesWaiting(recordFilledQueue) == 0) {
            closeRecordingFile(recordingFile, recordingPath, dataBytes,
                               writeError ? SdStatus::Error : SdStatus::Idle);
            fileOpen = false;
            stopRequested = false;
            writeError = false;
        }
    }
}

void captureTask(void *) {
    uint32_t frames = 0;
    int64_t reportStartUs = esp_timer_get_time();
    int64_t totalReadUs = 0;
    int64_t totalRecordQueueUs = 0;
    int64_t totalFftUs = 0;
    int64_t totalPublishFftUs = 0;
    int64_t totalPublishAnalyzerUs = 0;
    int64_t totalPublishVuUs = 0;
    int64_t totalPublishScopeUs = 0;
    int64_t totalActiveUs = 0;
    int64_t totalFrameUs = 0;

    for (;;) {
        if (!captureEnabled || !i2sReady || !fftReady) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        const int64_t t0 = esp_timer_get_time();
        if (!readFrameForFft()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        const int64_t t1 = esp_timer_get_time();
        queueRecordingFrame();
        const int64_t tRecord = esp_timer_get_time();
        fft_execute(fftPlan);
        const int64_t t2 = esp_timer_get_time();
        publishFftColumn();
        const int64_t t3 = esp_timer_get_time();
        publishAnalyzerFrame();
        const int64_t t4 = esp_timer_get_time();
        publishVuFrame();
        const int64_t tVu = esp_timer_get_time();
        publishOscilloscopeFrame();
        const int64_t t5 = esp_timer_get_time();

        notifyDisplay();

        totalReadUs += t1 - t0;
        totalRecordQueueUs += tRecord - t1;
        totalFftUs += t2 - tRecord;
        totalPublishFftUs += t3 - t2;
        totalPublishAnalyzerUs += t4 - t3;
        totalPublishVuUs += tVu - t4;
        totalPublishScopeUs += t5 - tVu;
        totalActiveUs += t5 - t1;
        totalFrameUs += t5 - t0;
        ++frames;

        if (frames % kDiagnosticsFrames == 0) {
            const int64_t reportWallUs = t5 - reportStartUs;
            char sdFileName[sizeof(sdCurrentFileName)];
            const SdStatus currentSdStatus =
                getSdStatusSnapshot(sdFileName, sizeof(sdFileName));
            const uint32_t queuedFrames = recordFilledQueue != nullptr
                                              ? uxQueueMessagesWaiting(recordFilledQueue)
                                              : 0;
            const uint32_t freeFrames = recordFreeQueue != nullptr
                                            ? uxQueueMessagesWaiting(recordFreeQueue)
                                            : 0;
            const int64_t sdAvgWriteUs =
                sdChunksWritten > 0
                    ? sdTotalWriteUs / (int64_t)sdChunksWritten
                    : 0;
            lockSerial();
            Serial.printf("capture avg: period=%lldus read(wait)=%lldus "
                          "rec_q=%lldus fft=%lldus pub_fft=%lldus pub_an=%lldus "
                          "pub_vu=%lldus pub_scope=%lldus active=%lldus duty=%.1f%% "
                          "slot=%d "
                          "pcm min=%d max=%d mean=%.1f rms=%.1f clip=%u "
                          "fft avg=%.1fdB peak=%.1fdB noise=%.1fdB "
                          "noise_cal=%u\n",
                          totalFrameUs / kDiagnosticsFrames,
                          totalReadUs / kDiagnosticsFrames,
                          totalRecordQueueUs / kDiagnosticsFrames,
                          totalFftUs / kDiagnosticsFrames,
                          totalPublishFftUs / kDiagnosticsFrames,
                          totalPublishAnalyzerUs / kDiagnosticsFrames,
                          totalPublishVuUs / kDiagnosticsFrames,
                          totalPublishScopeUs / kDiagnosticsFrames,
                          totalActiveUs / kDiagnosticsFrames,
                          percentOf(totalActiveUs, reportWallUs), kMicSlot,
                          lastFrameMin, lastFrameMax,
                          lastFrameMean, lastFrameRms, lastFrameClipped,
                          lastFrameAvgDb, lastFramePeakDb, noiseFloorDb,
                          displayNoiseCalibrated ? 1 : 0);
            Serial.printf("sd status: %s ready=%u file=%s q=%lu free=%lu "
                          "chunks=%lu bytes=%llu drops=%lu total_drops=%lu "
                          "q_high=%lu errors=%lu write_avg=%lldus "
                          "write_max=%lldus\n",
                          sdStatusName(currentSdStatus), sdCardReady ? 1 : 0,
                          sdFileName, (unsigned long)queuedFrames,
                          (unsigned long)freeFrames,
                          (unsigned long)sdChunksWritten,
                          (unsigned long long)sdBytesWritten,
                          (unsigned long)sdQueueDrops,
                          (unsigned long)sdQueueDropsTotal,
                          (unsigned long)sdQueueHighWater,
                          (unsigned long)sdErrors, sdAvgWriteUs,
                          sdMaxWriteUs);
            unlockSerial();
            reportStartUs = t5;
            totalReadUs = 0;
            totalRecordQueueUs = 0;
            totalFftUs = 0;
            totalPublishFftUs = 0;
            totalPublishAnalyzerUs = 0;
            totalPublishVuUs = 0;
            totalPublishScopeUs = 0;
            totalActiveUs = 0;
            totalFrameUs = 0;
        }
    }
}

void displayTask(void *) {
    uint32_t frames = 0;
    uint32_t notifiedFrames = 0;
    uint32_t timeoutFrames = 0;
    uint32_t droppedFramesSinceReport = 0;
    uint32_t droppedFramesThisSecond = 0;
    uint32_t droppedFramesLastSecond = 0;
    uint32_t droppedFramesTotal = 0;
    int64_t reportStartUs = esp_timer_get_time();
    int64_t dropSecondStartUs = reportStartUs;
    int64_t totalWaitUs = 0;
    int64_t totalClearUs = 0;
    int64_t totalFrameUs = 0;
    int64_t totalVuUs = 0;
    int64_t totalAnalyzerUs = 0;
    int64_t totalAnalogVuUs = 0;
    int64_t totalSpectrogramUs = 0;
    int64_t totalScopeUs = 0;
    int64_t totalActiveUs = 0;

    for (;;) {
        const int64_t loopStartUs = esp_timer_get_time();
        const uint32_t notified =
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
        const int64_t afterWaitUs = esp_timer_get_time();
        int64_t clearUs = 0;
        int64_t frameUs = 0;

        if (frameDirty) {
            frameDirty = false;
        }

        const int64_t vuStartUs = esp_timer_get_time();
        renderVuMeter();
        const int64_t analyzerStartUs = esp_timer_get_time();
        renderAnalyzer();
        const int64_t analogVuStartUs = esp_timer_get_time();
        renderAnalogVuMeter();
        const int64_t spectrogramStartUs = esp_timer_get_time();
        renderSpectrogram();
        const int64_t scopeStartUs = esp_timer_get_time();
        renderOscilloscope();
        const int64_t frameStartUs = esp_timer_get_time();
        drawFrame();
        renderCalibrationStatusLabel();
        const int64_t doneUs = esp_timer_get_time();

        totalWaitUs += afterWaitUs - loopStartUs;
        totalClearUs += clearUs;
        frameUs = doneUs - frameStartUs;
        totalFrameUs += frameUs;
        totalVuUs += analyzerStartUs - vuStartUs;
        totalAnalyzerUs += analogVuStartUs - analyzerStartUs;
        totalAnalogVuUs += spectrogramStartUs - analogVuStartUs;
        totalSpectrogramUs += scopeStartUs - spectrogramStartUs;
        totalScopeUs += frameStartUs - scopeStartUs;
        totalActiveUs += doneUs - afterWaitUs;
        if (notified > 0) {
            ++notifiedFrames;
            if (notified > 1) {
                const uint32_t dropped = notified - 1;
                droppedFramesSinceReport += dropped;
                droppedFramesThisSecond += dropped;
                droppedFramesTotal += dropped;
            }
        } else {
            ++timeoutFrames;
        }
        ++frames;

        if (doneUs - dropSecondStartUs >= kDropReportWindowUs) {
            droppedFramesLastSecond = droppedFramesThisSecond;
            droppedFramesThisSecond = 0;
            dropSecondStartUs = doneUs;
        }

        if (frames % kDiagnosticsFrames == 0) {
            const int64_t reportWallUs = doneUs - reportStartUs;
            lockSerial();
            Serial.printf("display avg: period=%lldus wait=%lldus clear=%lldus "
                          "frame=%lldus vu=%lldus analyzer=%lldus "
                          "analog_vu=%lldus fft=%lldus "
                          "scope=%lldus active=%lldus duty=%.1f%% "
                          "fft_px_max=%u fft_px_nz=%lu "
                          "notify=%u timeout=%u drop_1s=%u drop_report=%u "
                          "drop_total=%u\n",
                          reportWallUs / kDiagnosticsFrames,
                          totalWaitUs / kDiagnosticsFrames,
                          totalClearUs / kDiagnosticsFrames,
                          totalFrameUs / kDiagnosticsFrames,
                          totalVuUs / kDiagnosticsFrames,
                          totalAnalyzerUs / kDiagnosticsFrames,
                          totalAnalogVuUs / kDiagnosticsFrames,
                          totalSpectrogramUs / kDiagnosticsFrames,
                          totalScopeUs / kDiagnosticsFrames,
                          totalActiveUs / kDiagnosticsFrames,
                          percentOf(totalActiveUs, reportWallUs),
                          lastSpectrogramRenderMax,
                          (unsigned long)lastSpectrogramRenderNonzero,
                          notifiedFrames, timeoutFrames,
                          droppedFramesLastSecond, droppedFramesSinceReport,
                          droppedFramesTotal);
            unlockSerial();

            reportStartUs = doneUs;
            totalWaitUs = 0;
            totalClearUs = 0;
            totalFrameUs = 0;
            totalVuUs = 0;
            totalAnalyzerUs = 0;
            totalAnalogVuUs = 0;
            totalSpectrogramUs = 0;
            totalScopeUs = 0;
            totalActiveUs = 0;
            notifiedFrames = 0;
            timeoutFrames = 0;
            droppedFramesSinceReport = 0;
        }

        vTaskDelay(1);
    }
}

}  // namespace

void setup() {
    amoled.begin();
    amoled.setRotation(0);
    amoled.setBrightness(100);
    beginStartupScreen();

    if (kLrclkSquareWaveTestEnabled) {
        startupScreenStatus("LRCLK square-wave test",
                            "GPIO output test is active; I2S will not start",
                            10);
        pinMode(kI2SLrclk_GPIO, OUTPUT);
        digitalWrite(kI2SLrclk_GPIO, LOW);
        for (;;) {
            digitalWrite(kI2SLrclk_GPIO, HIGH);
            delay(kLrclkSquareWaveHalfPeriodMs);
            digitalWrite(kI2SLrclk_GPIO, LOW);
            delay(kLrclkSquareWaveHalfPeriodMs);
        }
    }

    Serial.begin(115200);
    startupScreenStatus("Opening USB serial",
                        "Waiting for the development monitor window", 10);
    pumpStartupUi(kDevSerialStartupDelayMs);
    serialMutex = xSemaphoreCreateMutex();
    Serial.println("AMOLED begin done");

    Serial.println("t4-i2s starting live I2S FFT display");

    startupScreenStatus("Initialising buttons", "Top toggles recording; side calibrates noise", 18);
    topButton.begin(kTopButton_GPIO);
    sideButton.begin(kSideButton_GPIO);

    Serial.println("buttons initialized");

    startupScreenStatus("Building colour palette", "Preparing analyser display colours", 24);
    buildPalette();
    Serial.println("palette initialized");

    startupScreenStatus("Building FFT window", "Preparing the Hann window", 30);
    buildHannWindow();
    Serial.println("palette and window initialized");

    startupScreenStatus("Preparing SD recorder", "Allocating recording buffers", 38);
    const bool recordBuffersReady = initRecordBuffers();
    startupScreenStatus("Reading SD card", "Listing existing WAV files to serial", 46);
    printSdFileListAtBoot();
    if (!recordBuffersReady) {
        sdCardReady = false;
        setSdStatus(SdStatus::NotReady, "-");
    }

    startupScreenStatus("Checking WiFi", "Scanning for known networks", 56);
    connectToWiFiAndInitOTA(true, 3);

    startupScreenStatus("Preparing display data", "Allocating graph state", 72);
    spectrogramMutex = xSemaphoreCreateMutex();
    clearSpectrogram();

    startupScreenStatus("Initialising FFT", "Preparing audio analysis", 82);
    Serial.println("FFT init");
    fftPlan = fft_init(kFftSize, FFT_REAL, FFT_FORWARD, fftInput, fftOutput);
    fftReady = fftPlan != nullptr;
    if (!fftReady) {
        Serial.println("FFT init failed");
    }

    startupScreenStatus("Initialising I2S", "Starting microphone clocks", 90);
    Serial.println("I2S init");
    i2sReady = initI2SMicrophone();
    if (!i2sReady) {
        Serial.println("I2S microphone init failed");
    }
    Serial.println("I2S init complete");

    Serial.println("clear screen");
    finishStartupScreen();
    clearScreen();
    Serial.println("clear screen complete");

    drawStaticBackgroundPanels();
    drawFrame();
    renderVuMeter();
    renderAnalyzer();
    renderAnalogVuMeter();
    renderSpectrogram();
    renderOscilloscope();
    drawWifiStatusLabelOnce();
    requestDisplayNoiseCalibration("startup");
    Serial.println("initial render complete");

    xTaskCreatePinnedToCore(displayTask, "display", 4096, nullptr, 3,
                            &displayTaskHandle, 1);
    xTaskCreatePinnedToCore(captureTask, "capture_fft", 12288, nullptr, 5,
                            &captureTaskHandle, 0);
    xTaskCreatePinnedToCore(buttonTask, "buttons", 3072, nullptr, 4,
                            &buttonTaskHandle, 0);
    if (recordBuffersReady && sdCardReady) {
        xTaskCreatePinnedToCore(sdWriterTask, "sd_writer", 8192, nullptr, 2,
                                &sdWriterTaskHandle, 0);
    } else {
        Serial.println("sd writer task not started");
    }
    Serial.println("tasks started");
}

void loop() {
    if (ftpActive) {
        ftpServer.handleFTP();
    }

    yield();
}

void _ftpConnectCallback(FtpOperation ftpOperation, uint32_t freeSpace, uint32_t totalSpace){
  switch (ftpOperation) {
    case FTP_CONNECT:
      USB_SERIAL_PRINTLN(F("FTP: Connected!"));
      break;
    case FTP_DISCONNECT:
      USB_SERIAL_PRINTLN(F("FTP: Disconnected!"));
      break;
    case FTP_FREE_SPACE_CHANGE:
      USB_SERIAL_PRINTF("FTP: Free space change, free %lu of %lu!\n", (unsigned long)freeSpace, (unsigned long)totalSpace);
      break;
    default:
      USB_SERIAL_PRINTF("FTP: Unknown Operation (connect), free %lu of %lu!\n", (unsigned long)freeSpace, (unsigned long)totalSpace);
      break;
  }
}

void _ftpTransferCallback(FtpTransferOperation ftpOperation, const char* name, uint32_t transferredSize){
  switch (ftpOperation) {
    case FTP_UPLOAD_START:
      ftpTransferDirection = FtpTransferDirection::Upload;
      USB_SERIAL_PRINTF("FTP: Upload start: %s size=%lu\n", name, (unsigned long)transferredSize);
      break;
    case FTP_UPLOAD:
      USB_SERIAL_PRINTF("FTP: Upload of file %s byte %lu\n", name, (unsigned long)transferredSize);
      break;
    case FTP_DOWNLOAD_START:
      ftpTransferDirection = FtpTransferDirection::Download;
      USB_SERIAL_PRINTF("FTP: Download start: %s size=%lu\n", name, (unsigned long)transferredSize);
      break;
    case FTP_DOWNLOAD:
      USB_SERIAL_PRINTF("FTP: Download of file %s byte %lu\n", name, (unsigned long)transferredSize);
      break;
    case FTP_TRANSFER_STOP:
      if (ftpTransferDirection == FtpTransferDirection::Upload) {
        USB_SERIAL_PRINTLN(F("FTP: Upload finished!"));
      } else if (ftpTransferDirection == FtpTransferDirection::Download) {
        USB_SERIAL_PRINTLN(F("FTP: Download finished!"));
      } else {
        USB_SERIAL_PRINTLN(F("FTP: Transfer finished!"));
      }
      ftpTransferDirection = FtpTransferDirection::None;
      break;
    case FTP_TRANSFER_ERROR:
      if (ftpTransferDirection == FtpTransferDirection::Upload) {
        USB_SERIAL_PRINTLN(F("FTP: Upload error!"));
      } else if (ftpTransferDirection == FtpTransferDirection::Download) {
        USB_SERIAL_PRINTLN(F("FTP: Download error!"));
      } else {
        USB_SERIAL_PRINTLN(F("FTP: Transfer error!"));
      }
      ftpTransferDirection = FtpTransferDirection::None;
      break;
    default:
      USB_SERIAL_PRINTF("FTP: Unknown Operation (transfer) op=%d name=%s byte=%lu\n",
                        (int)ftpOperation, name, (unsigned long)transferredSize);
      break;
  }

  /* FTP_UPLOAD_START = 0,
   * FTP_UPLOAD = 1,
   *
   * FTP_DOWNLOAD_START = 2,
   * FTP_DOWNLOAD = 3,
   *
   * FTP_TRANSFER_STOP = 4,
   * FTP_DOWNLOAD_STOP = 4,
   * FTP_UPLOAD_STOP = 4,
   *
   * FTP_TRANSFER_ERROR = 5,
   * FTP_DOWNLOAD_ERROR = 5,
   * FTP_UPLOAD_ERROR = 5
   */
};

void setupFTPServer()
{
  /*
  Filezilla needs to connect with Simple FTP, no TLS, and maximum connections =1
  Otherwise FileZilla will try to make two connections for a file transfer and it will
  fail at the waiting for welcome message.
  */
  // need to check here if filesystem is ready - ie an SD card is present
  ftpServer.setCallback(_ftpConnectCallback);
  ftpServer.setTransferCallback(_ftpTransferCallback);
  ftpServer.begin("mercator","oceanic");    //username, password for ftp.   (default 21, 50009 for PASV)
  USB_SERIAL_PRINTLN("FTP Server Online");
  startupScreenStatus("FTP server online", "Plain FTP is ready on port 21", 68);
  ftpActive = true;
}

const char* scanForKnownNetwork() // return first known network found
{
  const char* network = nullptr;

  int8_t scanResults = WiFi.scanNetworks();

  if (scanResults != 0)
  {
    for (int i = 0; i < scanResults; ++i) 
    {
      String SSID = WiFi.SSID(i);
      
      // Check if the current device starts with the peerSSIDPrefix
      if (strcmp(SSID.c_str(), ssid_1) == 0)
        network=ssid_1;
      else if (strcmp(SSID.c_str(), ssid_2) == 0)
        network=ssid_2;
      else if (strcmp(SSID.c_str(), ssid_3) == 0)
        network=ssid_3;

      if (network)
        break;
    }    
  }

  if (network)
  {
    USB_SERIAL_PRINTF("Found:\n%s\n",network);
  }
  else
  {
    USB_SERIAL_PRINTLN("No networks Found\n");
  }

  // clean up ram
  WiFi.scanDelete();

  return network;
}


bool connectToWiFiAndInitOTA(const bool wifiOnly, int repeatScanAttempts)
{
  ensureWiFiEventHandler();

  if (wifiOnly && wifiReadyForServices())
  {
    ssid_connected = WiFi.SSID();
    if (!ftpActive) {
      setupFTPServer();
    }
    return true;
  }

  const int totalScanAttempts = repeatScanAttempts;
  int scanAttempt = 0;

  while (repeatScanAttempts-- && (!wifiReadyForServices() || !wifiOnly))
  {
    ++scanAttempt;
    char scanDetail[96];
    snprintf(scanDetail, sizeof(scanDetail), "Scan attempt %d of %d",
             scanAttempt, totalScanAttempts);
    startupScreenStatus("Scanning WiFi", scanDetail, 58);

    const char* network = scanForKnownNetwork();

    if (!network)
    {
      startupScreenStatus("No known WiFi found",
                          "Recording can continue; FTP will stay offline", 62);
      pumpStartupUi(500);
      continue;
    }

    char foundDetail[96];
    snprintf(foundDetail, sizeof(foundDetail), "Found %s", network);
    startupScreenStatus("Known WiFi found", foundDetail, 62);
  
    int connectToFoundNetworkAttempts = 3;
    const int repeatDelay = 500;
      
    if (strcmp(network,ssid_1) == 0)
    {
      while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_1, password_1, label_1, timeout_1, wifiOnly))
        pumpStartupUi(repeatDelay);
    }
    else if (strcmp(network,ssid_2) == 0)
    {
      while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_2, password_2, label_2, timeout_2, wifiOnly))
        pumpStartupUi(repeatDelay);
    }
    else if (strcmp(network,ssid_3) == 0)
    {
      while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_3, password_3, label_3, timeout_3, wifiOnly))
        pumpStartupUi(repeatDelay);
    }
    
    pumpStartupUi(500);
  }

  bool connected=wifiReadyForServices();
  
  if (connected)
  {
    ssid_connected = WiFi.SSID();
    String ip = WiFi.localIP().toString();
    char detail[128];
    snprintf(detail, sizeof(detail), "%s%s", ip.c_str(),
             ftpActive ? " - FTP online" : "");
    startupScreenStatus("WiFi connected", detail, 68);
  }
  else
  {
    ssid_connected = ssid_not_connected;
    startupScreenStatus("WiFi unavailable",
                        "FTP server skipped; audio capture will still start",
                        68);
    pumpStartupUi(800);
  }
  
  return connected;
}

bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly)
{
  bool otaActive = false;
  if (wifiOnly && wifiReadyForServices())
  {
    USB_SERIAL_PRINTF("setupOTAWebServer: attempt to connect wifiOnly, already connected - otaActive=%i\n",otaActive);
    ssid_connected = WiFi.SSID();
    if (!ftpActive) {
      setupFTPServer();
    }
    return true;
  }

  USB_SERIAL_PRINTF("setupOTAWebServer: attempt to connect %s wifiOnly=%i when otaActive=%i\n",_ssid, wifiOnly,otaActive);
  char detail[128];
  snprintf(detail, sizeof(detail), "Connecting to %s", label != nullptr ? label : _ssid);
  startupScreenStatus("Connecting WiFi", detail, 64);

  bool forcedCancellation = false;

  bool connected = false;
  wifiGotIpEvent = false;
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("t4-i2s");

  WiFi.begin(_ssid, _password);

  const uint32_t connectStartMs = millis();
  while (!wifiReadyForServices() &&
         (uint32_t)(millis() - connectStartMs) < timeout)
  {
    pumpStartupUi(50);
  }

  pumpStartupUi(100);

  USB_SERIAL_PRINTF("setupOTAWebServer: connect wait complete status=%d got_ip=%u ip=%s\n",
                    WiFi.status(), wifiGotIpEvent ? 1 : 0,
                    WiFi.localIP().toString().c_str());
  if (wifiReadyForServices())
  {
    String ip = WiFi.localIP().toString();
    startupScreenStatus("WiFi connected", ip.c_str(), 66);
    setupFTPServer();
    
    if (wifiOnly == false && !otaActive)
    {
        /*
      USB_SERIAL_PRINTLN("setupOTAWebServer: WiFi connected ok, starting up OTA");

      USB_SERIAL_PRINTLN("setupOTAWebServer: calling asyncWebServer.on");

      asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(200, "text/plain", "To upload firmware use /update");
      });
        
      USB_SERIAL_PRINTLN("setupOTAWebServer: calling MercatorElegantOta.begin");

      MercatorElegantOta.setID(MERCATOR_OTA_DEVICE_LABEL);
      MercatorElegantOta.begin(&httpQueue, &asyncWebServer);    // Start MercatorElegantOta


      #ifdef USE_WEBSERIAL
      static bool webSerialInitialised = false;

      if (!webSerialInitialised)
      {
        WebSerial.begin(&asyncWebServer);
        WebSerial.msgCallback(webSerialReceiveMessage);
        webSerialInitialised = true;
      }
      #endif

      USB_SERIAL_PRINTLN("setupOTAWebServer: calling asyncWebServer.begin");

      asyncWebServer.begin();

      USB_SERIAL_PRINTLN("setupOTAWebServer: OTA setup complete");


//      compositeSprite->printf("%s\n",WiFi.localIP().toString());
//      compositeSprite->printf("%s\n",WiFi.macAddress().c_str());
//      mapScreen->copyCompositeSpriteToDisplay();

      connected = true;
      otaActive = true;
      
      delay(1000);

      connected = true;
      */
    }
    else
    {
      connected = true;
    }
  }
  else
  {
    USB_SERIAL_PRINTF("setupOTAWebServer: WiFi failed to connect %s\n",_ssid);
    startupScreenStatus("WiFi connection failed",
                        "Trying another configured network if available", 64);

    pumpStartupUi(2000);
  }

  return connected;
}
