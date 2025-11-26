#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TM1637Display.h>
#include <driver/twai.h>
#include <stdint.h>

// Inisialisasi LCD I2C dengan alamat 0x27, 16 kolom, 2 baris
// Alamat sudah dikonfirmasi melalui I2C Scanner
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Definisi pin tombol
#define BUTTON_DOWN 23   // Internal pull up
#define BUTTON_UP 39     // External pull up

// Pin komunikasi UART2: RX2 = GPIO16, TX2 = GPIO17
#define RXD2 16
#define TXD2 17

// Pin RPM ADC dan 7-Segment Display (sama dengan lengkap_esp2.ino)
#define RPM_ADC_PIN 36    // ADC untuk RPM input
#define CLK_PIN 19        // 7-Segment Clock
#define DIO_PIN 18        // 7-Segment Data

// Variabel untuk posisi cursor (volatile karena diakses dari multiple tasks)
volatile int cursorPosition = 0;  // 0: Manual, 1: Auto, 2: Rec, 3: Play
volatile int unitPosition = 0;    // Posisi unit yang dipilih (0-17)
volatile int modePosition = 0;    // Posisi mode yang dipilih (0: Pre-Collision, 1: Soft Brake, 2: Full Brake)
volatile bool inUnitSelectionMode = false; // Mode pemilihan unit
volatile bool inModeSelectionMode = false; // Mode pemilihan simulation mode
volatile bool displayNeedsUpdate = false;  // Flag untuk update display
volatile bool showModeAndUnit = false;     // Flag untuk menampilkan mode dan unit yang aktif
volatile bool inRecordMode = false;        // Mode UI untuk Rec
volatile bool inRecording = false;         // Sedang merekam (counter berjalan)
volatile unsigned long recordStartTime = 0; // Waktu mulai (millis)
volatile unsigned long recordElapsedMs = 0; // Akumulasi waktu saat stop (ms)
// Dialog konfirmasi saat BACK ketika sedang merekam
volatile bool inStopRecordingConfirm = false; // Menampilkan dialog konfirmasi stop
volatile int stopConfirmPosition = 0;         // 0: No, 1: Yes

// Save confirmation after stop (file operations)
volatile bool inSaveConfirm = false;          // Tampilkan dialog Save XX-YY ? > Yes No
volatile int saveConfirmPosition = 0;         // 0: Yes, 1: No
volatile bool inSaveResultScreen = false;     // Menampilkan hasil Saved/Not Saved 3 detik
volatile bool lastSaveResultSaved = false;    // Hasil terakhir
volatile unsigned long saveResultStartTime = 0; // Timer untuk hasil screen
String currentRecordingFilename = "";        // e.g., PC-01.csv (no leading slash)
String currentRecordingLabel = "";           // e.g., PC-01 (tanpa .csv)
String currentRecordingModeAbbr = "";        // e.g., PC/SB/FB chosen at REC_START

// SD Card init handshake state with ESP1
volatile bool inSDInitWait = false;            // Menampilkan layar inisialisasi SD
volatile long sdInitStartTime = 0;             // Waktu mulai tampilan SD init
volatile int sdInitStatus = -1;                // -1: pending, 0: fail, 1: ok
volatile bool inSDInitFailed = false;          // Menampilkan layar gagal SD
volatile long sdInitFailedStartTime = 0;       // Waktu mulai layar gagal
volatile int sdInitRetryCount = 0;             // Jumlah retry attempt (max 5 dalam 10 detik)
volatile unsigned long lastSDInitRequestTime = 0; // Waktu request terakhir ke ESP1

// SD Card removal detection during recording
volatile bool inSDCardMissing = false;         // Menampilkan layar SD card missing
volatile long sdCardMissingStartTime = 0;      // Waktu mulai layar missing

volatile int activeMode = -1;              // Mode yang sedang aktif (-1: none, 0: Manual, 1: Auto, 3: Play)
volatile int activeUnit = -1;              // Unit yang sedang aktif
volatile int activeSimMode = -1;           // Simulation mode yang aktif (-1: none, 0: Pre-Collision, 1: Soft Brake, 2: Full Brake, 3: Full Scenario)

// Radar Object Management
volatile bool inAddRadarMode = false;      // Mode pemilihan radar object untuk ditambahkan
volatile int radarObjectPosition = 0;      // Posisi radar object yang dipilih (0-3)
volatile int currentRadarId = 0;           // Current active radar ID (from ESP1)
volatile float currentRadarRcs = 20.0f;    // Current active radar RCS (from ESP1)
volatile int selectedRadarIds[10];         // Array untuk menyimpan ID yang dipilih (max 10)
volatile int selectedRadarCount = 0;       // Jumlah radar yang sudah dipilih

// Struktur data untuk radar object presets
struct RadarObjectPreset {
  uint8_t id;
  float velocity_lon;  // m/s
  float distance;      // m
  float rcs;
};

// Predefined radar objects untuk dipilih (3 pilihan + 1 reset)
const RadarObjectPreset radarPresets[] = {
  {1, -5.3f, 10.0f, 25.0f},  // ID:1 Vel_Lon:-5.3m/s Dist:10m RCS:25
  {2, -5.3f, 5.0f, 27.0f},   // ID:2 Vel_Lon:-5.3m/s Dist:5m RCS:27
  {3, -5.3f, 2.0f, 30.0f},   // ID:3 Vel_Lon:-5.3m/s Dist:2m RCS:30
  {0, 0.0f, 0.0f, 0.0f}      // ID:0 Special - Reset option
};
const int totalRadarPresets = 4;
const int MAX_SELECTED_RADARS = 3;  // Maksimal 3 ID yang bisa dipilih

// Display rotation untuk baris terpilih
volatile unsigned long displayRotationTimer = 0;
volatile int displayRotationIndex = 0;  // 0: Velocity, 1: Distance, 2: RCS
volatile bool needsClearScreen = false;  // Flag untuk clear screen sekali saat transisi

// RPM Configuration (sama dengan lengkap_esp2.ino)
float RPM_MAX = 3000.0f;                   // Maximum RPM range
volatile float currentRpm = 0.0f;          // Current RPM value
volatile float currentSpeed = 0.0f;        // Speed dari ESP1 via UART2
volatile bool isAutoMode = false;          // Flag untuk mode AUTO
volatile int autoSimMode = -1;             // AUTO simulation mode (-1: none, 0: Pre-Collision, 1: Soft Brake, 2: Full Brake, 3: Full Scenario)
volatile bool forceZeroOutputs = false;    // Force send 0 speed/RPM after exiting AUTO until new mode selected
volatile bool rpmRadarActive = false;      // Mirror radar active state for RPM display logic
volatile bool rpmRunningLEDMode = true;    // Mirror running LED mode for RPM display
volatile bool rpmBlinkState = false;       // Latest blink phase received from ESP1
volatile bool rpmBlinkSynced = false;      // True when at least one blink state received for current idle session
volatile unsigned long rpmBlinkLastSync = 0; // Timestamp of last blink sync (millis)
volatile uint16_t rpmBlinkSequence = 0;    // Sequence/timestamp from ESP1 for diagnostics

// SD Init screen state (untuk prevent re-render)
bool sdInitScreenRendered = false;
int lastDisplayedRetryCount = -1;
bool sdFailScreenRendered = false;
unsigned long lastFailScreenTime = 0;
bool sdMissingScreenRendered = false;
unsigned long lastMissingScreenTime = 0;

// CAN Bus Configuration (sama dengan lengkap_esp2.ino)
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// Unit types untuk CAN mapping (sama dengan lengkap_esp2.ino)
enum unit_type {
  HINO_500 = 0,               // CAN only (500kbps)
  FAW_JH6_420 = 1,           // Pulse only
  HOWO_400 = 2,              // Hybrid: Speed pulse + RPM CAN
  HOWO_400_V7X = 3,          // Pulse only
  HOWO_EV = 4,               // CAN only (Combined message)
  HANVAN_G7 = 5,             // Pulse only
  HANVAN_XG1 = 6,            // Pulse only
  HANVAN_EV = 7,             // CAN only (Special decode)
  X3000 = 8,                 // CAN only
  F3000_420 = 9,             // Pulse only
  F3000_430 = 10,            // Pulse only
  HINO_EURO_2 = 11,          // Pulse only
  HOWO_371 = 12,             // Hybrid: Speed pulse + RPM CAN
  DONGFENG_KY50 = 13,        // CAN only
  F3000_420_CAN = 14,        // CAN only
  F3000_430_CAN = 15,        // CAN only
  AUMAN_EST_380 = 16,        // CAN only
  SANY_SYZ_428_C = 17        // CAN only
};

// Unit Configuration (sama dengan lengkap_esp2.ino)
struct UnitConfig {
  const char* name;
  uint32_t can_baudrate;
  bool extended_id;
  uint32_t can_id_tx;  
  uint32_t can_id_rx;
  bool pulse_only;
  bool hybrid_mode;
};

const UnitConfig unitConfigs[] = {
  {"HINO_500", 500000, false, 0x18FEF117UL, 0x18F00417UL, false, false},         // 0: CAN only (500kbps)
  {"FAW_JH6_420", 0, false, 0, 0, true, false},                                  // 1: Pulse only
  {"HOWO_400", 250000, false, 0x00000000UL, 0x18FF0A17UL, false, true},          // 2: Hybrid (250kbps)
  {"HOWO_400_V7X", 0, false, 0, 0, true, false},                                 // 3: Pulse only
  {"HOWO_EV", 250000, true, 0x0CFE6CEEUL, 0x0CFE6CEEUL, false, false},          // 4: CAN only (250kbps, Combined msg)
  {"HANVAN_G7", 0, false, 0, 0, true, false},                                    // 5: Pulse only
  {"HANVAN_XG1", 0, false, 0, 0, true, false},                                   // 6: Pulse only
  {"HANVAN_EV", 500000, true, 0x18FEF1D0UL, 0x18F00203UL, false, false},        // 7: CAN only (500kbps, Special decode)
  {"X3000", 250000, false, 0x18FEF117UL, 0x18F00417UL, false, false},           // 8: CAN only (250kbps)
  {"F3000_420", 0, false, 0, 0, true, false},                                    // 9: Pulse only
  {"F3000_430", 0, false, 0, 0, true, false},                                    // 10: Pulse only
  {"HINO_EURO_2", 0, false, 0, 0, true, false},                                  // 11: Pulse only
  {"HOWO_371", 250000, false, 0x00000000UL, 0x18FF0A17UL, false, true},          // 12: Hybrid (250kbps)
  {"DONGFENG_KY50", 250000, false, 0x18FEF117UL, 0x18F00417UL, false, false},   // 13: CAN only (250kbps)
  {"F3000_420_CAN", 250000, false, 0x18FEF117UL, 0x18F00417UL, false, false},   // 14: CAN only (250kbps)
  {"F3000_430_CAN", 250000, false, 0x18FE6CEEUL, 0x18F00417UL, false, false},   // 15: CAN only (250kbps)
  {"AUMAN_EST_380", 250000, false, 0x18FEF117UL, 0x18F00417UL, false, false},   // 16: CAN only (250kbps)
  {"SANY_SYZ_428_C", 250000, false, 0x0CFE6CEEUL, 0x0CF00417UL, false, false}   // 17: CAN only (250kbps)
};

// Task handles
TaskHandle_t taskUARTHandle = NULL;
TaskHandle_t taskButtonDownHandle = NULL;
TaskHandle_t taskButtonUpHandle = NULL;
TaskHandle_t taskDisplayHandle = NULL;
TaskHandle_t taskRPMReadHandle = NULL;
TaskHandle_t taskRPMDisplayHandle = NULL;
TaskHandle_t taskCANSendHandle = NULL;

// CAN Variables
volatile bool canEnabled = false;
UnitConfig current_config;

// Semaphore untuk proteksi data
SemaphoreHandle_t xSemaphore = NULL;

// 7-Segment Display Object untuk RPM
TM1637Display rpmDisplay(CLK_PIN, DIO_PIN);

// Daftar unit truck
const char* truckUnits[] = {
  "HINO_500",
  "FAW_JH6_420",
  "HOWO_400",
  "HOWO_400_V7X",
  "HOWO_EV",
  "HANVAN_G7",
  "HANVAN_XG1",
  "HANVAN_EV",
  "X3000",
  "F3000_420",
  "F3000_430",
  "HINO_EURO_2",
  "HOWO_371",
  "DONGFENG_KY50",
  "F3000_420_CAN",
  "F3000_430_CAN",
  "AUMAN_EST_380",
  "SANY_SYZ_428_C"
};
const int totalUnits = 18;

// Daftar mode simulasi untuk Auto mode
const char* simulationModes[] = {
  "Pre-Collision",
  "Soft Brake",
  "Full Brake",
  "Full Scenario"
};
const int totalModes = 4;

// Rec modes (mirip Auto tapi khusus untuk Record)
volatile bool inRecUnitSelect = false;      // Flag pemilihan Unit untuk Rec mode
volatile int selectedRecUnitIndex = 0;      // Unit yang dipilih untuk Rec (0-17)
volatile bool inRecModeSelection = false;   // Flag pemilihan Rec Mode
volatile int recModePosition = 0;           // 0: Pre-Collision, 1: Soft Brake, 2: Full Brake
const char* recModes[] = {
  "Pre-Collision",
  "Soft Brake",
  "Full Brake"
};
const int totalRecModes = 3;

// Play modes (sama dengan Rec modes)
volatile bool inPlayModeSelection = false;  // Flag pemilihan Play Mode
volatile int playModePosition = 0;          // 0: Pre-Collision, 1: Soft Brake, 2: Full Brake
const char* playModes[] = {
  "Pre-Collision",
  "Soft Brake",
  "Full Brake"
};
const int totalPlayModes = 3;

// Play file selection
volatile bool inPlayFileSelection = false;  // Flag pemilihan file CSV untuk Play
volatile int playFilePosition = 0;          // Posisi file yang dipilih
String playFileList[20];                    // Daftar file CSV (max 20 files)
int totalPlayFiles = 0;                     // Jumlah file yang ditemukan
String selectedPlayModeAbbr = "";           // Abbreviation mode yang dipilih (PC/SB/FB)
volatile bool waitingForFileList = false;   // Menunggu response dari ESP1
// Play UI state
volatile bool inPlayView = false;           // Halaman Play aktif (tampilan timer + nama file)
volatile unsigned long playStartMillis = 0; // Waktu mulai timer lokal untuk Play UI
volatile unsigned long playElapsedMs = 0;   // Elapsed yang ditampilkan (disesuaikan dengan CSV)
volatile unsigned long playTotalMs = 0;     // Total durasi dari CSV (ms)
volatile bool playTimerReady = false;       // Flag: timer sudah ready untuk countdown (setelah PLAY_READY)
volatile unsigned long playViewEnteredAt = 0; // Timestamp saat Play UI diaktifkan (fallback sync)
volatile bool playTimerFallbackArmed = false; // True jika fallback countdown dipaksa aktif
String currentPlayFilename = "";            // Nama file CSV yang sedang diputar
volatile bool playCompletionHandled = false; // True ketika sesi play sudah dialihkan ke completed

struct PlayDurationCacheEntry {
  String filename;
  unsigned long duration;
};

PlayDurationCacheEntry playDurationCache[20];
int playDurationCacheCount = 0;
int playDurationCacheCursor = 0;
// Simulation completed state
volatile bool inSimulationCompleted = false; // Halaman "Simulation Completed!" aktif
volatile unsigned long simulationCompletedStartMs = 0; // Waktu mulai tampilan completed
volatile unsigned long lastPlayListRequestMs = 0;
volatile uint8_t playListRequestAttempts = 0;
const uint8_t PLAY_LIST_MAX_ATTEMPTS = 5;

// Kalman Filter Class untuk RPM (sama dengan lengkap_esp2.ino)
class KalmanFilter {
public:
  KalmanFilter(float q = 0.5f, float r = 150.0f, float p = 1.0f, float x = 0.0f) {
    Q = q; R = r; P = p; X = x; K = 0.0f;
  }
  float update(float measurement) {
    P = P + Q;
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;
    return X;
  }
  float get() { return X; }
  void setState(float x, float p = 1.0f) { X = x; P = p; }
private:
  float Q, R, P, K, X;
};

KalmanFilter kfRPM(0.15f, 0.3f, 1.0f, 0.0f);  // Parameter sama seperti ESP1 (lebih responsif)

// Function prototypes
void taskUART(void *parameter);
void taskButtonDown(void *parameter);
void taskButtonUp(void *parameter);
void taskDisplay(void *parameter);
void moveCursorDown();
void moveCursorUp();
void handleSelectButton();
void handleBackButton();
void displayMenu();
void displayUnitSelection();
void displayModeSelection();
void displayModeAndUnit();
void displayRadarObjectSelection();
void displayRecordScreen();
void displayRecUnitSelection();
void displayRecModeSelection();
void displayPlayModeSelection();
void displayPlayFileSelection();
void displayPlayScreen();
void displaySimulationCompleted();
void displayStopRecordingConfirm();
void displaySDInitScreen();
void displaySDInitFailed();
void displaySDCardMissing();
String buildRadarObjectText(int index);
void scanI2C();
void taskRPMRead(void *parameter);
void taskRPMDisplay(void *parameter);
void showFull4Digits(TM1637Display &disp, int value);
void taskCANSend(void *parameter);
void initCAN();
void sendHOWOCombinedMessage(float speed_kmh, uint16_t engine_rpm_raw);
void sendVehicleSpeed(float speed_kmh);
bool sendEngineSpeed(uint16_t value16);
void sendZeroOutputsOnce();
void applyModeToRPMState(const String &mode);
unsigned long getCachedPlayDuration(const String &filename);
void upsertPlayDurationCache(const String &filename, unsigned long durationMs);
void enterSimulationCompleted(const char *reason);
void enterSimulationCompletedLocked(const char *reason);
void requestPlayFileList(const String &abbrev);

void applyModeToRPMState(const String &mode) {
  if (mode == "IDLE") {
    rpmRadarActive = false;
    rpmRunningLEDMode = true;
  } else if (mode == "MANUAL" || mode == "AUTO" || mode == "PLAY") {
    rpmRadarActive = true;
    rpmRunningLEDMode = false;
  }
  rpmBlinkSynced = false;
  rpmBlinkState = false;
  rpmBlinkLastSync = 0;
  rpmBlinkSequence = 0;
}

unsigned long getCachedPlayDuration(const String &filename) {
  if (filename.length() == 0) {
    return 0;
  }
  for (int i = 0; i < playDurationCacheCount; i++) {
    if (playDurationCache[i].filename == filename) {
      return playDurationCache[i].duration;
    }
  }
  return 0;
}

void upsertPlayDurationCache(const String &filename, unsigned long durationMs) {
  if (filename.length() == 0) {
    return;
  }
  for (int i = 0; i < playDurationCacheCount; i++) {
    if (playDurationCache[i].filename == filename) {
      playDurationCache[i].duration = durationMs;
      return;
    }
  }
  if (playDurationCacheCount < 20) {
    playDurationCache[playDurationCacheCount].filename = filename;
    playDurationCache[playDurationCacheCount].duration = durationMs;
    playDurationCacheCount++;
  } else {
    playDurationCache[playDurationCacheCursor].filename = filename;
    playDurationCache[playDurationCacheCursor].duration = durationMs;
    playDurationCacheCursor = (playDurationCacheCursor + 1) % 20;
  }
}

void enterSimulationCompletedLocked(const char *reason) {
  if (inSimulationCompleted) {
    return;
  }

  inPlayView = false;
  inSimulationCompleted = true;
  playCompletionHandled = true;
  playViewEnteredAt = 0;
  playTimerFallbackArmed = false;
  playTimerReady = false;
  playStartMillis = 0;
  simulationCompletedStartMs = millis();
  playElapsedMs = playTotalMs;
  currentSpeed = 0.0f;
  currentRpm = 0.0f;
  forceZeroOutputs = true;
  sendZeroOutputsOnce();
  displayNeedsUpdate = true;
  Serial.printf("[Play Completion] Transition via %s\n", reason ? reason : "unknown");
}

void enterSimulationCompleted(const char *reason) {
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    enterSimulationCompletedLocked(reason);
    xSemaphoreGive(xSemaphore);
  }
}

void requestPlayFileList(const String &abbrev) {
  if (abbrev.length() == 0) {
    return;
  }
  Serial2.println("PLAY_LIST:" + abbrev);
  lastPlayListRequestMs = millis();
  playListRequestAttempts++;
  Serial.printf("[Play] Requesting file list for %s (attempt %u)\n", abbrev.c_str(), playListRequestAttempts);
}

void setup() {
  // Inisialisasi Serial untuk debugging
  Serial.begin(115200);
  Serial.println("Memulai program LCD I2C dengan RTOS...");
  
  // Inisialisasi Serial2 untuk komunikasi dengan ESP1
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Inisialisasi I2C dengan pin custom
  // SDA = GPIO 33, SCL = GPIO 22
  Wire.begin(33, 22);
  
  // Tunggu sebentar untuk stabilitas I2C
  delay(100);
  
  // Scan alamat I2C untuk debugging
  Serial.println("Scanning I2C devices...");
  scanI2C();
  
  // Inisialisasi LCD
  lcd.init();
  
  // Nyalakan backlight
  lcd.backlight();
  
  // Bersihkan layar
  lcd.clear();
  delay(100);
  
  // Setup pin tombol
  pinMode(BUTTON_DOWN, INPUT_PULLUP);  // Internal pull up
  pinMode(BUTTON_UP, INPUT);           // External pull up
  
  // Setup ADC untuk RPM (sama dengan lengkap_esp2.ino)
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetPinAttenuation(RPM_ADC_PIN, ADC_11db);  // GPIO36 untuk RPM
  pinMode(RPM_ADC_PIN, INPUT);
  
  // Initialize 7-Segment Display untuk RPM
  rpmDisplay.setBrightness(7);
  rpmDisplay.clear();
  Serial.println("RPM 7-Segment display initialized");
  
  // Initialize CAN Bus (disabled by default)
  // Will be enabled when CAN unit is selected
  Serial.println("CAN Bus initialization deferred until unit selection");
  
  // Buat semaphore untuk proteksi data
  xSemaphore = xSemaphoreCreateMutex();
  
  // Buat tasks
  xTaskCreatePinnedToCore(
    taskUART,
    "UART Task",
    4096,
    NULL,
    2,  // Priority tinggi untuk UART
    &taskUARTHandle,
    1   // Core 1
  );
  
  xTaskCreatePinnedToCore(
    taskButtonDown,
    "Button DOWN Task",
    2048,
    NULL,
    1,
    &taskButtonDownHandle,
    0   // Core 0
  );
  
  xTaskCreatePinnedToCore(
    taskButtonUp,
    "Button UP Task",
    2048,
    NULL,
    1,
    &taskButtonUpHandle,
    0   // Core 0
  );
  
  xTaskCreatePinnedToCore(
    taskDisplay,
    "Display Task",
    4096,
    NULL,
    1,
    &taskDisplayHandle,
    1   // Core 1
  );
  
  xTaskCreatePinnedToCore(
    taskRPMRead,
    "RPM Read Task",
    2048,
    NULL,
    3,  // Priority TERTINGGI untuk RPM read
    &taskRPMReadHandle,
    0   // Core 0
  );
  
  xTaskCreatePinnedToCore(
    taskRPMDisplay,
    "RPM Display Task",
    2048,
    NULL,
    1,
    &taskRPMDisplayHandle,
    1   // Core 1
  );
  
  xTaskCreatePinnedToCore(
    taskCANSend,
    "CAN Send Task",
    4096,
    NULL,
    3,  // Priority TERTINGGI untuk CAN send (speed/RPM)
    &taskCANSendHandle,
    0   // Core 0
  );
  
  // Tampilkan menu awal
  displayMenu();
  
  Serial.println("RTOS Tasks dimulai. Menu dengan navigasi siap digunakan");
}

void loop() {
  // Loop kosong karena menggunakan RTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void moveCursorDown() {
  if (inStopRecordingConfirm) {
    // Toggle pilihan No/Yes (2 opsi)
    stopConfirmPosition++;
    if (stopConfirmPosition > 1) stopConfirmPosition = 0;
  } else if (inSaveConfirm) {
    // Toggle Yes/No
    saveConfirmPosition++;
    if (saveConfirmPosition > 1) saveConfirmPosition = 0;
  } else if (inAddRadarMode) {
    // Mode pemilihan radar object - navigasi ke object selanjutnya yang belum dipilih
    int startPosition = radarObjectPosition;
    do {
      radarObjectPosition++;
      if (radarObjectPosition >= totalRadarPresets) {
        radarObjectPosition = 0; // Kembali ke awal
      }
      
      // Cek apakah object ini sudah dipilih (kecuali reset option)
      bool isSelected = false;
      uint8_t currentId = radarPresets[radarObjectPosition].id;
      if (currentId != 0) { // Skip check untuk reset option (ID:0)
        for (int i = 0; i < selectedRadarCount; i++) {
          if (selectedRadarIds[i] == currentId) {
            isSelected = true;
            break;
          }
        }
      }
      
      // Jika belum dipilih, break loop
      if (!isSelected) {
        break;
      }
      
      // Prevent infinite loop jika semua sudah dipilih
      if (radarObjectPosition == startPosition) {
        break;
      }
    } while (true);
    
    displayRotationIndex = 0; // Reset ke display pertama (Velocity)
    displayRotationTimer = millis();
    needsClearScreen = true; // Clear screen untuk object baru
  } else if (inModeSelectionMode) {
    // Mode pemilihan simulation mode - navigasi ke mode selanjutnya
    modePosition++;
    if (modePosition >= totalModes) {
      modePosition = 0; // Kembali ke awal
    }
  } else if (inRecUnitSelect) {
    // Mode pemilihan unit untuk Rec - navigasi ke unit selanjutnya
    selectedRecUnitIndex++;
    if (selectedRecUnitIndex >= totalUnits) {
      selectedRecUnitIndex = 0; // Kembali ke awal
    }
  } else if (inRecModeSelection) {
    // Mode pemilihan Rec Mode - navigasi ke mode selanjutnya
    recModePosition++;
    if (recModePosition >= totalRecModes) {
      recModePosition = 0; // Kembali ke awal
    }
  } else if (inPlayModeSelection) {
    // Mode pemilihan Play Mode - navigasi ke mode selanjutnya
    playModePosition++;
    if (playModePosition >= totalPlayModes) {
      playModePosition = 0; // Kembali ke awal
    }
  } else if (inPlayFileSelection) {
    // Mode pemilihan Play File - navigasi ke file selanjutnya
    playFilePosition++;
    if (playFilePosition >= totalPlayFiles) {
      playFilePosition = 0; // Kembali ke awal
    }
  } else if (inUnitSelectionMode) {
    // Mode pemilihan unit - navigasi ke unit selanjutnya
    unitPosition++;
    if (unitPosition >= totalUnits) {
      unitPosition = 0; // Kembali ke awal
    }
  } else {
    // Mode menu utama
    cursorPosition++;
    if (cursorPosition > 3) {
      cursorPosition = 0; // Kembali ke Manual
    }
  }
}

void moveCursorUp() {
  if (inStopRecordingConfirm) {
    // Toggle pilihan No/Yes (2 opsi)
    stopConfirmPosition--;
    if (stopConfirmPosition < 0) stopConfirmPosition = 1;
  } else if (inAddRadarMode) {
    // Mode pemilihan radar object - navigasi ke object sebelumnya yang belum dipilih
    int startPosition = radarObjectPosition;
    do {
      radarObjectPosition--;
      if (radarObjectPosition < 0) {
        radarObjectPosition = totalRadarPresets - 1; // Ke object terakhir
      }
      
      // Cek apakah object ini sudah dipilih (kecuali reset option)
      bool isSelected = false;
      uint8_t currentId = radarPresets[radarObjectPosition].id;
      if (currentId != 0) { // Skip check untuk reset option (ID:0)
        for (int i = 0; i < selectedRadarCount; i++) {
          if (selectedRadarIds[i] == currentId) {
            isSelected = true;
            break;
          }
        }
      }
      
      // Jika belum dipilih, break loop
      if (!isSelected) {
        break;
      }
      
      // Prevent infinite loop jika semua sudah dipilih
      if (radarObjectPosition == startPosition) {
        break;
      }
    } while (true);
    
    displayRotationIndex = 0; // Reset ke display pertama (Velocity)
    displayRotationTimer = millis();
    needsClearScreen = true; // Clear screen untuk object baru
  } else if (inModeSelectionMode) {
    // Mode pemilihan simulation mode - navigasi ke mode sebelumnya
    modePosition--;
    if (modePosition < 0) {
      modePosition = totalModes - 1; // Ke mode terakhir
    }
  } else if (inRecUnitSelect) {
    // Mode pemilihan unit untuk Rec - navigasi ke unit sebelumnya
    selectedRecUnitIndex--;
    if (selectedRecUnitIndex < 0) {
      selectedRecUnitIndex = totalUnits - 1; // Ke unit terakhir
    }
  } else if (inRecModeSelection) {
    // Mode pemilihan Rec Mode - navigasi ke mode sebelumnya
    recModePosition--;
    if (recModePosition < 0) {
      recModePosition = totalRecModes - 1; // Ke mode terakhir
    }
  } else if (inPlayModeSelection) {
    // Mode pemilihan Play Mode - navigasi ke mode sebelumnya
    playModePosition--;
    if (playModePosition < 0) {
      playModePosition = totalPlayModes - 1; // Ke mode terakhir
    }
  } else if (inPlayFileSelection) {
    // Mode pemilihan Play File - navigasi ke file sebelumnya
    playFilePosition--;
    if (playFilePosition < 0) {
      playFilePosition = totalPlayFiles - 1; // Ke file terakhir
    }
  } else if (inUnitSelectionMode) {
    // Mode pemilihan unit - navigasi ke unit sebelumnya
    unitPosition--;
    if (unitPosition < 0) {
      unitPosition = totalUnits - 1; // Ke unit terakhir
    }
  } else {
    // Mode menu utama
    cursorPosition--;
    if (cursorPosition < 0) {
      cursorPosition = 3; // Kembali ke Play
    }
  }
}

void handleSelectButton() {
  Serial.println("SELECT diterima dari ESP1");
  
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    // Saat sedang proses SD init atau layar gagal SD, abaikan SELECT
    if (inSDInitWait || inSDInitFailed) {
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Save confirmation dialog
    if (inSaveConfirm) {
      if (saveConfirmPosition == 0) {
        // Yes -> Save file
        Serial.println("[SAVE] User confirmed: Save");
        Serial2.println("REC_SAVE");
        Serial2.flush();
        lastSaveResultSaved = true;
      } else {
        // No -> Delete file
        Serial.println("[SAVE] User confirmed: Delete");
        Serial2.println("REC_DELETE");
        Serial2.flush();
        lastSaveResultSaved = false;
      }
      inSaveConfirm = false;
      inSaveResultScreen = true;
      saveResultStartTime = millis();
      displayNeedsUpdate = true;
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang di dialog konfirmasi stop recording
    if (inStopRecordingConfirm) {
      if (stopConfirmPosition == 0) {
        // No: Tutup dialog dan kembali ke Record UI tanpa menghentikan perekaman
        inStopRecordingConfirm = false;
        displayNeedsUpdate = true;
        Serial.println("Stop Recording? -> No (lanjut merekam)");
      } else {
        // Yes: Hentikan perekaman, kirim REC_STOP, lalu tampilkan dialog Save
        inStopRecordingConfirm = false;
        inRecording = false;
        unsigned long now = millis();
        recordElapsedMs = now - recordStartTime; // simpan elapsed untuk tampilan
        Serial.println("Stop Recording? -> Yes (STOP and open Save dialog)");
        Serial2.println("REC_STOP");
        inSaveConfirm = true;
        saveConfirmPosition = 0; // Default Yes
        displayNeedsUpdate = true;
      }
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang memilih unit untuk Rec, proceed ke SD Init dan kirim unit/mode ke ESP1
    if (inRecUnitSelect) {
      inRecUnitSelect = false;
      
      // Set current_config berdasarkan unit yang dipilih
      current_config = unitConfigs[selectedRecUnitIndex];
      
      // Set activeUnit untuk konsistensi dengan mode lain
      activeUnit = selectedRecUnitIndex;
      
      // Inisialisasi CAN jika unit memerlukan CAN
      if (!current_config.pulse_only) {
        // Unit ini menggunakan CAN
        if (canEnabled) {
          // Stop dan uninstall CAN sebelumnya jika ada
          twai_stop();
          twai_driver_uninstall();
          Serial.println("Previous CAN stopped");
        }
        
        // Install CAN dengan konfigurasi unit yang dipilih
        initCAN();
        canEnabled = true;
        Serial.printf("CAN initialized for Rec mode: %s (baudrate: %d)\n", 
                      current_config.name, current_config.can_baudrate);
      } else {
        canEnabled = false;
        Serial.printf("Pulse-only unit selected for Rec mode: %s\n", current_config.name);
      }
      
      // Kirim MODE:MANUAL dan UNIT ke ESP1 untuk enable radar
      Serial.printf("Rec Unit selected: %s (index %d)\n", truckUnits[selectedRecUnitIndex], selectedRecUnitIndex);
      Serial2.println("MODE:MANUAL");
      applyModeToRPMState("MANUAL");
      Serial2.print("UNIT:");
      Serial2.println(selectedRecUnitIndex);
      Serial2.flush(); // Ensure commands are sent immediately
      
      // Mulai proses inisialisasi SD di ESP1 dengan retry mechanism
      resetSDInitScreenState(); // Reset display state sebelum mulai
      inSDInitWait = true;
      sdInitStatus = -1; // pending
      sdInitStartTime = millis();
      sdInitRetryCount = 0; // Reset retry counter
      lastSDInitRequestTime = millis();
      displayNeedsUpdate = true;
      Serial.println("SDCard Initialization... (attempt 1/5)");
      Serial2.println("SD_INIT");
      
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang memilih Rec Mode, konfirmasi dan masuk ke UI Record
    if (inRecModeSelection) {
      inRecModeSelection = false;
      inRecordMode = true;
      inRecording = false;
      recordElapsedMs = 0;
      recordStartTime = millis();
      displayNeedsUpdate = true;
      Serial.printf("Rec Mode selected: %s -> Enter Record UI (RPM=%.1f, unit=%d)\n", 
                    recModes[recModePosition], currentRpm, selectedRecUnitIndex);
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang memilih Play Mode, lanjut ke pemilihan file CSV
    if (inPlayModeSelection) {
      inPlayModeSelection = false;
      inPlayFileSelection = true;
      waitingForFileList = true;
      playFilePosition = 0;
      totalPlayFiles = 0;
      
      // Simpan abbreviation mode yang dipilih
      selectedPlayModeAbbr = (playModePosition == 0) ? "PC" : (playModePosition == 1) ? "SB" : "FB";
      
      // Kirim permintaan daftar file ke ESP1
      playListRequestAttempts = 0;
      requestPlayFileList(selectedPlayModeAbbr);
      Serial.printf("Play Mode selected: %s -> Requesting file list from ESP1\n", playModes[playModePosition]);
      displayNeedsUpdate = true;
      
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang memilih Play File, konfirmasi dan masuk ke Play UI
    if (inPlayFileSelection) {
      if (totalPlayFiles > 0) {
        // Simpan file dan transisi ke layar Play
        String selectedFile = playFileList[playFilePosition];
        currentPlayFilename = selectedFile;
        inPlayFileSelection = false;
        inPlayView = true;
        
        // RESET SEMUA STATE untuk play session baru
        playElapsedMs = 0;
        playStartMillis = 0;  // Reset ke 0, akan diset ketika PLAY_READY diterima
        playTimerReady = false; // Timer belum ready sampai PLAY_READY diterima
        playViewEnteredAt = millis();
        playTimerFallbackArmed = false;
        playTotalMs = 0;
        playCompletionHandled = false;

        unsigned long cachedDuration = getCachedPlayDuration(selectedFile);
        if (cachedDuration > 0) {
          playTotalMs = cachedDuration;
          Serial.printf("[Play Timer] Using cached duration %lu ms for %s\n", cachedDuration, selectedFile.c_str());
        }

        // ROBUST CAN INITIALIZATION: Stop CAN jika masih aktif, lalu reinit
        if (canEnabled) {
          Serial.println("Stopping existing CAN for fresh init...");
          twai_stop();
          twai_driver_uninstall();
          canEnabled = false;
          delay(100); // Wait for clean shutdown
        }
        
        // ROBUST: Ensure activeUnit is valid, fallback to unitPosition if needed
        if (activeUnit < 0) {
          activeUnit = unitPosition;
          Serial.printf("WARNING: activeUnit was invalid, using unitPosition=%d\n", unitPosition);
        }
        
        // Update current_config untuk activeUnit
        current_config = unitConfigs[activeUnit];
        Serial.printf("Play file selection: Using unit %d (%s)\n", activeUnit, current_config.name);
        
        // Initialize CAN untuk Play mode yang baru (sesuai dengan unit config)
        if (!current_config.pulse_only) {
          initCAN();
          canEnabled = true;
          Serial.printf("CAN reinitialized for new Play session (Unit: %s, Baudrate: %d)\n", 
                        current_config.name, current_config.can_baudrate);
        } else {
          canEnabled = false;
          Serial.printf("Play Unit %s is pulse-only, CAN disabled\n", current_config.name);
        }

        // Set context ke Play mode (aktifkan mode seperti AUTO untuk replay data CSV)
        activeMode = 3; // Play mode
        // activeUnit sudah diset sebelumnya (saat pilih unit) dan divalidasi di atas
        activeSimMode = playModePosition; // Simpan mode yang dipilih (0: PC, 1: SB, 2: FB)
        isAutoMode = true;  // Set AUTO mode flag agar RPM mengikuti speed dari CSV
        autoSimMode = playModePosition; // Set simulation mode sesuai pilihan Play
        forceZeroOutputs = false;
        showModeAndUnit = false; // gunakan Play UI, bukan ModeAndUnit
        
        // Reset speed dan RPM ke 0 untuk start bersih
        currentSpeed = 0.0f;
        currentRpm = 0.0f;
        
    // Send MODE:PLAY dan UNIT ke ESP1 untuk aktifkan radar
    Serial2.println("MODE:PLAY");
    applyModeToRPMState("PLAY");
    Serial2.println("UNIT:" + String(activeUnit));
    Serial2.println("RESET_RADAR");
    Serial2.println("ADD_RADAR:1,0,50,20");
    Serial.printf("Sent MODE:PLAY, UNIT:%d and default radar to ESP1\n", activeUnit);
        
        displayNeedsUpdate = true;

        // Kirim perintah ke ESP1 untuk load file CSV yang dipilih
        Serial2.println("PLAY_FILE:" + selectedFile);
        Serial.printf("Play File selected: %s -> Enter Play UI (CAN reinitialized, radar activated)\n", selectedFile.c_str());

        xSemaphoreGive(xSemaphore);
        return;
      }
    }
    // Jika sedang di Record UI, toggle START/STOP
    if (inRecordMode) {
      if (!inRecording) {
        inRecording = true;
        unsigned long now = millis();
        // Lanjut dari elapsed jika pernah pause
        recordStartTime = now - recordElapsedMs;
        
        // Track chosen abbreviation and prepare for REC_STARTED response
        const char* abbr = (recModePosition == 0) ? "PC" : (recModePosition == 1) ? "SB" : "FB";
        currentRecordingModeAbbr = String(abbr);
        
        // Pre-populate label immediately (will be updated when REC_STARTED arrives)
        currentRecordingLabel = String(abbr) + "-??";
        currentRecordingFilename = "";
        
        Serial.printf("Record: START with mode %s\n", abbr);
        Serial2.print("REC_START:");
        Serial2.println(abbr);
        Serial2.flush(); // Ensure command is sent
      } else {
        inRecording = false;
        unsigned long now = millis();
        recordElapsedMs = now - recordStartTime;
        
        Serial.println("Record: STOP");
        Serial2.println("REC_STOP");
        Serial2.flush(); // Ensure command is sent
        
        // Wait briefly for REC_STOPPED response before showing save dialog
        vTaskDelay(pdMS_TO_TICKS(50));
        
        inSaveConfirm = true;
        saveConfirmPosition = 0; // Default Yes
      }
      displayNeedsUpdate = true;
      xSemaphoreGive(xSemaphore);
      return;
    }
    if (inAddRadarMode) {
      uint8_t selectedId = radarPresets[radarObjectPosition].id;
      
      // Cek apakah ini reset option (ID:0)
      if (selectedId == 0) {
        // Reset semua selected IDs
        selectedRadarCount = 0;
        Serial.println("RESET: All selected radar IDs cleared");
        
        // Kirim perintah RESET ke ESP1
        Serial2.println("RESET_RADAR");
        
        // Reset posisi ke ID pertama yang available
        radarObjectPosition = 0;
        displayRotationIndex = 0;
        displayRotationTimer = millis();
        needsClearScreen = true;
        displayNeedsUpdate = true;
        xSemaphoreGive(xSemaphore);
        return;
      }
      
      // Cek apakah sudah mencapai batas maksimal
      if (selectedRadarCount >= MAX_SELECTED_RADARS) {
        Serial.println("Maximum radar objects reached (3)");
        xSemaphoreGive(xSemaphore);
        return;
      }
      
      // Cek apakah ID sudah dipilih sebelumnya
      bool alreadySelected = false;
      for (int i = 0; i < selectedRadarCount; i++) {
        if (selectedRadarIds[i] == selectedId) {
          alreadySelected = true;
          break;
        }
      }
      
      if (alreadySelected) {
        Serial.printf("ID:%d already selected, skipping\n", selectedId);
        xSemaphoreGive(xSemaphore);
        return;
      }
      
      // Tambahkan ID yang dipilih ke array
      selectedRadarIds[selectedRadarCount] = selectedId;
      selectedRadarCount++;
      Serial.printf("Radar Object dipilih: ID:%d (Total selected: %d)\n", 
                    selectedId, selectedRadarCount);
      
      // Kirim perintah ke ESP1 untuk menambahkan radar object
      Serial2.printf("ADD_RADAR:%d,%.1f,%.0f,%.0f\n",
                     selectedId,
                     radarPresets[radarObjectPosition].velocity_lon,
                     radarPresets[radarObjectPosition].distance,
                     radarPresets[radarObjectPosition].rcs);
      
      // Pindah ke object berikutnya yang belum dipilih
      int startPosition = radarObjectPosition;
      do {
        radarObjectPosition++;
        if (radarObjectPosition >= totalRadarPresets) {
          radarObjectPosition = 0;
        }
        
        // Cek apakah object ini sudah dipilih (kecuali reset option)
        bool isSelected = false;
        uint8_t currentId = radarPresets[radarObjectPosition].id;
        if (currentId != 0) { // Skip check untuk reset option
          for (int i = 0; i < selectedRadarCount; i++) {
            if (selectedRadarIds[i] == currentId) {
              isSelected = true;
              break;
            }
          }
        }
        
        // Jika belum dipilih atau reset option, break
        if (!isSelected) {
          break;
        }
        
        // Prevent infinite loop
        if (radarObjectPosition == startPosition) {
          break;
        }
      } while (true);
      
      displayRotationIndex = 0;
      displayRotationTimer = millis();
      needsClearScreen = true;
      displayNeedsUpdate = true;
      xSemaphoreGive(xSemaphore);
      return;
    } else if (inModeSelectionMode) {
      // Keluar dari mode pemilihan simulation mode dan simpan pilihan
      inModeSelectionMode = false;
      Serial.print("Simulation mode dipilih: ");
      Serial.println(simulationModes[modePosition]);
      
          // Simpan mode simulasi yang aktif
          activeSimMode = modePosition;
          isAutoMode = true;  // Set AUTO mode flag
          autoSimMode = modePosition;  // Store simulation mode
          forceZeroOutputs = false;    // Clear zero forcing when a new mode starts
          
          // Simpan mode dan unit yang aktif
          activeMode = cursorPosition;
          activeUnit = unitPosition;
          showModeAndUnit = true;      // Setup unit configuration dan CAN
      current_config = unitConfigs[unitPosition];
      
      // Aktivasi CAN jika unit adalah CAN atau Hybrid
      if (!current_config.pulse_only) {
        Serial.printf("Activating CAN for unit: %s (Baudrate: %d)\n", 
                      current_config.name, current_config.can_baudrate);
        initCAN();
        canEnabled = true;
        
        // Aktifkan radar dengan default object (seperti Play mode)
        Serial2.println("RESET_RADAR");
        Serial2.println("ADD_RADAR:1,0,50,20");
        Serial.println("Radar activated with default object (ID:1, vel_lon:0, dist:50, rcs:20) for Auto mode");
      } else {
        canEnabled = false;
        Serial.printf("Unit %s is pulse-only, CAN disabled\n", current_config.name);
      }
      
      // Kirim mode, unit, dan simulation mode ke ESP1 via Serial2
      String modeStr = "";
      if (cursorPosition == 0) modeStr = "MANUAL";
      else if (cursorPosition == 1) modeStr = "AUTO";
      else if (cursorPosition == 3) modeStr = "PLAY";
      
      if (modeStr != "") {
        Serial2.println("MODE:" + modeStr);
        applyModeToRPMState(modeStr);
        Serial2.println("UNIT:" + String(unitPosition));
        if (cursorPosition == 1) { // Auto mode - kirim simulation mode
          String simModeStr = "";
          if (modePosition == 0) simModeStr = "PRE_COLLISION";
          else if (modePosition == 1) simModeStr = "SOFT_BRAKE";
          else if (modePosition == 2) simModeStr = "FULL_BRAKE";
          else if (modePosition == 3) simModeStr = "FULL_SCENARIO";
          Serial2.println("SIM_MODE:" + simModeStr);
          Serial.printf("Sent to ESP1: MODE:%s, UNIT:%d, SIM_MODE:%s\n", modeStr.c_str(), unitPosition, simModeStr.c_str());
        } else {
          Serial.printf("Sent to ESP1: MODE:%s, UNIT:%d\n", modeStr.c_str(), unitPosition);
        }
      }
      
      displayNeedsUpdate = true;
    } else if (inUnitSelectionMode) {
      // Keluar dari mode pemilihan unit
      inUnitSelectionMode = false;
      
      // Cek apakah Auto mode - jika ya, masuk ke mode selection
      if (cursorPosition == 1) { // Auto mode
        inModeSelectionMode = true;
        modePosition = 0; // Reset ke mode pertama
        displayNeedsUpdate = true;
        Serial.print("Unit dipilih: ");
        Serial.println(truckUnits[unitPosition]);
        Serial.println("Masuk mode pemilihan simulation mode untuk Auto");
      } else if (cursorPosition == 0) {
        // Manual - langsung selesai tanpa mode selection dan tanpa SD init
        Serial.print("Unit dipilih: ");
        Serial.println(truckUnits[unitPosition]);
        
        // Simpan mode dan unit yang aktif
        activeMode = cursorPosition;
        activeUnit = unitPosition;
        activeSimMode = -1; // Tidak ada simulation mode untuk Manual
        isAutoMode = false; // Reset AUTO mode flag
        autoSimMode = -1;   // Reset AUTO simulation mode
        forceZeroOutputs = false; // New session starts, clear any previous forcing
        showModeAndUnit = true;
        
        // Setup unit configuration dan CAN
        current_config = unitConfigs[unitPosition];
        
        // Aktivasi CAN jika unit adalah CAN atau Hybrid
        if (!current_config.pulse_only) {
          Serial.printf("Activating CAN for unit: %s (Baudrate: %d)\n", 
                        current_config.name, current_config.can_baudrate);
          initCAN();
          canEnabled = true;
        } else {
          canEnabled = false;
          Serial.printf("Unit %s is pulse-only, CAN disabled\n", current_config.name);
        }
        
        // Kirim mode dan unit ke ESP1 via Serial2
        Serial2.println("MODE:MANUAL");
    applyModeToRPMState("MANUAL");
        Serial2.println("UNIT:" + String(unitPosition));
        Serial.printf("Sent to ESP1: MODE:MANUAL, UNIT:%d\n", unitPosition);
        
        displayNeedsUpdate = true;
      } else if (cursorPosition == 3) {
        // Play - perlu inisialisasi SD Card seperti Rec mode
        Serial.print("Unit dipilih untuk Play: ");
        Serial.println(truckUnits[unitPosition]);
        
        // Simpan unit yang dipilih (radar dan CAN akan diaktifkan nanti setelah pilih file CSV)
        activeUnit = unitPosition;
        
        // Setup unit configuration (JANGAN aktifkan CAN/radar dulu, tunggu sampai pilih file)
        current_config = unitConfigs[unitPosition];
        Serial.printf("Play unit selected: %s (CAN/radar will activate after file selection)\n", current_config.name);
        
        // JANGAN kirim MODE:PLAY atau init CAN di sini!
        // Radar dan CAN hanya akan diaktifkan setelah user memilih file CSV
        
        // Mulai proses inisialisasi SD di ESP1 dengan retry mechanism
        resetSDInitScreenState();
        inSDInitWait = true;
        sdInitStatus = -1; // pending
        sdInitStartTime = millis();
        sdInitRetryCount = 0;
        lastSDInitRequestTime = millis();
        displayNeedsUpdate = true;
        Serial.println("Play mode: SDCard Initialization... (attempt 1/5)");
        Serial2.println("SD_INIT");
      }
    } else if (!inUnitSelectionMode) {
      // Cek apakah kita di halaman displayModeAndUnit (showModeAndUnit = true)
      if (showModeAndUnit) {
        // User menekan SELECT pada "> Add Radar OBJ"
        inAddRadarMode = true;
        
        // Reset selected IDs saat masuk mode Add Radar (jika ingin start fresh)
        // selectedRadarCount = 0; // COMMENTED: Keep previous selections
        
        // Set posisi awal ke object pertama yang belum dipilih
        radarObjectPosition = 0;
        bool found = false;
        for (int i = 0; i < totalRadarPresets; i++) {
          bool isSelected = false;
          uint8_t currentId = radarPresets[i].id;
          if (currentId != 0) { // Skip check untuk reset option
            for (int j = 0; j < selectedRadarCount; j++) {
              if (selectedRadarIds[j] == currentId) {
                isSelected = true;
                break;
              }
            }
          }
          if (!isSelected) {
            radarObjectPosition = i;
            found = true;
            break;
          }
        }
        
        // Jika semua sudah dipilih, default ke reset option
        if (!found) {
          radarObjectPosition = totalRadarPresets - 1; // Reset option at the end
        }
        
        displayRotationIndex = 0; // Reset ke display pertama
        displayRotationTimer = millis();
        needsClearScreen = true; // Set flag untuk clear screen sekali
        displayNeedsUpdate = true;
        Serial.println("Masuk mode Add Radar Object");
      }
      // Cek apakah posisi cursor di Manual, Auto, atau Play
      else if (cursorPosition == 0 || cursorPosition == 1 || cursorPosition == 3) {
        // Manual (0), Auto (1), atau Play (3) - Masuk ke mode pemilihan unit
        inUnitSelectionMode = true;
        unitPosition = 0; // Reset ke unit pertama
        displayNeedsUpdate = true;
        Serial.print("Masuk mode pemilihan unit dari: ");
        if (cursorPosition == 0) Serial.println("Manual");
        else if (cursorPosition == 1) Serial.println("Auto");
        else if (cursorPosition == 3) Serial.println("Play");
      } else if (cursorPosition == 2) {
        // Rec dipilih: mulai dengan pemilihan Unit (seperti Manual mode)
        inRecUnitSelect = true;
        selectedRecUnitIndex = 0; // Default ke unit pertama
        displayNeedsUpdate = true;
        Serial.println("Masuk mode pemilihan unit untuk Rec");
      }
    }
    xSemaphoreGive(xSemaphore);
  }
}

void handleBackButton() {
  Serial.println("BACK diterima dari ESP1");
  
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    // Jika sedang dalam proses SD Init atau Failed, batalkan dan kembali ke menu
    if (inSDInitWait || inSDInitFailed) {
      inSDInitWait = false;
      inSDInitFailed = false;
      sdInitStatus = -1;
      sdInitRetryCount = 0;
      resetSDInitScreenState();
      
      // Jika ini untuk Play mode, bersihkan CAN dan reset activeUnit
      if (cursorPosition == 3) {
        if (canEnabled) {
          canEnabled = false;
          twai_stop();
          twai_driver_uninstall();
          Serial.println("Play mode: CAN stopped (SD init cancelled)");
        }
        activeUnit = -1;
        activeMode = -1;
        Serial2.println("MODE:IDLE");
    applyModeToRPMState("IDLE");
        Serial.println("Play mode: Reset to IDLE (SD init cancelled)");
      }
      // Untuk Rec mode, reset Rec states
      else if (cursorPosition == 2) {
        inRecUnitSelect = false;
        selectedRecUnitIndex = 0;
        if (canEnabled) {
          canEnabled = false;
          twai_stop();
          twai_driver_uninstall();
          Serial.println("Rec mode: CAN stopped (SD init cancelled)");
        }
        Serial2.println("SD_MONITOR_STOP");
        Serial2.println("MODE:IDLE");
    applyModeToRPMState("IDLE");
        Serial.println("Rec mode: Reset to IDLE (SD init cancelled)");
      }
      
      // Reset Play mode states if cancelling during SD init
      inPlayModeSelection = false;
      inPlayFileSelection = false;
      waitingForFileList = false;
      playModePosition = 0;
      playFilePosition = 0;
      totalPlayFiles = 0;
      selectedPlayModeAbbr = "";
      
      lcd.clear();
      delay(50);
      displayNeedsUpdate = true;
      Serial.println("BACK pressed during SD Init - returning to main menu");
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang menampilkan SD Card Missing, langsung kembali ke menu
    if (inSDCardMissing) {
      inSDCardMissing = false;
      resetSDInitScreenState();
      
      // Reset ALL recording/save states robustly
      inSaveResultScreen = false;
      inSaveConfirm = false;
      inRecordMode = false;
      inRecording = false;
      inRecModeSelection = false;
      inStopRecordingConfirm = false;
      // Reset Play states as well
      inPlayModeSelection = false;
      inPlayFileSelection = false;
      waitingForFileList = false;
      recordStartTime = 0;
      recordElapsedMs = 0;
      currentRecordingFilename = "";
      currentRecordingLabel = "";
      currentRecordingModeAbbr = "";
      cursorPosition = 0;
      recModePosition = 0;
      saveConfirmPosition = 0;
      stopConfirmPosition = 0;
      // Reset Play selections
      playModePosition = 0;
      playFilePosition = 0;
      totalPlayFiles = 0;
      selectedPlayModeAbbr = "";

      // Stop CAN and SD monitoring, ensure radar idle
      if (canEnabled) {
        canEnabled = false;
        twai_stop();
        twai_driver_uninstall();
        Serial.println("CAN stopped (BACK during SD Missing)");
      }
      Serial2.println("SD_MONITOR_STOP");
      Serial2.println("MODE:IDLE");
  applyModeToRPMState("IDLE");
      
      lcd.clear();
      delay(50);
      displayNeedsUpdate = true;
      Serial.println("BACK pressed during SD Missing - returning to main menu with full reset + SD monitor stopped + CAN stopped");
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang di dialog konfirmasi stop recording, BACK = batal (No)
    if (inStopRecordingConfirm) {
      inStopRecordingConfirm = false;
      displayNeedsUpdate = true;
      Serial.println("Stop Recording? -> Cancel via BACK (lanjut merekam)");
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang di mode Add Radar Object, kembali ke displayModeAndUnit
    if (inAddRadarMode) {
      inAddRadarMode = false;
      // Reset selected radar IDs saat keluar dari mode Add Radar
      selectedRadarCount = 0;
      // Kirim perintah RESET ke ESP1 untuk mereset target radar
      Serial2.println("RESET_RADAR");
      displayNeedsUpdate = true;
      Serial.println("Keluar dari mode Add Radar Object - selected IDs cleared and ESP1 reset sent");
      xSemaphoreGive(xSemaphore);
      return;
    }
    
    // Jika sedang menampilkan mode dan unit aktif, kembali ke mode selection (untuk Auto) atau unit selection (untuk Manual/Play)
    if (showModeAndUnit) {
      showModeAndUnit = false;

      // Saat keluar dari tampilan aktif (khususnya AUTO), paksa kirim 0 untuk speed & RPM
      // dan teruskan pengiriman 0 sampai user memilih mode baru.
      currentSpeed = 0.0f;
      currentRpm = 0.0f;
      if (activeMode == 1) { // Hanya force untuk AUTO scenario
        forceZeroOutputs = true;
      }
      // Kirim satu kali frame CAN bernilai 0 segera (jika CAN aktif)
      if (canEnabled) {
        sendZeroOutputsOnce();
      }

      // Untuk Auto mode, jangan kirim MODE:IDLE agar radar tetap aktif
      if (activeMode != 1) {
        // Kirim MODE:IDLE ke ESP1 untuk stop radar (untuk Manual/Play)
        Serial2.println("MODE:IDLE");
        applyModeToRPMState("IDLE");
        Serial.println("Sent to ESP1: MODE:IDLE (radar stopped)");
      } else {
        Serial.println("Auto mode: Radar stays active when backing to mode selection");
      }
      
      // Cek apakah ini Auto mode (yang memiliki simulation mode)
      if (activeMode == 1 && activeSimMode >= 0) {
        // Auto mode: kembali ke mode selection
        inModeSelectionMode = true;
        inUnitSelectionMode = false;
        modePosition = activeSimMode; // Restore posisi mode yang sebelumnya dipilih
        Serial.println("Kembali ke pemilihan simulation mode (Auto)");
      } else {
        // Manual/Play mode: kembali ke unit selection
        inUnitSelectionMode = true;
        inModeSelectionMode = false;
        unitPosition = activeUnit; // Restore posisi unit yang sebelumnya dipilih
        Serial.println("Kembali ke pemilihan unit (Manual/Play)");
      }
      
      // Reset active states tapi simpan posisi untuk restore
      // activeMode, activeUnit, activeSimMode tetap untuk restore posisi
      isAutoMode = false;  // Reset AUTO mode flag
      autoSimMode = -1;    // Reset AUTO simulation mode
      
      displayNeedsUpdate = true;
    }
    // Jika sedang di UI Record
    else if (inRecordMode) {
      if (inRecording) {
        // Munculkan dialog konfirmasi tanpa menghentikan perekaman
        inStopRecordingConfirm = true;
        stopConfirmPosition = 0; // Default ke No
        displayNeedsUpdate = true;
        Serial.println("BACK saat merekam -> tampilkan dialog Stop Recording?");
        xSemaphoreGive(xSemaphore);
        return;
      } else {
        // Tidak sedang merekam: kembali 1 halaman ke pemilihan Rec Mode
        inRecordMode = false;
        inRecording = false;
        recordElapsedMs = 0;
        inRecModeSelection = true;
        displayNeedsUpdate = true;
        Serial.println("Keluar dari mode Record UI (tidak merekam), kembali ke pemilihan Rec Mode");
        xSemaphoreGive(xSemaphore);
        return;
      }
    }
    // Jika sedang di pemilihan Rec Mode, kembali ke pemilihan Unit Rec
    else if (inRecModeSelection) {
      inRecModeSelection = false;
      inRecUnitSelect = true; // Kembali ke unit selection untuk Rec
      recModePosition = 0; // Reset mode position
      displayNeedsUpdate = true;
      Serial.println("Keluar dari pemilihan Rec Mode, kembali ke pemilihan Unit Rec");
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang di Simulation Completed, kembali ke pemilihan Play File
    else if (inSimulationCompleted) {
      inSimulationCompleted = false;
      // Kembali ke Play file selection
      inPlayFileSelection = true;
      waitingForFileList = true;
      
      // Reset Play UI state
      playElapsedMs = 0;
      playTotalMs = 0;
      playStartMillis = 0;
  playViewEnteredAt = 0;
  playTimerFallbackArmed = false;
      currentPlayFilename = "";
      
      // Reset file selection state
      playFilePosition = 0;
      totalPlayFiles = 0;
      // Clear file list array
      for (int i = 0; i < 20; i++) {
        playFileList[i] = "";
      }
      
      // Reset AUTO mode flags
      isAutoMode = false;
      autoSimMode = -1;
      
      // Stop radar and disable CAN when BACK from simulation completed
      if (canEnabled) {
        canEnabled = false;
        twai_stop();
        twai_driver_uninstall();
        Serial.println("CAN stopped (BACK from simulation completed)");
      }
      
      // Reset activeUnit and activeMode to stop radar
      activeUnit = -1;
      activeMode = -1;
      
      // Send IDLE command to ESP1 to stop radar
      Serial2.println("MODE:IDLE");
    applyModeToRPMState("IDLE");
      Serial.println("Radar stopped (BACK from simulation completed)");
      
      // Request fresh file list from ESP1
      String abbrev = "";
      if (playModePosition == 0) abbrev = "PC";
      else if (playModePosition == 1) abbrev = "SB";
      else if (playModePosition == 2) abbrev = "FB";
      
      if (abbrev.length() > 0) {
        playListRequestAttempts = 0;
        requestPlayFileList(abbrev);
        Serial.println("BACK from Simulation Completed -> Request file list for " + abbrev);
      }
      
      displayNeedsUpdate = true;
      Serial.println("Keluar dari Simulation Completed, kembali ke pemilihan Play File + Radar/CAN stopped");
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang di Play UI, kembali ke pemilihan Play File
    else if (inPlayView) {
      inPlayView = false;
      // Tetap di Play flow: kembali ke daftar file
      inPlayFileSelection = true;
      
      // Reset timer lokal dan flag
      playElapsedMs = 0;
      playTotalMs = 0;
      playStartMillis = 0;
      playTimerReady = false;
  playViewEnteredAt = 0;
  playTimerFallbackArmed = false;
      currentPlayFilename = "";
      
      // Reset AUTO mode flags (kembali ke mode normal)
      isAutoMode = false;
      autoSimMode = -1;
      
      // KIRIM SPEED=0 dan RPM=0 (jangan disable CAN, biarkan tetap aktif)
      currentSpeed = 0.0f;
      currentRpm = 0.0f;
      if (canEnabled) {
        sendZeroOutputsOnce(); // Kirim sekali frame CAN dengan nilai 0
        Serial.println("Sent speed=0 and RPM=0 via CAN (BACK from Play UI)");
      }

  // Pastikan ESP1 menghentikan playback dan berpindah ke mode IDLE sebelum kembali ke daftar file
  Serial2.println("PLAY_STOP");
  Serial2.flush();
  Serial2.println("MODE:IDLE");
  applyModeToRPMState("IDLE");
  Serial2.flush();
  vTaskDelay(pdMS_TO_TICKS(20)); // Give UART time to drain so ESP1 surely receives MODE:IDLE

  // Reset radar kembali ke default agar tampilan menu rapi setelah kembali dari Play UI
  Serial2.println("RESET_RADAR");
  Serial2.println("ADD_RADAR:1,0,50,20");
  Serial2.flush();
  Serial.println("Radar reset to default object after BACK from Play UI");
      
      displayNeedsUpdate = true;
      Serial.println("Keluar dari Play UI, kembali ke pemilihan Play File");
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang di pemilihan Play Mode, kembali ke menu utama (dengan cleanup)
    else if (inPlayModeSelection) {
      inPlayModeSelection = false;
      playModePosition = 0; // Reset mode position
      
      // Stop CAN if it was enabled for Play mode
      if (canEnabled) {
        canEnabled = false;
        twai_stop();
        twai_driver_uninstall();
        Serial.println("CAN stopped (exiting Play mode selection)");
      }
      
      // Reset active states
      activeMode = -1;
      activeUnit = -1;
      activeSimMode = -1;
      
      // Send MODE:IDLE and stop SD monitoring on ESP1
      Serial2.println("SD_MONITOR_STOP");
      Serial2.println("MODE:IDLE");
      applyModeToRPMState("IDLE");
      
      displayNeedsUpdate = true;
      Serial.println("Keluar dari pemilihan Play Mode, kembali ke menu utama + CAN stopped");
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang di pemilihan Play File, kembali ke pemilihan Play Mode
    else if (inPlayFileSelection) {
      inPlayFileSelection = false;
      waitingForFileList = false;
      inPlayModeSelection = true; // Kembali ke mode selection
      playFilePosition = 0; // Reset file position
      totalPlayFiles = 0; // Reset file count
      displayNeedsUpdate = true;
      Serial.println("Keluar dari pemilihan Play File, kembali ke pemilihan Play Mode");
      xSemaphoreGive(xSemaphore);
      return;
    }
    // Jika sedang di pemilihan Unit Rec, kembali ke menu utama
    else if (inRecUnitSelect) {
      // ROBUST RESET when exiting Rec unit selection
      inRecUnitSelect = false;
      inRecModeSelection = false;
      inRecordMode = false;
      inRecording = false;
      inStopRecordingConfirm = false;
      inSaveConfirm = false;
      inSaveResultScreen = false;
      recordStartTime = 0;
      recordElapsedMs = 0;
      currentRecordingFilename = "";
      currentRecordingLabel = "";
      currentRecordingModeAbbr = "";
      cursorPosition = 0;
      recModePosition = 0;
      saveConfirmPosition = 0;
      stopConfirmPosition = 0;
      selectedRecUnitIndex = 0;
      
      // Stop CAN if it was enabled for Rec mode
      if (canEnabled) {
        currentSpeed = 0.0f;
        currentRpm = 0.0f;
        sendZeroOutputsOnce();
        twai_stop();
        twai_driver_uninstall();
        canEnabled = false;
        Serial.println("CAN stopped (exiting Rec mode)");
      }
      
      displayNeedsUpdate = true;
      // Send MODE:IDLE to stop radar on ESP1
      Serial2.println("MODE:IDLE");
    applyModeToRPMState("IDLE");
      activeMode = -1;
      activeUnit = -1;
      activeSimMode = -1;
      forceZeroOutputs = false;
      Serial.println("Keluar dari pemilihan Unit Rec, kembali ke menu utama dengan full reset + radar stopped + CAN stopped");
    }
    // Cek apakah dalam mode pemilihan simulation mode
    else if (inModeSelectionMode) {
      // Kembali ke mode pemilihan unit (step back)
      inModeSelectionMode = false;
      inUnitSelectionMode = true;  // Kembali ke unit selection
      Serial.println("Keluar dari mode pemilihan simulation mode, kembali ke pemilihan unit");
      displayNeedsUpdate = true;
    }
    // Cek apakah dalam mode pemilihan unit
    else if (inUnitSelectionMode) {
      // Kembali ke menu utama (step back)
      inUnitSelectionMode = false;
      // Saat benar-benar kembali ke menu utama, hentikan CAN agar tidak mengirim terus
      if (canEnabled) {
        currentSpeed = 0.0f;
        currentRpm = 0.0f;
        sendZeroOutputsOnce();
        twai_stop();
        twai_driver_uninstall();
        canEnabled = false;
        Serial.println("CAN Bus stopped and disabled (back to main menu)");
      }
      Serial2.println("MODE:IDLE");
      applyModeToRPMState("IDLE");
      Serial.println("Main menu: MODE:IDLE sent (Manual/Auto/Rec -> Idle)");
      forceZeroOutputs = false; // Clear forcing when leaving active/session context
      // Reset active states when going back to main menu
      activeMode = -1;
      activeUnit = -1;
      activeSimMode = -1;
      Serial.println("Keluar dari mode pemilihan unit, kembali ke menu utama");
      displayNeedsUpdate = true;
    }
    xSemaphoreGive(xSemaphore);
  }
}

void displayMenu() {
  // Cek apakah sedang dalam mode Add Radar Object
  // Jangan clear screen untuk running text mode (akan flicker)
  if (inStopRecordingConfirm) {
    displayStopRecordingConfirm();
    return;
  }
  
  // Cek layar SD Card Missing (prioritas tertinggi karena error critical)
  if (inSDCardMissing) {
    // Tampilkan layar missing, dan cek timeout 3 detik untuk kembali ke menu utama
    displaySDCardMissing();
    unsigned long now = millis();
    if (now - sdCardMissingStartTime >= 3000UL) {
      // Reset state dan kembali ke menu utama dengan clear screen
      inSDCardMissing = false;
      resetSDInitScreenState(); // Reset display state variables including missing
      
      // Reset ALL recording/save states robustly when SD missing timeout
      inSaveResultScreen = false;
      inSaveConfirm = false;
      inRecordMode = false;
      inRecording = false;
      inRecModeSelection = false;
      inStopRecordingConfirm = false;
      // Reset Play states as well
      inPlayModeSelection = false;
      inPlayFileSelection = false;
      waitingForFileList = false;
      showModeAndUnit = false;
      recordStartTime = 0;
      recordElapsedMs = 0;
      currentRecordingFilename = "";
      currentRecordingLabel = "";
      currentRecordingModeAbbr = "";
      cursorPosition = 0;
      recModePosition = 0;
      saveConfirmPosition = 0;
      stopConfirmPosition = 0;
      selectedRecUnitIndex = 0;
      // Reset Play selections
      playModePosition = 0;
      playFilePosition = 0;
      totalPlayFiles = 0;
      selectedPlayModeAbbr = "";
      playViewEnteredAt = 0;
      playTimerFallbackArmed = false;
      
      // Stop CAN if it was enabled for Rec mode
      if (canEnabled) {
        canEnabled = false;
        twai_stop();
        twai_driver_uninstall();
        Serial.println("CAN stopped (SD missing)");
      }
      
      lcd.clear(); // Clear screen sebelum kembali ke menu
      delay(50); // Delay kecil untuk I2C stabilization
      // Stop SD monitoring (sudah tidak ada SD card) and stop radar
      Serial2.println("SD_MONITOR_STOP");
      Serial2.println("MODE:IDLE");
      applyModeToRPMState("IDLE");
      displayNeedsUpdate = true;
    Serial.println("Returning to main menu after SD card missing timeout + full reset + SD monitoring stopped + radar stopped + CAN stopped");
      // Kembali ke menu utama
      return;
    }
    return; // Penting: return di sini agar tidak render menu di bawah
  }

  // Save confirmation dialog after stopping recording (lower priority than SD Missing)
  if (inSaveConfirm) {
    displaySaveConfirm();
    return;
  }
  // Save result screen (3 seconds)
  if (inSaveResultScreen) {
    displaySaveResult();
    unsigned long now = millis();
    if (now - saveResultStartTime >= 3000UL) {
      // Reset ALL recording and save states - return to main menu
      inSaveResultScreen = false;
      inSaveConfirm = false;
      inRecordMode = false;
      inRecording = false;
      inRecModeSelection = false;
      inRecUnitSelect = false;
      inStopRecordingConfirm = false;
      
      // Clear recording data
      recordStartTime = 0;
      recordElapsedMs = 0;
      currentRecordingFilename = "";
      currentRecordingLabel = "";
      currentRecordingModeAbbr = "";
      
      // Reset positions to defaults
      cursorPosition = 0;  // Back to Manual
      recModePosition = 0; // Reset to first rec mode
      saveConfirmPosition = 0;
      stopConfirmPosition = 0;
      selectedRecUnitIndex = 0;
      playViewEnteredAt = 0;
      playTimerFallbackArmed = false;
      
      // Stop CAN if it was enabled for Rec mode
      if (canEnabled) {
        canEnabled = false;
        twai_stop();
        twai_driver_uninstall();
        Serial.println("CAN stopped (save result timeout)");
      }
      
      // Stop SD monitoring and radar since we're leaving Rec flow entirely
      Serial2.println("SD_MONITOR_STOP");
      Serial2.println("MODE:IDLE");
      applyModeToRPMState("IDLE");
      
      // Force LCD clear and update
      lcd.clear();
      delay(50);
      displayNeedsUpdate = true;
      
      Serial.println("Save result timeout -> Reset to main menu + SD monitoring stopped + radar stopped + CAN stopped");
    }
    return;
  }
  
  // Cek layar SD Init wait/fail (prioritas tinggi)
  if (inSDInitFailed) {
    // Tampilkan layar gagal, dan cek timeout 3 detik untuk kembali ke menu utama
    displaySDInitFailed();
    unsigned long now = millis();
    if (now - sdInitFailedStartTime >= 3000UL) {
      // Reset state dan kembali ke menu utama dengan clear screen
      inSDInitFailed = false;
      inSDInitWait = false;
      sdInitStatus = -1;
      sdInitRetryCount = 0;
      resetSDInitScreenState(); // Reset display state variables
      
      // Jika ini untuk Play mode, bersihkan CAN dan reset activeUnit
      if (cursorPosition == 3) {
        if (canEnabled) {
          canEnabled = false;
          twai_stop();
          twai_driver_uninstall();
          Serial.println("Play mode: CAN stopped (SD init failed)");
        }
        activeUnit = -1;
        activeMode = -1;
        Serial2.println("SD_MONITOR_STOP");
        Serial2.println("MODE:IDLE");
    applyModeToRPMState("IDLE");
        Serial.println("Play mode: Reset to IDLE (SD init failed)");
        
        // Reset Play mode states
        inPlayModeSelection = false;
        inPlayFileSelection = false;
        waitingForFileList = false;
        playModePosition = 0;
        playFilePosition = 0;
        totalPlayFiles = 0;
        selectedPlayModeAbbr = "";
      }
      
      lcd.clear(); // Clear screen sebelum kembali ke menu
      delay(50); // Delay kecil untuk I2C stabilization
      displayNeedsUpdate = true;
      Serial.println("Returning to main menu after SD Init failed");
      // Kembali ke menu utama: tidak perlu set flag tambahan, cukup biarkan menu default
      return;
    }
    return; // Penting: return di sini agar tidak render menu di bawah
  }
  if (inSDInitWait) {
    // Tampilkan layar inisialisasi SD, handle retry mechanism
    displaySDInitScreen();
    unsigned long now = millis();
    unsigned long elapsed = now - sdInitStartTime;
    
    // Retry mechanism: retry setiap 2 detik jika tidak ada respon OK (max 5 retry dalam 10 detik)
    // Jika menerima FAIL, reset status ke pending untuk retry berikutnya
    if (sdInitStatus == 0 && sdInitRetryCount < 5 && elapsed < 10000UL) {
      // SD_FAIL diterima tapi masih ada kesempatan retry
      sdInitStatus = -1; // Reset ke pending untuk retry berikutnya
      Serial.printf("SD_FAIL received, will retry (attempt %d/5)\n", sdInitRetryCount + 1);
    }
    
    if (sdInitStatus == -1 && (now - lastSDInitRequestTime) >= 2000UL && sdInitRetryCount < 5 && elapsed < 10000UL) {
      sdInitRetryCount++;
      lastSDInitRequestTime = now;
      Serial.printf("SD Init retry %d/5...\n", sdInitRetryCount);
      Serial2.println("SD_INIT");
    }
    
    // Jika OK diterima dan minimal 1 detik sudah berlalu -> lanjut ke pemilihan Play/Rec Mode
    if (sdInitStatus == 1 && elapsed >= 1000UL) {
      inSDInitWait = false;
      sdInitStatus = -1;
      sdInitRetryCount = 0; // Reset retry counter
      lcd.clear(); // Clear screen before transition
      delay(50); // I2C stabilization
      
      // Cek apakah ini untuk mode Play atau Rec
      if (cursorPosition == 3) {
        // Mode Play - masuk ke Play Mode Selection (PC/SB/FB)
        inPlayModeSelection = true;
        playModePosition = 0;
        displayNeedsUpdate = true;
        // Mulai monitoring SD card untuk deteksi SD_MISSING selama Play flow
        Serial2.println("SD_MONITOR_START");
        Serial.println("Play mode: SD Init SUCCESS -> Play Mode Selection + SD monitoring started");

        // Aktifkan radar CAN di ESP1 dan set default target (ID=1, vl=0, dist=50, rcs=20)
        if (activeUnit >= 0) {
          Serial2.println("MODE:PLAY");
          applyModeToRPMState("PLAY");
          Serial2.println("UNIT:" + String(activeUnit));
          Serial2.println("RESET_RADAR");
          Serial2.println("ADD_RADAR:1,0,50,20");
          Serial.println("[Play] Radar CAN primed: MODE:PLAY, UNIT, RESET_RADAR, ADD_RADAR:1,0,50,20");
        }
      } else {
        // Mode Rec - masuk ke Rec Mode Selection
        inRecModeSelection = true;
        recModePosition = 0;
        displayNeedsUpdate = true;
        // Instruksikan ESP1 untuk mulai monitoring SD card
        Serial2.println("SD_MONITOR_START");
        Serial.println("Rec mode: SD Init SUCCESS -> Rec Mode Selection + SD monitoring started");
      }
    } else if (elapsed >= 10000UL || (sdInitStatus == 0 && sdInitRetryCount >= 5)) {
      // Timeout 10 detik atau gagal setelah 5 kali retry
      inSDInitWait = false;
      lcd.clear(); // Clear screen before showing failed message
      delay(50); // I2C stabilization
      inSDInitFailed = true;
      int totalAttempts = sdInitRetryCount;
      sdInitRetryCount = 0; // Reset retry counter
      sdInitFailedStartTime = now;
      displayNeedsUpdate = true;
      Serial.printf("SD Init FAILED after %d attempts\n", totalAttempts);
    }
    return;
  }
  if (inAddRadarMode) {
    displayRadarObjectSelection();
    return;
  }
  // Cek pemilihan Unit untuk Rec
  if (inRecUnitSelect) {
    displayRecUnitSelection();
    return;
  }
  // Cek pemilihan Rec Mode
  if (inRecModeSelection) {
    displayRecModeSelection();
    return;
  }
  // Cek pemilihan Play Mode
  if (inPlayModeSelection) {
    displayPlayModeSelection();
    return;
  }
  // Cek pemilihan Play File
  if (inPlayFileSelection) {
    displayPlayFileSelection();
    return;
  }
  // Cek UI untuk Play (setelah file dipilih)
  if (inPlayView) {
    displayPlayScreen();
    return;
  }
  // Cek UI untuk Simulation Completed (setelah CSV selesai)
  if (inSimulationCompleted) {
    displaySimulationCompleted();
    return;
  }
  // Cek UI untuk Record
  if (inRecordMode) {
    displayRecordScreen();
    return;
  }
  
  // Bersihkan layar untuk mode lain
  lcd.clear();
  
  // Cek apakah sedang menampilkan mode dan unit yang aktif
  if (showModeAndUnit) {
    displayModeAndUnit();
    return;
  }
  
  // Cek apakah dalam mode pemilihan simulation mode
  if (inModeSelectionMode) {
    displayModeSelection();
    return;
  }
  
  // Cek apakah dalam mode pemilihan unit
  if (inUnitSelectionMode) {
    displayUnitSelection();
    return;
  }
  
  // Tampilkan menu berdasarkan posisi cursor
  switch (cursorPosition) {
    case 0: // Manual
      goto manual_selected;
    case 1: // Auto
      goto auto_selected;
    case 2: // Rec
      goto rec_selected;
    case 3: // Play
      goto play_selected;
  }
  
  manual_selected:
    lcd.setCursor(0, 0);
    lcd.print("> Manual   Rec");
    lcd.setCursor(0, 1);
    lcd.print("  Auto     Play");
    return;
    
  auto_selected:
    lcd.setCursor(0, 0);
    lcd.print("  Manual   Rec");
    lcd.setCursor(0, 1);
    lcd.print("> Auto     Play");
    return;
    
  rec_selected:
    lcd.setCursor(0, 0);
    lcd.print("  Manual > Rec");
    lcd.setCursor(0, 1);
    lcd.print("  Auto     Play");
    return;
    
  play_selected:
    lcd.setCursor(0, 0);
    lcd.print("  Manual   Rec");
    lcd.setCursor(0, 1);
    lcd.print("  Auto   > Play");
    return;
}

// UI untuk Record mode: tampilkan header dengan counter dan info mode/unit
void displayRecordScreen() {
  // Hitung waktu yang ditampilkan
  unsigned long elapsed = inRecording ? (millis() - recordStartTime) : recordElapsedMs;
  // Konversi ke Menit:Detik:Milidetik(2-digit centiseconds untuk pas 8 char)
  unsigned int minutes = (elapsed / 60000UL) % 100; // 0-99
  unsigned int seconds = (elapsed / 1000UL) % 60;   // 0-59
  unsigned int centis  = (elapsed / 10UL) % 100;    // 0-99 (centiseconds)

  char timeBuf[9]; // "MM:SS:CC" + null
  snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u:%02u", minutes, seconds, centis);

  // Baris pertama: "RECORD: 00:00:00"
  lcd.setCursor(0, 0);
  String top = String("RECORD: ") + String(timeBuf);
  lcd.print(top);
  for (int i = top.length(); i < 16; i++) lcd.print(" ");

  // Baris kedua: Mode (PC/SB/FB) + Unit ID + START/STOP button
  // Format: "PC U:02 > START" atau "PC U:02 > STOP "
  lcd.setCursor(0, 1);
  char line2[17];
  
  // Get mode abbreviation
  const char* modeAbbr = (recModePosition == 0) ? "PC" : (recModePosition == 1) ? "SB" : "FB";
  String buttonText = inRecording ? "> STOP " : "> START"; // 7 chars each
  
  // Format: "PC U:02 > START" (2 + 1 + 4 + 1 + 7 = 15 chars, fits in 16)
  snprintf(line2, sizeof(line2), "%s U:%02d %s", 
           modeAbbr, selectedRecUnitIndex, buttonText.c_str());
  lcd.print(line2);
  // Pad remaining
  for (int i = strlen(line2); i < 16; i++) lcd.print(" ");

  // Log untuk debug
  Serial.printf("Display Record UI: %s, Mode:%s Unit:%02d %s\n", 
                top.c_str(), modeAbbr, selectedRecUnitIndex, buttonText.c_str());
}

// UI: Save confirmation after stopping record
void displaySaveConfirm() {
  // Line 1: "Save XX-YY ?"
  lcd.setCursor(0, 0);
  String name = (currentRecordingLabel.length() > 0) ? currentRecordingLabel : 
                (currentRecordingModeAbbr.length() > 0) ? (currentRecordingModeAbbr + "-??") : "FILE";
  String line1 = String("Save ") + name + String(" ?");
  if (line1.length() > 16) line1 = line1.substring(0, 16);
  lcd.print(line1);
  for (int i = line1.length(); i < 16; i++) lcd.print(" ");

  // Line 2: selection
  lcd.setCursor(0, 1);
  if (saveConfirmPosition == 0) {
    lcd.print("> Yes     No    ");
  } else {
    lcd.print("  Yes   > No    ");
  }
}

// UI: Save result (Saved/Not Saved) for 3 seconds
void displaySaveResult() {
  // Render-once per result session (based on saveResultStartTime)
  static unsigned long lastRenderStart = 0;
  if (lastRenderStart != saveResultStartTime) {
    lastRenderStart = saveResultStartTime;
    lcd.clear();
    delay(10);
    lcd.setCursor(0, 0);
    String name = (currentRecordingLabel.length() > 0) ? currentRecordingLabel : 
                  (currentRecordingModeAbbr.length() > 0) ? (currentRecordingModeAbbr + "-??") : "FILE";
    String top = name + (lastSaveResultSaved ? String(" Saved") : String(" Not"));
    if (top.length() > 16) top = top.substring(0, 16);
    lcd.print(top);
    for (int i = top.length(); i < 16; i++) lcd.print(" ");
    lcd.setCursor(0, 1);
    String bottom = lastSaveResultSaved ? String("Succesfully") : String("Saved");
    if (bottom.length() > 16) bottom = bottom.substring(0, 16);
    lcd.print(bottom);
    for (int i = bottom.length(); i < 16; i++) lcd.print(" ");
    
    Serial.printf("[UI] Save result displayed: %s\n", name.c_str());
  }
}

void displayUnitSelection() {
  // Baris pertama: "Select Unit : XX" dengan nomor urut
  lcd.setCursor(0, 0);
  lcd.print("Select Unit : ");
  
  // Tampilkan nomor unit dengan format 2 digit (00-17)
  if (unitPosition < 10) {
    lcd.print("0"); // Tambah leading zero
  }
  lcd.print(unitPosition);
  
  // Baris kedua: Tampilkan unit yang dipilih dengan cursor '>'
  lcd.setCursor(0, 1);
  
  // Ambil nama unit dan potong jika terlalu panjang (max 14 karakter karena ada '> ')
  String unitName = String(truckUnits[unitPosition]);
  if (unitName.length() > 14) {
    unitName = unitName.substring(0, 14);
  }
  
  // Cetak dengan format "> UNIT_NAME" dan padding
  lcd.print("> ");
  lcd.print(unitName);
  
  // Clear sisa karakter di baris kedua
  int remainingSpace = 14 - unitName.length();
  for (int i = 0; i < remainingSpace; i++) {
    lcd.print(" ");
  }
  
  // Tampilkan info unit di Serial Monitor
  Serial.print("Unit [");
  if (unitPosition < 10) Serial.print("0");
  Serial.print(unitPosition);
  Serial.print("]: ");
  Serial.println(truckUnits[unitPosition]);
}

void displayModeSelection() {
  // Baris pertama: "Mode :"
  lcd.setCursor(0, 0);
  lcd.print("Mode :          ");
  
  // Baris kedua: Tampilkan mode yang dipilih dengan cursor '>'
  lcd.setCursor(0, 1);
  
  // Ambil nama mode dan potong jika terlalu panjang (max 14 karakter karena ada '> ')
  String modeName = String(simulationModes[modePosition]);
  if (modeName.length() > 14) {
    modeName = modeName.substring(0, 14);
  }
  
  // Cetak dengan format "> MODE_NAME" dan padding
  lcd.print("> ");
  lcd.print(modeName);
  
  // Clear sisa karakter di baris kedua
  int remainingSpace = 14 - modeName.length();
  for (int i = 0; i < remainingSpace; i++) {
    lcd.print(" ");
  }
  
  // Tampilkan info mode di Serial Monitor
  Serial.print("Mode [");
  Serial.print(modePosition);
  Serial.print("]: ");
  Serial.println(simulationModes[modePosition]);
}

// UI pemilihan Unit untuk Rec (sama seperti manual mode)
void displayRecUnitSelection() {
  // Baris pertama: "Simulated Unit :" (16 chars max, tepat)
  lcd.setCursor(0, 0);
  lcd.print("Simulated Unit :");
  
  // Baris kedua: Tampilkan unit yang dipilih dengan cursor '>'
  lcd.setCursor(0, 1);
  
  // Ambil nama unit dan potong jika terlalu panjang (max 14 karakter karena ada '> ')
  String unitName = String(truckUnits[selectedRecUnitIndex]);
  if (unitName.length() > 14) {
    unitName = unitName.substring(0, 14);
  }
  
  // Cetak dengan format "> UNIT_NAME" dan padding
  lcd.print("> ");
  lcd.print(unitName);
  
  // Clear sisa karakter di baris kedua
  int remainingSpace = 14 - unitName.length();
  for (int i = 0; i < remainingSpace; i++) {
    lcd.print(" ");
  }
  
  // Tampilkan info unit di Serial Monitor
  Serial.printf("Rec Unit [%d]: %s\n", selectedRecUnitIndex, truckUnits[selectedRecUnitIndex]);
}

// UI pemilihan Rec Mode (mirip Auto mode selection)
void displayRecModeSelection() {
  // Baris pertama: "Rec Mode :"
  lcd.setCursor(0, 0);
  lcd.print("Rec Mode :      ");

  // Baris kedua: tampilkan mode dengan cursor '>'
  lcd.setCursor(0, 1);
  String modeName = String(recModes[recModePosition]);
  if (modeName.length() > 14) modeName = modeName.substring(0, 14);
  lcd.print("> ");
  lcd.print(modeName);
  for (int i = 0; i < (14 - modeName.length()); i++) lcd.print(" ");

  // Debug info
  Serial.printf("Rec Mode [%d]: %s\n", recModePosition, recModes[recModePosition]);
}

// UI pemilihan Play Mode (sama dengan Rec Mode)
void displayPlayModeSelection() {
  // Baris pertama: "Play Mode :"
  lcd.setCursor(0, 0);
  lcd.print("Play Mode :     ");

  // Baris kedua: tampilkan mode dengan cursor '>'
  lcd.setCursor(0, 1);
  String modeName = String(playModes[playModePosition]);
  if (modeName.length() > 14) modeName = modeName.substring(0, 14);
  lcd.print("> ");
  lcd.print(modeName);
  for (int i = 0; i < (14 - modeName.length()); i++) lcd.print(" ");

  // Debug info
  Serial.printf("Play Mode [%d]: %s\n", playModePosition, playModes[playModePosition]);
}

// UI pemilihan Play File
void displayPlayFileSelection() {
  // Baris pertama: "Play > XX-YY.csv" atau loading message
  lcd.setCursor(0, 0);
  unsigned long now = millis();
  if (waitingForFileList && selectedPlayModeAbbr.length() > 0) {
    if (playListRequestAttempts == 0) {
      requestPlayFileList(selectedPlayModeAbbr);
    } else if (now - lastPlayListRequestMs >= 1000UL) {
      if (playListRequestAttempts < PLAY_LIST_MAX_ATTEMPTS) {
        requestPlayFileList(selectedPlayModeAbbr);
      } else {
        waitingForFileList = false;
        totalPlayFiles = 0;
        playListRequestAttempts = 0;
        lastPlayListRequestMs = 0;
        Serial.println("[Play] File list request timed out - showing empty list");
      }
    }
  }
  
  if (waitingForFileList) {
    // Menunggu response dari ESP1
    lcd.print("Loading files...");
  } else if (totalPlayFiles == 0) {
    // Tidak ada file ditemukan
    lcd.print("No ");
    lcd.print(selectedPlayModeAbbr);
    lcd.print(" files     ");
  } else {
    // Tampilkan file yang dipilih dengan format "Play > XX-YY.csv"
    String selectedFile = playFileList[playFilePosition];
    lcd.print("Play > ");
    
    // Truncate filename jika terlalu panjang (max 9 karakter setelah "Play > ")
    if (selectedFile.length() > 9) {
      lcd.print(selectedFile.substring(0, 9));
    } else {
      lcd.print(selectedFile);
      // Pad dengan spasi sampai 16 karakter total
      for (int i = 7 + selectedFile.length(); i < 16; i++) {
        lcd.print(" ");
      }
    }
  }

  // Baris kedua: tampilkan file lain dengan alignment tepat di bawah file pertama
  lcd.setCursor(0, 1);
  if (waitingForFileList) {
    lcd.print("Please wait...  ");
  } else if (totalPlayFiles == 0) {
    lcd.print("Press BACK      ");
  } else if (totalPlayFiles > 1) {
    // Tampilkan file berikutnya dengan spasi yang sama seperti baris pertama
    int nextIndex = (playFilePosition + 1) % totalPlayFiles;
    String nextFile = playFileList[nextIndex];
    lcd.print("       "); // 7 spasi untuk align dengan file di atas (setelah "Play > ")
    
    if (nextFile.length() > 9) {
      lcd.print(nextFile.substring(0, 9));
    } else {
      lcd.print(nextFile);
      // Pad dengan spasi sampai 16 karakter total
      for (int i = 7 + nextFile.length(); i < 16; i++) {
        lcd.print(" ");
      }
    }
  } else {
    // Hanya 1 file
    lcd.print("                ");
  }

  // Debug info
  if (!waitingForFileList && totalPlayFiles > 0) {
    Serial.printf("Play File [%d/%d]: %s\n", playFilePosition + 1, totalPlayFiles, playFileList[playFilePosition].c_str());
  }
}

// UI Play: tampilkan Unit, timer mundur, dan nama file
void displayPlayScreen() {
  unsigned long now = millis();
  if (playViewEnteredAt == 0) {
    playViewEnteredAt = now;
  }

  bool timerReadyLocal = playTimerReady;
  unsigned long startLocal = playStartMillis;
  unsigned long totalMsLocal = playTotalMs;
  unsigned long elapsedSyncLocal = playElapsedMs;
  bool fallbackArmedLocal = playTimerFallbackArmed;

  bool armedFromSync = false;
  bool forcedFallback = false;

  if (!timerReadyLocal) {
    if (elapsedSyncLocal > 0) {
      unsigned long newStart = (elapsedSyncLocal >= now) ? now : (now - elapsedSyncLocal);
      playStartMillis = newStart;
      startLocal = newStart;
      playTimerReady = true;
      timerReadyLocal = true;
      playTimerFallbackArmed = false;
      armedFromSync = true;
    } else if (startLocal > 0 && totalMsLocal > 0) {
      playTimerReady = true;
      timerReadyLocal = true;
      playTimerFallbackArmed = false;
      armedFromSync = true;
    } else if (totalMsLocal > 0 && !fallbackArmedLocal && (now - playViewEnteredAt) >= 2000UL) {
      playStartMillis = now;
      startLocal = now;
      playTimerReady = true;
      timerReadyLocal = true;
      playTimerFallbackArmed = true;
      forcedFallback = true;
    }
  } else if (startLocal == 0 && elapsedSyncLocal > 0) {
    unsigned long newStart = (elapsedSyncLocal >= now) ? now : (now - elapsedSyncLocal);
    playStartMillis = newStart;
    startLocal = newStart;
  }

  unsigned long elapsedLocal = 0;
  if (timerReadyLocal && startLocal > 0) {
    elapsedLocal = now - startLocal;
  }
  if (elapsedSyncLocal > elapsedLocal) {
    elapsedLocal = elapsedSyncLocal;
  }

  bool hasDuration = (totalMsLocal > 0);
  bool timerValid = timerReadyLocal && hasDuration;

  unsigned long displayMs = 0;
  if (timerValid) {
    if (hasDuration) {
      long remaining = (long)totalMsLocal - (long)elapsedLocal;
      if (remaining < 0) {
        remaining = 0;
      }
      displayMs = (unsigned long)remaining;
    } else {
      displayMs = elapsedLocal;
    }
  }

  if (timerValid && hasDuration && displayMs == 0 && !playCompletionHandled) {
    Serial.println("[Play Timer] Countdown hit zero without PLAY_END - forcing completion");
    enterSimulationCompletedLocked("TIMER_ZERO_FALLBACK");
    Serial2.println("PLAY_STOP");
    Serial2.flush();
  }

  char timeBuf[9];
  if (timerValid) {
    unsigned int minutes = (displayMs / 60000UL) % 100;
    unsigned int seconds = (displayMs / 1000UL) % 60;
    unsigned int centis  = (displayMs / 10UL) % 100;
    snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u:%02u", minutes, seconds, centis);
  } else {
    snprintf(timeBuf, sizeof(timeBuf), "--:--:--");
  }

  lcd.setCursor(0, 0);
  char line1[17];
  int unitToShow = (activeUnit >= 0) ? activeUnit : unitPosition;
  snprintf(line1, sizeof(line1), "U : %02d  %s", unitToShow, timeBuf);
  lcd.print(line1);
  for (int i = strlen(line1); i < 16; i++) {
    lcd.print(" ");
  }

  lcd.setCursor(0, 1);
  String fname = currentPlayFilename;
  if (fname.length() > 16) {
    fname = fname.substring(0, 16);
  }
  lcd.print(fname);
  for (int i = fname.length(); i < 16; i++) {
    lcd.print(" ");
  }

  static unsigned long lastLog = 0;
  unsigned long logNow = millis();
  if (armedFromSync) {
    Serial.println("[Play Timer] Armed from sync data");
  }
  if (forcedFallback) {
    Serial.println("[Play Timer] Fallback armed after 2s without PLAY_READY");
  }
  if (logNow - lastLog > 1000UL) {
    lastLog = logNow;
    if (timerValid) {
      Serial.printf("[Play UI] U:%02d Remaining:%s File:%s (total=%lu, elapsed=%lu)\n",
                    unitToShow, timeBuf, currentPlayFilename.c_str(), totalMsLocal, elapsedLocal);
    } else {
      Serial.printf("[Play UI] U:%02d Waiting... (ready=%d,start=%lu,total=%lu,elapsed=%lu)\n",
                    unitToShow, timerReadyLocal, startLocal, totalMsLocal, elapsedSyncLocal);
    }
  }
}

// Dialog konfirmasi berhenti merekam
void displayStopRecordingConfirm() {
  // Baris pertama
  lcd.setCursor(0, 0);
  // Sesuai permintaan: uppercase dan spasi sebelum '?'
  lcd.print("STOP RECORDING ?");

  // Baris kedua dengan opsi No/Yes
  lcd.setCursor(0, 1);
  String bottom;
  if (stopConfirmPosition == 0) {
    // Default: No terpilih
    bottom = "> NO      YES";
  } else {
    // Yes terpilih
    bottom = "  NO    > YES";
  }
  lcd.print(bottom);
  for (int i = bottom.length(); i < 16; i++) lcd.print(" ");
}

// Layar inisialisasi SD: "SDCard" dan "Initialization..." dengan animasi loading
void displaySDInitScreen() {
  // Static variables untuk animasi
  static unsigned long lastAnimUpdate = 0;
  static int dotCount = 0; // 0-3 dots
  
  // Render header hanya sekali
  if (!sdInitScreenRendered) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SDCard");
    for (int i = 6; i < 16; i++) lcd.print(" ");
    sdInitScreenRendered = true;
  }
  
  // Update animasi setiap 500ms
  unsigned long now = millis();
  if (now - lastAnimUpdate >= 500) {
    lastAnimUpdate = now;
    dotCount = (dotCount + 1) % 4; // Cycle: 0, 1, 2, 3, 0, ...
    
    // Update baris kedua dengan animasi
    lcd.setCursor(0, 1);
    lcd.print("Initialization");
    
    // Tambahkan titik-titik sesuai dotCount (0-3 dots)
    for (int i = 0; i < dotCount; i++) {
      lcd.print(".");
    }
    
    // Clear sisa karakter (total 16 - "Initialization" (14) - dots)
    for (int i = 14 + dotCount; i < 16; i++) {
      lcd.print(" ");
    }
  }
}

// Reset function untuk dipanggil saat keluar dari SD init screen
void resetSDInitScreenState() {
  sdInitScreenRendered = false;
  lastDisplayedRetryCount = -1;
  sdFailScreenRendered = false;
  lastFailScreenTime = 0;
  sdMissingScreenRendered = false;
  lastMissingScreenTime = 0;
}

// Layar gagal SD: "Initialitation" dan "Failed!!!" (render once)
void displaySDInitFailed() {
  // Reset flag jika ini screen baru (lebih dari 5 detik dari terakhir)
  unsigned long now = millis();
  if (now - lastFailScreenTime > 5000UL) {
    sdFailScreenRendered = false;
  }
  
  if (!sdFailScreenRendered) {
    lcd.clear(); // Clear screen sekali di awal
    
    lcd.setCursor(0, 0);
    String top = "Initialitation"; // Mengikuti ejaan yang diminta
    if (top.length() > 16) top = top.substring(0, 16);
    lcd.print(top);
    for (int i = top.length(); i < 16; i++) lcd.print(" ");

    lcd.setCursor(0, 1);
    String bottom = "Failed!!!";
    lcd.print(bottom);
    for (int i = bottom.length(); i < 16; i++) lcd.print(" ");
    
    sdFailScreenRendered = true;
    lastFailScreenTime = now;
    Serial.println("SD Init Failed screen rendered");
  }
}

// Layar SD card missing: "SDCard" dan "Missing!!!" (render once)
void displaySDCardMissing() {
  // Reset flag jika ini screen baru (lebih dari 5 detik dari terakhir)
  unsigned long now = millis();
  if (now - lastMissingScreenTime > 5000UL) {
    sdMissingScreenRendered = false;
  }
  
  if (!sdMissingScreenRendered) {
    // FORCE clear screen untuk memastikan transisi bersih dari layar recording
    lcd.clear();
    delay(10); // Small delay untuk I2C stability
    
    lcd.setCursor(0, 0);
    String top = "SDCard";
    lcd.print(top);
    for (int i = top.length(); i < 16; i++) lcd.print(" ");

    lcd.setCursor(0, 1);
    String bottom = "Missing!!!";
    lcd.print(bottom);
    for (int i = bottom.length(); i < 16; i++) lcd.print(" ");
    
    sdMissingScreenRendered = true;
    lastMissingScreenTime = now;
    Serial.println(">>> SD Card Missing screen RENDERED <<<");
  }
}

// Layar Simulation Completed: "Simulation" dan "Completed !" (static display for 3 seconds)
void displaySimulationCompleted() {
  static bool screenRendered = false;
  static unsigned long lastZeroSend = 0;
  
  if (!screenRendered) {
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("Simulation");
    for (int i = 10; i < 16; i++) lcd.print(" ");

    lcd.setCursor(0, 1);
    lcd.print("Completed !");
    for (int i = 11; i < 16; i++) lcd.print(" ");
    
    screenRendered = true;
    Serial.println("Simulation Completed screen rendered");
  }

  unsigned long now = millis();
  if (now - lastZeroSend >= 500UL) {
    sendZeroOutputsOnce();
    lastZeroSend = now;
  }
  
  // Check if 3 seconds have passed, then return to file selection
  if (now - simulationCompletedStartMs >= 3000UL) {
    // Reset screen render flag for next time
    screenRendered = false;
    lastZeroSend = 0;
    
    // Reset to file selection screen
    inSimulationCompleted = false;
    inPlayFileSelection = true;
    waitingForFileList = true; // Request fresh file list
    playListRequestAttempts = 0;
    
    // Reset ALL Play UI state (termasuk flag timer baru)
    playElapsedMs = 0;
    playTotalMs = 0;
    playStartMillis = 0;
    playTimerReady = false; // Reset flag timer ready
    currentPlayFilename = "";
    
    // Reset file selection state
    playFilePosition = 0;
    totalPlayFiles = 0;
    // Clear file list array
    for (int i = 0; i < 20; i++) {
      playFileList[i] = "";
    }
    
    // Request fresh file list from ESP1
    String abbrev = "";
    if (playModePosition == 0) abbrev = "PC";
    else if (playModePosition == 1) abbrev = "SB";
    else if (playModePosition == 2) abbrev = "FB";
    
    if (abbrev.length() > 0) {
      requestPlayFileList(abbrev);
    }
    
    // Keep radar active but send speed=0 during simulation completed
    // CAN remains enabled, radar object stays active
    
    // Reset speed and RPM to 0
    currentSpeed = 0.0f;
    currentRpm = 0.0f;
    
    // Clear screen and update display
    lcd.clear();
    delay(50);
    displayNeedsUpdate = true;
    
    Serial.println("Simulation Completed timeout -> Return to Play File Selection + Radar stays active with speed=0");
  }
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning I2C bus...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.println("I2C scan complete.");
  }
  Serial.println();
}

// Fungsi untuk menampilkan mode dan unit yang aktif
void displayModeAndUnit() {
  // Baris pertama: Mode name dengan unit number
  lcd.setCursor(0, 0);
  String modeName = "";
  if (activeMode == 0) modeName = "Manual";
  else if (activeMode == 1) modeName = "Auto";
  else if (activeMode == 3) modeName = "Play";
  
  // Untuk Auto mode, tampilkan simulation mode jika ada
  if (activeMode == 1 && activeSimMode >= 0) {
    String simMode = String(simulationModes[activeSimMode]);
    if (simMode.length() > 13) simMode = simMode.substring(0, 13); // Leave space for " : 00"
    modeName = simMode;
  }
  
  // Format: "Mode : 00" (unit number with leading zero)
  String unitNumber = "";
  if (activeUnit < 10) unitNumber = "0" + String(activeUnit);
  else unitNumber = String(activeUnit);
  
  String line1 = modeName + " : " + unitNumber;
  lcd.print(line1);
  
  // Padding untuk clear sisa karakter di baris pertama
  for (int i = line1.length(); i < 16; i++) {
    lcd.print(" ");
  }
  
  // Baris kedua: "> Add Radar OBJ"
  lcd.setCursor(0, 1);
  lcd.print("> Add Radar OBJ ");
  
  // Tampilkan info di Serial Monitor
  if (activeMode == 1 && activeSimMode >= 0) {
    Serial.printf("Display: Mode=%s (%s) : %s, Line2=Add Radar OBJ\n", 
                  "Auto", simulationModes[activeSimMode], unitNumber.c_str());
  } else {
    Serial.printf("Display: Mode=%s : %s, Line2=Add Radar OBJ\n", modeName.c_str(), unitNumber.c_str());
  }
}

// Helper function untuk membuat teks radar object lengkap (untuk debug/logging)
String buildRadarObjectText(int index) {
  if (index < 0 || index >= totalRadarPresets) return "";
  
  const RadarObjectPreset &obj = radarPresets[index];
  String text = "ID:" + String(obj.id) + " ";
  text += "Vel_Lon:" + String(obj.velocity_lon, 1) + "m/s ";
  text += "Dist:" + String((int)obj.distance) + "m ";
  text += "RCS:" + String((int)obj.rcs);
  
  return text;
}

// Display halaman Add Radar Object
void displayRadarObjectSelection() {
  // Clear screen hanya sekali saat pertama kali masuk (mencegah flickering)
  if (needsClearScreen) {
    lcd.clear();
    needsClearScreen = false;
  }
  
  // Baris pertama (fixed): "ID:0XXX  RCS:20" - menampilkan ID:0 (selalu) + selected IDs
  // Format: "ID:X" di kiri, diikuti selected IDs, "RCS:YY" di kolom 9-14 (0-based: 8..13)
  lcd.setCursor(0, 0);
  
  // Build ID part dengan selected IDs
  // ID pertama selalu 0 (ID:0 adalah dynamic/best target di ESP1)
  String idPart = "ID:0";
  
  // Tambahkan selected IDs
  for (int i = 0; i < selectedRadarCount; i++) {
    idPart += String(selectedRadarIds[i]);
  }
  
  // Tambahkan dash (-) untuk sisa slot yang kosong (max 4 total characters after "ID:")
  // Total format: "ID:XXXX" (4 characters untuk IDs)
  int totalIdChars = 1 + selectedRadarCount; // currentRadarId + selected IDs
  int dashCount = 0;
  if (totalIdChars < 4) {
    dashCount = 4 - totalIdChars;
  }
  for (int i = 0; i < dashCount; i++) {
    idPart += "-";
  }
  
  String rcsPart = "RCS:" + String((int)currentRadarRcs);
  
  // Print ID part
  lcd.print(idPart);
  
  // Sisakan ruang agar RCS mulai di kolom 8 (0-based), yaitu menempati kolom 8..13 (6 karakter)
  const int RCS_START_COL = 8;  // 0-based index -> posisi 9 layar (1-based)
  const int RCS_WIDTH = 6;
  // Hitung spasi yang diperlukan antara ID part dan area RCS
  int spacesNeeded = RCS_START_COL - idPart.length();
  if (spacesNeeded < 0) spacesNeeded = 0;
  for (int i = 0; i < spacesNeeded; i++) {
    lcd.print(" ");
  }
  
  // Print RCS part di area 6 karakter mulai kolom 8 (posisi 9-14 1-based)
  lcd.setCursor(RCS_START_COL, 0);
  lcd.print(rcsPart);
  
  // Padding jika rcsPart < RCS_WIDTH karakter
  for (int i = rcsPart.length(); i < RCS_WIDTH; i++) {
    lcd.print(" ");
  }
  
  // Baris kedua: Tampilkan radar object dengan rotasi informasi setiap 2 detik
  // Format: ">ID:X" tetap, sisanya bergantian: VL / Dist / RCS
  // Special case: ID:0 menampilkan ">RESET"
  const RadarObjectPreset &obj = radarPresets[radarObjectPosition];
  
  // Cek apakah ini reset option (ID:0)
  if (obj.id == 0) {
    // Tampilkan ">RESET" untuk reset option
    char line2Buffer[17];
    memset(line2Buffer, ' ', 16);
    line2Buffer[16] = '\0';
    
    String resetText = ">RESET";
    for (int i = 0; i < resetText.length(); i++) {
      line2Buffer[i] = resetText.charAt(i);
    }
    
    lcd.setCursor(0, 1);
    lcd.print(line2Buffer);
    
    // Debug output untuk reset option
    static bool resetDebugPrinted = false;
    if (!resetDebugPrinted) {
      Serial.println("Display: RESET option");
      resetDebugPrinted = true;
    }
    return;
  }
  
  // Update rotasi display setiap 2 detik (2000ms)
  unsigned long currentTime = millis();
  if (currentTime - displayRotationTimer >= 2000) {
    displayRotationTimer = currentTime;
    displayRotationIndex++;
    if (displayRotationIndex > 2) {
      displayRotationIndex = 0; // Kembali ke Velocity
    }
  }
  
  // Fixed prefix: ">ID:X " (tidak ikut scroll) - tanpa spasi setelah >
  String fixedPrefix = ">ID:" + String(obj.id) + " ";
  
  // Buat text yang akan ditampilkan berdasarkan rotation index
  String displayText = "";
  switch (displayRotationIndex) {
    case 0: // Velocity
      displayText = "VL:" + String(obj.velocity_lon, 1) + "m/s";
      break;
    case 1: // Distance
      displayText = "Dist:" + String((int)obj.distance) + "m";
      break;
    case 2: // RCS
      displayText = "RCS:" + String((int)obj.rcs);
      break;
  }
  
  // Buat buffer lengkap 16 karakter untuk baris kedua
  char line2Buffer[17];  // 16 karakter + null terminator
  memset(line2Buffer, ' ', 16);  // Isi semua dengan spasi
  line2Buffer[16] = '\0';  // Null terminator
  
  // Copy fixed prefix ke buffer
  for (int i = 0; i < fixedPrefix.length(); i++) {
    line2Buffer[i] = fixedPrefix.charAt(i);
  }
  
  // Copy displayText ke buffer
  int startPos = fixedPrefix.length();
  for (int i = 0; i < displayText.length() && (startPos + i) < 16; i++) {
    line2Buffer[startPos + i] = displayText.charAt(i);
  }
  // Sisa sudah berupa spasi dari memset
  
  // Tulis seluruh baris kedua sekaligus (16 karakter penuh)
  lcd.setCursor(0, 1);
  lcd.print(line2Buffer);
  
  // Debug output
  static int lastPosition = -1;
  static int lastRotationIndex = -1;
  if (lastPosition != radarObjectPosition) {
    Serial.printf("Display Radar Object Selection: ID:%d Vel:%.1f Dist:%.0f RCS:%.0f\n",
                  radarPresets[radarObjectPosition].id,
                  radarPresets[radarObjectPosition].velocity_lon,
                  radarPresets[radarObjectPosition].distance,
                  radarPresets[radarObjectPosition].rcs);
    lastPosition = radarObjectPosition;
    lastRotationIndex = displayRotationIndex;
  } else if (lastRotationIndex != displayRotationIndex) {
    const char* infoType[] = {"Velocity", "Distance", "RCS"};
    Serial.printf("Display rotated to: %s\n", infoType[displayRotationIndex]);
    lastRotationIndex = displayRotationIndex;
  }
}

// Task untuk menangani komunikasi UART dari ESP1
void taskUART(void *parameter) {
  static unsigned long lastBlinkSyncMiss = 0;
  while (1) {
    // Cek apakah ada data dari ESP1 (Serial2)
    if (Serial2.available()) {
      String command = Serial2.readStringUntil('\n');
      command.trim();
      
      if (command == "SELECT") {
        handleSelectButton();
      } else if (command == "BACK") {
        handleBackButton();
      } else if (command == "SD_OK") {
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
          sdInitStatus = 1;
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
        }
        Serial.printf("ESP1 -> SD_OK received (after %d retries)\n", sdInitRetryCount);
      } else if (command == "SD_FAIL") {
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
          sdInitStatus = 0;
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
        }
        Serial.printf("ESP1 -> SD_FAIL received (retry %d/5)\n", sdInitRetryCount);
      } else if (command == "SD_MISSING") {
        // SD card removed after successful initialization (during Rec mode selection or Recording UI)
        Serial.println("ESP1 -> SD_MISSING: SD card removed!");
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
          // Stop recording if currently recording
          if (inRecording) {
            inRecording = false;
            Serial.println("Recording stopped due to SD card removal");
          }
          
          // ROBUST RESET: Clear ALL recording/save states completely
          inRecordMode = false;
          inRecUnitSelect = false;
          inRecModeSelection = false;
          inStopRecordingConfirm = false;
          inSaveConfirm = false;
          inSaveResultScreen = false;
          // Also clear Play flow states
          inPlayModeSelection = false;
          inPlayFileSelection = false;
          waitingForFileList = false;
          showModeAndUnit = false;
          recordStartTime = 0;
          recordElapsedMs = 0;
          currentRecordingFilename = "";
          currentRecordingLabel = "";
          currentRecordingModeAbbr = "";
          cursorPosition = 0;
          recModePosition = 0;
          saveConfirmPosition = 0;
          stopConfirmPosition = 0;
          selectedRecUnitIndex = 0;
      playViewEnteredAt = 0;
      playTimerFallbackArmed = false;
          // Reset Play selections
          playModePosition = 0;
          playFilePosition = 0;
          totalPlayFiles = 0;
          selectedPlayModeAbbr = "";
          
          // Reset missing screen flag untuk force render
          sdMissingScreenRendered = false;
          // Show SD missing screen (HIGHEST PRIORITY)
          inSDCardMissing = true;
          sdCardMissingStartTime = millis();
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
          Serial.println("inSDCardMissing=true, ALL states robustly reset (incl Unit Rec), force render enabled");
        }
      } else if (command.startsWith("BLINK:")) {
        String payload = command.substring(6);
        int commaPos = payload.indexOf(',');
        bool state = payload.toInt() != 0;
        uint16_t remoteStamp = 0;
        if (commaPos != -1) {
          state = payload.substring(0, commaPos).toInt() != 0;
          remoteStamp = static_cast<uint16_t>(payload.substring(commaPos + 1).toInt());
        } else {
          remoteStamp = static_cast<uint16_t>(millis() & 0xFFFF);
        }

        unsigned long nowSync = millis();
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
          rpmBlinkState = state;
          rpmBlinkLastSync = nowSync;
          rpmBlinkSequence = remoteStamp;
          rpmBlinkSynced = true;
          xSemaphoreGive(xSemaphore);
        } else if (nowSync - lastBlinkSyncMiss > 500UL) {
          lastBlinkSyncMiss = nowSync;
          Serial.println("WARN: Missed blink sync update (semaphore busy)");
        }
      } else if (command.startsWith("SPD:")) {
        // Parse speed data dari ESP1
        float speed = command.substring(4).toFloat();
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
          currentSpeed = speed;
          xSemaphoreGive(xSemaphore);
        }
      } else if (command.startsWith("RPM:")) {
        // Terima RPM dari ESP1 HANYA saat PLAY (CSV replay)
        // Hindari menimpa RPM lokal (ADC) saat Record/Manual/Auto
        bool acceptRpmFromUART = false;
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
          acceptRpmFromUART = inPlayView; // Hanya update saat Play UI aktif
          xSemaphoreGive(xSemaphore);
        }

        if (acceptRpmFromUART) {
          float rpm = command.substring(4).toFloat();
          if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
            currentRpm = rpm;
            xSemaphoreGive(xSemaphore);
          }
        } else {
          // Opsional: log ringkas untuk debug, tidak spam
          static unsigned long lastIgnoreLog = 0;
          unsigned long nowIgnore = millis();
          if (nowIgnore - lastIgnoreLog > 1000UL) {
            lastIgnoreLog = nowIgnore;
            Serial.println("UART RPM ignored (not in Play)");
          }
        }
      } else if (command.startsWith("RADAR_INFO:")) {
        // Parse radar info data dari ESP1: RADAR_INFO:id,rcs
        String params = command.substring(11); // Remove "RADAR_INFO:"
        int commaPos = params.indexOf(',');
        if (commaPos != -1) {
          int radarId = params.substring(0, commaPos).toInt();
          float radarRcs = params.substring(commaPos + 1).toFloat();
          
          if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
            currentRadarId = radarId;
            currentRadarRcs = radarRcs;
            xSemaphoreGive(xSemaphore);
          }
        }
      } else if (command.startsWith("REC_STARTED:")) {
        // ESP1 confirms recording started and returns filename base (e.g., PC-01.csv)
        String fname = command.substring(12);
        fname.trim();
        
        Serial.printf("ESP1 -> REC_STARTED: '%s'\n", fname.c_str());
        
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100))) {
          currentRecordingFilename = fname;
          // Base without extension
          String base = fname.endsWith(".csv") ? fname.substring(0, fname.length() - 4) : fname;
          
          // If ESP1 prefix mismatches, override with selected abbr for UI label
          int dashPos = base.indexOf('-');
          if (dashPos > 0 && currentRecordingModeAbbr.length() == 2) {
            currentRecordingLabel = currentRecordingModeAbbr + base.substring(dashPos);
          } else {
            currentRecordingLabel = base;
          }
          
          // Ensure UI shows recording state
          inRecordMode = true;
          inRecording = true;
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
          
          Serial.printf("Filename set: '%s', Label: '%s'\n", fname.c_str(), currentRecordingLabel.c_str());
        }
      } else if (command.startsWith("REC_STOPPED:")) {
        // ESP1 confirms recording stopped and returns base filename
        String fname = command.substring(12);
        fname.trim();
        
        Serial.printf("ESP1 -> REC_STOPPED: '%s'\n", fname.c_str());
        
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100))) {
          // Only update if filename is valid (not empty)
          if (fname.length() > 0) {
            currentRecordingFilename = fname;
            String base = fname.endsWith(".csv") ? fname.substring(0, fname.length() - 4) : fname;
            int dashPos = base.indexOf('-');
            if (dashPos > 0 && currentRecordingModeAbbr.length() == 2) {
              currentRecordingLabel = currentRecordingModeAbbr + base.substring(dashPos);
            } else {
              currentRecordingLabel = base;
            }
          } else if (currentRecordingLabel.length() == 0) {
            // Fallback if no label set yet
            currentRecordingLabel = currentRecordingModeAbbr.length() ? currentRecordingModeAbbr + "-??" : "FILE";
          }
          
          // Stay in Record UI but not recording
          inRecordMode = true;
          inRecording = false;
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
          
          Serial.printf("Filename set: '%s', Label: '%s'\n", fname.c_str(), currentRecordingLabel.c_str());
        }
      } else if (command.startsWith("REC_SAVED:")) {
        // File saved confirmed
        String fname = command.substring(10);
        fname.trim();
        
        Serial.printf("ESP1 -> REC_SAVED: '%s'\n", fname.c_str());
        
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100))) {
          if (fname.length() > 0) {
            currentRecordingFilename = fname;
            String base = fname.endsWith(".csv") ? fname.substring(0, fname.length() - 4) : fname;
            int dashPos = base.indexOf('-');
            if (dashPos > 0 && currentRecordingModeAbbr.length() == 2) {
              currentRecordingLabel = currentRecordingModeAbbr + base.substring(dashPos);
            } else {
              currentRecordingLabel = base;
            }
          } else if (currentRecordingLabel.length() == 0) {
            currentRecordingLabel = currentRecordingModeAbbr.length() ? currentRecordingModeAbbr + "-??" : "FILE";
          }
          
          lastSaveResultSaved = true;
          inSaveConfirm = false;
          inSaveResultScreen = true;
          saveResultStartTime = millis();
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
          
          Serial.printf("Save confirmed. Label: '%s'\n", currentRecordingLabel.c_str());
        }
      } else if (command.startsWith("REC_DELETED:")) {
        // File deleted confirmed
        String fname = command.substring(12);
        fname.trim();
        
        Serial.printf("ESP1 -> REC_DELETED: '%s'\n", fname.c_str());
        
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100))) {
          if (fname.length() > 0) {
            currentRecordingFilename = fname;
            String base = fname.endsWith(".csv") ? fname.substring(0, fname.length() - 4) : fname;
            int dashPos = base.indexOf('-');
            if (dashPos > 0 && currentRecordingModeAbbr.length() == 2) {
              currentRecordingLabel = currentRecordingModeAbbr + base.substring(dashPos);
            } else {
              currentRecordingLabel = base;
            }
          } else if (currentRecordingLabel.length() == 0) {
            currentRecordingLabel = currentRecordingModeAbbr.length() ? currentRecordingModeAbbr + "-??" : "FILE";
          }
          
          lastSaveResultSaved = false;
          inSaveConfirm = false;
          inSaveResultScreen = true;
          saveResultStartTime = millis();
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
          
          Serial.printf("Delete confirmed. Label: '%s'\n", currentRecordingLabel.c_str());
        }
      } else if (command.startsWith("REC_ERR:")) {
        // Handle recording errors from ESP1
        String errorCode = command.substring(8);
        errorCode.trim();
        Serial.printf("ESP1 -> REC_ERR: %s\n", errorCode.c_str());
        
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100))) {
          // Stop recording state on any error
          inRecording = false;
          inSaveConfirm = false;
          
          // Set fallback label
          if (currentRecordingLabel.length() == 0) {
            currentRecordingLabel = currentRecordingModeAbbr.length() ? currentRecordingModeAbbr + "-ERR" : "ERROR";
          }
          
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
        }
      } else if (command.startsWith("PLAY_FILES:")) {
        // ESP1 sends list of CSV files: PLAY_FILES:count,file1,file2,file3,...
        String params = command.substring(11); // Remove "PLAY_FILES:"
        int firstComma = params.indexOf(',');
        
        if (firstComma != -1) {
          int count = params.substring(0, firstComma).toInt();
          String fileListStr = params.substring(firstComma + 1);
          
          if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100))) {
            totalPlayFiles = min(count, 20); // Max 20 files
            
            // Parse file list
            int startPos = 0;
            for (int i = 0; i < totalPlayFiles; i++) {
              int nextComma = fileListStr.indexOf(',', startPos);
              if (nextComma != -1) {
                playFileList[i] = fileListStr.substring(startPos, nextComma);
                startPos = nextComma + 1;
              } else {
                // Last file
                playFileList[i] = fileListStr.substring(startPos);
                break;
              }
            }
            
            waitingForFileList = false;
            playFilePosition = 0; // Reset ke file pertama
            displayNeedsUpdate = true;
            xSemaphoreGive(xSemaphore);
            playListRequestAttempts = 0;
            lastPlayListRequestMs = 0;
            
            Serial.printf("ESP1 -> PLAY_FILES: %d files received for %s mode\n", totalPlayFiles, selectedPlayModeAbbr.c_str());
            for (int i = 0; i < totalPlayFiles; i++) {
              Serial.printf("  [%d] %s\n", i, playFileList[i].c_str());
            }
          }
        }
      } else if (command == "PLAY_NO_FILES") {
        // ESP1 reports no files found for the requested mode
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100))) {
          totalPlayFiles = 0;
          waitingForFileList = false;
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
          playListRequestAttempts = 0;
          lastPlayListRequestMs = 0;
          Serial.printf("ESP1 -> PLAY_NO_FILES: No files found for %s mode\n", selectedPlayModeAbbr.c_str());
        }
      } else if (command.startsWith("PLAY_READY:")) {
        // ESP1 confirms file is ready to play (now includes total duration)
        const int prefixLen = 11; // length of "PLAY_READY:"
        String payload = command.substring(prefixLen);
        payload.trim();
        int commaPos = payload.indexOf(',');
        String filename = (commaPos == -1) ? payload : payload.substring(0, commaPos);
        filename.trim();
        unsigned long totalFromReady = 0;
        if (commaPos != -1) {
          String totalStr = payload.substring(commaPos + 1);
          totalStr.trim();
          totalFromReady = (unsigned long) totalStr.toInt();
        }

        Serial.printf("ESP1 -> PLAY_READY: %s (total=%lu)\n", filename.c_str(), totalFromReady);

        bool readyProcessed = false;
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(50))) {
          if (!inPlayView) {
            inPlayView = true;
            inPlayFileSelection = false;
          }
          if (currentPlayFilename.length() == 0) {
            currentPlayFilename = filename;
          }

          playElapsedMs = 0;
          playViewEnteredAt = millis();
          playTimerFallbackArmed = false;
          playCompletionHandled = false;

          if (totalFromReady > 0) {
            playTotalMs = totalFromReady;
            upsertPlayDurationCache(currentPlayFilename, totalFromReady);
            playStartMillis = playViewEnteredAt;
            playTimerReady = true;
            Serial.printf("PLAY_READY: Countdown armed with total=%lu ms\n", playTotalMs);
          } else {
            playStartMillis = 0;
            playTimerReady = false;
            Serial.println("PLAY_READY: Total duration missing, waiting for PLAY_TOTAL or PLAY_TIME sync");
          }

          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
          readyProcessed = true;
        }
        if (readyProcessed) {
          Serial2.println("PLAY_ACK");
          Serial2.flush();
        }
      } else if (command.startsWith("PLAY_ERR:")) {
        // ESP1 reports play error
        String errorCode = command.substring(9);
        errorCode.trim();
        Serial.printf("ESP1 -> PLAY_ERR: %s\n", errorCode.c_str());
        // Kembali ke daftar file jika terjadi error saat prepare/play
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(50))) {
          inPlayView = false;
          inPlayFileSelection = true;
          playViewEnteredAt = 0;
          playTimerFallbackArmed = false;
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
        }
      } else if (command.startsWith("PLAY_TIME:")) {
        // ESP1 sends exact CSV time in ms for synchronization
        unsigned long tms = (unsigned long) command.substring(10).toInt();
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(20))) {
          playElapsedMs = tms;
          if (playStartMillis == 0 && tms > 0) {
            unsigned long nowSync = millis();
            playStartMillis = (tms >= nowSync) ? nowSync : (nowSync - tms);
          }
          if (!playTimerReady && playTotalMs > 0) {
            playTimerReady = true;
            playTimerFallbackArmed = false;
          }
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
        }
        // Debug log setiap 1 detik (1000 ms interval)
        static unsigned long lastLogTime = 0;
        if (tms - lastLogTime >= 1000) {
          lastLogTime = tms;
          Serial.printf("PLAY_TIME sync: %lu ms\n", tms);
        }
      } else if (command.startsWith("PLAY_TOTAL:")) {
        // ESP1 sends total duration of CSV in ms
        unsigned long total = (unsigned long) command.substring(11).toInt();
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(20))) {
          playTotalMs = total;
          upsertPlayDurationCache(currentPlayFilename, total);
          if (total > 0 && !playTimerReady) {
            if (playStartMillis == 0) {
              playStartMillis = millis();
            }
            playTimerReady = true;
            playTimerFallbackArmed = false;
          }
          displayNeedsUpdate = true;
          xSemaphoreGive(xSemaphore);
        }
        Serial.printf("ESP1 -> PLAY_TOTAL: %lu ms (timer will start on PLAY_READY)\n", total);
      } else if (command == "PLAY_END") {
        // ESP1 signals end of playback
        Serial.println("ESP1 -> PLAY_END: Playback finished");
        enterSimulationCompleted("PLAY_END");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Task untuk menangani tombol DOWN
void taskButtonDown(void *parameter) {
  bool lastDownState = HIGH;
  
  while (1) {
    bool currentDownState = digitalRead(BUTTON_DOWN);
    
    // Deteksi penekanan tombol DOWN (falling edge)
    if (lastDownState == HIGH && currentDownState == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
      if (digitalRead(BUTTON_DOWN) == LOW) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
          // Enable button di mode Add Radar Object untuk navigasi
          if (inAddRadarMode) {
            moveCursorDown();
            displayNeedsUpdate = true;
            Serial.println("Tombol DOWN ditekan - navigasi radar object");
          }
          // Disable button when showing active mode and unit (final display)
          else if (!showModeAndUnit) {
            moveCursorDown();
            displayNeedsUpdate = true;
            Serial.println("Tombol DOWN ditekan");  
          } else {
            Serial.println("Tombol DOWN disabled - dalam mode aktif");
          }
          xSemaphoreGive(xSemaphore);
        }
      }
    }
    
    lastDownState = currentDownState;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Task untuk menangani tombol UP
void taskButtonUp(void *parameter) {
  bool lastUpState = HIGH;
  
  while (1) {
    bool currentUpState = digitalRead(BUTTON_UP);
    
    // Deteksi penekanan tombol UP (falling edge)
    if (lastUpState == HIGH && currentUpState == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
      if (digitalRead(BUTTON_UP) == LOW) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
          // Enable button di mode Add Radar Object untuk navigasi
          if (inAddRadarMode) {
            moveCursorUp();
            displayNeedsUpdate = true;
            Serial.println("Tombol UP ditekan - navigasi radar object");
          }
          // Disable button when showing active mode and unit (final display)
          else if (!showModeAndUnit) {
            moveCursorUp();
            displayNeedsUpdate = true;
            Serial.println("Tombol UP ditekan");
          } else {
            Serial.println("Tombol UP disabled - dalam mode aktif");
          }
          xSemaphoreGive(xSemaphore);
        }
      }
    }
    
    lastUpState = currentUpState;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Task untuk menangani update display
void taskDisplay(void *parameter) {
  while (1) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      // CRITICAL: Check SD card missing first (highest priority error)
      if (inSDCardMissing) {
        // Render SD missing screen sekali saja (handled by displaySDCardMissing)
        displayMenu();
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(200)); // Much slower update - screen is static
        continue;
      } else if (inSDInitFailed) {
        // Render failed screen sekali saja (handled by displaySDInitFailed with static flag)
        displayMenu();
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(200)); // Much slower update - screen is static
        continue;
      } else if (inSDInitWait) {
        // Update layar SD init hanya saat retry count berubah (handled by displaySDInitScreen)
        displayMenu();
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100)); // Slower update untuk reduce I2C traffic
        continue;
      } else if (inAddRadarMode) {
        displayMenu(); // Will call displayRadarObjectSelection
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(50)); // Update lebih cepat untuk smooth running text
        continue;
      } else if (inPlayView) {
        // Update layar Play secara periodik (untuk timer berjalan)
        displayMenu(); // Will call displayPlayScreen
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      } else if (inSimulationCompleted) {
        // Update layar Simulation Completed untuk timeout handling
        displayMenu(); // Will call displaySimulationCompleted
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms for timeout
        continue;
      } else if (inRecordMode) {
        // Update layar secara periodik saat di Record UI (untuk counter waktu)
        displayMenu(); // Will call displayRecordScreen
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }
      xSemaphoreGive(xSemaphore);
    }
    
    if (displayNeedsUpdate) {
      if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
        displayMenu();
        displayNeedsUpdate = false;
        xSemaphoreGive(xSemaphore);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task untuk membaca RPM dari ADC (filter dikurangi seperti ESP1.ino)
void taskRPMRead(void *parameter) {
  const float rpm_alpha = 0.2f;  // Smoothing factor dikurangi (lebih responsif)
  float rpm_smoothed = 0.0f;
  
  // Moving average buffer dikurangi
  const int MA_SIZE = 3;  // Dikurangi dari 8 ke 3
  float ma_buffer[MA_SIZE] = {0};
  int ma_index = 0;
  float ma_sum = 0.0f;
  int ma_count = 0;
  
  // Hysteresis filter dikurangi
  float last_stable_rpm = 0.0f;
  const float HYSTERESIS_THRESHOLD = 2.0f;  // Dikurangi dari 5.0f ke 2.0f
  
  // Median filter buffer untuk recording mode (robust terhadap noise)
  const int MEDIAN_SIZE = 5;
  int median_buffer[MEDIAN_SIZE];
  int median_index = 0;
  
  while (1) {
    bool autoMode = false;
    int simMode = -1;
    bool playMode = false;
    bool recordingMode = false;
    bool recFlowActive = false; // Any Rec flow phase (unit select, mode select, or recording UI)
    
    // Check if in AUTO mode, PLAY mode, RECORDING mode, or any Rec flow
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
      autoMode = isAutoMode;
      simMode = autoSimMode;
      playMode = inPlayView; // Check if in Play UI
      recordingMode = inRecording; // Check if currently recording
      recFlowActive = (inRecUnitSelect || inRecModeSelection || inRecordMode); // Any Rec phase
      xSemaphoreGive(xSemaphore);
    }
    
    float rpm_final;
    
    if (playMode) {
      // PLAY mode: RPM comes from CSV via UART, don't override it
      // Just skip ADC reading and use currentRpm as-is
      if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
        rpm_final = currentRpm; // Use RPM from CSV
        xSemaphoreGive(xSemaphore);
      }
    } else if (autoMode && simMode >= 0) {
      // AUTO mode: Use simulated RPM value (500 RPM for all simulation modes)
      rpm_final = 500.0f;  // Fixed RPM for all simulation modes
      
      // Update global RPM value directly
      if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
        currentRpm = rpm_final;
        xSemaphoreGive(xSemaphore);
      }
    } else {
      // MANUAL/RECORDING mode: Read from ADC with enhanced filtering during Rec flow
      int adc_samples;
      if (recFlowActive) {
        // During ANY Rec flow (termasuk saat masuk UI): Use more samples and median filter
        adc_samples = 7;  // Increased from 3 to 7 for better noise rejection
      } else {
        // Normal Manual operation: Use fewer samples for responsiveness
        adc_samples = 3;
      }
      
      // Collect ADC samples
      int adc_values[adc_samples];
      for (int i = 0; i < adc_samples; i++) {
        adc_values[i] = analogRead(RPM_ADC_PIN);
        if (recFlowActive) {
          vTaskDelay(pdMS_TO_TICKS(2));  // Slightly longer delay during Rec flow
        } else {
          vTaskDelay(pdMS_TO_TICKS(1));
        }
      }
      
      int adc;
      if (recFlowActive) {
        // During Rec flow: Use median filter to reject outliers (CAN noise)
        // Sort the array to find median
        for (int i = 0; i < adc_samples - 1; i++) {
          for (int j = 0; j < adc_samples - i - 1; j++) {
            if (adc_values[j] > adc_values[j + 1]) {
              int temp = adc_values[j];
              adc_values[j] = adc_values[j + 1];
              adc_values[j + 1] = temp;
            }
          }
        }
        adc = adc_values[adc_samples / 2];  // Median value
      } else {
        // Normal Manual operation: Simple averaging
        int adc_sum = 0;
        for (int i = 0; i < adc_samples; i++) {
          adc_sum += adc_values[i];
        }
        adc = adc_sum / adc_samples;
      }
      
      // Convert ADC to RPM (0-4095 → 0-RPM_MAX)
      float rpm_raw = ((float)adc / 4095.0f) * RPM_MAX;
      rpm_raw = constrain(rpm_raw, 0.0f, RPM_MAX);
      
      // Apply Kalman filter
      float rpm_filtered = kfRPM.update(rpm_raw);
      
      // Apply exponential smoothing
      rpm_smoothed = (rpm_alpha * rpm_filtered) + ((1.0f - rpm_alpha) * rpm_smoothed);
      
      // Apply moving average for additional smoothing
      ma_sum -= ma_buffer[ma_index];
      ma_buffer[ma_index] = rpm_smoothed;
      ma_sum += rpm_smoothed;
      ma_index = (ma_index + 1) % MA_SIZE;
      if (ma_count < MA_SIZE) ma_count++;
      rpm_final = ma_sum / ma_count;
      
      // Apply hysteresis to prevent small fluctuations
      if (abs(rpm_final - last_stable_rpm) >= HYSTERESIS_THRESHOLD) {
        last_stable_rpm = rpm_final;
      } else {
        rpm_final = last_stable_rpm;
      }
      
      // Apply threshold clamping: <2 = 0, >=2998 = 3000
      if (rpm_final < 2.0f) {
        rpm_final = 0.0f;  // Set sangat rendah ke 0
      } else if (rpm_final >= 2998.0f) {
        rpm_final = 3000.0f;  // Set sangat tinggi ke maksimum
      }
      
      // Update global RPM value
      if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
        currentRpm = rpm_final;
        xSemaphoreGive(xSemaphore);
      }
    }
    
    // Periodically send RPM to ESP1 during recording (every ~100ms)
    static unsigned long lastRpmSend = 0;
    unsigned long now_send = millis();
    bool sendRpm = false;
    float rpm_to_send = rpm_final;
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(2))) {
      if (inRecordMode && inRecording) {
        sendRpm = true;
      }
      xSemaphoreGive(xSemaphore);
    }
    if (sendRpm && (now_send - lastRpmSend) >= 100UL) {
      lastRpmSend = now_send;
      int rpm_int = (int)roundf(rpm_to_send);
      Serial2.print("RPM:");
      Serial2.println(rpm_int);
    }

    vTaskDelay(pdMS_TO_TICKS(30));  // Read every 30ms (lebih responsif)
  }
}

// Fungsi untuk menampilkan 4 digit penuh (sama dengan lengkap_esp2.ino)
void showFull4Digits(TM1637Display &disp, int value) {
  // Clamp ke 0..9999
  if (value < 0) value = 0;
  if (value > 9999) value = 9999;

  int d0 = (value / 1000) % 10; // thousands (pos 0)
  int d1 = (value / 100) % 10;  // hundreds  (pos 1)
  int d2 = (value / 10) % 10;   // tens      (pos 2)
  int d3 = value % 10;          // units     (pos 3)

  uint8_t segs[4];

  // Leading zero suppression - hanya tampilkan digit yang diperlukan
  segs[0] = (value >= 1000) ? disp.encodeDigit(d0) : 0x00;
  segs[1] = (value >= 100)  ? disp.encodeDigit(d1) : 0x00;
  segs[2] = (value >= 10)   ? disp.encodeDigit(d2) : 0x00;
  segs[3] = disp.encodeDigit(d3);

  disp.setSegments(segs, 4, 0);
}

// Task untuk menampilkan RPM di 7-segmen
void taskRPMDisplay(void *parameter) {
  static bool cachedBlinkMode = true;
  static bool cachedRadarActive = false;
  static float cachedRpm = 0.0f;
  static bool cachedBlinkState = false;
  static bool cachedBlinkSynced = false;
  static unsigned long cachedBlinkTimestamp = 0;
  static uint16_t cachedBlinkSequence = 0;
  static unsigned long lastMutexMissLog = 0;

  static bool blinkModeActive = false;
  static unsigned long blinkModeEntry = 0;
  static bool fallbackBlinkState = false;
  static unsigned long fallbackLastToggle = 0;
  static unsigned long lastFallbackLog = 0;
  static unsigned long lastMasterLog = 0;
  
  while (1) {
    unsigned long loopNow = millis();

    // Refresh cache when semaphore available; fall back to cached values otherwise
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(20))) {
      cachedBlinkMode = rpmRunningLEDMode;
      cachedRadarActive = rpmRadarActive;
      cachedRpm = currentRpm;
      cachedBlinkState = rpmBlinkState;
      cachedBlinkSynced = rpmBlinkSynced;
      cachedBlinkTimestamp = rpmBlinkLastSync;
      cachedBlinkSequence = rpmBlinkSequence;
      xSemaphoreGive(xSemaphore);
    } else {
      if (loopNow - lastMutexMissLog >= 500UL) {
        lastMutexMissLog = loopNow;
        Serial.println("RPM Display: mutex busy, using cached values");
      }
    }
    
    if (cachedBlinkMode && !cachedRadarActive) {
      if (!blinkModeActive) {
        blinkModeActive = true;
        blinkModeEntry = loopNow;
        fallbackBlinkState = false;
        fallbackLastToggle = loopNow;
      }

  bool masterFresh = cachedBlinkSynced && (loopNow - cachedBlinkTimestamp) <= 900UL;
      bool displayOn;

      if (masterFresh) {
        displayOn = cachedBlinkState;
        fallbackBlinkState = cachedBlinkState;
        fallbackLastToggle = loopNow;

        if (loopNow - lastMasterLog >= 2000UL) {
          lastMasterLog = loopNow;
          Serial.printf("RPM Blink: synced (seq=%u state=%d)\n", cachedBlinkSequence, cachedBlinkState ? 1 : 0);
        }
      } else {
        bool inGraceWindow = !cachedBlinkSynced && (loopNow - blinkModeEntry) < 1000UL;
        if (!inGraceWindow && (loopNow - fallbackLastToggle) >= 500UL) {
          fallbackLastToggle = loopNow;
          fallbackBlinkState = !fallbackBlinkState;
        }
        displayOn = fallbackBlinkState;

        unsigned long fallbackLogInterval = cachedBlinkSynced ? 2000UL : 3500UL;
        if (loopNow - lastFallbackLog >= fallbackLogInterval) {
          lastFallbackLog = loopNow;
          Serial.println("RPM Blink: fallback phase (waiting or desynced)");
        }
      }

      if (displayOn) {
        uint8_t segs[4];
        uint8_t zero_digit = rpmDisplay.encodeDigit(0);
        segs[0] = zero_digit;
        segs[1] = zero_digit;
        segs[2] = zero_digit;
        segs[3] = zero_digit;
        rpmDisplay.setSegments(segs, 4, 0);
      } else {
        rpmDisplay.clear();
      }
    } else {
      if (blinkModeActive) {
        blinkModeActive = false;
        fallbackBlinkState = false;
      }

      float rpm_local = cachedRpm;
      int rpm_display = (int)round(rpm_local);
      showFull4Digits(rpmDisplay, rpm_display);
      
      static int debug_counter = 0;
      if (debug_counter++ % 20 == 0) {
        Serial.printf("RPM: %.1f (Display: %d)\n", rpm_local, rpm_display);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ========== CAN INITIALIZATION ==========
void initCAN() {
  Serial.printf("Initializing CAN at %d baud...\n", current_config.can_baudrate);
  
  // Install CAN driver
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  
  // Set baudrate based on unit config
  twai_timing_config_t t_config;
  if (current_config.can_baudrate == 250000) {
    t_config = TWAI_TIMING_CONFIG_250KBITS();
  } else if (current_config.can_baudrate == 500000) {
    t_config = TWAI_TIMING_CONFIG_500KBITS();
  } else {
    t_config = TWAI_TIMING_CONFIG_250KBITS(); // Default
  }
  
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("ERROR: Failed to install CAN driver!");
    canEnabled = false;
    return;
  }
  
  if (twai_start() != ESP_OK) {
    Serial.println("ERROR: Failed to start CAN!");
    canEnabled = false;
    return;
  }
  
  Serial.println("CAN Bus initialized successfully");
}

// ========== CAN TRANSMISSION FUNCTIONS (same as lengkap_esp2.ino) ==========
static inline bool transmitCAN(const twai_message_t &msg, TickType_t timeout = pdMS_TO_TICKS(20)) {
  return (twai_transmit(&msg, timeout) == ESP_OK);
}

void sendHOWOCombinedMessage(float speed_kmh, uint16_t engine_rpm_raw) {
  // HOWO_EV: Both engine and vehicle speed in the same CAN message (ID: 0xCFE6CEE)
  twai_message_t msg = {};
  msg.identifier = 0x0CFE6CEEUL;
  msg.extd = 1;
  msg.data_length_code = 8;
  memset(msg.data, 0, 8);
  
  // Engine Speed in bytes 4-5: frame[5] << 8 | frame[4]
  msg.data[4] = (uint8_t)(engine_rpm_raw & 0xFF);
  msg.data[5] = (uint8_t)((engine_rpm_raw >> 8) & 0xFF);
  
  // Vehicle Speed in bytes 6-7: ((frame[7] << 8) | frame[6])/256
  uint32_t speed_value16 = (uint32_t)roundf(speed_kmh * 256.0f);
  if (speed_value16 > 0xFFFF) speed_value16 = 0xFFFF;
  msg.data[6] = (uint8_t)(speed_value16 & 0xFF);
  msg.data[7] = (uint8_t)((speed_value16 >> 8) & 0xFF);
  
  transmitCAN(msg);
}

void sendVehicleSpeed(float speed_kmh) {
  // Skip sending vehicle speed for hybrid units (they use frequency pulse for speed)
  if (current_config.hybrid_mode) {
    return; // Do not send vehicle speed CAN for hybrid units
  }
  
  uint32_t value16 = (uint32_t)roundf(speed_kmh * 256.0f);
  if (value16 > 0xFFFF) value16 = 0xFFFF;

  twai_message_t msg = {};
  msg.identifier = current_config.can_id_tx;
  msg.extd = 1;
  msg.data_length_code = 8;
  memset(msg.data, 0, 8);
  
  if (activeUnit == 17) { // SANY_SYZ_428_C
    uint8_t speed_byte = (uint8_t)constrain(speed_kmh, 0, 255);
    msg.data[7] = speed_byte;
  } else if (activeUnit == 15) { // SHACMAN_F3000_430_CAN
    msg.data[6] = (uint8_t)(value16 & 0xFF);
    msg.data[7] = (uint8_t)((value16 >> 8) & 0xFF);
  } else {
    // Standard encoding: bytes 1-2
    msg.data[1] = (uint8_t)(value16 & 0xFF);
    msg.data[2] = (uint8_t)((value16 >> 8) & 0xFF);
  }

  transmitCAN(msg);
}

bool sendEngineSpeed(uint16_t value16) {
  twai_message_t msg = {};
  msg.identifier = current_config.can_id_rx;
  msg.extd = 1;
  msg.data_length_code = 8;
  memset(msg.data, 0, 8);
  
  // Handle special decode units
  if (current_config.extended_id && (activeUnit == 4 || activeUnit == 7)) { // HOWO_EV or HANVAN_EV
    switch(activeUnit) {
      case 4: // HOWO_EV
        msg.data[4] = (uint8_t)(value16 & 0xFF);
        msg.data[5] = (uint8_t)((value16 >> 8) & 0xFF);
        break;
      case 7: // HANVAN_EV
        msg.data[5] = (uint8_t)(value16 & 0xFF);
        msg.data[6] = (uint8_t)((value16 >> 8) & 0xFF);
        break;
      default:
        // Fallback to standard
        msg.data[3] = (uint8_t)(value16 & 0xFF);
        msg.data[4] = (uint8_t)((value16 >> 8) & 0xFF);
        break;
    }
  } else {
    // Check for HOWO_400/371 special case - they use bytes 1-2 like J1939 standard
    if (activeUnit == 2 || activeUnit == 12) { // HOWO_400 or HOWO_371
      // HOWO_400/371: frame[2] << 8 | frame[1] (bytes 1-2) - J1939 format
      msg.data[1] = (uint8_t)(value16 & 0xFF);
      msg.data[2] = (uint8_t)((value16 >> 8) & 0xFF);
      
      // Debug output for HOWO units
      float decoded_rpm = (float)value16 * 0.125f;
      Serial.printf("DEBUG %s: Sending RPM %.1f -> CAN value %u (0x%04X) to ID 0x%08X, bytes[1,2]=[0x%02X,0x%02X]\n", 
                    current_config.name, decoded_rpm, value16, value16, msg.identifier, msg.data[1], msg.data[2]);
    } else {
      // Standard encoding: frame[4] << 8 | frame[3] (bytes 3-4)
      msg.data[3] = (uint8_t)(value16 & 0xFF);
      msg.data[4] = (uint8_t)((value16 >> 8) & 0xFF);
    }
  }

  return transmitCAN(msg);
}

// Kirim satu kali frame CAN bernilai 0 untuk speed dan RPM sesuai tipe unit
void sendZeroOutputsOnce() {
  // Pastikan konfigurasi aktif tersedia
  if (activeUnit < 0 || !canEnabled) return;

  uint16_t zero_rpm_value16;
  if (activeUnit == 4) {
    // HOWO_EV: RPM langsung 0..65535, gunakan 0
    zero_rpm_value16 = 0;
  } else {
    // J1939: 0.125 RPM/bit -> 0 RPM = 0
    zero_rpm_value16 = 0;
  }

  if (activeUnit == 4) {
    // Combined message untuk HOWO_EV
    sendHOWOCombinedMessage(0.0f, zero_rpm_value16);
  } else if (current_config.hybrid_mode) {
    // Hybrid: hanya kirim RPM via CAN
    sendEngineSpeed(zero_rpm_value16);
  } else {
    // Standard: kirim speed dan RPM terpisah
    sendVehicleSpeed(0.0f);
    sendEngineSpeed(zero_rpm_value16);
  }
}



// ========== TASK: CAN MESSAGE SENDING ==========
void taskCANSend(void *parameter) {
  // CAN sending intervals (same as lengkap_esp2.ino)
  const unsigned long VEHICLE_SEND_MS = 60;
  const unsigned long ENGINE_SEND_MS = 60;
  
  unsigned long lastVehicle = millis();
  unsigned long lastEngine = millis();
  
  // Cache last known values to avoid sending zeros when mutex is busy (e.g., during LCD updates)
  static float last_speed_kmh = 0.0f;
  static float last_rpm = 0.0f;
  static bool have_cache = false;
  
  while (1) {
    // Check if CAN is enabled in normal mode OR if in any Rec mode phase with unit selected
    bool shouldSendCAN = false;
    int effectiveUnit = -1;
    bool isRecMode = false;
    
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5))) {
      if (canEnabled && activeUnit >= 0) {
        // Normal mode (Manual/Auto/Play)
        shouldSendCAN = true;
        effectiveUnit = activeUnit;
        isRecMode = false;
      } else if (selectedRecUnitIndex >= 0 && (inRecUnitSelect || inRecModeSelection || inRecordMode)) {
        // Any Rec mode phase with selected unit (unit select, mode select, or recording)
        shouldSendCAN = true;
        effectiveUnit = selectedRecUnitIndex;
        isRecMode = true;
      }
      xSemaphoreGive(xSemaphore);
    }
    
    if (shouldSendCAN) {
      unsigned long now = millis();
      float speed_local;
      float rpm_local;
      bool isSimCompleted = false;
      
      // Try to get current data; if mutex busy, use cached values to prevent 0 spikes
      if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(10))) {
        last_speed_kmh = currentSpeed;
        last_rpm = currentRpm;
        isSimCompleted = inSimulationCompleted;
        have_cache = true;
        xSemaphoreGive(xSemaphore);
      }
      // Use cached values if available; otherwise default to 0.0f on first iterations
      speed_local = have_cache ? last_speed_kmh : 0.0f;
      rpm_local = have_cache ? last_rpm : 0.0f;
      // Jika diminta paksa 0 (setelah keluar AUTO), override menjadi 0
      if (forceZeroOutputs && !isRecMode) {
        speed_local = 0.0f;
        rpm_local = 0.0f;
      }
      // Force 0 values during simulation completed screen (3 seconds)
      if (isSimCompleted) {
        speed_local = 0.0f;
        rpm_local = 0.0f;
      }
      
      // Get unit config for effectiveUnit
      UnitConfig unitConfig = unitConfigs[effectiveUnit];
      
      // Calculate CAN value based on unit type (same as lengkap_esp2.ino)
      uint16_t can_value16;
      if (effectiveUnit == 4) { // HOWO_EV: Use direct encoding for 0-2000 RPM range
        can_value16 = (uint16_t)(rpm_local);
        if (can_value16 > 0xFFFF) can_value16 = 0xFFFF;
      } else {
        // Standard: multiply by 8 for J1939 format (0.125 RPM resolution)
        uint32_t tmp = (uint32_t)roundf(rpm_local * 8.0f);
        if (tmp > 0xFFFF) tmp = 0xFFFF;
        can_value16 = (uint16_t)tmp;
      }
      
      // Handle CAN transmission based on unit type
      if (effectiveUnit == 4) { // HOWO_EV - combined message
        if ((now - lastVehicle) >= VEHICLE_SEND_MS) {
          lastVehicle = now;
          lastEngine = now; // Sync both timers since we send combined message
          sendHOWOCombinedMessage(speed_local, can_value16);
        }
      } else if (unitConfig.hybrid_mode) {
        // Hybrid mode: Only send RPM via CAN (speed comes from pulse)
        if ((now - lastEngine) >= ENGINE_SEND_MS) {
          lastEngine = now;
          sendEngineSpeed(can_value16);
        }
      } else {
        // Standard separate messages for other CAN units
        if ((now - lastVehicle) >= VEHICLE_SEND_MS) {
          lastVehicle = now;
          sendVehicleSpeed(speed_local);
        }

        if ((now - lastEngine) >= ENGINE_SEND_MS) {
          lastEngine = now;
          sendEngineSpeed(can_value16);
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
