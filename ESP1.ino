// ESP1 - Radar Simulator dengan Menu Navigation (Manual/Auto/Play)
// Komunikasi dengan ESP2 untuk kontrol menu via UART2

#include <driver/twai.h>
#include <TM1637Display.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

// ========== PIN DEFINITIONS (SAMA DENGAN lengkapRTOS.ino) ==========
// Buttons
#define BUTTON_SELECT 23  // Internal pull up
#define BUTTON_BACK 39    // External pull up (sudah digunakan ESP2, pakai pin lain untuk CAN di ESP1)

// LED indicator
#define LED_BUILTIN 2     // Blue LED onboard (GPIO2)

// UART2 Communication dengan ESP2
#define RXD2 16
#define TXD2 17

// CAN Bus
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define CAN_BAUDRATE 500000

// SD Card pins (same as lengkapRTOS.ino)
#define SD_SCK 14
#define SD_MOSI 15
#define SD_MISO 2
#define SD_CS 13

// ADC Inputs
#define ADC_PIN 35    // Speed input (input-only ADC1, VN)
#define ADC_PIN2 34   // Object distance (input-only ADC1, VM)
#define ADC_PIN3 36   // Velocity longitude (input-only ADC1, VP)

// 7-Segment Displays
#define CLK_PIN 22    // Speed display
#define DIO_PIN 21    // Speed display
#define CLK_PIN2 19   // Object distance display
#define DIO_PIN2 18
#define CLK_PIN3 26   // Velocity longitude display
#define DIO_PIN3 27

// ========== GLOBAL VARIABLES ==========
// Mode control
enum SystemMode { MODE_IDLE, MODE_MANUAL, MODE_AUTO, MODE_PLAY };
enum SimulationMode { SIM_PRE_COLLISION, SIM_SOFT_BRAKE, SIM_FULL_BRAKE, SIM_FULL_SCENARIO };
volatile SystemMode currentMode = MODE_IDLE;
volatile SimulationMode currentSimMode = SIM_PRE_COLLISION;
volatile uint8_t selectedUnitIndex = 0;  // 0-17 (18 units total)
volatile bool radarActive = false;
volatile bool runningLEDMode = true;  // Mode blinking "0000" untuk 7-segmen (default ON saat idle)

// Full Scenario timing variables
volatile unsigned long scenarioStartTime = 0;
volatile bool scenarioActive = false;

// Speed configuration (sama dengan lengkapRTOS.ino)
float SPEED_MAX = 80.0f;      // Maximum speed range (km/h) - sama dengan lengkapRTOS.ino

// Radar Target Management (sama dengan lengkapRTOS.ino)
std::vector<uint8_t> activeTargets; // Urutan target yang aktif berdasarkan input dari ESP2

// Radar data
volatile float speed = 0.0f;           // km/h
volatile float velocity_lon = 0.0f;    // m/s
volatile float distance_lon = 0.0f;    // meters

// CAN Message counters
uint16_t meas_count = 0;
uint8_t interface_version = 1;

// Task Handles
TaskHandle_t taskButtonSelectHandle = NULL;
TaskHandle_t taskButtonBackHandle = NULL;
TaskHandle_t taskUARTCommHandle = NULL;
TaskHandle_t taskSensorReadHandle = NULL;
TaskHandle_t taskCANSendHandle = NULL;
TaskHandle_t taskDisplayHandle = NULL;

// Semaphore untuk proteksi data
SemaphoreHandle_t xDataMutex = NULL;

// 7-Segment Display Objects
TM1637Display display(CLK_PIN, DIO_PIN);     // Speed
TM1637Display display2(CLK_PIN2, DIO_PIN2);  // Distance
TM1637Display display3(CLK_PIN3, DIO_PIN3);  // Velocity

// SD state
bool sdInitialized = false;
bool sdMonitoringEnabled = false; // Control monitoring window via ESP2 commands
unsigned long lastSDCheckTime = 0;
const unsigned long SD_CHECK_INTERVAL = 2000; // Check every 2 seconds when recording
bool sdCardWasPresent = false;

// Recording state (CSV logging)
bool recordingActive = false;
String currentRecFilename = "";   // e.g., "/PC-01.csv"
File recFile;                      // Active CSV file
unsigned long recordStartMillis = 0;
unsigned long lastRecordWriteMillis = 0;
int recordWriteCount = 0;          // For periodic flush
float latestRPM = 0.0f;            // Updated from ESP2 via UART
// Precise 100ms scheduler state
unsigned long nextRecordWriteTime = 0; // Absolute next write time (millis)
unsigned long recordSampleIndex = 0;   // 0,1,2.. -> t_ms = index*100

// Playback state (CSV replay)
bool playActive = false;           // Sedang replay file CSV
String currentPlayFilename = "";   // "/PC-01.csv" (full path)
File playFile;                      // Handle file yang diputar
unsigned long playStartWall = 0;    // millis() saat mulai replay
unsigned long currentPlayTimeMs = 0; // time_ms terakhir dari CSV yang diputar
unsigned long lastPlayTimeSend = 0;  // untuk rate-limit kirim PLAY_TIME
bool playHeaderSkipped = false;      // header CSV sudah dilewati
unsigned long playTotalMs = 0;       // Total durasi CSV (time_ms maksimum)
bool waitingPlayReadyAck = false;    // Menunggu ACK dari ESP2 untuk PLAY_READY payload
unsigned long playReadyLastSent = 0; // Timestamp kirim terakhir
uint16_t playReadyResendCount = 0;   // Jumlah pengiriman ulang PLAY_READY
String lastPlayReadyMessage = "";   // Payload PLAY_READY terakhir
const unsigned long PLAY_READY_RESEND_MS = 200; // Interval resend jika belum ada ACK (ms)
const uint16_t PLAY_READY_WARN_INTERVAL = 25;   // Interval log saat ACK belum diterima

// Helper: Scan CSV file to find maximum time_ms
unsigned long scanCSVMaxTime(String path) {
  File f = SD.open(path, FILE_READ);
  if (!f) return 0;
  
  unsigned long maxTime = 0;
  bool headerSkipped = false;
  
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    
    if (!headerSkipped) {
      if (line.startsWith("time_ms")) {
        headerSkipped = true;
        continue;
      }
    }
    
    int commaPos = line.indexOf(',');
    if (commaPos != -1) {
      unsigned long t = (unsigned long) line.substring(0, commaPos).toInt();
      if (t > maxTime) maxTime = t;
    }
  }
  
  f.close();
  return maxTime;
}

// Helper: Get next available filename for recording (PC-01.csv, PC-02.csv, etc.)
String getNextAvailableFilename(String abbr) {
  // Check numbers from 01 to 99
  for (int num = 1; num <= 99; num++) {
    char numStr[3];
    sprintf(numStr, "%02d", num); // Format as 01, 02, ..., 99
    
    String filename = "/" + abbr + "-" + String(numStr) + ".csv";
    
    if (!SD.exists(filename)) {
      return filename; // Found available slot
    }
  }
  
  return ""; // No available slots (01-99 all used)
}

// Helper: Write CSV header
void writeCSVHeader() {
  if (recFile) {
    recFile.println("time_ms,speed_kmh,RPM,distance_lon_m,velocity_lon_ms,rcs");
    recFile.flush();
  }
}

// Unit Configuration (sama dengan lengkapRTOS.ino)
struct UnitConfig {
  const char* name;
  float speed_max;
  float freq_calibration;
  float speed_calibration;
};

const UnitConfig unitConfigs[] = {
  {"HINO_500", 60.0f, 1.23f, 1.0f},
  {"FAW_JH6_420", 60.0f, 1.23f, 1.0f},
  {"HOWO_400", 60.0f, 5.377f, 1.0f},
  {"HOWO_400_V7X", 60.0f, 5.377f, 1.0f},
  {"HOWO_EV", 60.0f, 5.377f, 1.0f},
  {"HANVAN_G7", 60.0f, 2.515f, 1.0f},
  {"HANVAN_XG1", 60.0f, 2.515f, 1.0f},
  {"HANVAN_EV", 60.0f, 2.515f, 1.0f},
  {"X3000", 60.0f, 2.454f, 1.0f},
  {"F3000_420", 60.0f, 2.454f, 1.0f},
  {"F3000_430", 60.0f, 2.454f, 1.0f},
  {"HINO_EURO_2", 60.0f, 2.16f, 1.0f},
  {"HOWO_371", 60.0f, 5.377f, 1.0f},
  {"DONGFENG_KY50", 60.0f, 2.0f, 1.0f},
  {"F3000_420_CAN", 60.0f, 2.454f, 1.0f},
  {"F3000_430_CAN", 60.0f, 2.454f, 1.0f},
  {"AUMAN_EST_380", 60.0f, 2.3f, 1.0f},
  {"SANY_SYZ_428_C", 60.0f, 2.5f, 1.0f}
};
const int totalUnits = 18;

// Radar Target Structure
struct RadarTarget {
  uint8_t id;
  float distance_lon;
  float distance_lat;
  float velocity_lon;
  float velocity_lat;
  float rcs;
  float snr;
  uint8_t dynamic_prop;
};

// Default dynamic target (ID 0) - controlled by ADC
RadarTarget target = {0, 10.0f, 0.0f, -5.4f, 0.0f, 20.0f, 15.0f, 1};

// Predefined static targets (sama dengan lengkapRTOS.ino)
const RadarTarget target_default = {0, 10.0f, 0.0f, -5.4f, 0.0f, 20.0f, 15.0f, 1};
RadarTarget target1 = {1, 10.0f, 0.0f, -5.3f, 0.0f, 25.0f, 15.0f, 1}; // id=1, distance=10, velocity_lon=-5.3, rcs=25
RadarTarget target2 = {2, 5.0f, 0.0f, -5.3f, 0.0f, 27.0f, 15.0f, 1};  // id=2, distance=5, velocity_lon=-5.3, rcs=27
RadarTarget target3 = {3, 2.0f, 0.0f, -5.3f, 0.0f, 30.0f, 15.0f, 1};  // id=3, distance=2, velocity_lon=-5.3, rcs=30

// Kalman Filter Class (simplified)
class KalmanFilter {
public:
  KalmanFilter(float q=0.125f, float r=1.0f, float p=1.0f, float x=0.0f) {
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
private:
  float Q, R, P, K, X;
};

KalmanFilter kfSpeed(0.1f, 0.2f, 1.0f, 0.0f);        
KalmanFilter kfDistance(0.1f, 0.2f, 1.0f, 10.0f);     
KalmanFilter kfVelocity(0.15f, 0.25f, 1.0f, -5.4f);   

// Function Prototypes
void taskButtonSelect(void *parameter);
void taskButtonBack(void *parameter);
void taskUARTComm(void *parameter);
void taskSensorRead(void *parameter);
void taskCANSend(void *parameter);
void taskDisplay(void *parameter);
void initCAN();
void sendCANMessages();
void readSensors();
void updateDisplay();
void showAsTwoRightDigits(TM1637Display &disp, int value);
void broadcastBlinkState(bool state, bool force = false);

void setup() {
  // Inisialisasi Serial untuk debugging
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP1 - Radar Simulator with Menu System ===");
  
  // Inisialisasi Serial2 untuk komunikasi dengan ESP2 (baudrate sama dengan ESP2)
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("UART2 initialized for ESP2 communication");
  
  // Setup pin tombol
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  pinMode(BUTTON_BACK, INPUT);
  Serial.println("Button pins configured");
  
  // Setup LED indicator
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // LED off initially
  Serial.println("LED indicator configured (GPIO2)");
  
  // Setup ADC - Optimized for 3.3V VCC dengan linearitas maksimal
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  
  // ADC_11db range: 0-3.9V (covers full 3.3V VCC)
  // Namun di 3.3V hanya mencapai ~3465/4095 = 84.6% dari range
  // Untuk linearitas terbaik pada 3.3V, kita gunakan ADC_11db dengan kalibrasi manual
  analogSetAttenuation(ADC_11db);
  
  // Set attenuation per-pin untuk ADC1, ADC2 dan ADC3
  analogSetPinAttenuation(ADC_PIN, ADC_11db);   // GPIO35 - Speed (0-3.3V)
  analogSetPinAttenuation(ADC_PIN2, ADC_11db);  // GPIO34 - Distance (0-3.3V)
  analogSetPinAttenuation(ADC_PIN3, ADC_11db);  // GPIO36 - Velocity (0-3.3V)
  
  Serial.println("ADC configured: 12-bit, 11dB attenuation (0-3.9V hardware range)");
  Serial.println("Calibrated for 3.3V VCC potentiometer (0-3465 ADC counts)");
  
  // Initialize CAN Bus
  initCAN();
  
  // Initialize activeTargets with target_default (ID 0)
  activeTargets.push_back(0);
  Serial.println("Radar targets initialized with default target (ID 0)");
  
  // Initialize 7-Segment Displays
  display.setBrightness(7);
  display2.setBrightness(7);
  display3.setBrightness(7);
  display.clear();
  display2.clear();
  display3.clear();
  Serial.println("7-Segment displays initialized");
  
  // Create Mutex
  xDataMutex = xSemaphoreCreateMutex();
  if (xDataMutex == NULL) {
    Serial.println("ERROR: Failed to create mutex!");
    while(1);
  }
  
  // Create RTOS Tasks
  xTaskCreatePinnedToCore(taskButtonSelect, "ButtonSelect", 2048, NULL, 2, &taskButtonSelectHandle, 0);
  xTaskCreatePinnedToCore(taskButtonBack, "ButtonBack", 2048, NULL, 2, &taskButtonBackHandle, 0);
  xTaskCreatePinnedToCore(taskUARTComm, "UARTComm", 4096, NULL, 3, &taskUARTCommHandle, 1);
  xTaskCreatePinnedToCore(taskSensorRead, "SensorRead", 4096, NULL, 2, &taskSensorReadHandle, 1);
  xTaskCreatePinnedToCore(taskCANSend, "CANSend", 4096, NULL, 2, &taskCANSendHandle, 1);
  xTaskCreatePinnedToCore(taskDisplay, "Display", 2048, NULL, 1, &taskDisplayHandle, 0);
  
  Serial.println("RTOS Tasks created successfully");
  Serial.println("System ready. Waiting for mode selection from ESP2...\n");
}

void loop() {
  // Continuous SD Card initialization loop (keeps trying until successful)
  if (!sdInitialized) {
    Serial.println("[SD] Attempting to initialize SD Card...");
    
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
    delay(100); // Delay for SPI stabilization
    
    if (SD.begin(SD_CS, SPI)) {
      // Verify SD card is working by trying to open root
      File root = SD.open("/");
      if (root) {
        root.close();
        sdInitialized = true;
        sdCardWasPresent = true;
        lastSDCheckTime = millis();
        Serial.println("[SD] ✓ SD Card initialized successfully!");
      } else {
        Serial.println("[SD] ✗ SD Card mounted but cannot access root directory");
        SD.end();
        delay(2000); // Wait 2 seconds before retry
      }
    } else {
      Serial.println("[SD] ✗ SD Card not detected - retrying in 2 seconds...");
      delay(2000); // Wait 2 seconds before retry
    }
  } else {
    // SD Card already initialized, just delay
    delay(1000);
  }
}

// ========== CAN INITIALIZATION ==========
void initCAN() {
  Serial.println("Initializing TWAI (CAN)...");
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("ERROR: Failed to install TWAI driver!");
    while(1);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("ERROR: Failed to start TWAI!");
    while(1);
  }
  Serial.println("CAN Bus initialized successfully");
}

// ========== TASK: BUTTON SELECT ==========
void taskButtonSelect(void *parameter) {
  bool lastSelectState = HIGH;
  
  while (1) {
    bool currentSelectState = digitalRead(BUTTON_SELECT);
    
    if (lastSelectState == HIGH && currentSelectState == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
      if (digitalRead(BUTTON_SELECT) == LOW) {
        Serial2.println("SELECT");
        Serial.println("[BTN] SELECT pressed -> ESP2");
      }
    }
    
    lastSelectState = currentSelectState;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========== TASK: BUTTON BACK ==========
void taskButtonBack(void *parameter) {
  bool lastBackState = HIGH;
  
  while (1) {
    bool currentBackState = digitalRead(BUTTON_BACK);
    
    if (lastBackState == HIGH && currentBackState == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
      if (digitalRead(BUTTON_BACK) == LOW) {
        Serial2.println("BACK");
        Serial.println("[BTN] BACK pressed -> ESP2");
      }
    }
    
    lastBackState = currentBackState;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========== TASK: UART COMMUNICATION WITH ESP2 ==========
void taskUARTComm(void *parameter) {
  String rxBuffer = "";
  
  while (1) {
    // Read commands from ESP2
    while (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n') {
        rxBuffer.trim();
        
        // Parse commands from ESP2
        if (rxBuffer.startsWith("MODE:")) {
          String mode = rxBuffer.substring(5);
          if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
            if (mode == "MANUAL") {
              currentMode = MODE_MANUAL;
              radarActive = true;
              runningLEDMode = false;  // Matikan blinking saat radar aktif
              Serial.println("[UART] Mode: MANUAL - Radar ACTIVE");
            } else if (mode == "AUTO") {
              currentMode = MODE_AUTO;
              radarActive = true;
              runningLEDMode = false;  // Matikan blinking saat radar aktif
              Serial.println("[UART] Mode: AUTO - Radar ACTIVE");
            } else if (mode == "PLAY") {
              currentMode = MODE_PLAY;
              radarActive = true;
              runningLEDMode = false;  // Matikan blinking saat radar aktif
              Serial.println("[UART] Mode: PLAY - Radar ACTIVE");
            } else if (mode == "IDLE") {
              currentMode = MODE_IDLE;
              radarActive = false;
              runningLEDMode = true;  // Aktifkan blinking saat idle
              scenarioActive = false;  // Stop scenario when going idle
              waitingPlayReadyAck = false; // Reset state agar tidak resend ketika kembali idle
              playReadyResendCount = 0;
              lastPlayReadyMessage = "";
              broadcastBlinkState(false, true);  // Force immediate sync frame
              // Jika sedang playback, hentikan dan tutup file
              if (playActive) {
                playActive = false;
                if (playFile) { playFile.close(); }
                currentPlayFilename = "";
                Serial.println("[PLAY] Stopped due to MODE:IDLE");
              }
              Serial.println("[UART] Mode: IDLE - Radar INACTIVE");
            }
            xSemaphoreGive(xDataMutex);
          }
        }
        else if (rxBuffer.startsWith("UNIT:")) {
          uint8_t unitIdx = rxBuffer.substring(5).toInt();
          if (unitIdx < totalUnits) {
            if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
              selectedUnitIndex = unitIdx;
              Serial.printf("[UART] Unit selected: %s (index %d)\n", unitConfigs[unitIdx].name, unitIdx);
              xSemaphoreGive(xDataMutex);
            }
          }
        }
        else if (rxBuffer.startsWith("SIM_MODE:")) {
          String simMode = rxBuffer.substring(9);
          if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
            if (simMode == "PRE_COLLISION") {
              currentSimMode = SIM_PRE_COLLISION;
              scenarioActive = false;  // Disable scenario timing
              Serial.println("[UART] Simulation Mode: PRE_COLLISION");
            } else if (simMode == "SOFT_BRAKE") {
              currentSimMode = SIM_SOFT_BRAKE;
              scenarioActive = false;  // Disable scenario timing
              Serial.println("[UART] Simulation Mode: SOFT_BRAKE");
            } else if (simMode == "FULL_BRAKE") {
              currentSimMode = SIM_FULL_BRAKE;
              scenarioActive = false;  // Disable scenario timing
              Serial.println("[UART] Simulation Mode: FULL_BRAKE");
            } else if (simMode == "FULL_SCENARIO") {
              currentSimMode = SIM_FULL_SCENARIO;
              scenarioActive = true;   // Enable scenario timing
              scenarioStartTime = millis();  // Start scenario timer
              Serial.println("[UART] Simulation Mode: FULL_SCENARIO - Starting sequence");
            }
            xSemaphoreGive(xDataMutex);
          }
        }
        else if (rxBuffer == "SD_INIT") {
          // Attempt SD initialization with robust error handling
          Serial.println("[UART] SD_INIT received from ESP2");
          
          bool initSuccess = false;
          
          if (!sdInitialized) {
            // Initialize SPI bus
            SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
            delay(50); // Small delay for SPI stabilization
            
            // Try to initialize SD card
            if (SD.begin(SD_CS, SPI)) {
              // Verify SD card is actually working by trying to open root
              File root = SD.open("/");
              if (root) {
                root.close();
                sdInitialized = true;
                initSuccess = true;
                Serial.println("[SD] SD card initialized and verified successfully");
              } else {
                Serial.println("[SD] SD card mounted but cannot access root directory");
                SD.end();
              }
            } else {
              Serial.println("[SD] SD.begin() failed - card not detected or not formatted");
            }
          } else {
            // Already initialized, verify it's still working
            if (SD.exists("/")) {
              initSuccess = true;
              Serial.println("[SD] SD card already initialized and working");
            } else {
              // SD was initialized but now failing, try to reinitialize
              Serial.println("[SD] SD card lost connection, attempting reinitialization...");
              SD.end();
              sdInitialized = false;
              delay(100);
              
              SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
              delay(50);
              
              if (SD.begin(SD_CS, SPI)) {
                File root = SD.open("/");
                if (root) {
                  root.close();
                  sdInitialized = true;
                  initSuccess = true;
                  Serial.println("[SD] SD card reinitialized successfully");
                } else {
                  Serial.println("[SD] Reinitialization failed");
                }
              }
            }
          }
          
          // Send response to ESP2
          if (initSuccess) {
            Serial2.println("SD_OK");
            sdCardWasPresent = true; // Set flag for monitoring
            lastSDCheckTime = millis(); // Reset check timer
          } else {
            Serial2.println("SD_FAIL");
            sdCardWasPresent = false;
          }
        }
        else if (rxBuffer == "SD_MONITOR_START") {
          // Enable SD card monitoring (called by ESP2 after successful SD init)
          sdMonitoringEnabled = true;
          lastSDCheckTime = millis(); // Reset timer
          Serial.println("[SD] Monitoring ENABLED");
        }
        else if (rxBuffer == "SD_MONITOR_STOP") {
          // Disable SD card monitoring (called by ESP2 when exiting Rec mode)
          sdMonitoringEnabled = false;
          Serial.println("[SD] Monitoring DISABLED");
        }
        else if (rxBuffer.startsWith("REC_START:")) {
          // Start CSV recording with mode abbreviation (PC/SB/FB)
          String abbr = rxBuffer.substring(10); // after 'REC_START:'
          abbr.trim();
          
          // Validate abbreviation
          if (abbr != "PC" && abbr != "SB" && abbr != "FB") {
            Serial.printf("[REC] Invalid abbreviation: %s\n", abbr.c_str());
            Serial2.println("REC_ERR:INVALID_ABBR");
          } else if (!sdInitialized) {
            Serial.println("[REC] Cannot start - SD not initialized");
            Serial2.println("REC_ERR:SD_NOT_INIT");
          } else if (recordingActive) {
            Serial.println("[REC] Already recording - ignoring REC_START");
            Serial2.println("REC_ERR:ALREADY");
          } else {
            String path = getNextAvailableFilename(abbr);
            if (path.length() == 0) {
              Serial.println("[REC] No free filename (01..99) available");
              Serial2.println("REC_ERR:NO_SLOT");
            } else {
              File f = SD.open(path, FILE_WRITE);
              if (!f) {
                Serial.printf("[REC] Failed to open %s for write\n", path.c_str());
                Serial2.println("REC_ERR:OPEN_FAIL");
              } else {
                recFile = f;
                currentRecFilename = path; // store full path starting with '/'
                recordStartMillis = millis();
                // Schedule precise 100ms ticks starting at exactly t=0ms
                nextRecordWriteTime = recordStartMillis; // first write at start -> t_ms=0
                recordSampleIndex = 0;
                recordWriteCount = 0;
                latestRPM = 0.0f;
                recordingActive = true;
                writeCSVHeader();
                
                // Notify ESP2 immediately with base filename (without leading '/')
                String base = currentRecFilename.substring(1);
                Serial.printf("[REC] Started: %s -> Sending REC_STARTED:%s\n", currentRecFilename.c_str(), base.c_str());
                Serial2.print("REC_STARTED:");
                Serial2.println(base);
                Serial2.flush(); // Ensure message is sent immediately
              }
            }
          }
        }
        else if (rxBuffer == "REC_STOP") {
          // Stop recording (close file but keep it for save/delete decision)
          if (recordingActive) {
            recordingActive = false;
            if (recFile) {
              recFile.flush();
              recFile.close();
            }
            String base = currentRecFilename.length() ? currentRecFilename.substring(1) : String("FILE");
            Serial.printf("[REC] Stopped: %s -> Sending REC_STOPPED:%s\n", currentRecFilename.c_str(), base.c_str());
            Serial2.print("REC_STOPPED:");
            Serial2.println(base);
            Serial2.flush(); // Ensure message is sent immediately
          } else {
            Serial.println("[REC] REC_STOP received but not recording");
            Serial2.println("REC_ERR:NOT_ACTIVE");
          }
        }
        else if (rxBuffer == "REC_SAVE") {
          // Confirm save (nothing to do if file already closed and kept)
          if (currentRecFilename.length() > 0) {
            String base = currentRecFilename.substring(1);
            Serial.printf("[REC] Save confirmed: %s -> Sending REC_SAVED:%s\n", currentRecFilename.c_str(), base.c_str());
            Serial2.print("REC_SAVED:");
            Serial2.println(base);
            Serial2.flush(); // Ensure message is sent immediately
            // Clear filename after save confirmation
            currentRecFilename = "";
          } else {
            Serial.println("[REC] REC_SAVE received but no filename stored");
            Serial2.println("REC_ERR:NO_FILE");
          }
        }
        else if (rxBuffer == "REC_DELETE") {
          // Delete last recorded file if exists
          if (currentRecFilename.length() > 0) {
            String base = currentRecFilename.substring(1);
            if (SD.exists(currentRecFilename)) {
              SD.remove(currentRecFilename);
              Serial.printf("[REC] Deleted: %s -> Sending REC_DELETED:%s\n", currentRecFilename.c_str(), base.c_str());
              Serial2.print("REC_DELETED:");
              Serial2.println(base);
              Serial2.flush(); // Ensure message is sent immediately
            } else {
              Serial.printf("[REC] Delete requested but file not found: %s\n", currentRecFilename.c_str());
              Serial2.println("REC_ERR:DEL_NOT_FOUND");
            }
            currentRecFilename = "";
          } else {
            Serial.println("[REC] REC_DELETE received but no filename stored");
            Serial2.println("REC_ERR:NO_FILE");
          }
        }
        else if (rxBuffer.startsWith("RPM:")) {
          // Receive RPM from ESP2 during recording
          float rpm = rxBuffer.substring(4).toFloat();
          latestRPM = rpm;
        }
        else if (rxBuffer.startsWith("PLAY_LIST:")) {
          // ESP2 requests list of CSV files for Play mode
          String abbrev = rxBuffer.substring(10); // after 'PLAY_LIST:'
          abbrev.trim();
          
          Serial.printf("[PLAY] List request for mode: %s\n", abbrev.c_str());
          
          if (!sdInitialized) {
            Serial.println("[PLAY] SD not initialized - sending NO_FILES");
            Serial2.println("PLAY_NO_FILES");
          } else {
            // Scan SD card for files matching the pattern
            std::vector<String> matchingFiles;
            File root = SD.open("/");
            if (root) {
              File entry = root.openNextFile();
              while (entry) {
                String filename = entry.name();
                if (filename.endsWith(".csv") && filename.startsWith(abbrev + "-")) {
                  matchingFiles.push_back(filename);
                  Serial.printf("[PLAY] Found file: %s\n", filename.c_str());
                }
                entry.close();
                entry = root.openNextFile();
              }
              root.close();
              
              if (matchingFiles.size() == 0) {
                Serial.printf("[PLAY] No files found for %s\n", abbrev.c_str());
                Serial2.println("PLAY_NO_FILES");
              } else {
                // Sort files by number (PC-01, PC-02, etc.)
                std::sort(matchingFiles.begin(), matchingFiles.end());
                
                // Send file list to ESP2
                String response = "PLAY_FILES:" + String(matchingFiles.size());
                for (const String& file : matchingFiles) {
                  response += "," + file;
                }
                Serial2.println(response);
                Serial.printf("[PLAY] Sent %d files to ESP2\n", matchingFiles.size());
              }
            } else {
              Serial.println("[PLAY] Cannot open SD root directory");
              Serial2.println("PLAY_NO_FILES");
            }
          }
        }
        else if (rxBuffer.startsWith("PLAY_FILE:")) {
          // ESP2 requests to play a specific CSV file
          String filename = rxBuffer.substring(10); // after 'PLAY_FILE:'
          filename.trim();
          
          Serial.printf("[PLAY] File selected: %s\n", filename.c_str());
          
          if (!sdInitialized) {
            Serial.println("[PLAY] SD not initialized - cannot play file");
            Serial2.println("PLAY_ERR:SD_NOT_INIT");
          } else {
            String fullPath = "/" + filename;
            if (SD.exists(fullPath)) {
              Serial.printf("[PLAY] File exists, preparing playback: %s\n", fullPath.c_str());
              // Tutup playback sebelumnya jika masih aktif
              if (playActive) {
                playActive = false;
                if (playFile) { playFile.close(); }
              }
              // Buka file dan siapkan state replay
              playFile = SD.open(fullPath, FILE_READ);
              if (!playFile) {
                Serial.printf("[PLAY] Failed to open %s for read\n", fullPath.c_str());
                Serial2.println("PLAY_ERR:OPEN_FAIL");
              } else {
                currentPlayFilename = fullPath;
                playHeaderSkipped = false;
                currentPlayTimeMs = 0;
                playStartWall = millis();
                lastPlayTimeSend = 0;
                // Scan untuk total durasi
                playTotalMs = scanCSVMaxTime(fullPath);
                Serial.printf("[PLAY] Total duration: %lu ms\n", playTotalMs);

                if (playTotalMs == 0) {
                  Serial.println("[PLAY] CSV duration is 0 ms - aborting playback");
                  if (playFile) { playFile.close(); }
                  currentPlayFilename = "";
                  Serial2.println("PLAY_ERR:EMPTY_FILE");
                  Serial2.flush();
                } else {
                  playActive = true;
                  // Sertakan total durasi di payload agar ESP2 selalu tahu panjang file
                  String payload = "PLAY_READY:" + filename + "," + String(playTotalMs);
                  Serial2.println(payload);
                  Serial2.flush();
                  lastPlayReadyMessage = payload;
                  playReadyLastSent = millis();
                  waitingPlayReadyAck = true;
                  playReadyResendCount = 0;
                  Serial.printf("[PLAY] >>> Sent %s to ESP2\n", payload.c_str());
                  Serial.printf("[PLAY] Playback started: file=%s, total=%lu ms\n", filename.c_str(), playTotalMs);
                }
              }
            } else {
              Serial.printf("[PLAY] File not found: %s\n", fullPath.c_str());
              Serial2.println("PLAY_ERR:FILE_NOT_FOUND");
            }
          }
        }
        else if (rxBuffer == "PLAY_STOP") {
          // Stop current playback if any
          waitingPlayReadyAck = false; // Tidak perlu resend lagi setelah stop
          playReadyResendCount = 0;
          lastPlayReadyMessage = "";
          if (playActive) {
            playActive = false;
            if (playFile) { playFile.close(); }
            currentPlayFilename = "";
            Serial.println("[PLAY] Stopped by ESP2 request");
          }
        }
        else if (rxBuffer == "PLAY_ACK") {
          waitingPlayReadyAck = false;
          playReadyResendCount = 0;
          lastPlayReadyMessage = "";
          Serial.println("[PLAY] ACK received from ESP2");
        }
        else if (rxBuffer.startsWith("ADD_RADAR:")) {
          // Parse ADD_RADAR command: ADD_RADAR:id,velocity_lon,distance,rcs
          String params = rxBuffer.substring(10); // Remove "ADD_RADAR:"
          int comma1 = params.indexOf(',');
          int comma2 = params.indexOf(',', comma1 + 1);
          int comma3 = params.indexOf(',', comma2 + 1);
          
          if (comma1 != -1 && comma2 != -1 && comma3 != -1) {
            uint8_t id = params.substring(0, comma1).toInt();
            float vel_lon = params.substring(comma1 + 1, comma2).toFloat();
            float dist = params.substring(comma2 + 1, comma3).toFloat();
            float rcs = params.substring(comma3 + 1).toFloat();
            
            // Add target to activeTargets if not already present
            bool exists = false;
            for (auto existingId : activeTargets) {
              if (existingId == id) {
                exists = true;
                break;
              }
            }
            
            if (!exists && activeTargets.size() < 4) { // Max 4 targets (ID 0,1,2,3)
              activeTargets.push_back(id);
              
              // Update predefined target data if it's a static target
              if (id == 1) {
                target1.velocity_lon = vel_lon;
                target1.distance_lon = dist;
                target1.rcs = rcs;
              } else if (id == 2) {
                target2.velocity_lon = vel_lon;
                target2.distance_lon = dist;
                target2.rcs = rcs;
              } else if (id == 3) {
                target3.velocity_lon = vel_lon;
                target3.distance_lon = dist;
                target3.rcs = rcs;
              }
              
              Serial.printf("[UART] Target ID:%d added (VL:%.1f, Dist:%.0f, RCS:%.0f) - Active targets: %d\n", 
                           id, vel_lon, dist, rcs, activeTargets.size());
            } else if (exists) {
              Serial.printf("[UART] Target ID:%d already exists, skipping\n", id);
            } else {
              Serial.println("[UART] Maximum targets reached (4), skipping add");
            }
          }
        }
        else if (rxBuffer == "RESET_RADAR") {
          // Reset all targets except ID 0 (default)
          if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
            activeTargets.clear();
            activeTargets.push_back(0); // Always keep target_default (ID 0)
            Serial.println("[UART] All radar targets reset, only default target (ID 0) active");
            xSemaphoreGive(xDataMutex);
          }
        }
        else if (rxBuffer == "RUNNING") {
          // Perintah untuk mengaktifkan blinking mode
          if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
            if (!radarActive) {
              runningLEDMode = true;
              // Serial.println("[UART] Blinking Mode ACTIVE");  // Comment untuk mengurangi spam
            }
            xSemaphoreGive(xDataMutex);
          }
        }
        
        rxBuffer = "";
      } else {
        rxBuffer += c;
      }
    }
    
    // Resend PLAY_READY payload jika ACK belum diterima agar ESP2 selalu mendapatkan total durasi
    if (waitingPlayReadyAck && lastPlayReadyMessage.length() > 0) {
      unsigned long now = millis();
      if (now - playReadyLastSent >= PLAY_READY_RESEND_MS) {
        Serial2.println(lastPlayReadyMessage);
        Serial2.flush();
        playReadyLastSent = now;
        playReadyResendCount++;
        if (playReadyResendCount % PLAY_READY_WARN_INTERVAL == 0) {
          Serial.printf("[PLAY] Still waiting for ACK from ESP2 (resend #%u)\n", playReadyResendCount);
        }
      }
    }

    // Periodic SD card presence check (only if SD initialized AND monitoring enabled)
    if (sdInitialized && sdMonitoringEnabled) {
      unsigned long now = millis();
      if (now - lastSDCheckTime >= SD_CHECK_INTERVAL) {
        lastSDCheckTime = now;
        
        // Check if SD card is still present and accessible
        bool sdPresent = false;
        if (SD.exists("/")) {
          // SD card still present
          sdPresent = true;
          if (!sdCardWasPresent) {
            Serial.println("[SD] SD card detected again");
            sdCardWasPresent = true;
          }
        } else {
          // SD card missing or not accessible
          sdPresent = false;
          if (sdCardWasPresent) {
            Serial.println("[SD] SD card MISSING - notifying ESP2");
            Serial2.println("SD_MISSING");
            sdCardWasPresent = false;
            sdInitialized = false; // Reset init flag
            // Ensure recording stopped and file closed to prevent corruption
            if (recordingActive) {
              recordingActive = false;
              if (recFile) { recFile.flush(); recFile.close(); }
              Serial.println("[REC] Recording aborted due to SD missing");
            }
          }
        }
      }
    }

    // Periodic CSV logging every 100ms when recording active
    if (recordingActive && sdInitialized && recFile) {
      unsigned long now = millis();
      // Catch up if we missed intervals; bound to avoid long blocking
      int writesThisLoop = 0;
      while (recordingActive && (long)(now - nextRecordWriteTime) >= 0 && writesThisLoop < 5) {
        unsigned long t_ms = recordSampleIndex * 100UL; // exact 0,100,200,...

        // Read shared sensor values safely
        float spd_kmh, dist_m, vel_ms, rcs_val;
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
          spd_kmh = speed;                 // km/h
          dist_m = distance_lon;           // meters
          vel_ms = velocity_lon;           // m/s
          rcs_val = target.rcs;            // current target RCS
          xSemaphoreGive(xDataMutex);
        }

        // Debug logging to check what values are being written
        Serial.printf("[CSV] t=%lu, spd=%.2f, rpm=%.1f, dist=%.2f, vel=%.2f, rcs=%.1f\n",
                      t_ms, spd_kmh, latestRPM, dist_m, vel_ms, rcs_val);

        // Write CSV line
        recFile.print(t_ms);
        recFile.print(',');
        recFile.print(spd_kmh, 2);
        recFile.print(',');
        recFile.print(latestRPM, 1);
        recFile.print(',');
        recFile.print(dist_m, 2);
        recFile.print(',');
        recFile.print(vel_ms, 2);
        recFile.print(',');
        recFile.println(rcs_val, 1);

        // Update schedule for next tick
        recordSampleIndex++;
        nextRecordWriteTime += 100UL;
        writesThisLoop++;

        // Periodic flush to reduce data loss while limiting wear
        recordWriteCount++;
        if (recordWriteCount % 5 == 0) {
          recFile.flush();
        }
      }
    }
    
    // ===== CSV REPLAY LOOP =====
    if (playActive && playFile) {
      unsigned long now = millis();
      // Skip header on first run
      if (!playHeaderSkipped) {
        // Read first line and check header
        String header = playFile.readStringUntil('\n');
        header.trim();
        if (header.startsWith("time_ms")) {
          Serial.println("[PLAY] CSV header detected and skipped");
        } else {
          // If not a header, rewind to start of file
          playFile.seek(0);
          Serial.println("[PLAY] No header line, starting from beginning");
        }
        playHeaderSkipped = true;
        playStartWall = now;
        currentPlayTimeMs = 0;
        lastPlayTimeSend = 0;
      }

      // Advance through CSV lines up to target elapsed
      unsigned long targetElapsed = now - playStartWall;
      // Process up to a few lines per loop to avoid long blocking
      int processed = 0;
      while (processed < 5 && playFile.available()) {
        // Peek next line without losing data if time is in the future
        size_t posBefore = playFile.position();
        String line = playFile.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) {
          processed++;
          continue; // skip empty
        }
        // Parse CSV: time_ms,speed_kmh,RPM,dist,vel,rcs
        int c1 = line.indexOf(',');
        if (c1 == -1) { processed++; continue; }
        unsigned long t_ms = (unsigned long) line.substring(0, c1).toInt();
        if (t_ms > targetElapsed) {
          // Not yet time to render this row, rewind and break
          playFile.seek(posBefore);
          break;
        }
        // We can consume this row: parse speed, RPM, and radar data
        int c2 = line.indexOf(',', c1 + 1);
        int c3 = line.indexOf(',', c2 + 1);
        int c4 = line.indexOf(',', c3 + 1);
        int c5 = line.indexOf(',', c4 + 1);
        
        float spd = 0.0f;
        float rpm = 0.0f;
        float dist = 0.0f;
        float vel = 0.0f;
        float rcs_val = 0.0f;
        
        if (c2 != -1) {
          spd = line.substring(c1 + 1, c2).toFloat();
        }
        if (c3 != -1) {
          rpm = line.substring(c2 + 1, c3).toFloat();
        }
        if (c4 != -1) {
          dist = line.substring(c3 + 1, c4).toFloat();
        }
        if (c5 != -1) {
          vel = line.substring(c4 + 1, c5).toFloat();
          rcs_val = line.substring(c5 + 1).toFloat();
        }
        
        currentPlayTimeMs = t_ms;
        
        // Send time sync to ESP2 on each consumed row
        Serial2.print("PLAY_TIME:");
        Serial2.println(currentPlayTimeMs);
        
        // Send speed and RPM to ESP2 for CAN/7-seg display
        Serial2.print("SPD:");
        Serial2.println(spd, 1);
        
        Serial2.print("RPM:");
        Serial2.println((int)rpm);
        
        // Debug logging untuk first row dan setiap 1 detik
        static unsigned long lastDebugLog = 0;
        if (t_ms == 0 || (millis() - lastDebugLog > 1000)) {
          Serial.printf("[PLAY] t=%lu ms, SPD=%.1f, RPM=%d -> Sent to ESP2\n", t_ms, spd, (int)rpm);
          lastDebugLog = millis();
        }
        
        // Update radar data untuk ditampilkan di 7-segment ESP1
        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(5))) {
          speed = spd;
          distance_lon = dist;
          velocity_lon = vel;
          target.distance_lon = dist;
          target.velocity_lon = vel;
          target.rcs = rcs_val;
          xSemaphoreGive(xDataMutex);
        }
        
        processed++;
      }

      // If end of file reached, stop playback
      if (!playFile.available()) {
        Serial.println("[PLAY] Reached end of file, stopping playback");
        // Send final PLAY_TIME to ensure countdown reaches 00:00:00
        Serial2.println("PLAY_TIME:" + String(playTotalMs));
        Serial2.flush();
        delay(50);
        Serial2.println("PLAY_END");
        Serial2.flush();
        Serial.printf("[PLAY] >>> Sent PLAY_TIME:%lu and PLAY_END to ESP2\n", playTotalMs);
        playActive = false;
        playFile.close();
        currentPlayFilename = "";
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ========== TASK: SENSOR READING ==========
void taskSensorRead(void *parameter) {
  const TickType_t readInterval = pdMS_TO_TICKS(50); // Read every 50ms
  
  while (1) {
    bool isRadarActive = false;
    bool isRecording = false;
    bool isPlaying = false;
    
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
      isRadarActive = radarActive;
      isRecording = recordingActive;
      isPlaying = playActive; // Check if in playback mode
      xSemaphoreGive(xDataMutex);
    }
    
    // Read sensors if radar is active OR if recording is active (for CSV data)
    // BUT skip if currently playing back CSV (data comes from CSV, not ADC)
    if ((isRadarActive || isRecording) && !isPlaying) {
      readSensors();
    }
    
    vTaskDelay(readInterval);
  }
}

// ========== READ SENSORS FUNCTION ==========
void readSensors() {
  // Check if AUTO mode - use simulation data instead of ADC
  SystemMode mode;
  SimulationMode simMode;
  uint8_t unitIdx;
  
  if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
    mode = currentMode;
    simMode = currentSimMode;
    unitIdx = selectedUnitIndex;
    xSemaphoreGive(xDataMutex);
  }
  
  if (mode == MODE_AUTO) {
    // AUTO mode: Use simulation data based on selected simulation mode
    float sim_speed = 14.0f;        // Fixed speed: 14 km/h
    float sim_velocity_lon = -5.3f; // Fixed velocity: -5.3 m/s
    float sim_distance_lon;         // Variable distance based on mode
    
    // Handle Full Scenario timing
    if (simMode == SIM_FULL_SCENARIO) {
      unsigned long elapsed = millis() - scenarioStartTime;
      
      if (elapsed < 5000) {
        // 0-5 seconds: Pre-Collision phase
        sim_distance_lon = 10.0f;
        // Debug info every 1 second
        static unsigned long lastDebugTime = 0;
        if (millis() - lastDebugTime >= 1000) {
          Serial.printf("[SCENARIO] Pre-Collision phase: %.1fs/5.0s\n", elapsed / 1000.0f);
          lastDebugTime = millis();
        }
      } else if (elapsed < 10000) {
        // 5-10 seconds: Soft Brake phase
        sim_distance_lon = 5.0f;
        // Debug info every 1 second
        static unsigned long lastDebugTime2 = 0;
        if (millis() - lastDebugTime2 >= 1000) {
          Serial.printf("[SCENARIO] Soft Brake phase: %.1fs/5.0s\n", (elapsed - 5000) / 1000.0f);
          lastDebugTime2 = millis();
        }
      } else {
        // 10+ seconds: Full Brake phase (continues until BACK)
        sim_distance_lon = 1.0f;
        // Debug info every 2 seconds in full brake
        static unsigned long lastDebugTime3 = 0;
        if (millis() - lastDebugTime3 >= 2000) {
          Serial.printf("[SCENARIO] Full Brake phase: %.1fs (continuous)\n", (elapsed - 10000) / 1000.0f);
          lastDebugTime3 = millis();
        }
      }
    } else {
      // Static simulation modes
      // Set distance based on simulation mode
      switch (simMode) {
        case SIM_PRE_COLLISION:
          sim_distance_lon = 10.0f;  // 10 meters
          break;
        case SIM_SOFT_BRAKE:
          sim_distance_lon = 5.0f;   // 5 meters
          break;
        case SIM_FULL_BRAKE:
          sim_distance_lon = 1.0f;   // 1 meter
          break;
        default:
          sim_distance_lon = 10.0f;  // Default to pre-collision
          break;
      }
    }
    
    // Update shared variables with simulation data
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
      speed = sim_speed;
      distance_lon = sim_distance_lon;
      velocity_lon = sim_velocity_lon;
      target.distance_lon = sim_distance_lon;
      target.velocity_lon = sim_velocity_lon;
      // Ensure target.rcs has a valid value for CSV logging
      if (target.rcs <= 0.0f) {
        target.rcs = 20.0f; // Default RCS value
      }
      xSemaphoreGive(xDataMutex);
    }
    
    // Send speed data to ESP2 via UART2 (for RPM calculation)
    static unsigned long lastSpeedSend = 0;
    unsigned long now = millis();
    if (now - lastSpeedSend >= 100) {  // Send every 100ms
      Serial2.printf("SPD:%.1f\n", sim_speed);
      lastSpeedSend = now;
    }
    
    return; // Skip ADC reading for AUTO mode
  }
  
  // MANUAL/PLAY mode: Read from ADC as before
  // Read ADC (Speed), ADC2 (Object Distance), ADC3 (Velocity) - multiple samples for stability
  const int samples = 5;
  int adc1_sum = 0, adc2_sum = 0, adc3_sum = 0;
  
  for (int i = 0; i < samples; i++) {
    adc1_sum += analogRead(ADC_PIN);
    adc2_sum += analogRead(ADC_PIN2);
    adc3_sum += analogRead(ADC_PIN3);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  
  int raw1 = adc1_sum / samples;
  int raw2 = adc2_sum / samples;
  int raw3 = adc3_sum / samples;
  
  // ===== KALIBRASI SAMA DENGAN lengkapRTOS.ino =====
  // Menggunakan full ADC range (0-4095) seperti di lengkapRTOS.ino untuk konsistensi
  const float ADC_MAX_RANGE = 4095.0f;  // Full 12-bit ADC range
  
  // Convert ADC1 to speed - SAMA dengan lengkapRTOS.ino
  // Formula: (raw1 / 4095.0f) * SPEED_MAX * current_config.speed_calibration
  UnitConfig current_config = unitConfigs[unitIdx];
  float speed_raw = (raw1 / ADC_MAX_RANGE) * SPEED_MAX * current_config.speed_calibration;
  // Clamp untuk mencegah overshoot
  if (speed_raw > SPEED_MAX) speed_raw = SPEED_MAX;
  if (speed_raw < 0.0f) speed_raw = 0.0f;
  float speed_filtered = kfSpeed.update(speed_raw);
  
  // Convert ADC2 to distance (1-80 meters) - SAMA dengan lengkapRTOS.ino
  // Formula: 1.0f + (raw2 / 4095.0f) * 79.0f
  float dist_raw = 1.0f + (raw2 / ADC_MAX_RANGE) * 79.0f;
  // Clamp untuk mencegah overshoot
  if (dist_raw > 80.0f) dist_raw = 80.0f;
  if (dist_raw < 1.0f) dist_raw = 1.0f;
  float dist_filtered = kfDistance.update(dist_raw);
  
  // Convert ADC3 to velocity (-20.0 to 5.0 m/s) - SAMA dengan lengkapRTOS.ino
  // Formula: (raw3 / 4095.0f) * 25.0f - 20.0f
  float vel_raw = (raw3 / ADC_MAX_RANGE) * 25.0f - 20.0f;
  // Clamp untuk mencegah overshoot
  if (vel_raw > 5.0f) vel_raw = 5.0f;
  if (vel_raw < -20.0f) vel_raw = -20.0f;
  float vel_filtered = kfVelocity.update(vel_raw);
  
  // Update shared variables
  if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
    speed = speed_filtered;
    distance_lon = dist_filtered;
    velocity_lon = vel_filtered;
    target.distance_lon = dist_filtered;
    target.velocity_lon = vel_filtered;
    // Ensure target.rcs has a valid value for CSV logging (from best active target or default)
    if (activeTargets.size() > 0) {
      // Use RCS from first active target
      uint8_t firstActiveId = activeTargets[0];
      if (firstActiveId == 1) target.rcs = target1.rcs;
      else if (firstActiveId == 2) target.rcs = target2.rcs;
      else if (firstActiveId == 3) target.rcs = target3.rcs;
      else target.rcs = 20.0f; // Default
    } else {
      target.rcs = 20.0f; // Default RCS when no targets are active
    }
    xSemaphoreGive(xDataMutex);
  }
  
  // Send speed data to ESP2 via UART2 (efficient format)
  static unsigned long lastSpeedSend = 0;
  static unsigned long lastRadarInfoSend = 0;
  unsigned long now = millis();
  if (now - lastSpeedSend >= 100) {  // Send every 100ms
    Serial2.printf("SPD:%.1f\n", speed_filtered);
    lastSpeedSend = now;
  }
  
  // Send current radar info to ESP2 every 500ms (for display update)
  if (now - lastRadarInfoSend >= 500) {
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
      // Find best target (closest distance, highest RCS if same distance)
      uint8_t currentRadarId = 0;
      float currentRadarRcs = target.rcs;
      
      // Check if there are other active targets with better priority
      for (auto id : activeTargets) {
        RadarTarget* checkTarget = nullptr;
        if (id == 1) checkTarget = &target1;
        else if (id == 2) checkTarget = &target2;
        else if (id == 3) checkTarget = &target3;
        
        if (checkTarget && 
            (checkTarget->distance_lon < target.distance_lon || 
             (checkTarget->distance_lon == target.distance_lon && checkTarget->rcs > target.rcs))) {
          currentRadarId = id;
          currentRadarRcs = checkTarget->rcs;
        }
      }
      
      Serial2.printf("RADAR_INFO:%d,%.0f\n", currentRadarId, currentRadarRcs);
      xSemaphoreGive(xDataMutex);
    }
    lastRadarInfoSend = now;
  }
}

// ========== TASK: CAN MESSAGE SENDING ==========
void taskCANSend(void *parameter) {
  const TickType_t sendInterval = pdMS_TO_TICKS(60); // Send every 60ms (radar interval)
  const TickType_t hbInterval = pdMS_TO_TICKS(1000); // Heartbeat every 1s
  
  TickType_t lastRadarSend = xTaskGetTickCount();
  TickType_t lastHeartbeat = xTaskGetTickCount();
  
  while (1) {
    TickType_t now = xTaskGetTickCount();
    
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
      bool active = radarActive;
      xSemaphoreGive(xDataMutex);
      
      if (active) {
        // Send radar messages every 60ms
        if ((now - lastRadarSend) >= sendInterval) {
          sendCANMessages();
          lastRadarSend = now;
        }
        
        // Send heartbeat every 1s
        if ((now - lastHeartbeat) >= hbInterval) {
          sendHeartbeat();
          lastHeartbeat = now;
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========== SEND CAN MESSAGES WITH MULTI-TARGET SUPPORT ==========
void sendCANMessages() {
  // Build and send radar messages with multiple targets (sama dengan lengkapRTOS.ino)
  
  // 60A: Status (jumlah target, dsb)
  twai_message_t status_msg = {};
  status_msg.identifier = 0x60A;
  status_msg.extd = 0;
  status_msg.data_length_code = 8;
  meas_count++;
  status_msg.data[0] = activeTargets.size(); // Number of active targets
  status_msg.data[1] = (meas_count >> 8) & 0xFF;
  status_msg.data[2] = meas_count & 0xFF;
  status_msg.data[3] = (interface_version & 0x0F);
  for (int i = 4; i < 8; ++i) status_msg.data[i] = 0;
  twai_transmit(&status_msg, pdMS_TO_TICKS(10));
  
  // Prepare active targets data
  std::vector<RadarTarget> activeTargetsData;
  
  // Always include target_default (dynamic from ADC)
  RadarTarget defaultTarget = target; // target_default dinamis
  defaultTarget.id = 0;
  activeTargetsData.push_back(defaultTarget);
  
  // Add other active targets from trigger (skip ID 0 karena sudah default)
  for (auto id : activeTargets) {
    if (id == 1) {
      RadarTarget currentTarget = target1;
      currentTarget.id = 1;
      activeTargetsData.push_back(currentTarget);
    } else if (id == 2) {
      RadarTarget currentTarget = target2;
      currentTarget.id = 2;
      activeTargetsData.push_back(currentTarget);
    } else if (id == 3) {
      RadarTarget currentTarget = target3;
      currentTarget.id = 3;
      activeTargetsData.push_back(currentTarget);
    }
  }
  
  // Find best target (lowest distance, if same then highest RCS)
  size_t bestIndex = 0;
  for (size_t i = 1; i < activeTargetsData.size(); ++i) {
    if (
      (activeTargetsData[i].distance_lon < activeTargetsData[bestIndex].distance_lon) ||
      (activeTargetsData[i].distance_lon == activeTargetsData[bestIndex].distance_lon && activeTargetsData[i].rcs > activeTargetsData[bestIndex].rcs)
    ) {
      bestIndex = i;
    }
  }
  
  // Limit to maximum 4 active targets (ID 0,1,2,3)
  if (activeTargetsData.size() > 4) {
    // Sort by priority and take 4 best
    std::sort(activeTargetsData.begin(), activeTargetsData.end(), [](const RadarTarget& a, const RadarTarget& b) {
      if (a.distance_lon != b.distance_lon) {
        return a.distance_lon < b.distance_lon; // Closest distance first
      } else {
        return a.rcs > b.rcs; // Highest RCS first
      }
    });
    // Take 4 best targets
    activeTargetsData.resize(4);
    // Re-assign ID 0..3
    for (size_t i = 0; i < activeTargetsData.size(); ++i) {
      activeTargetsData[i].id = i;
    }
  }
  
  // Ensure best target is always ID 0
  if (bestIndex != 0) {
    uint8_t tempId = activeTargetsData[bestIndex].id;
    activeTargetsData[bestIndex].id = 0;
    activeTargetsData[0].id = tempId;
    std::swap(activeTargetsData[0], activeTargetsData[bestIndex]);
  }
  
  // Sort by ID for transmission (0 first, then 1, 2, ...)
  std::sort(activeTargetsData.begin(), activeTargetsData.end(), [](const RadarTarget& a, const RadarTarget& b) {
    return a.id < b.id;
  });
  
  // Burst: Send all targets in ID order (0 first, then 1, 2, ...)
  for (size_t i = 0; i < activeTargetsData.size(); ++i) {
    RadarTarget sendTarget = activeTargetsData[i];
    
    // Set sensorID for compatibility (sama dengan lengkapRTOS.ino)
    uint8_t sensorID = sendTarget.id;
    
    // Debug output per target (sama dengan lengkapRTOS.ino)
    Serial.printf("Sending Target ID: %d, DIST=%.2f, VEL_LON=%.2f, RCS=%.1f\n", 
                  sendTarget.id, sendTarget.distance_lon, sendTarget.velocity_lon, sendTarget.rcs);
    
    // 60B: Object General Info
    uint16_t dist_lon_raw = (uint16_t)round((sendTarget.distance_lon + 500.0f) / 0.2f);
    uint16_t dist_lat_raw = (uint16_t)round((sendTarget.distance_lat + 204.6f) / 0.2f);
    uint16_t vrel_lon_raw = (uint16_t)round((sendTarget.velocity_lon + 128.0f) / 0.25f);
    uint16_t vrel_lat_raw = (uint16_t)round((sendTarget.velocity_lat + 64.0f) / 0.25f);
    
    twai_message_t general_msg = {};
    general_msg.identifier = 0x60B;
    general_msg.extd = 0;
    general_msg.data_length_code = 8;
    general_msg.data[0] = sendTarget.id & 0xFF;
    general_msg.data[1] = (dist_lon_raw >> 5) & 0xFF;
    general_msg.data[2] = (uint8_t)(((dist_lon_raw & 0x1F) << 3) | ((dist_lat_raw >> 8) & 0x07));
    general_msg.data[3] = dist_lat_raw & 0xFF;
    general_msg.data[4] = (vrel_lon_raw >> 2) & 0xFF;
    general_msg.data[5] = (uint8_t)(((vrel_lon_raw & 0x03) << 6) | ((vrel_lat_raw >> 3) & 0x3F));
    general_msg.data[6] = (uint8_t)(((vrel_lat_raw & 0x07) << 5) | (sendTarget.dynamic_prop & 0x07));
    general_msg.data[7] = 0;
    twai_transmit(&general_msg, pdMS_TO_TICKS(10));
    
    // 60C: Object Other Info
    uint32_t rcs_raw = 0, snr_raw = 0;
    memcpy(&rcs_raw, &sendTarget.rcs, sizeof(float));
    memcpy(&snr_raw, &sendTarget.snr, sizeof(float));
    
    twai_message_t other_msg = {};
    other_msg.identifier = 0x60C;
    other_msg.extd = 0;
    other_msg.data_length_code = 8;
    other_msg.data[0] = (uint8_t)(rcs_raw & 0xFF);
    other_msg.data[1] = (uint8_t)((rcs_raw >> 8) & 0xFF);
    other_msg.data[2] = (uint8_t)((rcs_raw >> 16) & 0xFF);
    other_msg.data[3] = (uint8_t)((rcs_raw >> 24) & 0xFF);
    other_msg.data[4] = (uint8_t)(snr_raw & 0xFF);
    other_msg.data[5] = (uint8_t)((snr_raw >> 8) & 0xFF);
    other_msg.data[6] = (uint8_t)((snr_raw >> 16) & 0xFF);
    other_msg.data[7] = (uint8_t)((snr_raw >> 24) & 0xFF);
    twai_transmit(&other_msg, pdMS_TO_TICKS(10));
  }
}

// ========== SEND HEARTBEAT ==========
void sendHeartbeat() {
  twai_message_t hb_msg = {};
  hb_msg.identifier = 0x700;
  hb_msg.extd = 0;
  hb_msg.data_length_code = 8;
  hb_msg.data[0] = 0x02;
  hb_msg.data[1] = 0x00;
  hb_msg.data[2] = 0x10;
  for (int i = 3; i < 8; i++) hb_msg.data[i] = 0;
  twai_transmit(&hb_msg, pdMS_TO_TICKS(10));
}

// ========== TASK: DISPLAY UPDATE ==========
void taskDisplay(void *parameter) {
  const TickType_t updateInterval = pdMS_TO_TICKS(100); // Update every 100ms
  
  while (1) {
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
      // Selalu update display (baik radar aktif maupun blinking mode)
      updateDisplay();
      xSemaphoreGive(xDataMutex);
    }
    
    vTaskDelay(updateInterval);
  }
}

// ========== SHOW AS TWO RIGHT DIGITS FUNCTION ==========
void showAsTwoRightDigits(TM1637Display &disp, int value) {
  if (value > 99) value = 99;
  if (value < -99) value = -99;

  int absv = abs(value);
  int tens = (absv / 10) % 10;
  int units = absv % 10;

  // segment buffer (4 digits)
  uint8_t segs[4] = {0x00, 0x00, 0x00, 0x00};

  if (value < 0) {
    segs[0] = 0x00;                 
    segs[1] = 0x40;              // Minus sign
    segs[2] = disp.encodeDigit(tens);  // tens (jika <10 akan jadi 0)
    segs[3] = disp.encodeDigit(units);
  } else {
    segs[0] = 0x00;
    segs[1] = 0x00;
    segs[2] = disp.encodeDigit(tens); 
    segs[3] = disp.encodeDigit(units);
  }

  disp.setSegments(segs, 4, 0);
}

// Broadcast current blink state to ESP2 so both 7-seg displays stay in phase
void broadcastBlinkState(bool state, bool force) {
  static bool lastState = false;
  static unsigned long lastSent = 0;
  unsigned long now = millis();
  uint16_t stamp = static_cast<uint16_t>(now & 0xFFFF);

  if (force || state != lastState || (now - lastSent) >= 1000UL) {
    Serial2.printf("BLINK:%d,%u\n", state ? 1 : 0, stamp);
    lastState = state;
    lastSent = now;
  }
}

// ========== UPDATE DISPLAY FUNCTION ==========
void updateDisplay() {
  static bool blinkState = false;          // Current blink phase
  static unsigned long lastBlinkUpdate = 0;
  static bool blinkModeActive = false;     // Tracks entry into blink mode
  static int blink_debug_counter = 0;      // Rate-limit logging

  // Blinking "0000" Mode: Semua digit menampilkan "0" berkedip bersamaan
  if (runningLEDMode && !radarActive) {
    unsigned long currentTime = millis();
    
    if (!blinkModeActive) {
      blinkModeActive = true;
      blink_debug_counter = 0;
      blinkState = false;
      lastBlinkUpdate = currentTime;
      broadcastBlinkState(blinkState, true);
    }

    // Toggle state setiap 500ms (blinking)
    if (currentTime - lastBlinkUpdate >= 500) {
      lastBlinkUpdate = currentTime;
      blinkState = !blinkState;
      broadcastBlinkState(blinkState, false);
    }
    
    uint8_t segs1[4] = {0x00, 0x00, 0x00, 0x00};  // Display (Speed)
    uint8_t segs2[4] = {0x00, 0x00, 0x00, 0x00};  // Display2 (Distance)
    uint8_t segs3[4] = {0x00, 0x00, 0x00, 0x00};  // Display3 (Velocity)
    
    if (blinkState) {
      // State ON: Tampilkan "0000" di semua digit
      uint8_t zero_digit = display.encodeDigit(0);  // Encode digit "0"
      
      segs1[0] = zero_digit;
      segs1[1] = zero_digit;
      segs1[2] = zero_digit;
      segs1[3] = zero_digit;
      
      segs2[0] = zero_digit;
      segs2[1] = zero_digit;
      segs2[2] = zero_digit;
      segs2[3] = zero_digit;
      
      segs3[0] = zero_digit;
      segs3[1] = zero_digit;
      segs3[2] = zero_digit;
      segs3[3] = zero_digit;
    }
    // State OFF: Semua digit mati (sudah 0x00)
    
    display.setSegments(segs1, 4, 0);
    display2.setSegments(segs2, 4, 0);
    display3.setSegments(segs3, 4, 0);
    
    if (blink_debug_counter++ % 40 == 0) {
      Serial.println("7-Seg Display: Blinking mode active (master phase)");
    }

    return;  // Keluar dari fungsi, tidak perlu tampilkan data radar
  }
  else {
    if (blinkModeActive) {
      blinkModeActive = false;
      blink_debug_counter = 0;
    }
  }
  
  // Normal radar display mode
  // Display Speed on display (0-SPEED_MAX km/h, show as integer, right-aligned)
  int speed_display = max(-99, min(99, (int)round(speed))); // clamp -99..99
  showAsTwoRightDigits(display, speed_display);
  
  // Display Distance on display2 (0-80m, show as integer, right-aligned)
  int dist_display = max(0, min(99, (int)round(distance_lon))); // clamp 0..99
  showAsTwoRightDigits(display2, dist_display);
  
  // Display Velocity on display3 (-20.0 to 5.0 m/s, format: -XX.X)
  float vel = velocity_lon;
  if (vel > 5.0f) vel = 5.0f;
  if (vel < -20.0f) vel = -20.0f;
  
  uint8_t segs3[4] = {0x00, 0x00, 0x00, 0x00};
  bool is_negative = (vel < 0.0f);
  
  // Round to 1 decimal place first
  float rounded_vel = roundf(fabs(vel) * 10.0f) / 10.0f;
  
  // Extract integer and fractional parts after rounding
  int int_part = (int)rounded_vel;
  int frac_part = (int)roundf((rounded_vel - int_part) * 10.0f);
  
  // Handle edge case where rounding pushes frac to 10 (e.g., 4.95 -> 5.0 or 19.95 -> 20.0)
  if (frac_part >= 10) {
    int_part++;
    frac_part = 0;
  }
  
  // Clamp int_part to valid range after potential increment
  if (int_part > 20) int_part = 20;
  if (int_part < 0) int_part = 0;
  
  int tens_v = int_part / 10;
  int units_v = int_part % 10;
  int decimal = frac_part % 10;
  
  if (is_negative) {
    segs3[0] = 0x40; // Minus sign
  } else {
    segs3[0] = 0x00; // Blank
  }
  
  segs3[1] = display3.encodeDigit(tens_v);
  segs3[2] = display3.encodeDigit(units_v) | 0x80; // With decimal point
  segs3[3] = display3.encodeDigit(decimal);
  
  display3.setSegments(segs3, 4, 0);
}
