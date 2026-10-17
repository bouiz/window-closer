#include <Arduino.h>
#include <driver/i2s.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

/*
  Actuator + microphone test, controllable over USB serial or Bluetooth (BLE).
  - Actuator via BTS7960. The actuator has built-in end-of-stroke switches,
    so the timer here is only a cutoff that releases the driver after a full stroke.
  - INMP441 over I2S: prints sound level so you can see what a train looks like
    compared to background noise, and can optionally close on a loud sustained sound.
  - BLE uses the Nordic UART Service: the phone writes the same one-letter commands
    as the serial monitor, and receives the same messages plus a compact level line.
  Board: ESP32 Dev Module
*/

// --- Actuator pins ---
const int RPWM = 27;
const int LPWM = 26;
const int EN   = 25;

// --- Microphone pins (INMP441) ---
const int MIC_SCK = 21;
const int MIC_WS  = 22;
const int MIC_SD  = 19;

// --- PWM config ---
const int PWM_FREQ = 20000;   // 20 kHz — above audible range
const int PWM_RES  = 8;       // 8-bit duty, 0-255
const int PWM_CH_R = 0;
const int PWM_CH_L = 1;

// 150 mm stroke @ 15 mm/s = 10 s full travel. The built-in limit switches cut
// the motor at each end, so a bit of extra time is harmless.
const unsigned long STROKE_TIME_MS = 11000;

// --- I2S / audio config ---
const i2s_port_t I2S_PORT   = I2S_NUM_0;
const int SAMPLE_RATE       = 16000;
const int BLOCK_FRAMES      = 1600;   // 100 ms per level reading
// Both I2S slots are read (stereo). The INMP441 only talks in one of them
// (picked by its L/R pin), but the ESP32 driver is known to swap left/right,
// so the active slot is detected at startup instead of hard-coded.
int32_t samples[BLOCK_FRAMES * 2];   // interleaved: slot 0, slot 1, slot 0, ...
int micSlot = 0;

// --- Auto-close trigger (crude loudness test — a real classifier comes later) ---
float thresholdDb = -30.0;            // dBFS; tune with '+' / '-'
const int TRIGGER_BLOCKS = 15;        // must stay above threshold this many blocks (1.5 s)
bool autoArmed  = false;
bool printLevel = true;
int  loudBlocks = 0;

// --- Bluetooth (Nordic UART Service) ---
const char *BLE_NAME   = "WindowCloser";
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> ESP32
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 -> phone
const size_t BLE_CHUNK = 20;          // fits the default BLE packet size

BLECharacteristic *bleTx = nullptr;
volatile bool bleConnected = false;
QueueHandle_t bleCommands;            // command chars written by the phone

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override { bleConnected = true; }
  void onDisconnect(BLEServer *server) override {
    bleConnected = false;
    BLEDevice::startAdvertising();    // let the phone reconnect
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  // Runs in the BLE task, so just queue the chars for loop() to handle
  void onWrite(BLECharacteristic *c) override {
    std::string value = c->getValue();
    for (char ch : value) xQueueSend(bleCommands, &ch, 0);
  }
};

// Sends one line to the phone, split into packets; the phone joins them back on '\n'
void bleSendLine(const char *line) {
  if (!bleConnected) return;
  size_t len = strlen(line);
  for (size_t i = 0; i < len; i += BLE_CHUNK) {
    size_t n = min(BLE_CHUNK, len - i);
    bleTx->setValue((uint8_t *)line + i, n);
    bleTx->notify();
  }
}

// Prints to USB serial and, line by line, to the phone
class DualOutput : public Print {
  char line[160];
  size_t len = 0;
public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (c == '\r') return 1;
    line[len++] = c;
    if (c == '\n' || len == sizeof(line) - 2) {
      if (c != '\n') line[len++] = '\n';
      line[len] = '\0';
      bleSendLine(line);
      len = 0;
    }
    return 1;
  }
};
DualOutput out;

void setupBle() {
  bleCommands = xQueueCreate(32, sizeof(char));
  BLEDevice::init(BLE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(NUS_SERVICE_UUID);
  bleTx = service->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  BLECharacteristic *rx = service->createCharacteristic(
      NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);         // device name goes in the scan response
  BLEDevice::startAdvertising();
}

// Next command char from USB serial or the phone, or -1 if none is waiting
int readCommand() {
  if (Serial.available()) return Serial.read();
  char c;
  if (xQueueReceive(bleCommands, &c, 0) == pdTRUE) return c;
  return -1;
}

// Motion state for the phone UI: 0 = idle, 1 = extending, 2 = retracting
void sendMotion(int state) {
  char msg[8];
  snprintf(msg, sizeof(msg), "M,%d\n", state);
  bleSendLine(msg);
}

void stopActuator() {
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);
}

// Runs for STROKE_TIME_MS, or stops early if 's' arrives (serial or phone)
bool runTimed(int channelOn, int channelOff) {
  unsigned long start = millis();
  ledcWrite(channelOff, 0);
  ledcWrite(channelOn, 255);
  while (millis() - start < STROKE_TIME_MS) {
    if (readCommand() == 's') {
      stopActuator();
      out.println("Manual stop.");
      sendMotion(0);
      return false;
    }
    delay(5);                         // lets the BLE task run
  }
  stopActuator();
  sendMotion(0);
  return true;
}

void extendActuator() {
  out.println("Extending... (send 's' to stop early)");
  sendMotion(1);
  if (runTimed(PWM_CH_R, PWM_CH_L)) out.println("Extend finished.");
}

void retractActuator() {
  out.println("Retracting... (send 's' to stop early)");
  sendMotion(2);
  if (runTimed(PWM_CH_L, PWM_CH_R)) out.println("Retract finished.");
}

void setupMic() {
  i2s_config_t cfg = {};
  cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate          = SAMPLE_RATE;
  cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;  // INMP441: 24-bit data in a 32-bit slot
  cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;  // read both slots
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count        = 8;
  cfg.dma_buf_len          = 256;
  cfg.use_apll             = false;

  i2s_pin_config_t pins = {};
  pins.mck_io_num   = I2S_PIN_NO_CHANGE;
  pins.bck_io_num   = MIC_SCK;
  pins.ws_io_num    = MIC_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = MIC_SD;

  if (i2s_driver_install(I2S_PORT, &cfg, 0, NULL) != ESP_OK ||
      i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
    out.println("I2S init failed — check wiring / pin numbers.");
    return;
  }
  // The INMP441 lets SD float during the slot it isn't using; a pull-down keeps it at 0
  gpio_pulldown_en((gpio_num_t)MIC_SD);
  i2s_zero_dma_buffer(I2S_PORT);
}

// Fills samples[]; returns the number of stereo frames read
int readBlock() {
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, samples, sizeof(samples), &bytesRead, portMAX_DELAY);
  return bytesRead / (2 * sizeof(int32_t));
}

// RMS level of one slot of the current block in dBFS (0 = full scale), peak via pointer
float slotLevelDb(int slot, int frames, float *peakDb) {
  *peakDb = -140;
  if (frames == 0) return -140;

  // 24-bit signed sample sits in the top of the 32-bit word
  double mean = 0;
  for (int i = 0; i < frames; i++) mean += samples[2 * i + slot] >> 8;
  mean /= frames;  // remove the mic's small DC offset

  double sumSq = 0, peak = 0;
  for (int i = 0; i < frames; i++) {
    double s = (samples[2 * i + slot] >> 8) - mean;
    sumSq += s * s;
    if (fabs(s) > peak) peak = fabs(s);
  }
  const double FULL_SCALE = 8388608.0;  // 2^23
  double rms = sqrt(sumSq / frames);
  if (peak > 0) *peakDb = 20 * log10(peak / FULL_SCALE);
  return rms > 0 ? 20 * log10(rms / FULL_SCALE) : -140;
}

float readLevelDb(float *peakDb) {
  return slotLevelDb(micSlot, readBlock(), peakDb);
}

// Prints raw data from both slots, to tell a wiring fault from a channel mix-up
void runDiagnostic() {
  readBlock();                 // discard a block that may be stale
  int frames = readBlock();
  out.printf("Diagnostic: %d frames read\n", frames);
  for (int slot = 0; slot < 2; slot++) {
    int32_t mn = INT32_MAX, mx = INT32_MIN;
    for (int i = 0; i < frames; i++) {
      int32_t v = samples[2 * i + slot];
      mn = min(mn, v);
      mx = max(mx, v);
    }
    float pk;
    float db = slotLevelDb(slot, frames, &pk);
    out.printf("  slot %d: raw %08X %08X %08X %08X  min %08X  max %08X  level %.1f dB\n",
               slot, samples[slot], samples[2 + slot], samples[4 + slot], samples[6 + slot],
               mn, mx, db);
  }
  out.printf("  using slot %d\n", micSlot);
}

// Picks whichever slot actually carries audio
void detectMicSlot() {
  readBlock();                 // first block after start-up can be empty
  float pk;
  int frames = readBlock();
  float db0 = slotLevelDb(0, frames, &pk);
  float db1 = slotLevelDb(1, frames, &pk);
  micSlot = db1 > db0 ? 1 : 0;
  out.printf("Mic slot levels: slot0 %.1f dB, slot1 %.1f dB -> using slot %d\n", db0, db1, micSlot);
  if (db0 <= -139 && db1 <= -139) {
    out.println("WARNING: no signal on either slot — the mic isn't sending data. Check wiring (send 'd').");
  }
}

// USB serial only: a text bar meter
void printBar(float db, float peakDb) {
  // Bar spans -90 dBFS (quiet) to 0 dBFS (clipping); '|' marks the threshold
  const int WIDTH = 45;
  int len    = constrain((int)((db + 90) / 2), 0, WIDTH);
  int thrPos = constrain((int)((thresholdDb + 90) / 2), 0, WIDTH);
  char bar[WIDTH + 1];
  for (int i = 0; i < WIDTH; i++) bar[i] = i < len ? '#' : (i == thrPos ? '|' : ' ');
  bar[WIDTH] = '\0';
  Serial.printf("%6.1f dB  pk %6.1f  [%s]%s\n", db, peakDb, bar,
                autoArmed ? (loudBlocks > 0 ? " LOUD" : " armed") : "");
}

// Phone only: level, peak, armed, threshold, loud blocks, mic slot
void sendLevel(float db, float peakDb) {
  char msg[64];
  snprintf(msg, sizeof(msg), "L,%.1f,%.1f,%d,%.1f,%d,%d\n",
           db, peakDb, autoArmed ? 1 : 0, thresholdDb, loudBlocks, micSlot);
  bleSendLine(msg);
}

void printHelp() {
  out.println("Commands: e=extend  r=retract  s=stop  m=level printing on/off");
  out.println("          a=auto-close on/off  +/-=threshold up/down 3 dB");
  out.println("          d=mic diagnostic  c=switch mic slot  h=help");
  out.printf("Threshold %.1f dBFS, auto-close %s\n", thresholdDb, autoArmed ? "ON" : "OFF");
}

void setup() {
  Serial.begin(115200);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH); // enable the driver

  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM, PWM_CH_R);
  ledcAttachPin(LPWM, PWM_CH_L);
  stopActuator();

  setupBle();
  setupMic();
  detectMicSlot();
  out.printf("Ready. Bluetooth name: %s\n", BLE_NAME);
  printHelp();
}

void handleCommand(char c) {
  switch (c) {
    case 'e': extendActuator(); break;
    case 'r': retractActuator(); break;
    case 's': stopActuator(); break;
    case 'm': printLevel = !printLevel; break;
    case 'a':
      autoArmed = !autoArmed;
      loudBlocks = 0;
      out.printf("Auto-close %s (threshold %.1f dBFS)\n", autoArmed ? "ON" : "OFF", thresholdDb);
      break;
    case '+': thresholdDb += 3; out.printf("Threshold %.1f dBFS\n", thresholdDb); break;
    case '-': thresholdDb -= 3; out.printf("Threshold %.1f dBFS\n", thresholdDb); break;
    case 'd': runDiagnostic(); break;
    case 'c': micSlot ^= 1; out.printf("Using mic slot %d\n", micSlot); break;
    case 'h': printHelp(); break;
  }
  // The actuator blocks the loop, so throw away audio buffered while it was moving
  if (c == 'e' || c == 'r') i2s_zero_dma_buffer(I2S_PORT);
}

void loop() {
  int c;
  while ((c = readCommand()) >= 0) handleCommand(c);

  float peakDb;
  float db = readLevelDb(&peakDb);
  if (printLevel) printBar(db, peakDb);

  if (autoArmed) {
    loudBlocks = db > thresholdDb ? loudBlocks + 1 : 0;
    if (loudBlocks >= TRIGGER_BLOCKS) {
      out.println(">>> Sustained loud sound — closing. Auto-close disarmed; send 'a' to re-arm.");
      autoArmed = false;
      loudBlocks = 0;
      extendActuator();
      i2s_zero_dma_buffer(I2S_PORT);
    }
  }
  sendLevel(db, peakDb);
}
