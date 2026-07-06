#include <NeoPixelBus.h>
#include <config.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <esp_system.h>
#include <esp_task_wdt.h>

#define LOOP_WDT_TIMEOUT_S 15

RTC_NOINIT_ATTR static uint32_t crashMagic;
RTC_NOINIT_ATTR static int      lastState;
RTC_NOINIT_ATTR static char     lastProblem[256];
#define CRASH_MAGIC 0xC0FFEE42

static const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "PANIC / exception";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog (hang)";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (power sag)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "unknown";
  }
}

static void breadcrumb(int stateNow, const char *problem) {
  lastState = stateNow;
  if (problem != nullptr) {
    strncpy(lastProblem, problem, sizeof(lastProblem) - 1);
    lastProblem[sizeof(lastProblem) - 1] = '\0';
  }
  crashMagic = CRASH_MAGIC;
}

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#ifdef GRB
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2811Method> strip(PixelCount, PixelPin);
#else
NeoPixelBus<NeoRgbFeature, NeoEsp32Rmt0Ws2811Method> strip(PixelCount, PixelPin);
#endif

RgbColor red(brightness, 0, 0);
RgbColor green(0, brightness, 0);
RgbColor blue(0, 0, brightness);
RgbColor yellow(additionalledbrightness, additionalledbrightness, 0);
RgbColor cyan(0, brightness, brightness);
RgbColor pink(brightness, 0, brightness/2);
RgbColor violet(brightness/2, 0, brightness);
RgbColor black(0);

// Onboard status NeoPixel (Feather ESP32 V2): green = powered on, blue = BLE connected.
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 0
#endif
#ifndef NEOPIXEL_I2C_POWER
#define NEOPIXEL_I2C_POWER 2
#endif
const uint8_t statusbrightness = 24;
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt1Ws2812xMethod> statusPixel(1, (uint8_t)PIN_NEOPIXEL);
RgbColor statusGreen(0, statusbrightness, 0);
RgbColor statusBlue(0, 0, statusbrightness);

static void setStatusColor(RgbColor color) {
  statusPixel.SetPixelColor(0, color);
  statusPixel.Show();
}

// Boot self-test: cycle each color as a single pixel travelling up the strip,
// then clear. Feeds the task watchdog so the long animation can't trip it.
static void startupAnimation() {
  RgbColor colors[] = {green, blue, yellow, cyan, pink, violet, red};
  for (RgbColor color : colors) {
    strip.SetPixelColor(0, color);
    for (int i = 0; i < PixelCount; i++) {
      strip.ShiftRight(1);
      strip.Show();
      esp_task_wdt_reset();
      delay(10);
    }
  }
  strip.ClearTo(black);
  strip.Show();
}

int state = 0; // Variable to store the current state of the problem string parser
char problemstring[500]=""; // Variable to store the current problem string
char problemstringstore[500]=""; // Variable to store the current problem string
bool useadditionalled = false; // Variable to store the additional LED setting

static const size_t RX_BUF_SIZE = 1024;
static volatile uint8_t rxBuf[RX_BUF_SIZE];
static volatile size_t rxHead = 0;
static volatile size_t rxTail = 0;
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

static void rxPush(const uint8_t *data, size_t len) {
  portENTER_CRITICAL(&rxMux);
  for (size_t i = 0; i < len; i++) {
    size_t next = (rxHead + 1) % RX_BUF_SIZE;
    if (next == rxTail) break;
    rxBuf[rxHead] = data[i];
    rxHead = next;
  }
  portEXIT_CRITICAL(&rxMux);
}

static size_t rxAvailable() {
  portENTER_CRITICAL(&rxMux);
  size_t n = (rxHead + RX_BUF_SIZE - rxTail) % RX_BUF_SIZE;
  portEXIT_CRITICAL(&rxMux);
  return n;
}

static int rxRead() {
  int c = -1;
  portENTER_CRITICAL(&rxMux);
  if (rxTail != rxHead) {
    c = rxBuf[rxTail];
    rxTail = (rxTail + 1) % RX_BUF_SIZE;
  }
  portEXIT_CRITICAL(&rxMux);
  return c;
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    Serial.println("BLE central connected");
    setStatusColor(statusBlue);
  }
  void onDisconnect(BLEServer *server) override {
    Serial.println("BLE central disconnected — restarting advertising");
    setStatusColor(statusGreen);
    server->getAdvertising()->start();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    uint8_t *data = characteristic->getData();
    size_t len = characteristic->getLength();
    if (data != nullptr && len > 0) {
      rxPush(data, len);
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(200);

  esp_reset_reason_t reason = esp_reset_reason();
  Serial.println("\n==================== BOOT ====================");
  Serial.printf("Reset reason: %s\n", resetReasonName(reason));

  bool crashed = (reason == ESP_RST_PANIC || reason == ESP_RST_TASK_WDT ||
                  reason == ESP_RST_INT_WDT || reason == ESP_RST_WDT ||
                  reason == ESP_RST_BROWNOUT);
  if (crashed && crashMagic == CRASH_MAGIC) {
    Serial.println("!! Recovered from a crash. Last activity before it died:");
    Serial.printf("   parser state = %d\n", lastState);
    Serial.printf("   problem string = \"%s\"\n", lastProblem);
  } else if (reason == ESP_RST_POWERON) {
    Serial.println("(clean power-on — no prior crash context)");
  }
  crashMagic = 0;
  Serial.println("=============================================");

  esp_task_wdt_init(LOOP_WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  strip.Begin(); // Initialize LED strip
  strip.Show();  // Good practice to call Show() in order to clear all LEDs
  strip.ClearTo(black);
  strip.Show();

  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH); // power the onboard NeoPixel rail
  delay(10);
  statusPixel.Begin();
  setStatusColor(statusGreen); // green = powered on, not yet connected

  BLEDevice::init("Moonboard");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(NUS_SERVICE_UUID);

  BLECharacteristic *txCharacteristic =
      service->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *rxCharacteristic = service->createCharacteristic(
      NUS_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxCharacteristic->setCallbacks(new RxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(NUS_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising as \"Moonboard\"");

  startupAnimation(); // LED self-test; BLE is already advertising so the board stays discoverable
}

void loop() {
  esp_task_wdt_reset();

  // Check if message parts are available from the BLE UART
  while (rxAvailable() > 0) {
    char c = (char)rxRead();

    // State 0: wait for configuration instructions (sent only if V2 option is enabled in app) or beginning of problem string
    if (state == 0) {
      if (c == '~') {
        state = 1; // Switch to state 1 (read configuration)
        continue;
      }
      if (c == 'l') {
        state = 2; // Switch to state 2 (read problem string)
        continue;
      }
    }

    // State 1: read configuration option (sent only if V2 option is enabled in app)
    if (state == 1) {
      if (c == 'D') {
        useadditionalled = true;
        state = 2; // Switch to state 2 (read problem string)
        continue;
      }
      if (c == 'l') {
        state = 2; // Switch to state 2 (read problem string)
        continue;
      }
    }

    // State 2: wait for the second part of the beginning of a new problem string (# after the lower-case L)
    if (state == 2) {
      if (c == '#') {
        state = 3; // Switch to state 3
        continue;
      }
    }

    // State 3: store hold descriptions in problem string
    if (state == 3) {
      if (c == '#') { // problem string ends with #
        state = 4; // Switch to state 4 (start parsing and show LEDs)
        continue;
      }

      size_t len = strlen(problemstring);
      if (len < sizeof(problemstring) - 1) {
        problemstring[len] = c;
        problemstring[len + 1] = '\0';
      }
    }
  }

  // State 4: complete problem string received, start parsing
  if (state == 4) {
    breadcrumb(state, problemstring);
    strip.ClearTo(black); // Turn off all LEDs in LED string
    Serial.println("\n---------");
    Serial.print("Problem string: ");
    Serial.println(problemstring);
    Serial.println("");

    strcpy(problemstringstore, problemstring); // store copy of problem string

    if (useadditionalled) { // only render additional LEDs in first loop
      Serial.println("Additional LEDs:");

      char *hold = strtok(problemstring, ", ");
      // Keep printing tokens while one of the
      // delimiters present in str[].
      while (hold != NULL) {

        char holdtype = hold[0]; // Hold descriptions consist of a hold type (S, P, E) ...
        int holdnumber = atoi(&hold[1]); // ... and a hold number
        if (holdnumber < 0 || holdnumber >= (int)PixelCount) {
          Serial.printf("  ! ignoring out-of-range hold %c%d\n", holdtype, holdnumber);
        } else {
        int lednumber = ledmapping[holdnumber];
        int additionallednumber = lednumber + additionalledmapping[holdnumber];
        if (additionalledmapping[holdnumber] != 0) {
          Serial.print(holdtype);
          Serial.print(holdnumber);
          Serial.print(" --> ");
          Serial.print(additionallednumber);
          if (holdtype == 'S') { // Start hold
            strip.SetPixelColor(additionallednumber, yellow);
            Serial.println(" (yellow)");
          }
          // Right, left, match, or foot hold
          if (holdtype == 'R' || holdtype == 'L' || holdtype == 'M' || holdtype =='F' || holdtype =='P') {
            strip.SetPixelColor(additionallednumber, yellow);
            Serial.println(" (yellow)");
          }
          // Finish holds don't get an additional LED!
        }
        }

        hold = strtok(NULL, ", ");// get next hold
      }

      strcpy(problemstring, problemstringstore); // Restore problem string for rendering normal hold LEDs
    }

    Serial.println("Problem LEDs:");
    char *hold = strtok(problemstring, ", ");
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (hold != NULL) {

      char holdtype = hold[0]; // Hold descriptions consist of a hold type (S, P, E) ...
      int holdnumber = atoi(&hold[1]); // ... and a hold number
      if (holdnumber < 0 || holdnumber >= (int)PixelCount) {
        Serial.printf("  ! ignoring out-of-range hold %c%d\n", holdtype, holdnumber);
      } else {
      int lednumber = ledmapping[holdnumber];
      Serial.print(holdtype);
      Serial.print(holdnumber);
      Serial.print(" --> ");
      Serial.print(lednumber);
      if (holdtype == 'S') { // Start hold
        strip.SetPixelColor(lednumber, green);
        Serial.println(" (green)");
      }
      if (holdtype == 'R' || holdtype =='P') { // Right hold
        strip.SetPixelColor(lednumber, blue);
        Serial.println(" (blue)");
      }
      if (holdtype == 'L') { // Left hold
        strip.SetPixelColor(lednumber, violet);
        Serial.println(" (violet)");
      }
      if (holdtype == 'M') { // Match hold
        strip.SetPixelColor(lednumber, pink);
        Serial.println(" (pink)");
      }
      if (holdtype == 'F') { // Foot hold
        strip.SetPixelColor(lednumber, cyan);
        Serial.println(" (cyan)");
      }
      if (holdtype == 'E') { // End hold
        strip.SetPixelColor(lednumber, red);
        Serial.println(" (red)");
      }
      }

      hold = strtok(NULL, ", "); // get next hold

      if (hold == NULL) { // Last hold has been processed!
        strip.Show(); // Light up all hold (and additional) LEDs
        strcpy(problemstring, ""); // Reset problem string
        useadditionalled = false; // Reset additional LED option
        state = 0; // Switch to state 0 (wait for new problem string or configuration)
        Serial.println("---------\n");
        break;
      }
    }
  }
}
