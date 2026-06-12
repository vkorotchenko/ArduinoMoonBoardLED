#include <ArduinoBLE.h>
#include <NeoPixelBus.h>
#include <config.h>
#include <malloc.h>
#include <Adafruit_SleepyDog.h>
#include <FlashStorage.h>

// Hardware watchdog timeout (ms). The SAMD21 caps this at ~16 s. If the loop stops
// feeding the watchdog for this long — e.g. the NINA BLE co-processor wedges and
// BLE.poll() blocks — the board resets, reboots, and re-advertises so the app can
// reconnect, instead of sitting dead until a manual reset.
#define WDT_TIMEOUT_MS 16000

// If a connection drops and nothing reconnects within this window, reboot to force a
// clean BLE bring-up. Catches the case where the NINA stops advertising after a
// disconnect but the loop keeps running — so the watchdog (which only catches hangs)
// would never fire.
#define REBOOT_AFTER_DISCONNECT_MS 60000

// --- Persist the last displayed pattern across reboots ----------------------
// The watchdog reboot (and any power cycle) clears the LEDs, and the MoonBoard app
// won't necessarily re-send the current problem. We save the last problem string to
// flash and re-render it on boot so the board restores its pattern on its own.
typedef struct {
  bool valid;
  bool useadditional;
  char problem[500];
} StoredProblem;
FlashStorage(problemStore, StoredProblem);

// RAM cache of what's currently in flash, so we only erase/write flash when the
// problem actually changes (re-sending the same problem costs no flash wear).
char lastSavedProblem[500] = "";

void persistProblem(const char *problem, bool additional) {
  if (strcmp(problem, lastSavedProblem) == 0) return; // unchanged — skip the flash write
  StoredProblem sp;
  sp.valid = true;
  sp.useadditional = additional;
  strncpy(sp.problem, problem, sizeof(sp.problem) - 1);
  sp.problem[sizeof(sp.problem) - 1] = '\0';
  problemStore.write(sp);
  strcpy(lastSavedProblem, sp.problem);
  Serial.println("[flash] saved current pattern");
}

// --- Memory / timeout instrumentation ---------------------------------------
// The SAMD21 has only 32 KB of SRAM and no heap compaction, so the heavy use of
// the Arduino String class on the receive path can fragment the heap over time.
// These helpers let you watch the trend: if freeRAM falls (or free chunks climb)
// across commands and a timeout follows, fragmentation is the cause.
extern "C" char *sbrk(int incr);

// Bytes between the top of the heap and the current stack pointer (overall headroom).
static int freeRam() {
  char stackTop;
  return &stackTop - reinterpret_cast<char *>(sbrk(0));
}

static void logMemory(const char *tag) {
  struct mallinfo mi = mallinfo();
  Serial.print("[mem] ");
  Serial.print(tag);
  Serial.print(" freeRAM=");
  Serial.print(freeRam());
  Serial.print(" heapUsed=");
  Serial.print(mi.uordblks);
  Serial.print(" heapFree=");
  Serial.print(mi.fordblks);
  Serial.print(" freeChunks=");   // rising = fragmenting
  Serial.println(mi.ordblks);
}

#ifdef GRB
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
#else
NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
#endif

BLEService uartService = BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic receiveCharacteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, 20);
BLECharacteristic transmitCharacteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20);


RgbColor red(brightness, 0, 0);
RgbColor green(0, brightness, 0);
RgbColor blue(0, 0, brightness);
RgbColor yellow(additionalledbrightness, additionalledbrightness, 0);
RgbColor cyan(0, brightness, brightness);
RgbColor pink(brightness, 0, brightness/2);
RgbColor violet(brightness/2, 0, brightness);
RgbColor black(0);

int state = 0; // Variable to store the current state of the problem string parser
char problemstring[500] = "";      // current problem string (fixed buffer — no heap allocation)
char problemstringstore[500] = ""; // copy used for the additional-LED render pass (strtok is destructive)
bool useadditionalled = false; // Variable to store the additional LED setting


void ConnectHandler(BLEDevice central) {
  if (Serial) { Serial.print("Connected event, central: "); Serial.println(central.address()); }
  // Start every new connection with a clean parser, so a problem string left
  // half-received by a previous disconnect can't corrupt this session's first message.
  state = 0;
  problemstring[0] = '\0';
  useadditionalled = false;
  BLE.advertise(); // keep advertising so additional centrals can connect (multi-user)
}

void DisconnectHandler(BLEDevice central) {
  if (Serial) { Serial.print("Disconnected event, central: "); Serial.println(central.address()); }
  BLE.advertise();
}

void display(byte incoming[], int length){
// Check if message parts are available on the BLE UART
  for (int i = 0; i <length; i++) {
    char c = incoming[i];

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
      // Append the character to the fixed buffer (bounded + null-terminated).
      size_t len = strlen(problemstring);
      if (len < sizeof(problemstring) - 1) {
        problemstring[len] = c;
        problemstring[len + 1] = '\0';
      } else {
        Serial.println("[warn] problemstring full, dropping char (missing '#' terminator?)");
      }
    }
  }


  // State 4: complete problem string received, start parsing
  if (state == 4) {
    strip.ClearTo(black); // Turn off all LEDs in LED string
    Serial.println("\n---------");
    Serial.print("Problem string: ");
    Serial.println(problemstring);
    Serial.println("");

    strcpy(problemstringstore, problemstring); // store copy (strtok below is destructive)

    if (useadditionalled) { // only render additional LEDs in first loop
      Serial.println("Additional LEDs:");

      // Holds are separated by commas/spaces, e.g. "S5, R10, E18".
      char *hold = strtok(problemstring, ", ");
      while (hold != NULL) {

        char holdtype = hold[0]; // Hold descriptions consist of a hold type (S, P, E) ...
        int holdnumber = atoi(&hold[1]); // ... and a hold number
        if (holdnumber < 0 || holdnumber >= (int)PixelCount) { // guard against bad input
          Serial.print("[warn] ignoring out-of-range hold ");
          Serial.print(holdtype);
          Serial.println(holdnumber);
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
        } // end bounds-valid

        hold = strtok(NULL, ", "); // get next hold
      }

      strcpy(problemstring, problemstringstore); // Restore problem string for rendering normal hold LEDs
    }

    Serial.println("Problem LEDs:");
    char *hold = strtok(problemstring, ", ");
    while (hold != NULL) {

      char holdtype = hold[0]; // Hold descriptions consist of a hold type (S, P, E) ...
      int holdnumber = atoi(&hold[1]); // ... and a hold number
      if (holdnumber < 0 || holdnumber >= (int)PixelCount) { // guard against bad input
        Serial.print("[warn] ignoring out-of-range hold ");
        Serial.print(holdtype);
        Serial.println(holdnumber);
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
      } // end bounds-valid

      hold = strtok(NULL, ", "); // get next hold

      if (hold == NULL) { // Last hold has been processed!
        strip.Show(); // Light up all hold (and additional) LEDs
        persistProblem(problemstringstore, useadditionalled); // remember it across reboots
        problemstring[0] = '\0'; // Reset problem string
        useadditionalled = false; // Reset additional LED option
        state = 0; // Switch to state 0 (wait for new problem string or configuration)
        Serial.println("---------\n");
        logMemory("after-parse"); // freeRAM should now stay flat across problems
        break;
      }
    }
  }
}

void characteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  int length = characteristic.valueLength();
  byte incoming[length];

  characteristic.readValue(incoming, length);

  // Echo the chunk as readable text plus its length, so dropped/garbled BLE
  // packets are visible in the log.
  Serial.print("[rx] len=");
  Serial.print(length);
  Serial.print(" data=\"");
  for (int i = 0; i < length; i++) {
    Serial.print((char)incoming[i]);
  }
  Serial.println("\"");

  unsigned long t0 = millis();
  display(incoming, length);
  unsigned long elapsed = millis() - t0;

  // A long parse blocks BLE servicing and can trip the supervision timeout.
  if (elapsed > 50) {
    Serial.print("[warn] display() took ");
    Serial.print(elapsed);
    Serial.println(" ms (BLE not serviced during this time)");
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT); // onboard-LED liveness indicator (see loop())

  // Bring BLE up FIRST, before the ~20 s LED animation, so the board becomes
  // discoverable within ~2 s of a (re)boot instead of waiting for the animation.
  if (!BLE.begin()) {
    // BLE bring-up failed. The watchdog isn't enabled yet (it must come after the
    // animation), so reboot explicitly to retry rather than hanging forever.
    delay(50);
    NVIC_SystemReset();
  }
  BLE.setLocalName("Moonboard");
  BLE.setDeviceName("Moonboard");
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(receiveCharacteristic);
  uartService.addCharacteristic(transmitCharacteristic);
  receiveCharacteristic.setEventHandler(BLEWritten, characteristicWritten);
  BLE.addService(uartService);
  BLE.setEventHandler(BLEConnected, ConnectHandler);
  BLE.setEventHandler(BLEDisconnected, DisconnectHandler);
  BLE.advertise();

  strip.Begin(); // Initialize LED strip
  strip.Show(); // Good practice to call Show() in order to clear all LEDs

  // Test LEDs by cycling through the colors and then turning the LEDs off again
  strip.SetPixelColor(0, green);
  for (int i = 0; i < PixelCount; i++) {
    strip.ShiftRight(1);
    strip.Show();
    delay(10);
  }
  strip.SetPixelColor(0, blue);
  for (int i = 0; i < PixelCount; i++) {
    strip.ShiftRight(1);
    strip.Show();
    delay(10);
  }
  strip.SetPixelColor(0, yellow);
  for (int i = 0; i < PixelCount; i++) {
    strip.ShiftRight(1);
    strip.Show();
    delay(10);
  }
  strip.SetPixelColor(0, cyan);
  for (int i = 0; i < PixelCount; i++) {
    strip.ShiftRight(1);
    strip.Show();
    delay(10);
  }
  strip.SetPixelColor(0, pink);
  for (int i = 0; i < PixelCount; i++) {
    strip.ShiftRight(1);
    strip.Show();
    delay(10);
  }
  strip.SetPixelColor(0, violet);
  for (int i = 0; i < PixelCount; i++) {
    strip.ShiftRight(1);
    strip.Show();
    delay(10);
  }
  strip.SetPixelColor(0, red);
  for (int i = 0; i < PixelCount; i++) {
    strip.ShiftRight(1);
    strip.Show();
    delay(10);
  }

  strip.ClearTo(black);
  strip.Show();

  // Enable the watchdog AFTER the (~20 s) boot animation so it can't trip during it.
  // From here on, any code path that stalls longer than WDT_TIMEOUT_MS reboots the board.
  int wdtActual = Watchdog.enable(WDT_TIMEOUT_MS);
  Serial.print("Watchdog enabled, timeout ");
  Serial.print(wdtActual);
  Serial.println(" ms");

  // Restore the last displayed pattern after a reboot (watchdog reset / power cycle),
  // so the board re-lights on its own without waiting for the app to resend.
  StoredProblem sp = problemStore.read();
  if (sp.valid && sp.problem[0] != '\0') {
    strcpy(lastSavedProblem, sp.problem); // seed the cache so we don't re-write it
    strcpy(problemstring, sp.problem);
    useadditionalled = sp.useadditional;
    state = 4; // jump straight to the render stage
    Serial.print("Restoring last pattern: ");
    Serial.println(problemstring);
    display(NULL, 0); // no new bytes — just render problemstring and reset state
  }
}

void loop() {
  Watchdog.reset(); // fed every iteration; only a blocked BLE.poll() (a true hang) trips it
  BLE.poll();       // service BLE and deliver incoming writes (characteristicWritten)

  // Liveness indicator, independent of serial: blink the onboard LED ~1 Hz. If this keeps
  // blinking on external power with no computer attached, the loop is running (not asleep).
  // If it freezes, the loop has truly hung — and the watchdog should reboot within ~16 s.
  static unsigned long lastBlink = 0;
  static bool ledOn = false;
  if (millis() - lastBlink > 500) {
    lastBlink = millis();
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
  }

  static bool wasConnected = false;
  static unsigned long lastBeat = 0;
  static unsigned long disconnectedAt = 0;

  BLEDevice central = BLE.central();
  bool connected = central && central.connected();

  if (connected) {
    if (!wasConnected) {
      wasConnected = true;
      disconnectedAt = 0; // clear the recovery timer
      if (Serial) { Serial.print("Connected to central: "); Serial.println(central.address()); }
    }

    // Heartbeat — only when a monitor is attached, so field operation never blocks on
    // the USB-CDC serial endpoint. A healthy idle link stays connected indefinitely here.
    if (Serial && millis() - lastBeat > 5000) {
      lastBeat = millis();
      Serial.print("[beat] up=");
      Serial.print(millis() / 1000);
      Serial.print("s ");
      logMemory("idle");
    }
  } else {
    if (wasConnected) {
      // Just disconnected: re-advertise from the main-loop context (more reliable on the
      // NINA than doing it only inside the disconnect event handler) and start the timer.
      wasConnected = false;
      disconnectedAt = millis();
      if (Serial) Serial.println("Disconnected from central — re-advertising");
      BLE.advertise();
    }

    // If a dropped link doesn't recover, reboot for a clean BLE bring-up. The watchdog
    // can't catch this — the loop is running fine, the NINA just stopped advertising.
    if (disconnectedAt != 0 && millis() - disconnectedAt > REBOOT_AFTER_DISCONNECT_MS) {
      if (Serial) Serial.println("BLE did not recover after disconnect — rebooting");
      delay(20);
      NVIC_SystemReset();
    }
  }
}