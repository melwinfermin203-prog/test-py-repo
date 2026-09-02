#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <EEPROM.h>
#include "DFRobot_PH.h"

/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║   RAIN FILTER — ESP32-C3 Super Mini Web Dashboard Controller  ║
 * ║   v4.6 — pH calibration offset changed to -1.6                 ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * WHAT CHANGED IN v4.6
 *   - PH_CALIBRATION_OFFSET changed from -2.0 to -1.6.
 *
 * WHAT CHANGED IN v4.5
 *   - Added PH_CALIBRATION_OFFSET (-2.0), applied to every raw pH
 *     reading right at the source in readPHSensor(). Since g_phValue
 *     is the single value everything downstream uses -- the sample
 *     buffer/median, dry-detection, state-machine decisions, the
 *     dashboard, and the IoT send -- offsetting it here means the
 *     shift is applied exactly once and stays consistent everywhere.
 *     Adjust the constant if -2.0 isn't the right correction for
 *     your probe.
 *
 * WHAT CHANGED IN v4.4
 *   - REJECT now also stops if the Collection tank empties mid-
 *     reject (checks g_collectionEmpty, same as TRANSFER already
 *     did) -- previously it would keep running until pH looked
 *     "recovered" or the dry-check tripped, risking a dry-run.
 *   - pH decisions now use the MEDIAN of several readings instead of
 *     a single instantaneous one. PH_STABILIZE_MS (10s) and the
 *     existing 1s sensor sampling already line up to give ~10
 *     readings per stabilize window -- readPHSensor() now records
 *     each one into a buffer while in PSTATE_READING, and the moment
 *     the stabilize timer elapses, medianOfPhSamples() computes the
 *     median (not mean, so one noisy spike can't skew it) and that
 *     becomes the decision-making g_phValue for evaluateAndTransition().
 *     Applies everywhere READING is entered: initial arm, REJECT/
 *     HOLD recovery, and every post-DOSE_WAIT recheck. The live
 *     g_phValue shown on the dashboard between decisions is still
 *     the latest raw instantaneous reading -- only the value used to
 *     actually decide reject/dose/hold/transfer is the median.
 *     Sample count is exposed as phSampleCount on /api/status and
 *     shown on the dashboard while sampling.
 *
 * WHAT CHANGED IN v4.3
 *   - Dashboard's own JS polling slowed from every 2s to every 30s,
 *     so the pH value (and everything else) on the web page only
 *     updates as often as the IoT send does. The sensor itself
 *     still reads every PH_READ_INTERVAL_MS (1s internally) so dry-
 *     detection and the auto state machine stay responsive -- only
 *     the browser's poll rate changed, not the underlying sampling.
 *   - BUGFIX: DIAPHRAGM_WATCHDOG_MS was 8s, well under the new 30s
 *     poll interval -- with the slower poll, the diaphragm pump's
 *     dashboard-disconnect watchdog would have force-stopped it
 *     within 8 seconds of every normal page load, mistaking normal
 *     30s polling for a dropped connection. Raised to 75s (comfortably
 *     above 30s) so real polling never false-trips it, while a truly
 *     closed tab or dead connection still gets caught.
 *
 * WHAT CHANGED IN v4.2
 *   - New pH bands added on the alkaline side (previously anything
 *     >= PH_DOSE_BELOW was accepted with no upper limit):
 *       pH > 9.0 (PH_REJECT_ABOVE)              -> REJECT (too alkaline)
 *       8.5 < pH <= 9.0 (PH_HOLD_ABOVE..REJECT_ABOVE) -> HOLD (new
 *         PSTATE_HOLD: pauses with no pumps running, keeps
 *         monitoring pH, moves to TRANSFER/DOSING/REJECT the moment
 *         it drifts into a decisive band again)
 *       6.5 <= pH <= 8.5                         -> TRANSFER (unchanged)
 *     BUGFIX while wiring this in: the old REJECT exit condition
 *     was "pH >= PH_REJECT_BELOW", which would have immediately
 *     kicked a too-ALKALINE rejection (pH > 9) back to READING even
 *     though it was still far too alkaline, since 9 >= 5.5 is true.
 *     REJECT now only exits once pH is back inside
 *     [PH_REJECT_BELOW, PH_REJECT_ABOVE].
 *   - Added periodic pH reporting to an external IoT endpoint via
 *     HTTPClient POST, every PH_IOT_SEND_INTERVAL_MS (30s). Fill in
 *     IOT_ENDPOINT_URL with your real server/platform URL -- it's a
 *     placeholder right now. Sends {"ph","stage","isDry","uptimeMs"}
 *     as JSON. Best-effort: a failed/unreachable send is logged to
 *     Serial and never blocks or affects the state machine.
 *
 * WHAT CHANGED IN v4.1
 *   - Reversed COLLECTION_EMPTY_ACTIVE_LOW to false (was true). The
 *     Collection Tank EMPTY float now reads "empty" on HIGH instead
 *     of LOW -- matches a float switch that closes (grounds the pin)
 *     when the tank is full/has water and opens when it's empty. No
 *     other logic changed; g_collectionEmpty is still derived the
 *     same way in readFloats() and consumed the same way everywhere
 *     else (TRANSFER stage exit, manual transfer-toggle interlock).
 *
 * WHAT CHANGED IN v4.0
 *   Hardware reality check: this board only has 4 relays wired up,
 *   not 5, and there's no Storage-full float and no separate
 *   dispense outlet. Reworked around the real hardware:
 *   - REMOVED: Mixing pump entirely (no 5th relay for it). All auto
 *     and manual mixing logic, the MIXING button, DOSE_CYCLE_MS's
 *     mixing phase description, and SSR_MIXING_PIN are gone.
 *   - REMOVED: FLOAT_STORAGE_FULL_PIN / Storage-full float. There is
 *     no sensor on Storage tank, so the auto TRANSFER stage and the
 *     manual transfer toggle no longer pause or block on "storage
 *     full" -- that safety check is gone along with the sensor.
 *   - BUGFIX: SSR_PERISTALTIC_PIN is confirmed on GPIO20 (not GPIO4
 *     -- the v3.7 "fix" had actually swapped it onto the pin that
 *     used to be mixing's; GPIO20 is correctly the peristaltic/
 *     dosing relay).
 *   - RENAMED/REPURPOSED: the old "Submersible" pump (GPIO7,
 *     "Storage -> Dispense, manual") is renamed SUBMERSIBLE PUMP
 *     and now does what "Accept" used to do: Collection -> Storage.
 *     It runs automatically during the TRANSFER stage and can also
 *     be toggled manually while in MANUAL mode (same interlocks as
 *     the old Accept: blocked if line is dry or Collection is empty
 *     -- the storage-full block is gone since there's no sensor).
 *   - RENAMED/REPURPOSED: the old "Accept" pump (GPIO21, Collection
 *     -> Storage) is renamed DIAPHRAGM PUMP and now moves water
 *     Dosing tank -> Collection tank. It is a manual-only toggle
 *     that works in either AUTO or MANUAL mode (same pattern the old
 *     "Dispense" toggle used), with the same runtime cap +
 *     dashboard-watchdog auto-stop as a safety backstop since
 *     nothing senses when the Collection tank is full from this
 *     pump's perspective.
 *   - The 4 relays now in use: Peristaltic (GPIO20), Diaphragm
 *     (GPIO21), Reject (GPIO5), Submersible (GPIO7).
 *
 * WHAT CHANGED IN v3.7
 *   - Bugfix: SSR_PERISTALTIC_PIN and SSR_MIXING_PIN were swapped
 *     relative to the actual wiring -- the mixing pump's relay is
 *     physically on GPIO20, and the dosing/peristaltic pump's relay
 *     is on GPIO4. The code previously had this backwards (mixing
 *     on GPIO4, peristaltic on GPIO20), so the MIXING button on the
 *     dashboard was toggling an unconnected pin while the real
 *     mixing pump only responded to the DOSING button. Both
 *     #defines are now swapped to match the real wiring; no other
 *     code changed since everything references the pins by name.
 *
 * WHAT CHANGED IN v3.6
 *   - Mixing pump is back on SSR_MIXING_PIN (GPIO4), and this time
 *     it runs BOTH ways:
 *       AUTO: same as the old v3.4 behavior -- starts together with
 *       the peristaltic pump at the top of each dosing cycle, keeps
 *       running after the peristaltic stops, for DOSE_CYCLE_MS (60s
 *       total) before the next pH re-check.
 *       MANUAL: a 5th toggle button on the dashboard (PUMP_MIXING),
 *       same pattern as Accept/Reject/Dosing -- only usable while in
 *       MANUAL mode. Switching to Manual mode calls allAutoPumpsOff()
 *       (now includes mixing), so an in-progress auto mixing cycle
 *       is safely stopped before manual control takes over.
 *   - Added a 5-minute runtime-cap backstop on mixing, same pattern
 *     as the other pumps, in case a manual toggle gets left on.
 *
 * WHAT CHANGED IN v3.5
 *   - (Superseded by v3.6 above) Removed the mixing pump entirely.
 *
 * WHAT CHANGED IN v3.4
 *   - PH_STABILIZE_MS 20s -> 10s.
 *   - Added a 3rd float: FLOAT_COLLECTION_EMPTY_PIN (GPIO6). LOW
 *     float at the bottom of the Collection tank. TRANSFER now runs
 *     Accept until this float triggers, then stops Accept and loops
 *     back to IDLE automatically, ready to re-arm on the next
 *     Collection FULL rising edge -- instead of relying only on the
 *     Storage-full pause to know when to stop. Storage-full is kept
 *     as a backup safety pause on top of this (Accept still
 *     pauses/resumes if Storage fills mid-transfer). Manual Accept
 *     toggle also blocks if Collection reads empty, same as the
 *     existing dry/storage-full interlocks.
 *   - Confirmed: the pH probe and the Collection FULL float sit at
 *     the same physical level in the tank -- no new sensor there,
 *     that was a placement clarification only.
 *
 * WHAT CHANGED FROM v3.1 (accumulated through v3.3)
 *   - SSR_ACCEPT_PIN moved GPIO4 -> GPIO21.
 *   - SSR_PERISTALTIC_PIN (dosing) moved GPIO6 -> GPIO20.
 *   - LED_ALERT_PIN (was GPIO20) removed/disabled -- no free pin
 *     remains for it now that GPIO20 drives the peristaltic relay.
 *     LED_STATUS_PIN (GPIO10) is unaffected.
 *   - Relay logic flipped to ACTIVE-LOW to match this relay board:
 *     driving a relay pin LOW now turns the pump ON, HIGH turns it
 *     OFF (previously HIGH=on, LOW=off). All 4 relays -- Accept,
 *     Reject, Peristaltic, Submersible -- use the RELAY_ON/
 *     RELAY_OFF macros, including the boot-time "all off" init.
 *   - Retimed the dosing loop (v3.3): PH_STABILIZE_MS 3s -> 20s
 *     (now 10s as of v3.4), DOSING_PULSE_MS 30s -> 15s.
 *
 * BOARD: ESP32-C3 "Super Mini" (bare module, no name-brand carrier
 * board). In Arduino IDE, select board "ESP32C3 Dev Module". This
 * chip has very few usable GPIOs, so the physical button/LED/buzzer
 * panel from earlier versions has been dropped entirely — the web
 * dashboard is now the ONLY control surface. GPIO18/19 are reserved
 * for native USB on this board and must never be used as GPIO;
 * GPIO2/8/9 are strapping pins (boot-mode select) and are avoided
 * here too.
 *
 * WHAT CHANGED FROM v2
 *   Reviewed the auto state machine end-to-end. One question was
 *   flagged and resolved: armIfCollectionFull() -> READING fires
 *   before any pump has switched on, so if the pH probe sat in the
 *   transfer line (only wet once a pump moves water past it), the
 *   very first reading would see a dry/garbage line and abort the
 *   cycle before it ever got a real reading -- and since re-arming
 *   only happens on a fresh rising edge of the Collection float,
 *   that could strand the system needing a manual toggle every
 *   batch. CONFIRMED: the probe is submerged in the Collection tank
 *   itself, which is wet any time the float reads FULL -- so the
 *   reading at the start of READING is always valid and this isn't
 *   a live bug. No logic changes were needed; this note (and the
 *   PH_PIN comment below) just records that assumption so it stays
 *   correct if the probe is ever relocated. Everything else in the
 *   state machine -- thresholds, dosing pulse/wait timing, runtime
 *   caps, dispense watchdog, manual/auto interlocks -- checked out
 *   against the spec below with no changes needed.
 *
 * WHAT CHANGED FROM v1 (besides the board)
 *   System now boots straight into AUTOMATIC mode (this was already
 *   the default, just confirming it explicitly) and processes each
 *   batch as a state machine instead of one continuous decision:
 *
 *     1) Collection tank FULL float trips -> ARM, go to READING.
 *     2) READING: wait PH_STABILIZE_MS (10s) so the sensor settles
 *        before anything acts on it, then look at the reading:
 *          pH < 5.5            -> REJECT (dump to waste) until pH
 *                                  recovers, then re-stabilize
 *          5.5 <= pH < 6.5      -> too acidic to transfer as-is:
 *                                  run the DOSING pulse below
 *          6.5 <= pH <= 8.5     -> TRANSFER to Storage
 *          8.5 < pH <= 9.0      -> HOLD: pause, keep monitoring pH
 *                                  (no pumps run) until it drifts
 *                                  back into a decisive band
 *          pH > 9.0             -> REJECT (dump to waste, too
 *                                  alkaline) until pH recovers
 *     3) DOSING: peristaltic + mixing pumps both start together.
 *        Peristaltic runs for DOSING_PULSE_MS (15s) then stops;
 *        mixing keeps running until the cycle reaches DOSE_CYCLE_MS
 *        (60s total, measured from when dosing started), then it
 *        stops too and it goes back to READING for a fresh 10s-
 *        stabilized pH check. This dose -> mix -> recheck loop
 *        repeats (each loop counts as one "dosing cycle") until the
 *        water reads clean and moves to TRANSFER, or reads bad
 *        enough to REJECT. Mixing can also be run by hand from the
 *        dashboard's Manual Controls, independent of this loop.
 *     4) TRANSFER: Accept pumps Collection -> Storage until the
 *        Collection Tank EMPTY float trips, then stops and loops
 *        back to IDLE automatically, ready to re-arm on the next
 *        Collection FULL trigger. Storage-full still pauses Accept
 *        as a backup safety if Storage fills before Collection
 *        empties (Accept resumes automatically once Storage drains).
 *     5) MAX_DOSING_CYCLES caps how many times it will loop before
 *        giving up and raising an alert (reservoir empty, sensor
 *        fault, etc.) — acknowledge it from the dashboard.
 *     6) Whenever the line reads dry (pH-based proxy, same caveat as
 *        before — tune PH_DRY_THRESHOLD/DRY_CONFIRM_MS against your
 *        probe's real dry-air reading), the whole cycle aborts back
 *        to IDLE regardless of which state it was in.
 *
 *   The Submersible (dispense) pump is unchanged in spirit — manual
 *   only — but since there's no physical hold-button anymore, the
 *   web dashboard uses a toggle instead. A safety watchdog auto-
 *   stops it if the dashboard stops polling for
 *   DISPENSE_WATCHDOG_MS (e.g. the browser tab was closed) so it
 *   can't be left running by a dropped connection, on top of the
 *   existing MAX_DISPENSE_RUNTIME_MS hard cap.
 *
 *   Manual mode (Accept/Reject/Dosing one-shot toggles) is still
 *   available from the dashboard, with the same interlocks as
 *   before (blocked if line reads dry; Accept blocked if Storage
 *   full). Switching to Manual stops the auto state machine, and
 *   switching back to Auto re-arms immediately if the Collection
 *   float is still sitting FULL (no need for a fresh edge).
 *
 * PIN ASSIGNMENTS (ESP32-C3 Super Mini) -- 4 relays only
 *   GPIO0  -> pH sensor analog input (ADC1_CH0) -- probe is
 *             submerged IN THE COLLECTION TANK (confirmed), so it's
 *             wet any time FLOAT_COLLECTION_FULL_PIN reads FULL.
 *             If this probe is ever moved downstream into the
 *             transfer line instead, the auto state machine needs
 *             rework: READING currently starts before any pump is
 *             on, so a line-mounted probe would read dry on the
 *             very first check of every batch.
 *   GPIO1  -> Float: Collection Tank FULL   (active-LOW, internal pull-up)
 *   GPIO5  -> SSR: Reject Pump              (Collection -> Waste)
 *   GPIO6  -> Float: Collection Tank EMPTY  (active-LOW, internal pull-up; stops Submersible transfer when drained)
 *   GPIO7  -> SSR: Submersible Pump         (Collection -> Storage; auto TRANSFER + manual toggle)
 *   GPIO10 -> LED: STATUS (optional, heartbeat)
 *   GPIO20 -> SSR: Peristaltic Pump         (neutralizer dosing)
 *   GPIO21 -> SSR: Diaphragm Pump           (Dosing tank -> Collection tank; manual-only, works in either mode)
 *   AVOID: GPIO18/19 (native USB), GPIO2/8/9 (strapping pins)
 *   NOTE: all 4 relays are ACTIVE-LOW on this board -- see
 *   RELAY_ON/RELAY_OFF below. No Storage-full float on this build --
 *   Storage tank level is not monitored. Alert LED removed (its old
 *   pin, GPIO20, is now the peristaltic relay).
 *
 * LIBRARIES REQUIRED
 *   - DFRobot_PH (by DFRobot)
 *   - WiFi, WebServer -> bundled with the ESP32 board package
 *
 * pH CALIBRATION: same as before — Serial Monitor @115200, type
 * ENTERPH, dip in buffer solution, send the buffer value, EXITPH.
 * Stored in EEPROM.
 */

// ── WiFi credentials ─────────────────────────────────────────
const char* WIFI_SSID     = "BAWAL CONNECT 4G";
const char* WIFI_PASSWORD = "kenhiroseJR2025";
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;

const char* AP_SSID     = "RainFilter-Setup";
const char* AP_PASSWORD = "rainfilter123";

// ── Float switch polarity ─────────────────────────────────────
#define COLLECTION_FULL_ACTIVE_LOW   true
#define COLLECTION_EMPTY_ACTIVE_LOW  false

// ── Pin map (ESP32-C3 Super Mini) ──────────────────────────────
// v4.0: rebuilt around the real 4-relay hardware -- no Storage-full
// float, no Mixing relay. Peristaltic confirmed on GPIO20. GPIO21
// (old "Accept") now drives the Diaphragm pump (Dosing->Collection).
// GPIO7 (old "Submersible/Dispense") now drives the Collection-
// >Storage transfer pump, taking over the role "Accept" used to have.
#define PH_PIN                0

#define FLOAT_COLLECTION_FULL_PIN   1
#define FLOAT_COLLECTION_EMPTY_PIN  6   // low float at bottom of Collection tank -- stops Submersible transfer when drained

#define SSR_REJECT_PIN        5   // Diaphragm: Collection -> Waste
#define SSR_SUBMERSIBLE_PIN   7   // Collection -> Storage (auto TRANSFER + manual toggle)
#define SSR_PERISTALTIC_PIN   20  // Neutralizer dosing
#define SSR_DIAPHRAGM_PIN     21  // Dosing tank -> Collection tank (manual-only, works in either mode)

#define LED_STATUS_PIN        10

// ── Relay logic polarity ────────────────────────────────────
// This relay board is ACTIVE-LOW: driving the pin LOW closes the
// relay (pump ON), driving it HIGH opens it (pump OFF).
#define RELAY_ACTIVE_LOW      true
#define RELAY_ON   (RELAY_ACTIVE_LOW ? LOW  : HIGH)
#define RELAY_OFF  (RELAY_ACTIVE_LOW ? HIGH : LOW)

// ── pH thresholds ────────────────────────────────────────────
const float PH_REJECT_BELOW = 5.5;   // below this -> reject to waste
const float PH_DOSE_BELOW   = 6.5;   // below this (but >= reject) -> dosing loop
const float PH_HOLD_ABOVE   = 8.5;   // above this (but <= reject-above) -> HOLD, pause and monitor
const float PH_REJECT_ABOVE = 9.0;   // above this -> reject to waste (too alkaline)
const float PH_ASSUMED_TEMP_C = 25.0;
const float PH_CALIBRATION_OFFSET = -1.2; // applied to every raw sensor reading below

const float PH_DRY_THRESHOLD   = 1.0;
const unsigned long DRY_CONFIRM_MS = 5000;

// ── State-machine timing ────────────────────────────────────
const unsigned long PH_STABILIZE_MS   = 10000;  // wait after arming/re-check before deciding
const unsigned long DOSING_PULSE_MS   = 15000;  // peristaltic on-time per dose
const unsigned long DOSE_CYCLE_MS     = 60000;  // total time per dose cycle, starting when dosing starts
                                                 // (peristaltic runs the first 15s of this window, then it
                                                 // sits off for the rest -- a passive settle period to let the
                                                 // neutralizer diffuse -- before the next pH re-check)
const int           MAX_DOSING_CYCLES = 10;     // give up & alert after this many loops

// ── Runtime safety caps (backstop) ────────────────────────────
const unsigned long MAX_TRANSFER_RUNTIME_MS   = 10UL * 60UL * 1000UL; // Submersible/Reject continuous
const unsigned long MAX_DOSING_RUNTIME_MS     = 5UL  * 60UL * 1000UL; // Peristaltic continuous (backstop; pulses stay well under this)
const unsigned long MAX_DIAPHRAGM_RUNTIME_MS  = 3UL  * 60UL * 1000UL; // Diaphragm (Dosing tank -> Collection tank)
const unsigned long DIAPHRAGM_WATCHDOG_MS     = 75000UL; // auto-stop diaphragm if dashboard stops polling (must stay well above the dashboard's 30s poll interval so normal polling never false-trips it)

// ── Timing ───────────────────────────────────────────────────
const unsigned long PH_READ_INTERVAL_MS    = 1000;
const unsigned long STATUS_LOG_INTERVAL_MS = 5000;

// ── IoT pH reporting ─────────────────────────────────────────
// Fill in your actual endpoint below. Sends a small JSON POST every
// PH_IOT_SEND_INTERVAL_MS with the current pH reading and stage.
// If you're using a specific platform (ThingSpeak, Blynk, Firebase,
// a custom server, etc.) the URL/payload format may need to change
// to match that platform's API -- this is a generic JSON POST.
const char* IOT_ENDPOINT_URL = "http://YOUR-IOT-SERVER/api/ph";
const unsigned long PH_IOT_SEND_INTERVAL_MS = 30000UL; // every 30 sec

// ── System state ─────────────────────────────────────────────
enum SystemMode { MODE_AUTO, MODE_MANUAL };
SystemMode g_mode = MODE_AUTO; // boots straight into AUTO

enum ManualPump { PUMP_REJECT = 0, PUMP_DOSING = 1, PUMP_TRANSFER = 2, PUMP_COUNT = 3 };

enum ProcessState { PSTATE_IDLE, PSTATE_READING, PSTATE_TRANSFER, PSTATE_REJECT, PSTATE_DOSING, PSTATE_DOSE_WAIT, PSTATE_HOLD };
ProcessState g_state = PSTATE_IDLE;
unsigned long g_stateEnteredMs = 0;
int g_dosingCycleCount = 0;

bool g_collectionFull  = false;
bool g_collectionEmpty = false;
bool g_lastCollectionFull = false;

bool g_reject      = false;
bool g_submersible = false; // Collection -> Storage: auto TRANSFER + manual toggle
bool g_peristaltic = false;
bool g_diaphragm   = false; // Dosing tank -> Collection tank: manual-only, works in either mode

bool g_processingArmed = false;
float g_phValue = 7.0;
unsigned long g_dryTimerStart = 0;
bool g_isDry = true;
unsigned long g_doseCycleStartMs = 0; // when the current dosing cycle began (drives DOSE_CYCLE_MS)

// pH sampling during READING: collects one sample per PH_READ_INTERVAL_MS
// tick while stabilizing, so decisions use a median of ~10 readings over
// PH_STABILIZE_MS instead of a single instantaneous reading.
const int MAX_PH_SAMPLES = 20; // headroom above the ~10 expected in a 10s window
float g_phSamples[MAX_PH_SAMPLES];
int g_phSampleCount = 0;

unsigned long g_rejectStartMs      = 0;
unsigned long g_peristalticStartMs = 0;
unsigned long g_submersibleStartMs = 0;
unsigned long g_diaphragmStartMs   = 0;

bool g_transferCapTripped   = false;
bool g_dosingCapTripped     = false;
bool g_diaphragmCapTripped  = false;

unsigned long g_lastStatusPollMs = 0;

DFRobot_PH ph;
WebServer server(80);

// ── Forward declarations ────────────────────────────────────
void readFloats();
void readPHSensor();
float medianOfPhSamples();
void armIfCollectionFull();
void abortProcessing();
void transitionTo(ProcessState s);
void startDosingPulse();
void evaluateAndTransition();
void runAutoStateMachine();
void enforceRuntimeCaps();
void clearAllCaps();
void setReject(bool on);
void setSubmersible(bool on);
void setPeristaltic(bool on);
void setDiaphragm(bool on);
void allAutoPumpsOff();
void updateStatusLed();
void updateAlertLed();
void logStatus();
void sendPhToIotIfDue();
void setupWiFi();
void setupWebServer();
void handleRoot();
void handleApiStatus();
void handleApiSetMode();
void handleApiRelay();
const char* stageName();
unsigned long stageRemainingMs();

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  EEPROM.begin(32);
  ph.begin();

  pinMode(FLOAT_COLLECTION_FULL_PIN, INPUT_PULLUP);
  pinMode(FLOAT_COLLECTION_EMPTY_PIN, INPUT_PULLUP);

  pinMode(SSR_REJECT_PIN, OUTPUT);
  pinMode(SSR_SUBMERSIBLE_PIN, OUTPUT);
  pinMode(SSR_PERISTALTIC_PIN, OUTPUT);
  pinMode(SSR_DIAPHRAGM_PIN, OUTPUT);
  // Boot with every relay OFF -- active-LOW board, so OFF = HIGH.
  digitalWrite(SSR_REJECT_PIN, RELAY_OFF);
  digitalWrite(SSR_SUBMERSIBLE_PIN, RELAY_OFF);
  digitalWrite(SSR_PERISTALTIC_PIN, RELAY_OFF);
  digitalWrite(SSR_DIAPHRAGM_PIN, RELAY_OFF);

  pinMode(LED_STATUS_PIN, OUTPUT);
  // Alert LED disabled: its former pin (GPIO20) is now SSR_PERISTALTIC_PIN.

  setupWiFi();
  setupWebServer();

  // Read floats once before deciding whether to arm immediately
  readFloats();
  armIfCollectionFull();

  Serial.println(F("RAIN FILTER ESP32-C3 dashboard ready (AUTO mode)."));
}

// ── Main loop ────────────────────────────────────────────────
void loop() {
  server.handleClient();

  readFloats();
  readPHIfDue();

  if (g_mode == MODE_AUTO) {
    runAutoStateMachine();
  }

  enforceRuntimeCaps();
  enforceDiaphragmWatchdog();
  updateStatusLed();
  updateAlertLed();
  sendPhToIotIfDue();

  static unsigned long lastLog = 0;
  if (millis() - lastLog >= STATUS_LOG_INTERVAL_MS) {
    lastLog = millis();
    logStatus();
  }

  g_lastCollectionFull = g_collectionFull;
}

void readPHIfDue() {
  static unsigned long lastPhRead = 0;
  if (millis() - lastPhRead >= PH_READ_INTERVAL_MS) {
    lastPhRead = millis();
    readPHSensor();
  }
}

// ── WiFi / Web server setup ─────────────────────────────────
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("Connecting to WiFi"));

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(300);
    Serial.print(F("."));
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print(F("Connected. Dashboard: http://"));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println(F("WiFi connect failed -> starting fallback AP."));
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print(F("AP started. Dashboard: http://"));
    Serial.println(WiFi.softAPIP());
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/mode", HTTP_GET, handleApiSetMode);
  server.on("/api/relay", HTTP_GET, handleApiRelay);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
  server.begin();
}

// ── Sensors / floats ─────────────────────────────────────────
void readFloats() {
  int rawC  = digitalRead(FLOAT_COLLECTION_FULL_PIN);
  int rawCE = digitalRead(FLOAT_COLLECTION_EMPTY_PIN);
  g_collectionFull  = COLLECTION_FULL_ACTIVE_LOW  ? (rawC  == LOW) : (rawC  == HIGH);
  g_collectionEmpty = COLLECTION_EMPTY_ACTIVE_LOW ? (rawCE == LOW) : (rawCE == HIGH);

  if (g_mode == MODE_AUTO && g_collectionFull && !g_lastCollectionFull) {
    armIfCollectionFull();
  }
}

void readPHSensor() {
  float voltage = analogRead(PH_PIN) / 4095.0 * 3300.0; // ESP32-C3 ADC: 12-bit, ~3.3V ref
  g_phValue = ph.readPH(voltage, PH_ASSUMED_TEMP_C) + PH_CALIBRATION_OFFSET;

  // Collect this reading into the stabilization sample buffer while
  // waiting to make a decision -- see medianOfPhSamples() and the
  // PSTATE_READING case in runAutoStateMachine().
  if (g_state == PSTATE_READING && g_phSampleCount < MAX_PH_SAMPLES) {
    g_phSamples[g_phSampleCount++] = g_phValue;
  }

  bool lowNow = (g_phValue <= PH_DRY_THRESHOLD);
  if (lowNow) {
    if (g_dryTimerStart == 0) g_dryTimerStart = millis();
    if (millis() - g_dryTimerStart >= DRY_CONFIRM_MS) {
      g_isDry = true;
    }
  } else {
    g_dryTimerStart = 0;
    g_isDry = false;
  }
}

// Returns the median of the samples collected so far during the current
// READING stabilization window. Median (not mean) so a single noisy/spike
// reading can't skew the dosing decision. Falls back to the live g_phValue
// if somehow no samples were collected.
float medianOfPhSamples() {
  if (g_phSampleCount == 0) return g_phValue;

  float sorted[MAX_PH_SAMPLES];
  for (int i = 0; i < g_phSampleCount; i++) sorted[i] = g_phSamples[i];

  // Small N (~10-20) -- plain insertion sort is plenty fast here.
  for (int i = 1; i < g_phSampleCount; i++) {
    float key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  int mid = g_phSampleCount / 2;
  if (g_phSampleCount % 2 == 0) {
    return (sorted[mid - 1] + sorted[mid]) / 2.0;
  }
  return sorted[mid];
}

// ── Auto state machine ───────────────────────────────────────
void armIfCollectionFull() {
  if (g_mode == MODE_AUTO && g_collectionFull && g_state == PSTATE_IDLE) {
    g_processingArmed = true;
    g_dosingCycleCount = 0;
    transitionTo(PSTATE_READING);
    Serial.println(F("Collection tank FULL -> armed, reading pH..."));
  }
}

void transitionTo(ProcessState s) {
  g_state = s;
  g_stateEnteredMs = millis();
  if (s == PSTATE_READING) {
    g_phSampleCount = 0; // start a fresh sampling window for this stabilization period
  }
}

void abortProcessing() {
  allAutoPumpsOff();
  transitionTo(PSTATE_IDLE);
  g_processingArmed = false;
  Serial.println(F("Line reads dry (or aborted) -> processing cycle ended."));
}

void startDosingPulse() {
  if (g_dosingCycleCount >= MAX_DOSING_CYCLES) {
    setPeristaltic(false);
    setSubmersible(false);
    g_dosingCapTripped = true;
    transitionTo(PSTATE_IDLE);
    g_processingArmed = false;
    Serial.println(F("Max dosing cycles reached -> alert raised, needs acknowledgement."));
    return;
  }
  g_dosingCycleCount++;
  g_doseCycleStartMs = millis(); // anchors DOSE_CYCLE_MS for this whole dose+settle cycle
  setPeristaltic(true);
  transitionTo(PSTATE_DOSING);
}

void evaluateAndTransition() {
  if (g_phValue < PH_REJECT_BELOW || g_phValue > PH_REJECT_ABOVE) {
    setReject(true);
    transitionTo(PSTATE_REJECT);
  } else if (g_phValue < PH_DOSE_BELOW) {
    startDosingPulse();
  } else if (g_phValue > PH_HOLD_ABOVE) {
    transitionTo(PSTATE_HOLD);
  } else {
    setSubmersible(true);
    transitionTo(PSTATE_TRANSFER);
  }
}

void runAutoStateMachine() {
  switch (g_state) {
    case PSTATE_IDLE:
      allAutoPumpsOff();
      break;

    case PSTATE_READING:
      if (g_isDry) { abortProcessing(); break; }
      if (millis() - g_stateEnteredMs >= PH_STABILIZE_MS) {
        g_phValue = medianOfPhSamples(); // decide off the stabilized median, not one instantaneous reading
        evaluateAndTransition();
      }
      break;

    case PSTATE_TRANSFER:
      if (g_isDry) { abortProcessing(); break; }
      if (g_collectionEmpty) {
        // Collection tank has drained -> transfer done. Stop Submersible and
        // loop back to IDLE, ready to re-arm on the next Collection FULL trigger.
        setSubmersible(false);
        transitionTo(PSTATE_IDLE);
        g_processingArmed = false;
        Serial.println(F("Collection tank EMPTY -> transfer complete, cycle finished."));
        break;
      }
      if (g_phValue < PH_REJECT_BELOW || g_phValue > PH_REJECT_ABOVE) {
        setSubmersible(false);
        setReject(true);
        transitionTo(PSTATE_REJECT);
      } else if (g_phValue < PH_DOSE_BELOW) {
        setSubmersible(false);
        startDosingPulse();
      } else if (g_phValue > PH_HOLD_ABOVE) {
        setSubmersible(false);
        transitionTo(PSTATE_HOLD);
      } else {
        setSubmersible(true); // no Storage-full sensor on this build, so nothing gates this
      }
      break;

    case PSTATE_REJECT:
      if (g_isDry) { abortProcessing(); break; }
      if (g_collectionEmpty) {
        // Collection tank drained while rejecting -- stop the reject pump
        // rather than let it run dry, and loop back to IDLE, ready to
        // re-arm on the next Collection FULL trigger.
        setReject(false);
        transitionTo(PSTATE_IDLE);
        g_processingArmed = false;
        Serial.println(F("Collection tank EMPTY during REJECT -> stopped, cycle finished."));
        break;
      }
      if (g_phValue >= PH_REJECT_BELOW && g_phValue <= PH_REJECT_ABOVE) {
        setReject(false);
        transitionTo(PSTATE_READING); // re-stabilize before next decision
      }
      break;

    case PSTATE_HOLD:
      // Too alkaline to accept but not bad enough to reject yet -- pause with
      // no pumps running and keep monitoring. Re-evaluates the moment pH
      // moves back into a decisive band on either side.
      if (g_isDry) { abortProcessing(); break; }
      if (g_phValue > PH_REJECT_ABOVE) {
        setReject(true);
        transitionTo(PSTATE_REJECT);
      } else if (g_phValue <= PH_HOLD_ABOVE) {
        evaluateAndTransition();
      }
      break;

    case PSTATE_DOSING:
      // Peristaltic runs for the first DOSING_PULSE_MS of the cycle.
      if (g_isDry) { abortProcessing(); break; }
      if (millis() - g_doseCycleStartMs >= DOSING_PULSE_MS) {
        setPeristaltic(false);
        transitionTo(PSTATE_DOSE_WAIT);
      }
      break;

    case PSTATE_DOSE_WAIT:
      // Passive settle period (no relay running) until the cycle reaches
      // DOSE_CYCLE_MS total, then re-reads pH.
      if (g_isDry) { abortProcessing(); break; }
      if (millis() - g_doseCycleStartMs >= DOSE_CYCLE_MS) {
        transitionTo(PSTATE_READING);
      }
      break;
  }
}

const char* stageName() {
  switch (g_state) {
    case PSTATE_IDLE:       return "IDLE";
    case PSTATE_READING:    return "READING";
    case PSTATE_TRANSFER:   return "TRANSFERRING";
    case PSTATE_REJECT:     return "REJECTING";
    case PSTATE_DOSING:     return "DOSING";
    case PSTATE_DOSE_WAIT:  return "SETTLING";
    case PSTATE_HOLD:       return "HOLDING";
    default:                return "?";
  }
}

unsigned long stageRemainingMs() {
  unsigned long elapsed = millis() - g_stateEnteredMs;
  unsigned long doseElapsed = millis() - g_doseCycleStartMs; // DOSING/DOSE_WAIT both count from cycle start
  switch (g_state) {
    case PSTATE_READING:   return (elapsed < PH_STABILIZE_MS) ? (PH_STABILIZE_MS - elapsed) : 0;
    case PSTATE_DOSING:    return (doseElapsed < DOSING_PULSE_MS) ? (DOSING_PULSE_MS - doseElapsed) : 0;
    case PSTATE_DOSE_WAIT: return (doseElapsed < DOSE_CYCLE_MS) ? (DOSE_CYCLE_MS - doseElapsed) : 0;
    default:               return 0;
  }
}

// ── Manual pump control (web only) ────────────────────────────
void toggleManualPump(ManualPump p, bool on, bool &ok, String &reason) {
  switch (p) {
    case PUMP_TRANSFER:
      g_transferCapTripped = false;
      if (on && g_isDry) { ok = false; reason = "line dry"; return; }
      if (on && g_collectionEmpty) { ok = false; reason = "collection tank empty"; return; }
      setSubmersible(on);
      ok = true;
      break;
    case PUMP_REJECT:
      g_transferCapTripped = false;
      if (on && g_isDry) { ok = false; reason = "line dry"; return; }
      setReject(on);
      ok = true;
      break;
    case PUMP_DOSING:
      g_dosingCapTripped = false;
      setPeristaltic(on);
      ok = true;
      break;
    default:
      ok = false;
      reason = "unknown pump";
      break;
  }
}

// ── Runtime safety caps ─────────────────────────────────────
void enforceRuntimeCaps() {
  unsigned long now = millis();

  if (g_submersible && g_submersibleStartMs && (now - g_submersibleStartMs >= MAX_TRANSFER_RUNTIME_MS)) {
    setSubmersible(false);
    g_transferCapTripped = true;
    if (g_state == PSTATE_TRANSFER) { transitionTo(PSTATE_IDLE); g_processingArmed = false; }
    Serial.println(F("SUBMERSIBLE (transfer) hit max runtime -> stopped, needs acknowledgement."));
  }
  if (g_reject && g_rejectStartMs && (now - g_rejectStartMs >= MAX_TRANSFER_RUNTIME_MS)) {
    setReject(false);
    g_transferCapTripped = true;
    if (g_state == PSTATE_REJECT) { transitionTo(PSTATE_IDLE); g_processingArmed = false; }
    Serial.println(F("REJECT hit max runtime -> stopped, needs acknowledgement."));
  }
  if (g_peristaltic && g_peristalticStartMs && (now - g_peristalticStartMs >= MAX_DOSING_RUNTIME_MS)) {
    setPeristaltic(false);
    g_dosingCapTripped = true;
    if (g_state == PSTATE_DOSING) { transitionTo(PSTATE_IDLE); g_processingArmed = false; }
    Serial.println(F("DOSING hit max runtime -> stopped, check reservoir & acknowledge."));
  }
  if (g_diaphragm && g_diaphragmStartMs && (now - g_diaphragmStartMs >= MAX_DIAPHRAGM_RUNTIME_MS)) {
    setDiaphragm(false);
    g_diaphragmCapTripped = true;
    Serial.println(F("DIAPHRAGM hit max runtime -> stopped, re-enable from dashboard to continue."));
  }
}

void enforceDiaphragmWatchdog() {
  if (g_diaphragm && g_lastStatusPollMs != 0 &&
      (millis() - g_lastStatusPollMs > DIAPHRAGM_WATCHDOG_MS)) {
    setDiaphragm(false);
    Serial.println(F("Dashboard connection lost while diaphragm pump was running -> auto-stopped."));
  }
}

void clearAllCaps() {
  g_transferCapTripped = false;
  g_dosingCapTripped = false;
  g_diaphragmCapTripped = false;
}

// ── Relay/SSR helpers ────────────────────────────────────────
// Board is active-LOW: RELAY_ON = LOW (closes/pump runs), RELAY_OFF = HIGH.
void setReject(bool on) {
  if (on == g_reject) return;
  g_reject = on;
  digitalWrite(SSR_REJECT_PIN, on ? RELAY_ON : RELAY_OFF);
  g_rejectStartMs = on ? millis() : 0;
}
void setSubmersible(bool on) {
  if (on == g_submersible) return;
  g_submersible = on;
  digitalWrite(SSR_SUBMERSIBLE_PIN, on ? RELAY_ON : RELAY_OFF);
  g_submersibleStartMs = on ? millis() : 0;
}
void setPeristaltic(bool on) {
  if (on == g_peristaltic) return;
  g_peristaltic = on;
  digitalWrite(SSR_PERISTALTIC_PIN, on ? RELAY_ON : RELAY_OFF);
  g_peristalticStartMs = on ? millis() : 0;
}
void setDiaphragm(bool on) {
  if (on == g_diaphragm) return;
  g_diaphragm = on;
  digitalWrite(SSR_DIAPHRAGM_PIN, on ? RELAY_ON : RELAY_OFF);
  g_diaphragmStartMs = on ? millis() : 0;
}

void allAutoPumpsOff() {
  setSubmersible(false);
  setReject(false);
  setPeristaltic(false);
  // diaphragm is independent / manual-only (works in either mode), not touched here
}

// ── LEDs ─────────────────────────────────────────────────────
void updateStatusLed() {
  unsigned long now = millis();
  unsigned long period = (g_mode == MODE_AUTO) ? 1000 : 250;
  bool on = (now % period) < (period / 2);
  digitalWrite(LED_STATUS_PIN, on ? HIGH : LOW);
}

void updateAlertLed() {
  // Disabled in v3.2: GPIO20 (its old pin) now drives SSR_PERISTALTIC_PIN.
  // Alert/cap-tripped state is still visible on the web dashboard.
}

// ── Logging ──────────────────────────────────────────────────
void logStatus() {
  Serial.print(F("[Mode="));
  Serial.print(g_mode == MODE_AUTO ? F("AUTO") : F("MANUAL"));
  Serial.print(F(" Stage=")); Serial.print(stageName());
  Serial.print(F("] pH=")); Serial.print(g_phValue, 2);
  Serial.print(F(" dry=")); Serial.print(g_isDry);
  Serial.print(F(" phSamples=")); Serial.print(g_phSampleCount);
  Serial.print(F(" cycle=")); Serial.print(g_dosingCycleCount);
  Serial.print(F(" | CollectionFull=")); Serial.print(g_collectionFull);
  Serial.print(F(" CollectionEmpty=")); Serial.print(g_collectionEmpty);
  Serial.print(F(" | Reject=")); Serial.print(g_reject);
  Serial.print(F(" Submersible=")); Serial.print(g_submersible);
  Serial.print(F(" Peristaltic=")); Serial.print(g_peristaltic);
  Serial.print(F(" Diaphragm=")); Serial.println(g_diaphragm);
}

// ── IoT pH reporting ─────────────────────────────────────────
// POSTs a small JSON payload every PH_IOT_SEND_INTERVAL_MS. Best-effort:
// failures are logged but never block or affect the state machine.
void sendPhToIotIfDue() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend < PH_IOT_SEND_INTERVAL_MS) return;
  lastSend = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("IoT send skipped: WiFi not connected."));
    return;
  }

  String payload = "{";
  payload += "\"ph\":" + String(g_phValue, 2) + ",";
  payload += "\"stage\":\"" + String(stageName()) + "\",";
  payload += "\"isDry\":" + String(g_isDry ? "true" : "false") + ",";
  payload += "\"uptimeMs\":" + String(millis());
  payload += "}";

  HTTPClient http;
  http.begin(IOT_ENDPOINT_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(4000); // don't let a slow/unreachable server stall the loop

  int httpCode = http.POST(payload);
  if (httpCode > 0) {
    Serial.print(F("IoT pH send -> HTTP "));
    Serial.println(httpCode);
  } else {
    Serial.print(F("IoT pH send failed: "));
    Serial.println(http.errorToString(httpCode));
  }
  http.end();
}

// ── Web API handlers ─────────────────────────────────────────
void handleApiStatus() {
  g_lastStatusPollMs = millis();

  String json = "{";
  json += "\"mode\":\"" + String(g_mode == MODE_AUTO ? "AUTO" : "MANUAL") + "\",";
  json += "\"stage\":\"" + String(stageName()) + "\",";
  json += "\"stageRemainingMs\":" + String(stageRemainingMs()) + ",";
  json += "\"dosingCycle\":" + String(g_dosingCycleCount) + ",";
  json += "\"maxDosingCycles\":" + String(MAX_DOSING_CYCLES) + ",";
  json += "\"ph\":" + String(g_phValue, 2) + ",";
  json += "\"phSampleCount\":" + String(g_phSampleCount) + ",";
  json += "\"isDry\":" + String(g_isDry ? "true" : "false") + ",";
  json += "\"processingArmed\":" + String(g_processingArmed ? "true" : "false") + ",";
  json += "\"collectionFull\":" + String(g_collectionFull ? "true" : "false") + ",";
  json += "\"collectionEmpty\":" + String(g_collectionEmpty ? "true" : "false") + ",";
  json += "\"reject\":" + String(g_reject ? "true" : "false") + ",";
  json += "\"submersible\":" + String(g_submersible ? "true" : "false") + ",";
  json += "\"peristaltic\":" + String(g_peristaltic ? "true" : "false") + ",";
  json += "\"diaphragm\":" + String(g_diaphragm ? "true" : "false") + ",";
  json += "\"transferCapTripped\":" + String(g_transferCapTripped ? "true" : "false") + ",";
  json += "\"dosingCapTripped\":" + String(g_dosingCapTripped ? "true" : "false") + ",";
  json += "\"diaphragmCapTripped\":" + String(g_diaphragmCapTripped ? "true" : "false") + ",";
  json += "\"uptimeMs\":" + String(millis());
  json += "}";
  server.send(200, "application/json", json);
}

void handleApiSetMode() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "missing value");
    return;
  }
  String v = server.arg("value");
  SystemMode newMode = (v == "manual") ? MODE_MANUAL : MODE_AUTO;
  if (newMode != g_mode) {
    g_mode = newMode;
    if (g_mode == MODE_MANUAL) {
      allAutoPumpsOff();
      transitionTo(PSTATE_IDLE);
      g_processingArmed = false;
    } else {
      armIfCollectionFull(); // resume immediately if float is still full
    }
    clearAllCaps();
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiRelay() {
  // /api/relay?name=transfer|reject|dosing|diaphragm|ack&state=1|0
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "missing name");
    return;
  }
  String name = server.arg("name");
  bool state = server.hasArg("state") && server.arg("state") == "1";
  bool ok = true;
  String reason = "";

  if (name == "transfer") {
    if (g_mode != MODE_MANUAL) { ok = false; reason = "not in manual mode"; }
    else toggleManualPump(PUMP_TRANSFER, state, ok, reason);
  } else if (name == "reject") {
    if (g_mode != MODE_MANUAL) { ok = false; reason = "not in manual mode"; }
    else toggleManualPump(PUMP_REJECT, state, ok, reason);
  } else if (name == "dosing") {
    if (g_mode != MODE_MANUAL) { ok = false; reason = "not in manual mode"; }
    else toggleManualPump(PUMP_DOSING, state, ok, reason);
  } else if (name == "diaphragm") {
    g_diaphragmCapTripped = false;
    setDiaphragm(state);
  } else if (name == "ack") {
    clearAllCaps();
    if (g_mode == MODE_AUTO) armIfCollectionFull();
  } else {
    ok = false;
    reason = "unknown relay";
  }

  String json = "{\"ok\":" + String(ok ? "true" : "false");
  if (!ok) json += ",\"reason\":\"" + reason + "\"";
  json += "}";
  server.send(ok ? 200 : 409, "application/json", json);
}

// ── Web dashboard (served from flash, single page) ────────────
void handleRoot() {
  static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Rain Filter — Live Dashboard</title>
<style>
  :root{
    --bg:#0b1420; --panel:#111f30; --panel2:#0f1a29; --border:#1e3348;
    --teal:#2dd4bf; --teal-dim:#0f5c53; --amber:#f5a524; --amber-dim:#6b4a12;
    --red:#f0495a; --red-dim:#5c1a22; --text:#e6f1f5; --sub:#7b93a8;
    --green:#3ddc84;
  }
  *{box-sizing:border-box;}
  body{
    margin:0; font-family:'Segoe UI',system-ui,-apple-system,sans-serif;
    background:radial-gradient(circle at 20% -10%, #12293b 0%, var(--bg) 55%);
    color:var(--text); padding:20px; min-height:100vh;
  }
  .wrap{max-width:1100px;margin:0 auto;}
  header{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:10px;margin-bottom:20px;}
  h1{font-size:1.4rem;margin:0;letter-spacing:.5px;}
  h1 span{color:var(--teal);}
  .conn{font-size:.75rem;color:var(--sub);display:flex;align-items:center;gap:6px;}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green);}
  .dot.off{background:var(--red);box-shadow:0 0 8px var(--red);}

  .stagebar{
    background:linear-gradient(135deg,var(--panel),var(--panel2));
    border:1px solid var(--border);border-radius:14px;padding:16px 18px;margin-bottom:18px;
    display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:12px;
  }
  .stagebar .stagelabel{font-size:.7rem;color:var(--sub);text-transform:uppercase;letter-spacing:1px;}
  .stagebar .stageval{font-size:1.3rem;font-weight:800;letter-spacing:.5px;}
  .stagebar .stagesub{font-size:.78rem;color:var(--sub);margin-top:2px;}
  .stageval.st-idle{color:var(--sub);}
  .stageval.st-reading{color:#8fb8ff;}
  .stageval.st-transferring{color:var(--teal);}
  .stageval.st-rejecting{color:var(--red);}
  .stageval.st-dosing{color:var(--amber);}
  .stageval.st-settling{color:var(--amber);}
  .stageval.st-holding{color:var(--amber);}

  .modebar{
    display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:12px;
    background:linear-gradient(135deg,var(--panel),var(--panel2));
    border:1px solid var(--border);border-radius:14px;padding:14px 18px;margin-bottom:18px;
  }
  .modebar .label{font-size:.8rem;color:var(--sub);text-transform:uppercase;letter-spacing:1px;}
  .switch{
    position:relative;width:150px;height:40px;background:#0a1622;border-radius:20px;
    border:1px solid var(--border);cursor:pointer;display:flex;align-items:center;
  }
  .switch .knob{
    position:absolute;top:3px;left:3px;width:71px;height:32px;border-radius:16px;
    background:linear-gradient(135deg,var(--teal),#0ea89a);transition:.25s ease;
    display:flex;align-items:center;justify-content:center;font-size:.75rem;font-weight:600;color:#04211d;
  }
  .switch.manual .knob{left:76px;background:linear-gradient(135deg,var(--amber),#c97e0c);color:#2b1c00;}
  .switch .txt{position:absolute;width:100%;display:flex;justify-content:space-between;padding:0 14px;font-size:.7rem;color:var(--sub);pointer-events:none;}

  .ack{
    background:var(--red-dim);border:1px solid var(--red);color:#ffd6da;
    padding:8px 14px;border-radius:10px;font-size:.8rem;cursor:pointer;display:none;
  }

  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:14px;margin-bottom:18px;}
  .card{
    background:linear-gradient(160deg,var(--panel),var(--panel2));
    border:1px solid var(--border);border-radius:14px;padding:16px;position:relative;overflow:hidden;
  }
  .card .t{font-size:.72rem;color:var(--sub);text-transform:uppercase;letter-spacing:1px;margin-bottom:6px;}
  .card .v{font-size:1.5rem;font-weight:700;}
  .badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:.72rem;font-weight:600;margin-top:6px;}
  .badge.on{background:rgba(61,220,132,.15);color:var(--green);border:1px solid rgba(61,220,132,.4);}
  .badge.off{background:rgba(123,147,168,.12);color:var(--sub);border:1px solid var(--border);}
  .badge.warn{background:rgba(245,165,36,.15);color:var(--amber);border:1px solid rgba(245,165,36,.4);}
  .badge.danger{background:rgba(240,73,90,.15);color:var(--red);border:1px solid rgba(240,73,90,.4);}

  .phcard .v{color:var(--teal);}
  .phband{height:6px;border-radius:4px;margin-top:10px;
    background:linear-gradient(90deg,var(--red) 0%, var(--amber) 45%, var(--teal) 65%, var(--teal) 100%);}
  .phmarker{width:2px;height:14px;background:#fff;position:relative;top:-14px;border-radius:2px;box-shadow:0 0 4px #fff;}

  section{margin-bottom:22px;}
  h2{font-size:.95rem;color:var(--sub);text-transform:uppercase;letter-spacing:1px;margin:0 0 10px 2px;}

  .relaygrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;}
  .relay{
    background:var(--panel2);border:1px solid var(--border);border-radius:12px;padding:14px;text-align:center;
  }
  .relay .name{font-size:.78rem;color:var(--sub);margin-bottom:8px;}
  .relay .state{font-size:.95rem;font-weight:700;}
  .relay.active{border-color:var(--teal);box-shadow:0 0 16px rgba(45,212,191,.25);}
  .relay.active .state{color:var(--teal);}
  .relay.inactive .state{color:var(--sub);}

  .manualpanel{display:none;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;}
  .manualpanel.show{display:grid;}
  .btn{
    background:var(--panel2);border:1px solid var(--border);border-radius:12px;padding:14px;
    color:var(--text);cursor:pointer;font-size:.85rem;font-weight:600;transition:.15s;
  }
  .btn:hover{border-color:var(--teal);}
  .btn.on{background:linear-gradient(135deg,var(--teal-dim),#0a3b35);border-color:var(--teal);color:var(--teal);}
  .btn small{display:block;font-weight:400;color:var(--sub);margin-top:4px;font-size:.7rem;}

  .chartbox{
    background:linear-gradient(160deg,var(--panel),var(--panel2));
    border:1px solid var(--border);border-radius:14px;padding:16px;
  }
  canvas{width:100%;height:220px;display:block;}
  footer{text-align:center;color:var(--sub);font-size:.72rem;margin-top:26px;}
</style>
</head>
<body>
<div class="wrap">
  <header>
    <h1>RAIN <span>FILTER</span> — Live Dashboard</h1>
    <div class="conn"><span class="dot" id="connDot"></span><span id="connTxt">connecting…</span></div>
  </header>

  <div class="stagebar">
    <div>
      <div class="stagelabel">Process stage</div>
      <div class="stageval st-idle" id="stageVal">—</div>
      <div class="stagesub" id="stageSub"></div>
    </div>
    <div style="text-align:right;">
      <div class="stagelabel">Dosing cycles</div>
      <div class="stageval" style="font-size:1.1rem;" id="cycleVal">0 / 10</div>
    </div>
  </div>

  <div class="modebar">
    <div>
      <div class="label">System mode</div>
      <div class="switch" id="modeSwitch" onclick="toggleMode()">
        <div class="txt"><span>AUTO</span><span>MANUAL</span></div>
        <div class="knob" id="modeKnob">AUTO</div>
      </div>
    </div>
    <button class="ack" id="ackBtn" onclick="ackAlert()">⚠ Acknowledge alert</button>
  </div>

  <div class="grid">
    <div class="card"><div class="t">Collection Tank</div><div class="v" id="collVal">—</div>
      <span class="badge off" id="collBadge">unknown</span></div>
    <div class="card"><div class="t">Collection Empty?</div><div class="v" id="collEmptyVal">—</div>
      <span class="badge off" id="collEmptyBadge">unknown</span></div>
    <div class="card"><div class="t">Transfer Line</div><div class="v" id="lineVal">—</div>
      <span class="badge off" id="lineBadge">unknown</span></div>
    <div class="card phcard"><div class="t">pH Reading</div><div class="v" id="phVal">—</div>
      <div class="phband"></div><div class="phmarker" id="phMarker"></div></div>
  </div>

  <section>
    <h2>Relay Status</h2>
    <div class="relaygrid">
      <div class="relay inactive" id="relaySubmersible"><div class="name">SUBMERSIBLE<br>Collection → Storage</div><div class="state">OFF</div></div>
      <div class="relay inactive" id="relayReject"><div class="name">REJECT<br>Collection → Waste</div><div class="state">OFF</div></div>
      <div class="relay inactive" id="relayPeristaltic"><div class="name">PERISTALTIC<br>Neutralizer dosing</div><div class="state">OFF</div></div>
      <div class="relay inactive" id="relayDiaphragm"><div class="name">DIAPHRAGM<br>Dosing tank → Collection</div><div class="state">OFF</div></div>
    </div>
  </section>

  <section>
    <h2>Manual Controls <span id="manualHint" style="text-transform:none;font-weight:400;"></span></h2>
    <div class="manualpanel" id="manualPanel">
      <div class="btn" id="btnTransfer" onclick="setRelay('transfer', !state.submersible)">TRANSFER<small>Submersible — Collection → Storage</small></div>
      <div class="btn" id="btnReject" onclick="setRelay('reject', !state.reject)">REJECT<small>Collection → Waste</small></div>
      <div class="btn" id="btnDosing" onclick="setRelay('dosing', !state.peristaltic)">DOSING<small>Peristaltic pump</small></div>
    </div>
  </section>

  <section>
    <h2>Diaphragm Pump (works in either mode)</h2>
    <div class="manualpanel show">
      <div class="btn" id="btnDiaphragm" onclick="setRelay('diaphragm', !state.diaphragm)">DIAPHRAGM<small>Dosing tank → Collection — toggle, safety-capped, auto-stops if this page disconnects</small></div>
    </div>
  </section>

  <section>
    <h2>pH History</h2>
    <div class="chartbox"><canvas id="phChart"></canvas></div>
  </section>

  <footer>Rain Filter ESP32-C3 Dashboard · polls every 30s · uptime <span id="uptime">—</span></footer>
</div>

<script>
let state = {};
let history = [];
const MAX_POINTS = 60;

function toggleMode(){
  const next = state.mode === 'AUTO' ? 'manual' : 'auto';
  fetch('/api/mode?value=' + next).then(refresh);
}
function setRelay(name, on){
  fetch('/api/relay?name=' + name + '&state=' + (on ? '1':'0')).then(refresh);
}
function ackAlert(){
  fetch('/api/relay?name=ack&state=1').then(refresh);
}

function badge(el, on, textOn, textOff, cls){
  el.textContent = on ? textOn : textOff;
  el.className = 'badge ' + (on ? (cls||'on') : 'off');
}

function relayCard(id, on){
  const el = document.getElementById(id);
  el.className = 'relay ' + (on ? 'active' : 'inactive');
  el.querySelector('.state').textContent = on ? 'RUNNING' : 'OFF';
}

function drawChart(){
  const c = document.getElementById('phChart');
  const ctx = c.getContext('2d');
  const w = c.clientWidth, h = 220;
  c.width = w; c.height = h;
  ctx.clearRect(0,0,w,h);

  ctx.strokeStyle = '#1e3348';
  ctx.lineWidth = 1;
  for(let i=0;i<=4;i++){
    const y = 10 + (h-30) * i/4;
    ctx.beginPath(); ctx.moveTo(30,y); ctx.lineTo(w-10,y); ctx.stroke();
    ctx.fillStyle = '#7b93a8'; ctx.font = '10px sans-serif';
    ctx.fillText((14 - i*3.5).toFixed(1), 4, y+3);
  }

  if(history.length < 2) return;
  const minX = 30, maxX = w-10, minY = h-20, maxY = 10;
  const n = history.length;

  ctx.beginPath();
  history.forEach((p,i)=>{
    const x = minX + (maxX-minX) * (i/(MAX_POINTS-1));
    const y = maxY + (minY-maxY) * (1 - (p.ph/14));
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  });
  ctx.strokeStyle = '#2dd4bf';
  ctx.lineWidth = 2;
  ctx.stroke();

  ctx.lineTo(minX + (maxX-minX) * ((n-1)/(MAX_POINTS-1)), minY);
  ctx.lineTo(minX, minY);
  ctx.closePath();
  ctx.fillStyle = 'rgba(45,212,191,0.08)';
  ctx.fill();
}

function stageSubtext(d){
  if(d.stage === 'READING') return 'Sampling pH (' + d.phSampleCount + ' readings so far)… ' + Math.ceil(d.stageRemainingMs/1000) + 's left';
  if(d.stage === 'DOSING') return 'Peristaltic running… ' + Math.ceil(d.stageRemainingMs/1000) + 's left';
  if(d.stage === 'SETTLING') return 'Waiting to re-check pH… ' + Math.ceil(d.stageRemainingMs/1000) + 's';
  if(d.stage === 'TRANSFERRING') return 'Clean water flowing to Storage';
  if(d.stage === 'REJECTING') return d.ph > 8 ? 'Too alkaline — dumping to waste' : 'Too acidic — dumping to waste';
  if(d.stage === 'HOLDING') return 'pH 8.5–9.0 — holding, monitoring for change';
  if(d.stage === 'IDLE') return d.mode === 'MANUAL' ? 'Manual mode' : 'Waiting for Collection tank to fill';
  return '';
}

function refresh(){
  fetch('/api/status').then(r=>{
    document.getElementById('connDot').classList.remove('off');
    document.getElementById('connTxt').textContent = 'connected';
    return r.json();
  }).then(d=>{
    state = d;

    const sv = document.getElementById('stageVal');
    sv.textContent = d.stage;
    sv.className = 'stageval st-' + d.stage.toLowerCase();
    document.getElementById('stageSub').textContent = stageSubtext(d);
    document.getElementById('cycleVal').textContent = d.dosingCycle + ' / ' + d.maxDosingCycles;

    document.getElementById('modeSwitch').className = 'switch' + (d.mode==='MANUAL' ? ' manual':'');
    document.getElementById('modeKnob').textContent = d.mode;

    badge(document.getElementById('collBadge'), d.collectionFull, 'FULL','not full');
    document.getElementById('collVal').textContent = d.collectionFull ? 'FULL' : 'OK';
    badge(document.getElementById('collEmptyBadge'), d.collectionEmpty, 'EMPTY','has water', d.collectionEmpty?'warn':'on');
    document.getElementById('collEmptyVal').textContent = d.collectionEmpty ? 'EMPTY' : 'OK';
    badge(document.getElementById('lineBadge'), !d.isDry, 'WET / active','DRY', d.isDry?'danger':'on');
    document.getElementById('lineVal').textContent = d.isDry ? 'DRY' : 'WET';

    document.getElementById('phVal').textContent = d.ph.toFixed(2);
    document.getElementById('phMarker').style.marginLeft = Math.max(0,Math.min(100,(d.ph/14*100))) + '%';

    relayCard('relaySubmersible', d.submersible);
    relayCard('relayReject', d.reject);
    relayCard('relayPeristaltic', d.peristaltic);
    relayCard('relayDiaphragm', d.diaphragm);

    const manualPanel = document.getElementById('manualPanel');
    manualPanel.className = 'manualpanel' + (d.mode==='MANUAL' ? ' show':'');
    document.getElementById('manualHint').textContent = d.mode==='MANUAL' ? '' : '(switch to MANUAL to enable)';

    ['btnTransfer','btnReject','btnDosing'].forEach(id=>{
      document.getElementById(id).classList.remove('on');
    });
    if(d.submersible) document.getElementById('btnTransfer').classList.add('on');
    if(d.reject) document.getElementById('btnReject').classList.add('on');
    if(d.peristaltic) document.getElementById('btnDosing').classList.add('on');
    if(d.diaphragm) document.getElementById('btnDiaphragm').classList.add('on'); else document.getElementById('btnDiaphragm').classList.remove('on');

    const anyCap = d.transferCapTripped || d.dosingCapTripped || d.diaphragmCapTripped;
    document.getElementById('ackBtn').style.display = anyCap ? 'inline-block' : 'none';

    document.getElementById('uptime').textContent = Math.floor(d.uptimeMs/1000) + 's';

    history.push({t:Date.now(), ph:d.ph});
    if(history.length > MAX_POINTS) history.shift();
    drawChart();
  }).catch(()=>{
    document.getElementById('connDot').classList.add('off');
    document.getElementById('connTxt').textContent = 'disconnected — retrying…';
  });
}

refresh();
setInterval(refresh, 30000);
window.addEventListener('resize', drawChart);
</script>
</body>
</html>
)HTML";
  server.send_P(200, "text/html", PAGE);
}
