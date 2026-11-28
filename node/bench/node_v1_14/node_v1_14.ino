/*
 * PROJECT: Homeostat pot node, bring-up
 * DEVICE:  ESP32-DevKitC-32E + Adafruit DRV8833 + Alps 16 mm motorised pot (100k)
 *
 * v1.14: one clock for the ring. v1.13 stamped what it heard with millis() but judged ages
 * against the loop's 'now', taken a moment earlier, so a line heard this pass was "minus one
 * millisecond" old: unsigned, a few billion. On a loopback jumper the ring read open and whole by
 * turns; running, every other node would have looked unheard. Everything now uses the loop's 'now'.
 *
 * v1.13: the wired ring. The nodes talk to each other on UART2 (TX GPIO17 through 220 R to the
 * next node's RX GPIO16, round all four, one shared ground), so the homeostat runs with no laptop.
 * Every line a node hears from another node it passes on, one hop further; its own line coming
 * back means the ring is whole (and a jumper from 17 to 16 on one node is the first test). Always,
 * running or not: a ping every PING_MS and the firmware version every 5 s, so 'ring?' shows who is
 * on the ring and what each runs. 'ring' starts a run here and sends it round ('G'), so every node
 * starts the same preset, seed and timer; 'ringstop' stops them all ('S'). While running, each
 * node sends its unit every RING_MS ('X': model time, x, uniselector, hand). The lowest unit heard
 * keeps the clock and the others step only as far as its time, as on the sim page; a unit not
 * heard for RING_STALE_MS is simulated here from where it was last heard, so a broken ring slows
 * nothing and a node that comes back takes its unit back.
 *
 * v1.12: a feed-forward that fits the motor. FF0 550 dated from v1.9, when the feed-forward was
 * spiky and mostly off; steady (v1.10 on) it drove the needle at ~15 deg/s while TL usually wants
 * 3-6, so the needle ran ahead, the push eased off, it stalled, and went again: jerky, worst around
 * zero where TL passes most. Measured speed against duty on both nodes (node 1 starts at ~30 %,
 * node 2 at ~20 %; ~2 % duty per deg/s above that), so FF0 and KV now default to node 1's fit and
 * are kept in flash per node like the calibration ('ff0 N' / 'kv X' save as they set). With the
 * lighter push a stiff stretch could stall the needle short of a slow setpoint (node 2 at +38),
 * which the hand check then took for a hand; so behind by LAG_FLOOR, the duty floor applies.
 *
 * v1.11: any unit. A node now knows which unit it is (TL, TR, BR or BL: 'id N', kept in flash)
 * and keeps its own pot calibration in flash ('lo' / 'zero' / 'hi' / 'r' / 'min' save as they set), so
 * the same sketch goes on every node. Linked to the sim page, one node keeps the clock (the
 * master) and the others step only up to the time the page passes on from it, so two or more
 * nodes run one homeostat in lockstep. 'N' now carries all four needles: N t x0 x1 x2 x3.
 *
 * v1.10: smoother needle. v1.9 took the setpoint's speed from whole counts over one 2-3 ms loop,
 * so it read 0 most loops and ~400 counts/s the next: the feed-forward banged between nothing and
 * full duty, which was the jerk. Now the setpoint is a float in counts, its speed is low-passed
 * (SP_TAU), the feed-forward switches on and off with hysteresis, and TL is interpolated between
 * the model's 10 ms steps (each one is 40 ms of real time at x0.25) instead of jumping per step.
 * A steady feed-forward ran the needle a few degrees ahead of TL, and the hand check took that
 * for a hand (error growing while driving) and froze TL: run, pause, run. So the feed-forward
 * eases off when the needle is ahead, and only a push towards the target counts for the hand.
 *
 * v1.9: linked to the sim page. 'link' starts TL here with TR/BR/BL run by the page (through
 * the layout editor's /api/node): the page sends its three needles ('N t x1 x2 x3', ~20 Hz) and
 * this node keeps the clock, integrating TL against them and sending back a JSON line every
 * 50 ms (TL, its uniselector, the hand, the physical needle, model time). The page steps its
 * units to this node's time, so the whole homeostat runs at this node's SPEED. If the page goes
 * quiet for LINK_STALE_MS the model pauses here (no running on with stale neighbours).
 *
 * v1.8: the unit runs here. 'run' starts PT's four-unit model (unit.h, line for line from the
 * browser sim) with this node as TL: the needle follows TL, TR/BR/BL are simulated until real
 * nodes join. A hand on the knob holds TL where the hand puts it, as dragging TL's dial does in
 * the sim; let go and TL carries on from there, and the other three feel it. The model runs at
 * SPEED x real time (the motor manages ~28 deg/s), and the needle's setpoint is rate-limited
 * with a feed-forward push so it keeps up. Uniselector steps are printed as they happen.
 *
 * v1.7: the hand. In 'follow' the motor holds the needle where it is told, with no time limit.
 * Take the knob and push against it: the motor goes slack and the hand has the needle. Leave
 * the knob still for STILL_MS and the motor takes it back, holding it where the hand left it.
 * The hand outranks the machine: it proposes, a person can overrule. Judged by displacement
 * (no touch track), as on the TL fader: off by more than HAND_ERR and not winning back
 * HAND_GAIN within HAND_MS while driving.
 * Also: a wiper jump filter (a reading the knob couldn't have made is ignored unless it stays)
 * and a fault when the wiper reads where this wiring can't go (a lost wiper reads ~4095).
 *
 * Motor is OFF at boot and only moves when told.
 *
 * CONFIGURATION:
 *   POS_LOW / POS_ZERO / POS_HIGH   counts at the -50 / 0 / +50 deg marks ('lo' / 'zero' / 'hi')
 *   motorReversed        flip with 'r' if + jog makes the counts go DOWN
 *
 * WIRING (see node/img/node_wiring.svg):
 *   pot track ends -> 3V3 / GND (own GND pin), wiper -> GPIO34 + 100 nF to pot GND
 *   DRV8833 AIN1 -> GPIO25, AIN2 -> GPIO26, SLP -> GPIO27, FLT -> GPIO35 (+10k to 3V3)
 *   DRV8833 AOUT1/AOUT2 -> pot motor, VM/GND -> motor supply, GND common with ESP32
 *   ring: GPIO17 (TX2) -> 220 R -> next node's GPIO16 (RX2); GPIO16 <- previous node's TX; GND shared by all
 *
 * SERIAL (115200):
 *   V            version
 *   ?            one status line now
 *   q            stream on/off (every 100 ms: raw, mV, deg, flt, motor)
 *   + / -        jog 150 ms at jog duty, prints counts before/after
 *   j N          jog duty 0-100 % (default 35)
 *   u N          speed test: run N % for 1 s towards the middle, print deg/s
 *   id N         this node is unit N (0 TL, 1 TR, 2 BR, 3 BL), saved      cal  show the saved calibration
 *   link P S G T M  start this unit linked to the sim page: preset P, seed S, signed G (0/1), timer T s,
 *                M 1 = keep the clock, 0 = follow the time in N
 *   ring [P S G T]  start a run on the ring from here: preset P, seed S, signed G, timer T s (all
 *                optional); every node on the ring starts it too.   ringstop  stop it on every node
 *   ring?        the ring: whole or open, who is on it and what each runs
 *   N t x0 x1 x2 x3  all four needles at the clock's time t (keeps the link alive; own one ignored)
 *   d0 X         this unit's coil D (self-feedback), as the page's slider
 *   run [P]      start the unit (preset P 0-5, default 0) from where the knob is; 'stop' to stop
 *   speed X      model time per real second (default 0.25)     period X   uniselector timer, s
 *   seed N       seed for the next 'run' (0 = the preset's)    rate X     setpoint deg/s limit
 *   ff0 N / kv X feed-forward: duty to get moving, duty per count/s of setpoint speed. Saved
 *   tau X        low-pass on the setpoint's speed, s (default 0.08)
 *   lag N        counts behind at which the duty floor applies while moving (default 14)
 *   ahead N      counts ahead of the setpoint at which the feed-forward is gone (default 12)
 *   check        run preset 0 headless 100 frames: compare with homeostatHeadless(0, 100) in the sim
 *   f D          follow: hold D degrees (-50..50) with no time limit; the hand can take over
 *   f            follow from where the needle is now
 *   he N / hg N / hm N / still N   hand: error counts, win-back counts, window ms, release ms
 *   h D          hold at D degrees (-50..50) for 8 s, then coast (no hand detection)
 *   s            stop (coast)      b  brake
 *   r            reverse motor direction
 *   kp X / min N / max N   hold tuning: duty per count of error, duty floor, duty cap (0-1023)
 *   db N         hold deadband in counts (~5.5 counts a degree)
 *   lo / zero / hi  take the current reading as -50 / 0 / +50 deg (lo N etc: set the count). Saved
 *   sleep / wake DRV8833 SLP low / high
 */

#define VERSION "1.14"
#include "unit.h"
#include <Preferences.h>         // the node's unit and calibration, kept in flash (v1.11)
#if __has_include("version.h")
#include "version.h"            // written by node/flash.ps1 from git
#endif
#ifndef FW_VERSION
#define FW_VERSION "unversioned build"
#endif

// ---- pins ----
const int PIN_WIPER = 34;   // ADC1_CH6, input only: keeps working when the radio is on
const int PIN_AIN1  = 25;
const int PIN_AIN2  = 26;
const int PIN_SLP   = 27;
const int PIN_FLT   = 35;   // input only, external 10k pull-up; low = DRV8833 fault

// ---- motor ----
const uint32_t PWM_FREQ = 20000;   // above hearing
const uint8_t  PWM_BITS = 10;      // duty 0-1023
const int      FULL     = (1 << PWM_BITS) - 1;
int  jogPercent = 35;
bool motorReversed = true;          // v1.3: measured on the bench, + duty = counts up (towards CCW)
const uint32_t JOG_MS  = 150;
const uint32_t HOLD_MS = 8000;     // RK168 is slow by design: ~12 s per 300 deg
float KP = 4.0f;                   // duty per count of error while holding (v1.5: 1.5 was slow on the
                                   // real scale, ~5.5 counts a degree: most moves sat at MIN_DUTY)
int   MIN_DUTY = 500, MAX_DUTY = 1023; // 4.5 V motor on its own 5 V: full duty is in spec
int DEADBAND = 6;                  // counts: ~1 deg on the real scale (v1.6; 15 stopped ~2.7 deg short)

// ---- the needle's window: 100 deg of the pot's ~300 ----
// v1.2: 3V3 on pot pin 2 (behind ~88k), GND on pin 3: wiper runs 0..~1.67 V, raw 0..~1925 over 300 deg.
// CCW = high counts. -50 deg is the CCW side. ~7 counts a degree; set from a hand sweep, refine with marks.
// v1.4: read off the marks on the faceplate, knob resting, 2 s each (+-2 counts).
// The halves differ (260 vs 290 counts), so two straight lines meet at 0: every mark exact.
int POS_LOW  = 1779;               // -50 deg mark (CCW side)
int POS_ZERO = 1519;               //   0 deg mark
int POS_HIGH = 1229;               // +50 deg mark (CW side)
const int WINDOW_SLACK = 150;      // counts beyond the window the motor may still push outward

// ---- the hand (v1.7) ----
// ~5.5 counts a degree. The motor is slow (~28 deg/s flat out, a few deg/s near the duty floor),
// so the test is "not winning back", not "losing".
int HAND_ERR  = 22;                // counts (~4 deg) off target before a hand is suspected
int HAND_GAIN = 3;                 // counts the motor must win back per window to clear suspicion
uint32_t HAND_MS  = 250;           // the window
uint32_t STILL_MS = 700;           // knob still this long under a hand = let go
const int STILL_BAND = 6;          // counts (~1 deg): "still", above the +-2 count noise

// ---- the wiper's honesty ----
const int WIPER_MAX = 2300;        // this wiring tops out ~1925 (1.65 V); above = wiper lost
const int JUMP      = 120;         // counts (~22 deg) in one reading: not a knob, a contact
const int JUMP_KEEP = 8;           // ...unless it stays this many readings, then believe it
const uint32_t LOST_MS = 300;      // wiper out of range this long while driving = fault

enum MotorMode { COAST, BRAKE, JOG, HOLD, FOLLOW, HELD, FAULT };
MotorMode mode = COAST;
const char *modeName[] = {"coast", "brake", "jog", "hold", "follow", "held", "fault"};

bool stream = true;
int holdTarget = 0;
uint32_t modeUntil = 0, lastReport = 0;
int lastDuty = 0;
String line;

int pos = -1;                      // the needle, as believed (filtered)
int jumpRun = 0, jumpRef = 0;      // a run of readings far from pos, and where it is
uint32_t lostSince = 0;
uint32_t winStart = 0; int winErr = 0;                           // hand window
uint32_t driveSince = 0;           // when the push started: a motor starting from rest is slow,
                                   // so the first HAND_MS of any push is never judged
uint32_t stillSince = 0; int stillRef = 0;                       // held: is the knob still?

// ---- who this node is (v1.11) ----
Preferences prefs;
int ME = 0;                        // unit index: 0 TL, 1 TR, 2 BR, 3 BL ('id N')
bool clockMaster = true;           // linked: this node keeps the clock; else it follows N's t
double clockLimit = 0;             // a follower steps no further than this

extern int FF0; extern float KV;  // declared with the unit below
void saveCal() {
  prefs.putInt("lo", POS_LOW); prefs.putInt("zero", POS_ZERO); prefs.putInt("hi", POS_HIGH);
  prefs.putBool("rev", motorReversed); prefs.putInt("min", MIN_DUTY);
  prefs.putInt("ff0", FF0); prefs.putFloat("kv", KV);
}

// ---- the unit (v1.8) ----
unit::Model model;
bool  unitOn = false;
int   presetIdx = 0;
uint32_t seedOverride = 0;
bool  signedSel = true;            // "uniselector can reverse polarity", ticked by default in the sim
float SPEED   = 0.25f;             // model seconds per real second (the sim's slowest setting)
float SP_RATE = 15.0f;             // deg/s: the needle's setpoint chases TL no faster than this
int   FF0 = 280;                   // feed-forward: duty to get the motor moving (v1.12: measured, node 1; node 2 190)
float KV  = 2.8f;                  // ...plus this much per count/s of setpoint speed (node 1 ~15 duty per deg/s; node 2 4.1)
double spDeg = 0;                  // the needle's setpoint, deg (rate-limited TL)
float spVel = 0;                   // its speed, counts/s, low-passed (for the feed-forward)
float SP_TAU = 0.08f;              // s: the low-pass on that speed ('tau X')
bool  ffOn = false;                // feed-forward engaged (hysteresis: on above FF_ON, off below FF_OFF)
const float FF_ON = 4.0f, FF_OFF = 2.0f;   // counts/s
int LAG_FLOOR = 14;                // counts (~2.5 deg) behind: the duty floor applies even while moving ('lag N')
int AHEAD = 12;                    // counts (~2 deg) ahead of the setpoint: the feed-forward is gone ('ahead N')
double modelAcc = 0;               // model time owed, s
uint32_t lastUnitMs = 0;

// ---- the link to the sim page (v1.9) ----
bool linkOn = false;               // TR/BR/BL come from the page
uint32_t lastN = 0;                // when the page last sent its needles
const uint32_t LINK_STALE_MS = 1000;
uint32_t lastJson = 0;
int lastStepMask = 0;              // TL's last uniselector step, reported once in the JSON

// ---- the wired ring (v1.13) ----
const int PIN_RING_RX = 16, PIN_RING_TX = 17;   // UART2: in from the previous node, out to the next
const uint32_t RING_BAUD = 115200;
const uint32_t RING_MS = 40;       // own unit this often while running: 4 nodes x ~32 bytes x 25/s, under half the line
const uint32_t PING_MS = 500;      // own ping, always: the ring test, running or not
const uint32_t RING_STALE_MS = 600;   // a unit not heard this long is simulated here
const uint32_t WHOLE_MS = 1500;    // own ping not back this long: the ring is open
const int RING_MAX_HOPS = 4;       // a line passed on this many times is dropped (a loop with no owner)
bool ringOn = false;               // the unit runs with the other nodes over the ring
uint32_t ringEpoch = 0;            // which run: a 'G' for the run already going is ignored
uint32_t heardX[4] = {0, 0, 0, 0};   // when each unit's X was last heard
uint32_t heardAny[4] = {0, 0, 0, 0}; // when each node was last heard at all
float ringT[4] = {0, 0, 0, 0};     // each unit's model time, as last heard
String ringVersion[4];
uint32_t ringSeq = 0, lastRingX = 0, lastPing = 0, lastVer = 0;
uint32_t pingSeq = 0, pingSentMs = 0, pingBackMs = 0, pingRtt = 0;
uint32_t ringBad = 0, ringDropped = 0, ringPassed = 0;
bool ringWhole = false;
String ringLine;

// ---- reading the needle ----
int readRaw() {                    // 16 samples averaged: the 100k track is a soft source
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PIN_WIPER);
  return sum / 16;
}

// one reading through the filter. Returns false while the wiper reads where it can't be.
bool readNeedle() {
  int raw = readRaw();
  if (raw > WIPER_MAX) { if (!lostSince) lostSince = millis(); return false; }
  lostSince = 0;
  if (pos < 0) { pos = raw; return true; }
  if (abs(raw - pos) <= JUMP) { pos = raw; jumpRun = 0; return true; }
  // a jump: believe it only if it stays put
  if (jumpRun > 0 && abs(raw - jumpRef) <= JUMP) jumpRun++;
  else { jumpRun = 1; jumpRef = raw; }
  if (jumpRun >= JUMP_KEEP) {
    Serial.printf("needle re-anchored %d -> %d (a real move, or a contact that stayed)\n", pos, raw);
    pos = raw; jumpRun = 0;
  }
  return true;
}

float rawToDeg(int raw) {          // two lines, -50..0 and 0..+50, meeting at the 0 mark
  bool lowSide = (POS_LOW > POS_ZERO) ? raw >= POS_ZERO : raw <= POS_ZERO;
  if (lowSide) return -50.0f * (raw - POS_ZERO) / float(POS_LOW - POS_ZERO);
  return 50.0f * (raw - POS_ZERO) / float(POS_HIGH - POS_ZERO);
}

float degToRawF(float d) {         // the same, unrounded: counts as a float
  if (d <= 0) return POS_ZERO + -d / 50.0f * (POS_LOW - POS_ZERO);
  return POS_ZERO + d / 50.0f * (POS_HIGH - POS_ZERO);
}
int degToRaw(float d) { return int(lroundf(degToRawF(d))); }

// ---- driving it ----
// signed duty: + = forward. Slow decay (one input held high, the other PWM'd inverted):
// gentler at low speed than fast decay, which suits a small geared pot motor.
void drive(int duty) {
  duty = constrain(duty, -FULL, FULL);
  if (motorReversed) duty = -duty;
  lastDuty = duty;
  if (duty > 0) {
    ledcWrite(PIN_AIN1, FULL);
    ledcWrite(PIN_AIN2, FULL - duty);
  } else if (duty < 0) {
    ledcWrite(PIN_AIN1, FULL + duty);
    ledcWrite(PIN_AIN2, FULL);
  } else {
    ledcWrite(PIN_AIN1, 0);
    ledcWrite(PIN_AIN2, 0);
  }
}

void coast() { drive(0); mode = COAST; }
void brake() { ledcWrite(PIN_AIN1, FULL); ledcWrite(PIN_AIN2, FULL); lastDuty = 0; mode = BRAKE; }

// would this duty push the needle further out of its window?
bool pushingOut(int raw, int duty) {
  int lo = min(POS_LOW, POS_HIGH) - WINDOW_SLACK;
  int hi = max(POS_LOW, POS_HIGH) + WINDOW_SLACK;
  // duty > 0 is assumed to raise counts (that is what 'r' is for)
  return (raw > hi && duty > 0) || (raw < lo && duty < 0);
}

void report(const char *tag) {
  int raw = readRaw();
  bool tracking = (mode == HOLD || mode == FOLLOW);
  Serial.printf("%s raw %4d  mV %4d  deg %6.1f  tgt %6.1f  flt %s  motor %s %d\n", tag, raw,
                analogReadMilliVolts(PIN_WIPER), rawToDeg(pos >= 0 ? pos : raw),
                tracking ? rawToDeg(holdTarget) : NAN,
                digitalRead(PIN_FLT) ? "ok" : "LOW", modeName[mode], lastDuty);
}

// the motor's push towards a target: P with a floor, kept inside the window
int pushTowards(int target, int raw) {
  int err = target - raw;
  // feed-forward while the setpoint is moving (the unit running): push at the speed it is going
  int ff = 0;
  if (!unitOn) ffOn = false;
  else if (fabsf(spVel) > FF_ON) ffOn = true;
  else if (fabsf(spVel) < FF_OFF) ffOn = false;
  if (ffOn) {
    ff = (spVel > 0 ? 1 : -1) * int(FF0 + KV * fabsf(spVel));
    // ahead of the setpoint (err against the way it is going): ease off, and by AHEAD counts
    // ahead stop pushing on, so the needle waits for TL instead of running away from it
    if ((err > 0) != (spVel > 0)) ff = int(ff * max(0.0f, 1.0f - abs(err) / float(AHEAD)));
  }
  if (!ffOn && abs(err) < DEADBAND) return 0;
  int duty = constrain(ff + int(KP * err), -MAX_DUTY, MAX_DUTY);
  if (!ffOn && abs(duty) < MIN_DUTY) duty = duty > 0 ? MIN_DUTY : -MIN_DUTY;
  // behind by LAG_FLOOR while the setpoint creeps: the light feed-forward can stall on a stiff
  // stretch of the track, so push towards the target at least MIN_DUTY (v1.12)
  if (ffOn && abs(err) >= LAG_FLOOR && (duty > 0) == (err > 0) && abs(duty) < MIN_DUTY) duty = err > 0 ? MIN_DUTY : -MIN_DUTY;
  if (pushingOut(raw, duty)) return 0;
  return duty;
}

// ---- the unit ----
void printStep(int mask) {
  if (mask <= 0) return;
  String who;
  for (int i = 0; i < unit::N; i++) if (mask & (1 << i)) { if (who.length()) who += ", "; who += unit::NAMES[i]; }
  Serial.printf("t=%6.1f  pulse  stepped %s  -> pos %d/%d/%d/%d%s\n", model.t, who.c_str(),
                model.q[0] + 1, model.q[1] + 1, model.q[2] + 1, model.q[3] + 1,
                (mask & (1 << ME)) ? "   <- this unit reorganised" : "");
}

void startUnit(int p) {
  presetIdx = constrain(p, 0, unit::NPRESETS - 1);
  const unit::Preset &pr = unit::PRESETS[presetIdx];
  model.start(pr, seedOverride ? seedOverride : pr.seed, signedSel);
  model.x[ME] = constrain(rawToDeg(pos), -unit::STOP, unit::STOP);   // this unit starts where the knob is
  spDeg = model.x[ME]; spVel = 0; modelAcc = 0; lastUnitMs = millis();
  unitOn = true;
  startFollow(degToRaw(spDeg));
  Serial.printf("unit running: preset %d (%s), seed %lu, D %.1f/%.1f/%.1f/%.1f, timer %.1f s, x%.2f real time. %s from the knob at %.1f\n",
                presetIdx, pr.name, (unsigned long)(seedOverride ? seedOverride : pr.seed),
                pr.D[0], pr.D[1], pr.D[2], pr.D[3], model.period, SPEED, unit::NAMES[ME], model.x[ME]);
}

void stopUnit() { unitOn = false; linkOn = false; ringOn = false; model.held[ME] = false; spVel = 0; ffOn = false; }

// link P S G T M: this unit here, the rest on the page (or on other nodes, through the page).
// Same preset, seed, polarity and timer as the page. M 1: this node keeps the clock.
void startLink(String args) {
  int p = 0; unsigned long s = 0; int g = 1; float per = 0; int m = 1;
  sscanf(args.c_str(), "%d %lu %d %f %d", &p, &s, &g, &per, &m);
  clockMaster = m != 0; clockLimit = 0;
  seedOverride = s; signedSel = g != 0;
  bool quiet = stream; stream = false;
  startUnit(p);
  if (per > 0) { model.period = per; model.nextPulse = per; }
  for (int i = 0; i < unit::N; i++) model.remote[i] = i != ME;
  linkOn = true; lastN = millis();
  Serial.printf("linked: %s here, the rest through the page. x%.2f real time, %s\n", unit::NAMES[ME], SPEED,
                clockMaster ? "keeping the clock" : "following the clock");
  (void)quiet;
}

// N t x0 x1 x2 x3: all four needles at the clock's time t. This node's own is ignored. A follower
// steps up to t; the master keeps the clock, so t is only for the record there.
void takeNeighbours(String args) {
  float t, x[4];
  if (sscanf(args.c_str(), "%f %f %f %f %f", &t, &x[0], &x[1], &x[2], &x[3]) == 5 && linkOn) {
    for (int i = 0; i < unit::N; i++) if (i != ME) model.x[i] = x[i];
    if (t > clockLimit) clockLimit = t;
    lastN = millis();
  }
}

// ---- the ring (v1.13) ----
void ringSend(const String &l) {     // never blocks: a full buffer drops the line (the next is 40 ms away)
  if (Serial2.availableForWrite() >= (int)l.length() + 1) Serial2.print(l + "\n");
  else ringDropped++;
}

// a run started here or heard on the ring: the same preset, seed, polarity and timer on every node
void startRing(int p, unsigned long seed, int g, float per, uint32_t epoch, bool announce) {
  seedOverride = seed; signedSel = g != 0;
  bool quiet = stream; stream = false;
  startUnit(p);
  stream = quiet;
  if (per > 0) { model.period = per; model.nextPulse = per; }
  for (int i = 0; i < unit::N; i++) { model.remote[i] = false; heardX[i] = 0; ringT[i] = 0; }
  ringOn = true; ringEpoch = epoch; clockMaster = true; clockLimit = 0;
  if (announce) {
    char b[80];
    snprintf(b, sizeof b, "G %d 0 %lu %d %lu %d %.2f", ME, (unsigned long)epoch, p, seed, g, model.period);
    ringSend(b);
  }
  Serial.printf("ring run %s: preset %d, seed %lu, timer %.1f s, x%.2f real time. %s here; the others on their nodes when heard\n",
                announce ? "started here and sent round" : "heard on the ring", p, seed, model.period, SPEED, unit::NAMES[ME]);
}

void stopRing(bool announce) {
  if (announce && ringOn) { char b[40]; snprintf(b, sizeof b, "S %d 0 %lu", ME, (unsigned long)ringEpoch); ringSend(b); }
  stopUnit(); coast();
  Serial.printf("ring run stopped%s. motor off\n", announce ? " here and sent round" : " (heard on the ring)");
}

// one line off the ring: "<type> <from> <hops> ...". Pass on what is not ours, act on it.
void ringTake(const String &l, uint32_t now) {
  char type = 0; int from = -1, hops = -1, n = 0;
  if (sscanf(l.c_str(), "%c %d %d%n", &type, &from, &hops, &n) < 3 || from < 0 || from >= unit::N || hops < 0) { ringBad++; return; }
  String rest = l.substring(n); rest.trim();
  if (from == ME) {                // ours, back round the ring: the ring is whole. Not passed on.
    if (type == 'P' && (uint32_t)rest.toInt() == pingSeq) { pingRtt = now - pingSentMs; pingBackMs = now; }
    return;
  }
  if (hops + 1 < RING_MAX_HOPS) { ringSend(String(type) + " " + from + " " + (hops + 1) + " " + rest); ringPassed++; }
  heardAny[from] = now;
  if (type == 'V') ringVersion[from] = rest;
  else if (type == 'X') {
    unsigned long seq; float t, x; int q, held;
    if (sscanf(rest.c_str(), "%lu %f %f %d %d", &seq, &t, &x, &q, &held) != 5) { ringBad++; return; }
    if (!ringOn) return;
    heardX[from] = now; ringT[from] = t;
    model.x[from] = x; model.q[from] = constrain(q, 0, unit::POS - 1);
  }
  else if (type == 'G') {
    unsigned long ep, seed; int p, g; float per;
    if (sscanf(rest.c_str(), "%lu %d %lu %d %f", &ep, &p, &seed, &g, &per) != 5) { ringBad++; return; }
    if (linkOn) { Serial.println("ring run heard, but linked to the sim page: ignored"); return; }
    if (!ringOn || ep != ringEpoch) startRing(p, seed, g, per, ep, false);
  }
  else if (type == 'S') {
    if (ringOn && (uint32_t)rest.toInt() == ringEpoch) stopRing(false);
  }
}

// before each pass of the model: who is on their own node, and whose clock to follow
void ringRoles(uint32_t now) {
  int master = ME;
  for (int i = 0; i < unit::N; i++) {
    if (i == ME) continue;
    bool fresh = heardX[i] && now - heardX[i] < RING_STALE_MS;
    if (fresh != model.remote[i])
      Serial.printf(fresh ? "ring: %s heard, its node runs it\n" : "ring: %s not heard, simulating it here\n", unit::NAMES[i]);
    model.remote[i] = fresh;
    if (fresh && i < master) master = i;
  }
  bool was = clockMaster;
  clockMaster = master == ME;
  if (clockMaster != was) Serial.printf("ring: %s\n", clockMaster ? "keeping the clock" : "following the clock");
  if (!clockMaster) clockLimit = ringT[master];
}

void ringStatus() {
  uint32_t now = millis();
  if (pingBackMs && now - pingBackMs < WHOLE_MS) Serial.printf("ring whole: my own ping comes back in %lu ms\n", (unsigned long)pingRtt);
  else Serial.printf("ring open: my own ping %s\n", pingBackMs ? "has stopped coming back" : "has never come back");
  for (int i = 0; i < unit::N; i++) {
    if (i == ME) { Serial.printf("  %s  this node  node_v%s %s%s\n", unit::NAMES[i], VERSION, FW_VERSION, ringOn ? (clockMaster ? "  (keeps the clock)" : "  (follows the clock)") : ""); continue; }
    if (!heardAny[i]) { Serial.printf("  %s  not heard\n", unit::NAMES[i]); continue; }
    Serial.printf("  %s  heard %.1f s ago  %s", unit::NAMES[i], (now - heardAny[i]) / 1000.0f, ringVersion[i].length() ? ringVersion[i].c_str() : "(version not heard yet)");
    if (ringOn) Serial.printf("  %s t %.2f", model.remote[i] ? "on its node" : "simulated here", ringT[i]);
    Serial.println();
  }
  Serial.printf("  %s; lines passed on %lu, bad %lu, dropped %lu\n", ringOn ? "running on the ring" : "not running",
                (unsigned long)ringPassed, (unsigned long)ringBad, (unsigned long)ringDropped);
}

void ringService(uint32_t now) {
  while (Serial2.available()) {
    char ch = Serial2.read();
    if (ch == '\n') { if (ringLine.length()) ringTake(ringLine, now); ringLine = ""; }
    else if (ch != '\r') { ringLine += ch; if (ringLine.length() > 120) { ringLine = ""; ringBad++; } }
  }
  if (now - lastPing >= PING_MS) {
    lastPing = now; pingSeq++; pingSentMs = now;
    ringSend(String("P ") + ME + " 0 " + pingSeq);
  }
  if (lastVer == 0 || now - lastVer >= 5000) { lastVer = now ? now : 1; ringSend(String("V ") + ME + " 0 node_v" + VERSION + " " + FW_VERSION); }
  bool whole = pingBackMs && now - pingBackMs < WHOLE_MS;
  if (whole != ringWhole) {
    ringWhole = whole;
    if (whole) Serial.printf("ring whole: my own ping came back in %lu ms\n", (unsigned long)pingRtt);
    else Serial.println("ring open: my own ping stopped coming back");
  }
  if (ringOn && now - lastRingX >= RING_MS) {
    lastRingX = now;
    char b[80];
    snprintf(b, sizeof b, "X %d 0 %lu %.3f %.2f %d %d", ME, (unsigned long)++ringSeq, model.t, model.x[ME], model.q[ME], mode == HELD ? 1 : 0);
    ringSend(b);
  }
}

void sendJson(uint32_t now) {
  bool fresh = now - lastN < LINK_STALE_MS;
  Serial.printf("{\"u\":%d,\"master\":%s,\"t\":%.3f,\"x\":%.2f,\"v\":%.2f,\"q\":%d,\"held\":%s,\"needle\":%.2f,\"sp\":%.2f,"
                "\"mode\":\"%s\",\"speed\":%.2f,\"period\":%.2f,\"d\":%.2f,\"link\":\"%s\",\"stepped\":%d}\n",
                ME, clockMaster ? "true" : "false", model.t, model.x[ME], model.v[ME], model.q[ME], mode == HELD ? "true" : "false",
                rawToDeg(pos), spDeg, modeName[mode], SPEED, model.period, model.D[ME],
                ringOn ? (ringWhole ? "ring" : "ring open") : (fresh ? "ok" : "waiting"), lastStepMask);
  lastStepMask = 0;
}

// one pass: hand -> TL, run the model the time owed, TL -> the needle's setpoint
void runUnit(uint32_t now) {
  float dt = (now - lastUnitMs) / 1000.0f; lastUnitMs = now;
  if (dt > 0.1f) dt = 0.1f;
  bool hand = (mode == HELD);
  model.held[ME] = hand;
  if (hand) { model.x[ME] = constrain(rawToDeg(pos), -unit::STOP, unit::STOP); model.v[ME] = 0; }
  bool paused = linkOn && now - lastN > LINK_STALE_MS;   // the page went quiet: wait for it
  if (!paused) modelAcc += dt * SPEED;
  if (ringOn) ringRoles(now);
  bool follower = (linkOn || ringOn) && !clockMaster;
  while (modelAcc >= unit::DT && (!follower || model.t + unit::DT <= clockLimit + 1e-6)) {
    int m = model.step();
    if (m > 0 && (m & (1 << ME))) lastStepMask = m;
    if (!linkOn) printStep(m);
    else if (m > 0 && (m & (1 << ME))) Serial.printf("%s reorganised: uniselector to pos %d\n", unit::NAMES[ME], model.q[ME] + 1);
    modelAcc -= unit::DT;
  }
  if (follower && modelAcc > unit::DT) modelAcc = unit::DT;      // waiting for the clock: owe no more than a step

  // TL between model steps: where it is plus how far it has come in the time owed (< one DT)
  double target = constrain(model.x[ME] + model.v[ME] * modelAcc, -unit::STOP, unit::STOP);
  if (hand) { spDeg = model.x[ME]; spVel = 0; return; }   // the hand is the setpoint
  if (dt <= 0) return;
  double maxStep = SP_RATE * dt, before = spDeg;
  spDeg += constrain(target - spDeg, -maxStep, maxStep);
  float v = (degToRawF(spDeg) - degToRawF(before)) / dt;          // counts/s, unrounded
  spVel += (v - spVel) * min(1.0f, dt / SP_TAU);
  if (mode == FOLLOW) holdTarget = degToRaw(spDeg);
}

void checkModel() {               // compare: window.homeostatHeadless(0, 100) in the sim
  static unit::Model m;
  m.start(unit::PRESETS[0], unit::PRESETS[0].seed, true);
  for (int f = 0; f < 100; f++) for (int r = 0; r < 3; r++) m.step();
  Serial.printf("check preset 0, 100 frames: x %.6f %.6f %.6f %.6f  t %.2f\n", m.x[0], m.x[1], m.x[2], m.x[3], m.t);
  Serial.printf("check sched[0][0] %.6f %.6f %.6f %.6f  sched[3][24] %.6f %.6f %.6f %.6f\n",
                m.schedule[0][0][0], m.schedule[0][0][1], m.schedule[0][0][2], m.schedule[0][0][3],
                m.schedule[3][24][0], m.schedule[3][24][1], m.schedule[3][24][2], m.schedule[3][24][3]);
}

void startFollow(int target) {
  // a target outside the needle's +-50 deg is brought to the nearest edge
  int lo = min(POS_LOW, POS_HIGH), hi = max(POS_LOW, POS_HIGH);
  holdTarget = constrain(target, lo, hi);
  mode = FOLLOW;
  winStart = 0; driveSince = 0;
}

void jog(int sign) {
  int before = readRaw();
  if (pushingOut(before, sign)) { Serial.println("jog refused: already past the window that way"); return; }
  mode = JOG;
  drive(sign * jogPercent * FULL / 100);
  uint32_t t = millis();
  while (millis() - t < JOG_MS) {
    if (!digitalRead(PIN_FLT)) break;
    delay(2);
  }
  coast();
  delay(60);
  int after = readRaw();
  Serial.printf("jog %c %d%%: %d -> %d (%+d counts)%s\n", sign > 0 ? '+' : '-', jogPercent,
                before, after, after - before,
                abs(after - before) < 10 ? "  [didn't move: more duty, or check motor wiring/supply]" : "");
}

// how fast does the needle actually go? run at a fixed duty for 1 s, heading for the middle
void speedTest(int pct) {
  int before = readRaw();
  int mid = POS_ZERO;
  int sign = before > mid ? -1 : +1;
  mode = JOG;
  drive(sign * pct * FULL / 100);
  uint32_t t = millis();
  while (millis() - t < 1000) {
    int raw = readRaw();
    if (!digitalRead(PIN_FLT) || pushingOut(raw, sign)) break;
    delay(2);
  }
  uint32_t ms = millis() - t;
  coast();
  delay(80);
  int after = readRaw();
  float dps = (rawToDeg(after) - rawToDeg(before)) * 1000.0f / ms;
  Serial.printf("speed %d%%: %.1f -> %.1f deg in %lu ms = %.1f deg/s\n", pct,
                rawToDeg(before), rawToDeg(after), ms, dps);
}

void handleCommand(String c) {
  c.trim();
  if (c == "V") { Serial.printf("node_v%s %s  unit %s\n", VERSION, FW_VERSION, unit::NAMES[ME]);
                  Serial.printf("{\"version\":\"node_v%s %s\",\"u\":%d}\n", VERSION, FW_VERSION, ME); }
  else if (c == "?") report("status");
  else if (c == "q") { stream = !stream; Serial.printf("stream %s\n", stream ? "on" : "off"); }
  else if (c == "+") jog(+1);
  else if (c == "-") jog(-1);
  else if (c.startsWith("j ")) { jogPercent = constrain(c.substring(2).toInt(), 0, 100); Serial.printf("jog duty %d%%\n", jogPercent); }
  else if (c.startsWith("h ")) {
    float d = constrain(c.substring(2).toFloat(), -50.0f, 50.0f);
    holdTarget = degToRaw(d); mode = HOLD; modeUntil = millis() + HOLD_MS;
    Serial.printf("hold %.1f deg (raw %d) for %lu ms\n", d, holdTarget, HOLD_MS);
  }
  else if (c.startsWith("N ")) takeNeighbours(c.substring(2));
  else if (c.startsWith("link ")) startLink(c.substring(5));
  else if (c == "ring?") ringStatus();
  else if (c == "ringstop") stopRing(true);
  else if (c == "ring" || c.startsWith("ring ")) {
    int p = presetIdx; unsigned long sd = 0; int g = 1; float per = 0;
    if (c.length() > 5) sscanf(c.c_str() + 5, "%d %lu %d %f", &p, &sd, &g, &per);
    p = constrain(p, 0, unit::NPRESETS - 1);
    if (!sd) sd = unit::PRESETS[p].seed;
    linkOn = false;
    startRing(p, sd, g, per, esp_random() | 1, true);
  }
  else if (c.startsWith("d0 ")) { model.D[ME] = c.substring(3).toFloat(); }
  else if (c.startsWith("id ")) {
    int n = c.substring(3).toInt();
    if (n >= 0 && n < unit::N) { stopUnit(); ME = n; prefs.putInt("id", ME); Serial.printf("this node is unit %d, %s (saved)\n", ME, unit::NAMES[ME]);
                                 Serial.printf("{\"version\":\"node_v%s %s\",\"u\":%d}\n", VERSION, FW_VERSION, ME); }
    else Serial.println("id 0-3: 0 TL, 1 TR, 2 BR, 3 BL");
  }
  else if (c == "cal") Serial.printf("unit %s  lo %d  zero %d  hi %d  reversed %d  min %d  ff0 %d  kv %.2f\n", unit::NAMES[ME], POS_LOW, POS_ZERO, POS_HIGH, motorReversed, MIN_DUTY, FF0, KV);
  else if (c == "run") startUnit(presetIdx);
  else if (c.startsWith("run ")) startUnit(c.substring(4).toInt());
  else if (c == "stop") { stopUnit(); coast(); Serial.println("unit stopped, motor off"); }
  else if (c.startsWith("speed ")) { SPEED = c.substring(6).toFloat(); Serial.printf("SPEED x%.2f\n", SPEED); }
  else if (c.startsWith("period ")) { model.period = c.substring(7).toFloat(); Serial.printf("timer %.1f s\n", model.period); }
  else if (c.startsWith("seed ")) { seedOverride = c.substring(5).toInt(); Serial.printf("seed %lu for the next run\n", (unsigned long)seedOverride); }
  else if (c.startsWith("rate ")) { SP_RATE = c.substring(5).toFloat(); Serial.printf("SP_RATE %.1f deg/s\n", SP_RATE); }
  else if (c.startsWith("ff0 ")) { FF0 = c.substring(4).toInt(); saveCal(); Serial.printf("FF0 %d (saved)\n", FF0); }
  else if (c.startsWith("lag ")) { LAG_FLOOR = max(1, (int)c.substring(4).toInt()); Serial.printf("LAG_FLOOR %d\n", LAG_FLOOR); }
  else if (c.startsWith("ahead ")) { AHEAD = max(1, (int)c.substring(6).toInt()); Serial.printf("AHEAD %d\n", AHEAD); }
  else if (c.startsWith("tau ")) { SP_TAU = max(0.005f, c.substring(4).toFloat()); Serial.printf("SP_TAU %.3f s\n", SP_TAU); }
  else if (c.startsWith("kv ")) { KV = c.substring(3).toFloat(); saveCal(); Serial.printf("KV %.2f (saved)\n", KV); }
  else if (c == "check") checkModel();
  else if (c == "f") { stopUnit(); startFollow(pos); Serial.printf("follow %.1f deg (here)\n", rawToDeg(holdTarget)); }
  else if (c.startsWith("f ")) {
    float d = constrain(c.substring(2).toFloat(), -50.0f, 50.0f);
    stopUnit();
    startFollow(degToRaw(d));
    Serial.printf("follow %.1f deg (raw %d)\n", d, holdTarget);
  }
  else if (c.startsWith("he ")) { HAND_ERR = c.substring(3).toInt(); Serial.printf("HAND_ERR %d\n", HAND_ERR); }
  else if (c.startsWith("hg ")) { HAND_GAIN = c.substring(3).toInt(); Serial.printf("HAND_GAIN %d\n", HAND_GAIN); }
  else if (c.startsWith("hm ")) { HAND_MS = c.substring(3).toInt(); Serial.printf("HAND_MS %lu\n", HAND_MS); }
  else if (c.startsWith("still ")) { STILL_MS = c.substring(6).toInt(); Serial.printf("STILL_MS %lu\n", STILL_MS); }
  else if (c.startsWith("u ")) speedTest(constrain(c.substring(2).toInt(), 0, 100));
  else if (c.startsWith("db ")) { DEADBAND = max(1, (int)c.substring(3).toInt()); Serial.printf("DEADBAND %d\n", DEADBAND); }
  else if (c.startsWith("kp ")) { KP = c.substring(3).toFloat(); Serial.printf("KP %.2f\n", KP); }
  else if (c.startsWith("min ")) { MIN_DUTY = constrain(c.substring(4).toInt(), 0, FULL); saveCal(); Serial.printf("MIN_DUTY %d (saved)\n", MIN_DUTY); }
  else if (c.startsWith("max ")) { MAX_DUTY = constrain(c.substring(4).toInt(), 0, FULL); Serial.printf("MAX_DUTY %d\n", MAX_DUTY); }
  else if (c == "s") { coast(); Serial.println("coast"); }
  else if (c == "b") { brake(); Serial.println("brake"); }
  else if (c == "r") { motorReversed = !motorReversed; coast(); saveCal(); Serial.printf("motorReversed %d (saved)\n", motorReversed); }
  else if (c == "lo" || c.startsWith("lo ")) { POS_LOW = c.length() > 3 ? c.substring(3).toInt() : readRaw(); saveCal(); Serial.printf("POS_LOW = %d (-50 deg, saved)\n", POS_LOW); }
  else if (c == "zero" || c.startsWith("zero ")) { POS_ZERO = c.length() > 5 ? c.substring(5).toInt() : readRaw(); saveCal(); Serial.printf("POS_ZERO = %d (0 deg, saved)\n", POS_ZERO); }
  else if (c == "hi" || c.startsWith("hi ")) { POS_HIGH = c.length() > 3 ? c.substring(3).toInt() : readRaw(); saveCal(); Serial.printf("POS_HIGH = %d (+50 deg, saved)\n", POS_HIGH); }
  else if (c == "sleep") { coast(); digitalWrite(PIN_SLP, LOW); Serial.println("driver asleep"); }
  else if (c == "wake") { digitalWrite(PIN_SLP, HIGH); Serial.println("driver awake"); }
  else if (c.length()) Serial.printf("? unknown '%s'\n", c.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  prefs.begin("node", false);       // a new board has none: node 1's numbers until it is calibrated
  ME = constrain(prefs.getInt("id", 0), 0, unit::N - 1);
  POS_LOW = prefs.getInt("lo", POS_LOW); POS_ZERO = prefs.getInt("zero", POS_ZERO); POS_HIGH = prefs.getInt("hi", POS_HIGH);
  motorReversed = prefs.getBool("rev", motorReversed);
  MIN_DUTY = prefs.getInt("min", MIN_DUTY);
  FF0 = prefs.getInt("ff0", FF0); KV = prefs.getFloat("kv", KV);   // a stiffer pot needs a firmer floor (node 2: 650)
  Serial.printf("node_v%s %s  unit %s\n", VERSION, FW_VERSION, unit::NAMES[ME]);   // first line: which code, which unit
  Serial.printf("{\"version\":\"node_v%s %s\",\"u\":%d}\n", VERSION, FW_VERSION, ME);   // the same, for the editor's bridge
  Serial.printf("calibration: lo %d  zero %d  hi %d  reversed %d  min %d  ff0 %d  kv %.2f%s\n", POS_LOW, POS_ZERO, POS_HIGH, motorReversed, MIN_DUTY, FF0, KV,
                prefs.isKey("zero") ? "" : "  (defaults: not calibrated on this node yet)");

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_WIPER, ADC_11db);          // 0..~3.1 V
  pinMode(PIN_FLT, INPUT);
  pinMode(PIN_SLP, OUTPUT);
  digitalWrite(PIN_SLP, HIGH);
  Serial2.setRxBufferSize(1024);
  Serial2.setTxBufferSize(1024);
  Serial2.begin(RING_BAUD, SERIAL_8N1, PIN_RING_RX, PIN_RING_TX);   // the wired ring (v1.13)
  ledcAttach(PIN_AIN1, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_AIN2, PWM_FREQ, PWM_BITS);
  coast();

  Serial.println("motor off. 'f 0' to follow 0 deg; push the knob and the hand takes over. '?' for status, 'ring?' for the ring");
  report("boot");
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (line.length()) handleCommand(line); line = ""; }
    else line += ch;
  }

  if (!digitalRead(PIN_FLT) && mode != FAULT && mode != COAST && mode != HELD) {
    coast(); mode = FAULT;
    Serial.println("FAULT: DRV8833 FLT low (over-current or over-temperature). motor off");
  }

  bool good = readNeedle();
  uint32_t now = millis();
  ringService(now);

  // a lost wiper: never drive on a reading this wiring can't make
  if (!good && (mode == HOLD || mode == FOLLOW) && now - lostSince > LOST_MS) {
    coast(); mode = FAULT;
    Serial.println("FAULT: wiper reads out of range (lost contact on pot pin 4 / GPIO34?). motor off");
  }

  if (mode == HOLD && good) {
    int err = holdTarget - pos;
    if (now > modeUntil) { coast(); Serial.printf("hold done: target %d, at %d (%+d)\n", holdTarget, pos, -err); }
    else drive(pushTowards(holdTarget, pos));
  }

  if (unitOn && good && mode != FAULT) runUnit(now);

  if (mode == FOLLOW && good) {
    int err = abs(holdTarget - pos);
    int duty = pushTowards(holdTarget, pos);
    drive(duty);
    // the hand: off by more than HAND_ERR, driving the whole window, and not winning back
    if (duty == 0) driveSince = 0;
    else if (driveSince == 0) driveSince = now;
    // only a push TOWARDS the target can be resisted by a hand: a needle that ran on past its
    // setpoint is the motor's own overshoot, not a hand (v1.10)
    bool towards = (duty > 0) == (holdTarget > pos);
    if (duty == 0 || !towards || err <= HAND_ERR || now - driveSince < HAND_MS) winStart = 0;
    else if (winStart == 0) { winStart = now; winErr = err; }
    else if (now - winStart >= HAND_MS) {
      if (winErr - err < HAND_GAIN) {
        coast(); mode = HELD;
        stillRef = pos; stillSince = now;
        Serial.printf("hand on at %.1f deg (%.1f off, won back %d counts in %lu ms). motor slack\n",
                      rawToDeg(pos), rawToDeg(pos) - rawToDeg(holdTarget), winErr - err, HAND_MS);
      } else { winStart = now; winErr = err; }
    }
  }

  if (mode == HELD && good) {
    if (abs(pos - stillRef) > STILL_BAND) { stillRef = pos; stillSince = now; }
    else if (now - stillSince >= STILL_MS) {
      startFollow(pos);
      Serial.printf("hand off at %.1f deg: holding there\n", rawToDeg(holdTarget));
    }
  }

  if ((linkOn || ringOn) && now - lastJson >= 50) { lastJson = now; sendJson(now); }
  if (stream && millis() - lastReport >= 100) {
    lastReport = millis();
    if (unitOn)
      Serial.printf("unit t %6.1f  TL %+6.1f TR %+6.1f BR %+6.1f BL %+6.1f  pos %d/%d/%d/%d  needle %+6.1f sp %+6.1f  %s %d\n",
                    model.t, model.x[0], model.x[1], model.x[2], model.x[3],
                    model.q[0] + 1, model.q[1] + 1, model.q[2] + 1, model.q[3] + 1,
                    rawToDeg(pos), spDeg, modeName[mode], lastDuty);
    else report("pot");
  }
  delay(2);
}

/*
 * FLASH:  powershell -File node\flash.ps1 -Sketch bench\node_v1_14
 *   board "ESP32 Dev Module" (esp32:esp32:esp32), DevKitC shows as a CP210x COM port.
 *   If upload stalls at "Connecting...", hold BOOT until it starts writing.
 */
