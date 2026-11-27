// unit.h: PT's minimal Homeostat model (essential_Homeostat_EN_physically_realistic), ported line
// for line from the browser sim (homeostat-arm/arm/homeostat.html), which ported it from PT's Python.
// Four units of magnet and trough, coupled all to all, 25-position uniselectors, a shared timer.
// This node is TL (unit 0) and carries the physical needle; TR, BR, BL are simulated here until
// real nodes join on the ring.
//
// The uniselector schedules come from Python's random (MT19937, random.seed(int)), reproduced bit
// for bit, so a seed gives the same machine here as in the sim and in PT's code. Checked with the
// 'check' command against window.homeostatHeadless() in the sim.
#pragma once
#include <math.h>
#include <stdint.h>

// ---------------- Python's random, as the sim's PyRandom ----------------
struct PyRandom {
  uint32_t mt[624];
  int mti = 625;

  void initGenrand(uint32_t s) {
    mt[0] = s;
    for (int i = 1; i < 624; i++) mt[i] = 1812433253u * (mt[i - 1] ^ (mt[i - 1] >> 30)) + (uint32_t)i;
    mti = 624;
  }
  void seed(uint32_t n) {                        // random.seed(int) for 0 <= n < 2^32: key = [n]
    initGenrand(19650218u);
    int i = 1, j = 0;
    for (int k = 624; k; k--) {
      mt[i] = (mt[i] ^ ((mt[i - 1] ^ (mt[i - 1] >> 30)) * 1664525u)) + n + (uint32_t)j;
      i++; j++;
      if (i >= 624) { mt[0] = mt[623]; i = 1; }
      if (j >= 1) j = 0;
    }
    for (int k = 623; k; k--) {
      mt[i] = (mt[i] ^ ((mt[i - 1] ^ (mt[i - 1] >> 30)) * 1566083941u)) - (uint32_t)i;
      i++;
      if (i >= 624) { mt[0] = mt[623]; i = 1; }
    }
    mt[0] = 0x80000000u;
  }
  uint32_t u32() {
    uint32_t y;
    if (mti >= 624) {
      for (int kk = 0; kk < 624; kk++) {
        y = (mt[kk] & 0x80000000u) | (mt[(kk + 1) % 624] & 0x7fffffffu);
        mt[kk] = mt[(kk + 397) % 624] ^ (y >> 1) ^ ((y & 1) ? 0x9908b0dfu : 0u);
      }
      mti = 0;
    }
    y = mt[mti++];
    y ^= y >> 11; y ^= (y << 7) & 0x9d2c5680u; y ^= (y << 15) & 0xefc60000u; y ^= y >> 18;
    return y;
  }
  double random() { uint32_t a = u32() >> 5, b = u32() >> 6; return (a * 67108864.0 + b) / 9007199254740992.0; }
  uint32_t getrandbits(int k) { return u32() >> (32 - k); }
  uint32_t randbelow(uint32_t n) {
    int k = 0; for (uint32_t m = n; m; m >>= 1) k++;      // n.bit_length()
    uint32_t r = getrandbits(k); while (r >= n) r = getrandbits(k); return r;
  }
  double uniform(double a, double b) { return a + (b - a) * random(); }
};

// ---------------- the model ----------------
namespace unit {
const int N = 4, POS = 25;
const double LIMIT = 45.0, STOP = 50.0, DT = 0.01;
const double GAIN = 4.0, DAMP = 4.0, SAT = 80.0, I_MAX = 80.0;
const char *NAMES[N] = {"TL", "TR", "BR", "BL"};

struct Preset { const char *name; uint32_t seed; double D[N]; double x0[N]; double period; };
const Preset PRESETS[] = {      // found by PT's search_spiral.py for this model (as in the sim)
  {"435017 10 s (default)",          435017, {-0.8, -0.8, -0.5, -0.3}, {-5, 10, -17, 49},  2.0},
  {"104283 12 s edges, deep spiral", 104283, { 0.1,  0.0, -0.9, -0.5}, {-43, -6, 28, -17}, 1.5},
  {"489235 18 s, 97% periphery",     489235, {-0.3, -0.5,  0.4, -0.7}, {-47, -47, 31, -35}, 2.5},
  {"186963 22 s, most conical",      186963, {-0.2, -0.9,  0.1, -0.5}, {28, 28, 3, 33},    2.0},
  {"476128 9 s, 8.3 turns",          476128, {-0.7, -0.8,  0.0, -0.3}, {-49, 46, 22, 24},  3.0},
  {"634613 9 s, deepest",            634613, {-0.3, -0.9, -0.9, -0.7}, {-45, 10, -8, 22},  3.0},
};
const int NPRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

struct Model {
  double schedule[N][POS][N];
  double D[N], x[N], v[N];
  int q[N];
  bool held[N];
  double t, nextPulse, period;
  long pulses, steps;

  void makeSchedule(uint32_t seed, bool sgn) {
    PyRandom rng; rng.seed(seed);
    for (int i = 0; i < N; i++)
      for (int p = 0; p < POS; p++)
        for (int j = 0; j < N; j++) {
          if (j == i) { schedule[i][p][j] = 0; continue; }
          double sign = rng.randbelow(2) ? 1.0 : -1.0;      // choice([-1, 1])
          double mag = rng.uniform(0.1, 1.0);              // same draw order as PT's code
          schedule[i][p][j] = (sgn ? sign : 1.0) * mag;
        }
  }
  void start(const Preset &p, uint32_t seed, bool sgn) {
    makeSchedule(seed, sgn);
    for (int i = 0; i < N; i++) { D[i] = p.D[i]; x[i] = p.x0[i]; v[i] = 0; q[i] = 0; held[i] = false; }
    t = 0; period = p.period; nextPulse = period; pulses = 0; steps = 0;
  }
  static double out(double th) { return I_MAX * tanh(th / SAT); }
  void accel(const double *th, const double *om, double *a) {
    double g[N];
    for (int j = 0; j < N; j++) g[j] = out(th[j]);
    for (int i = 0; i < N; i++) {
      const double *row = schedule[i][q[i]];
      double s = 0; for (int j = 0; j < N; j++) s += row[j] * g[j];
      a[i] = GAIN * cos(th[i] * M_PI / 180.0) * (s + D[i] * g[i]) - DAMP * om[i];
    }
  }
  // one DT of PT's loop. Returns a bitmask of the units whose uniselector stepped (0 if none,
  // -1 if a pulse came with all units within limits).
  int step() {
    double a1[N], a2[N], a3[N], a4[N], x1[N], v1[N], x2[N], v2[N], x3[N], v3[N];
    accel(x, v, a1);
    for (int i = 0; i < N; i++) { x1[i] = x[i] + 0.5 * DT * v[i];  v1[i] = v[i] + 0.5 * DT * a1[i]; }
    accel(x1, v1, a2);
    for (int i = 0; i < N; i++) { x2[i] = x[i] + 0.5 * DT * v1[i]; v2[i] = v[i] + 0.5 * DT * a2[i]; }
    accel(x2, v2, a3);
    for (int i = 0; i < N; i++) { x3[i] = x[i] + DT * v2[i];       v3[i] = v[i] + DT * a3[i]; }
    accel(x3, v3, a4);
    for (int i = 0; i < N; i++) {
      if (held[i]) { v[i] = 0; continue; }
      x[i] += DT / 6 * (v[i] + 2 * v1[i] + 2 * v2[i] + v3[i]);
      v[i] += DT / 6 * (a1[i] + 2 * a2[i] + 2 * a3[i] + a4[i]);
      if (fabs(x[i]) > STOP) { x[i] = x[i] > 0 ? STOP : -STOP; v[i] = 0; }
    }
    t += DT;
    if (t >= nextPulse - 1e-9) {                  // timer pulse: relay F closed -> step
      nextPulse = t + period; pulses++;
      int mask = 0;
      for (int i = 0; i < N; i++)
        if (fabs(out(x[i])) > out(LIMIT)) { q[i] = (q[i] + 1) % POS; steps++; mask |= 1 << i; }
      return mask ? mask : -1;
    }
    return 0;
  }
};
}  // namespace unit
