/*
 * TelemetryLog.h  --  EEPROM flight telemetry logger
 *
 * Target: Seeed Studio XIAO RA4M1 (Renesas RA4M1) using the stock Arduino
 *         EEPROM library.  Works unchanged on AVR / SAMD boards too, it just
 *         gets less room.
 *
 * Columns: t_ms, roll, pitch, pid_roll, pid_pitch, alt_m, aux
 *
 * IMPORTANT (RA4M1): this chip has no real EEPROM.  The core emulates it in
 * 8 KB of on-chip data flash.  A byte write is a read-modify-write of a
 * 64-byte flash block, so it is FAR slower than an AVR EEPROM write and it
 * costs an erase cycle on the whole block.  Everything below is arranged to
 * keep writes contiguous, block-aligned, and as rare as possible:
 *
 *   - records are packed to 16 bytes so exactly 4 fit in one 64-byte block
 *   - records are staged in RAM and flushed one full block at a time
 *   - no counter / index is updated per record (that would thrash block 0);
 *     the record count is recovered by scanning at boot
 *   - every byte write is read-compare-write, so unchanged bytes cost nothing
 *
 * Storage layout
 *   [0 .. 63]        header (magic, version, record size) -- one flash block
 *   [64 .. end]      record array, filled front to back
 *
 * A record is "empty" when its timestamp field reads 0xFFFFFFFF.  clear()
 * just invalidates timestamps, which is what makes it reasonably quick.
 *
 * Capacity on the XIAO RA4M1: (8192 - 64) / 16 = 508 records.
 *   @ 25 Hz -> ~20 s   @ 20 Hz -> ~25 s   @ 15 Hz -> ~34 s   @ 10 Hz -> ~51 s
 *
 * ---------------------------------------------------------------------------
 * Usage
 *
 *   #include "TelemetryLog.h"
 *   TelemetryLog tlog;
 *
 *   void setup() {
 *     Serial.begin(115200);         // do NOT add while(!Serial) -- USB CDC
 *     tlog.begin();                 // does NOT erase; old flight survives reset
 *   }
 *
 *   void loop() {
 *     tlog.poll();                  // serial commands: h d s e f  (ground only)
 *     ...
 *     if (armed)  tlog.log(roll, pitch, pidRoll, pidPitch, altitude);
 *     if (landed) tlog.flush();     // push the partial RAM block to flash
 *   }
 *
 * Serial commands (single char, case-insensitive):
 *   h  help          s  status        d  dump CSV
 *   f  flush         e  erase (asks for y/n confirmation)
 *
 * Version history
 *   1 - t_ms, roll, pitch, pid_roll, pid_pitch, aux0, aux1
 *   2 - aux0 became a dedicated altitude field with its own scale.  begin()
 *       reformats a v1 log rather than misreading its columns, so DUMP ANY
 *       v1 FLIGHT DATA BEFORE FLASHING THIS VERSION.
 * ---------------------------------------------------------------------------
 */

#pragma once

#include <Arduino.h>
#include <EEPROM.h>

/* ------------------------------ configuration ---------------------------- */

/* Fixed-point scale for the attitude / PID columns.  Stored value =
 * round(engineering_value * TLOG_SCALE) in an int16, so the representable
 * range is +/- 32767/TLOG_SCALE.
 *   100.0f -> +/-327.67 with 0.01 resolution   (good for degrees)
 *  1000.0f -> +/- 32.767 with 0.001 resolution (good for normalized PID out)
 * Define this before including the header if 100 does not suit your PID units. */
#ifndef TLOG_SCALE
#define TLOG_SCALE 100.0f
#endif

/* Separate scale for altitude, because metres need range where degrees need
 * resolution.
 *    10.0f -> +/-3276.7 m at 0.1 m   (default; below baro noise anyway)
 *   100.0f -> +/- 327.67 m at 0.01 m (low hops only -- it SATURATES above 327 m)
 *     1.0f -> +/-32767 m at 1 m      (high-altitude flights)
 * Altitude is normally AGL: zero the barometer on the pad before arming. */
#ifndef TLOG_ALT_SCALE
#define TLOG_ALT_SCALE 10.0f
#endif

/* Records staged in RAM before a flash write.  4 = exactly one 64-byte flash
 * block = the sweet spot.  Set to 1 to write through immediately (slower loop,
 * more flash wear, but nothing is lost on a brownout). */
#ifndef TLOG_BUFFER_RECORDS
#define TLOG_BUFFER_RECORDS 508
#endif

/* Byte offset of the log region inside EEPROM.  Raise it if the flight code
 * already stores config (trims, gains) at the bottom of EEPROM. */
#ifndef TLOG_BASE_ADDR
#define TLOG_BASE_ADDR 0
#endif

/* Reserved header size.  64 keeps records block-aligned -- keep it a multiple
 * of 64 on the RA4M1. */
#ifndef TLOG_HEADER_BYTES
#define TLOG_HEADER_BYTES 64
#endif

#define TLOG_MAGIC    0x54564331UL   /* 'TVC1' */
#define TLOG_VERSION  2
#define TLOG_EMPTY_T  0xFFFFFFFFUL

/* ------------------------------- record ---------------------------------- */

struct TlogRecord {
  uint32_t t_ms;      /* millis() at capture, 0xFFFFFFFF == empty slot */
  int16_t  roll;      /* these four are engineering units * TLOG_SCALE */
  int16_t  pitch;
  int16_t  pidRoll;
  int16_t  pidPitch;
  int16_t  alt;       /* altitude * TLOG_ALT_SCALE, normally AGL metres */
  int16_t  aux;       /* spare: servo command, yaw, flight state... */
};

static_assert(sizeof(TlogRecord) == 16, "TlogRecord must stay 16 bytes");

/* ------------------------------- logger ---------------------------------- */

class TelemetryLog {
public:
  TelemetryLog()
      : _capacity(0), _next(0), _bufCount(0), _dropped(0),
        _lastWriteUs(0), _pendingErase(false) {}

  /* Prepare the log.  Formats the header if this is a virgin/foreign/older
   * EEPROM, then scans to find the first free slot so an existing flight is
   * preserved across a reset.  Returns false only if EEPROM is too small. */
  bool begin() {
    uint16_t size = (uint16_t)EEPROM.length();
    if (size <= TLOG_BASE_ADDR + TLOG_HEADER_BYTES + sizeof(TlogRecord))
      return false;

    _capacity = (uint16_t)((size - TLOG_BASE_ADDR - TLOG_HEADER_BYTES) /
                           sizeof(TlogRecord));

    uint32_t magic = 0;
    uint8_t  ver = 0, recSize = 0;
    readRaw(TLOG_BASE_ADDR + 0, &magic, 4);
    readRaw(TLOG_BASE_ADDR + 4, &ver, 1);
    readRaw(TLOG_BASE_ADDR + 5, &recSize, 1);

    if (magic != TLOG_MAGIC || ver != TLOG_VERSION ||
        recSize != sizeof(TlogRecord)) {
      format();
    }

    _next     = scanNext();
    _bufCount = 0;
    _dropped  = 0;
    return true;
  }

  /* ---- writing ---- */

  /* Log one row, timestamped with millis().  Returns false if the log is full
   * (the flight code can ignore the return value; it simply stops recording). */
  bool log(float roll, float pitch, float pidRoll, float pidPitch,
           float altitude) {
    return log(millis(), roll, pitch, pidRoll, pidPitch, altitude, 0.0f);
  }

  /* Full form: explicit timestamp and the spare column. */
  bool log(uint32_t t_ms, float roll, float pitch, float pidRoll,
           float pidPitch, float altitude, float aux = 0.0f) {
    if (_next + _bufCount >= _capacity)   { _dropped++; return false; }
    if (_bufCount >= TLOG_BUFFER_RECORDS) { _dropped++; return false; }

    if (t_ms == TLOG_EMPTY_T) t_ms = TLOG_EMPTY_T - 1;  /* never look empty */

    TlogRecord &r = _buf[_bufCount];
    r.t_ms     = t_ms;
    r.roll     = quantize(roll,     TLOG_SCALE);
    r.pitch    = quantize(pitch,    TLOG_SCALE);
    r.pidRoll  = quantize(pidRoll,  TLOG_SCALE);
    r.pidPitch = quantize(pidPitch, TLOG_SCALE);
    r.alt      = quantize(altitude, TLOG_ALT_SCALE);
    r.aux      = quantize(aux,      TLOG_SCALE);
    _bufCount++;
    return true;
  }

  /* Push staged records to flash.  Call after touchdown / on disarm, otherwise
   * up to TLOG_BUFFER_RECORDS-1 rows are lost on power-down.  Cheap no-op when
   * the RAM buffer is empty, so it is safe to call every loop once landed. */
  bool flush() {
    if (_bufCount == 0) return true;

    uint32_t t0 = micros();
    uint16_t  n  = _bufCount;
    for (uint16_t i = 0; i < n; i++) {
      writeRaw(recordAddr(_next + i), &_buf[i], sizeof(TlogRecord));
    }
    _lastWriteUs = micros() - t0;

    _next    += n;
    _bufCount = 0;
    return true;
  }

  bool commit() { return flush(); }

  /* ---- erasing ---- */

  /* Clear all data: invalidates every record's timestamp so the log reads as
   * empty and refills from slot 0.  Payload bytes are left as-is (they get
   * overwritten by the next flight) -- this is ~4x fewer flash writes than a
   * full wipe.  Takes a while; do it on the pad, not in flight. */
  void clear(Stream *progress = nullptr) {
    const uint32_t empty = TLOG_EMPTY_T;
    for (uint16_t i = 0; i < _capacity; i++) {
      writeRaw(recordAddr(i), &empty, sizeof(empty));
      if (progress && (i % 64) == 0) progress->print('.');
    }
    if (progress) progress->println();
    _next     = 0;
    _bufCount = 0;
    _dropped  = 0;
  }

  /* Paranoid version: writes 0xFF over every byte of the record area.  Much
   * slower than clear(); only useful if you want no residue from old flights. */
  void wipe(Stream *progress = nullptr) {
    for (uint16_t i = 0; i < _capacity; i++) {
      uint16_t a = recordAddr(i);
      for (uint8_t b = 0; b < sizeof(TlogRecord); b++) updateByte(a + b, 0xFF);
      if (progress && (i % 64) == 0) progress->print('.');
    }
    if (progress) progress->println();
    _next     = 0;
    _bufCount = 0;
    _dropped  = 0;
  }

  /* ---- reading ---- */

  /* Fetch record i (0 .. count()-1).  Reads through the RAM buffer so records
   * not yet flushed are still visible. */
  bool get(uint16_t i, TlogRecord &out) const {
    if (i >= count()) return false;
    if (i >= _next) { out = _buf[i - _next]; return true; }
    readRaw(recordAddr(i), &out, sizeof(TlogRecord));
    return true;
  }

  /* Decoded convenience accessors, in engineering units. */
  static float rollOf(const TlogRecord &r)     { return r.roll     / TLOG_SCALE; }
  static float pitchOf(const TlogRecord &r)    { return r.pitch    / TLOG_SCALE; }
  static float pidRollOf(const TlogRecord &r)  { return r.pidRoll  / TLOG_SCALE; }
  static float pidPitchOf(const TlogRecord &r) { return r.pidPitch / TLOG_SCALE; }
  static float altOf(const TlogRecord &r)      { return r.alt / TLOG_ALT_SCALE; }
  static float auxOf(const TlogRecord &r)      { return r.aux      / TLOG_SCALE; }

  /* Highest altitude in the log, in metres.  Handy for a quick apogee readout
   * without pulling the whole CSV. */
  float peakAltitude() const {
    TlogRecord r;
    float best = 0.0f;
    uint16_t n = count();
    for (uint16_t i = 0; i < n; i++) {
      if (!get(i, r)) break;
      float a = altOf(r);
      if (i == 0 || a > best) best = a;
    }
    return best;
  }

  /* Dump the whole log as CSV.  Flushes first so nothing is missing.
   * Blocking -- run it on the ground, never with the control loop live. */
  void dumpCSV(Stream &out = Serial) {
    flush();
    out.println(F("#BEGIN"));
    out.println(F("t_ms,roll,pitch,pid_roll,pid_pitch,alt_m,aux"));

    TlogRecord r;
    uint16_t n = count();
    for (uint16_t i = 0; i < n; i++) {
      if (!get(i, r)) break;
      out.print(r.t_ms);                       out.print(',');
      out.print(rollOf(r),     2);             out.print(',');
      out.print(pitchOf(r),    2);             out.print(',');
      out.print(pidRollOf(r),  2);             out.print(',');
      out.print(pidPitchOf(r), 2);             out.print(',');
      out.print(altOf(r),      2);             out.print(',');
      out.println(auxOf(r),    2);
    }
    out.println(F("#END"));
  }

  void printStatus(Stream &out = Serial) {
    out.print(F("records "));     out.print(count());
    out.print('/');               out.print(_capacity);
    out.print(F("  free "));      out.print(_capacity - count());
    out.print(F("  dropped "));   out.print(_dropped);
    out.print(F("  staged "));    out.print(_bufCount);
    out.print(F("  peakAlt "));   out.print(peakAltitude(), 1);
    out.print(F(" m  lastWrite ")); out.print(_lastWriteUs);
    out.println(F(" us"));
  }

  /* ---- serial command handling ---- */

  /* Call from loop().  Consumes at most one character per call, so it is safe
   * to leave in the flight loop -- though 'd' and 'e' will block once issued. */
  void poll(Stream &io = Serial) {
    if (!io.available()) return;
    int c = io.read();
    if (c < 0 || c == '\r' || c == '\n' || c == ' ') return;

    if (_pendingErase) {
      _pendingErase = false;
      if (c == 'y' || c == 'Y') {
        io.println(F("erasing"));
        clear(&io);
        io.println(F("erased"));
      } else {
        io.println(F("erase cancelled"));
      }
      return;
    }

    switch (c) {
      case 'h': case 'H': case '?':
        io.println(F("h help | s status | d dump csv | f flush | e erase"));
        break;
      case 's': case 'S': printStatus(io); break;
      case 'd': case 'D': dumpCSV(io);     break;
      case 'f': case 'F': flush(); io.println(F("flushed")); break;
      case 'e': case 'E':
        _pendingErase = true;
        io.println(F("erase all telemetry? y/n"));
        break;
      default:
        io.print(F("? unknown cmd '")); io.print((char)c);
        io.println(F("' (h for help)"));
        break;
    }
  }

  /* ---- accessors ---- */

  uint16_t count()     const { return _next + _bufCount; }
  uint16_t capacity()  const { return _capacity; }
  uint16_t remaining() const { return _capacity - count(); }
  bool     full()      const { return count() >= _capacity; }
  uint16_t dropped()   const { return _dropped; }
  /* Duration of the last flash flush, in microseconds -- measure this on your
   * board before choosing a log rate. */
  uint32_t lastWriteMicros() const { return _lastWriteUs; }

private:
  uint16_t   _capacity;
  uint16_t   _next;                    /* next free slot in flash */
  uint16_t   _bufCount;                /* records staged in RAM */
  uint16_t   _dropped;
  uint32_t   _lastWriteUs;
  bool       _pendingErase;
  TlogRecord _buf[TLOG_BUFFER_RECORDS];

  static uint16_t recordAddr(uint16_t i) {
    return (uint16_t)(TLOG_BASE_ADDR + TLOG_HEADER_BYTES + i * sizeof(TlogRecord));
  }

  static int16_t quantize(float v, float scale) {
    if (!(v == v)) return 0;            /* NaN-safe without <math.h> macros */
    float s = v * scale;
    if (s >  32767.0f) return  32767;   /* saturate rather than wrap */
    if (s < -32768.0f) return -32768;
    return (int16_t)lroundf(s);
  }

  /* Read-compare-write: skips the flash block erase when the byte already
   * holds the target value. */
  static void updateByte(uint16_t addr, uint8_t val) {
    if (EEPROM.read(addr) != val) EEPROM.write(addr, val);
  }

  static void writeRaw(uint16_t addr, const void *src, uint8_t len) {
    const uint8_t *p = (const uint8_t *)src;
    for (uint8_t i = 0; i < len; i++) updateByte(addr + i, p[i]);
  }

  static void readRaw(uint16_t addr, void *dst, uint8_t len) {
    uint8_t *p = (uint8_t *)dst;
    for (uint8_t i = 0; i < len; i++) p[i] = EEPROM.read(addr + i);
  }

  void format() {
    const uint32_t magic   = TLOG_MAGIC;
    const uint8_t  ver     = TLOG_VERSION;
    const uint8_t  recSize = sizeof(TlogRecord);
    writeRaw(TLOG_BASE_ADDR + 0, &magic, 4);
    writeRaw(TLOG_BASE_ADDR + 4, &ver, 1);
    writeRaw(TLOG_BASE_ADDR + 5, &recSize, 1);
    clear();
  }

  /* Records fill front to back, so the first empty timestamp marks the end. */
  uint16_t scanNext() const {
    for (uint16_t i = 0; i < _capacity; i++) {
      uint32_t t;
      readRaw(recordAddr(i), &t, sizeof(t));
      if (t == TLOG_EMPTY_T) return i;
    }
    return _capacity;
  }
};
