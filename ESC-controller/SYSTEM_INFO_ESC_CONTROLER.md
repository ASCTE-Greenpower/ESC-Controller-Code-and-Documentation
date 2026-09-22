# ESC Controller — System Reference

## What This Project Is

An Arduino sketch running on a plain **ESP32 WROOM-32 dev board** that acts as a throttle/ESC controller with dynamic PWM ramping, three selectable drive modes, and a live OLED display. It sits between a potentiometer/trigger input and a PWM-controlled ESC, shaping the throttle curve so acceleration is smooth and mode-appropriate rather than instantaneous.

Originally built on an Arduino Nano ESP32 (ESP32-S3); switched to a plain ESP32 WROOM-32 as of V23, which meant a full pin remap (the two chips don't share a pin layout, and WROOM-32's flash chip occupies GPIO6–11 internally — several of the Nano ESP32's pin choices weren't even safe to reuse) and a different upload procedure.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Microcontroller | ESP32 WROOM-32 dev board (plain ESP32, 3.3V) |
| OLED | 128×64 SSD1306 via I2C |
| OLED wiring | SDA → GPIO21, SCL → GPIO22 |
| PWM output | LEDC, pin-based API (core 3.x+), ~31 kHz (silent), 8-bit (0–255) |
| ADC | 12-bit (0–4095) — same as the previous Nano ESP32 |
| Libraries | Adafruit SSD1306, Adafruit GFX, Wire, Arduino |

### Pin Assignments

| Pin | Role |
|-----|------|
| GPIO21 | OLED SDA |
| GPIO22 | OLED SCL |
| GPIO13 | Trigger / button (active-low) |
| GPIO27 | ECO mode switch (active-low) |
| GPIO14 | SPORT mode switch (active-low) |
| GPIO25 | PWM output to ESC |
| GPIO34 | Throttle potentiometer (ADC1 — deliberately not ADC2, which WiFi disables) |
| GPIO17 | Serial1 TX → display_receiver RX |
| GPIO16 | Serial1 RX → display_receiver TX |
| GPIO4, 15, 18, 19, 23, 26, 32, 33, 35–39 | Free / unassigned (35/36/39 are input-only) |
| GPIO0, 2, 5, 12, 15 | Boot-strapping — avoided for any input/output in this sketch |
| GPIO6–11 | **Never usable** — internally wired to the module's flash chip |
| GPIO1, 3 | UART0 (USB serial / programming) — avoided for anything else |

> **Upload method:** Standard Arduino IDE upload — WROOM-32's USB-to-serial bridge chip (CP2102/CH340) handles reset automatically. No DFU double-tap procedure (that was Nano ESP32-specific, native-USB behavior that doesn't exist on this board). Board setting: **"ESP32 Dev Module"**. If a upload ever fails to start, hold the BOOT button during the "Connecting..." phase as a fallback.

---

## File List

| File | Description | Editable? |
|------|-------------|-----------|
| `throttle_controller.ino` | Main and only sketch file | Yes — primary edit target |
| `SYSTEM_INFO.md` | This document — human-readable reference | Update after significant changes |
| `CLAUDE.md` | AI assistant briefing | Update after significant changes |

---

## How the Code Works

### Modes

Selected via active-low switches. ECO wins if both switches are pulled low simultaneously.

| Mode | Ramp Time | Decay Time | Curve |
|------|-----------|------------|-------|
| ECO | ~30 s | 10 s | Exponential: `exp(1.8p) / 2.047` |
| NORMAL | ~15 s | 7.5 s | Shifted sigmoid: `sig(p)*1.5 + 0.15`, center=0.15 |
| SPORT | ~5 s | 7.5 s | Square root: `min(0.5/√p, 5.0)` |

### State Machine

Four states: `IDLE → REENGAGING → RAMPING → HOLDING`

```
IDLE        Trigger released. rampPwm and currentPwm decay at mode rate.
            Output hard-zeroed by trigger safety guard.

REENGAGING  Trigger re-pressed while currentPwm > 2% (RESUME_THRESHOLD).
            rampPwm climbs from 0 to resumeTarget using sport curve at
            REENGAGE_CLIMB_RATE (~3 s). Hands off to RAMPING on arrival.

RAMPING     Climbs toward potPct using the active mode's curve.
            Progress = (rampPwm - rampStartPwm) / 100.0 — always relative
            to segment start, preventing fast-blast on high-value resume.

HOLDING     rampPwm has reached potPct. Tracks pot directly.
            Pot drop: follows immediately.
            Pot rise: re-enters RAMPING with rampStartPwm = rampPwm.
```

### Pot Smoothing

EMA filter: `smoothedPot = POT_ALPHA * raw + (1 - POT_ALPHA) * smoothedPot`  
`POT_ALPHA = 0.2` — filters ±0.5% ADC jitter. Higher = more responsive, lower = smoother.

### OLED Layout (refreshes every 100 ms)

```
NORMAL                RAMP    ← mode left, state right-aligned (size 1)
━━━━━━━━━━━━━━━━━━━━━━━━━━━
        75.0%                 ← output %, size 3, centred, y=13
━━━━━━━━━━━━━━━━━━━━━━━━━━━
Pot:  80.0%                   ← y=42 (size 1)
Ramp: 62.3%                   ← y=54 (size 1)
```

### Serial Telemetry (USB debug)

Emitted every 20 ms (control loop tick):
```
NORMAL | RAMP | pot:80.0% ramp:62.3% resume:0.0% out:62.3%
```

### UART Telemetry (Serial1 → display_receiver)

Emitted every 50 ms (20 Hz) on GPIO17(TX)/GPIO16(RX) at 115200 baud. Sends only the fields this device knows — speedMph, batV, rpm, and amps come from separate sensors and are merged downstream.

```
mode,state,setpointPct,livePct,rampPct
```

Example:
```
NORMAL,RAMP,80.0,62.3,62.3
```
Wiring: GPIO17(TX) → receiver RX, GPIO16(RX) → receiver TX, GND → GND.

---

## Key Constants / Parameters

| Constant | Value | Meaning |
|----------|-------|---------|
| `ECO_CLIMB_RATE` | 0.00333 | %/ms base climb rate → ~30 s full ramp |
| `NORMAL_CLIMB_RATE` | 0.00667 | %/ms → ~15 s full ramp |
| `SPORT_CLIMB_RATE` | 0.02000 | %/ms → ~5 s full ramp |
| `REENGAGE_CLIMB_RATE` | 0.03333 | %/ms → ~3 s re-engage climb |
| `ECO_DECAY_RATE` | 0.01000 | %/ms → 10 s full decay |
| `NORMAL_DECAY_RATE` | 0.01333 | %/ms → 7.5 s full decay |
| `SPORT_DECAY_RATE` | 0.01333 | %/ms → 7.5 s full decay |
| `RESUME_THRESHOLD` | 2.0 % | Minimum currentPwm to trigger re-engage path |
| `POT_ALPHA` | 0.2 | EMA smoothing coefficient for pot ADC |
| `LEDC_FREQ_HZ` | 31372 | PWM frequency — above human hearing |
| `LEDC_RES_BITS` | 8 | PWM resolution (0–255) |
| `OLED_ADDR` | 0x3C | I2C address of SSD1306 display |
| `UART_TX_PIN` | 17 | Serial1 TX pin (→ display_receiver RX) |
| `UART_RX_PIN` | 16 | Serial1 RX pin (→ display_receiver TX) |
| `UART_MS` | 50 ms | UART send interval (20 Hz) |
| Control loop | 20 ms | `delay(20)` at bottom of loop() |
| Display refresh | 100 ms | Independent of control loop |
| UART send | 50 ms | Independent timer, 115200 baud |

---

## Version History

| Version | Notes |
|---------|-------|
| V1–V15 | Development history (pre-repo) |
| V16 | Re-engage timing fixed; OLED layout finalized; rampStartPwm added to fix fast-blast-on-resume bug |
| V17 | In this folder at repo creation |
| V18 | Added Serial1 UART telemetry on D10/D11 — 20 Hz CSV to display_receiver |
| V19 | Removed speedMph/batV/rpm/amps from UART packet — supplied by separate sensors downstream |
| V20 | Fixed UART GPIO bug — Serial1.begin() needs GPIO numbers (21/38), not Arduino pin labels (10/11) |
| V21–V22 | Undocumented — the `.ino` had already reached V22 (with `UART_TX_PIN`/`UART_RX_PIN` at 12/11) by the time this history table was last updated. Whatever changed here wasn't recorded; noting the gap rather than guessing at it. |
| V23 | **Hardware changed: Arduino Nano ESP32 → plain ESP32 WROOM-32.** Full pin remap (previous pins fell inside WROOM-32's internal flash range or on strapping pins), upload procedure changed (no more DFU double-tap — standard IDE upload via the board's USB-serial bridge), board setting changed to "ESP32 Dev Module". Throttle pot deliberately placed on an ADC1 pin (GPIO34) so it stays readable if a WiFi/ESP-NOW link is ever added to this board. |
| V24 | **Fixed a compile break caused by an ESP32 Arduino core upgrade (core 3.x / ESP-IDF 5), not a code bug that was always there.** That core version removed the old channel-based LEDC API (`ledcSetup()` + `ledcAttachPin(pin, channel)`, `ledcWrite(channel, duty)`) in favor of a pin-based one (`ledcAttach(pin, freq, resolutionBits)`, `ledcWrite(pin, duty)`) — the concept of a separately-numbered LEDC channel is gone from the sketch's point of view; the core manages that internally now. `LEDC_CHANNEL` constant removed; every call site now uses `PWM_PIN` directly. No pin/behavior change — this only affects which underlying Arduino-ESP32 core version the sketch compiles against. |
