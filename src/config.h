#ifndef CONFIG_H
#define CONFIG_H

#include "hal/adc_types.h"

// =============================================================================
// HARDWARE: Teyleten Robot ESP32-C3 SuperMini
// =============================================================================
// GPIO 8 has the onboard LED, active-LOW on most SuperMini boards.

// Onboard LED
#define LED_GPIO              8
#define LED_ACTIVE_LOW        1

// =============================================================================
// POOL PUMP CONTROL
// =============================================================================
// The firmware drives one output to switch the pump on/off, and reads one
// input to confirm the output is actually doing what it was told.
//
//   OUTPUT  (GPIO 10): the control line. Driven HIGH (3.3 V) to enable the
//                     pump, LOW (0 V) to disable it. Set PUMP_OUTPUT_ACTIVE_LOW
//                     to 1 if your relay/SSR module is active-low instead.
//                     (GPIO 10 is a plain, non-strapping GPIO. We avoid GPIO 21
//                     here: it's UART0 TX, which the ROM bootloader drives HIGH
//                     during boot — energising an active-high relay for a brief
//                     pulse before the firmware reclaims the pin.)
//
//   FEEDBACK (GPIO 3 / ADC1_CH3): a sense line tapped off the output through a
//                     10K resistor. We read it on the ADC so the dashboard can
//                     show the real voltage, then threshold it to a yes/no
//                     "is the line actually energised" — which lets us flag a
//                     mismatch (relay stuck, blown driver, broken wire).
//                     GPIO 3 is non-strapping; avoid GPIO 2 here (it's a
//                     strapping pin that must read HIGH at reset).
//
// Why ADC and not a plain digital read: the 10K + ADC gives us the actual
// voltage for diagnostics, and a tunable threshold is more robust against a
// partially-pulled line than a fixed logic level. We use ADC1 only — ADC2
// conflicts with WiFi on the C3.
#define PUMP_OUTPUT_GPIO          10
#define PUMP_OUTPUT_ACTIVE_LOW    0     // 0: drive HIGH to enable (3.3 V = on)

#define PUMP_FEEDBACK_ADC_UNIT    ADC_UNIT_1
#define PUMP_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_3
#define PUMP_FEEDBACK_GPIO        3

// Feedback decision: above this many mV on the sense line, we consider the
// output "energised" (pump actually on). Field-tunable from the dashboard.
#define PUMP_FEEDBACK_ON_THRESHOLD_MV   1600

// Hysteresis band (mV): once ON it only flips back OFF below
// (threshold - hysteresis). Stops feedback_on from chattering when the sense
// voltage hovers right at the threshold, which would otherwise emit a retained
// MQTT state publish every second.
#define PUMP_FEEDBACK_HYSTERESIS_MV     200

// The C3 ADC pegs near ~3.1 V with ADC_ATTEN_DB_12. A driven 3.3 V line read
// straight through 10K will saturate — expected and fine (clearly "on"); we
// just surface the flag for diagnostics.
#define PUMP_FEEDBACK_SATURATION_MV     3000

// Feedback sampling: oversample the ADC at this rate, average to a 1 Hz value.
#define PUMP_FEEDBACK_OVERSAMPLE_HZ     100
#define PUMP_SAMPLE_INTERVAL_MS         1000

// Grace window after a commanded change before a commanded/feedback mismatch
// is treated as a real fault. Covers relay pull-in time and ADC settling.
#define PUMP_SETTLE_MS                  2000

// =============================================================================
// WATER PRESSURE (analog transducer via voltage divider)
// =============================================================================
//   PRESSURE (GPIO 0 / ADC1_CH0): a ratiometric pressure transducer (0.5 V at
//                     0 psi rising to 4.5 V at full scale) read through a
//                     resistor divider that scales it under the 3.3 V ADC limit.
//
//   Only ADC1 (GPIO 0-4) works while WiFi is on — ADC2/GPIO 5 conflicts with
//   WiFi, and GPIO 6-10/20/21 aren't ADC at all. GPIO 2 is a strapping pin.
//   That leaves GPIO 0/1/4 as the usable analog inputs; feedback uses GPIO 3.
//   GPIO 0 is non-strapping and free (no RTC crystal on the SuperMini).
//
//   Divider: transducer ── R_top ──┬── R_series(1k) ── GPIO 0
//                                   └── R_bottom ── GND
//   With 10k top / 20k bottom the ADC sees 2/3 of the transducer voltage
//   (4.5 V -> 3.0 V, safely under the cap). The 1k series resistor forms a
//   small RC filter / pin protection and does not change the ratio (the ADC
//   input is high-impedance). The firmware multiplies the measured voltage by
//   (R_top + R_bottom) / R_bottom to recover the true transducer voltage.
#define PRESSURE_ADC_UNIT       ADC_UNIT_1
#define PRESSURE_ADC_CHANNEL    ADC_CHANNEL_0
#define PRESSURE_GPIO           0

// Divider resistor values — set these to match your wiring.
#define PRESSURE_DIV_R_TOP_OHMS     10000   // transducer -> ADC tap
#define PRESSURE_DIV_R_BOTTOM_OHMS  20000   // ADC tap -> GND
#define PRESSURE_DIVIDER_MULT \
    (((float)PRESSURE_DIV_R_TOP_OHMS + (float)PRESSURE_DIV_R_BOTTOM_OHMS) / (float)PRESSURE_DIV_R_BOTTOM_OHMS)

// Transducer transfer function (on the RECOVERED, pre-divider voltage):
// psi = (V - V_MIN) / (V_MAX - V_MIN) * PSI_MAX, clamped at 0.
#define PRESSURE_V_MIN_MV       500     // transducer output at 0 psi (mV)
#define PRESSURE_V_MAX_MV       4500    // transducer output at full scale (mV)
#define PRESSURE_PSI_MAX        80.0f   // psi at full scale

// MQTT report-by-exception: also flush when pressure moves at least this much
// (psi) since the last publish, so the cloud sees meaningful changes promptly
// without streaming every 1 Hz reading.
#define REPORT_PRESSURE_DELTA_PSI   2.0f

// Safety: the pump ALWAYS boots OFF. The commanded state is never persisted,
// so a power loss, reboot, or OTA can never silently re-energise the pump — it
// stays off until an explicit command turns it on.

// =============================================================================
// REPORTING
// =============================================================================
// The dashboard gets a 1 Hz live push over the WebSocket. MQTT is report-by-
// exception: we publish the moment the pump state changes (commanded, feedback,
// or fault), plus a periodic heartbeat so the backend knows we're alive.
#define REPORT_HEARTBEAT_SECONDS   30

// =============================================================================
// FIRMWARE VERSION (injected by version.py at build time)
// =============================================================================

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "0.0.0"
#endif

#ifndef GIT_HASH
#define GIT_HASH "unknown"
#endif

#endif // CONFIG_H
