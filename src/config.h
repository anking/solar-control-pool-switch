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
