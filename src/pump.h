#ifndef PUMP_H
#define PUMP_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// A single snapshot of the pump controller's state. Produced once per second
// by the feedback sampler, read by the dashboard / MQTT / LED layers.
typedef struct {
    bool      valid;            // true once we have at least one feedback sample
    bool      commanded_on;     // what we are driving the output to (desired)
    bool      feedback_on;      // measured: is the sense line actually energised
    bool      mismatch;         // commanded != feedback past the settle window
                                // → relay stuck, blown driver, or broken wire
    float     feedback_v;       // 1-second average voltage on the sense line
    int       feedback_mv;      // same, in mV
    int       raw_mv;           // most recent single ADC sample (mV), diagnostics
    int       peak_mv;          // peak instantaneous mV in the last window
    bool      saturated;        // peak_mv >= PUMP_FEEDBACK_SATURATION_MV
    int       threshold_mv;     // active on/off decision threshold
    // Water pressure transducer.
    bool      pressure_valid;   // false: no successful ADC sample this window —
                                // psi/voltage below are 0 and must not be trusted
    float     pressure_v;       // 1-second average voltage at the transducer
    int       pressure_mv;      // same, in mV (raw at the ADC pin, post-divider)
    float     pressure_psi;     // derived pressure (clamped >= 0)
    // True when the firmware's failsafe (broker-loss dead-man / runtime cap)
    // turned the pump off. Cleared by the next real commanded change, so the
    // cloud can tell a failsafe trip from a normal off.
    bool      failsafe_off;
    int       output_gpio;      // control output pin
    int       feedback_gpio;    // sense input pin
    int       pressure_gpio;    // pressure transducer pin
    uint32_t  on_seconds;       // cumulative seconds the pump has been ON since boot
    uint32_t  sample_count;     // total ADC samples taken since boot
    int64_t   last_sample_us;
    int64_t   last_change_us;   // when commanded state last changed
} pump_reading_t;

esp_err_t pump_init(void);
void pump_get(pump_reading_t *out);

// Command the pump. Drives the output and resets the settle timer. The
// commanded state is deliberately NOT persisted — the pump always boots OFF, so
// a power loss / reboot / OTA can never silently re-energise it.
esp_err_t pump_set(bool on);
esp_err_t pump_toggle(void);
bool      pump_is_on(void);

// True if the controller currently believes the relay/feedback don't agree
// (i.e. r.mismatch). Cheap helper for the LED fault indicator.
bool      pump_has_fault(void);

// Mark the current OFF state as caused by a safety failsafe (call right after
// pump_set(false)). Surfaces as `failsafe_off` in the reading until the next
// real commanded change.
void      pump_note_failsafe_off(void);

// Feedback decision threshold (mV) — persisted in NVS.
int       pump_get_threshold_mv(void);
esp_err_t pump_set_threshold_mv(int mv);

// Parse and apply an MQTT command payload. Recognised forms:
//   {"pump":"on"} / {"pump":"off"} / {"pump":true} / {"pump":false}
//   {"toggle":true}
//   {"threshold_mv":1600}
// Safe to call from the MQTT event task.
void      pump_handle_command(const char *json, int len);

#endif // PUMP_H
