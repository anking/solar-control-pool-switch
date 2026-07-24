#ifndef PUMP_H
#define PUMP_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Why the safety lockout tripped. Reported (as a string) in MQTT state so the
// cloud can word its alert email; persisted in NVS so the lockout survives a
// reboot (a tripped safety stays tripped until a manual reset).
typedef enum {
    PUMP_SAFETY_OK = 0,
    PUMP_SAFETY_LOW_PRESSURE  = 1,   // didn't reach min psi after starting
    PUMP_SAFETY_HIGH_PRESSURE = 2,   // critical psi held past the sustain window
    PUMP_SAFETY_PRESSURE_LOSS = 3,   // primed, then fell back below min psi
                                     // (ruptured filter casing / burst pipe)
} pump_safety_reason_t;

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
    // Pressure safety. safety_lockout latches when a pressure failsafe trips —
    // the pump refuses to start until reset. safety_reason says which rule.
    // pressure_warning is a non-latching "pressure has been high too long" flag.
    bool      safety_lockout;
    int       safety_reason;       // pump_safety_reason_t
    bool      pressure_warning;
    // Active pressure thresholds (psi) — the cloud is the source of truth and
    // pushes these down; reported back here for confirmation.
    float     min_psi;
    float     max_psi;
    float     critical_psi;
    // Active prime grace window (s) and mid-run pressure-loss rule switch —
    // cloud-pushed alongside the thresholds, reported back for confirmation.
    int       prime_grace_s;
    bool      pressure_loss_enabled;
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

// ---- Pressure safety --------------------------------------------------------
// Pressure failsafe thresholds (psi) — persisted in NVS; the cloud pushes them
// down. set returns ESP_ERR_INVALID_ARG unless 0 <= min < max < critical <= 100.
void      pump_get_pressure_limits(float *min_psi, float *max_psi, float *critical_psi);
esp_err_t pump_set_pressure_limits(float min_psi, float max_psi, float critical_psi);

// Prime grace window (seconds a starting pump has to reach min psi) — persisted
// in NVS; the cloud pushes it down. set returns ESP_ERR_INVALID_ARG outside
// PUMP_PRIME_GRACE_S_MIN..MAX.
int       pump_get_prime_grace_s(void);
esp_err_t pump_set_prime_grace_s(int seconds);

// Mid-run pressure-loss rule (primed, then back below min psi → lockout) —
// enable/disable, persisted in NVS; the cloud pushes it down.
bool      pump_get_pressure_loss_enabled(void);
esp_err_t pump_set_pressure_loss_enabled(bool enabled);

// Latch / clear the safety lockout. Tripping does NOT switch the pump itself —
// the caller turns it off first; this latches the lockout + reason (persisted)
// so any later "on" (manual, MQTT, schedule) is refused until reset.
void      pump_trip_lockout(pump_safety_reason_t reason);
void      pump_reset_lockout(void);
bool      pump_is_locked_out(void);

// Set/clear the non-latching high-pressure warning flag (surfaced in state so
// the cloud can email). Not persisted.
void      pump_set_pressure_warning(bool on);

// Stable wire string for a safety reason ("none" / "low_pressure" /
// "high_pressure" / "pressure_loss"), used in the MQTT/WS state JSON.
const char *pump_safety_reason_str(int reason);

// Parse and apply an MQTT command payload. Recognised forms:
//   {"pump":"on"} / {"pump":"off"} / {"pump":true} / {"pump":false}
//   {"toggle":true}
//   {"threshold_mv":1600}
//   {"min_psi":5,"max_psi":25,"critical_psi":30}   (all three required)
//   {"prime_grace_s":15}
//   {"pressure_loss_en":true}
//   {"reset_failsafe":true}
// Safe to call from the MQTT event task.
void      pump_handle_command(const char *json, int len);

#endif // PUMP_H
