# solar-control-pool-switch

WiFi-connected **pool pump switch** on an **ESP32-C3 SuperMini** (Teyleten
Robot or equivalent). It drives one output to turn the pump on/off and reads a
feedback line through a 10 K resistor to confirm the output is actually doing
what it was told — surfacing a **mismatch** fault if the relay is stuck, the
driver is blown, or the sense wire is broken. It serves a live dashboard,
publishes state over MQTT, and accepts remote on/off commands.

Companion to [solar-control-anemometer](../solar-control-anemomenter) and the
parent `solar-inverter-esp32`. Same WiFi/MQTT/OTA/dashboard scaffold, with the
wind sensor swapped for relay control + feedback sensing.

## Hardware

| Part                                  | Notes                                          |
|---------------------------------------|------------------------------------------------|
| Teyleten Robot ESP32-C3 SuperMini     | Native USB-C, 4 MB flash, onboard LED on GPIO 8|
| Relay / SSR module (3.3 V logic)      | Switches the pump's mains contactor             |
| 10 K resistor                         | Series resistor on the feedback sense tap       |
| Pressure transducer (0.5–4.5 V)       | Ratiometric, e.g. 0–80 psi water pressure       |
| 10K / 20K / 1K resistors              | Voltage divider scaling the transducer to ADC   |

### Wiring

```
   GPIO 10 (output) ──────────────> relay/SSR IN   (3.3 V = pump enabled)
        |
        +--[ 10K ]--+---> GPIO 3   (ADC1_CH3, feedback sense)
                    |
                  (GND)            (optional pull-down to firm up the "off" level)

   transducer ──[ 10K ]──┬──[ 1K ]──> GPIO 0   (ADC1_CH0, water pressure)
                         |
                      [ 20K ]
                         |
                       (GND)         (ADC sees 2/3 of the transducer voltage)
```

- **Output — GPIO 10.** Driven **HIGH (3.3 V)** to enable the pump, **LOW (0 V)**
  to disable it. If your relay board is active-low, flip
  `PUMP_OUTPUT_ACTIVE_LOW` in [src/config.h](src/config.h). GPIO 10 is a plain,
  non-strapping GPIO — we avoid GPIO 21 here because it's UART0 TX, which the
  ROM bootloader drives HIGH during the boot/reset window, briefly energising an
  active-high relay before the firmware reclaims the pin.
- **Feedback — GPIO 3 (ADC1_CH3).** A sense line tapped off the output through
  a 10 K resistor. It is read on the ADC so the dashboard shows the real
  voltage, then thresholded (default 1600 mV) to a yes/no "is the line actually
  energised". Comparing that against the commanded state is what flags a fault.
  ADC1 only — ADC2 conflicts with WiFi on the C3. GPIO 3 is non-strapping;
  avoid GPIO 2 here (it's a strapping pin that must read HIGH at reset).
- **Pressure — GPIO 0 (ADC1_CH0).** A ratiometric water-pressure transducer:
  0.5 V at 0 psi rising to 4.5 V at full scale (default 80 psi), read through a
  **10K / 20K** divider (+ 1K series filter) that scales it to 2/3 — so full
  scale is ~3.0 V, safely under the C3's 3.3 V ADC limit. The firmware undoes
  the divider (`× (R_top + R_bottom) / R_bottom`) to recover the transducer
  voltage, then converts to psi (`psi = (V − 0.5) / 4.0 × 80`, clamped at 0).
  Set the divider resistor values and transfer function in
  [src/config.h](src/config.h). Only ADC1 (GPIO 0–4) works with WiFi on; GPIO 1
  is a free non-strapping channel (feedback is on GPIO 3).

Pin assignments and tunables live in [src/config.h](src/config.h).

> The driven 3.3 V line read straight through 10 K will peg the C3 ADC (it
> saturates near ~3.1 V). That's expected — it reads as a clear "on". The
> firmware exposes a `saturated` flag for diagnostics.

## Build / Flash

Requires [PlatformIO](https://platformio.org/) with the espressif32 platform.

```powershell
pio run                              # build
pio run -t upload                    # flash via USB
pio run -t monitor                   # serial monitor (115200)
```

On first boot with no saved WiFi the device opens an open hotspot named
**`PoolPump-XXXXXX`** (last 3 MAC bytes). Join it and browse to
`http://192.168.4.1/` or `http://poolpump.local/` to configure WiFi.

After it joins your network, the dashboard is at `http://<device-ip>/` or
`http://poolpump-XXXXXX.local/`.

## Dashboard

- **Pump**: big ON/OFF state, a one-tap toggle (with confirm), commanded vs
  measured-feedback cells, a mismatch warning banner, live sense voltage, and
  cumulative runtime.
- **Calibration**: the feedback on/off threshold (mV).
- **WiFi**: status, scan, connect, forget.
- **MQTT**: broker config (saved to NVS, optional).
- **System**: firmware version, heap, restart, LED behavior.
- **Update**: upload a `.bin` over the air; writes the inactive OTA slot and
  reboots into it. The pump boots OFF after the update.

Live updates push over `/ws` once per second; falls back to polling
`/api/status` if the WebSocket drops.

## Boot state & daily reboot

**The pump always boots OFF.** The commanded state is *never* persisted, so a
power loss, reboot, or OTA update can't silently re-energise the pump — it stays
off until an explicit command turns it on. This is the safe default for an
unattended mains pump.

A background task reboots the device once per UTC day (at midnight if SNTP
synced, else on a 24 h uptime fallback, hard-capped at 25 h). Note the
consequence of the always-OFF rule: **the daily reboot turns the pump off.** If
the pump is driven on a schedule, that schedule lives in whatever commands the
pump (the cloud / your automation), which simply re-issues the on command — the
device itself never auto-resumes.

## MQTT

Topic prefix is `pumps/<mac>/` (MAC with dashes, e.g. `pumps/a1-b2-c3-d4-e5-f6/`).

| Topic                     | Dir | Payload                                                    |
|---------------------------|-----|------------------------------------------------------------|
| `pumps/<mac>/status`      | out | `{"online":true}` / `{"online":false}` (retained, LWT)     |
| `pumps/<mac>/state`       | out | pump state (retained, QoS 1) — see below                   |
| `pumps/<mac>/info`        | out | `{model, firmware, output_gpio, feedback_gpio, threshold_mv, ui_url, ui_host}` |
| `pumps/<mac>/cmd`         | in  | commands (not retained) — see below                        |

**State** is published report-by-exception: the moment any of `commanded_on`,
`feedback_on`, or `mismatch` changes, plus a `REPORT_HEARTBEAT_SECONDS` (30 s)
heartbeat. Payload:

```json
{"commanded_on":true,"feedback_on":true,"mismatch":false,
 "feedback_v":3.105,"feedback_mv":3105,"saturated":true,
 "threshold_mv":1600,"pressure_psi":18.4,"pressure_v":1.420,"on_seconds":7200}
```

State is also flushed when the pressure moves by `REPORT_PRESSURE_DELTA_PSI`
(2 psi) since the last publish, so meaningful pressure changes reach the cloud
promptly without streaming every 1 Hz reading.

**Commands** on `pumps/<mac>/cmd`:

```jsonc
{"pump": "on"}          // or {"pump": true}  / {"pump": 1}
{"pump": "off"}         // or {"pump": false} / {"pump": 0}
{"toggle": true}        // flip the current state
{"threshold_mv": 1600}  // change the feedback on/off threshold (100–3300)
```

Commands should be published **non-retained** so the pump stays OFF after a
reboot. (A *retained* command would be redelivered on every reconnect and would
re-energise the pump right after boot — exactly the auto-resume behaviour the
always-OFF design avoids. The SolarCloud server publishes these non-retained.)

As a safety net the **firmware itself defends against this**: any command that
arrives with the MQTT retain flag set (i.e. a stored command the broker replays
on connect) is ignored *and deleted* from the broker. Only live, non-retained
commands ever switch the pump — so a stale retained `{"pump":"on"}` left on the
broker can never turn the pump on at boot.

## Fault detection

After each commanded change the firmware waits `PUMP_SETTLE_MS` (2 s, covers
relay pull-in) before judging the feedback. If commanded ≠ measured past that
window, `mismatch` goes true: the dashboard shows a red banner, MQTT publishes
the change immediately, and the onboard LED triple-blinks (in the default
"errors only" LED mode).

## Project layout

```
platformio.ini          # board = esp32-c3-devkitm-1, framework = espidf
partitions.csv          # 4 MB dual-OTA layout (ota_0/ota_1 + otadata)
sdkconfig.defaults      # USB-Serial-JTAG console, WS support
version.py              # SemVer derived from conventional commits
CMakeLists.txt          # project root
src/
  config.h              # pin map, feedback threshold, reporting tunables
  main.c                # boot sequence + 1 Hz WS broadcaster + RBE MQTT
  pump.{c,h}            # relay output + ADC feedback sampler, mismatch logic,
                        # NVS state persistence, MQTT command parser
  wifi_manager.{c,h}    # STA + soft-AP fallback, mDNS, SNTP
  wifi_config.{c,h}     # NVS-persisted credentials
  nvs_store.{c,h}       # thin NVS wrapper
  led_status.{c,h}      # onboard LED state machine (pump-fault aware)
  http_server.{c,h}     # /api/* + /ws
  mqtt_bridge.{c,h}     # publishes state/info, subscribes to <mac>/cmd
  index.html            # embedded dashboard
```

## License

Same as the parent project.
