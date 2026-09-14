# aqua-bridge

**Fan controller** on a Raspberry Pi for an air-cooled **DAS enclosure**
(direct-attached storage): up to 15 hot-swap drives in zones, cooled by
an air stream from 8–10 fans. The goal is the **least modelled fan noise
with every drive within its temperature limit**. Fans are driven and
read through Aqua Computer **aquaero 6 XT** + **Quadro**, 24–30
temperature sensors sit next to the drives and in the air stream
(aquaero/Quadro thermistor inputs and DS18B20 on the Pi's 1-Wire buses),
SMART temperatures from the PC the DAS is attached to are optional,
**Digole** (touch) is the display, telemetry goes to **Home Assistant**.

Develop on a desktop or laptop. The Pi is runtime and hardware.

**Project language is English only** — docs, comments, commit messages,
issues, and PR descriptions.

Hosts, usernames, and LAN details belong in `private.md` (gitignored), not
in this file.

**Two config modes, one code base.** Earlier revisions of this document
described a watercooling loop (a coolant setpoint, radiator and intake
fans). That was never the target; the plant is the DAS described in §1
and §2. The safety core written for that draft is plant-agnostic and
stayed: sensor gate, hold-then-high fallback, rate limit and clamp,
supervisor, loop, hwmon adapter, HTTP/MQTT, deploy. The code now runs in
one of two modes, chosen by the config alone:

- **DAS mode** — `mpc.topology` is present (`config.example-das.yaml`):
  zones, bays, sensor roles, drive classes and fans are declared; trust
  and fallback run per zone; drive temperatures are estimated; the
  solvers minimise fan noise under per-drive limits. This is the target
  and what §1–§13 mostly describe.
- **Legacy mode** — no `topology` (`config.example.yaml`): one implicit
  zone holding every channel and temperature, one setpoint tracked by PI
  or a small linear MPC, a two-node coolant/air RC simulator. It behaves
  bit for bit as before the DAS work and is kept as the reference for
  the safety core: its goldens (`tests/golden/*.pi.json`,
  `*.mpc.json`) never move. The placeholder names `coolant`, `air`,
  `radiator`, `intake` live only there, in the legacy tests and in the
  legacy HTTP/MQTT examples.

Where a rule differs between the modes, the text says so; "per zone"
always reads "for the one implicit zone" in legacy mode.

---

## 1. Purpose

| Priority | What |
|----------|------|
| 1 | Keep every drive below its temperature limit with the least modelled fan noise. Two solvers behind one `Solver` API (`mpc.solver`): the **DAS MPC** (`mpc`, noise-minimising, model-based) and the **PI-like DAS form** (`pi`, margin deficit per output), which is also the MPC's model fallback. A fault never reduces cooling; trust and fallback are per zone |
| 2 | Drive temperatures from sensors next to each drive plus inlet and zone-air sensors. **DS18B20 on 1-Wire is a first-class source**, next to the aquaero/Quadro thermistor inputs. The controller works on an estimate with an uncertainty margin; SMART from the PC calibrates it when available |
| 3 | Digole + touch: pages, override, diagnostics |
| 4 | MQTT + HA discovery: drive estimates and margins, air temperatures, fan speeds, noise index, model status, Pi health (not raw PWM in Auto) |
| 5 | HTTP: view state, estimates, bays and model; control (drive limits, bays, mode, manual PWM, preset, identification experiments) |
| upgrade | same codebase on Zero 2 W |

The plant:

- **Drives:** up to 15, hot-swap, mixed HDDs and SSDs; heat output varies a
  lot between drives and with activity, some SSDs run very hot. Bays can be
  empty. Limits per drive class (owner decision): HDD 50 °C, SATA SSD
  65 °C, NVMe 70 °C, with comfort bands 5 / 10 / 10 °C.
- **Zones:** drives and fans are grouped into zones. Inside a zone they
  influence each other strongly, between zones weakly. Some fans work on
  the same air path and act as one group.
- **Fans:** 8–10: 4–8 on the aquaero 6 XT's 4 outputs and 2–4 on the
  Quadro's 4 outputs, so 1–2 fans per output (splitters) and **one
  tachometer per output**.
- **Sensors:** 24–30 in total. 8–12 on the XT6 + Quadro thermistor inputs
  (the XT6 has 8; how many Quadro inputs are usable is **unknown until
  the Quadro is connected**, and nothing in the code or the config
  assumes a number), the rest DS18B20 on 1-Wire. Sensors sit next to the
  drives or in the air stream; they are never glued to a drive.
- **SMART:** optional, from an agent on the PC. The DAS has **no SES
  backplane**, so nobody reports which bay a serial sits in; the
  estimator finds out by correlation (§3).
- **No install-specific assumptions:** which bays hold hot SSDs, where the
  fast sensors pay off and how strongly a fan cools a zone are found by
  the estimator, the identified model and the offline fit, not written
  into the code.
- **Noise:** there is no microphone; noise is modelled from fan speed
  (fan laws), per fan model.

Aquaero and Quadro run autonomously. The Pi is the controller. If the
daemon dies, fans must not stay pinned at the last USB PWM. XT6 firmware
curves (or an aquaero software-sensor timeout, see the USB spike) are the
hardware watchdog. That behaviour is **not** assumed: the spike must
measure it.

---

## 2. Hardware and topology

```
          DAS enclosure: zones of hot-swap bays, 8–10 fans on 8 PWM outputs
  drive-proximal sensors, zone air, inlet, exhaust      fans (1–2 per output,
          │                          │                  one tach per output)
          ▼                          ▼                        ▲
  aquaero 6 XT: 8 thermistors,   DS18B20, 3-wire, 4.7 kΩ      │
  4 PWM + 4 tach (xt1–xt4)       w1-gpio bus A: GPIO4         │
  Quadro: thermistors (count     w1-gpio bus B: GPIO17        │
  unknown), 4 PWM + 4 tach       (bus C: GPIO27 if needed)    │
  (qd1–qd4); aquabus to the XT6        │                      │
  or its own USB port                  │                      │
          │ USB (hwmon)                │ 1-Wire (sysfs)       │
          ▼                            ▼                      │
  ┌──────────────────────────── Raspberry Pi ─────────────────┴───┐
  │ gate per sensor → zone trust → estimator (latent drives)      │
  │ → DAS MPC or PI-like DAS form → per-zone fallback → PWM (hwmon)│
  └───────────────────────────────────────────────────────────────┘
          ▲                   │               │              │
          │ MQTT in/smart     ▼               ▼              ▼
  PC with the DAS:          Digole         MQTT/HA        HTTP API
  tools/smart_agent.py      touch
  (optional SMART)
```

- **The Pi does not see the drives.** Drive temperatures come from
  sensors that sit next to a drive (in the bay's air gap or on the cage
  rail touching the carrier); they are never glued on, because the drives
  are hot-swap. A sensor next to an empty bay reads air. The controller
  estimates each drive's temperature from these sensors (§3, *DAS thermal
  model and estimator*).
- **Fan controller (decided):** aquaero 6 XT (4 PWM outputs, 4 tachs,
  8 thermistor inputs) + Quadro (4 PWM outputs, 4 tachs, thermistor
  inputs). 4–8 fans on the XT6, 2–4 on the Quadro: 1–2 fans per output,
  a splitter where there are two, and only the first fan of a splitter
  drives the output's tachometer.
- **Quadro:** on aquabus with the **XT6** as master, the XT6 on USB to the
  Pi (`lsusb`: vendor `0c70`, product `f001`). If the spike shows the
  Quadro's PWM is not writable through the aquaero, the **Quadro goes on
  its own USB port** (`f00d`): the driver then exposes it as a second
  hwmon device and the config lists both under `hwmon:` (§3 Track B).
  How many Quadro temperature inputs show up in hwmon is known only once
  it is connected; `config.example-das.yaml` binds none of them.
- The daemon reads and writes through **hwmon sysfs**
  (`aquacomputer_d5next`, `/sys/class/hwmon/hwmonN/{tempK_input,fanK_input,pwmK}`),
  not raw HID. The HID udev rules stay for liquidctl and the spike. The
  hwmon ABI the adapter assumes (§3 Track B) is unconfirmed until the
  spike.
- **Sensors per role** (recommended plan for 15 bays / 4 zones; the config
  binding is the only thing that changes):

  | Role | Count | Source | Notes |
  |------|-------|--------|-------|
  | `zone_air` | 1–2 per zone | thermistor inputs first (0.01 °C, τ ≈ 5 s) | zone outlet side, mid-height; the estimator's fastest signal |
  | `inlet` | 2 | one thermistor + one DS18B20 | in front of the intake; two sources so one bus or controller loss keeps an inlet |
  | `drive_proximal` | ≥ 1 per bay | DS18B20 (1/16 °C, τ 10–20 s); spare thermistors on the bays the fit ranks tightest | in the bay's air gap; a second sensor on a bay is `redundant` |
  | `exhaust` | 1–2 | DS18B20, or a spare thermistor | behind the fans; read by the gate and identification, never a constraint |
  | SMART | up to 15 | MQTT from the PC agent | 1 °C, 30–60 s cadence; calibration and secondary measurement only |

  The thermistor inputs are the scarce, fast, high-resolution channels:
  zone air first, one inlet, then the proximal positions that
  `tools/fit_model.py` ranks by smallest margin and fastest drive
  dynamics. Plan with the XT6's 8 inputs until the Quadro's are
  confirmed.
- **DS18B20 buses:** externally powered (3-wire) sensors only (parasite
  power disables bulk read without a strong pullup), linear topology,
  4.7 kΩ pullup per bus. Two `w1-gpio` buses on GPIO 4 and GPIO 17 (§9),
  a third on GPIO 27 only if a bus exceeds about 12 sensors; splitting by
  zone pairs keeps one broken wire from blinding every zone. Each sensor
  is bound to a logical name by its ROM id (`onewire.sensors`,
  `tools/w1_commission.py`, §10); the name carries zone, bay and role
  (`mpc.sensors`).
- **SMART (optional):** `tools/smart_agent.py` on the PC the DAS is
  attached to publishes one retained MQTT message per drive; reading must
  never wake a drive from standby (`smartctl -n standby`). There is no
  SES, so the agent reports serials, not bays.
- A **powered USB hub** is still required: Zero W OTG is flaky; do not
  power devices from the board port. XT6/Quadro 12 V is their own supply.
- Digole: UART `/dev/serial0` (= `ttyAMA0`), plus I2C/SPI if wired that
  way. Bluetooth is off (`dtoverlay=disable-bt`); serial console is
  removed from UART.
- Zero 2 W upgrade: same 32-bit userland or 64-bit Lite, no `armv6` in
  the code. USB OTG is unchanged.

**Status:** only the Pi is up. The aquaero 6 XT, Quadro, Digole, the
DS18B20 buses and the DAS itself are not connected yet; the spike
questions below and the whole hardware validation (§8, §13) are open.
Everything else in this document runs against simulators and fake sysfs
trees.

### Risk (first-evening USB spike)

The `aquacomputer_d5next` driver (already in 6.18) exposes aquaero’s
**own 4 fans**. Quadro channels over aquabus may be sensors only, with
no PWM write. Until verified:

1. `sensors` and `/sys/class/hwmon/` — which `temp*`, `fan*`, `pwm*`
   exist, their units, and whether `pwmK_enable` exists.
2. Whether Quadro PWM is writable through XT6 (hwmon or HID).
3. **Does XT6 revert on its own after the Pi stops writing?** Write a PWM
   via hwmon/HID, then stop. If the fan stays pinned, firmware curves are
   *not* a watchdog. Then try the alternative: write the controller output
   into an aquaero **software / virtual sensor**, with the firmware’s
   timeout fallback and an identity curve onto PWM. That is a hardware
   watchdog for free — only if the Linux driver or liquidctl can write
   that sensor. Record which path works; the daemon follows it.

4. Which Quadro temperature inputs appear in hwmon (through the XT6, or
   on the Quadro's own device), so they can be bound in `temp_map`.

If (2) fails: the Quadro goes on its own USB port (owner decision), a
second hwmon device next to the aquaero, run with `--source hwmon` and a
`hwmon:` list of both devices (§3 Track B). This blocks the hardware
track only, not the control core.

If (3) fails, a **software** watchdog inside the daemon (ramp to
`fallback_pwm`) only covers faults the process can still act on. It does
**not** survive SIGKILL, OOM-kill, USB-hub dropout that takes the write
path with it, or **Pi power loss**. Residual risk, state it plainly: PWM
can sit at a **low** last value; then drive activity rises (hot SSDs heat
up in minutes) and nothing ramps the fans. Mitigations that still run on the Pi:

- a dead sensor path is a gate fault like any other: hold, then ramp
  high on the channels of the zones it blinds (§3); the loop keeps
  applying that command (§3 Glue)
- systemd `WatchdogSec=` + `sd_notify` (`WATCHDOG=1` every tick);
  `Restart=always` (not `on-failure`)
- on clean stop: the daemon's **SIGTERM stop path** writes `fallback_pwm`
  then exits. The signal handler only sets a stop event; the main thread
  does the write. Do not use `ExecStop=` (see §9)

Those shrink hang/crash windows. They do **not** cover Pi power loss.
That is an **accepted risk** unless spike (3) proves XT6 firmware reverts
on its own.

At exit the adapter leaves `pwmK_enable` in manual mode with
`fallback_pwm` on the fans. `Xt6Adapter.release()` (restore the original
`pwmK_enable`, handing the channels back to firmware curves) exists but is
**not** called, because it would undo the `fallback_pwm` write. If spike
(3) shows the firmware curve is the better state after exit, calling it
is a one-line change in `__main__.py`.

---

## 3. Architecture: two independent tracks

Hardware **must not** import control code; it may import only the
contract (`model.py`). Control **must not** know USB, sysfs, Digole, HTTP
or MQTT. `tests/test_hw_map.py` checks the import rule statically.

### Contract (`src/aqua_bridge/model.py`)

Every dataclass is frozen and round-trips through plain JSON
(`to_dict` / `from_dict`).

```text
PlantObservation                      # one raw sample
  temps: dict[str, float | None]      # °C, logical names; None / NaN are raw data, the gate decides
  rpm:   dict[str, float | None]
  pwm:   dict[str, float | None]      # read-back duty 0..1
  ts:    float                        # monotonic seconds, not wall clock; must be finite
  inputs: dict[str, Any]              # exogenous, never gated: inputs["smart"] = {serial: {temp_c, age_s, model}};
                                      # finite JSON; omitted from to_dict while empty

MpcCommand
  pwm:  dict[str, float]              # 0..1 per logical channel
  mode: Mode                          # auto | saturated | degraded | fallback
  diagnostics: dict[str, Any]         # str keys, JSON-serialisable, never NaN

WindowSample
  raw_temps: dict[str, float | None]  # sanitised raw temps for config.temps; None, never NaN
  cmd_pwm:   dict[str, float]         # command of that tick (the loop writes the applied one)

MpcState                              # MpcState.cold(): everything empty, no fault
  last_cmd: MpcCommand | None
  last_good_obs: PlantObservation | None    # last trusted non-fallback tick; legacy: replaced as a whole,
                                            # DAS: updated per trusted sensor
  last_raw_temps: dict[str, float | None] | None  # previous raw sample, even if untrusted
  window: tuple[WindowSample, ...]    # newest last, trimmed to window_ticks; feeds median3 and Stuck
  fault_since_ts: float | None        # first ts any fallback cause became active (DAS: earliest zone fault)
  fault_reason: FaultReason | None    # sensor_gate | solver; set and cleared together with fault_since_ts
  trusted_streak: int                 # consecutive trusted ticks; 0 on untrusted tick or solver fault
                                      # (DAS: the smallest zone streak)
  integrator: dict[str, float]        # per channel: PI integral / legacy MPC u_ss / DAS MPC first block
  solver_memory: dict[str, Any]       # JSON, layout below
  zone_faults: dict[str, ZoneFault]   # DAS: one per zone {since_ts, reason, streak, ticks};
                                      # empty in legacy mode and omitted from to_dict
```

`solver_memory` keys (all plain, finite JSON): `last_ts`, `fault_ticks`,
`stall_ticks`, `stuck_latch` (both modes); `stuck_slow` and `stuck_seq`
(decimated Stuck windows, DAS); `estimator` (the Kalman filters,
occupancy, calibrations, association); `thermal` (the identified model,
with `model_shadow`); `store_seed` (a model loaded from the store, until
the first tick applies it), `store` (what was loaded) and `fan_curves`
(model store); and one slot per solver under its name (`pi`, `mpc`).

`window` is a tuple, not a `deque`: a mutable deque inside a frozen state
breaks “same `(obs, config, state)` → same `(command, state)`” as soon as
a caller reuses a state object. Observation *structure* is validated at
construction (str keys, number-or-`None` values, finite `ts`); malformed
input raises `TypeError` / `ValueError`. `MpcCommand` construction checks
structure only; value invariants are `step`’s job (§4.1).

**`MpcConfig`** is the `mpc:` section of `config.yaml`. It validates
itself in `__post_init__`, so an invalid config cannot exist and `step`
never sees one. Unknown or missing keys, non-finite numbers and bools
where numbers belong are rejected with `ConfigError` (a `ValueError`).
Lists become tuples; ints are accepted for floats.

| Field | Default | Unit / rule |
|-------|---------|-------------|
| `dt` | required | seconds per tick; > 0 (DAS example: 5) |
| `horizon` | required | int ≥ 1. Legacy MPC: ticks. DAS MPC: prediction steps of `mpc_pred_dt_s` (example 20 × 30 s). The PI solver ignores it |
| `temps` | required | plant temperatures the gate checks; non-empty, unique. DAS: every sensor, keys of `sensors` |
| `setpoints` | required | °C per controlled temperature; keys ⊆ `temps`; each strictly inside `(temp_min_c, temp_max_c)`. Legacy: non-empty. DAS: `{}` means drive-limit regulation (the DAS solvers); a zoned config that keeps setpoints regulates them per zone, and they may only sit on `zone_air` / `drive_proximal` sensors |
| `channels` | required | logical fan channels; non-empty, unique |
| `pwm_min`, `pwm_max` | required | each in `[0, 1]`; `pwm_min < pwm_max` (equal is rejected: no valid `fallback_pwm` could exist) |
| `d_pwm_max` | required | largest PWM change per tick; > 0 |
| `fallback_pwm` | required | high cooling on fault; keys exactly `channels`; each in `(pwm_min, pwm_max]` |
| `fallback_hold_s` | required | seconds; ≥ 0 |
| `confirm_s` | required | seconds of consecutive trusted ticks; ≥ `2 * dt` and ≤ `fallback_hold_s` |
| `dT_max_c_per_s` | required | gate slew limit, °C/s; > 0 |
| `stuck_s` | required | Stuck window, seconds; ≥ `2 * dt` (useful range: minutes). DAS: validated, but each sensor uses `sensors.<name>.stuck_s` |
| `stuck_eps_c` | required | “unchanged” band, °C; > 0 (≥ sensor resolution, aquaero 0.01 °C). DAS: per sensor, `sensors.<name>.stuck_eps_c` |
| `stuck_pwm_net` | required | net PWM move that must show up in T; in `(0, 1]` |
| `stuck_sibling_dT_c` | required | net move of another temperature, °C; > 0 |
| `median3` | `false` | pre-filter; must be a YAML bool (`"false"` is rejected) |
| `temp_min_c`, `temp_max_c` | `-20.0`, `120.0` | gate absolute valid range, °C; min < max |
| `solver` | `pi` | `pi` or `mpc`. Legacy: PI / small linear MPC. DAS without setpoints: PI-like DAS form / DAS MPC |
| `solver_max_iter` | `50` | int ≥ 1; hitting it is a solver fault (DAS MPC: per box QP) |
| `channel_temps` | `{}` | legacy only (must be absent with `topology`). Channel → temperatures it controls. `{}`: every channel controls every setpoint temperature. When given: keys exactly `channels`, lists non-empty, every listed temperature has a setpoint; a single string is a one-element list |
| `pi_kp` | `0.05` | PWM per °C (DAS: per °C of margin deficit); > 0 (placeholder, untuned; DAS example 0.02) |
| `pi_ki` | `0.002` | PWM per °C·s; ≥ 0 (placeholder, untuned; DAS example 0.0001) |
| `weights` | `{}` | legacy MPC tracking weight per temperature, per °C²; keys ⊆ `temps`; ≥ 0; missing → 1.0 |
| `weight_pwm` | `0.0` | legacy MPC effort penalty per PWM²; ≥ 0 |
| `weight_dpwm` | `0.05` | MPC move penalty per PWM² per tick (DAS MPC: between move blocks); ≥ 0 (legacy example 60, DAS example 1.0) |
| `mpc_tau_s` | `120.0` | legacy MPC model time constant, seconds; > 0 |
| `mpc_gain_c_per_pwm` | `8.0` | legacy MPC model: steady-state °C drop per +1.0 PWM on one channel; > 0 |
| `mpc_estimator_gain` | `0.1` | legacy MPC disturbance estimator gain per tick; in `(0, 1]` |
| `budget_ms` | `600.0` | runtime alarm (`control/loop.py`) and CI/Pi bench gate: `step()` wall time past this logs a warning, ms; > 0 and < `budget_alarm_ms` |
| `budget_alarm_ms` | `750.0` | as above, logs an error instead, ms; > 0 |
| `budget_log_interval_s` | `60.0` | rate limit for both budget log lines, seconds; > 0 |

With `solver: mpc` in legacy mode the config also needs `weight_pwm +
weight_dpwm > 0` (strictly convex QP) and at least one setpoint
temperature with a positive weight. The MPC weights are in raw units
(°C² against PWM²): with the example horizon of 8 ticks on a plant with
minutes of time constant the tracking term is small, which is why the
example move penalty is 60, not 0.05. The DAS MPC tracks no setpoint and
needs `noise.weight_noise + weight_dpwm > 0` instead.

**DAS sections of `mpc`** (all absent = legacy mode; every one of them
except `topology` is rejected without `topology`, so a half-declared
layout never silently runs in legacy mode; with `topology` the absent
ones get their defaults):

| Section / key | Default | Rule |
|---------------|---------|------|
| `topology.zones.<z>.channels` | required | non-empty ⊆ `channels`: every output whose fans move air through the zone (a channel touching several zones is listed in each); every channel appears in ≥ 1 zone |
| `topology.zones.<z>.coupled_to` | `[]` | zones it exchanges air with; declared **symmetrically** (an asymmetric or self coupling is rejected) |
| `topology.zones.<z>.inlet` | `mix` | a sensor of role `inlet`, or `mix` (every inlet sensor) |
| `topology.bays.<b>.zone` | required | an existing zone |
| `topology.bays.<b>.class` | `default_class` | a drive class |
| `topology.bays.<b>.occupied` | `auto` | `true`, `false` or `auto` (the estimator decides; unknown is constrained like occupied) |
| `topology.bays.<b>.serial` | none | declared drive serial (wins over the association by correlation); one bay per serial |
| `topology.bays.<b>.limit_c` | none | tightens the class limit for this bay only (`min` of both) |
| `topology.default_class` | strictest class | lowest `limit_c`, then lowest `limit_c - comfort_c`, then name |
| `sensors.<name>` | – | keys exactly `temps`. `role` ∈ `inlet`, `zone_air`, `drive_proximal`, `exhaust`; `zone` required for zone air and proximal; `bay` required for proximal (and only there), in that zone |
| `sensors.<name>.quant_c` | `0.0625` | > 0 (thermistor 0.01, DS18B20 0.0625) |
| `sensors.<name>.stuck_s` | per role | proximal 1800, zone air 180, inlet / exhaust 600 s; ≥ `2 * dt` |
| `sensors.<name>.stuck_eps_c` | `1.5 * quant_c` | ≥ `quant_c` |
| `sensors.<name>.stuck_decimate` | automatic | int ≥ 1; absent: `max(1, stuck_ticks // 60)` |
| `sensors.<name>.redundant` | `false` | a backup member of its group; every zone-air group, every bay and the inlets need ≥ 1 non-redundant member |
| `sensors.<name>.tau_s` | none | sensor lag, s; > 0 (estimator default 15, thermal model: 5 thermistor / 15 DS18B20) |
| `drive_classes.<c>` | `hdd` 50 / 5 / 720, `ssd_sata` 65 / 10 / 200, `nvme` 70 / 10 / 120 | `limit_c` inside `(temp_min_c, temp_max_c)`, `comfort_c ≥ 0`, `tau_d_s > 0`, `models` a list of regexes matched against the SMART model; present means exactly the classes given |
| `fans.<ch>` | – | keys exactly `channels`; `model` ∈ `fan_models`; `count` ≥ 1 (fans on the output); `group` (shared air path, default the channel); `noise_weight` ≥ 0 (1); `forbidden_pwm` list of `[lo, hi]` with `pwm_min ≤ lo < hi ≤ pwm_max`, width ≤ 0.2 |
| `fan_models.<m>` | – | `rpm_max` > 0; `deadband` in `[0, 0.5)` (0.1); `exponent` in `[0.5, 1.5]` (1.0); `noise_db_at_max` finite (0: an index) |
| `zones.trust_rule` | `strict` | `strict` \| `sigma` (`sigma` is accepted and currently applies `strict`, §8) |
| `zones.fault_coupling` | `declared` | `declared` (a fault reaches the coupled zones' channels) \| `none` |
| `noise.exponent` / `weight_noise` / `band_hysteresis` | 5 / 1.0 / 0.02 | `[3, 7]` / ≥ 0 / ≥ 0 |
| `estimator.k_sigma` | 2.0 | `[0, 4]`; margin `= k_sigma * sigma` |
| `estimator.sigma_fault_c` / `sigma_air_fault_c` | 4.0 / 2.0 | > 0; reserved for `trust_rule: sigma` |
| `estimator.q_t_air` / `q_d_air` / `q_t_drive` / `q_t_sensor` / `q_heat` | 1e-4 / 4e-7 / 1e-5 / 1e-4 / 4e-7 | > 0; process noise per tick |
| `estimator.sensor_noise_c` | 0.03 | ≥ 0 |
| `estimator.smart_max_age_s` / `smart_reject_c` | 300 / 8.0 | ≥ `dt` / > 0 |
| `estimator.occupied_dT_c` / `empty_dT_c` / `empty_confirm_s` | 2.0 / 0.7 / 300 | `occupied_dT_c > empty_dT_c > 0`; `empty_confirm_s ≥ 2 * dt` |
| `estimator.bay_settle_s` | 600 | ≥ 0 |
| `estimator.calibration_max_age_days` | 30 | > 0 |
| `estimator.associate_window_s` / `associate_min_corr` / `associate_margin` | 3600 / 0.8 / 0.15 | ≥ 600 / `(0, 1)` / `(0, 1)` |

**Flat DAS keys of `mpc`** (validated always, inert in legacy mode; the
booleans need `topology`; the rules that depend on `dt` are checked only
where they matter, so a default never invalidates a legacy config with a
long `dt`):

| Key | Default | Rule |
|-----|---------|------|
| `model_shadow` | `false` | learn and predict the zoned thermal model online (needs `topology`) |
| `model_window_s` | 120 | `(0, 600]`; ≥ `2 * dt` with `model_shadow` |
| `model_lambda` | 0.9995 | `(0.99, 1]`, forgetting per excited window |
| `model_p_trace_max` | 100 | > 0 |
| `model_converged_rel_se` | 0.25 | `(0, 1)` |
| `model_max_pred_err_c` | 1.0 | > 0 |
| `model_use_rpm` | `false` | airflow from the tach for identification (needs `topology`) |
| `mpc_pred_dt_s` | 30 | > 0; ≥ `dt` with `topology` |
| `mpc_blocks` | `[]` | ints ≥ 1 summing to `horizon`; `[]` = `[1, 1, 2, 4, 6, 6]` cut to `horizon` |
| `mpc_every_ticks` | 1 | int ≥ 1 (DAS example 2) |
| `rho_soft` / `rho_hard` | 40 / 4000 | > 0, `rho_hard ≥ rho_soft` |
| `solver_outer_max` | 4 | int ≥ 1 |
| `model_max_drift_c_per_min` | 0.5 | > 0 |
| `model_accept_prior` | `false` | the DAS MPC may act on a model that has not converged (needs `topology`) |
| `model_store_interval_s` | 600 | > 0 |
| `model_store_max_age_days` | 30 | > 0 |
| `model_reconfirm_s` | 3600 | ≥ 0 |
| `ident_enabled` | `false` | an explicit start may run an experiment (needs `topology`) |
| `ident_amplitude` | 0.15 | `(0, 0.3]` |
| `ident_levels` | `above` | `above` \| `symmetric` |
| `ident_hold_s` | `[60, 120, 180]` | non-empty, positive; each ≥ `5 * dt` when enabled |
| `ident_max_duration_s` | 1800 | `(0, 7200]` |
| `ident_settle_s` | 600 | ≥ 0; ≥ `confirm_s` when enabled |
| `ident_start_band_c` / `ident_max_over_c` | 1.0 / 3.0 | > 0 |
| `ident_seed` | 1 | int ≥ 0 |

`budget_ms` / `budget_alarm_ms` (600 ms / 750 ms default, at `dt = 5 s` in the
DAS example) gate both the benchmark (`tools/bench_step.py`,
`tests/test_bench_budget.py`) and the runtime alarm (`control/loop.py`, §4.3
"Loop / glue"); both read the config, never a hardcoded literal.

Derived tick quantities are properties, never YAML keys:

- `dT_max_tick = dT_max_c_per_s * dt`
- `confirm_ticks = max(2, ceil(confirm_s / dt))`
- `stuck_ticks = max(2, ceil(stuck_s / dt))`
- `temps_for_channel(ch)`: `channel_temps[ch]`, or every setpoint
  temperature in `temps` order (DAS: the setpoint sensors of the zones
  listing `ch`)
- `weight_for(temp)`: `weights.get(temp, 1.0)`
- DAS views: `is_das`; `regulates_drive_limits` (topology and no
  setpoints); `zone_layout` (zones, channels per zone, coupling, the
  required sensor groups per zone, `reach` = channels a zone fault puts
  under fallback policy, `served` = zones a channel works for: the zones
  listing it plus their `coupled_to`); `stuck_params(name)` (window
  ticks, band, decimation, evidence channels and siblings of one sensor);
  `window_ticks` (length of `MpcState.window`: `stuck_ticks` in legacy
  mode, the longest undecimated sensor window, ≥ 3, in DAS mode);
  `bay_class(bay)`, `bay_limit(bay)`, `bay_comfort(bay)`; `blocks()`

Changing `dt` must not silently change the °C/s slew limit. The config is
rejected if `confirm_s > fallback_hold_s`: a real Jump would hit
hold/ramp-high before the new level confirms. Do not “accept the fan
spike” as default.

The rest of `config.yaml` (`config.py`, `AppConfig`): `mpc` is required
and typed; `mqtt`, `host`, `xt6`, `http`, `digole`, `onewire` go to their
owners as plain dicts (each must be a mapping or absent); `hwmon` is the
one section shaped as a list (one mapping per hwmon device); unknown
top-level sections are kept in `AppConfig.extra`, visible but not fatal.
`xt6`, `hwmon` and `onewire` are read by Track B (below), `http` by §6,
`mqtt` and `host` by §7. Read from `extra`: `record_path`,
`record_max_bytes`, `record_backup_count` (the tick recorder, §3 Glue)
and `sim.das` (`{topology?, preset?, seed?}`, the DAS simulator for
`--sim-plant das`). `digole`, `onewire.enabled`, `onewire.buses` and
`xt6.prefer` are parsed but no code reads them.

Channel names are logical (`xt1`, `qd2`, …; `radiator` in legacy mode).
Mapping onto `hwmon pwmN` lives only in the hardware adapter.

One controller step is a pure function of observation, config, **and
state**. It returns the command **and** the next state (integrator,
last-good PWM, fault timers):

```text
mpc.step(observation, config, state, *, solver=None) -> (command, state)
```

No files, no sockets, no `time.time()` inside the solver (`observation.ts`
and `config.dt` are the clock). Tests inject `state` and may inject a
`solver`. Same `(observation, config, state)` → same `(command, state)`
(deterministic). `step` raises `TypeError` only for arguments of the
wrong type; every runtime problem becomes a fault of the zones it
affects (`mode=fallback`, or `degraded` when only some zones are hit).

**Order inside `step`** (`control/mpc.py`; the DAS-only steps are
skipped in legacy mode):

1. Resolve `prev`, what the command is rate-limited against.
2. Time check of `obs.ts` against `solver_memory["last_ts"]`.
3. Sensor gate, fed the Stuck latch from `solver_memory["stuck_latch"]`
   (DAS: the decimated Stuck windows are advanced first).
   - 3b. Zone trust (`control/zones.py`); legacy mode is one implicit zone
     whose verdict is exactly the whole-tick gate verdict.
   - 3b'. DAS, first tick only: apply a model loaded from the store
     (`control/persist.py`).
   - 3c. DAS: the estimator (`control/estimator.py`), every tick, fault
     ticks included; an estimator error faults the zones with constrained
     bays (reason `solver`) when the solver regulates on estimates.
4. Fault bookkeeping per zone (timer, streak).
5. Solver, only for the zones that are trusted and fault-free or on their
   `confirm_ticks`-th consecutive trusted tick; the channels of the other
   zones reach it as fixed inputs.
6. Fallback policy per channel while a zone that reaches it is in fault.
7. Rate limit against `prev` (`|Δ| <= d_pwm_max`), then clamp into
   `[pwm_min, pwm_max]`.
8. Mode.
   - 8b. DAS with `model_shadow`: the thermal model's online
     identification (`control/thermal.py`), after the command is final,
     so it learns without acting; an exception resets it to its prior
     with status `error`, never a fault.
9. Next state: raw temps and `cmd.pwm` always go into `window` and
   `last_raw_temps`.

**`prev` and “trusted `obs.pwm`”.** In order: `state.last_cmd.pwm` (when
it has a finite value for every channel); else `obs.pwm` when it is
*usable*; else `config.fallback_pwm`. **Never** `pwm_min`. `obs.pwm` is
usable when every channel is present, finite and inside
`[max(0, pwm_min - d_pwm_max), min(1, pwm_max + d_pwm_max)]`: exactly the
band from which one rate-limited step can reach `[pwm_min, pwm_max]`, so
the bound and the rate-limit invariants (§4.1) can both hold.
`diagnostics["prev_source"]` says which one was used.

**Per-zone trust and the fault closure** (`control/zones.py`, pure). One
flaky DS18B20 must not put all 8–10 fans on `fallback_pwm`. The gate
stays per sensor; zone `z` is **trusted** this tick iff the time status
is `first` / `ok`, `obs.temps` has no unknown key, and its trust rule
holds:

- `strict` (default): every **required group** of the zone has at least
  one gate-trusted member. The groups are the zone's `zone_air` sensors,
  the `drive_proximal` sensors of each bay declared `occupied: true` or
  `auto`, and each setpoint sensor on its own (a zoned setpoint config
  has no estimator that could stand in for a lost regulated sensor).
  Redundant members only matter as "one of them is enough"; sensors
  outside every group (inlet, exhaust, the proximal sensor of an
  `occupied: false` bay) are still gated but never fault a zone.
- `sigma`: accepted by the config, but the rule on the estimator's
  uncertainty is not implemented yet; `strict` applies and the
  diagnostics report `trust_rule: strict` (§8).

`F` is the set of zones in fault; the closure `F* = F ∪ {coupled_to of
every zone in F}` uses the **declared** topology, never identified
numbers (`zones.fault_coupling: none` makes `F* = F`). A channel is
**under fallback policy** iff it is listed in `topology.zones.<z>.channels`
for some `z ∈ F*` (`ZoneLayout.reach`); every other channel follows the
solver. "A fault never reduces cooling" holds per zone and across
coupling, in three parts:

1. every channel that moves air through a faulted zone or a zone it
   exchanges air with is never commanded below `prev` while the fault
   lasts (the hold-then-high policy below, restricted to that set);
2. the solver gets those channels as known inputs at their command
   (`SolverRequest.fixed_channels`), only the sensors and estimates of
   zones that are not in fault, and no rows for the faulted zones' drives;
   the DAS MPC still predicts the fixed channels' airflow and the faulted
   zones' air through the model, so a neighbour's fault can only add
   cooling to a healthy zone;
3. the solver cannot lower a healthy zone's fan below what that zone's
   own constraints require, and those constraints never read a faulted
   zone's sensors. The opposite direction (a healthy zone's fans slowing
   and starving a faulted neighbour) is excluded by part 1, because
   coupled zones are in `F*`.

With `coupled_to` empty everywhere the rule is strictly per zone. A
channel reached by several faulted zones follows the oldest of their
timers (the first to ramp high). A solver fault faults every zone the
solver was asked to drive and restarts every zone's confirmation count.
When every channel stays under fallback policy the solver is not called.
The legacy `mpc` solver has one coupled problem over all channels and
refuses fixed channels, so on a zoned setpoint config any zone fault
becomes a fault of every zone it drives (whole-enclosure fallback); the
DAS solvers plan beside faulted zones.

**Mode** (both modes): `fallback` iff every zone is in fault (legacy: the
one zone), `degraded` iff some but not all are, else `auto` /
`saturated`. `fault_since_ts` / `fault_reason` / `trusted_streak` are the
aggregates over zones (earliest fault and its reason, smallest streak),
so the legacy invariants hold verbatim; `zone_faults` carries the per-zone
timers and `confirm_ticks` applies per zone.

**Safe PWM on untrusted observation** (per channel under fallback
policy, with the timer of the zone that reaches it): never jump toward
`pwm_min`. That would cut cooling and would also violate `|Δpwm| <=
d_pwm_max` if the last command was high.

1. Hold `prev` (rate-limit vs that command, not vs `pwm_min`) while the
   fault has lasted **no longer** than `fallback_hold_s` (strict `>`
   ends the hold; `fallback_hold_s = 0` ramps from the second fault
   tick).
2. After that, ramp each channel toward **`max(prev, fallback_pwm)`** at
   `d_pwm_max` per step. `fallback_pwm` is a floor the fans are raised
   to, never a level they are pulled down to: a channel already above it
   (a hot, saturated plant at `pwm_max`) is held. Owner decision: a
   fault never lowers the fans.
3. The clamp into `[pwm_min, pwm_max]` still applies, so a `prev` below
   `pwm_min` (a usable `obs.pwm` on a cold start) is raised to `pwm_min`.
4. **First tick**, no `last_cmd`, and `obs.pwm` not usable:
   `prev = config.fallback_pwm`. An untrusted tick then commands
   `cmd.pwm = config.fallback_pwm` (`Δ = 0`), and
   `assert_command_safe(..., prev_pwm=fallback_pwm)` sees that same dict.
   A trusted tick is `auto` at once and the bumpless solver starts from
   `fallback_pwm`.
5. First tick with usable `obs.pwm`: `prev = obs.pwm`; an untrusted tick
   holds it.
6. `mode=fallback` (DAS: `fallback` or `degraded`) for the whole time a
   fallback cause is active; that mode is set exactly when the returned
   state has `fault_since_ts` set.

Elapsed fault time is `max(obs.ts - fault_since_ts, (fault_ticks - 1) *
dt)` with `fault_ticks` counted in `solver_memory`, so a stuck clock still
ramps high by tick count.

**One** timer per zone (legacy: one `fault_since_ts`) covers every reason
`step` cannot emit a trusted command for it: sensor gate **or** solver
fault. The reason records which. The hold/ramp-high policy is identical.
Do not run two independent hold timers for one zone. A solver fault on a trusted tick also resets
`trusted_streak` and keeps the original `fault_since_ts`: otherwise a
persistently failing solver would restart the timer every tick and never
ramp high.

Clear `fault_since_ts` / `fault_reason` and return to `auto` only after
`confirm_ticks` **consecutive** trusted **observations**
(`trusted_streak`); the `confirm_ticks`-th trusted tick is itself the
first `auto` tick. One good sample is not enough. A sensor that lies
every other tick (Flicker) must keep `mode=fallback` and must not reset
the hold timer. Any untrusted tick sets `trusted_streak = 0`.
`confirm_ticks` only clears an **active** fault: a trusted first tick
from a cold state is `auto` immediately (bumpless, output equals `prev`).

`stuck_s` is a **window**, not a debounce: within it the sensed node must
have had time to answer a PWM move. Size it at several plant time
constants (minutes). 30 s flags a healthy sensor right after a setpoint
step, because PWM moves in seconds and the plant in minutes.

**Time policy** (`GAP_TICKS_MAX = 3` is a module constant, not config):

- First tick (no `last_ts`): no time check (`time.status = "first"`).
- `ts <= last_ts` (duplicate, stalled or backwards clock): untrusted tick
  (`"not_advancing"`), `sensor_gate` fault. `last_ts` still follows
  `obs.ts`, so a clock reset costs `confirm_ticks` ticks and a
  permanently stuck clock keeps the controller in fallback, ramping high
  by tick count.
- `ts - last_ts > 3 * dt` (missed ticks): untrusted tick (`"gap"`);
  `window` and `last_raw_temps` are dropped so the next tick compares
  against this fresh sample; `last_good_obs` and the Stuck latch are kept.
- Otherwise (`"ok"`) the slew limit is `dT_max_c_per_s * max(ts - last_ts,
  dt)`, never below `dT_max_tick`.

**Trusted tick (sensor gate), whole observation** (`control/gate.py`,
`evaluate_gate`; never raises on malformed temperatures, never touches
PWM):

1. Pre-filter, **only if `config.median3`** (default `false`): the value
   checked is the median of the last three raw samples (the two newest
   window entries plus the current one). With fewer than three samples,
   or a `None` among them, the current raw value is used unfiltered. The
   “previous raw” slew reference is then the previous tick’s *filtered*
   value (recomputed from the window), and `last_good_obs` stores the
   filtered temperatures; `last_raw_temps` and `window` always keep raw
   values. Taken literally (median against the raw previous sample) a
   Jump would produce zero untrusted ticks; this reading gives the
   “surfaces one tick later, confirms one tick later” behaviour below.
2. Per temperature in `config.temps`: present, not `None`, finite, inside
   `[temp_min_c, temp_max_c]`, **and** `|T - ref| <= slew limit` for **at
   least one** of `ref = last_good_obs.temps[name]` **or** `ref =` the
   previous (filtered) raw value. With neither reference (cold state) the
   slew check passes. A key missing from `obs.temps`, or a key not in
   `config.temps`, makes the tick untrusted (§4.3), never a crash.
   `diagnostics["gate"]["reasons"]` names the failed rules: `missing`,
   `null`, `non_finite`, `range`, `slew`, `stuck`.
3. **Stuck:** needs a full window of `stuck_ticks` samples. Over the
   window plus the current tick every checked value (the median when
   `median3` is on) stayed within `stuck_eps_c` of the oldest window
   sample, **and** in that window either
   - the **net** commanded PWM displacement on some channel exceeds
     `stuck_pwm_net`, measured from the oldest window sample to the
     sample `max(0, min(stuck_ticks // 4, stuck_ticks - 2))` ticks
     **before** the newest (`stuck_pwm_lag`), **or**
   - another temperature in `config.temps` moved (net) by more than
     `stuck_sibling_dT_c` along a physically plausible path: every
     sample finite and in range, no single step above `dT_max_tick`.

   Net, **not** a running sum of `|Δpwm|`: at equilibrium the solver
   dithers by a few thousandths each tick, the sum crosses any threshold,
   the net stays near zero. The lag is the chosen reading of the sizing
   rule above: a move commanded two ticks ago has had no time to show up
   in the coolant, and the literal `|pwm[t] - pwm[t - stuck_ticks]|`
   flags a calm, healthy sensor on every fast move from a still
   equilibrium (a setpoint step, the return from a fault) and makes
   nominal operation flicker into fallback. A sustained ramp across the
   window is still caught. The plausible-path condition keeps a Spike or
   Jump on one sensor from branding a calm sibling as Stuck. A `None`
   inside the run breaks the check.

   Once fired the flag is **latched** on its band reference (the oldest
   window sample at that moment): the temperature stays untrusted while
   its value stays within `stuck_eps_c` of that reference, whether or not
   the evidence is still inside the window. The flag clears when the
   value leaves the band; then the normal `confirm_ticks` rule applies.
   A `None` value keeps the latch (nothing proved the sensor alive); a
   gap tick drops the window but keeps the latch. Without the latch the
   fallback hold freezes the command, one window later the net PWM move
   is gone, the frozen reading confirms as “at setpoint” and the
   controller limit-cycles between auto and fallback on a dead sensor.
   The latch lives in `solver_memory["stuck_latch"]` (temperature → band
   reference °C) and in `diagnostics["gate"]["stuck_latch"]`.
4. The **tick** is trusted iff **every** temperature in `config.temps`
   is trusted, `obs.temps` has no unknown key, and the time status is
   `first` or `ok`. Legacy: `trusted_streak` is one integer for the whole
   observation, not per channel, and `last_good_obs` is replaced **as a
   whole** on every trusted tick that yields a non-fallback command
   (steady auto: every tick; after a fault: the confirming tick), with
   non-finite rpm/pwm stored as `None`. DAS: this whole-tick verdict is
   only informative; trust is decided per zone from the per-sensor
   verdicts (above), each zone keeps its own streak, and `last_good_obs`
   is updated per trusted sensor.
5. Always push this tick’s raw values (`None` for missing / NaN / inf,
   restricted to `config.temps`) and `cmd.pwm` into `window` and
   `last_raw_temps`, trusted or not.

- **Spike** (+125 °C for one sample, then back): one untrusted tick.
  The next sample is near last good → streak can resume; last good
  does not move.
- **Jump** (real +30 °C that then holds): first new-level sample
  untrusted (far from both refs). Following samples are near
  **previous raw** → those channels are trusted. When the **whole
  tick** is trusted `confirm_ticks` times, `last_good_obs := obs`.
  A Jump on one of two sensors makes that tick untrusted until that
  channel confirms; the other channel does not get a partial last-good
  update.
- **With `median3 = true`:** a single Spike is removed by the median
  and produces **zero** untrusted ticks; a Jump surfaces one tick later
  and confirms one tick later. §4.4 rows state the default (`false`);
  the same tests are parametrised for `true`.
- **Swap and impossible combinations:** the gate has no history-free
  cross-sensor plausibility rule. Two proximal sensors exchanged (the
  ROM ids of `prox_b01` and `prox_b07` swapped in `onewire.sensors`), or
  a persisting “zone air 18 °C, proximal 90 °C”, is a Jump on both
  sensors: untrusted at onset (their zones hold), confirmed after
  `confirm_ticks`, and because `confirm_s <= fallback_hold_s` it never
  reaches the ramp. A cross-sensor rule would trap every simultaneous
  double Jump in fallback forever. A real swap is a binding mistake; it is
  caught at commissioning (`tools/w1_commission.py --identify`, warming
  each thermistor, §10), not by the gate. (Legacy wording: a persisting
  coolant/air swap in `xt6.temp_map`.)

A gate that only compares to last good would keep Jump untrusted
forever and slam fans to `fallback_pwm`. That is a spec bug; do not
implement it. A gate without Stuck would treat a frozen reading as
trusted forever.

**Stuck sizing for slow, coarse sensors (DAS).** A DS18B20 next to a
drive with a time constant of about 12 minutes legitimately sits on one
1/16 °C code for minutes while the fans of its zone move; the global rule
would brand it Stuck, latch it and fault the zone. With `topology` every
number of rule 3 comes from `MpcConfig.stuck_params(name)`:

- window and band are the sensor's own: `sensors.<name>.stuck_s`
  (defaults per role: `drive_proximal` 1800 s, `zone_air` 180 s, `inlet`
  and `exhaust` 600 s) and `stuck_eps_c` (default `1.5 × quant_c`, never
  below `quant_c`);
- the PWM evidence counts only the channels of the sensor's zone (an
  inlet without a zone has none: the fans do not move the inlet);
- the sibling evidence counts only other sensors of the same zone **and**
  role.

Long windows are **decimated**: a sensor whose window has `n` ticks keeps
one sample every `stuck_decimate` ticks (default `max(1, n // 60)`, so
1800 s at `dt = 5` keeps 60 samples, one every 6 ticks) in
`solver_memory["stuck_slow"]`, fed from the previous tick's dense sample
(which carries the applied command). The check interpolates nothing; a
sibling's single step is bounded by `k × dT_max_tick`. An excursion
shorter than `k` ticks can go unseen, which can only flag Stuck more
readily (its zone faults, cooling rises), never less. A malformed stored
window starts over, as after a gap.

**Mode and saturation.** `fallback` while every zone is in fault,
`degraded` while some are; `saturated`
when some channel’s solver demand exceeds `pwm_max` **and** the emitted
PWM sits at `pwm_max` (while the rate limit is still ramping toward the
rail the mode stays `auto`); `auto` otherwise. The flag must not chatter
at an unreachable setpoint: the PI solver uses clamp-only anti-windup
(integral clamped into `[pwm_min, pwm_max]`, not conditional
integration), so its demand stays strictly above the rail while the
error is positive, at the cost of a de-saturation lag of up to
`pi_kp * e / (pi_ki * |e|)` seconds that errs toward more cooling; the
MPC reports its first move plus the gradient pressure on an active bound.
There is no `manual` value in `Mode`; the human-facing control mode lives
in the supervisor (§6).

**Fan stall** (`STALL_TICKS = 10`, module constant): per channel,
`solver_memory["stall_ticks"]` counts consecutive ticks with a finite
`obs.rpm[ch] <= 0` while the previous command for `ch` was at or above
the midpoint of `[pwm_min, pwm_max]`; `rpm > 0` resets it, a missing
reading leaves it. After 10 ticks `diagnostics["fan_stall"][ch]` is
`true`. Policy: **flag only**, no mode change. The PI integral is capped
at `pwm_max`, so it cannot wind up; the demand on a stalled channel
saturates honestly; fallback would add no cooling (a stalled fan moves no
air at `fallback_pwm` either) and would stop regulating the healthy
channels. Stall detection needs `obs.rpm` keyed by channel name: the
`rpm` attribute of the channel's `xt6.fans` or `hwmon[].fans` entry
(Track B); a tach-less output (no `rpm:`) is never flagged.

**Bumpless transfer.** While `mode=fallback` the solver does not run, so
neither the PI integral nor the MPC disturbance estimate updates (no
windup on an unused output). On the tick that returns to `auto`, and
whenever `integrator` lacks a channel (cold state, or a channel just
released from a manual override, §6), the solver is re-initialised so its
first output **equals** `prev` before the rate limit. After a long
fallback, PWM must not rail because I accumulated in the dark, and must
not jump because I was left at a stale value. When `prev` lies outside
`[pwm_min, pwm_max]` (a usable `obs.pwm` of 0.06 with `pwm_min` 0.15)
equality is unattainable: the PI demand equals `prev` and the clamp emits
`pwm_min`; the MPC targets `clip(prev, pwm_min, pwm_max)` for its first
planned move, and its demand may carry honest pressure past either rail.
Either way the emitted command is `clip(prev)`. In DAS mode the same
holds per channel: a channel released from fallback policy (its zones
confirmed) or from an override has no integrator entry, and the solver
initialises on it bumplessly; the DAS MPC does it with a decaying output
offset (below).

**Solvers.** `mpc.solver` selects one from a registry in `control/mpc.py`
(`solver_for(cfg)`): `SOLVERS` in legacy mode and on zoned setpoint
configs (`pi` → `PiSolver`, `mpc` → `MpcSolver`), `DAS_SOLVERS` when
`regulates_drive_limits` (`pi` → `PiSolver` in its margin-deficit form,
`mpc` → `DasMpcSolver` in `control/solver_das.py`). All implement
the `Solver` protocol in `control/solver_pi.py`: `name`;
`initialise(cfg, req) -> (integrator, memory)` such that an immediate
`solve` returns `req.prev_pwm`; `solve(cfg, req) -> SolverResult` with the
**unclamped** demand per channel, the next integrator and memory,
`converged` and `iterations`. `step` owns the gate, timer, fallback, rate
limit and clamp; a solver only turns trusted (filtered) temperatures into
a demand. Each solver keeps its own memory under
`solver_memory[solver.name]`. An exception, a result that is not a
`SolverResult`, wrong channel keys, a non-finite demand or integrator,
memory that is not finite JSON, `converged=False`, `iterations >
solver_max_iter`, or no registered solver is a `solver` fault, never an
exception out of `step`. `SolverRequest` carries, besides the trusted
temperatures and `prev_pwm`: `fixed_channels` (channels under fallback
policy with their command; the solver returns a demand for them, which
`step` ignores), `zone_trust`, and in DAS mode `estimates` (per
constrained bay of an eligible zone), `occupancy`, `ts`, `plant` (the
estimator's zones and bays) and `thermal` (the previous tick's thermal
memory).

- **PI** (per channel): `e = max(T - setpoint)` over
  `temps_for_channel(ch)` (the hottest deviation drives the fan);
  `u = pi_kp * e + I`; `I' = clamp(I + pi_ki * e * dt, pwm_min, pwm_max)`;
  bumpless `I = prev - pi_kp * e`.
- **PI-like DAS form** (same `PiSolver`, `name = "pi"`, when
  `regulates_drive_limits`): the error is the **margin deficit** of the
  worst drive the channel cools,
  `e = max(t − soft)` over the constrained bays (occupied or
  unknown, not found empty this tick) of the zones in
  `zone_layout.served[ch]` whose zone is trusted, with
  `soft = limit − comfort − k·σ` from the estimates. Integrator,
  anti-windup and bumpless start are the PI above. `k·σ` counts once
  (owner decision, §8.1): it is inside `soft` and not added to `t`, so
  the worst drive settles at `soft = limit − comfort − k·σ`, the same
  target as the DAS MPC's soft rows. A channel whose
  served zones hold no constrained bay of a trusted zone has `e = 0`
  (holds its command, listed in `diagnostics["unconstrained"]`); a
  constrained bay of a trusted zone without an estimate raises, which
  `step` turns into a fault. Fixed channels get no error and no
  integrator entry. This form is the DAS MPC's model fallback and the
  last-resort solver: it needs no model, only estimates.
- **Legacy MPC** (`control/solver_mpc.py`, numpy only): per controlled
  temperature `j` (every temperature with a setpoint)
  `T_j[k+1] = a T_j[k] + Σ_i B_ji u_i[k] + d_j` with
  `a = exp(-dt / mpc_tau_s)` and `B_ji = -(1 - a) * mpc_gain_c_per_pwm`
  when `j` is in `temps_for_channel(i)`. Cost over `horizon`: weighted
  tracking error, `weight_pwm * |u|²`, `weight_dpwm * |Δu|²` with
  `u_{-1} = prev`; box `pwm_min <= u <= pwm_max`. The condensed QP is
  solved by a primal active-set method, warm-started from the previous
  plan shifted by one tick; `solver_max_iter` caps the active-set
  iterations. `d_pwm_max` is not a QP constraint; `step` enforces it.
  Offset-free: `d <- d + mpc_estimator_gain * (T_now - predicted)` using
  the previous emitted command. `integrator` holds `u_ss`, the clamped
  equilibrium PWM for the setpoints under `d`. Bumpless initialisation
  chooses `d` so the constrained first move equals `clip(prev)`: it keeps
  the physical prior `d_eq = (1 - a) T_0 - B prev` when that already
  satisfies the condition, otherwise it searches, and a solution pinned
  on a bound is pulled back toward `d_eq` to the nearest satisfying point
  (the knee). Any other point of that half-line is a fictitious heat load
  that the estimator unwinds over the next ticks, moving the PWM the
  wrong way on a cold loop. The first solve after `initialise` skips the
  estimator.
- **DAS MPC** (`control/solver_das.py`, `name = "mpc"`): least modelled
  fan noise with every drive under its soft and hard targets, planned on
  the identified zoned thermal model; see the next subsection.

**Diagnostics** (`MpcCommand.diagnostics` from `step`, all JSON):
`trusted`, `gate` (`trusted`, `per_temp`, `filtered`, `raw`, `reasons`,
`unknown_keys`, `stuck`, `stuck_latch`), `time` (`status`, `dt_obs`,
`dT_limit`), `prev_source`, `prev_pwm`, `obs_pwm_usable`, `target_pwm`,
`rate_limited`, `saturated`, `policy` (`hold` | `ramp_high` | `solver`),
`fault_reason`, `fault_since_ts`, `fault_elapsed_s`, `fault_ticks`,
`trusted_streak`, `confirm_ticks`, `returning_to_auto`, `solver`,
`solver_ran`, `solver_error`, `solver_diag`, `fan_stall`, `median3`. The
supervisor adds `supervisor`; the loop’s emergency and shutdown commands
carry `policy: emergency` / `policy: shutdown`.

DAS mode adds: `zones` (per zone: trusted, reasons, fault timer,
`in_closure`, channels, policy), `zones_in_fault`, `fallback_channels`,
`policy_by_channel` (`policy` becomes `mixed` when channels differ),
`trust_rule`; `estimates` (per constrained bay: `t_c`, `sigma_c`,
`margin_c`, `soft_c`, `hard_c`, `limit_c`, `limit_margin_c`, `occupancy`,
`class`, `zone`, `zone_trusted`, `calibrated`, `source`, `q_w`); `bays`
(every bay: occupancy, class and its source, serial, association,
calibration, candidates, pending occupancy evidence); `estimator`
(status, error, per zone air estimate, SMART counters); `noise`
(`db_index` from the fans' speed, `db_index_cmd` at the new command, per
channel rpm and its source); `thermal` (with `model_shadow`: status,
`pred_err_c`, per zone and bay the coefficients with relative standard
errors, a pending hold); `store` (with a model store); and inside
`solver_diag` of the DAS MPC `model` (`active`, `reason`, `since_ts`,
`status`, `checks`), `demand`, `pressure`, `bias`, `band`, `pred_err_c`.
A legacy config gains none of these keys.

### DAS thermal model and estimator

Everything here is pure (no clock, no I/O, no randomness; plain-JSON
memory in `solver_memory`) and runs only in DAS mode. Units: W, J/K,
W/K, °C, s.

**Latent drive temperature** (`control/estimator.py`). Drives are never
measured. Per zone `z` the estimator runs **one Kalman filter** over
`[T_a, d_a, T_d…, T_s…, q…]` for every bay of the zone (the layout never
changes with occupancy):

```text
C_a dT_a/dt   = Σ_j g_j (T_d,j − T_a) − (Q_z + leak)(T_a − T_in) + Σ_z' κ (T_a,z' − T_a) + C_a d_a
C_d dT_d/dt   = C_d q_j − g_j (T_d,j − T_a)                g_j = g0 + k·Qn_z
τ_s dT_s/dt   = s_j T_d,j + (1 − s_j) T_a + b_j − T_s,j    s_j = 1 − β_j
dd_a/dt = 0,  dq_j/dt = 0                                  (integrating disturbances)
```

`T_d` is the drive-reported temperature equivalent (what SMART reports
and what limits mean). The proximal sensor is its own node, lagging both
drive and air: a static mixing map on a lagging sensor makes a fan
increase look like drive heating. **Offset-free rule:** one integrating
disturbance per measured node (`d_a` per zone, `q` per drive), with small
process noise on the physical states, so the open-loop prediction stays
consistent at steady state instead of drifting hot. Priors: `C_a` 200,
`leak` 1, `κ` 3 for declared pairs, `E` 33 W/K per fan split over the
zones listing the channel, `g0` 0.3, `k` 0.5, `C_d = tau_d_s·(g0 + k)`,
sensor map `s = 0.7`, `b = −2.1 °C`. Airflow per channel
`φ = clip((u − deadband)/(1 − deadband), 0, 1)^exponent` at `u = prev`;
`Q_z = Σ E φ`, `Qn_z = Q_z / Σ E`; `T_in` from the zone's trusted inlet
sensors (`inlet` or `mix`). Transition: the **exact discretisation** of
the affine system (matrix exponential, numpy only; low-order Euler is
unstable for the ~2 s air node at `dt = 5 s`). Measurements are
sequential scalar updates in **Joseph form** on every trusted zone-air
sensor and every trusted proximal sensor, `R = sensor_noise_c² +
quant_c²/12` (an empty bay's sensor measures air at weight 0.5);
untrusted sensors are skipped. SMART enters as a measurement of `T_d`
(`R = 1 °C²`) only for a calibrated bay. The filter runs every tick,
fault ticks included. A **fast-swap rule** inflates a bay's drive and
sensor variance on a proximal innovation above 0.5 °C and 6σ, so a
pulled-and-replaced drive widens its margin at once.

Output per constrained bay: `t = T̂_d`, `σ = sqrt(P_dd + σ_cal²)`, margin
`k_sigma·σ`, `soft = limit − comfort − k·σ`, `hard = limit − k·σ`,
`q_w`; per zone air estimate and drift. `σ_cal` is 1.5 °C uncalibrated and
`max(0.5, EW-RMS residual)` calibrated; the filter cannot shrink it. This
is the honest cost of not touching the drives: without SMART the absolute
sensor-to-drive offset is a prior, and it decides how loud the fans run.

**Occupancy** (`topology.bays.<b>.occupied`, runtime `POST /api/bay`):
`true` is always `occupied`, `false` always `empty`; `auto` runs a machine
on `ΔT = T̂_s − T̂_a` and the heat the filter attributes to the drive:

- `unknown` at start and after a tick without any trusted proximal member
  of the bay (a redundant member keeps the state);
- `unknown → occupied` at once when `ΔT > occupied_dT_c` or a SMART sample
  of the bay's associated serial arrives;
- `occupied | unknown → empty` after `empty_confirm_s` with
  `ΔT < empty_dT_c`, heat < 1 W and no fresh SMART from an associated
  serial, every one of them on ticks with a trusted zone-air sensor;
- `empty → occupied` when `ΔT > occupied_dT_c` on 3 consecutive ticks,
  with the drive state reset to the sensor and a 25 °C² variance.

`unknown` and `occupied` both carry constraints (conservative). A removed
drive never faults a zone; an inserted one raises the fans through its
margin within a few ticks. The zone trust groups stay those of the
declared config.

**Drive class:** `bays.<b>.class` or `default_class`, overridden when the
SMART model of the bay's serial matches a `drive_classes.<c>.models`
regex. A serial found by correlation may only make the class stricter; a
declared serial may relax it. The bay limit is `min(class limit_c,
bays.<b>.limit_c)`, and a runtime `SetLimit` can only tighten or restore it.

**SMART calibration** (per bay and serial): each fresh SMART sample within
`smart_reject_c` of the estimate gives a row
`T_s − T̂_a = s (T_smart − T̂_a) + b` for a two-parameter RLS without
forgetting, prior `[0.7, −2.1]`, prior covariance `diag(0.05, 4)`,
slope clamped to `[0.3, 0.98]`. Accepted at 20 fresh samples with slope
variance < 0.01; the filter then uses its `s, b`. A different serial
starts from the prior; the same serial re-inserted finds its calibration
again. **Expiry (owner decision):** without an accepted sample for
`calibration_max_age_days` (default 30) the calibration keeps `s, b` as
a starting point but `σ_cal` returns to 1.5 °C until 20 fresh samples
confirm it again. SMART is never a gate input and cannot fault a zone.

**Association without SES** (`control/associate.py`). The PC reports
serials, not bays. (1) A declared `bays.<b>.serial` (config or `POST
/api/bay`) wins. (2) Otherwise, over `associate_window_s`, each
unassigned serial's SMART series (relative to its candidate zone's air
estimate, so common fan moves do not correlate everything) is correlated
with each candidate bay's `T̂_s − T̂_a` series, both detrended; a pair needs
20 paired samples and a SMART history spanning 90 % of the window.
(3) Greedy by score: accepted only with correlation ≥
`associate_min_corr` that beats the runner-up of both the serial and the
bay by `associate_margin`, and on 3 consecutive evaluations (every
60 s). (4) Dropped on a change into or out of `empty`, a proximal jump,
or a serial silent for `smart_max_age_s`. Until associated, a serial's
samples calibrate nothing. `GET /api/bays` shows the candidates with
scores.

**Zoned thermal model** (`control/thermal.py`). The network the DAS MPC
plans on: one air node per zone, a latent drive node and a proximal
sensor node per bay, with identified parameters:

```text
C_a dT_a,z/dt = Σ_j g_j (T_d,j − T_a,z) − (Q_z + leak_z)(T_a,z − T_in,z)
                + Σ_z' κ_zz' (T_a,z' − T_a,z) + p_air,z + C_a d_air,z
g_j = g0_j + k_j·Qn_z,   Q_z = Σ_G E_zG φ_zG(u)
```

`E` is sparse by declaration: channel `i` reaches zone `z` with the prior
weight `33 W/K × count / (zones listing i)` when `z` lists it, `0.1×`
when `z` is only coupled to such a zone, else not at all. Channels of one
`fans.<ch>.group` share one coefficient per zone.

| Key | Unit | Bounds | Prior | Identified from |
|-----|------|--------|-------|-----------------|
| `E.<zone>.<group>` | W/K | [0, 200] | 33/fan × count / zones listing (0.1× coupled) | air-node RLS with fan excitation; weak cross-zone E ridged |
| `leak.<zone>` | W/K | [0, 20] | 1 | air-node RLS, ridge |
| `kappa.<z>.<z2>` | W/K | [0, 50] | 3, declared pairs only | air-node RLS, ridge |
| `p_air.<zone>` | W | [−100, 100] | 0 | air-node constant, ridged toward 0 |
| `g0.<bay>` | W/K | [0.05, 5] | 0.3 | proximal RLS, ridge |
| `k.<bay>` | W/K | [0.05, 5] | 0.5 | proximal RLS with fan excitation |
| `q_s.<bay>` | K/s | [−0.2, 0.2] | 0 | proximal constant |
| `c_air.<zone>` | J/K | [50, 2000] | 200 | prior only (not identifiable at `dt = 5 s`) |
| `c_drive.<bay>` | J/K | [50, 2000] | `tau_d_s·(g0 + k)` | prior only |
| `beta.<bay>`, `b.<bay>` | –, °C | [0.02, 0.7], [−10, 10] | 0.3, −2.1 | **SMART calibration only** (estimator) |
| `tau_s.<bay>` | s | [3, 120] | `sensors.<name>.tau_s`, else by sensor type | prior only (config) |
| `u0.<model>`, `n.<model>` | – | [0, 0.5), [0.5, 1.5] | `fan_models` deadband / exponent | config; `tools/fit_fans.py` offline |

**Identifiability:** each proximal sensor has its own two-coefficient
regression, so more sensors never hurt; `E` has fewer unknowns than there
are air readings per zone. What cannot be learnt without SMART: the
absolute sensor-to-drive map (`β, b`) and the drive's time constant; the
fan gains on the drives are still learnt through the sensors, and the
residual uncertainty is carried in `σ_cal`. **Identification from
regulation alone never converges** (the fan groups move together); the
experiments below supply the excitation.

**Identification** (`thermal.update`, step 8b, only with `model_shadow`):
two windowed regressions per zone over contiguous trusted, fault-free
ticks with the same set of trusted sensors: a per-bay rc form on the
unlagged sensor target (for `g0`, `k`, `q_s`) and the zone-air balance
anchored on the drive heat (for `E`, `leak`, `κ`, `p_air`). Windows of
`model_window_s` are weighted by `sin²` (a modulating function: no
endpoint values, no data derivatives) with an exact first-order lag
correction for every sensor except the inlet. Constrained RLS per block:
Joseph form, forgetting `model_lambda` per excited window, conditional
(Schmidt) update when the fan-direction regressors do not move, ridge on
the weak directions, covariance trace bound `model_p_trace_max`, 5 %
trust region per window, projection on the bounds, residual clipping. A
PE monitor over ~30 windows and a one-window prediction error
`pred_err_c` feed the status machine per zone:
`prior → learning → converged → suspect → learning`, `error` after an
exception (memory reset to the prior), `frozen` for a zone loaded
converged from a fresh store file (never moves its coefficients, goes
`suspect` by the converged rule). `converged` needs enough excited
windows, `pe_min` above its floor, every in-zone `E` and every `k` with a
relative standard error below `model_converged_rel_se`, and `pred_err_c
< model_max_pred_err_c`. The model's status is the least advanced zone's;
`stale` while a stale store file re-confirms (below). Measured on
`sim/das.py` with group experiments, realistic quantisation, lags and
placement: per-bay `k` within 7–16 % and in-zone `E` within 9–22 %
across 8 seeds. `E` is sensitive to relative zone-air/inlet sensor
offsets (0.1 °C moves it 15–35 %, because a zone's air rise is only
0.3–1 °C); `k` is not.

**Objective and constraints** (`control/solver_das.py`, `control/noise.py`).
Noise per output from the fan laws: `r = clip((u − u0)/(1 − u0), 0, 1)`,
`L_i = noise_db_at_max + 10·log10(count) + 10·n·log10(r)`, linear power
`P_i = count·10^(L_max/10)·r^n` with `n = noise.exponent`, and the
energetic total `noise_db = 10·log10(Σ P_i)` (an index; absolute only with
datasheet `noise_db_at_max`). Per tick, over `n_b` move blocks
(`blocks()`, lengths summing to `horizon` prediction steps of
`mpc_pred_dt_s`):

```text
min  Σ_b len_b Σ_i weight_noise·w_i·[g_i (u_ib − u_i) + ½ h_i (u_ib − u_i)²] / P_ref
   + weight_dpwm Σ_b ||u_b − u_(b−1)||²                              u_(−1) = prev
   + rho_soft Σ_rows (y_r − soft_r)₊² + rho_hard Σ_rows (y_r − hard_r)₊²
s.t. pwm_min ≤ U ≤ pwm_max
```

The noise term is the quadratic surrogate of `P_i` at `prev`, normalised
by the power at full speed `P_ref` (dimensionless), with the per-fan
`fans.<ch>.noise_weight` `w_i` and a curvature floor of 15 % of the
maximum. Rows `y` are the predicted drive temperatures of every
constrained bay of a trusted zone at every prediction step, plus
**terminal equilibrium rows** `T_d,ss(u_last) = −C A⁻¹(B u_last + c)`: a
10-minute horizon is one drive time constant. `k·σ` counts once here,
as in PI-like DAS. Soft constraints are slack penalties; the
small steady violation of the soft target they leave is absorbed by the
comfort band.

**Prediction model:** the thermal network with the identified parameters
(`thermal.current_model`), reduced to air and drive nodes (exact: the
sensor nodes feed nothing back; empty bays are not states), linearised
each solve tick at the estimator's state and `prev`, with the estimator's
`q` and `d_air` low-passed over 120 s in the affine term, discretised by
an exact zero-order hold. `A` and its pieces are memoised on the
parameters and the command quantised to 1/64 (a memo, not state: cached
or recomputed, bit-identical).

**Solver:** an active-piece SQP on the legacy `solve_box_qp`: with the
violated rows as the active piece, solve the box QP warm-started (on a
normalised problem with a projected-Newton restart), backtrack on the
true penalised objective so it never increases, recompute the piece, stop
when it is unchanged after a full step, on a tiny decrease or step, or
after `solver_outer_max`. A box QP that hits `solver_max_iter` is
`converged=False`, a solver fault. Honest saturation: the demand is the
first block plus the pressure on an active bound. **Forbidden bands**
(`fans.<ch>.forbidden_pwm`) are handled after the solve: a demand inside
a band snaps to its upper edge (more cooling); leaving upward needs the
demand above the edge, leaving downward the demand below `lo −
noise.band_hysteresis`. `mpc_every_ticks` replays the stored plan between
solves (and re-solves at once when fixed channels, trusted zones,
constrained bays or the active model change).

**Validity gate and model fallback.** On every solve tick the model must
pass: thermal status `converged` or `frozen` (with `model_accept_prior:
true` also `prior`, `learning` and `off`; `stale` never); every parameter
finite and inside its bounds; every eigenvalue of `A` real and negative
(so every eigenvalue of `Ad` is in `(0, 1)`); every constrained bay has a
channel with a non-zero steady-state gain on it; the rolling one-step
(one prediction step ahead) prediction error of the drive rows ≤
`model_max_pred_err_c`; the model's equilibrium drift at the estimate ≤
`model_max_drift_c_per_min`. Bays within `bay_settle_s` of an occupancy
change, a fast-swap jump or a calibration change are left out of the last
two checks. A failure switches to the **PI-like DAS form on the same
estimates**: `mode` stays `auto`, `diagnostics.solver_diag.model` says
`active: pi_das` and why. The MPC returns only after the checks pass at
0.5× their numeric limits for 300 s. Every switch is bumpless. The
fallback regulates every drive at its soft target with the same margins,
so a model fallback changes loudness, not safety. Without
`model_accept_prior` nothing can act before a model has converged, which
needs experiments (or a fresh store file).

**Bumpless (DAS MPC):** when a channel starts being driven (no integrator
entry) or the active model switches, the solver records `bias = prev −
demand` so the first output equals `prev`; the offset decays with 15 s
when more cooling is wanted and 60 s when less. `integrator` holds the
first block per driven channel.

**Budget at `dt = 5 s`.** The Zero W is about 100× slower than the
development machine for numpy-heavy code (legacy MPC: p99 0.3–0.4 ms
there, 34.5 ms on the Zero W). Hard gate per tick 600 ms (12 % of `dt`; raised from 500 ms by the owner
after the Zero W measured a DAS MPC p99 of 507–552 ms), alarm 750 ms. Measured with `tools/bench_step.py` on the development
machine (`--sim-plant das` for the DAS rows); the Zero W column is the
100× extrapolation, to be replaced by the hardware validation:

| Configuration | Step p99 (dev machine) | Zero W (×100) | Verdict |
|---------------|------------------------|---------------|---------|
| Legacy MPC, 2 temps × 2 channels, `dt = 2` | 0.3–0.4 ms | 34.5 ms measured | legacy reference |
| DAS MPC, 25 sensors, 15 bays, 8 channels, N = 20 × 30 s, blocks `[1, 1, 2, 4, 6, 6]`, estimator every tick | 3.3–3.6 ms (9–11× the legacy MPC) | ~350 ms on a solve tick | within the gate with `mpc_every_ticks: 2` (solve every 10 s) |
| PI-like DAS form + estimator, same layout | ~1.4 ms | ~140 ms | always available |

CI checks the ratio (DAS MPC p99 ≤ 12× legacy MPC p99 in the same
process, `tests/test_bench_budget.py`); the absolute 600 ms gate runs only
on the Pi (marker `pi`). If the Pi measures worse, the reductions in
order: `mpc_every_ticks: 4`, fewer or longer blocks with a shorter
horizon, a Zero 2 W.

### DAS persistence, stale rule and experiments

**Model store** (`modelstore.py` does the I/O, `control/persist.py`
applies it purely). A zoned config loads `--model-store PATH`, else
`$STATE_DIRECTORY/model.json` (`StateDirectory=aqua-bridge` in the unit),
into its first state; a legacy config has no store (`STATE_DIRECTORY`
ignored, `--model-store` exits 2). File (schema
`aqua-bridge-model-store`, version 1, plain finite JSON): `fingerprint`
(sha256 over `dt`, channels, temps, zones, the bay-to-zone map, sensor
placement and fans; policy such as limits, classes, serials and gains is
left out, so tightening a limit keeps the model), `saved_wall`,
`thermal` (the thermal memory), `fan_curves`, `calibration` (per bay and
serial, with wall-clock `last_sample_wall` and `expires_wall` so expiry
survives a reboot) and `bays` (last occupancy, class, serial,
association; report only). `ModelPersister` (a loop `on_tick` observer)
writes atomically (temp file, `fsync`, `os.replace`, directory `fsync`)
at most every `model_store_interval_s` and once at a clean stop; a
failure keeps the previous file. A missing, corrupt, oversized,
wrong-schema or wrong-fingerprint file loads the prior with a warning.
Occupancy restarts `unknown` and correlation associations form again
after every start (drives may have moved while the daemon was down).

**Stale rule (owner decision).** A file older than
`model_store_max_age_days`, or with a wall clock behind it (a Pi without
RTC before NTP), loads `stale`: the thermal model starts in shadow with a
**hold**, its status reads `stale` whatever its zones say, so neither the
validity gate nor `model_accept_prior` lets the MPC act; the PI-like DAS
form acts meanwhile with the loaded calibrations at `σ_cal × 2` until 20
SMART samples confirm each (inflated indefinitely without SMART). The
hold is released once every zone has been `converged` with its
prediction error under `model_max_pred_err_c` for `model_reconfirm_s`;
the DAS MPC then still waits for its own validity gate and dwell,
including its one-step prediction-error guard. A **fresh** file loads
the zones that were converged as `frozen` and the MPC acts on them at
once, guard on. Saving does not launder staleness: a pending hold and the
calibration inflation survive a save and reload. `diagnostics["store"]`
and `GET /api/model` show `source` (`fresh` | `stale` | `prior`), age,
sections and warnings.

**Active identification experiments** (`control/ident.py`, pure; driven
by the supervisor). Explicit start only (`POST /api/ident`, MQTT
`cmd/ident`), with `ident_enabled: true`. The unit of excitation is a
**fan group** (`fans.<ch>.group`; a channel without one is its own
group): the whole group first, then each of its channels alone while its
siblings hold their frozen base. The sequence is a two-level random
telegraph: levels `u_base` and `u_base + ident_amplitude` (`above`,
never less cooling than the solver's level at start) or `u_base ±
ident_amplitude` (`symmetric`), holds drawn from `ident_hold_s` by a
seeded LFSR, `ident_max_duration_s` split over the phases. The levels are
ordinary supervisor overrides, so `compose` rate-limits and clamps them
and fallback beats them. **Start preconditions**, each refused with a
named reason: control mode `auto` without human overrides
(`control_mode`), last command `auto` (`mode:<m>`), no saturation, band
or stall on the target (`saturated:`, `band:`, `no_command:`,
`fan_stall:`), every served zone (listing a target channel, plus
`coupled_to`) trusted and fault-free for `ident_settle_s` (`settle:`),
no bay there `unknown` or mid-transition (`bay_unknown:`,
`bay_transition:`), no calibration in its first 20 samples
(`calibrating:`), and every drive within `ident_start_band_c` of its soft
target (`start_band:`). **Envelope** on the estimates, every tick, for
every constrained bay of the served zones, with `upper = T̂_d + k·σ`:
`upper ≤ soft + ident_max_over_c` (3 °C, owner-accepted), `upper ≤ hard`
always, and `upper < limit − 1 °C` (the absolute abort per drive; there
is no `ident_abort_temp_c`). **Abort list:** any fallback tick or tick
without a solver command, any `degraded` tick, two failed applies, a
clock running backwards or jumping past the duration, the envelope, an
untrusted served zone, an `unknown` bay, a stalled experiment fan, `stop`,
and any human intent. An abort releases the channels; the solver
re-initialises bumplessly on them and moves at most `d_pwm_max` per tick.
The control mode stays `auto` during an experiment; its status is in
`snapshot().extra["experiment"]`. A restart never resumes an experiment.
The envelope still adds `k·σ` to `T̂_d` on top of `soft` and `hard`, which
already subtract it. PI-DAS counts `k·σ` once and rides `T̂_d = soft`, so a
settled enclosure under PI-DAS (or the MPC's PI-DAS fallback) has
`upper = soft + k·σ`: with the uncalibrated σ (`k·σ` about 3 °C) that is
outside `ident_start_band_c` and on the edge of `ident_max_over_c`, and an
experiment starts only while the drives sit at least `k·σ −
ident_start_band_c` below their soft targets (open, not changed by the
PI-DAS change). Experiments are needed where PI-DAS acts: a model
converges only with them.

### Track A — core (dev machine / CI, no Pi)

- Safety core: `control/gate.py`, `control/zones.py`, `control/mpc.py`,
  `control/solver_pi.py`, `control/solver_mpc.py`, `sim/plant.py`,
  `tests/test_gate.py`, `tests/test_zones.py`, `tests/test_mpc_*.py`
- DAS core: `control/estimates.py`, `control/estimator.py`,
  `control/associate.py`, `control/thermal.py`, `control/noise.py`,
  `control/solver_das.py`, `control/persist.py`, `control/ident.py`,
  `sim/das.py`, and their suites (§4.10)
- **DAS truth simulator** (`aqua_bridge.sim.das`), the reference plant for
  every DAS milestone and deliberately richer than any controller model:
  one air node per zone with inlet, leak, extra heat and symmetric
  inter-zone leakage; a latent drive node per bay with class nominals and
  activity-dependent heat (`heat_schedule` or seeded bursts), `h = g0 +
  k·Qn`; one lag node per sensor with placement offset, air fraction `β`
  for proximal sensors, quantisation per sensor type (thermistor 0.01,
  DS18B20 0.0625 °C), optional noise and dropout; SMART with an internal
  offset and lag, 1 °C quantisation and a seeded 30–60 s cadence, never
  exposing the bay; per output a dead band, airflow exponent, `count`
  fans (only fan 0 drives the tach), optional tach, stall, spin-up lag,
  fouling drift and resonance bands; inlet drift, swing and steps; hot
  swap through `bay_schedule` or `remove()` / `insert()`. Backward-Euler
  substeps conserve energy exactly in the discrete sense (tested).
  Presets `basic` (nominal, no noise, no drift) and `rich` (the unknown
  physical parameters drawn from the seed). Deterministic per seed.
  `run_das_closed_loop(plant, cfg, controller, ticks, ...)` returns the
  truth time series (`worst_margin_c()`, `violations()`), and
  `topology_from_config(cfg)` maps a zoned config onto the simulator.
- RC thermal plant (`aqua_bridge.sim.plant`), legacy mode only: lumped
  coolant and case air, radiator fans move heat coolant → air, intake
  fans air → ambient; actuator delay, stalled channels and Gaussian
  sensor noise are parameters; deterministic for a seed. It exercises the
  safety core in legacy mode and backs the legacy goldens.
  `run_closed_loop(..., observe_hook=...)` injects lies.
- Every solver lives behind the same contract. `config.example-das.yaml`
  runs `solver: pi` (PI-like DAS form); `mpc` becomes the default only
  after the model ladder (§13) on the real enclosure.
- **Zero W budget.** Legacy mode, measured with `tools/bench_step.py`
  (600 closed-loop ticks, example config, dt 2 s) on a Zero W with Python
  3.13.5 and numpy 2.2.4: PI step p99 about 8.9 ms, MPC step p99 about
  34.5 ms (the first MPC step builds its matrices, about 55 ms); both
  under 2 % of `dt`. DAS mode is not yet measured on the Pi: the budget
  table above extrapolates the development machine by 100× (DAS MPC
  ~350 ms on a solve tick, PI-like DAS ~140 ms, against the 600 ms gate at
  `dt = 5 s`); the hardware validation replaces those numbers
  (`tools/bench_step.py --sim-plant das` on the Pi). numpy only, no
  CasADi/IPOPT, no scipy; acados after 2W.

### Track B — hardware (Pi for USB; mapping on any machine)

- `hw/map.py`: `HwmonMap(hwmon_name, pwm_map, temp_map, fan_map={},
  root="/sys/class/hwmon")` resolves logical names to sysfs files. The
  device is found by reading every `hwmon*/name`, **on every call**:
  `hwmonN` numbers are not stable across re-plugs. `pwm_map` values are
  bare `pwmN` (the read/write file, with a `pwmN_enable` sibling);
  `temp_map` / `fan_map` values are bare `tempN` / `fanN` and resolve to
  the `*_input` file. Two logical names on one attribute are rejected,
  and every `fan_map` key must be a `pwm_map` channel (RPM is consumed
  per channel).
  `resolve(check_files=True)` raises for a missing attribute (startup
  check); the hot path resolves with `check_files=False`.
- `hw/xt6.py`: `Xt6Adapter(hwmon_map, clock)` with
  `read() -> PlantObservation` and `apply(MpcCommand)`. `ts` comes from
  the injected monotonic clock.
  - Assumed ABI (to be confirmed by the spike): `tempK_input`
    millidegrees °C, `fanK_input` RPM, `pwmK` 0..255 read/write,
    optional `pwmK_enable` where `1` is manual.
  - `read`: a single missing, unreadable or garbage file becomes `None`
    for that value; a vanished device directory raises
    `DeviceUnavailable`.
  - `apply`: every mapped channel must have a finite value in `[0, 1]`
    (else `ValueError`, nothing written; no silent clamping); writes
    `round(pwm * 255)`. Before every write it re-reads each
    `pwmK_enable` and writes `1` where it is not `1`, remembering the
    first value seen: a USB dropout and re-plug resets `pwmK_enable` to
    firmware control, and a write into a firmware-controlled channel
    would look like success. A failed write raises `DeviceUnavailable`.
  - `release()` restores the remembered `pwmK_enable` values (§2; not
    called at exit).
- `build_map_from_config(xt6, channels=mpc.channels, temps=mpc.temps)`:
  `xt6.hwmon_name` is required. Each fan is **one** `xt6.fans` entry
  that names the channel once and carries both attributes:

  ```yaml
  xt6:
    hwmon_name: aquaero
    fans:
      radiator: {pwm: pwm1, rpm: fan1}
      intake:   {pwm: pwm2}          # rpm optional
    temp_map:
      coolant: temp1
  ```

  A PWM output and its tachometer therefore cannot drift apart into two
  differently spelt channels. Only `pwm` and `rpm` are allowed inside an
  entry (a typo such as `rmp:` is rejected); `pwm` must look like `pwmN`,
  `rpm` like `fanN`, `temp_map` values like `tempN`. The former
  `xt6.map` / `xt6.fan_map` keys are rejected with a pointer to
  `xt6.fans`. `xt6.fans` keys must **equal** `mpc.channels` and
  `xt6.temp_map` keys must **equal** `mpc.temps`, or
  the daemon exits with a `ConfigError` (code 2) before the loop starts.
  Without that check a channel missing from the map is silently never
  written, and a temperature missing from or extra in `temp_map` keeps
  the gate in permanent fallback with no visible error. Optional
  `xt6.root` (default `/sys/class/hwmon`). `--source xt6` is this single
  device, unchanged for legacy configs.
- `hw/sources.py` (`--source hwmon`, the DAS source): `CompositeSource`
  merges any number of hwmon devices and an optional 1-Wire source into
  one `PlantObservation` and puts the SMART inbox's snapshot into
  `obs.inputs["smart"]`; `apply()` writes the whole command to every
  device, each `Xt6Adapter` picking out its own channels.
  `build_composite_from_config(hwmon_section=, xt6_section=, onewire_section=, channels=, temps=, dt=, smart=)`:

  ```yaml
  hwmon:                      # a list, one entry per hwmon device
    - name: aquaero           # alias of hwmon_name
      fans: {xt1: {pwm: pwm1, rpm: fan1}, ...}
      temp_map: {inlet_a: temp1, air_z0: temp2, ...}
    - name: quadro            # e.g. on its own USB port
      fans: {qd1: {pwm: pwm1, rpm: fan1}, ...}
      temp_map: {}            # bind Quadro inputs once they appear in hwmon
  onewire:
    sensors: {prox_b01: 28-0316a27a0aff, ...}   # logical name -> ROM id
    resolution_bits: 12       # 9..12
    max_age_s: 7.5            # default 1.5 * dt
    root: /sys/bus/w1/devices # default
  ```

  An `xt6:` section is still accepted as one more device. Every
  `mpc.temps` name must be bound exactly once across all `temp_map`s and
  `onewire.sensors`, every channel exactly once across all `fans` maps,
  or the daemon exits 2 before the loop starts. A failing device's
  exception propagates from `read()` / `apply()` (the loop's existing
  partial-write and blank-observation policies apply). A ROM id missing
  from every bus at start is a warning, not fatal: a sensor may be
  unplugged with its drive.
- `hw/onewire.py`: `W1Source`, DS18B20 over the kernel's `w1_therm`
  bulk-read ABI (assumed layout, unverified on hardware, kept name-based
  and rooted at `onewire.root`): `w1_bus_master<N>/therm_bulk_read`
  (write `trigger`, poll until `1`) and `w1_bus_master<N>/<rom>/temperature`
  (millidegrees) / `resolution` (written once per sensor). One daemon
  reader thread per discovered bus master runs trigger → poll → read
  every present slave and publishes into a lock-protected latest-sample
  dict. `read()` on the loop thread never blocks or touches the
  filesystem: a CRC failure (EIO or garbage), a stalled bulk read, or a
  sample older than `max_age_s` is `None` for that sensor, never an
  exception. With 22 DS18B20 on two buses a cycle is about 1 s (750 ms
  conversion at 12 bit plus ~12 ms of kernel bit-banging per sensor), well
  inside `dt = 5 s`; at `dt = 2 s` use 11 bit. `onewire.buses` is
  documentation only: bus masters are discovered, the overlays are in
  `config.txt` (§9).
- `publishers/inputs.py`: `SmartInbox`, the Pi side of the SMART path.
  Fed by MQTT `{node_id}/in/smart/<serial>` (on the existing broker
  connection, `MqttClient.add_topic_handler`) and by `POST /api/in/smart`;
  stores the latest `{temp_c, model}` per serial with the Pi's own
  monotonic receipt time (never the agent's `ts_wall`); `snapshot()` →
  `{serial: {temp_c, age_s, model}}` for entries within 300 s. A
  malformed payload only increments `rejected`; it never raises and never
  faults a zone. `hw/` reads it through a duck-typed protocol and never
  imports `publishers`.
- `tools/smart_agent.py` (runs on the PC, stdlib + `smartctl`, publishes
  with paho-mqtt): discovers SATA/SAS by-id devices (deduplicated,
  partitions skipped) and NVMe namespaces, reads `smartctl -j -n standby
  -A` (SATA/SAS: never spins up a sleeping drive) or `smartctl -j -A`
  (NVMe) every `--interval` seconds, publishes retained
  `{serial, model, temp_c, ts_wall}`. `deploy/aqua-bridge-smart-agent.service`
  is a systemd user unit example for the PC.
- `tools/w1_commission.py` (Pi, before the daemon): `--list` (every ROM
  id per bus with its reading), `--identify` (ranks sensors by warming
  rate while you warm one with a finger), `--check --config PATH` (builds
  the exact composite the daemon would, then reports the bulk-read cycle
  time per bus and the CRC error rate per sensor over `--cycles`).
- Unit tests against a **fake hwmon tree** and a **fake w1 tree** in a temp
  directory (CI, no Pi). `pytest.mark.hardware` only for the live device;
  `tests/conftest.py` skips those tests when no hwmon device named
  `aquaero` exists.
- **Does not import control** (`hw/*.py`, checked statically).

### Glue (`control/loop.py`, `control/supervisor.py`, `__main__.py`)

```text
plan = supervisor.plan_tick()                 # effective config, control mode, overrides, released
cmd, state = mpc.step(obs, plan.cfg, state)
cmd = supervisor.compose(cmd, plan, prev)     # overrides (human, experiment), except on channels under fallback policy
sink.apply(cmd)                               # state.last_cmd := the command actually applied
notifier.watchdog()
```

The `Loop` talks to a `Source` (`read()`), a `Sink` (`apply()`) and a
`Notifier`; it knows nothing about sysfs, HTTP or MQTT. Policies:

- **`read()` raises or returns a non-observation:** the tick is not
  skipped. A blank observation (no temps) with `ts = last obs.ts + dt`
  goes through `step`, the gate rejects it, and the hold / ramp-high
  command **is applied** (rate-limited against the last applied PWM).
  That is the software watchdog of §2; `WATCHDOG=1` is still sent
  because the loop itself is healthy.
- **`apply()` raises:** logged; the new state is committed (fault
  timers, window, integrator advance), but `last_cmd` and the newest
  `WindowSample.cmd_pwm` are rewritten to the last command that actually
  reached the fans (`None` if none ever did), so the next rate limit is
  measured from what is on the fans. A failed write is assumed not to
  have happened.
- **`step` or `compose` raises** (a controller bug; `step` promises not
  to): state untouched, an emergency command ramps every channel toward
  `max(last applied, fallback_pwm)` at `d_pwm_max`, and `WATCHDOG=1` is
  **withheld** so systemd restarts the process after `WatchdogSec`.
- **Applied-command feedback:** after every tick `state.last_cmd` and
  the newest window sample mirror the applied command (overrides,
  failures). Channels released from a manual override have their
  integrator entry dropped for one tick, so `step` re-initialises
  bumplessly from what is on the fan.
- **Timing:** ticks every `dt` of monotonic time; an overrun restarts the
  schedule from “now” instead of bursting.
- **`READY=1`** once, after the first tick whose command was applied.
- **`on_tick` hook:** called on the loop thread with every `TickResult`
  after the supervisor snapshot is updated; its exceptions are logged and
  dropped. `recorder.chain_on_tick` composes several observers, each
  isolated: the MQTT state publisher, the tick `Recorder` and the model
  store's `ModelPersister`.
- **Experiments:** `supervisor.record_tick(..., ts=obs.ts)` after every
  tick advances the experiment's settle tracker and a running
  experiment; a tick whose applied command is the loop's emergency
  fallback aborts it.
- **`shutdown()`:** writes `fallback_pwm` once (`mode=fallback`,
  deliberately not rate-limited, §9), then `STOPPING=1`; survives a sink
  failure.

The `Supervisor` is the `ControlSurface` (§6) that HTTP, MQTT and, later,
Digole use: it owns control mode, overrides, setpoints, preset, runtime
drive limits (`RuntimeLimits`), runtime bay declarations
(`RuntimeBays`) and the identification experiment; it provides the
effective config for each tick and composes overrides into the solver
command. Digole and MQTT subscribe to `(obs, cmd)` through it, not to
HID. In DAS mode the estimator, zone trust and the thermal model run
inside `step`, so the loop and the supervisor stay the same for both
modes.

`python -m aqua_bridge` (`__main__.py`): `--config PATH` (required),
`--source xt6|hwmon|sim` (default `xt6`: one aquaero; `hwmon`: the DAS
composite of `hwmon:` devices and `onewire:`; `sim`: a simulated plant
with names taken from the config), `--sim-plant basic|rich|das` (sim
only: the RC plant, the RC plant with one tick of actuator delay and
0.02 °C noise, or the DAS truth plant built from `sim.das.topology` or
else the zoned `mpc` topology), `--once` (one tick, print obs/cmd JSON),
`--ticks N`, `--sim-speed X` (sim only; 1 real time, 0 as fast as
possible), `--record PATH` (JSONL tick recording; overrides
`record_path`), `--model-store PATH` (DAS only; default
`$STATE_DIRECTORY/model.json`), `--log-level`. Every run that started
the loop, `--once` and `--ticks` included, ends with the `fallback_pwm`
stop write (and a final model-store write in DAS mode). Exit codes: `0`
clean stop, `1` loop crashed, `2` bad arguments or config (including the
xt6 map and composite binding checks, a DAS sim without topology,
`--model-store` with a legacy config), `3` source/sink could not be
built. HTTP and MQTT start next to the loop when enabled (§6, §7); a
publisher that fails to start is logged and the daemon keeps controlling
the fans. One `SmartInbox` is built per run and wired into
`--source hwmon`, `POST /api/in/smart` and the MQTT `in/smart` topic.

**Tick recorder** (`recorder.py`, `--record PATH` or top-level
`record_path`, rotation by `record_max_bytes` / `record_backup_count`):
one JSON line per tick with the raw readings, `prev`, the command, the
gate-trusted temperatures, the trusted zones and the estimator's bays,
exactly what `thermal.update` needs to replay the tick. Never raises,
never blocks beyond a local append.

### Repo layout

```text
aqua-bridge/
  PROJECT.md                 # this file
  README.md
  pyproject.toml             # deps + extras http/mqtt/dev, pytest markers, ruff
  config.example.yaml        # legacy mode (no topology): placeholder coolant/air names
  config.example-das.yaml    # DAS mode: 4 zones, 15 bays, 8 outputs, 25 sensors
  .github/workflows/ci.yml   # lint, test (latest, pi-parity), nightly-fuzz (§12)
  deploy/
    packages-rpi.txt         # apt packages (§9)
    aqua-bridge.service      # systemd unit, StateDirectory=aqua-bridge (§9)
    aqua-bridge-das.conf     # drop-in: ExecStart= --source hwmon (§9, §10, install-pi.sh --das)
    aqua-bridge-smart-agent.service  # systemd *user* unit example for the PC
    99-aquacomputer.rules    # udev: usb, hidraw, hwmon pwm group write (§9)
    host-usb.sh              # dwc2 host overlay, idempotent (§10)
    install-pi.sh            # provisioning, self-signed HTTPS certificate, --das (§10)
  src/aqua_bridge/
    __main__.py              # python -m aqua_bridge: wiring, signals, exit codes
    model.py                 # the contract, incl. the DAS config sections
    config.py                # YAML -> AppConfig
    modelstore.py            # model.json: load, fingerprint, atomic write, ModelPersister
    recorder.py              # JSONL tick recorder (on_tick observer)
    control/gate.py          # sensor gate, per-sensor Stuck sizing, decimated windows
    control/zones.py         # zone trust, fault closure, per-channel fallback policy
    control/mpc.py           # step(): prev, time, gate, zones, estimator, faults, solver, fallback, rate limit, mode
    control/solver_pi.py     # Solver protocol + PI (legacy and margin-deficit DAS form)
    control/solver_mpc.py    # small linear MPC (numpy), solve_box_qp
    control/estimates.py     # per-bay estimates block, prior map
    control/estimator.py     # per-zone Kalman filter, occupancy, SMART calibration
    control/associate.py     # serial -> bay association by correlation
    control/thermal.py       # zoned thermal model, linearisation, online identification
    control/noise.py         # fan noise index and cost surrogate
    control/solver_das.py    # DAS MPC: active-piece SQP, bands, validity gate, bumpless
    control/persist.py       # apply a stored model to the controller memory (pure)
    control/ident.py         # identification experiments on fan groups (pure)
    control/intents.py       # intents, ControlSurface, ControlSnapshot, payloads
    control/supervisor.py    # control mode, overrides, setpoints, limits, bays, presets, experiments, compose
    control/loop.py          # read -> step -> compose -> apply -> watchdog
    hw/map.py                # logical name -> hwmon sysfs file
    hw/xt6.py                # aquaero adapter
    hw/sources.py            # composite of several hwmon devices + 1-Wire + SMART inputs
    hw/onewire.py            # DS18B20 w1_therm bulk-read reader threads
    sim/plant.py             # legacy RC plant (inside the package so `pip install -e .` sees it)
    sim/das.py               # DAS truth plant, run_das_closed_loop
    sdnotify.py              # stdlib sd_notify
    hostinfo.py              # CPU temp, load, RAM, disk, Wi-Fi RSSI, uptime
    publishers/http.py       # REST + HTML over HTTPS, auth middleware, same intents as Digole
    publishers/httpauth.py   # http: settings, credentials file, PBKDF2, TLS context, authenticator
    publishers/static/index.html
    publishers/mqtt_ha.py    # topics, Discovery, command parsing, paho wrapper
    publishers/inputs.py     # SmartInbox (MQTT in/smart, POST /api/in/smart)
    publishers/runtime.py    # HTTP / MQTT threads next to the loop
    ui/digole/               # planned, after Command is stable
  tools/
    bench_step.py            # step() timing per solver: RC plant or --sim-plant das
    w1_commission.py         # --list, --identify, --check (Pi)
    smart_agent.py           # SMART over MQTT (PC)
    fit_model.py             # offline zoned model fit from recordings
    fit_fans.py              # PWM -> RPM curve per fan model
    replay.py                # replay recordings through the thermal model
    http_user.py             # create or update an HTTPS API user (Pi, as root)
  tests/
    conftest.py              # Hypothesis profiles, fixtures, SolverCase, hardware auto-skip
    invariants.py            # §4.1 helpers, incl. per-zone checks
    das_fixtures.py          # small zoned config and observations
    http_fixtures.py         # test user, authenticator, openssl certificate
    golden/                  # <scenario>.<solver>.json: legacy pi/mpc, DAS pi_das/mpc_das
    test_model_config.py
    test_gate.py
    test_mpc_invariants.py
    test_mpc_nominal.py
    test_mpc_failures.py
    test_mpc_sensor_faults.py
    test_mpc_fuzzy.py
    test_mpc_closedloop.py
    test_mpc_solver.py
    test_zones.py            # DAS contract, zone trust, closure, modes, compose
    test_mpc_zone_fallback.py
    test_das_core.py         # core invariants and goldens on the DAS config
    test_das_config.py
    test_das_intents.py      # limits, bays, DAS views, HA drive entities
    test_estimates.py
    test_estimator.py
    test_associate.py
    test_hotswap.py
    test_pi_das.py
    test_thermal_model.py
    test_thermal_ident.py    # identifiability on sim/das (sweeps: nightly)
    test_solver_das.py
    test_noise_regression.py # MPC noise vs the quietest uniform curve (sweeps: nightly)
    test_bench_budget.py     # relative step budget; absolute budget on the Pi
    test_modelstore.py
    test_ident_experiment.py
    test_recorder.py
    test_tools_fit_replay.py
    test_sim_das.py
    test_supervisor.py
    test_loop.py
    test_main.py
    test_sdnotify.py
    test_http_api.py
    test_http_auth.py        # HTTPS, basic auth, credentials file, http_user.py
    test_mqtt_ha.py
    test_inputs_smart.py
    test_smart_agent.py
    test_hostinfo.py
    test_publishers_runtime.py
    test_hw_map.py           # fake sysfs, CI; static no-control-import check
    test_hw_xt6.py           # fake hwmon in CI; live device with pytest.mark.hardware
    test_hw_sources.py
    test_hw_onewire.py       # fake w1 tree
    test_w1_commission.py
    test_deploy.py           # units, udev rules, install script, shellcheck
```

CI on GitHub runs every test except `hardware` and `nightly` on every PR
and push (§12), `fuzzy` and `slow` included; the nightly job adds the
`nightly` sweeps. Hypothesis example counts are capped per profile, not
unbounded. Live device tests run only on the Pi. Fake-hwmon tests run
everywhere.

---

## 4. Testing

MPC is the primary function of the board. A green USB spike does **not**
replace this. The suites below exist and CI runs them on every PR and
every push to `main` (§12); they must stay that way.

`mpc.step(obs, config, state)` is a pure function: tests never open
sockets, files, or HID. Time comes from `observation.ts` and `config.dt`.
Pass `state` in; take `state` out.

**Every solver.** The core suites must survive a solver swap (§8). The
`solver_kind` argument is parametrised by a `pytest_generate_tests` hook
in `tests/conftest.py` over `SolverCase`: `pi` and `mpc` (the legacy
config, `config.example.yaml`) and `pi_das` and `mpc_das` (the DAS
config, `config.example-das.yaml`, with the PI-like DAS form and the DAS
MPC; the DAS suites run the MPC with `model_accept_prior: true` so its
MPC path acts rather than its fallback). The marker
`@pytest.mark.solver_cases(...)` restricts the cases. The legacy core
suites `test_mpc_invariants.py`, `test_mpc_nominal.py`,
`test_mpc_failures.py`, `test_mpc_sensor_faults.py` and
`test_mpc_closedloop.py` are written against the coolant example and run
`pi` and `mpc`; `test_das_core.py` runs the same kinds of invariants,
closed loops and goldens for `pi_das` and `mpc_das`; `test_mpc_fuzzy.py`
parametrises the same way. `test_mpc_solver.py` holds the legacy
MPC-only tests (box QP against a projected-gradient reference, iteration
cap, warm start, model matrices, caching, bumpless initialisation
including prev outside the box and the knee rule, offset-free tracking
with a wrong model, pressure at the rail, config rules, horizon
extremes); `test_solver_das.py` the DAS MPC's (§4.10).

**Fixtures** (`tests/conftest.py`): `cfg` (the legacy example config),
`fast_cfg` (`dt=1`, `confirm_ticks=2`, `fallback_hold_s=4`,
`stuck_ticks=4`, PWM limits unchanged), `das_example_cfg` (the DAS example
config), `solver_kind`, `example_config_path`, `example_das_config_path`,
`aquaero_hwmon` (skips when absent). Gate suites use a `gcfg` fixture
parametrised over `median3`. `tests/das_fixtures.py` builds a small zoned
config (`das_mapping`, `das_cfg`) and observations (`das_obs`,
`default_temps`) for the zone, gate and fallback suites.

**Markers** (`--strict-markers`): `hardware` (live aquaero; auto-skipped
when no hwmon device named `aquaero` exists), `fuzzy` (Hypothesis), `slow`
(long closed-loop runs, subprocess SIGTERM), `nightly` (heavy sweeps and
long simulations: excluded from PR and `main` runs, run by the nightly
job), `solver_cases(*cases)` (restricts `solver_kind`), `pi` (only on the
Raspberry Pi, e.g. the absolute step budget; skipped elsewhere). PR and
`main` CI run `-m "not hardware and not nightly"`; the nightly job runs
`-m "not hardware"`.

**Hypothesis profiles** (`HYPOTHESIS_PROFILE`, default `dev`):

| Profile | Examples | Randomized | Used by |
|---------|----------|------------|---------|
| `dev` | 50 | yes | local runs |
| `ci` | 200 | no (`derandomize`) | PR and `main` CI runs: a red check is reproducible |
| `nightly` | 1000 | yes | scheduled `nightly-fuzz` job; failures print a `@reproduce_failure` blob |
| `pi` | 20 | yes | runs on the Pi |

A synthetic test whose temperatures are exactly constant for longer than
`stuck_ticks` while the PWM moves by more than `stuck_pwm_net` trips the
Stuck rule by design. Tests that mean something else (fan stall, manual
override ramps) wobble the temperatures by more than `stuck_eps_c`.

### 4.1 Invariants (assert on every `step` and every closed-loop tick)

These are the contract. Nominal, failure, lie, and fuzzy tests all check
them. They live in `tests/invariants.py`:
`assert_command_safe(obs, cfg, cmd, prev_pwm)` (`prev_pwm` a dict or the
previous `MpcCommand`), `assert_state_finite(state)`, and
`checked_step(obs, cfg, state)`, which wraps `step` in both plus the
multi-value checks below. Helpers: `resolve_prev_pwm`, `obs_pwm_trusted`
(mirrors `mpc.obs_pwm_usable`), `obs_structurally_untrusted`, `make_obs`.

- `cmd.pwm` has **exactly** `config.channels` — no extras, no missing keys
- every PWM is finite and in `[pwm_min, pwm_max]`
- per channel `|pwm_k - prev_pwm_k| <= d_pwm_max`. `prev` is, in order:
  `state.last_cmd`; else usable `obs.pwm` (§3: every channel finite and in
  `[max(0, pwm_min - d_pwm_max), min(1, pwm_max + d_pwm_max)]`); else
  **`config.fallback_pwm`** (§3 item 4). **Never** `pwm_min`.
- no `NaN` / `Inf` in PWM, diagnostics, or returned state; the state
  round-trips through JSON
- `cmd.mode` is one of `auto | saturated | fallback | degraded`
  (`degraded` never occurs in legacy mode)
- `fault_since_ts` and `fault_reason` are set together or not at all
- `mode` is `fallback` or `degraded` exactly when the returned state is in
  fault (legacy: `fallback`); the returned state carries the returned
  command as `last_cmd`; the input state is not mutated
- an observation the gate must reject without history (missing / `None`
  / NaN temperature, unknown key, or out of range when `median3` is off)
  → `mode=fallback`. With `median3` on, a finite out-of-range value is
  not structural: the median removes a single Spike.
- untrusted observation → `mode=fallback` and the hold-then-high policy
  in §3 (never a step toward `pwm_min` *because* of the fault)
- same `(obs, config, state)` → same `(cmd, state)` (deterministic)

On a zoned config `checked_step` also runs `assert_zone_step_safe`:

- `zone_faults` holds exactly the zones; `fallback` iff every zone is in
  fault, `degraded` iff some are, `auto` / `saturated` otherwise;
  `diagnostics.zones_in_fault` lists them
- the global fields are the aggregates: `fault_since_ts` the earliest
  zone fault, `trusted_streak` the smallest zone streak
- the channels under fallback policy (`diagnostics.fallback_channels`)
  are exactly the reach of the faulted zones (own channels plus declared
  coupling), none of them is commanded below `clip(prev)`, and none keeps
  an integrator entry
- an observation that must fault a zone without history (every member of
  one of its required groups missing, `None`, non-finite or, with
  `median3` off, out of range; or an unknown key, which faults every
  zone) faults that zone (`structurally_faulted_zones`, checked by
  `assert_command_safe`)

A test that produces a command which violates an invariant is a failure
even if the “story” of the test passed.

`tests/test_mpc_invariants.py` covers the first-tick `prev` rules, type
errors, determinism, JSON round-trip, documented diagnostics keys, and the
agreement between `obs_pwm_trusted` and `step`.

### 4.2 Nominal (ordinary operation)

`tests/test_mpc_nominal.py`, closed-loop via `aqua_bridge.sim.plant` where needed.

- **At setpoint:** temps already at target → PWM settles; ripple below a
  bound for N steps; open-loop output constant
- **Step up / step down:** setpoint change → PWM moves the right way;
  temp crosses toward the target within a deadline (sim)
- **Disturbance:** extra heat in the plant → PWM rises, then recovers
- **Cold start:** obs PWM 0, temps high → PWM ramps up monotonically, at
  most `d_pwm_max` per tick, to the rail. With `pwm_min <= d_pwm_max`
  the ramp starts from `obs.pwm = 0`. With `pwm_min > d_pwm_max` (the
  example: 0.15 > 0.1) no command is both in range and within
  `d_pwm_max` of 0, so `obs.pwm` is not usable, `prev = fallback_pwm`,
  and the loop starts at `fallback_pwm` (high cooling) and ramps from
  there. That is the chosen reading; the test runs both.
- **Warm start:** already at `pwm_max` (trusted `obs.pwm`, bumpless
  first tick), temps then fall → PWM ramps down; it must not rise while
  the coolant is below the setpoint and still falling (how far it
  undershoots before turning is the solver’s business)
- **Multi-channel:** two+ fans; one channel can increase while another
  holds; no silent drop of a channel. Channels that regulate the same
  temperature are not required to converge to the same duty: the split
  must narrow to less than `d_pwm_max`, no more.
- **Saturation is honest:** if the plant needs more than `pwm_max`,
  `mode=saturated`, PWM pinned at max, no exception, and the tail of the
  run is saturated on every tick (no chatter); `saturated` is never
  reported while PWM is below max
- **Golden trajectories:** dump `(t, temps, pwm)` for a fixed seed and
  plant; regression with a numeric tolerance (not bitwise float). One
  file per scenario and solver under `tests/golden/`
  (`regulation_noise_disturbance`, `setpoint_steps`); regenerate only with
  `AQUA_BRIDGE_REGEN_GOLDEN=1`, never in CI, and only when a solver’s
  behaviour is meant to change. The legacy files
  (`regulation_noise_disturbance.{pi,mpc}.json`,
  `setpoint_steps.{pi,mpc}.json`) are the bit-identity check of legacy
  mode and never move.

These tests stay; swapping the default solver must not delete them.

**Nominal in DAS terms** (`tests/test_das_core.py`, `pi_das` and
`mpc_das`, closed loop on `sim/das.py` built from
`config.example-das.yaml`): the example config's layout and bindings;
cold first ticks (untrusted → exactly `fallback_pwm`, trusted → bumpless
from `obs.pwm`); determinism and JSON round trip; a zone fault never
lowers its reach while the rest regulates; saturation pinned at
`pwm_max` and reported; drives at their target hold the output; random
zone lies keep the invariants; scenarios keep every drive within its
limit; a hot-swap insert and an activity burst raise the fans of their
zone. Goldens `das_regulation.{pi_das,mpc_das}.json` and
`das_hotswap.{pi_das,mpc_das}.json` (480 ticks at `dt = 5` with each
sensor type's white noise) are new file names; they move only when a DAS
solver or the estimator is meant to change.

### 4.3 Failures (everything that is not a lying sensor)

`tests/test_mpc_failures.py` — one case per fault, plus combinations.

Observation / time:

- `obs.temps` keys ≠ `config.temps`: missing key, extra unknown key,
  empty `temps` → untrusted tick, never a crash
- `rpm` / `pwm` missing for a channel that exists in config → still
  steps; on a cold state `prev` falls through to `fallback_pwm`
- `ts` not advancing, `ts` going backwards, duplicate `step` with
  identical `ts` → untrusted tick, deterministic; a permanently stuck
  clock ramps high by tick count
- gap `> 3 * dt` (missed ticks) → untrusted tick, history dropped,
  recovers after `confirm_ticks`; a moderate gap scales the slew limit

Values:

- `None`, NaN, ±inf in a temperature → untrusted, stored as `None`
- out-of-range temps (outside `[temp_min_c, temp_max_c]`, default
  −20..120 °C, e.g. 32767, 65535, millidegrees 35000) as **invalid**,
  not as a real plant
- PWM obs outside `[0, 1]` or NaN → not usable as `prev`, never a crash
- RPM = 0 for many steps while commanded PWM is high (stall / unplugged
  fan) → `fan_stall` flag after 10 ticks, integral capped at `pwm_max`,
  no mode change (§3); a low command does not count

Config / solver:

- `horizon = 1` and a large horizon (200) → invariants hold, MPC stays
  under the iteration cap
- every rejection rule of the `MpcConfig` table in §3 → `ConfigError`,
  do not step (`tests/test_model_config.py` and here)
- solver exception, NaN demand, wrong keys, non-finite integrator,
  memory that is not finite JSON, not a `SolverResult`, `converged=False`,
  `iterations > solver_max_iter`, no registered solver → same
  hold/ramp-high as the sensor gate (`fault_reason=solver`), **no throw**
  out of `step`; one timer across retries; recovery after
  `confirm_ticks`
- cold `state` (`None` last_cmd) on first call with untrusted obs: command
  **equals** `fallback_pwm` when `obs.pwm` is not usable, holds `obs.pwm`
  when it is (§3 items 4–5)
- a fault that starts above `fallback_pwm` (at `pwm_max`, or between)
  never lowers the fans; the ramp target is `max(prev, fallback_pwm)` per
  channel

Loop / glue (`tests/test_loop.py`, still no HID):

- `read()` raises, returns a non-observation or an empty observation → a
  fallback tick through the gate; the command is applied and the
  watchdog kicked; recovery needs `confirm_ticks`
- `apply()` raises → next tick still runs; the rate limit continues from
  the last applied command; a first-tick failure leaves `last_cmd`
  `None`
- `step` / `compose` raise → emergency ramp, state untouched, no
  `WATCHDOG=1`
- step budget alarm: `step()` wall time (a monotonic clock outside
  `step`, which stays pure) past `mpc.budget_ms` logs a warning, past
  `mpc.budget_alarm_ms` an error instead, each rate limited to one line per
  `mpc.budget_log_interval_s` naming the exceedances since the last line;
  `step_ms_last`, `step_ms_max` and both cumulative counters reach
  `/api/health` and the MQTT state blob (an injected clock drives the test)

### 4.4 Lying sensors (fault injection)

`tests/test_mpc_sensor_faults.py`

Temperature sensors lie. The controller must **not** chase a lie to
`pwm_max` in a few ticks (`d_pwm_max` is necessary but not sufficient).

Inject on top of an otherwise nominal closed-loop:

| Lie | Example | Expected |
|-----|---------|----------|
| Stuck | frozen for `stuck_s` while **net** PWM moved > `stuck_pwm_net` (a quarter window old, §3) or a sibling T moved > `stuck_sibling_dT_c` | untrusted; do not treat as at-setpoint; latched while the value stays in the band. Either evidence may fire first (with the MPC at an exact equilibrium the sibling rule can flag the calm sensor on the first tick of the lie). **Negative case:** frozen T at equilibrium, PWM dithering a few thousandths per tick, net ≈ 0 → stays trusted |
| Spike | single sample −40 °C or +125 °C | `median3=false`: exactly one untrusted tick, last good unchanged; `median3=true`: zero untrusted ticks; rate-limit PWM either way |
| Jump | step +30 °C and stay | first tick untrusted; next ticks trusted vs previous raw; the `confirm_ticks`-th trusted tick is auto and the whole `last_good_obs` := obs; with `median3=true` everything shifts one tick later |
| Drift | slow bias +0.05 °C/step | indistinguishable from heating, so it is followed, never faster than `d_pwm_max`. Acceptable lag: PWM off the rail while the accumulated bias is below 1 °C; integrator inside `[pwm_min, pwm_max]` throughout; honest `saturated` once railed; never fallback |
| Swap | coolant/air keys exchanged | onset disagrees with history → fallback, hold, never toward min. A persisting swap confirms like a Jump after `confirm_ticks`, before the hold expires, so the ramp is never reached (§3) |
| Impossible combo | coolant 18 °C, “air” 90 °C, PWM 1.0 | untrusted at onset (slew), PWM held, not chased; recovers when the lie ends. No cross-sensor rule: a persisting combination confirms like a Jump (§3) |
| Impossible dT/dt | +15 °C in one `dt` | reject sample (far from both references) |
| Dropout | `None` / missing for k steps (short, and past the hold), then back | no NaN; fallback for k + `confirm_ticks` − 1 ticks; resume without a PWM spike |
| Raw garbage | 32767 / 0xFFFF leftovers, millidegree passed as °C | out of range → untrusted. With `median3=true` a single sample is filtered, so the test injects 2–3 consecutive samples and the first untrusted tick shifts by one |
| Flicker | alternate good/bad every step (spike, missing, `None`) | `mode` stays `fallback` (streak never reaches `confirm_ticks`); PWM must not chatter at `d_pwm_max` each tick; hold timer must not reset |
| Constant noise | Gaussian **σ = 0.2 °C** (aquaero-scale, not ±2 °C) | gate reject rate under 1 % over a long run; `mode` stays auto; no PWM chatter. A companion test shows σ = 2 °C would trip the slew gate constantly. Median-of-3 is extra filtering, not a substitute for a realistic σ |
| DS18B20 plateau (DAS) | a proximal DS18B20 on one 1/16 °C code for minutes while its zone's fans move | stays trusted: its window is `stuck_s` 1800 s decimated, its band `1.5 × 0.0625 °C`, its evidence only its zone's channels and same-role siblings (`tests/test_gate.py`) |
| Frozen sensor (DAS) | a sensor frozen for its whole (decimated) window while its zone's channels moved net | Stuck, **only its zone** faults (hold, then high on its reach), other zones keep regulating (`degraded`); a frozen value hidden behind median3 glitches is still flagged (`tests/test_gate.py`) |
| Lie in one zone (DAS) | any row above on a sensor of one zone | only that zone (and its declared neighbours' channels) under fallback policy; a Flicker there never resets another zone's streak; a dropout inside a redundant group is no fault (`tests/test_mpc_zone_fallback.py`) |
| Swapped proximal sensors (DAS) | ROM ids of two bays exchanged | a Jump on both at onset (their zones hold), confirmed like any Jump; caught at commissioning, not by the gate (§3) |

The **sensor gate** is explicit (`control/gate.py`) and uses the
trusted-tick rule of §3. Tests target that gate (`tests/test_gate.py`,
and the gate-level cases here) **and** the full `step`. They include:
Jump must not stay in fallback forever; Stuck must flag when **net** PWM
or a sibling sensor moves, and must **not** flag at equilibrium with
small PWM dithering or on a sibling Spike / Jump; noise σ = 0.2 °C must
not flicker `mode`; every gate test runs with `median3` both `false` and
`true`. Do not hide filtering only inside numpy and leave it untested.

### 4.5 Fuzzy / property tests

`tests/test_mpc_fuzzy.py` — Hypothesis, marked `pytest.mark.fuzzy`, every
solver case. The DAS suites add their own property tests: random zone
lies (`test_das_core.py`, `test_zones.py`), random measurement sequences
keeping the estimator's covariances symmetric and PSD
(`test_estimator.py`), random convex penalty problems against a
projected-gradient reference (`test_solver_das.py`), malformed memories
and seeds that never raise out of `step`.

Properties (N random examples per test, shrinking on failure):

- **Random valid obs** in plausible boxes (temps 10–80 °C, pwm 0–1,
  rpm 0–4000, `ts` increasing by `dt`) → invariants hold
- **Random sequences** of 20–100 steps, each obs a small perturbation of
  the last → invariants every tick, deterministic, never a fault; PWM
  total variation at most
  `(n − 1) * d_pwm_max`. That is the only bound that holds for any
  solver; a tighter PI-specific bound would not survive the solver swap.
- **Random lies mixed in:** with probability p (and random `median3`),
  apply a lie from §4.4 → still invariants; `mode=fallback` whenever the
  gate says untrusted; an untrusted tick never lowers a channel below
  `prev` except through the clamp
- **Random setpoints** in a sane band (25–45 °C), plant held still →
  compared tick to tick, a loop more than 0.5 °C hot never lowers PWM
  and a loop more than 0.5 °C cold never raises it, from a rail too
- **Random plant parameters** in closed-loop (time constants, gain) →
  no NaN, no PWM outside limits (stability/overshoot asserts may be
  looser than goldens)
- **Malformed Observation construction:** extra keys, wrong types —
  either reject (TypeError/ValueError) or fallback; never a silent
  bad PWM; arbitrary temperature dicts never crash `step`

Other property tests: random sequences with lies in
`test_mpc_invariants.py`, `compose` bounds and rate for any override in
`test_supervisor.py`, and the HTTP fuzz below. Cap `max_examples` per
profile (§4) so CI is minutes, not hours. Failures must shrink to a
replayable example; every profile prints the reproduction blob.

HTTP: fuzz JSON bodies for `/api/pwm` and `/api/setpoint` (missing
fields, strings, arrays, out of range) → 4xx, never a 500 and never an
intent that reaches the surface with a value outside the configured
range.

### 4.6 Closed-loop sim

`tests/test_mpc_closedloop.py` + `aqua_bridge.sim.plant`

- Controller model **≠** plant (fast/slow loop, weak/strong fans, high
  gain) — still bounded PWM and no crash
- Actuator delay (commanded PWM appears in `obs.pwm` 1–2 ticks late)
- Fan stall: plant RPM 0 regardless of PWM, from the start and mid-run —
  flagged, bounded, no windup
- Long run (15 min of sim at `dt=2`) — no drift to NaN; a long run at the
  rail does not leak growth
- Bumpless: long fallback then return to auto — first auto PWM equals
  `last_cmd`, integrator (PI) / disturbance estimate (MPC) re-initialised,
  no jump afterwards (here and in `test_mpc_failures.py`)

### 4.7 Hardware-adapter tests (track B)

**No control import.** Mapping and the adapter must not wait for the board.

- `tests/test_hw_map.py` — logical names ↔ a fake hwmon directory tree
  (temp files): device found by name not number, renumbering after a
  re-plug, missing device / root / attribute, duplicate targets, default
  root, and the static check that `hw/` never imports `control`. Runs in
  CI.
- `tests/test_hw_xt6.py` — same fake tree for `read`/`apply` (unit
  conversion, garbage and missing files become `None`, device gone,
  rounding, `pwmK_enable` set before the first write and re-checked on
  every apply, re-plug that resets it, `release`, rejected NaN /
  out-of-range / missing channel) and `build_map_from_config` key checks.
  Live device is `pytest.mark.hardware` (`test_live_read_and_writeback`:
  read once, write back the PWM already in effect, `release()`) and
  skipped when no aquaero hwmon device exists.
- `tests/test_hw_sources.py` — `CompositeSource` over two fake hwmon
  devices and a fake 1-Wire source: merged reads, each channel written to
  its own device, a failing device propagates, `xt6:` plus a `hwmon:` list,
  every name bound exactly once (exit 2 otherwise), a missing ROM at start
  does not block, `inputs["smart"]` only with an inbox.
- `tests/test_hw_onewire.py` — a fake `w1_bus_master*` tree: discovery,
  the trigger/poll/read cycle, sensors of another bus ignored, CRC
  failure and garbage → `None` and counted, a bulk read that never
  completes or a failing trigger → `None` without raising, resolution
  written once, stale samples → `None`, missing ROMs.
- `tests/test_w1_commission.py` — `--list`, `--identify` ranking by
  warming rate, `--check` building the daemon's composite.

These never replace §4.1–4.6.

### 4.8 HTTP tests

`tests/test_http_api.py`. See also §6. A stub `ControlSurface`; no
hwmon. `/api/state` and `/api/health` bodies equal
`ControlSnapshot.state_payload()` / `health_payload()` with exactly the
documented keys. Auto mode: `POST /api/pwm` → 409. Malformed JSON, body
not an object, missing field, unknown channel, PWM outside `[0, 1]` or
outside `[pwm_min, pwm_max]` → 400. Unknown `/api/...` route → 404.
The DAS routes are tested in `tests/test_das_intents.py` (`POST
/api/limit` and `/api/bay` on DAS and legacy configs, fuzzed bodies never
5xx and never exceed a configured limit, `GET /api/estimate`, `/api/bays`,
`/api/model`, 404 on legacy), `POST /api/ident` 200 / 400 / 409 with
the reasons (here) and
`tests/test_inputs_smart.py` (`POST /api/in/smart`). Every app is built
with an authenticator (`tests/http_fixtures.py`: a test user at the
minimum iteration count) and the clients send its credentials.

`tests/test_http_auth.py` (HTTPS and basic auth): hashing round trip,
per-hash salt and stored iteration count, malformed hashes never verify;
credentials file parsing (comments, duplicates, bad names), updates that
keep other lines, the mode checks and an empty file; `http:` defaults
equal to both example configs and every invalid value rejected; the
`Authorization` parser; the authenticator's cache, per-client backoff
(doubling, capped, reset on success) and reload of a changed, invalid or
deleted file; TLS context refusals. Over the wire, with a certificate
generated by `openssl` in a fixture (skipped with a reason when `openssl`
is missing): `401` with the realm on every route (`GET /`, every API
route, unknown paths) without, with wrong or with non-basic credentials,
not `401` with the right ones (`200` for `/`, `/api/state`,
`/api/health`, `POST /api/mode`); a plain-HTTP request gets no HTTP
response; the backoff answers `429` with `Retry-After`; `HttpService`
serves over TLS and refuses to start without a certificate, key,
credentials file or users, with a permissive credentials file or an
invalid `http:` value (logged). Fuzzed `Authorization` headers never
authenticate, never raise and never get a 5xx. `tools/http_user.py`:
create, update, comments kept, mode 0640, stdin and prompt, refusals
(empty password, bad name, invalid file never clobbered, no password
option). `tests/test_main.py` runs the daemon with `http.enabled` and no
certificate: the loop runs, the error is logged. The suites skip
themselves when binding a localhost socket is denied (some sandboxes); CI
runners allow it.

### 4.9 Glue, publishers, deploy

- `tests/test_supervisor.py` — control-mode rules, overrides, setpoints,
  presets, `compose` (rate limit, fallback beats overrides, purity),
  snapshot and health flags, concurrent HTTP and loop threads (`slow`).
  Degraded compose per channel is in `tests/test_zones.py`; limits, bays
  and DAS presets in `tests/test_das_intents.py`.
- `tests/test_loop.py` — §4.3 loop cases, READY/WATCHDOG/STOPPING,
  overrides and bumpless release through the loop, an override on frozen
  temperatures tripping Stuck, run scheduling, the `on_tick` hook, the
  step budget alarm (injected clock: tracking, warn/error thresholds,
  log rate limiting, `/api/health` and the MQTT state blob), sim
  closed loops (`slow`).
- `tests/test_main.py` — CLI parsing, exit codes, `--once`, sim wiring
  (`--sim-plant basic|rich|das`), xt6 map mismatch → exit 2, `--source
  hwmon` wiring and a missing binding → exit 2, `--record` and
  `record_path`, `--model-store` and `STATE_DIRECTORY` (legacy: ignored /
  exit 2), publisher wiring and start failures, the
  SIGTERM stop path in-process, a SIGTERM delivered inside a stderr write,
  a second SIGTERM during shutdown, and a real subprocess (`slow`).
- `tests/test_sdnotify.py` — address resolution, no-op without
  `$NOTIFY_SOCKET`, failures return `False`, a real `AF_UNIX` datagram
  socket (skipped where binding one is denied).
- `tests/test_mqtt_ha.py` — topics, Discovery entities, PWM numbers only
  in manual and deleted on leaving it, state payload, command parsing
  never raises, `on_message` never raises, `cmd/ident` and a retained
  `start` that never starts an experiment, `add_topic_handler` /
  `topic_matches`. No broker. DAS entities and topics:
  `tests/test_das_intents.py`.
- `tests/test_inputs_smart.py`, `tests/test_smart_agent.py` — inbox
  staleness by receipt time and rejection of malformed payloads; the
  agent's discovery, `smartctl -j` fixtures including `-n standby`, never
  raising.
- `tests/test_recorder.py`, `tests/test_tools_fit_replay.py` — record
  shape (DAS and legacy), rotation, never raising; `fit_model.py`,
  `fit_fans.py` and `replay.py` end to end on synthetic `sim/das.py`
  recordings.
- `tests/test_publishers_runtime.py` — `HttpService` reporting a start
  failure (serving over TLS and the refusals: `tests/test_http_auth.py`); `MqttService` with a fake client: nothing
  while disconnected, Discovery per connect and on the manual boundary,
  host refresh interval, swallowed publish errors.
- `tests/test_hostinfo.py` — every host metric against fake procfs /
  sysfs files, `None` on missing or malformed input.
- `tests/test_deploy.py` — unit file (`Type=notify`, `NotifyAccess=main`,
  `Restart=always`, watchdog, no `ExecStop=`, venv `ExecStart`,
  `TimeoutStartSec >= WatchdogSec`), udev rules against the unit’s
  groups (hwmon pwm group write, usb, hidraw), install script triggers
  udev, install script creates the self-signed certificate at the
  `http:` default paths only when absent and creates no users, `bash -n` and `shellcheck` on both scripts (shellcheck skipped
  when not installed; GitHub runners have it), `StateDirectory=` for the
  model store, and the SMART agent unit being a user unit that
  `install-pi.sh` does not install.

### 4.10 DAS suites (estimator, model, solver, experiments)

Everything runs against `sim/das.py`, the DAS truth plant, or small zoned
configs; no hardware. "PR" runs in every PR and `main` run; "nightly"
tests carry the `nightly` marker.

| File | Proves | Where |
|------|--------|-------|
| `tests/test_zones.py` | DAS config parsing, defaults and rejections; `strict` trust per group; closure `F*` with `declared` / `none`; per-zone timers and confirmation; `degraded` vs `fallback`; legacy = one implicit zone; per-channel `compose`; per-role Stuck sizing; the DEGRADED banner and health field | PR |
| `tests/test_mpc_zone_fallback.py` | a fault in zone A never lowers any channel of its reach below `prev` (hold, then `max(prev, fallback_pwm)`); channels outside keep regulating; the solver request never carries faulted-zone sensors and healthy commands do not depend on their values; per-zone recovery is bumpless; Flicker in one zone never resets another; a dropout in a redundant group is no fault; solver faults; legacy `mpc` turns a zone fault into whole fallback | PR (one sweep nightly) |
| `tests/test_das_core.py` | the core invariants, closed loops and DAS goldens for `pi_das` and `mpc_das` (§4.2) | PR |
| `tests/test_pi_das.py`, `tests/test_estimates.py`, `tests/test_das_config.py` | the margin-deficit PI (served zones, unconstrained channels, fixed channels, occupancy), the estimates block and prior map, `noise` / `limit_c` / served-zone config | PR |
| `tests/test_estimator.py` | exact discretisation and Joseph form (random sequences keep P symmetric PSD); first tick and constant readings; σ grows while a bay is unobserved and shrinks back; redundant members; the occupancy machine incl. "never empty while zone air is unobserved"; SMART calibration acceptance, rejection, serial change and expiry after `calibration_max_age_days`; a guessed serial never relaxes a class; determinism, JSON round trip, malformed memory | PR |
| `tests/test_associate.py` | detrended correlation, greedy assignment with margins, confirmation, full-window history, drops on silence / jump / empty, a declared serial wins; on the truth sim the right bays are found and indistinguishable bays refused | PR |
| `tests/test_hotswap.py` | slow swap, quick swap (never through `empty`, margin widens, class follows the new drive) and empty-at-boot on the truth sim: no limit violation, no zone fault, fans rise, constraints removed on empty | PR |
| `tests/test_thermal_model.py` | structure, fan groups, parameter table, Jacobians vs finite differences, `eig` vs matrix exponential and the Euler fallback, windows, lag correction, RLS safeguards, status machine, never raising in `step`, shadow never changes the command | PR |
| `tests/test_thermal_ident.py` | identifiability with group experiments on the truth sim: in-zone `E` within 15 % (PR seed), per-bay `k` within 25 %, `leak` / `κ` at prior, convergence; regulation only never converges; the seed sweep (bound 25 %), sensor offsets, the rich preset | PR: 4 cases; nightly: sweeps |
| `tests/test_solver_das.py` | active-piece SQP vs a projected-gradient reference, monotone objective, iteration cap, forbidden-band snap and hysteresis, zero-order hold, prediction vs thermal Jacobians, noise index and surrogate, bumpless offset, fixed channels, validity gate and model fallback with dwell, a clock stepped back, horizon and block extremes | PR |
| `tests/test_noise_regression.py` | calibrated DAS MPC noise ≤ 0.8× the quietest uniform curve at equal or better worst true margin (measured 0.31–0.48×); uncalibrated bound 2.0 (0.30–0.69×); rich preset bound 1.3 (up to 1.21×); MPC vs PI-DAS reported, not asserted (PI-DAS has not settled within the window on several seeds) | PR: 2 seeds; nightly: 8-seed sweeps |
| `tests/test_bench_budget.py` | DAS MPC step p99 ≤ 12× (named constant) the legacy MPC p99, the 75th percentile of several interleaved, warm-up-discarded repeats (§8 item 6); `bench_step.py` runs both DAS solvers; absolute p99 ≤ `mpc.budget_ms` only on `armv6l` | PR / Pi |
| `tests/test_modelstore.py` | config keys, store path (CLI, env, legacy), fingerprint covers structure not policy, corrupt / truncated / wrong-schema / wrong-fingerprint files → prior, fresh vs stale by age (a clock behind the file is stale), fresh loads `frozen` and the MPC acts at once, stale holds until `model_reconfirm_s` with the prediction error in bounds, calibration keyed by serial and inflated when stale, a save does not reset a stale hold, atomic writes, malformed seeds never raise | PR |
| `tests/test_ident_experiment.py` | config rules; groups, targets and served zones; the seeded two-level sequence; every precondition with its reason; the envelope at its threshold; the aborts (human intent, stop, fallback, every zone in fault, emergency command, apply failures, a frozen sensor); bumpless release; overrides through `compose` and fallback beating them; a restart never resumes; legacy refuses | PR |
| `tests/test_sim_das.py` | the truth plant: energy balance through transients and hot swap, steady state, more airflow never warms anything, dead band and exponent, quantisation per sensor type, lags, SMART cadence, determinism per seed | PR |

Test cost: the PR selection is about 1,900 tests in under five minutes
on the development machine (`HYPOTHESIS_PROFILE=ci`); PR CI stays under
its 15-minute job timeout (§12).

---

## 5. Digole + touch (after Command is stable)

Not implemented yet. Page state machine, not a web of `if`. Source of
truth is the daemon: the screen reads `ControlSurface.snapshot()` and
sends intents through `ControlSurface.submit()`, exactly like HTTP (§6).
The screen renders `(obs, cmd)` and emits `tap` / `swipe` / `hold`.

Navigation: bottom bar or swipe. Debounce touch; hit-test rectangles.

```text
[ Overview ] [ Drives ] [ Zones/Fans ] [ Model ] [ Host ]
```

1. **Overview** — hottest drive and its margin to the limit
   (`limit_margin_c`), noise index (`noise_db`), zone status (ok /
   degraded / fault), inlet air, control mode, MQTT. Tap a drive → Drives,
   tap a zone → Zones/Fans.
2. **Drives** — per bay: estimated temperature, σ, margin, class, limit,
   occupancy, SMART calibration and association (`GET /api/estimate`,
   `/api/bays` equivalents). Tap — 1–2 min RAM buffer.
3. **Zones/Fans** — per zone trust and fault timer; XT6+Quadro channels
   with RPM+PWM and whether each is under fallback policy. Tap — override
   slider. “Auto” clears override.
4. **Model** — active solver (`mpc` / `pi_das`) and why, thermal model
   status and prediction error, store source, Quiet/Normal/Cool presets,
   a running experiment. No matrices on screen.
5. **Host** — CPU °C, load, RSSI, disk, uptime.

Global **FAULT** (USB gone, controller in fallback) covers everything; a
**DEGRADED** strip names the zones in fault.

Control modes (`ControlMode`): `auto` (the solver drives every channel),
`manual` (every channel holds an override), `mixed` (some channels
overridden). Rules in §6.

Frame rate: on state change or 2–4 Hz, not 20. No animation on Zero W.

Digole protocol is our own layer (UART/I2C/SPI), not Arduino libraries.

---

## 6. HTTP view and control

Same source of truth as Digole: `(obs, cmd)` and the same `apply()` path.
HTTP must **not** write PWM itself. It builds intents with
`parse_intent(kind, body)` and hands them to `ControlSurface.submit()`
(`control/intents.py`); the `Supervisor` owns the arithmetic and the loop
turns the result into the applied `MpcCommand`.

The server runs on its own asyncio loop in a daemon thread
(`publishers/runtime.py`). LAN only for v1.

Runtime control state (mode, overrides, setpoints, preset, runtime limits
and bay declarations, a running experiment) lives in memory only: the
daemon starts in `auto` with preset `normal` and the setpoints, limits
and bays from `config.yaml`, and nothing of it is written back to disk.
(The model store persists the controller's model and calibrations, not
human intents.)

### HTTPS and authentication

**Owner decision (2026-09-14):** the API and the page are served over
**HTTPS with HTTP basic auth** only. Several routes relax cooling
(`/api/bay`, `/api/limit`, `/api/pwm`, `/api/ident`), so there is no
unauthenticated route and no plain-HTTP listener
(`publishers/httpauth.py`, `publishers/http.py`).

Config `http:` (parsed and validated by `HttpSettings` in
`publishers/httpauth.py`; every key is shown in both example configs):

| Key | Default | Meaning |
|-----|---------|---------|
| `enabled` | `false` | must be `true` for the daemon to start the service |
| `bind` | `0.0.0.0` | listen address |
| `port` | `8443` | TLS port (`0` picks a free port; tests) |
| `tls_cert` | `/etc/aqua-bridge/tls/cert.pem` | PEM certificate (chain) |
| `tls_key` | `/etc/aqua-bridge/tls/key.pem` | PEM key; owned by root or the service user, not readable by others nor group-writable (`0640 root:<service user>`) |
| `credentials_file` | `/etc/aqua-bridge/http-users` | `user:hash` lines, same owner and mode rule |
| `realm` | `aqua-bridge` | basic-auth realm (1–64 printable ASCII, no `"` or `\`) |
| `hash_iterations` | `100000` | PBKDF2 iterations for hashes written by `tools/http_user.py` (≥ 1000) |
| `auth_cache_s` | `300` | seconds a verified user:password stays cached in memory; `0` verifies every request |
| `auth_fail_limit` | `5` | consecutive failed logins from one client address before the backoff; `0` disables it |
| `auth_backoff_s` | `1` | first backoff, doubled with every further failure |
| `auth_backoff_max_s` | `300` | longest backoff (≥ `auth_backoff_s`); a client's count is forgotten once this long has passed without a failure after its backoff ended |

- **TLS:** TLS 1.2 or newer, the certificate and key from `tls_cert` /
  `tls_key`. `deploy/install-pi.sh` creates a self-signed ECDSA P-256
  certificate for the host name (and `<host>.local`) when neither file
  exists, and never overwrites either (§10). A browser warns about a
  self-signed certificate until it is trusted; an owner-provided
  certificate goes to the same paths.
- **Credentials file:** one `user:hash` line per user; `#` comments and
  blank lines are allowed. User names use `A-Z a-z 0-9 . _ @ + -`. The
  hash is `pbkdf2_sha256$<iterations>$<salt>$<key>` (PBKDF2-HMAC-SHA256,
  stdlib `hashlib`; a 16-byte random salt per user and the 32-byte key in
  unpadded URL-safe base64). The iteration count travels with each hash,
  so changing `hash_iterations` affects only users written afterwards.
  Verification uses `hmac.compare_digest`; an unknown user costs one
  derivation at the highest iteration count stored in the file, like a
  known one. The file is reread whenever it changes on
  disk (no restart); if it becomes missing, unreadable, too permissive or
  invalid, every request is denied (logged) until it is fixed.
- **`tools/http_user.py`** creates or updates one user: `sudo
  .venv/bin/python tools/http_user.py --config
  /etc/aqua-bridge/config.yaml --group <service user> <name>`. The
  password comes from a prompt (twice) or, with `--stdin`, from the first
  line of standard input, never from the command line. The user's line is
  replaced or appended (other lines and comments kept), the file is
  written atomically with mode 0640, an existing file keeps its owner and
  group and a new one gets `--group`. `--file` overrides
  `credentials_file`. An invalid existing file is refused, not rewritten.
- **Every route** — `GET /`, every `/api/...` route and unknown paths —
  passes the auth middleware first. No or wrong credentials: `401` with
  `WWW-Authenticate: Basic realm="<realm>", charset="UTF-8"` and
  `{"error": "authentication required"}`. A client over
  `auth_fail_limit` consecutive failures (a malformed `Authorization`
  header counts; a missing one does not) gets `429` with `Retry-After`
  and no password check until the backoff ends; a success resets its
  count. A check that needs PBKDF2 runs in a worker thread, one at a
  time, so the event loop keeps serving; the page's 2 s poll is answered
  from the cache. On the Zero W an uncached check costs on the order of a
  second of CPU at the default iteration count (not measured on the Pi
  yet): that is the price of the hash, and the cache and backoff keep it
  rare.
- **Startup refusal:** with `http.enabled: true`, the service validates
  `http:`, loads the certificate and key and the credentials file (at
  least one user) **before** opening a socket. Any problem — an unknown
  key or invalid value, a missing or unreadable certificate or key, a key
  or credentials file readable by others or owned by another non-root
  account, an empty credentials file —
  leaves the API off with one error in the journal (`http: HTTPS API not
  started (fan control continues without it): ...`), as does a bind
  failure; the daemon keeps controlling the fans (publishers never touch
  the control path). The `http:` section is validated here, not at config
  load, so a mistake in it never stops the controller.
- **Plain HTTP is refused:** there is no listener without TLS; an
  `http://` request to the port fails the TLS handshake and gets no HTTP
  response.
- **MQTT:** the MQTT counterparts of the cooling-relaxing routes
  (`cmd/limit`, `cmd/bay`, `cmd/ident`, `cmd/pwm`, `cmd/mode`, ...) have
  no authentication of their own: they rely on the **broker's**
  authentication and ACLs (§7).

### View

| Method | Path | Returns |
|--------|------|---------|
| `GET` | `/api/state` | `ControlSnapshot.state_payload()` |
| `GET` | `/api/health` | `ControlSnapshot.health_payload()` |
| `GET` | `/api/estimate` | DAS: `{"estimates": {bay: …}, "estimator": {…}}` of the last command; legacy: 404 |
| `GET` | `/api/bays` | DAS: `{"bays": {bay: {"declared": {zone, occupied, class, serial}, "estimator": {occupancy, class, serial, association, calibration, candidates, …} \| null}}}`; legacy: 404 |
| `GET` | `/api/model` | DAS: `{"thermal", "parameters", "calibration", "store", "experiment"}`; legacy: 404 |
| `GET` | `/` | `publishers/static/index.html` |

`/api/state` keys:

- `obs` — last successfully read `PlantObservation` (`temps`, `rpm`,
  `pwm`, `ts`, and `inputs` when not empty), `null` before the first; a
  failed read does not replace it
- `cmd` — last command handed to the sink, applied or not (`pwm`,
  `mode`, `diagnostics`, including `diagnostics.supervisor` and, in DAS
  mode, `zones`, `zones_in_fault`, `estimates`, `bays`, `noise`,
  `thermal`, `store`), `null` before the first tick; after shutdown the
  stop write
- `mode` — control mode `auto` | `manual` | `mixed`
- `preset` — `quiet` | `normal` | `cool`
- `setpoints` — the user’s setpoints, without the preset offset (empty in
  DAS mode)
- `overrides` — manual PWM per overridden channel (human overrides only;
  an experiment's levels are not listed)
- `channels`, `temps`, `pwm_min`, `pwm_max` — from the base config
- DAS only: `limits` (`{"classes": {class: limit_c}, "bays": {bay:
  limit_c}}`, the limits in force) and `bays` (`{bay: {zone, occupied,
  class, serial}}`, the declarations in force). A legacy payload keeps
  its shape.

`/api/estimate`: per constrained bay `t_c`, `sigma_c`, `margin_c`
(`k·σ`), `soft_c`, `hard_c`, `limit_c`, `limit_margin_c` (`hard_c −
t_c`), `occupancy`, `class`, `zone`, `zone_trusted`, `calibrated`,
`source`, `q_w`; plus the estimator summary (status, per zone air
estimate, SMART counters). `/api/model`: `thermal` is
`diagnostics["thermal"]` (status, prediction error, per zone and bay the
coefficients with relative standard errors; `{"status": "off"}` without
`model_shadow`), `parameters` the static table of §3 (unit, bounds,
prior, identified from), `calibration` per bay (serial, calibrated,
`sigma_cal_c`, calibration details), `store` what the model store loaded
(`{"source": "off"}` without a store), `experiment` the identification
experiment's status (running, target, phase, level, elapsed and remaining
seconds, the last result and abort reason).

`/api/health` keys:

- `usb_present` — the last tick both read and applied successfully
- `mqtt_connected` — `null` when MQTT is disabled, `false` until connected
- `solver` — `ok` (last solver command `auto` or `saturated`),
  `degraded` (some zones in fault; their channels and those of zones
  coupled to them under fallback policy, the rest regulated),
  `fallback` (every zone in fault: gate or solver fault active), `fault`
  (no solver command yet)
- `fault_reason` — `sensor_gate` | `solver` | `null`
- `fault_since_ts` — observation clock seconds | `null` (DAS: the
  earliest zone fault)
- `uptime_s` — seconds since the supervisor started (monotonic)
- `version` — package version
- `step_ms_last` / `step_ms_max` — the last and largest `step()` wall time in
  milliseconds, measured outside `step` (`control/loop.py`); 0 before the loop
  has run a tick
- `budget_warn_count` / `budget_alarm_count` — cumulative ticks whose `step()`
  exceeded `mpc.budget_ms` / `mpc.budget_alarm_ms` since the process started
  (`control/loop.py` module docstring, "Step budget alarm")

JSON is the API. HTML is a thin, view-only client (the browser asks for
the basic-auth credentials once and sends them with every poll): it polls
`/api/state` and `/api/health` every 2 s (no websockets until 2W) and shows the five
sections Overview, Temps, Fans, MPC, Host. The FAULT banner shows when
`solver` is `fallback` or `fault`, or when a poll fails; a **DEGRADED**
banner names the zones from `cmd.diagnostics.zones_in_fault` when
`solver` is `degraded`. Controls are JSON-only for now. The Host section
reads host metrics from `/api/state`, which does not carry them yet, so
it shows dashes, and the page does not yet show the drive estimates (§8).

### Control

Every `POST` body is a JSON object; an empty body means `{}`. Success:
`200 {"ok": true}`. Errors: `{"error": "..."}` with `400` for invalid
input (bad JSON or UTF-8, body not an object, unknown or missing field,
wrong type, non-finite number, unknown channel, temperature, bay, class
or group, value out of range, a change whose effective config fails
validation, a DAS-only intent on a legacy config) and `409` for a
well-formed request the current state forbids. Unknown paths are `404`.
All of this comes after authentication: without valid credentials every
route answers `401` (or `429` while the client backs off), above.

| Method | Path | Body | Effect |
|--------|------|------|--------|
| `POST` | `/api/mode` | `{ "mode": "auto"\|"manual"\|"mixed" }` | Control mode (below) |
| `POST` | `/api/setpoint` | `{ "channel": "coolant", "celsius": 35 }` | Legacy: target T for a temperature that has a setpoint in the config |
| `POST` | `/api/limit` | `{ "bay": "b03", "limit_c": 45 }` or `{ "class": "hdd", "limit_c": 48 }` | DAS: absolute drive limit of one bay or class; legacy: 400 |
| `POST` | `/api/bay` | `{ "bay": "b03", "occupied"?: true\|false\|"auto", "class"?: "ssd_sata", "serial"?: "X" }` | DAS: declare a bay's occupancy, class or serial; `null` restores the configured value; legacy: 400 |
| `POST` | `/api/pwm` | `{ "channel": "xt1", "pwm": 0.4 }` | Manual override; 409 in `auto` |
| `POST` | `/api/preset` | `{ "name": "quiet"\|"normal"\|"cool" }` | Solver aggressiveness |
| `POST` | `/api/auto` | `{ "channel": "xt1" }` or `{}` | Clear override (one channel or all) |
| `POST` | `/api/ident` | `{ "action": "start", "group": "g" }`, `{ "action": "start", "channel": "xt1" }` or `{ "action": "stop" }` | DAS: start or stop an identification experiment |
| `POST` | `/api/in/smart` | `{ "serial", "model", "temp_c", "ts_wall" }` | The SMART agent's non-MQTT twin: into the `SmartInbox`; a malformed body is 400 |

Rules (`control/supervisor.py`):

- **`/api/mode`:** `auto` clears every override (the channels return to
  the solver bumplessly). `manual` gives every channel without an
  override one, seeded from the last applied PWM (or `fallback_pwm`
  before anything was applied) and clamped into `[pwm_min, pwm_max]`, so
  entering manual does not move a fan. `mixed` keeps the overrides as
  they are (possibly none).
- **`/api/pwm`:** `pwm` must be a number in `[0, 1]`, the channel must
  exist, and the value must lie in `[pwm_min, pwm_max]` (checked before
  the mode). In `auto` the request is a **409 and nothing changes**: a
  raw PWM never flips the mode; the client posts `/api/mode` first. In
  `manual` / `mixed` the override is stored and the mode stays as set.
- **`/api/auto`:** clears one override (unknown channel → 400) or all.
  No override left → `auto`; some left while `manual` → `mixed`.
- **`/api/setpoint`:** `channel` is a temperature name. Only temperatures
  with a setpoint in `config.yaml` can be changed: giving a monitored-only
  temperature a target at runtime would change what the fans chase. The
  effective value (with the preset offset) must lie strictly inside
  `(temp_min_c, temp_max_c)`. A DAS config without setpoints has nothing
  to set (400); drive limits replace setpoints there.
- **`/api/limit`** (DAS): exactly one of `bay` / `class`, which must
  exist; `limit_c` must lie in `(temp_min_c, configured limit]` — the
  class's `limit_c`, or the bay's `bay_limit` as written. A runtime limit
  can tighten or restore what the owner wrote, never exceed it. A bay's
  limit in force is `min(class limit, bay limit)`, so lowering a class
  lowers every bay of it. Limits apply on top of the preset.
- **`/api/bay`** (DAS): overrides `topology.bays.<bay>.occupied`, `class`
  and `serial` in the effective config; the bay and class must exist and
  the result must be valid (a serial on one bay only). `occupied: false`
  removes the bay's constraints like the config file does; a declared
  serial wins over the association by correlation; `GET /api/bays` lists
  the candidates to confirm.
- **`/api/ident`** (DAS): `start` needs exactly one of `group` / `channel`
  (unknown → 400); `409` when `ident_enabled` is false, an experiment is
  already running, or a precondition fails — the error names every failed
  precondition, e.g. `settle:z1`, `mode:degraded`, `start_band:b03`,
  `bay_unknown:b07`, `calibrating:b02`, `control_mode` (§3, *Active
  identification experiments*). `stop` aborts the running experiment
  (a no-op without one). Any other intent submitted while an experiment
  runs aborts it first.
- **`/api/preset`:** a documented transform of the base config, applied
  before every `step`, never written to disk. Legacy mode:

  | Preset | Setpoints | `pi_kp`, `pi_ki` | `weight_dpwm` (MPC) |
  |--------|-----------|------------------|---------------------|
  | `quiet` | +2 °C | ×0.5 | ×2 |
  | `normal` | as configured | ×1 | ×1 |
  | `cool` | −2 °C | ×2 | ×0.5 |

  DAS mode (drive-limit regulation) changes the meaning:

  | Preset | Every class `comfort_c` | `noise.weight_noise` |
  |--------|-------------------------|----------------------|
  | `quiet` | −2 °C (floored at 0) | ×2 |
  | `normal` | as configured | ×1 |
  | `cool` | +2 °C | ×0.5 |

  A smaller comfort band raises the soft target `limit − comfort − k·σ`
  (the fans may run slower); the absolute limit and the hard target never
  move with a preset, and PI gains and the move penalty are not scaled. A
  zoned config that still declares setpoints gets the legacy setpoint
  transform as well.
- **Composition:** each tick, every overridden channel gets its override,
  rate-limited to `|Δ| <= d_pwm_max` against the last **applied** PWM and
  clamped into `[pwm_min, pwm_max]`; other channels keep the solver’s
  value; the command keeps the solver’s `mode`, and
  `diagnostics.supervisor` records control mode, preset, overrides and
  whether they were applied. An experiment's levels are composed the same
  way. Do not bypass `d_pwm_max` unless mode is an explicit emergency
  (not in v1).
- **Fallback beats manual** (owner decision), per channel: while the
  solver command is `fallback`, no override is applied; while it is
  `degraded`, overrides are not applied on the channels in
  `diagnostics.fallback_channels` (listed in
  `diagnostics.supervisor.overrides_blocked`), and a degraded command
  without a readable channel list blocks every override. The hold /
  ramp-high command goes to those fans unchanged: a human-pinned low duty
  must not reduce cooling while the controller is blind. Overrides are
  kept and resume when the fault clears. A manual override does not
  bypass the gate either: moving a fan by more than `stuck_pwm_net` while
  the temperatures do not answer within `stuck_s` is, by §3 rule 3, a
  Stuck sensor.

### Tests (no hardware)

See §4.8. API is required in CI; HTML is optional.

Zero W: JSON + one static page. Heavier UI after Zero 2 W.

---

## 7. Home Assistant

MQTT Discovery first; no custom HA integration in v1.
`publishers/mqtt_ha.py` builds topics, Discovery payloads and command
parsing as pure functions; `MqttClient` is a thin paho-mqtt 2.x wrapper;
`publishers/runtime.py` (`MqttService`) runs it next to the loop.

Config `mqtt:` — `enabled` (must be `true`; `false` in the example),
`host`, `port` (1883), `username`, `password` (both optional),
`discovery_prefix` (`homeassistant`), `node_id` (`aqua-bridge`). `host:` —
`interval_s` (5): how often host metrics are refreshed.

Topics:

- `{node_id}/status` — availability: LWT `offline` (retained, qos 1);
  `online` on every connect; `offline` published before a clean
  disconnect
- `{node_id}/state` — one retained JSON blob every tick (qos 0):
  `ControlSnapshot.to_dict()` (the `/api/state` keys plus `health` and
  `extra`, which carries `experiment` in DAS mode) plus `host`
  (`cpu_temp_c`, `load1`, `load5`, `load15`, `mem_used_pct`,
  `mem_total_kb`, `disk_used_pct`, `disk_free_gb`, `wifi_rssi_dbm`,
  `uptime_s`; `null` when unreadable)
- inbound commands, raw payloads (not JSON): `{node_id}/cmd/mode` (control
  mode), `{node_id}/cmd/preset`, `{node_id}/cmd/auto` (channel, or empty
  for all), `{node_id}/cmd/setpoint/<temp>` (number, °C, one per setpoint
  temperature), `{node_id}/cmd/pwm/<channel>` (number 0..1)
- inbound, DAS mode only: `{node_id}/cmd/limit/<bay>` and
  `{node_id}/cmd/limit/class/<class>` (number, °C; a tail after `limit/`
  that starts with `class/` is a class), `{node_id}/cmd/bay/<bay>`
  (`occupied` | `empty` | `auto`; class and serial go through `POST
  /api/bay`), `{node_id}/cmd/ident` (`start:group:<group>` |
  `start:channel:<channel>` | `start:<channel>` | `stop`). A **retained**
  `start` is ignored and logged (a broker would redeliver it on every
  reconnect); a retained `stop` still stops.
- inbound SMART: `{node_id}/in/smart/<serial>`, retained JSON `{serial,
  model, temp_c, ts_wall}` from `tools/smart_agent.py`, into the
  `SmartInbox` on the same broker connection
  (`MqttClient.add_topic_handler`)

Discovery config topics are
`{discovery_prefix}/{component}/{node_id}/{object_id}/config`, retained,
qos 1; `unique_id` is `{node_id}_{object_id}`; every entity carries the
availability topic and one device block. Entities:

- sensors `host_cpu_temp_c`, `host_load1`, `host_mem_used_pct`,
  `host_disk_used_pct`, `host_wifi_rssi_dbm`, `host_uptime_s`
- sensor `temp_<temp>` per `mpc.temps`; `rpm_<channel>` and
  `pwm_<channel>` (commanded PWM in %) per channel
- number `setpoint_<temp>` per setpoint (range `temp_min_c..temp_max_c`,
  step 0.5); a DAS config without setpoints publishes none
- number `pwm_cmd_<channel>` per channel (range `pwm_min..pwm_max`, step
  0.01) **only while the control mode is `manual`** — not in `mixed`,
  where some channels are still solver-driven
- DAS mode, per bay: sensors `drive_temp_<bay>` (estimated drive
  temperature, °C), `drive_margin_<bay>` (`limit_margin_c = limit − k·σ −
  t`: degrees left to the limit after the uncertainty margin; negative
  means the drive may be over its limit), `drive_sigma_<bay>` (σ, °C);
  binary sensor `bay_occupied_<bay>` (`empty` is off, `occupied` and
  `unknown` are on). An empty bay's drive sensors read unknown.
- DAS mode: sensors `noise_db` (the fan noise index), `model_status`
  (`prior` | `learning` | `converged` | `suspect` | `error` | `frozen` |
  `stale`, `off` without `model_shadow`) and `model_pred_err_c` (worst
  zone's one-window prediction error, °C); binary sensor `ident_running`;
  number `limit_<class>` per drive class (range `temp_min_c` .. the
  configured limit, state from `limits.classes.<class>`)

HA sends a **setpoint** (legacy) or a **limit** (DAS), not raw PWM, while
Auto. A raw PWM command in `auto` is rejected by the supervisor exactly
like HTTP’s 409, logged and dropped. Inbound commands never raise:
`parse_command` returns nothing for a garbage topic or payload, and
`on_message` catches every rejected intent and any other exception,
because paho runs callbacks on its network thread and an exception there
would end it.

Lifecycle: the client connects asynchronously with automatic reconnects
(1–60 s), so a broker that is down at boot does not stall the daemon.
Connection changes feed `/api/health`. On the loop thread, every tick:
nothing while disconnected; Discovery after every (re)connect and
whenever the control mode crosses the `manual` boundary (entities that
exist only in another mode are deleted with an empty retained payload);
then the state blob. Publish errors are counted and logged, never raised
into the loop. The publisher is an observer: a dead broker or a publisher
bug never touches `read → step → apply`.

Broker: Home Assistant Mosquitto (or any MQTT broker on the LAN). Host
and credentials stay in `config.yaml` / `private.md`, not here.

**Security:** the inbound command topics (`cmd/mode`, `cmd/pwm`,
`cmd/setpoint`, `cmd/preset`, `cmd/auto`, and in DAS mode `cmd/limit`,
`cmd/bay`, `cmd/ident`) and `in/smart` are accepted from whoever can
publish on them: unlike the HTTPS API (§6), they rely entirely on the
**broker's authentication and ACLs**. Use a broker that requires a user
and password (Home Assistant's Mosquitto add-on does), give aqua-bridge
its own account, and limit publishing to `{node_id}/cmd/#` and
`{node_id}/in/smart/#` to the accounts that need it. Not yet
exercised against a live broker or Home Assistant (§8).

---

## 8. TODO

Open items are numbered once and keep their number; a finished item moves
to §8.5 with its number, new items take the next free number. Refer to
them as "§8 item N".

### 8.1 Owner decisions (2026-09-14)

- PI-like DAS counts the uncertainty margin `k·σ` **once** (item 1).
- `topology.zones.<z>.coupled_to` stays symmetric: an asymmetric or self
  coupling is rejected (implemented).
- The HTTP API is served over **HTTPS with basic auth** (item 4).
- The DAS step budget on the Zero W is **600 ms** p99 (was 500 ms), alarm
  750 ms (implemented in `tools/bench_step.py` and
  `tests/test_bench_budget.py`; `pytest -m pi` passes on the Zero W, where
  the DAS MPC measured p99 507–552 ms).
- Live MQTT and Home Assistant checks use the owner's Home Assistant
  broker; its host name is in `private.md` (item 21).
- An experiment holding back a cooling increase is deferred to the
  Zero 2 W upgrade (item 52).

### 8.2 Open — no DAS hardware needed (dev machine, CI, the Pi, the PC)

1. **Done (PR #22):** the PI-like DAS error is now `max(t̂ − soft)`, counting `k·σ` once inside `soft`. PI-like DAS: count `k·σ` once. The per-channel error becomes
   `max(t̂ − soft)` with `soft = limit − comfort − k·σ` (today `k·σ` is also
   added to `t̂`). Regenerate only the DAS goldens `das_*.pi_das.json`; keep
   every per-zone invariant.
2. **Done (PR #21):** `tests/test_hw_xt6.py::test_live_read_and_writeback` no
   longer writes `0.0` to a channel whose PWM read returns `None`; such
   channels are excluded from the write-back and reported, and the test
   skips with a clear reason if none is readable. **Must land before item 36.**
3. False zone fault on a healthy enclosure: the Stuck rule's sibling
   evidence faults a zone when an idle bay's DS18B20 stays inside its
   1.5-LSB band for `stuck_s` while a sibling's activity changes and the
   controller compensates (about one in three 75-minute `sim/das.py`
   runs). Fix the evidence rule; regression: long DAS sim runs with zero
   false zone faults.
4. **Done (PR #24):** the API and page are served only over HTTPS with basic auth (§6), with `tools/http_user.py` for users and a self-signed certificate from `install-pi.sh`.
   HTTPS with basic auth for the API and the page: certificate and key
   paths in `http:`, hashed credentials in a root-owned 0640 file under
   `/etc/aqua-bridge`, every route authenticated, plain HTTP refused.
   Document that the MQTT counterparts (`cmd/bay`, `cmd/limit`,
   `cmd/ident`) rely on broker authentication.
5. **Done (PR #23):** Runtime budget alarm: `control/loop.py` logs a
   rate-limited warning past `mpc.budget_ms` and error past
   `mpc.budget_alarm_ms`, both now config keys, with `step_ms_last` /
   `step_ms_max` / exceedance counters in `/api/health` and the MQTT state
   blob.
6. **Done (PR #23):** The relative budget gate in CI is thin (DAS MPC p99
   9–11× the legacy MPC against 12×); `tests/test_bench_budget.py` now
   interleaves, discards a warm-up repeat and gates on the 75th percentile
   of several per-repeat ratios instead of one min/min pair.
7. **Done (PR #25):** `install-pi.sh --das` installs `config.example-das.yaml` and the `deploy/aqua-bridge-das.conf` systemd drop-in (`ExecStart=` with `--source hwmon`); without `--das` the legacy path is unchanged.
8. `zones.trust_rule: sigma` (zone trust from the estimator's σ); the
   config accepts it, `strict` applies today.
9. A Jump on a redundant group member is accepted after one tick without
   `confirm_ticks`.
10. The drift check's hysteresis can hold the PI-DAS model fallback for
    tens of minutes after a load step (return threshold 0.25 °C/min against
    0.26–0.29 °C/min physical transients); tune it.
11. The prediction-error guard also scores drives in faulted zones (extra,
    louder fallbacks only); restrict it to eligible zones.
12. Reset a bay's thermal coefficients on a hot swap to a different drive
    (kept today; the MPC's settle exclusion covers only the transient).
13. Split a fan group's shared `E` into per-channel coefficients from the
    single-channel experiment phases.
14. Online fan-curve fit: `fan_curves` in the store is validated but
    nothing produces or reads it; `tools/fit_fans.py` output is copied into
    `fan_models` by hand.
15. Load `tools/fit_model.py`'s `model.json` into the model store
    (different file shape today).
16. A switch that freezes online adaptation of a converged model (today
    only a fresh store file loads zones `frozen`).
17. Estimator accuracy on the `rich` sim preset: calibrated estimates up to
    2 °C off make the MPC up to 1.21× the uniform-curve noise.
18. Serial → bay association: a wrong correlation pair can still feed
    another drive's SMART into a bay's calibration (bounded by
    `smart_reject_c`); tighten acceptance.
19. Occupancy debounce: a one-tick proximal dropout on an empty bay (CRC
    failure, clock glitch) moves it to `unknown` with drive variance
    25 °C² (louder, not unsafe).
20. Experiments: settle timers are not persisted (after a restart a start
    waits `ident_settle_s` + `bay_settle_s`); a start that arrives between
    `plan_tick` and `record_tick` shifts the levels by one tick.
21. Live MQTT and Home Assistant check against the owner's Home Assistant
    broker (host in `private.md`): discovery entities appear, limit and
    setpoint numbers work, PWM numbers exist only in manual, `in/smart`
    arrives through the broker. Needs the Pi and Home Assistant, not the
    DAS.
22. `GET /api/zones` and the HA entity `zone_status_<zone>` (zone state is
    only in `cmd.diagnostics` today).
23. `POST /api/calibrate {bay, drive_temp_c}`: calibration with a handheld
    thermometer when SMART is absent.
24. HTML page: drive estimates, bays, zone and model status (the JSON views
    exist).
25. HTML Host section shows dashes: `/api/state` carries no host metrics.
26. CI time: `test (latest)` reaches 10–10.5 min on slow runners against
    the 11-minute guideline; move heavy PR tests to `nightly` or split the
    job.
27. Test gap: the DS18B20 plateau test uses an 1800 s sine, so no plateau
    is longer than `stuck_s`; add one.
28. `tools/bench_step.py` reports `plant.preset: basic` for
    `--sim-plant das`.
29. Stale docstrings: `thermal.py` and `noise.py` still call experiments
    and `fit_fans` a later milestone; `FanSpec` says `forbidden_pwm` is not
    honoured.
30. README: bring-up on a fresh Pi (checklist §10).

### 8.3 Open — needs the DAS hardware

31. USB host: `dtoverlay=dwc2,dr_mode=host` (`deploy/host-usb.sh`), powered
    hub.
32. Spike: is the Quadro's PWM writable through the XT6? If not, the Quadro
    goes on its own USB port (`--source hwmon`).
33. Spike: does the XT6 revert after the Pi stops writing? If not, software
    sensor plus firmware timeout (§2); then decide whether `release()` runs
    at exit.
34. Spike: which Quadro temperature inputs appear in hwmon; bind them in
    its `temp_map`.
35. Confirm the hwmon ABI on the real device (`tempK_input` millidegrees,
    `pwmK` 0..255, `pwmK_enable` semantics) and that the udev rule makes
    `pwmK` / `pwmK_enable` group-writable for the service user.
36. `pytest -m hardware` on the Pi with the aquaero attached (after
    item 2).
37. Verify every `temp_map` entry against its physical sensor (warm one,
    watch it move).
38. `w1-gpio` overlays on GPIO 4 and 17; confirm the `w1_therm` sysfs
    layout the reader assumes (`therm_bulk_read`, per-slave `temperature`
    and `resolution`).
39. Wire the DS18B20 buses (3-wire, 4.7 kΩ), bind every ROM id with
    `tools/w1_commission.py --identify`, measure cycle time and CRC error
    rate with `--check` (< 1 %; 11 bit if a cycle exceeds `0.4 dt`).
40. Cross-check zone-air against inlet sensor offsets at commissioning
    (0.1 °C of offset biases `E` by 15–35 %).
41. Run the SMART agent on the PC against the real drives (smartctl
    permissions, standby behaviour, NVMe namespaces); watch the
    association find the bays.
42. Enable the service once the spike is answered
    (`systemctl enable --now aqua-bridge`).
43. `control/loop.py` on the Pi against the aquaero with the service
    running.
44. Priors against the real enclosure (33 W/K per fan, `g0` / `k`, drive
    and sensor time constants, the prior sensor map `β = 0.3`,
    `b = −2.1 °C`).
45. Tune the PI-like DAS gains (`pi_kp`, `pi_ki`).
46. Promotion ladder (§13): record, fit, SMART calibration, shadow with
    experiments, MPC; decide when `config.example-das.yaml` switches to
    `solver: mpc`.
47. Move the spare thermistor inputs to the bays `tools/fit_model.py` ranks
    tightest.
48. Time `model.json` writes on the Pi's SD card.
49. Digole: protocol, pages (Overview, Drives, Zones/Fans, Model, Host),
    touch, hit-test.

### 8.4 Open — Zero 2 W upgrade

50. Run on a Zero 2 W with the same config.
51. 64-bit Lite if needed, with no API change.
52. Experiments: with `above` levels the low level is the solver's base
    frozen at start, so a solver that later wants more cooling on that
    channel is held back until the +3 °C envelope trips. Deferred to the
    Zero 2 W (owner, 2026-09-14): re-plan the levels from the live solver
    demand, which the Zero 2 W's compute allows every tick.

### 8.5 Done

#### Docs / repo

- [x] Create the GitHub repo (`gh`, §12)
- [x] `pyproject.toml` (ruff, pytest), `config.example.yaml`
- [x] CI: ruff + pytest for track A (invariants, nominal, failures, sensor lies, fuzzy, closed-loop; fake hwmon; no live USB)
- [x] Branch protection on `main` with required checks; rebase merges only (§12)
- [x] Nightly randomized Hypothesis run (`nightly-fuzz`, §12)
- [x] `nightly` marker: heavy sweeps out of PR CI (`-m "not hardware and not nightly"`), into the nightly job
- [x] `config.example-das.yaml` and this document describe the merged DAS code (docs pass)

#### Track A — MPC (dev machine, parallel with hardware)

- [x] `model.py`: Observation / Config / Command / State
- [x] `mpc.step(obs, config, state) -> (command, state)`: PI first
- [x] Sensor gate (stuck / spike / dT/dt / stale) in front of the solver
- [x] Hold-last-good then ramp to `max(prev, fallback_pwm)` (high); never `pwm_min` on fault
- [x] `aqua_bridge.sim.plant`: RC thermal plant
- [x] `assert_command_safe` on every step
- [x] `test_mpc_nominal.py`: setpoint, steps, disturbance, cold/warm start, saturation, goldens
- [x] `test_mpc_failures.py`: missing keys, bad ts, stall RPM, solver throw → fallback
- [x] `test_mpc_sensor_faults.py`: stuck, spike, jump, drift, swap, dropout, flicker, garbage
- [x] `test_mpc_fuzzy.py`: Hypothesis on random obs, sequences, mixed lies, random plants
- [x] `test_mpc_closedloop.py`: model mismatch, actuator delay, long run
- [x] Tests for hold-then-high fallback (never pwm_min on sensor fault)
- [x] First untrusted tick: cmd == prev == fallback_pwm
- [x] Reject config: fallback_pwm keys, values > pwm_min, fallback_hold_s,
      confirm_s, dT_max_c_per_s, stuck_s; confirm_s <= fallback_hold_s;
      temps ⊇ setpoints keys; stuck_eps_c, stuck_pwm_net, stuck_sibling_dT_c; median3 bool
- [x] Flicker: mode stays fallback until confirm_s of consecutive trusted ticks
- [x] Jump: new level confirms; not stuck in fallback / not forced to fallback_pwm
- [x] Spike: one untrusted tick (median3=false) / zero (median3=true), last good unchanged
- [x] Stuck: frozen T while **net** PWM or sibling T moves → untrusted (latched)
- [x] Stuck negative: equilibrium with PWM dithering, net ≈ 0 → stays auto
- [x] Every gate test parametrised over median3
- [x] Noise σ = 0.2 °C: gate reject rate under 1 %, mode stays auto
- [x] Bumpless transfer after fallback (`test_mpc_failures.py`)
- [x] Small linear MPC (numpy) behind the same `Solver` protocol; every core suite runs for `pi` and `mpc`
- [x] Legacy placeholder names (`coolant`, `air`, `radiator`, `intake`): kept only in legacy mode (`config.example.yaml`, legacy suites and goldens) as the bit-identity reference; the DAS example uses its own names

#### Track A2 — DAS target (dev machine)

- [x] Config schema for the DAS: zones, bays, fans and fan groups, fan models, sensors with role (inlet, zone air, drive-proximal, exhaust), drive classes HDD 50/5, SATA SSD 65/10, NVMe 70/10 (`model.py`)
- [x] DAS truth simulator: zones, drives with activity-dependent heat, hot swap and empty bays, placement offsets, per-sensor quantisation, SMART, tach-less outputs, splitters, `rich` preset (`sim/das.py`, `--sim-plant das`)
- [x] Per-zone trust and per-zone fallback with the declared closure; `Mode.degraded`; per-role Stuck sizing with decimated windows; per-channel `compose` (`control/zones.py`)
- [x] PI-like DAS solver on the margin deficit of the drives it cools (`control/solver_pi.py`), DAS goldens
- [x] Drive temperature estimator: per-zone Kalman filter, offset-free disturbances, occupancy machine, SMART calibration with expiry after `calibration_max_age_days`, fast-swap rule (`control/estimator.py`)
- [x] Serial → bay association by correlation, no SES (`control/associate.py`); `GET /api/bays`, `POST /api/bay`
- [x] Zoned thermal model, sin²-window RLS identification in shadow, status machine, `GET /api/model`, HA `model_status` / `model_pred_err_c` (`control/thermal.py`)
- [x] Tick recorder and offline tools: `--record`, `tools/fit_model.py --topology`, `tools/fit_fans.py`, `tools/replay.py`
- [x] DAS MPC: noise surrogate, soft / hard and terminal rows, move blocks, active-piece SQP, forbidden bands, validity gate with PI-DAS model fallback, bumpless, `mpc_every_ticks`, `noise_db` (`control/solver_das.py`, `control/noise.py`)
- [x] Model store with calibration and bays sections, fresh → `frozen`, stale → shadow hold re-confirmed for `model_reconfirm_s`, `StateDirectory=` (`modelstore.py`, `control/persist.py`)
- [x] Active identification experiments per fan group with the +3 °C envelope on estimates, `POST /api/ident`, MQTT `cmd/ident`, HA `ident_running` (`control/ident.py`)
- [x] Runtime drive limits: `POST /api/limit`, MQTT `cmd/limit/...`, HA `limit_<class>`; DAS presets (comfort band, noise weight)

#### Track B — hardware (Pi USB; fake sysfs anywhere)

- [x] `hw/xt6.py` read/apply, udev `0c70`, sysfs root injectable
- [x] `hw/xt6.py`: `pwmK_enable` re-checked on every apply (re-plug)
- [x] Startup rejection: `xt6.fans` keys == `mpc.channels`, `xt6.temp_map` keys == `mpc.temps`
- [x] `xt6.fans`: one entry per fan with `pwm` and optional `rpm`; legacy `map` / `fan_map` rejected
- [x] `test_hw_map.py` / fake hwmon in CI
- [x] `hw/sources.py`: several hwmon devices + 1-Wire, every name bound exactly once (`--source hwmon`)
- [x] systemd unit: `Type=notify`, `Wants=`+`After=network-online.target`,
      `Restart=always`, `WatchdogSec`, `TimeoutStartSec`,
      `ExecStart=/opt/aqua-bridge/.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml`,
      SIGTERM stop path writes `fallback_pwm` (no `ExecStop=`), `StateDirectory=aqua-bridge`
- [x] udev rule for hwmon `pwm*` group write (`plugdev`)
- [x] `deploy/install-pi.sh`; provisioning verified on a Zero W (service left disabled)

#### Track B2 — 1-Wire and SMART (fake sysfs and fixtures anywhere; hardware on the Pi / PC)

- [x] `hw/onewire.py`: `w1_therm` bulk-read reader threads, CRC / stall / age → `None`, resolution written once
- [x] `tools/w1_commission.py`: `--list`, `--identify`, `--check`
- [x] `tools/smart_agent.py` (`smartctl -j -n standby`), `deploy/aqua-bridge-smart-agent.service`, `publishers/inputs.py` (`SmartInbox`), MQTT `in/smart/<serial>`, `POST /api/in/smart`

#### Glue and UI

- [x] `control/loop.py`: read → step → compose → apply → watchdog; smoke-run on the Pi with `--source sim`
- [x] MQTT + HA discovery code (host + temps + fans), publisher thread, reconnects
- [x] MQTT + HA DAS entities: drive temperature / margin / σ, bay occupancy, noise index, model status, limits, experiments
- [x] HTTP API: `GET /api/state`, `/api/health`, `/api/estimate`, `/api/bays`, `/api/model`; `POST /api/mode`, `/api/setpoint`, `/api/limit`, `/api/bay`, `/api/pwm`, `/api/preset`, `/api/auto`, `/api/ident`, `/api/in/smart`
- [x] HTTP HTML: same five pages as Digole, poll `/api/state`; DEGRADED banner naming the zones
- [x] Tests: HTTP talks to the command sink, not hwmon; Auto rejects raw PWM; fuzz JSON → 4xx

## 9. Raspberry Pi packages and settings

Machine-readable list: `deploy/packages-rpi.txt` (Raspberry Pi OS Lite
32-bit, Trixie). `deploy/install-pi.sh` installs it (§10); by hand:

```bash
sudo apt-get update
sudo apt-get install -y $(grep -vE '^#|^$' deploy/packages-rpi.txt | xargs)
```

| Package | Why |
|---------|-----|
| build-essential gcc make pkg-config | build |
| git cmake flex bison libssl-dev bc | kernel modules / MPC tooling |
| linux-headers-rpi-v6 | out-of-tree modules (Zero / Pi 1 kernel) |
| i2c-tools python3-smbus | I2C / Digole |
| python3 python3-dev python3-pip python3-venv | daemon |
| python3-spidev python3-lgpio libgpiod-dev gpiod | GPIO/SPI |
| minicom | Digole UART debug |
| lm-sensors | `sensors`, hwmon |
| openssl | self-signed HTTPS certificate (`install-pi.sh`) |
| python3-hid python3-usb liquidctl | HID path, spike |
| python3-numpy python3-yaml | controller (runtime deps) |
| python3-aiohttp python3-paho-mqtt | HTTP, MQTT |
| python3-pytest python3-hypothesis | tests on the Pi |

The Python packages come from **apt**, so a venv with system
site-packages imports them. Versions on Trixie as verified on a Zero W:
Python 3.13.5, numpy 2.2.4, PyYAML 6.0.2, pytest 8.3.5, hypothesis
6.130.5, aiohttp 3.11.16, paho-mqtt 2.1.0. CI’s `pi-parity` job pins
exactly these (§12).

**venv vs apt (pick this, do not mix the other way):**

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -e . --no-deps   # aqua-bridge only; numpy/yaml/aiohttp/paho come from apt
```

A venv *without* `--system-site-packages` cannot see apt numpy. Do not
install numpy with pip on the Zero W unless you have to.
`install-pi.sh` uses `--no-deps` and aborts if the pip log shows numpy
being downloaded or built.

**pyproject ranges must accept the apt versions.** Even with
`--system-site-packages`, a `pip install -e .` without `--no-deps` pulls
numpy from PyPI and tries to build it if `Requires-Dist` excludes the
Debian package. Current ranges: `numpy>=2.2,<3` and `pyyaml>=6.0,<7`
(runtime); extras `http` (`aiohttp>=3.11,<4`), `mqtt`
(`paho-mqtt>=2.1,<3`) and `dev` (`pytest>=8.3,<9`,
`hypothesis>=6.130,<7`, `ruff==0.16.7`). Before changing them, on the Pi:

```bash
apt-cache policy python3-numpy python3-yaml python3-paho-mqtt python3-aiohttp
python3 -c "import numpy,yaml,paho.mqtt,aiohttp; print(numpy.__version__)"
```

piwheels **does** publish `numpy` cp313 `linux_armv6l` for Trixie (e.g.
2.5.3 as of 2026-09). Use it only if the Debian package is missing or
too old:

```bash
pip install numpy --extra-index-url https://www.piwheels.org/simple
```

Building numpy from source on a Zero W is not a fallback.

**pytest on the Pi:** a `--system-site-packages` venv has **no** `pytest`
console script (apt installs it to `/usr/bin` only), so always run
`.venv/bin/python -m pytest ...`. The full non-hardware suite takes about
two hours on the Zero W’s single core; it belongs to CI. The practical Pi
check is `HYPOTHESIS_PROFILE=pi .venv/bin/python -m pytest -m hardware`
plus a short `--source sim` smoke run (§11).

**sd_notify** (`sdnotify.py`): do not add `python3-systemd` or
`python3-sdnotify`. A UNIX datagram to `$NOTIFY_SOCKET` with the stdlib
(`@` prefix = abstract namespace). Unset (running in a terminal): no-op.
Every send failure returns `False`, never raises. `READY=1` once, after
the first tick whose command was applied; `WATCHDOG=1` after every tick
without a controller exception (read and apply failures included);
`STOPPING=1` after the stop write.

### HTTPS files

| Path | Mode | Created by |
|------|------|------------|
| `/etc/aqua-bridge/tls/` | `0755 root:root` | `install-pi.sh` |
| `/etc/aqua-bridge/tls/cert.pem` | `0644 root:root` | `install-pi.sh` (self-signed, only if neither file exists) or the owner |
| `/etc/aqua-bridge/tls/key.pem` | `0640 root:<service user>` | same |
| `/etc/aqua-bridge/http-users` | `0640 root:<service user>` | `tools/http_user.py --group <service user>` |

The daemon refuses a key or credentials file that others can read or the
group can write, or that belongs to an account other than root or the
service user (§6). The paths are the `http:`
defaults (`tests/test_deploy.py` checks the script against them).

### Overlays and modules

`/boot/firmware/config.txt` (`[all]` section):

```text
dtparam=i2c_arm=on
dtparam=spi=on
enable_uart=1
dtoverlay=disable-bt
```

When USB host is needed (`deploy/host-usb.sh` adds it idempotently):

```text
dtoverlay=dwc2,dr_mode=host
```

For the DS18B20 sensors, two buses (part of the base bring-up, §2):

```text
dtoverlay=w1-gpio,gpiopin=4
dtoverlay=w1-gpio,gpiopin=17
```

A third bus on `gpiopin=27` only if one bus carries more than about 12
sensors. The kernel creates one `w1_bus_master<N>` per overlay under
`/sys/bus/w1/devices`; `hw/onewire.py` discovers them and never needs the
GPIO numbers. **CPU note:** 1-Wire is bit-banged by the kernel with
busy-waits (about 12 ms per sensor read); 12–22 DS18B20 cost roughly
0.15–0.3 s of the single core per bulk cycle. At `dt = 5 s` and 12-bit
resolution that is a few percent of the CPU; measure with
`tools/w1_commission.py --check` and drop to 11 bit if a cycle exceeds
`0.4 × dt`.

I2C userspace module: `/etc/modules-load.d/i2c-dev.conf` → `i2c-dev`.

UART: `cmdline.txt` has no `console=serial0,115200` — the line is free for
Digole.

### User and sudo

- Dedicated Linux user in groups: `sudo, gpio, i2c, spi, dialout, plugdev, netdev`
- Optional: NOPASSWD sudo (`/etc/sudoers.d/`, visudo). Together with an
  unencrypted deploy key this is root on the Pi for anyone who can read
  the key. Acceptable only as a LAN toy; otherwise passphrase + ssh-agent
  (macOS keychain). Details in `private.md`.
- SSH public keys in that user's `~/.ssh/authorized_keys`

Account name and keys: `private.md`.

### Swap

1 GB `/swapfile` — needed for apt/builds on Zero W.

### systemd

`deploy/aqua-bridge.service` → `/etc/systemd/system/` (`install-pi.sh`
does it, substituting `User=`, then `daemon-reload` and
`systemd-analyze verify`):

```text
[Unit]
Description=aqua-bridge MPC fan controller (aquaero 6 XT + Quadro)
Wants=network-online.target
After=network-online.target

[Service]
Type=notify
NotifyAccess=main
# User= is a placeholder; install-pi.sh writes the real service account
User=aqua
SupplementaryGroups=dialout plugdev
ExecStart=/opt/aqua-bridge/.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml
StateDirectory=aqua-bridge
Restart=always
WatchdogSec=30
TimeoutStartSec=120

[Install]
WantedBy=multi-user.target
```

`StateDirectory=aqua-bridge` makes systemd create `/var/lib/aqua-bridge`
owned by `User=` and export it as `$STATE_DIRECTORY`: a DAS config keeps
its thermal model, SMART calibrations and last bay view in
`$STATE_DIRECTORY/model.json` (§3, *Model store*), written atomically at
most every `model_store_interval_s` and once at the clean stop. A legacy
config ignores it. Delete `model.json` for a clean prior (the
calibrations go with it).

The unit's `ExecStart=` above has no `--source`, i.e. `xt6` (a single
aquaero over hwmon, §3): that is the legacy install path, unchanged.
The DAS install path (`--source hwmon`, the composite `hwmon:` +
`onewire:` source) adds `deploy/aqua-bridge-das.conf` as a systemd
drop-in, `/etc/systemd/system/aqua-bridge.service.d/das.conf`, changing
only `ExecStart=` (an empty `ExecStart=` line clears the base unit's
before the real one is set — repeated-directive semantics,
`systemd.unit(5)`). `install-pi.sh --das` installs it (§10) and
`tests/test_deploy.py` checks that the drop-in's `ExecStart=` is the
base unit's plus exactly `--source hwmon`, nothing else.

(The file itself carries the reasoning as comments.) `After=` does not
pull in the target; `Wants=network-online.target` must sit next to it.
`Type=notify` is required or `READY=1` is ignored. `NotifyAccess=main`
means the **main** process is Python: `ExecStart=` is the venv
interpreter and `-m aqua_bridge` directly — no `sh -c`, no wrapper
script. `%h` would be the *manager*’s home (`/root` for a system unit),
not `User=`’s, hence the fixed install dir `/opt/aqua-bridge`, owned by
the service user.

`plugdev` is the group the udev rule gives write access to the hwmon
`pwmK` / `pwmK_enable` attributes (the kernel creates them `root:root
0644`); without it every `apply()` fails with `EACCES`. `dialout` is for
the HID path.

`READY=1` waits for the first applied command, so a device that is
absent at boot (USB not enumerated, hwmon permissions wrong) shows up as
a start timeout after `TimeoutStartSec=120` followed by `Restart=always`:
the visible “device absent” state. `WATCHDOG=1` every tick (`dt` 2 s in
the legacy example, 5 s in the DAS example), period far below
`WatchdogSec`. `Restart=on-failure` is not enough: OOM
and watchdog timeouts need `always`.

**Stop path:** the daemon installs a SIGTERM/SIGINT handler that only
sets a stop event and records the signal number — no logging, no I/O,
nothing that can raise. The main thread leaves the loop, logs the
signal, writes `fallback_pwm` once through the sink (`Loop.shutdown()`),
sends `STOPPING=1`, stops the publishers and exits 0. A second signal
during shutdown is only counted. (A handler that logged first could
re-enter the stderr writer when the signal landed inside a log write,
raise, never set the event, and leave systemd to SIGKILL the daemon
without the fallback write. That happened on the Pi and is covered by
`tests/test_main.py`.) Do **not** set `ExecStop=`. systemd runs
`ExecStop=` while the main process is still alive and sends SIGTERM after
that — two writers to hwmon at once.

**Deploy side effect (accepted):** `systemctl restart` is a clean stop, so
fans go to `fallback_pwm` (0.8 in the example). Coming back down at
`d_pwm_max=0.1` and `dt=2` takes on the order of **ten seconds**. The
unit comments say so, so a restart is not mistaken for a thermal event.

**Stop write ignores `d_pwm_max` (accepted):** the stop path writes
`fallback_pwm` in one step, not rate-limited. When the loop is hot and
the fans run above `fallback_pwm` (e.g. 1.0), a stop drops them to 0.8
at once. Accepted: the process is exiting and cannot ramp, 0.8 is a high
duty, and XT6 or the next daemon start takes over from there. Do not
"fix" this by skipping the write or by writing `max(last, fallback_pwm)`
without revisiting this decision. (Faults while running are different:
there the ramp target is `max(prev, fallback_pwm)`, §3.)

Pi **power loss** is still uncovered: last PWM stays on the fans until
XT6 firmware reverts (spike §2.3) or the board comes back. Accepted if
(3) fails.

### udev

`deploy/99-aquacomputer.rules` → `/etc/udev/rules.d/` (`install-pi.sh`
reloads the rules and re-triggers `hwmon` and the `0c70` USB devices so
an attached device gets them without a re-plug):

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="0c70", MODE="0660", GROUP="plugdev"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0c70", MODE="0660", GROUP="plugdev"
ACTION=="add", SUBSYSTEM=="hwmon", ATTRS{idVendor}=="0c70", RUN+="/bin/sh -c 'for f in /sys%p/pwm*; do [ -e $f ] && chgrp plugdev $f && chmod g+w $f; done'"
```

The first two are the HID path (liquidctl, the spike). The third is the
path the daemon writes: it hands `pwm*` (including `pwmK_enable`) of the
aquaero’s hwmon device to `plugdev` with group write.
`tempK_input` / `fanK_input` are world-readable already. Unverified on a
real device (§8).

---

## 10. Bring up on another Raspberry Pi

There is no project-specific image — flash Lite 32-bit (Zero W) or Lite
32/64-bit (Zero 2 W) and follow this checklist. Steps 1–6 have been run
on a Zero W; the hardware steps are waiting for the aquaero.

1. Flash Raspberry Pi OS Lite (Trixie), create a sudo user, enable SSH
   and Wi-Fi, set timezone and Wi-Fi country.
2. Overlays §9, `i2c-dev`, drop serial console from cmdline;
   `deploy/host-usb.sh` for the dwc2 host overlay (`--check` only
   reports; it backs up `config.txt` and says whether a reboot is
   needed). Reboot.
3. Groups, optional NOPASSWD, SSH keys (see `private.md`). The service
   account must exist before step 5.
4. 1 GB swap if Zero W.
5. Code: `sudo install -d -o USER -g USER /opt/aqua-bridge`, then rsync
   or clone the repo **there** (§11; no root needed after the chown).
   Then, from `/opt/aqua-bridge`: `deploy/install-pi.sh --user USER`
   (`USER` is the service account and its group; add `--das` for a DAS
   enclosure, see below). Idempotent. It
   installs the apt packages, creates the install dir and the
   `--system-site-packages` venv, runs `pip install -e . --no-deps` as
   the service user (aborting if pip tries to fetch numpy), installs
   `config.example.yaml` (`config.example-das.yaml` with `--das`) as
   `/etc/aqua-bridge/config.yaml` mode 640
   `root:USER` **only if absent** (the MQTT password must not be
   world-readable), creates a self-signed HTTPS certificate
   `/etc/aqua-bridge/tls/cert.pem` with key `key.pem` (mode 640
   `root:USER`, valid `--tls-days` days, default 3650) **only if neither
   file exists** (an owner-provided certificate is never overwritten;
   `openssl` comes from the package list), installs the udev rule and
   re-triggers it, installs the unit with `User=` substituted and, with
   `--das`, the `deploy/aqua-bridge-das.conf` drop-in (`--source hwmon`,
   §9) into `aqua-bridge.service.d/das.conf`, reloads
   systemd and verifies the unit (drop-in included). It does **not**
   enable or start the
   service, and it creates no HTTPS users (it prints the
   `tools/http_user.py` command). Run before the
   code is in place, it creates the directory, warns, skips the pip step,
   and finishes; rerun it after the rsync (`--das` on a rerun still only
   installs the config if none exists, but always re-installs the
   drop-in, so switching an existing legacy install to DAS is
   `install-pi.sh --user USER --das` plus step 6's hand-edit if
   `config.yaml` already existed).
6. Config. Without `--das` at step 5 (or to start from a fresh copy),
   install the DAS example by hand (the installer only installs
   `config.example.yaml` unless `--das` was given, and only when no
   config exists):
   `sudo install -m 640 -o root -g USER config.example-das.yaml
   /etc/aqua-bridge/config.yaml`. Either way, edit `mpc.channels` / `mpc.temps` /
   `mpc.sensors` / `mpc.topology` for the enclosure, the `hwmon:` devices
   (`fans` with `{pwm: pwmN, rpm: fanN}` per output, `temp_map` per
   thermistor input actually present), `onewire.sensors` (step 9), MQTT
   host and credentials (the MQTT command topics rely on the broker's
   authentication, §7), `http.enabled` / `mqtt.enabled`. Every name must
   be bound exactly once (the daemon exits 2 otherwise). Legacy mode:
   `xt6.fans` and `xt6.temp_map` with exactly the keys of `mpc.channels` /
   `mpc.temps`. Always pass `--config /etc/aqua-bridge/config.yaml`.
   **HTTPS users** (when `http.enabled: true`): `sudo
   /opt/aqua-bridge/.venv/bin/python /opt/aqua-bridge/tools/http_user.py
   --config /etc/aqua-bridge/config.yaml --group USER <name>` prompts for
   the password and writes `/etc/aqua-bridge/http-users` mode 640
   `root:USER`; rerun it to change a password or add a user (no restart).
   Without a certificate, key or user the API stays off and the journal
   says why; the fans are controlled regardless.
7. USB: dwc2 host, powered hub, XT6 on USB; the Quadro on aquabus, or on
   its own USB port if its PWM is not writable through the aquaero (§2).
8. `lsusb` / `sensors` — the USB spike (§2): attribute names and units,
   Quadro PWM, which Quadro temperature inputs exist, firmware revert.
   Check `ls -l /sys/class/hwmon/hwmon*/pwm*` shows group `plugdev` with
   write; run `.venv/bin/python -m pytest -m hardware` as the service
   user. Warm each mapped thermistor and watch the right `obs.temps` key
   move (a swapped `temp_map` is invisible to the gate, §3).
9. **Sensor commissioning** (DS18B20, once, before the daemon; `w1-gpio`
   overlays of §9 active):
   - `.venv/bin/python tools/w1_commission.py --list` prints every ROM
     id per bus with its current reading;
   - `.venv/bin/python tools/w1_commission.py --identify` samples every
     sensor while you warm one with a finger and ranks them by warming
     rate, so each ROM id is bound to its name (`prox_b01`, `inlet_b`, …)
     in `onewire.sensors`; repeat per sensor;
   - `.venv/bin/python tools/w1_commission.py --check --config
     /etc/aqua-bridge/config.yaml` builds the exact composite the daemon
     would (every name bound once, every ROM present) and reports the
     bulk-read cycle time per bus and the CRC error rate per sensor over
     20 cycles: aim for < 1 % and a cycle under `0.4 × dt`.
10. One diagnostic tick as the service user:
    `.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml --source hwmon --once`
    (legacy: without `--source`, which defaults to `xt6`) reads every
    source, prints the observation and the command, applies it, and ends
    with the stop write: the fans are left at `fallback_pwm` with
    `pwmK_enable` in manual mode.
11. DAS: the unit's `ExecStart=` has no `--source`, i.e. `xt6`. `--das`
    at step 5 already installed the `--source hwmon` drop-in
    (`deploy/aqua-bridge-das.conf`, §9); confirm with `systemctl cat
    aqua-bridge` (its `ExecStart=` lines show the override). Without
    `--das`, add it by hand: `sudo systemctl edit aqua-bridge` — an
    empty `ExecStart=` line, then the full `ExecStart=` line with
    `--source hwmon` — then `sudo systemctl daemon-reload` and `sudo
    systemd-analyze verify /etc/systemd/system/aqua-bridge.service`.
    Optionally add `--record
    /var/lib/aqua-bridge/rec.jsonl` or set `record_path` for the model
    ladder (§13).
12. `sudo systemctl enable --now aqua-bridge`; `journalctl -u aqua-bridge`
    (a start timeout loop means the device or its permissions are
    missing; the model store logs whether `model.json` loaded `fresh`,
    `stale` or `prior`); `https://<host>:<http.port>/api/health` (e.g.
    `curl -k -u <name> https://<host>:8443/api/health`; `-k` for the
    self-signed certificate) and, in DAS mode, `/api/estimate` (every bay
    occupied or empty as physically true, no zone in fault) if HTTP is
    enabled; MQTT entities in HA; Digole later.
13. Optional, on the PC with the DAS: the SMART agent,
    `python tools/smart_agent.py --mqtt HOST --node-id aqua-bridge
    --interval 60` or the user unit `deploy/aqua-bridge-smart-agent.service`
    (edit its paths and broker first; smartctl usually needs root or
    capabilities). `--node-id` must match `mqtt.node_id`.

Zero W: 32-bit only. Do not install a desktop.

The Pi should be reachable on the LAN (mDNS `hostname.local` or DHCP name).

---

## 11. Development: host PC ↔ Pi

Host PC: git, tests, simulator, edits.
Pi: USB, PWM, touch, MQTT on the live loop.

Concrete hostnames and the rsync/ssh lines: `private.md`.

On the development machine (Python ≥ 3.13):

```bash
python3 -m venv .venv
.venv/bin/pip install -e ".[dev,http,mqtt]"
.venv/bin/ruff check . && .venv/bin/ruff format --check .
.venv/bin/python -m pytest -m "not hardware and not nightly"   # what PR CI runs; HYPOTHESIS_PROFILE=dev by default
.venv/bin/python -m pytest -m nightly               # the sweeps the nightly job adds
.venv/bin/python -m aqua_bridge --config config.example.yaml --source sim --ticks 5 --sim-speed 0
.venv/bin/python tools/bench_step.py                # legacy: step() timing per solver, JSON
```

DAS mode on the development machine (the DAS truth plant built from the
zoned config; `sim.das: {preset: rich, seed: 3}` in a copy of the config
selects the rich preset):

```bash
# simulate 5 hours at full speed, recording every tick (add model_shadow: true to learn in shadow)
.venv/bin/python -m aqua_bridge --config config.example-das.yaml --source sim --sim-plant das \
  --sim-speed 0 --ticks 3600 --record /tmp/rec.jsonl --model-store /tmp/model-store.json
# offline zoned model fit: windowed LS, air-block output-error refinement, hold-out validation
.venv/bin/python tools/fit_model.py --config config.example-das.yaml --topology --out /tmp/model.json /tmp/rec.jsonl
# PWM -> RPM curve per fan model (copy the result into fan_models)
.venv/bin/python tools/fit_fans.py --config config.example-das.yaml --out /tmp/fan_curves.json /tmp/rec.jsonl
# prediction errors: learning as it goes, or a fitted model frozen
.venv/bin/python tools/replay.py --config config.example-das.yaml /tmp/rec.jsonl
.venv/bin/python tools/replay.py --config config.example-das.yaml --model /tmp/model.json --frozen /tmp/rec.jsonl
# step() timing of both DAS solvers against the DAS plant (budget_ms / budget_alarm_ms in the report)
.venv/bin/python tools/bench_step.py --sim-plant das --solver pi --solver mpc
```

Golden trajectories change only on purpose:
`AQUA_BRIDGE_REGEN_GOLDEN=1 .venv/bin/python -m pytest tests/test_das_core.py`
regenerates the DAS goldens (`das_*.pi_das.json`, `das_*.mpc_das.json`),
then review the diff. The legacy goldens (`tests/test_mpc_nominal.py`)
are never regenerated: they are the bit-identity check of legacy mode.

To the Pi:

```bash
# from the repo root on the development machine
rsync -az --delete --exclude .venv --exclude __pycache__ \
  --exclude private.md --exclude secrets \
  ./ USER@PI-HOST:/opt/aqua-bridge/

ssh USER@PI-HOST 'cd /opt/aqua-bridge && HYPOTHESIS_PROFILE=pi .venv/bin/python -m pytest -m hardware'
ssh USER@PI-HOST 'cd /opt/aqua-bridge && .venv/bin/python -m aqua_bridge --config config.example.yaml --source sim --ticks 10'
ssh USER@PI-HOST 'cd /opt/aqua-bridge && .venv/bin/python tools/bench_step.py'
ssh USER@PI-HOST 'cd /opt/aqua-bridge && .venv/bin/python tools/bench_step.py --sim-plant das'
ssh USER@PI-HOST 'cd /opt/aqua-bridge && HYPOTHESIS_PROFILE=pi .venv/bin/python -m pytest tests/test_bench_budget.py -m pi'
```

The DAS bench on the Pi is the budget check of §3: p99 ≤ 600 ms at
`dt = 5 s` with `mpc_every_ticks: 2`, else apply the reductions listed
there.

Rerun `deploy/install-pi.sh --user USER` on the Pi when `pyproject.toml`,
the unit or the udev rule changed. Or `git pull` from GitHub on the Pi.
Avoid committing from a Zero W (slow card, easy to mix up branches).

Running the daemon by hand on the Pi (stop the service first: two
processes would both write hwmon):

```bash
ssh USER@PI-HOST
cd /opt/aqua-bridge
.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml
```

Ctrl-C takes the same stop path as SIGTERM.

---

## 12. GitHub workflow

The repo is **public** on GitHub; `origin` is HTTPS. Need a GitHub
account and `gh` on the development machine.

```bash
# macOS
brew install gh git
gh auth login
gh auth status
gh repo clone <USER>/aqua-bridge
```

SSH remote (optional). Use a **dedicated** filename so it does not
overwrite `~/.ssh/id_ed25519`:

```bash
ssh-keygen -t ed25519 -C "aqua-bridge-github" -f ~/.ssh/id_ed25519_github_aqua-bridge
gh ssh-key add ~/.ssh/id_ed25519_github_aqua-bridge.pub --title "aqua-bridge github"
```

`ssh` does **not** try non-default key names. Add `~/.ssh/config` or
HTTPS will keep working and SSH push will hang or ask for a password:

```text
Host github.com
  HostName github.com
  User git
  IdentityFile ~/.ssh/id_ed25519_github_aqua-bridge
  IdentitiesOnly yes
```

Commits use the GitHub **noreply** address
(`git config user.email <ID>+<USER>@users.noreply.github.com`), never a
personal one: the history is public.

**`main` is protected:** required status checks `lint`, `test (latest)`
and `test (pi-parity)`; the branch must be up to date with `main` before
merging; linear history; no force pushes; the rules apply to admins too.
Only **rebase merges** are enabled, and branches are deleted on merge.

Day to day:

```bash
git checkout -b <topic> origin/main
git push -u origin <topic>
gh pr create --fill
gh pr checks --watch
git fetch origin && git rebase origin/main && git push --force-with-lease   # when main moved
gh pr merge --rebase
```

One topic per branch. Track B stays on its own branches, not mixed with
the core. `main` only gets what is green in CI and does not break the
`model.py` contract. A config without the DAS sections must behave bit for
bit as before, and the legacy goldens never move. The code arrived as
three PRs — `mpc-core` (contract, gate, PI and MPC, CI), `hw-xt6` (hwmon
adapter), `glue` (loop, supervisor, entry point, HTTP, MQTT, deploy) —
followed by spec and config updates (`spec`, `readme-badges`, `xt6-fans`,
`das-premise`) and the DAS milestones, one PR each: `das-truth-sim`,
`onewire-source`, `zones-gate`, `smart-agent`, `pi-das`, `estimator`,
`thermal-model-das`, `record-and-fit`, `mpc-das`, `ident-experiments`,
`model-store`, and `docs-das` (this documentation pass). The remaining
milestone, `hw-validation`, needs the enclosure (§8, §13).

### CI (`.github/workflows/ci.yml`)

Triggers: push to `main`, every pull request, a nightly schedule
(`17 3 * * *`), and `workflow_dispatch`. Permissions `contents: read`.
Superseded PR runs are cancelled; runs on `main` and the nightly never
are.

| Job | When | What | Timeout |
|-----|------|------|---------|
| `lint` | push, PR, dispatch | Python 3.13, `ruff==0.16.7`: `ruff check .`, `ruff format --check .` | 5 min |
| `test (latest)` | push, PR, dispatch | Python 3.14, `pip install -e ".[dev,http,mqtt]"`, `HYPOTHESIS_PROFILE=ci`, `pytest -m "not hardware and not nightly" --durations=15` | 15 min |
| `test (pi-parity)` | push, PR, dispatch | Python 3.13 with the Pi’s apt versions pinned (numpy 2.2.4, pyyaml 6.0.2, pytest 8.3.5, hypothesis 6.130.5, aiohttp 3.11.16, paho-mqtt 2.1.0), `pip install -e . --no-deps`, same pytest | 15 min |
| `nightly-fuzz (latest, pi-parity)` | schedule, dispatch | same installs, `HYPOTHESIS_PROFILE=nightly` (randomized, 1000 examples), `pytest -m "not hardware"`: everything the PR jobs run plus the `nightly` sweeps | 60 min |

**Selectors.** PR and `main` runs exclude `hardware` (no device on a
runner) and `nightly` (seed × placement sweeps of the identification,
8-seed noise sweeps on the basic and rich presets, long zone-fallback
sweeps). The nightly job keeps only `hardware` out. Keep PR runs under
about 11 minutes of the 15-minute job timeout: a new heavy sweep or long
simulation gets the `nightly` marker, with a cheap PR-sized case next to
it. The **budget gate** in PR CI is relative: `tests/test_bench_budget.py`
asserts the DAS MPC step p99 is at most 12× the legacy MPC p99 (§8 item 6:
the 75th percentile of several interleaved, warm-up-discarded repeats, not a
single measurement, so a shared runner's hiccup does not flake it); the
absolute `mpc.budget_ms` gate (marker `pi`) runs only on the Pi.

The ruff pin in `pyproject.toml` `[dev]` and in the workflow move
together. `shellcheck` is present on the runners, so `tests/test_deploy.py`
runs it. First runs took about 8 s for `lint`, 4–6 min for
`test (latest)` and 3–5 min for `test (pi-parity)`; with the DAS suites
the PR jobs take about 7–10.5 min (`test (latest)`, runner speed varies)
and 5–9 min (`test (pi-parity)`), close to the 11-minute guideline. A nightly failure prints a `@reproduce_failure`
blob in the log.

---

## 13. Rollout order (with parallelism)

1. **Done.** GitHub repo exists, public, protected `main` (§12).
   Site-specific inventory in `private.md`.
2. **In parallel:** the control core on the dev machine **and**, on the
   Pi, the USB spike (hub, XT6, hwmon, Quadro PWM and inputs, **firmware
   revert**) plus **DS18B20 and sensor commissioning** (`w1-gpio` buses,
   `tools/w1_commission.py --list / --identify / --check`, §10). Neither
   needs the other. Core **done** (legacy PI and MPC, and the whole DAS
   track: zones, estimator, PI-like DAS form, thermal model, DAS MPC,
   store, experiments; CI). Spike and commissioning **open**: the
   aquaero, the Quadro and the sensors are not connected.
3. Hardware adapters once the spike answers the Quadro PWM **and** revert
   questions. **Written** against the assumed hwmon and `w1_therm` ABIs
   and fake trees (`hw/xt6.py`, `hw/sources.py`, `hw/onewire.py`);
   confirm on the devices, adjust the bindings or the adapters if the
   spike disagrees.
4. Glue loop, MQTT. **Code done**; Pi provisioned with `install-pi.sh`,
   sim smoke run and benchmark on the Zero W; service left disabled; no
   live broker yet.
5. HTTP API + HTML (same pages as Digole). **Done** (Host section and
   drive views gap, §8).
6. Hardware bring-up of the DAS: spike, hwmon permissions, `-m hardware`,
   bind every sensor, `--source hwmon` drop-in, DAS step budget on the
   Pi, enable the service, MQTT/HA live check, SMART agent on the PC.
7. **Model ladder** on the running enclosure, each stage ≥ 24 h unless
   stated, one change at a time:

   | Stage | Config | Decide by | Watch |
   |-------|--------|-----------|-------|
   | 0 commission + record | `solver: pi` (PI-like DAS), `zones.trust_rule: strict`, `record_path` | zero zone faults from sensors, bulk cycle time, every bay `occupied` / `empty` as physically true, no drive over its soft target | `/api/estimate`, `/api/bays`, `/api/health`, journal |
   | 1 offline fit + fan curves | – | `tools/fit_model.py`: `E` per group and per-bay `k` pinned (relative SE under `model_converged_rel_se`); `tools/fit_fans.py` RMS < 5 % rpm, copied into `fan_models`; spare thermistors moved to the bays the fit ranks tightest | fit and replay reports |
   | 2 SMART calibration (if used) | agent on the PC | most bays `calibrated`, `σ_cal` ≤ 0.7 °C, calibrated estimates within 2 °C of SMART, associations match the physical bays | HA `drive_sigma_*`, `/api/model` calibration, `/api/bays` |
   | 3 shadow + experiments | `model_shadow: true`, `ident_enabled: true`, store on; experiments one group at a time under PI-DAS | every zone `converged`, prediction error < 0.5 °C, no experiment abort on the envelope | HA `model_status`, `model_pred_err_c`, `ident_running`, `/api/model` |
   | 4 MPC | `solver: mpc` | the validity gate keeps `active: mpc` (no model fallbacks), no `degraded`, `noise_db` lower than stage 0 at equal or better `drive_margin_*`, step p99 under budget | `/api/health`, `solver_diag.model`, `noise_db`, `drive_margin_*` |
   | 5 restart check | restart the daemon | `model.json` loads `fresh` and the zones `frozen`; after > `model_store_max_age_days` offline it loads `stale` and re-confirms in shadow before the MPC acts | `/api/model` `store`, journal |
   | 6 `trust_rule: sigma` | after it is implemented (§8) | fewer zone faults than `strict`, no violation | zone graphs |

   **Rollback**, one line each: `solver: pi` (PI-like DAS); `model_shadow:
   false`; `ident_enabled: false`; `zones.trust_rule: strict`; delete
   `$STATE_DIRECTORY/model.json` for a clean prior (the calibrations go
   with it). Removing `topology` returns to legacy mode only with a
   legacy-shaped config (setpoints, `xt6:`). Every restart passes the
   `fallback_pwm` stop write (about ten seconds of loud fans, accepted,
   §9).
8. Digole + touch.
9. Zero 2 W with no API change.
