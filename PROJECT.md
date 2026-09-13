# aqua-bridge

Bridge and **MPC fan controller** on a Raspberry Pi for Aqua Computer
**aquaero 6 XT** + **Quadro** (aquabus), **Digole** (touch) as the display,
telemetry to **Home Assistant**. Direct DS18B20 sensors come last.

Develop on a desktop or laptop. The Pi is runtime and hardware.

**Project language is English only** — docs, comments, commit messages,
issues, and PR descriptions.

Hosts, usernames, and LAN details belong in `private.md` (gitignored), not
in this file.

---

## 1. Purpose

| Priority | What |
|----------|------|
| 1 | MPC (or PI first, same API) sets fan PWM |
| 2 | Digole + touch: pages, override, diagnostics |
| 3 | MQTT + HA discovery: sensors, temperatures, Pi health, setpoint (not raw PWM in Auto) |
| 4 | HTTP: view state and control (setpoint, mode, manual PWM) |
| later | DS18B20 on GPIO |
| upgrade | same codebase on Zero 2 W |

Aquaero and Quadro run autonomously. The Pi is the controller. If the
daemon dies, fans must not stay pinned at the last USB PWM. XT6 firmware
curves (or an aquaero software-sensor timeout, see the USB spike) are the
hardware watchdog. That behaviour is **not** assumed: the spike must
measure it.

---

## 2. Hardware and topology

```
XT6 sensors + Quadro sensors (aquabus)
                 │
                 ▼
         USB HID: XT6 only
                 │
            Raspberry Pi
         (MPC → PWM setpoints)
                 │
                 ▼
         USB → XT6 → aquabus → Quadro fans
                 │
        ┌────────┼────────┐
        ▼        ▼        ▼
     Digole    MQTT/HA   (DS18B20 later)
     touch
```

- **Quadro on aquabus**, master is **XT6**. Only XT6 is on USB to the Pi
  (`lsusb`: vendor `0c70`, product `f001`). Quadro (`f00d`) is not needed
  on the Pi.
- A **powered USB hub** is still required: Zero W OTG is flaky; do not
  power devices from the board port. XT6/Quadro 12 V is their own supply.
- Digole: UART `/dev/serial0` (= `ttyAMA0`), plus I2C/SPI if wired that
  way. Bluetooth is off (`dtoverlay=disable-bt`); serial console is
  removed from UART.
- DS18B20: GPIO4 + 4.7 kΩ, `dtoverlay=w1-gpio` — **last**.
- Zero 2 W upgrade: same 32-bit userland or 64-bit Lite, no `armv6` in
  the code. USB OTG is unchanged.

### Risk (first-evening USB spike)

The `aquacomputer_d5next` driver (already in 6.18) exposes aquaero’s
**own 4 fans**. Quadro channels over aquabus may be sensors only, with
no PWM write. Until verified:

1. `sensors` and `/sys/class/hwmon/` — which `temp*`, `fan*`, `pwm*` exist.
2. Whether Quadro PWM is writable through XT6 (hwmon or HID).
3. **Does XT6 revert on its own after the Pi stops writing?** Write a PWM
   via hwmon/HID, then stop. If the fan stays pinned, firmware curves are
   *not* a watchdog. Then try the alternative: write the controller output
   into an aquaero **software / virtual sensor**, with the firmware’s
   timeout fallback and an identity curve onto PWM. That is a hardware
   watchdog for free — only if the Linux driver or liquidctl can write
   that sensor. Record which path works; the daemon follows it.

If (2) fails: HID writes into XT6, or temporarily a second USB device
(breaks the chosen topology). This blocks the hardware track only, not
the MPC core.

If (3) fails, a **software** watchdog inside the daemon (ramp to
`fallback_pwm`) only covers faults the process can still act on. It does
**not** survive SIGKILL, OOM-kill, USB-hub dropout that takes the HID
write path with it, or **Pi power loss**. Residual risk, state it
plainly: PWM can sit at a **low** last value; then the PC load rises and
nothing ramps the fans. Mitigations that still run on the Pi:

- systemd `WatchdogSec=` + `sd_notify` (`WATCHDOG=1` every loop);
  `Restart=always` (not `on-failure`)
- on clean stop: a **SIGTERM handler in the daemon** writes
  `fallback_pwm` then exits. Do not use `ExecStop=` (see §9)

Those shrink hang/crash windows. They do **not** cover Pi power loss.
That is an **accepted risk** unless spike (3) proves XT6 firmware reverts
on its own.

---

## 3. Architecture: two independent tracks

Hardware **must not** import MPC. MPC **must not** know USB, Digole, or MQTT.

### Contract (`src/aqua_bridge/model.py`)

```text
PlantObservation
  temps: dict[str, float]   # °C, logical names
  rpm:   dict[str, float]
  pwm:   dict[str, float]   # actual 0..1
  ts:    float              # monotonic cycle seconds, not wall clock

MpcConfig
  dt, horizon
  setpoints: dict[str, float]
  weights, pwm_min/max, d_pwm_max
  fallback_pwm: dict[str, float]  # high cooling; each value > pwm_min
  fallback_hold_s: float          # hold last good this long, then ramp to fallback_pwm
  confirm_ticks: int              # consecutive trusted ticks before leaving fallback; >= 2
  dT_max: float                   # °C per tick; gate rate limit
  channels: tuple[str, ...]       # e.g. ("radiator", "intake", ...)

MpcState
  last_cmd: MpcCommand | None
  last_good_obs: PlantObservation | None
  last_raw_temps: dict[str, float] | None  # previous raw sample, even if untrusted
  fault_since_ts: float | None    # first ts any fallback cause became active
  fault_reason: sensor_gate | solver | None
  trusted_streak: int             # consecutive trusted ticks; reset on untrusted
  integrator: dict[str, float]    # PI / MPC internals; see bumpless transfer below
  # any other solver memory; must be serialisable for tests

MpcCommand
  pwm: dict[str, float]      # 0..1
  mode: auto | saturated | fallback
  diagnostics: dict
```

Channel names are logical (`radiator`, …). Mapping onto `hwmon pwmN`
lives only in the hardware adapter.

One controller step is a pure function of observation, config, **and
state**. It returns the command **and** the next state (needed for PI
integrator, last-good PWM, fault timers):

```text
mpc.step(observation, config, state) -> (command, state)
```

No files, no sockets, no `time.time()` inside the solver (`observation.ts`
and `config.dt` are the clock). Tests inject `state`. Same
`(observation, config, state)` → same `(command, state)` (deterministic).

**Safe PWM on untrusted observation (cooling loop):** never jump toward
`pwm_min`. That would cut cooling and would also violate `|Δpwm| <=
d_pwm_max` if the last command was high.

1. Hold `state.last_cmd` (rate-limit vs that command, not vs `pwm_min`).
2. If the fault lasts longer than `fallback_hold_s`, ramp each channel
   toward `config.fallback_pwm` (a **high** duty, typically near
   `pwm_max`) at `d_pwm_max` per step.
3. **First tick**, no `last_cmd`, and `obs.pwm` is missing or untrusted:
   `prev = config.fallback_pwm`, `cmd.pwm = config.fallback_pwm`
   (`Δ = 0`). `assert_command_safe(..., prev_cmd=fallback_pwm)` must
   see that same dict — do not invent a rate-limit against `pwm_min` or
   against an empty last command.
4. First tick with **trusted** `obs.pwm`: `prev = obs.pwm` as usual.
5. `mode=fallback` for the whole time a fallback cause is active.

**One** `fault_since_ts` covers every reason `step` cannot emit a trusted
`auto` command: sensor gate **or** solver exception / iteration cap.
`fault_reason` records which. The hold/ramp-high policy is identical.
Do not run two independent hold timers.

Clear `fault_since_ts` / `fault_reason` and return to `auto` only after
`confirm_ticks` **consecutive** trusted ticks (`trusted_streak`). One
good sample is not enough — that is the same “until confirmed” rule as
Jump in §4.4. A sensor that lies every other tick (Flicker) must keep
`mode=fallback` and must not reset the hold timer. Any untrusted tick
sets `trusted_streak = 0`.

**Trusted tick (sensor gate):** a sample is trusted iff it is in the
absolute valid range **and** `|T - ref| <= dT_max` for **at least one**
of: `ref = last_good` **or** `ref = previous raw sample` (`last_raw_temps`).
Always store this tick’s raw value into `last_raw_temps`, trusted or not.

- **Spike** (+125 °C for one sample, then back): one untrusted tick
  (far from both refs). The next sample is near last good → streak
  resumes; last good does not move.
- **Jump** (real +30 °C that then holds): the first new-level sample is
  far from last good *and* from previous raw → untrusted. Following
  samples are near **previous raw** (the new level) even though still
  far from last good → they **are** trusted. When `trusted_streak`
  reaches `confirm_ticks`, set `last_good_obs` to this observation
  (the new level is now last good). If the new level never confirms
  before `fallback_hold_s`, ramp high as usual.

A gate that only compares to last good would keep Jump untrusted
forever and slam fans to `fallback_pwm`. That is a spec bug; do not
implement it.

**Bumpless transfer (integrator):** while `mode=fallback`, the solver
**must not** update `integrator` (no windup on an unused output). On
the tick that returns to `auto`, re-initialise the integrator so the
solver’s first auto output **equals** `last_cmd.pwm` (before rate
limit). After a long fallback, PWM must not rail because I accumulated
in the dark, and must not jump because I was left at a stale value.

### Track A — core (dev machine / CI, no Pi)

- `control/mpc.py`, `sim/plant.py` (package), `tests/test_mpc_*.py`
- RC thermal plant in the simulator (`aqua_bridge.sim.plant`).
- First `step()` may be PI with the same contract; swap in MPC later.
- On Zero W keep the solver small: 2–4 temperatures, 4–8 PWM, dt 1–2 s,
  horizon 10–20 s, numpy, no CasADi/IPOPT. acados after 2W.

### Track B — hardware (Pi for USB; mapping on any machine)

- `hw/xt6.py`: `read() -> PlantObservation`, `apply(MpcCommand)`
- `hw/map.py`: logical name → sysfs (root configurable, default
  `/sys/class/hwmon`)
- Unit tests against a **fake hwmon tree** in a temp directory (CI, no
  Pi). `pytest.mark.hardware` only for live USB.
- **Does not import mpc.**

### Glue (when both tracks are stable)

```text
cmd, state = mpc.step(obs, cfg, state)
sink.apply(cmd)
```

Digole and MQTT subscribe to `(obs, cmd)`, not to HID.

### Repo layout

```text
aqua-bridge/
  PROJECT.md                 # this file
  README.md
  pyproject.toml
  config.example.yaml
  deploy/
    packages-rpi.txt
    aqua-bridge.service
    99-aquacomputer.rules
    host-usb.sh
  src/aqua_bridge/
    model.py
    control/mpc.py
    control/loop.py          # glue — later
    hw/xt6.py
    hw/map.py
    sim/plant.py             # inside the package so `pip install -e .` sees it
    ui/digole/               # after Command is stable
    publishers/mqtt_ha.py
    publishers/http.py       # REST + HTML, same intents as Digole
  tests/
    conftest.py
    test_mpc_invariants.py
    test_mpc_nominal.py
    test_mpc_failures.py
    test_mpc_sensor_faults.py
    test_mpc_fuzzy.py
    test_mpc_closedloop.py
    test_http_api.py
    test_hw_map.py           # fake sysfs, CI
    test_hw_xt6.py           # fake hwmon in CI; live USB with pytest.mark.hardware
```

CI on GitHub runs every test except `hardware`. Marker `fuzzy` may be
`slow`; still required in CI (cap examples, not unbounded). Live USB tests
run only on the Pi. Fake-hwmon tests run everywhere.

---

## 4. Testing

MPC is the primary function of the board. A green USB spike does **not**
replace this. Track A is not done until the suites below exist and CI
runs them on every PR.

`mpc.step(obs, config, state)` is a pure function: tests never open
sockets, files, or HID. Time comes from `observation.ts` and `config.dt`.
Pass `state` in; take `state` out.

### 4.1 Invariants (assert on every `step` and every closed-loop tick)

These are the contract. Nominal, failure, lie, and fuzzy tests all check
them. Put them in one helper (`assert_command_safe(obs, cfg, cmd, prev_cmd)`):

- `cmd.pwm` has **exactly** `config.channels` — no extras, no missing keys
- every PWM is finite and in `[pwm_min, pwm_max]`
- per channel `|pwm_k - prev_pwm_k| <= d_pwm_max`. `prev` is, in order:
  `state.last_cmd`; else trusted `obs.pwm`; else **`config.fallback_pwm`**
  (first untrusted tick, §3 item 3). **Never** `pwm_min`.
- no `NaN` / `Inf` in PWM, diagnostics, or returned state
- `cmd.mode` is one of `auto | saturated | fallback`
- untrusted observation → `mode=fallback` and the hold-then-high policy
  in §3 (never a step toward `pwm_min` *because* of the fault)
- same `(obs, config, state)` → same `(cmd, state)` (deterministic)

A test that produces a command which violates an invariant is a failure
even if the “story” of the test passed.

### 4.2 Nominal (ordinary operation)

`tests/test_mpc_nominal.py`, closed-loop via `aqua_bridge.sim.plant` where needed.

- **At setpoint:** temps already at target → PWM settles; ripple below a
  bound for N steps
- **Step up / step down:** setpoint change → PWM moves the right way;
  temp crosses toward the target within a deadline (sim)
- **Disturbance:** extra heat in the plant → PWM rises, then recovers
- **Cold start:** obs PWM 0, temps high → ramps up without skipping
  `d_pwm_max`
- **Warm start:** already at `pwm_max`, temps then fall → ramps down
- **Multi-channel:** two+ fans; one channel can increase while another
  holds; no silent drop of a channel
- **Saturation is honest:** if the plant needs more than `pwm_max`,
  `mode=saturated`, PWM pinned at max, no exception
- **Golden trajectories:** dump `(t, temps, pwm)` for a fixed seed and
  plant; regression with a numeric tolerance (not bitwise float)

First implementation may be PI. These tests stay; swapping in MPC must
not delete them.

### 4.3 Failures (everything that is not a lying sensor)

`tests/test_mpc_failures.py` — one case per fault, plus combinations.

Observation / time:

- missing temp key, extra unknown key, empty `temps`
- `rpm` / `pwm` missing for a channel that exists in config
- `ts` not advancing, `ts` going backwards, gap `>> dt` (missed ticks)
- duplicate `step` with identical `ts`

Values:

- `None` in a dict value (if the type allows before validation)
- out-of-range temps (e.g. < -20 °C or > 120 °C) as **invalid**, not
  as a real plant
- PWM obs outside `[0, 1]`
- RPM = 0 for many steps while commanded PWM is high (stall / unplugged
  fan) → must not wind up forever; diagnostics + cap or fallback

Config / solver:

- `horizon = 1` and a large horizon (must still satisfy invariants)
- `dt = 0` or negative → reject config, do not step
- `pwm_min > pwm_max` → reject config
- empty `channels`
- `fallback_pwm` keys **exactly** `channels`
- each `fallback_pwm[ch]` in `(pwm_min, pwm_max]` — equal to `pwm_min`
  is invalid (`never pwm_min` is a rejected config, not a comment)
- `fallback_hold_s >= 0`
- `confirm_ticks >= 2` (integer)
- `dT_max > 0`
- solver exception or iteration cap → same hold/ramp-high as the sensor
  gate (`fault_reason=solver`), **no throw** out of `step`
- cold `state` (`None` last_cmd) on first call: if obs is untrusted,
  command **equals** `fallback_pwm` (see §3 item 3)

Loop / glue (once `loop.py` exists, still no HID):

- `read()` raises or returns empty → do not call `apply` with garbage;
  fallback command or skip with watchdog
- `apply()` raises → next tick still runs; do not leave solver state
  half-updated

### 4.4 Lying sensors (fault injection)

`tests/test_mpc_sensor_faults.py`

Watercooling sensors lie. The controller must **not** chase a lie to
`pwm_max` in a few ticks (`d_pwm_max` is necessary but not sufficient).

Inject on top of an otherwise nominal closed-loop:

| Lie | Example | Expected |
|-----|---------|----------|
| Stuck | value frozen while plant T moves | fault flag; do not treat as at-setpoint |
| Spike | single sample −40 °C or +125 °C | ignore sample; rate-limit PWM |
| Jump | step +30 °C and stay | first tick untrusted; next ticks trusted vs previous raw; after `confirm_ticks` last good := new level (not stuck in fallback) |
| Drift | slow bias +0.05 °C/step | must not silently walk PWM to the rail without bound (document acceptable lag) |
| Swap | coolant/air keys exchanged | disagreement → fallback; hold then ramp **high**, never min |
| Impossible combo | coolant 18 °C, “air” 90 °C, PWM 1.0 | untrusted |
| Impossible dT/dt | +15 °C in one `dt` | reject sample |
| Dropout | `None` / missing for k steps, then back | no NaN; resume without a PWM spike |
| Raw garbage | 32767 / 0xFFFF leftovers, millidegree passed as °C | out of range → untrusted |
| Flicker | alternate good/bad every step | `mode` stays `fallback` (streak never reaches `confirm_ticks`); PWM must not chatter at `d_pwm_max` each tick; hold timer must not reset |
| Constant noise | ±2 °C Gaussian | still stable; no chatter at `d_pwm_max` |

Define an explicit **sensor gate** in front of the solver using the
trusted-tick rule above (absolute range, `dT_max` vs last good **or**
previous raw, `confirm_ticks` before adopting a new last good). Tests
target that gate **and** the full `step`. Include a Jump closed-loop
that must *not* stay in fallback forever. Do not hide filtering only
inside numpy and leave it untested.

### 4.5 Fuzzy / property tests

`tests/test_mpc_fuzzy.py` — Hypothesis (CI) or an equivalent
property runner. Mark `pytest.mark.fuzzy`.

Properties (N random examples per test, shrinking on failure):

- **Random valid obs** in plausible boxes (temps 10–80 °C, pwm 0–1,
  rpm 0–4000, `ts` increasing by `dt`) → invariants hold
- **Random sequences** of 20–100 steps, each obs a small perturbation of
  the last → invariants every tick; PWM total variation bounded
- **Random lies mixed in:** with probability p, apply a lie from §4.4
  → still invariants; `mode=fallback` when the gate says untrusted
- **Random setpoints** in a sane band (e.g. 25–45 °C)
- **Random plant parameters** in closed-loop (time constants, gain) →
  no NaN, no PWM outside limits (stability/overshoot asserts may be
  looser than goldens)
- **Malformed Observation construction:** extra keys, wrong types —
  either reject (TypeError/ValueError) or fallback; never a silent
  bad PWM

Cap Hypothesis `max_examples` so CI is minutes, not hours. Failures
must shrink to a replayable seed; store the seed in the test output.

HTTP (when the API exists): fuzz JSON bodies for `/api/pwm` and
`/api/setpoint` (missing fields, strings, arrays, out of range) → 4xx,
never a 500 and never a command that breaks invariants.

### 4.6 Closed-loop sim

`tests/test_mpc_closedloop.py` + `aqua_bridge.sim.plant`

- Controller model **≠** plant (gain/time-constant mismatch) — still
  bounded PWM and no crash
- Actuator delay (commanded PWM appears in `obs.pwm` 1–2 ticks late)
- Fan stall: plant RPM 0 regardless of PWM
- Long run (e.g. 15 min of sim at `dt=2`) — no drift to NaN
- Bumpless: long fallback then return to auto — first auto PWM equals
  `last_cmd`, integrator did not wind (`test_mpc_failures.py`)

### 4.7 Hardware-adapter tests (track B)

**No mpc import.** Mapping and the adapter must not wait for the board.

- `tests/test_hw_map.py` — logical names ↔ a fake hwmon directory tree
  (temp files). Runs in CI.
- `tests/test_hw_xt6.py` — same fake tree for `read`/`apply` (write a
  file, read it back). Live USB is `pytest.mark.hardware` and skipped
  off-Pi: device present, `apply` moves RPM or PWM readback.

These never replace §4.1–4.6.

### 4.8 HTTP tests

See also §6. Stub the command sink. Auto mode: `POST /api/pwm` → 409.
Malformed JSON → 4xx. `/api/state` matches `model.py`.

---

## 5. Digole + touch (after Command is stable)

Page state machine, not a web of `if`. Source of truth is the daemon.
The screen renders `(obs, cmd)` and emits `tap` / `swipe` / `hold`.

Navigation: bottom bar or swipe. Debounce touch; hit-test rectangles.

```text
[ Overview ] [ Temps ] [ Fans ] [ MPC ] [ Host ]
```

1. **Overview** — coolant T, air T, max PWM, Auto/Manual, MQTT.
   Tap T → Temps, tap % → Fans.
2. **Temps** — XT6/Quadro/(later 1-wire). Tap — 1–2 min RAM buffer.
3. **Fans** — XT6+Quadro channels: RPM+PWM. Tap — override slider.
   “Auto” clears override.
4. **MPC** — target T, Quiet/Normal/Cool presets, solver status
   (`ok` / `fallback` / `fault`). No matrices on screen.
5. **Host** — CPU °C, load, RSSI, disk, uptime.

Global **FAULT** (USB gone, MPC timeout) covers everything.

Modes: `auto` (MPC writes all PWM), `manual`, `mixed` (some channels
overridden).

Frame rate: on state change or 2–4 Hz, not 20. No animation on Zero W.

Digole protocol is our own layer (UART/I2C/SPI), not Arduino libraries.

---

## 6. HTTP view and control

Same source of truth as Digole: `(obs, cmd)` and the same `apply()` path.
HTTP must **not** write PWM itself. It posts intents; the loop/MPC/manual
layer turns them into `MpcCommand`.

Bind: `0.0.0.0:8080` (see `config.example.yaml`). LAN only for v1.
No auth on the local network in MVP; add a bearer token before exposing
the API further.

### View

| Method | Path | Returns |
|--------|------|---------|
| `GET` | `/api/state` | `PlantObservation` + last `MpcCommand` + mode + setpoints |
| `GET` | `/api/health` | USB present, MQTT, solver `ok`/`fallback`/`fault`, uptime |
| `GET` | `/` | One HTML page, same five sections as Digole (Overview, Temps, Fans, MPC, Host) |

JSON is the API. HTML is a thin client over `/api/state` (poll 1–2 s on
Zero W; no websockets until 2W). FAULT banner matches Digole.

### Control

| Method | Path | Body | Effect |
|--------|------|------|--------|
| `POST` | `/api/mode` | `{ "mode": "auto"\|"manual"\|"mixed" }` | Global mode |
| `POST` | `/api/setpoint` | `{ "channel": "coolant", "celsius": 35 }` | Target T for MPC (Auto) |
| `POST` | `/api/pwm` | `{ "channel": "radiator", "pwm": 0.4 }` | Manual override 0..1; implies mixed/manual |
| `POST` | `/api/preset` | `{ "name": "quiet"\|"normal"\|"cool" }` | MPC aggressiveness |
| `POST` | `/api/auto` | `{ "channel": "radiator" }` or `{}` | Clear override (one channel or all) |

Reject out-of-range PWM (`pwm_min`/`pwm_max`). Do not bypass
`d_pwm_max` unless mode is an explicit emergency (not in v1).

### Tests (no hardware)

See §4.8. API is required in CI; HTML is optional.

Zero W: JSON + one static page. Heavier UI after Zero 2 W.

---

## 7. Home Assistant

MQTT Discovery first; no custom HA integration in v1.

- LWT: `{node_id}/status` = `online`/`offline` (`node_id` from config)
- HA sends a **setpoint** (target T), not raw PWM, while Auto.
- PWM number entities in HA only in Manual.
- Host: CPU temp, load, RAM, disk, Wi-Fi RSSI, uptime.

Broker: Home Assistant Mosquitto (or any MQTT broker on the LAN). Host
and credentials stay in `config.yaml` / `private.md`, not here.

---

## 8. TODO

### Docs / repo

- [x] Create the GitHub repo (`gh`, §12)
- [ ] `pyproject.toml` (ruff, pytest), `config.example.yaml`
- [ ] CI: ruff + pytest for track A (invariants, nominal, failures, sensor lies, fuzzy, closed-loop; fake hwmon; no live USB)

### Track A — MPC (dev machine, parallel with hardware)

- [ ] `model.py`: Observation / Config / Command / State
- [ ] `mpc.step(obs, config, state) -> (command, state)`: PI first
- [ ] Sensor gate (stuck / spike / dT/dt / stale) in front of the solver
- [ ] Hold-last-good then ramp to `fallback_pwm` (high); never `pwm_min` on fault
- [ ] `aqua_bridge.sim.plant`: RC thermal plant
- [ ] `assert_command_safe` on every step
- [ ] `test_mpc_nominal.py`: setpoint, steps, disturbance, cold/warm start, saturation, goldens
- [ ] `test_mpc_failures.py`: missing keys, bad ts, stall RPM, solver throw → fallback
- [ ] `test_mpc_sensor_faults.py`: stuck, spike, jump, drift, swap, dropout, flicker, garbage
- [ ] `test_mpc_fuzzy.py`: Hypothesis on random obs, sequences, mixed lies, random plants
- [ ] `test_mpc_closedloop.py`: model mismatch, actuator delay, long run
- [ ] Tests for hold-then-high fallback (never pwm_min on sensor fault)
- [ ] First untrusted tick: cmd == prev == fallback_pwm
- [ ] Reject config: fallback_pwm keys, values > pwm_min, fallback_hold_s,
      confirm_ticks >= 2
- [ ] Flicker: mode stays fallback until confirm_ticks consecutive trusted ticks
- [ ] Jump: new level confirms; not stuck in fallback / not forced to fallback_pwm
- [ ] Spike: one untrusted tick, last good unchanged
- [ ] Bumpless transfer after fallback (`test_mpc_failures.py`)
- [ ] Replace PI with a small linear MPC (numpy); keep the same tests

### Track B — hardware (Pi USB; fake sysfs anywhere)

- [ ] USB host: `dtoverlay=dwc2,dr_mode=host`, powered hub
- [ ] Spike: `lsusb`, `sensors`, pwm list; **is Quadro writable via XT6**
- [ ] Spike: **does XT6 revert after the Pi stops writing?** If not, software
      sensor + firmware timeout (see §2)
- [ ] `hw/xt6.py` read/apply, udev `0c70`, sysfs root injectable
- [ ] `test_hw_map.py` / fake hwmon in CI
- [ ] `pytest.mark.hardware` live USB on the Pi
- [ ] systemd unit: `Type=notify`, `Wants=`+`After=network-online.target`,
      `Restart=always`, `WatchdogSec`,
      `ExecStart=%h/aqua-bridge/.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml`,
      SIGTERM handler writes `fallback_pwm` (no `ExecStop=`)

### Glue and UI

- [ ] `control/loop.py` on the Pi
- [ ] MQTT + HA discovery (host + temps + fans)
- [ ] HTTP API: `GET /api/state`, `/api/health`; `POST /api/mode`, `/api/setpoint`, `/api/pwm`, `/api/preset`, `/api/auto`
- [ ] HTTP HTML: same five pages as Digole, poll `/api/state`
- [ ] Tests: HTTP talks to the command sink, not hwmon; Auto rejects raw PWM; fuzz JSON → 4xx
- [ ] Digole: protocol, pages, touch, hit-test
- [ ] DS18B20 + `w1-gpio` last
- [ ] README: bring-up on a **fresh** Pi (checklist §10)

### Upgrade

- [ ] Run on Zero 2 W, same config
- [ ] 64-bit Lite if needed — no API change

---

## 9. Raspberry Pi packages and settings

Machine-readable list: `deploy/packages-rpi.txt`.
Install with:

```bash
sudo apt-get update
sudo apt-get install -y $(grep -v '^#' deploy/packages-rpi.txt | xargs)
```

Base set (install on every Pi):

| Package | Why |
|---------|-----|
| build-essential gcc make pkg-config | build |
| git cmake flex bison libssl-dev bc | kernel modules / MPC tooling |
| linux-headers-rpi-v6 | out-of-tree modules |
| i2c-tools python3-smbus | I2C / Digole |
| python3 python3-dev python3-pip python3-venv | daemon |
| python3-spidev python3-lgpio libgpiod-dev gpiod | GPIO/SPI |
| minicom | Digole UART debug |
| lm-sensors | `sensors`, hwmon |

When USB/MQTT/MPC work starts, still via **apt** (same list, so a venv
with system site-packages can import them):

```text
python3-hid
python3-usb
python3-numpy
python3-yaml
python3-aiohttp
python3-paho-mqtt
liquidctl
```

**venv vs apt (pick this, do not mix the other way):**

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -e .          # aqua-bridge only; numpy/yaml/paho come from apt
```

A venv *without* `--system-site-packages` cannot see apt numpy. Do not
install numpy with pip on the Zero W unless you have to.

**sd_notify:** do not add `python3-systemd` or `python3-sdnotify`. Write a
UNIX datagram to `$NOTIFY_SOCKET` with the stdlib (`socket.socket`,
about ten lines) for `READY=1` and `WATCHDOG=1`. If the env var is
unset (running in a terminal), no-op.

**pyproject ranges must accept the apt versions.** Even with
`--system-site-packages`, `pip install -e .` will pull numpy from PyPI
and try to build it if `Requires-Dist` excludes the Debian package.
Before locking deps, on the Pi:

```bash
apt-cache policy python3-numpy python3-yaml python3-paho-mqtt python3-aiohttp
python3 -c "import numpy,yaml,paho.mqtt,aiohttp; print(numpy.__version__)"
```

Set lower/upper bounds so those versions satisfy the metadata, **or**
install the project with `pip install -e . --no-deps`. Prefer ranges that
match apt so a forgotten `--no-deps` does not compile numpy on the Zero W.

piwheels **does** publish `numpy` cp313 `linux_armv6l` for Trixie (e.g.
2.5.3 as of 2026-09). Use it only if the Debian package is missing or
too old:

```bash
pip install numpy --extra-index-url https://www.piwheels.org/simple
```

Building numpy from source on a Zero W is not a fallback.

### Overlays and modules

`/boot/firmware/config.txt` (`[all]` section):

```text
dtparam=i2c_arm=on
dtparam=spi=on
enable_uart=1
dtoverlay=disable-bt
```

When USB host is needed:

```text
dtoverlay=dwc2,dr_mode=host
```

When DS18B20 is added:

```text
dtoverlay=w1-gpio
```

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

### systemd (once the daemon exists)

`deploy/aqua-bridge.service` → `/etc/systemd/system/`

```text
Type=notify
NotifyAccess=main
Wants=network-online.target
After=network-online.target
Restart=always
WatchdogSec=30
User=          # service account
# dialout / plugdev for HID
# Code lives in the service user's home (rsync, no root). Config is not cwd.
ExecStart=%h/aqua-bridge/.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml
```

`After=` does not pull in the target; `Wants=network-online.target` must
sit next to it. `Type=notify` is required or `READY=1` is ignored.
`NotifyAccess=main` means the **main** process is Python: `ExecStart=`
is the venv interpreter and `-m aqua_bridge` directly — no `sh -c`, no
wrapper script.

`sd_notify` is stdlib-only (§9). `WATCHDOG=1` every loop, period shorter
than `WatchdogSec`. `Restart=on-failure` is not enough: OOM and watchdog
timeouts need `always`.

**Stop path:** only a SIGTERM handler in the daemon writes `fallback_pwm`
then exits. Do **not** set `ExecStop=`. systemd runs `ExecStop=` while
the main process is still alive and sends SIGTERM after that — two
writers to hwmon at once.

**Deploy side effect (accepted):** `systemctl restart` is a clean stop, so
fans go to `fallback_pwm` (0.8 in the example). Coming back down at
`d_pwm_max=0.1` and `dt=2` takes on the order of **ten seconds**. Write
that in the unit comments so a restart is not mistaken for a thermal
event.

Pi **power loss** is still uncovered: last PWM stays on the fans until
XT6 firmware reverts (spike §2.3) or the board comes back. Accepted if
(3) fails.

### udev

`deploy/99-aquacomputer.rules`:

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="0c70", MODE="0660", GROUP="plugdev"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0c70", MODE="0660", GROUP="plugdev"
```

---

## 10. Bring up on another Raspberry Pi

There is no project-specific image — flash Lite 32-bit (Zero W) or Lite
32/64-bit (Zero 2 W) and follow this checklist.

1. Flash Raspberry Pi OS Lite, create a sudo user, enable SSH and Wi-Fi,
   set timezone and Wi-Fi country.
2. `sudo apt-get update && sudo apt-get install -y` from
   `deploy/packages-rpi.txt`.
3. Overlays §9, `i2c-dev`, drop serial console from cmdline, reboot.
4. Groups, optional NOPASSWD, SSH keys (see `private.md`).
5. 1 GB swap if Zero W.
6. USB: dwc2 host, powered hub, XT6 on USB, Quadro on aquabus only.
7. `lsusb` / `sensors` — hwmon regression.
8. Clone the repo to **`~/aqua-bridge`** (service user home — same path
   as `ExecStart=` / rsync). `python3 -m venv --system-site-packages .venv`,
   `pip install -e .`. Install config (not next to the daemon cwd):
   `sudo mkdir -p /etc/aqua-bridge && sudo cp config.example.yaml /etc/aqua-bridge/config.yaml`
   and edit MQTT host, credentials, channel names. Always pass
   `--config /etc/aqua-bridge/config.yaml` (systemd does; a terminal
   run must too — the process has no default relative to `/`).
9. udev + systemd, `systemctl enable --now aqua-bridge`.
10. Check MQTT in HA, PWM, Digole later.

Zero W: 32-bit only. Do not install a desktop.

The Pi should be reachable on the LAN (mDNS `hostname.local` or DHCP name).

---

## 11. Development: host PC ↔ Pi

Host PC: git, MPC tests, simulator, edits.
Pi: USB, PWM, touch, MQTT on the live loop.

Concrete hostnames and the rsync/ssh lines: `private.md`.

Typical pattern:

```bash
# from the repo root on the development machine
rsync -az --delete --exclude .venv --exclude __pycache__ \
  --exclude private.md --exclude secrets \
  ./ USER@PI-HOST:~/aqua-bridge/

ssh USER@PI-HOST 'cd ~/aqua-bridge && .venv/bin/pytest -m hardware'
```

Or `git pull` from GitHub on the Pi. Avoid committing from a Zero W
(slow card, easy to mix up branches).

Until systemd exists:

```bash
ssh USER@PI-HOST
cd ~/aqua-bridge
source .venv/bin/activate
python -m aqua_bridge --config /etc/aqua-bridge/config.yaml
```

---

## 12. GitHub workflow

The repo already exists. Need a GitHub account and `gh` on the development
machine. Skip `gh repo create` if `origin` is set.

```bash
# macOS
brew install gh git
gh auth login
gh auth status
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

The current `origin` is HTTPS, so this is not blocking until someone
switches the remote to SSH.

From this directory:

```bash
cd /path/to/aqua-bridge
git init -b main
git add PROJECT.md README.md deploy config.example.yaml .gitignore
git commit -m "Initial project spec for aqua-bridge."
gh repo create aqua-bridge --private --source=. --remote=origin --push
```

If the repo already exists on GitHub:

```bash
git remote add origin git@github.com:<USER>/aqua-bridge.git
git push -u origin main
```

`gh repo view --web` to check.

Day to day:

```bash
git checkout -b mpc-core
git push -u origin mpc-core
gh pr create --fill
```

Track B uses separate branches (`hw-xt6`), not mixed with `mpc-core`.
`main` only gets what is green in CI (track A) and does not break the
`model.py` contract.

Clone:

```bash
gh repo clone <USER>/aqua-bridge
```

---

## 13. Rollout order (with parallelism)

1. GitHub repo exists (§12). Site-specific inventory in `private.md`.
2. **In parallel:** track A (MPC + tests) on the dev machine **and** USB
   spike on the Pi (hub, XT6, hwmon, **firmware revert**). The core does
   not need hardware.
3. XT6 adapter once the spike answers the Quadro PWM **and** revert
   questions.
4. Glue loop, MQTT.
5. HTTP API + HTML (same pages as Digole).
6. Digole + touch.
7. DS18B20.
8. Zero 2 W with no API change.
