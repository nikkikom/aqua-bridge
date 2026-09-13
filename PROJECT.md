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
| 1 | Controller sets fan PWM: PI or small linear MPC, same API (`mpc.solver`) |
| 2 | Digole + touch: pages, override, diagnostics |
| 3 | MQTT + HA discovery: sensors, temperatures, Pi health, setpoint (not raw PWM in Auto) |
| 4 | HTTP: view state and control (setpoint, mode, manual PWM, preset) |
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
- The daemon reads and writes through **hwmon sysfs**
  (`aquacomputer_d5next`, `/sys/class/hwmon/hwmonN/{tempK_input,fanK_input,pwmK}`),
  not raw HID. The HID udev rules stay for liquidctl and the spike. The
  hwmon ABI the adapter assumes (§3 Track B) is unconfirmed until the
  spike.
- A **powered USB hub** is still required: Zero W OTG is flaky; do not
  power devices from the board port. XT6/Quadro 12 V is their own supply.
- Digole: UART `/dev/serial0` (= `ttyAMA0`), plus I2C/SPI if wired that
  way. Bluetooth is off (`dtoverlay=disable-bt`); serial console is
  removed from UART.
- DS18B20: GPIO4 + 4.7 kΩ, `dtoverlay=w1-gpio` — **last**.
- Zero 2 W upgrade: same 32-bit userland or 64-bit Lite, no `armv6` in
  the code. USB OTG is unchanged.

**Status:** only the Pi is up. The aquaero 6 XT, Quadro, Digole and
DS18B20 are not connected yet; the spike questions below are open.

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

If (2) fails: HID writes into XT6, or temporarily a second USB device
(breaks the chosen topology). This blocks the hardware track only, not
the MPC core.

If (3) fails, a **software** watchdog inside the daemon (ramp to
`fallback_pwm`) only covers faults the process can still act on. It does
**not** survive SIGKILL, OOM-kill, USB-hub dropout that takes the write
path with it, or **Pi power loss**. Residual risk, state it plainly: PWM
can sit at a **low** last value; then the PC load rises and nothing ramps
the fans. Mitigations that still run on the Pi:

- a dead sensor path is a gate fault like any other: hold, then ramp
  high (§3); the loop keeps applying that command (§3 Glue)
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

MpcCommand
  pwm:  dict[str, float]              # 0..1 per logical channel
  mode: Mode                          # auto | saturated | fallback
  diagnostics: dict[str, Any]         # str keys, JSON-serialisable, never NaN

WindowSample
  raw_temps: dict[str, float | None]  # sanitised raw temps for config.temps; None, never NaN
  cmd_pwm:   dict[str, float]         # command of that tick (the loop writes the applied one)

MpcState                              # MpcState.cold(): everything empty, no fault
  last_cmd: MpcCommand | None
  last_good_obs: PlantObservation | None    # last trusted non-fallback tick, replaced as a whole
  last_raw_temps: dict[str, float | None] | None  # previous raw sample, even if untrusted
  window: tuple[WindowSample, ...]    # newest last, trimmed to stuck_ticks; feeds median3 and Stuck
  fault_since_ts: float | None        # first ts any fallback cause became active
  fault_reason: FaultReason | None    # sensor_gate | solver; set and cleared together with fault_since_ts
  trusted_streak: int                 # consecutive trusted ticks; 0 on untrusted tick or solver fault
  integrator: dict[str, float]        # per channel: PI integral term / MPC equilibrium PWM u_ss
  solver_memory: dict[str, Any]       # JSON: last_ts, fault_ticks, stall_ticks, stuck_latch, one slot per solver
```

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
| `dt` | required | seconds per tick; > 0 |
| `horizon` | required | MPC ticks; int ≥ 1; the PI solver ignores it |
| `temps` | required | plant temperatures the gate checks; non-empty, unique |
| `setpoints` | required | °C per controlled temperature; non-empty; keys ⊆ `temps`; each strictly inside `(temp_min_c, temp_max_c)` |
| `channels` | required | logical fan channels; non-empty, unique |
| `pwm_min`, `pwm_max` | required | each in `[0, 1]`; `pwm_min < pwm_max` (equal is rejected: no valid `fallback_pwm` could exist) |
| `d_pwm_max` | required | largest PWM change per tick; > 0 |
| `fallback_pwm` | required | high cooling on fault; keys exactly `channels`; each in `(pwm_min, pwm_max]` |
| `fallback_hold_s` | required | seconds; ≥ 0 |
| `confirm_s` | required | seconds of consecutive trusted ticks; ≥ `2 * dt` and ≤ `fallback_hold_s` |
| `dT_max_c_per_s` | required | gate slew limit, °C/s; > 0 |
| `stuck_s` | required | Stuck window, seconds; ≥ `2 * dt` (useful range: minutes) |
| `stuck_eps_c` | required | “unchanged” band, °C; > 0 (≥ sensor resolution, aquaero 0.01 °C) |
| `stuck_pwm_net` | required | net PWM move that must show up in T; in `(0, 1]` |
| `stuck_sibling_dT_c` | required | net move of another temperature, °C; > 0 |
| `median3` | `false` | pre-filter; must be a YAML bool (`"false"` is rejected) |
| `temp_min_c`, `temp_max_c` | `-20.0`, `120.0` | gate absolute valid range, °C; min < max |
| `solver` | `pi` | `pi` or `mpc` |
| `solver_max_iter` | `50` | int ≥ 1; hitting it is a solver fault |
| `channel_temps` | `{}` | channel → temperatures it controls. `{}`: every channel controls every setpoint temperature. When given: keys exactly `channels`, lists non-empty, every listed temperature has a setpoint; a single string is a one-element list |
| `pi_kp` | `0.05` | PWM per °C; > 0 (placeholder, untuned) |
| `pi_ki` | `0.002` | PWM per °C·s; ≥ 0 (placeholder, untuned) |
| `weights` | `{}` | MPC tracking weight per temperature, per °C²; keys ⊆ `temps`; ≥ 0; missing → 1.0 |
| `weight_pwm` | `0.0` | MPC effort penalty per PWM²; ≥ 0 |
| `weight_dpwm` | `0.05` | MPC move penalty per PWM² per tick; ≥ 0 (example config: 60) |
| `mpc_tau_s` | `120.0` | MPC model time constant, seconds; > 0 |
| `mpc_gain_c_per_pwm` | `8.0` | MPC model: steady-state °C drop per +1.0 PWM on one channel; > 0 |
| `mpc_estimator_gain` | `0.1` | MPC disturbance estimator gain per tick; in `(0, 1]` |

With `solver: mpc` the config also needs `weight_pwm + weight_dpwm > 0`
(strictly convex QP) and at least one setpoint temperature with a
positive weight. The MPC weights are in raw units (°C² against PWM²): with the
example horizon of 8 ticks on a plant with minutes of time constant the
tracking term is small, which is why the example move penalty is 60, not
0.05.

Derived tick quantities are properties, never YAML keys:

- `dT_max_tick = dT_max_c_per_s * dt`
- `confirm_ticks = max(2, ceil(confirm_s / dt))`
- `stuck_ticks = max(2, ceil(stuck_s / dt))`
- `temps_for_channel(ch)`: `channel_temps[ch]`, or every setpoint
  temperature in `temps` order
- `weight_for(temp)`: `weights.get(temp, 1.0)`

Changing `dt` must not silently change the °C/s slew limit. The config is
rejected if `confirm_s > fallback_hold_s`: a real Jump would hit
hold/ramp-high before the new level confirms. Do not “accept the fan
spike” as default.

The rest of `config.yaml` (`config.py`, `AppConfig`): `mpc` is required
and typed; `mqtt`, `host`, `xt6`, `http`, `digole`, `onewire` go to their
owners as plain dicts (each must be a mapping or absent); unknown
top-level sections are kept in `AppConfig.extra`, visible but not fatal.
`xt6` is read by Track B (below), `http` by §6, `mqtt` and `host` by §7.
`digole`, `onewire` and `xt6.prefer` are parsed but no code reads them
yet.

Channel names are logical (`radiator`, …). Mapping onto `hwmon pwmN`
lives only in the hardware adapter.

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
wrong type; every runtime problem becomes `mode=fallback`.

**Order inside `step`** (`control/mpc.py`):

1. Resolve `prev`, what the command is rate-limited against.
2. Time check of `obs.ts` against `solver_memory["last_ts"]`.
3. Sensor gate, fed the Stuck latch from `solver_memory["stuck_latch"]`.
4. Fault bookkeeping (one timer, streak).
5. Solver, only on a trusted tick that is fault-free or the
   `confirm_ticks`-th consecutive trusted one.
6. Fallback policy while a fault is active.
7. Rate limit against `prev` (`|Δ| <= d_pwm_max`), then clamp into
   `[pwm_min, pwm_max]`.
8. Mode.
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

**Safe PWM on untrusted observation (cooling loop):** never jump toward
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
6. `mode=fallback` for the whole time a fallback cause is active;
   `mode == fallback` exactly when the returned state has
   `fault_since_ts` set.

Elapsed fault time is `max(obs.ts - fault_since_ts, (fault_ticks - 1) *
dt)` with `fault_ticks` counted in `solver_memory`, so a stuck clock still
ramps high by tick count.

**One** `fault_since_ts` covers every reason `step` cannot emit a trusted
`auto` command: sensor gate **or** solver fault. `fault_reason` records
which. The hold/ramp-high policy is identical. Do not run two
independent hold timers. A solver fault on a trusted tick also resets
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

`stuck_s` is a **window**, not a debounce: within it the coolant must
have had time to answer a PWM move. Size it at several plant time
constants (minutes). 30 s flags a healthy sensor right after a setpoint
step, because PWM moves in seconds and coolant in minutes.

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
   `first` or `ok`. `trusted_streak` is one integer for the whole
   observation, not per channel. `last_good_obs` is replaced **as a
   whole** on every trusted tick that yields a non-fallback command
   (steady auto: every tick; after a fault: the confirming tick), with
   non-finite rpm/pwm stored as `None`.
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
  cross-sensor plausibility rule. A persisting coolant/air swap, or a
  persisting “coolant 18 °C, air 90 °C”, is a Jump on both sensors:
  untrusted at onset, confirmed after `confirm_ticks`, and because
  `confirm_s <= fallback_hold_s` it never reaches the ramp. A
  cross-sensor rule would trap every simultaneous double Jump in fallback
  forever. A real key swap is a mapping mistake in `xt6.temp_map`; it is
  checked at bring-up (§10), not by the gate.

A gate that only compares to last good would keep Jump untrusted
forever and slam fans to `fallback_pwm`. That is a spec bug; do not
implement it. A gate without Stuck would treat a frozen coolant
reading as trusted forever.

**Mode and saturation.** `fallback` while a fault is active; `saturated`
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
`rpm` attribute of the channel's `xt6.fans` entry (Track B).

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
Either way the emitted command is `clip(prev)`.

**Solvers.** `mpc.solver` selects one from the registry `SOLVERS` in
`control/mpc.py` (`pi` → `PiSolver`, `mpc` → `MpcSolver`). Both implement
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
exception out of `step`.

- **PI** (per channel): `e = max(T - setpoint)` over
  `temps_for_channel(ch)` (the hottest deviation drives the fan);
  `u = pi_kp * e + I`; `I' = clamp(I + pi_ki * e * dt, pwm_min, pwm_max)`;
  bumpless `I = prev - pi_kp * e`.
- **MPC** (`control/solver_mpc.py`, numpy only): per controlled
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

### Track A — core (dev machine / CI, no Pi)

- `control/gate.py`, `control/mpc.py`, `control/solver_pi.py`,
  `control/solver_mpc.py`, `sim/plant.py`, `tests/test_gate.py`,
  `tests/test_mpc_*.py`
- RC thermal plant in the simulator (`aqua_bridge.sim.plant`): lumped
  coolant and case air, radiator fans move heat coolant → air, intake
  fans air → ambient; actuator delay, stalled channels and Gaussian
  sensor noise are parameters; deterministic for a seed.
  `run_closed_loop(..., observe_hook=...)` injects lies.
- PI and MPC both exist behind the same contract; the example config
  stays on `pi` until both are tuned on the real loop (§8).
- On Zero W keep the solver small: 2–4 temperatures, 4–8 PWM, dt 1–2 s,
  horizon 10–20 s, numpy, no CasADi/IPOPT. acados after 2W. Measured
  with `tools/bench_step.py` (600 closed-loop ticks, example config,
  dt 2 s) on a Zero W with Python 3.13.5 and numpy 2.2.4: PI step p99
  about 8.9 ms, MPC step p99 about 34.5 ms (the first MPC step builds
  its matrices, about 55 ms). Both stay under 2 % of `dt`.

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
  `xt6.root` (default `/sys/class/hwmon`).
- Unit tests against a **fake hwmon tree** in a temp directory (CI, no
  Pi). `pytest.mark.hardware` only for the live device;
  `tests/conftest.py` skips those tests when no hwmon device named
  `aquaero` exists.
- **Does not import control.**

### Glue (`control/loop.py`, `control/supervisor.py`, `__main__.py`)

```text
plan = supervisor.plan_tick()                 # effective config, control mode, overrides, released
cmd, state = mpc.step(obs, plan.cfg, state)
cmd = supervisor.compose(cmd, plan, prev)     # manual overrides, unless cmd.mode == fallback
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
  after the supervisor snapshot is updated (MQTT publishes there); its
  exceptions are logged and dropped.
- **`shutdown()`:** writes `fallback_pwm` once (`mode=fallback`,
  deliberately not rate-limited, §9), then `STOPPING=1`; survives a sink
  failure.

The `Supervisor` is the `ControlSurface` (§6) that HTTP, MQTT and, later,
Digole use: it owns control mode, overrides, setpoints and preset,
provides the effective config for each tick, and composes overrides into
the solver command. Digole and MQTT subscribe to `(obs, cmd)` through it,
not to HID.

`python -m aqua_bridge` (`__main__.py`): `--config PATH` (required),
`--source xt6|sim` (default `xt6`; `sim` drives the RC plant with names
taken from the config), `--once` (one tick, print obs/cmd JSON),
`--ticks N`, `--sim-speed X` (sim only; 1 real time, 0 as fast as
possible), `--log-level`. Every run that started the loop, `--once`
and `--ticks` included, ends with the `fallback_pwm` stop write. Exit
codes: `0` clean stop, `1` loop crashed, `2` bad arguments or config
(including the xt6 map check), `3` source/sink could not be built. HTTP and MQTT start next to
the loop when enabled (§6, §7); a publisher that fails to start is logged
and the daemon keeps controlling the fans.

### Repo layout

```text
aqua-bridge/
  PROJECT.md                 # this file
  README.md
  pyproject.toml             # deps + extras http/mqtt/dev, pytest markers, ruff
  config.example.yaml
  .github/workflows/ci.yml   # lint, test (latest, pi-parity), nightly-fuzz (§12)
  deploy/
    packages-rpi.txt         # apt packages (§9)
    aqua-bridge.service      # systemd unit (§9)
    99-aquacomputer.rules    # udev: usb, hidraw, hwmon pwm group write (§9)
    host-usb.sh              # dwc2 host overlay, idempotent (§10)
    install-pi.sh            # provisioning (§10)
  src/aqua_bridge/
    __main__.py              # python -m aqua_bridge: wiring, signals, exit codes
    model.py                 # the contract
    config.py                # YAML -> AppConfig
    control/gate.py          # sensor gate
    control/mpc.py           # step(): prev, time, gate, fault timer, fallback, rate limit, mode
    control/solver_pi.py     # Solver protocol + PI
    control/solver_mpc.py    # small linear MPC (numpy)
    control/intents.py       # intents, ControlSurface, ControlSnapshot, payloads
    control/supervisor.py    # control mode, overrides, setpoints, presets, compose
    control/loop.py          # read -> step -> compose -> apply -> watchdog
    hw/map.py                # logical name -> hwmon sysfs file
    hw/xt6.py                # aquaero adapter
    sim/plant.py             # inside the package so `pip install -e .` sees it
    sdnotify.py              # stdlib sd_notify
    hostinfo.py              # CPU temp, load, RAM, disk, Wi-Fi RSSI, uptime
    publishers/http.py       # REST + HTML, same intents as Digole
    publishers/static/index.html
    publishers/mqtt_ha.py    # topics, Discovery, command parsing, paho wrapper
    publishers/runtime.py    # HTTP / MQTT threads next to the loop
    ui/digole/               # planned, after Command is stable
  tools/
    bench_step.py            # step() timing per solver against the RC plant
  tests/
    conftest.py              # Hypothesis profiles, fixtures, hardware auto-skip
    invariants.py            # §4.1 helpers
    golden/                  # <scenario>.<solver>.json
    test_model_config.py
    test_gate.py
    test_mpc_invariants.py
    test_mpc_nominal.py
    test_mpc_failures.py
    test_mpc_sensor_faults.py
    test_mpc_fuzzy.py
    test_mpc_closedloop.py
    test_mpc_solver.py
    test_supervisor.py
    test_loop.py
    test_main.py
    test_sdnotify.py
    test_http_api.py
    test_mqtt_ha.py
    test_hostinfo.py
    test_publishers_runtime.py
    test_hw_map.py           # fake sysfs, CI
    test_hw_xt6.py           # fake hwmon in CI; live device with pytest.mark.hardware
    test_deploy.py           # unit, udev rules, install script, shellcheck
```

CI on GitHub runs every test except `hardware` (§12), `fuzzy` and `slow`
included; Hypothesis example counts are capped per profile, not
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

**Both solvers.** The core suites must survive a solver swap (§8). The `solver_kind` fixture in `tests/conftest.py` (ids `pi`,
`mpc`) overrides `cfg` in `test_mpc_invariants.py`, `test_mpc_nominal.py`,
`test_mpc_failures.py`, `test_mpc_sensor_faults.py` and
`test_mpc_closedloop.py`, so every scenario runs once per solver;
`test_mpc_fuzzy.py` parametrises the same way. `test_mpc_solver.py` holds
the MPC-only tests (box QP against a projected-gradient reference,
iteration cap, warm start, model matrices, caching, bumpless
initialisation including prev outside the box and the knee rule,
offset-free tracking with a wrong model, pressure at the rail, config
rules, horizon extremes).

**Fixtures** (`tests/conftest.py`): `cfg` (the example config),
`fast_cfg` (`dt=1`, `confirm_ticks=2`, `fallback_hold_s=4`,
`stuck_ticks=4`, PWM limits unchanged), `solver_kind`,
`example_config_path`, `aquaero_hwmon` (skips when absent). Gate suites
use a `gcfg` fixture parametrised over `median3`.

**Markers** (`--strict-markers`): `hardware` (live aquaero; auto-skipped
when no hwmon device named `aquaero` exists), `fuzzy` (Hypothesis), `slow`
(long closed-loop runs, subprocess SIGTERM). CI runs `-m "not hardware"`.

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
- `cmd.mode` is one of `auto | saturated | fallback`
- `fault_since_ts` and `fault_reason` are set together or not at all
- `mode == fallback` exactly when the returned state is in fault; the
  returned state carries the returned command as `last_cmd`; the input
  state is not mutated
- an observation the gate must reject without history (missing / `None`
  / NaN temperature, unknown key, or out of range when `median3` is off)
  → `mode=fallback`. With `median3` on, a finite out-of-range value is
  not structural: the median removes a single Spike.
- untrusted observation → `mode=fallback` and the hold-then-high policy
  in §3 (never a step toward `pwm_min` *because* of the fault)
- same `(obs, config, state)` → same `(cmd, state)` (deterministic)

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
  behaviour is meant to change.

These tests stay; swapping the default solver must not delete them.

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

### 4.4 Lying sensors (fault injection)

`tests/test_mpc_sensor_faults.py`

Watercooling sensors lie. The controller must **not** chase a lie to
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

The **sensor gate** is explicit (`control/gate.py`) and uses the
trusted-tick rule of §3. Tests target that gate (`tests/test_gate.py`,
and the gate-level cases here) **and** the full `step`. They include:
Jump must not stay in fallback forever; Stuck must flag when **net** PWM
or a sibling sensor moves, and must **not** flag at equilibrium with
small PWM dithering or on a sibling Spike / Jump; noise σ = 0.2 °C must
not flicker `mode`; every gate test runs with `median3` both `false` and
`true`. Do not hide filtering only inside numpy and leave it untested.

### 4.5 Fuzzy / property tests

`tests/test_mpc_fuzzy.py` — Hypothesis, marked `pytest.mark.fuzzy`, both
solvers.

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

These never replace §4.1–4.6.

### 4.8 HTTP tests

`tests/test_http_api.py`. See also §6. A stub `ControlSurface`; no
hwmon. `/api/state` and `/api/health` bodies equal
`ControlSnapshot.state_payload()` / `health_payload()` with exactly the
documented keys. Auto mode: `POST /api/pwm` → 409. Malformed JSON, body
not an object, missing field, unknown channel, PWM outside `[0, 1]` or
outside `[pwm_min, pwm_max]` → 400. Unknown `/api/...` route → 404.
The suite skips itself when binding a localhost socket is denied (some
sandboxes); CI runners allow it.

### 4.9 Glue, publishers, deploy

- `tests/test_supervisor.py` — control-mode rules, overrides, setpoints,
  presets, `compose` (rate limit, fallback beats overrides, purity),
  snapshot and health flags, concurrent HTTP and loop threads (`slow`).
- `tests/test_loop.py` — §4.3 loop cases, READY/WATCHDOG/STOPPING,
  overrides and bumpless release through the loop, an override on frozen
  temperatures tripping Stuck, run scheduling, the `on_tick` hook, sim
  closed loops (`slow`).
- `tests/test_main.py` — CLI parsing, exit codes, `--once`, sim wiring,
  xt6 map mismatch → exit 2, publisher wiring and start failures, the
  SIGTERM stop path in-process, a SIGTERM delivered inside a stderr write,
  a second SIGTERM during shutdown, and a real subprocess (`slow`).
- `tests/test_sdnotify.py` — address resolution, no-op without
  `$NOTIFY_SOCKET`, failures return `False`, a real `AF_UNIX` datagram
  socket (skipped where binding one is denied).
- `tests/test_mqtt_ha.py` — topics, Discovery entities, PWM numbers only
  in manual and deleted on leaving it, state payload, command parsing
  never raises, `on_message` never raises. No broker.
- `tests/test_publishers_runtime.py` — `HttpService` serving and
  reporting a start failure; `MqttService` with a fake client: nothing
  while disconnected, Discovery per connect and on the manual boundary,
  host refresh interval, swallowed publish errors.
- `tests/test_hostinfo.py` — every host metric against fake procfs /
  sysfs files, `None` on missing or malformed input.
- `tests/test_deploy.py` — unit file (`Type=notify`, `NotifyAccess=main`,
  `Restart=always`, watchdog, no `ExecStop=`, venv `ExecStart`,
  `TimeoutStartSec >= WatchdogSec`), udev rules against the unit’s
  groups (hwmon pwm group write, usb, hidraw), install script triggers
  udev, `bash -n` and `shellcheck` on both scripts (shellcheck skipped
  when not installed; GitHub runners have it).

---

## 5. Digole + touch (after Command is stable)

Not implemented yet. Page state machine, not a web of `if`. Source of
truth is the daemon: the screen reads `ControlSurface.snapshot()` and
sends intents through `ControlSurface.submit()`, exactly like HTTP (§6).
The screen renders `(obs, cmd)` and emits `tap` / `swipe` / `hold`.

Navigation: bottom bar or swipe. Debounce touch; hit-test rectangles.

```text
[ Overview ] [ Temps ] [ Fans ] [ MPC ] [ Host ]
```

1. **Overview** — coolant T, air T, max PWM, control mode, MQTT.
   Tap T → Temps, tap % → Fans.
2. **Temps** — XT6/Quadro/(later 1-wire). Tap — 1–2 min RAM buffer.
3. **Fans** — XT6+Quadro channels: RPM+PWM. Tap — override slider.
   “Auto” clears override.
4. **MPC** — target T, Quiet/Normal/Cool presets, solver status
   (`ok` / `fallback` / `fault`, §6). No matrices on screen.
5. **Host** — CPU °C, load, RSSI, disk, uptime.

Global **FAULT** (USB gone, controller in fallback) covers everything.

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

Config `http:` — `enabled` (must be `true` to start; `false` in the
example), `bind` (default `0.0.0.0`), `port` (default `8080`). The server
runs on its own asyncio loop in a daemon thread
(`publishers/runtime.py`); a bind failure is logged and the daemon keeps
controlling the fans without the API. LAN only for v1. No auth on the
local network in MVP; add a bearer token before exposing the API further.

Runtime control state (mode, overrides, setpoints, preset) lives in
memory only: the daemon starts in `auto` with preset `normal` and the
setpoints from `config.yaml`, and nothing is written back to disk.

### View

| Method | Path | Returns |
|--------|------|---------|
| `GET` | `/api/state` | `ControlSnapshot.state_payload()` |
| `GET` | `/api/health` | `ControlSnapshot.health_payload()` |
| `GET` | `/` | `publishers/static/index.html` |

`/api/state` keys:

- `obs` — last successfully read `PlantObservation` (`temps`, `rpm`,
  `pwm`, `ts`), `null` before the first; a failed read does not replace it
- `cmd` — last command handed to the sink, applied or not (`pwm`,
  `mode`, `diagnostics`, including `diagnostics.supervisor`), `null`
  before the first tick; after shutdown the stop write
- `mode` — control mode `auto` | `manual` | `mixed`
- `preset` — `quiet` | `normal` | `cool`
- `setpoints` — the user’s setpoints, without the preset offset
- `overrides` — manual PWM per overridden channel
- `channels`, `temps`, `pwm_min`, `pwm_max` — from the base config

`/api/health` keys:

- `usb_present` — the last tick both read and applied successfully
- `mqtt_connected` — `null` when MQTT is disabled, `false` until connected
- `solver` — `ok` (last solver command `auto` or `saturated`),
  `fallback` (gate or solver fault active), `fault` (no solver command
  yet)
- `fault_reason` — `sensor_gate` | `solver` | `null`
- `fault_since_ts` — observation clock seconds | `null`
- `uptime_s` — seconds since the supervisor started (monotonic)
- `version` — package version

JSON is the API. HTML is a thin, view-only client: it polls `/api/state`
and `/api/health` every 2 s (no websockets until 2W) and shows the five
sections Overview, Temps, Fans, MPC, Host. The FAULT banner shows when
`solver` is `fallback` or `fault`, or when a poll fails. Controls are
JSON-only for now. The Host section reads host metrics from
`/api/state`, which does not carry them yet, so it shows dashes (§8).

### Control

Every `POST` body is a JSON object; an empty body means `{}`. Success:
`200 {"ok": true}`. Errors: `{"error": "..."}` with `400` for invalid
input (bad JSON or UTF-8, body not an object, unknown or missing field,
wrong type, non-finite number, unknown channel or temperature, value out
of range, a change whose effective config fails validation) and `409`
for a well-formed request the current mode forbids. Unknown paths are
`404`.

| Method | Path | Body | Effect |
|--------|------|------|--------|
| `POST` | `/api/mode` | `{ "mode": "auto"\|"manual"\|"mixed" }` | Control mode (below) |
| `POST` | `/api/setpoint` | `{ "channel": "coolant", "celsius": 35 }` | Target T for a temperature that has a setpoint in the config |
| `POST` | `/api/pwm` | `{ "channel": "radiator", "pwm": 0.4 }` | Manual override; 409 in `auto` |
| `POST` | `/api/preset` | `{ "name": "quiet"\|"normal"\|"cool" }` | Solver aggressiveness |
| `POST` | `/api/auto` | `{ "channel": "radiator" }` or `{}` | Clear override (one channel or all) |

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
  `(temp_min_c, temp_max_c)`.
- **`/api/preset`:** a documented transform of the base config, applied
  before every `step`, never written to disk:

  | Preset | Setpoints | `pi_kp`, `pi_ki` | `weight_dpwm` (MPC) |
  |--------|-----------|------------------|---------------------|
  | `quiet` | +2 °C | ×0.5 | ×2 |
  | `normal` | as configured | ×1 | ×1 |
  | `cool` | −2 °C | ×2 | ×0.5 |

- **Composition:** each tick, every overridden channel gets its override,
  rate-limited to `|Δ| <= d_pwm_max` against the last **applied** PWM and
  clamped into `[pwm_min, pwm_max]`; other channels keep the solver’s
  value; the command keeps the solver’s `mode`, and
  `diagnostics.supervisor` records control mode, preset, overrides and
  whether they were applied. Do not bypass `d_pwm_max` unless mode is an
  explicit emergency (not in v1).
- **Fallback beats manual** (owner decision): while the solver command is
  `fallback`, overrides are **not** applied and the hold / ramp-high
  command goes to the fans unchanged. A human-pinned low duty must not
  reduce cooling while the controller is blind. Overrides are kept and
  resume when the fault clears. A manual override does not bypass the
  gate either: moving a fan by more than `stuck_pwm_net` while the
  temperatures do not answer within `stuck_s` is, by §3 rule 3, a Stuck
  sensor.

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
  `extra`) plus `host` (`cpu_temp_c`, `load1`, `load5`, `load15`,
  `mem_used_pct`, `mem_total_kb`, `disk_used_pct`, `disk_free_gb`,
  `wifi_rssi_dbm`, `uptime_s`; `null` when unreadable)
- inbound, raw payloads (not JSON): `{node_id}/cmd/mode` (control mode),
  `{node_id}/cmd/preset`, `{node_id}/cmd/auto` (channel, or empty for
  all), `{node_id}/cmd/setpoint/<temp>` (number, °C, one per setpoint
  temperature), `{node_id}/cmd/pwm/<channel>` (number 0..1)

Discovery config topics are
`{discovery_prefix}/{component}/{node_id}/{object_id}/config`, retained,
qos 1; `unique_id` is `{node_id}_{object_id}`; every entity carries the
availability topic and one device block. Entities:

- sensors `host_cpu_temp_c`, `host_load1`, `host_mem_used_pct`,
  `host_disk_used_pct`, `host_wifi_rssi_dbm`, `host_uptime_s`
- sensor `temp_<temp>` per `mpc.temps`; `rpm_<channel>` and
  `pwm_<channel>` (commanded PWM in %) per channel
- number `setpoint_<temp>` per setpoint (range `temp_min_c..temp_max_c`,
  step 0.5)
- number `pwm_cmd_<channel>` per channel (range `pwm_min..pwm_max`, step
  0.01) **only while the control mode is `manual`** — not in `mixed`,
  where some channels are still solver-driven

HA sends a **setpoint** (target T), not raw PWM, while Auto. A raw PWM
command in `auto` is rejected by the supervisor exactly like HTTP’s 409,
logged and dropped. Inbound commands never raise: `parse_command` returns
nothing for a garbage topic or payload, and `on_message` catches every
rejected intent and any other exception, because paho runs callbacks on
its network thread and an exception there would end it.

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
and credentials stay in `config.yaml` / `private.md`, not here. Not yet
exercised against a live broker or Home Assistant (§8).

---

## 8. TODO

### Docs / repo

- [x] Create the GitHub repo (`gh`, §12)
- [x] `pyproject.toml` (ruff, pytest), `config.example.yaml`
- [x] CI: ruff + pytest for track A (invariants, nominal, failures, sensor lies, fuzzy, closed-loop; fake hwmon; no live USB)
- [x] Branch protection on `main` with required checks; rebase merges only (§12)
- [x] Nightly randomized Hypothesis run (`nightly-fuzz`, §12)

### Track A — MPC (dev machine, parallel with hardware)

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
- [ ] Tune `pi_kp` / `pi_ki` and the MPC model (`mpc_tau_s`, `mpc_gain_c_per_pwm`, weights) on the real loop; decide `solver: pi` vs `mpc` in `config.example.yaml` (needs the aquaero)

### Track B — hardware (Pi USB; fake sysfs anywhere)

- [ ] USB host: `dtoverlay=dwc2,dr_mode=host` (`deploy/host-usb.sh`), powered hub
- [ ] Spike: `lsusb`, `sensors`, pwm list; **is Quadro writable via XT6**
- [ ] Spike: **does XT6 revert after the Pi stops writing?** If not, software
      sensor + firmware timeout (see §2); then decide whether `release()` runs at exit
- [x] `hw/xt6.py` read/apply, udev `0c70`, sysfs root injectable
- [x] `hw/xt6.py`: `pwmK_enable` re-checked on every apply (re-plug)
- [x] Startup rejection: `xt6.fans` keys == `mpc.channels`, `xt6.temp_map` keys == `mpc.temps`
- [x] `xt6.fans`: one entry per fan with `pwm` and optional `rpm`; legacy `map` / `fan_map` rejected
- [x] `test_hw_map.py` / fake hwmon in CI
- [ ] Confirm the hwmon ABI on the real device (`tempK_input` millidegrees, `pwmK` 0..255,
      `pwmK_enable` semantics) and that the udev rule makes `pwmK` / `pwmK_enable`
      group-writable for the service user (`ls -l /sys/class/hwmon/hwmon*/pwm*`)
- [ ] `pytest.mark.hardware` live device test on the Pi with the aquaero attached
      (the test exists; so far it only skips)
- [ ] Verify every `xt6.temp_map` entry against the physical sensor (warm one, watch it move)
- [x] systemd unit: `Type=notify`, `Wants=`+`After=network-online.target`,
      `Restart=always`, `WatchdogSec`, `TimeoutStartSec`,
      `ExecStart=/opt/aqua-bridge/.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml`,
      SIGTERM stop path writes `fallback_pwm` (no `ExecStop=`)
- [x] udev rule for hwmon `pwm*` group write (`plugdev`)
- [x] `deploy/install-pi.sh`; provisioning verified on a Zero W (service left disabled)
- [ ] Enable the service on the Pi once the spike is answered (`systemctl enable --now aqua-bridge`)

### Glue and UI

- [x] `control/loop.py`: read → step → compose → apply → watchdog; smoke-run on the Pi with `--source sim`
- [ ] `control/loop.py` on the Pi against the aquaero (service running)
- [x] MQTT + HA discovery code (host + temps + fans), publisher thread, reconnects
- [ ] MQTT against a live broker; HA discovery check (entities appear, setpoint number works,
      PWM numbers appear only in manual and disappear when leaving it)
- [x] HTTP API: `GET /api/state`, `/api/health`; `POST /api/mode`, `/api/setpoint`, `/api/pwm`, `/api/preset`, `/api/auto`
- [x] HTTP HTML: same five pages as Digole, poll `/api/state`
- [ ] HTTP HTML Host section: `/api/state` carries no host metrics, so it shows dashes
- [x] Tests: HTTP talks to the command sink, not hwmon; Auto rejects raw PWM; fuzz JSON → 4xx
- [ ] Digole: protocol, pages, touch, hit-test
- [ ] DS18B20 + `w1-gpio` last
- [ ] README: bring-up on a **fresh** Pi (checklist §10)

### Upgrade

- [ ] Run on Zero 2 W, same config
- [ ] 64-bit Lite if needed — no API change

---

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
Restart=always
WatchdogSec=30
TimeoutStartSec=120

[Install]
WantedBy=multi-user.target
```

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
the visible “device absent” state. `WATCHDOG=1` every tick (`dt` 2 s),
period far below `WatchdogSec`. `Restart=on-failure` is not enough: OOM
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
   (`USER` is the service account and its group). Idempotent. It
   installs the apt packages, creates the install dir and the
   `--system-site-packages` venv, runs `pip install -e . --no-deps` as
   the service user (aborting if pip tries to fetch numpy), installs
   `config.example.yaml` as `/etc/aqua-bridge/config.yaml` mode 640
   `root:USER` **only if absent** (the MQTT password must not be
   world-readable), installs the udev rule and re-triggers it, installs
   the unit with `User=` substituted, reloads systemd and verifies the
   unit. It does **not** enable or start the service. Run before the
   code is in place, it creates the directory, warns, skips the pip step,
   and finishes; rerun it after the rsync.
6. Edit `/etc/aqua-bridge/config.yaml`: `mpc.channels` / `mpc.temps`,
   `xt6.fans` (one `{pwm: pwmN, rpm: fanN}` entry per channel) and
   `xt6.temp_map` with exactly the same keys (the daemon exits 2
   otherwise), MQTT
   host and credentials, `http.enabled` / `mqtt.enabled`. Always pass
   `--config /etc/aqua-bridge/config.yaml`.
7. USB: dwc2 host, powered hub, XT6 on USB, Quadro on aquabus only.
8. `lsusb` / `sensors` — the USB spike (§2): attribute names and units,
   Quadro PWM, firmware revert. Check `ls -l /sys/class/hwmon/hwmon*/pwm*`
   shows group `plugdev` with write; run
   `.venv/bin/python -m pytest -m hardware` as the service user. Warm
   each mapped temperature sensor and watch the right `obs.temps` key
   move (a swapped `temp_map` is invisible to the gate, §3).
9. One diagnostic tick as the service user:
   `.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml --once`
   prints the observation and the command, applies it, and ends with the
   stop write: the fans are left at `fallback_pwm` with `pwmK_enable` in
   manual mode.
10. `sudo systemctl enable --now aqua-bridge`; `journalctl -u aqua-bridge`
    (a start timeout loop means the device or its permissions are
    missing); `/api/health` if HTTP is enabled; MQTT entities in HA;
    Digole later.

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
.venv/bin/python -m pytest -m "not hardware"        # HYPOTHESIS_PROFILE=dev by default
.venv/bin/python -m aqua_bridge --config config.example.yaml --source sim --ticks 5 --sim-speed 0
.venv/bin/python tools/bench_step.py                # step() timing per solver, JSON
```

Golden trajectories change only on purpose:
`AQUA_BRIDGE_REGEN_GOLDEN=1 .venv/bin/python -m pytest tests/test_mpc_nominal.py`,
then review the diff.

To the Pi:

```bash
# from the repo root on the development machine
rsync -az --delete --exclude .venv --exclude __pycache__ \
  --exclude private.md --exclude secrets \
  ./ USER@PI-HOST:/opt/aqua-bridge/

ssh USER@PI-HOST 'cd /opt/aqua-bridge && HYPOTHESIS_PROFILE=pi .venv/bin/python -m pytest -m hardware'
ssh USER@PI-HOST 'cd /opt/aqua-bridge && .venv/bin/python -m aqua_bridge --config config.example.yaml --source sim --ticks 10'
ssh USER@PI-HOST 'cd /opt/aqua-bridge && .venv/bin/python tools/bench_step.py'
```

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
`model.py` contract. The code arrived as three PRs — `mpc-core` (contract,
gate, PI and MPC, CI), `hw-xt6` (hwmon adapter), `glue` (loop,
supervisor, entry point, HTTP, MQTT, deploy) — and this spec update was
the fourth.

### CI (`.github/workflows/ci.yml`)

Triggers: push to `main`, every pull request, a nightly schedule
(`17 3 * * *`), and `workflow_dispatch`. Permissions `contents: read`.
Superseded PR runs are cancelled; runs on `main` and the nightly never
are.

| Job | When | What | Timeout |
|-----|------|------|---------|
| `lint` | push, PR, dispatch | Python 3.13, `ruff==0.16.7`: `ruff check .`, `ruff format --check .` | 5 min |
| `test (latest)` | push, PR, dispatch | Python 3.14, `pip install -e ".[dev,http,mqtt]"`, `HYPOTHESIS_PROFILE=ci`, `pytest -m "not hardware" --durations=15` | 15 min |
| `test (pi-parity)` | push, PR, dispatch | Python 3.13 with the Pi’s apt versions pinned (numpy 2.2.4, pyyaml 6.0.2, pytest 8.3.5, hypothesis 6.130.5, aiohttp 3.11.16, paho-mqtt 2.1.0), `pip install -e . --no-deps`, same pytest | 15 min |
| `nightly-fuzz (latest, pi-parity)` | schedule, dispatch | same installs, `HYPOTHESIS_PROFILE=nightly` (randomized, 1000 examples) | 60 min |

The ruff pin in `pyproject.toml` `[dev]` and in the workflow move
together. `shellcheck` is present on the runners, so `tests/test_deploy.py`
runs it. First runs took about 8 s for `lint`, 4–6 min for
`test (latest)` and 3–5 min for `test (pi-parity)`. A nightly failure
prints a `@reproduce_failure` blob in the log.

---

## 13. Rollout order (with parallelism)

1. **Done.** GitHub repo exists, public, protected `main` (§12).
   Site-specific inventory in `private.md`.
2. **In parallel:** track A (MPC + tests) on the dev machine **and** USB
   spike on the Pi (hub, XT6, hwmon, **firmware revert**). The core does
   not need hardware. Track A **done** (PI and MPC, CI). Spike **open**:
   the aquaero is not connected.
3. XT6 adapter once the spike answers the Quadro PWM **and** revert
   questions. **Written** against the assumed hwmon ABI and a fake tree;
   confirm on the device, adjust the map or the adapter if the spike
   disagrees.
4. Glue loop, MQTT. **Code done**; Pi provisioned with `install-pi.sh`,
   sim smoke run and benchmark on the Zero W; service left disabled; no
   live broker yet.
5. HTTP API + HTML (same pages as Digole). **Done** (Host section gap,
   §8).
6. Hardware bring-up: spike, hwmon permissions, `-m hardware`, enable
   the service, tune and choose the solver, MQTT/HA live check.
7. Digole + touch.
8. DS18B20.
9. Zero 2 W with no API change.
