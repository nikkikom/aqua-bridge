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
supervisor, loop, hardware adapter, HTTP/MQTT, deploy. The code now runs in
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
| upgrade | same codebase, same config, on a Raspberry Pi Zero 2 W (**done**, 2026-09-16, §8 items 50, 51) |

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
          │ USB (HID)                  │ 1-Wire (sysfs)       │
          ▼                            ▼                      │
  ┌──────────────────────────── Raspberry Pi ─────────────────┴───┐
  │ gate per sensor → zone trust → estimator (latent drives)      │
  │ → DAS MPC or PI-like DAS form → per-zone fallback → PWM (HID)  │
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
  Pi (`lsusb`: vendor `0c70`, product `f001`). Its PWM is writable through
  the aquaero (§8 item 32), and the daemon commands it that way: the
  Quadro's outputs 1–4 are the aquaero's outputs `pwm5..pwm8` with
  tachometers `fan5..fan8`, its sensors 1–4 the aquaero's aquabus slots
  `bus1..bus4` (§3 Track B, §8 item 85). On its own USB port (`f00d`)
  instead, it is a second hidraw device and the config lists both under
  `aquacomputer:`; a config that commands aquaero `pwm5..pwm8` and a quadro
  entry without `serial:` is refused, because a Quadro on aquabus ignores
  writes over its USB (a quadro entry with a serial is taken as a second
  Quadro on its own USB port, with a warning). How many Quadro
  temperature inputs are usable is known only once it is connected;
  `config.example-das.yaml` binds none of them.
- **Supported topology (owner decision 2026-09-16, §8.1):** exactly one
  controlling controller — the aquaero — with slave devices, the Quadro
  among them, hanging off it over aquabus. Two independent controllers are
  not supported yet, though they might be later. Reaching one controller
  over two access paths at once — aquabus and its own USB, both live —
  is out of design and must not be relied on: with the aquabus-attached
  Quadro also reachable on its own USB, the first control-report write
  over that USB link failed (`SET feature report 0x03: [Errno 110]
  Connection timed out`), after which its control endpoint stalled for
  good (`GET ...: [Errno 32] Broken pipe`); a USB-level reset went through
  but the kernel still could not configure the device (`usb 1-1.2: can't
  set config #1, error -32`), and only a physical replug recovered it. Its
  status reports kept working throughout, and the fan it drove kept
  running, because control went through the aquaero the whole time.
  Hidraw support for the Quadro's own USB port stays in the code
  (`hw/aquacomputer.py`, `hw/sources.py`) and is simply unused in this
  installation.
- The daemon reads and writes both controllers through **hidraw**
  (`/dev/hidrawN`, `hw/hidraw.py`), not the Linux `aquacomputer_d5next`
  hwmon driver (owner decision 2026-09-15, §8.1). It reads the status
  report each controller sends about once per second (2 ms on the Pi) and
  writes every channel of one controller with a single control feature
  report (9–13 ms, §2 "hidraw check"); through hwmon a `pwmK` read took 210 ms and a write 420 ms,
  about 5 s per tick with 8 outputs (§2 "USB spike results"). No kernel
  driver beyond `hid-generic` is needed. Raspberry Pi OS kernels do not
  include the hwmon driver anyway; `deploy/install-aquacomputer-dkms.sh`
  still builds it with DKMS and a local fix for later use, but
  `install-pi.sh` does not run it (§9).
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
- **Board (2026-09-16, §8 items 50, 51):** the controller is a Raspberry
  Pi **Zero 2 W** (4 cores at 1.0 GHz, 64-bit Raspberry Pi OS Lite,
  trixie, kernel 6.18, aarch64), running the same code and the same DAS
  config as the single-core, 32-bit Zero W it replaced; no `armv6`-only
  code existed to remove and nothing in the API changed. USB OTG is
  unchanged. The Zero W stays the reference for the older board's
  numbers where those are still cited (the step-budget fallback of §8
  item 73, and measurements taken before the move) — it is not what the
  code now runs on.

**Status:** only the Pi is up. The aquaero 6 XT, Quadro, Digole, the
DS18B20 buses and the DAS itself are not connected yet; the spike
questions below and the whole hardware validation (§8, §13) are open.
Everything else in this document runs against simulators, fake sysfs
trees and captured HID reports.

### Risk (first-evening USB spike)

Over USB the aquaero exposes its **own 4 fans** (status and control
reports, §3 Track B). Quadro channels over aquabus may be sensors only,
with no PWM write. Until verified (answered 2026-09-15: the Quadro's
outputs are the aquaero's outputs 5–8 and writable, §8 items 32, 85):

1. Which temperature, fan and PWM channels each controller reports, and
   their units. Answered for USB-attached devices (results below);
   `tools/aquacomputer_probe.py` shows them.
2. Whether Quadro PWM is writable through XT6 (HID).
3. **Does XT6 revert on its own after the Pi stops writing?** Write a PWM
   over HID, then stop. If the fan stays pinned, firmware curves are
   *not* a watchdog. Then try the alternative: write the controller output
   into an aquaero **software / virtual sensor**, with the firmware’s
   timeout fallback and an identity curve onto PWM. That is a hardware
   watchdog for free — only if the control report (or liquidctl) can
   write that sensor. Record which path works; the daemon follows it.

4. Which Quadro temperature inputs carry a reading in a status report
   (through the XT6, or on the Quadro's own device), so they can be
   bound in `temp_map`.

If (2) fails: the Quadro goes on its own USB port (owner decision), a
second hidraw device next to the aquaero, run with `--source composite`
and an `aquacomputer:` list of both devices (§3 Track B). This blocks the hardware
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

At exit the adapter leaves every commanded channel at `fallback_pwm` (an
aquaero channel assigned to its manual preset with power limits 0 / 100 %).
Every write is live and not saved (§8 item 86): after a power cycle the
controllers run the configuration last saved in their memory, whatever the
daemon wrote before. `AquacomputerAdapter.release()` (restore, with a live
write, the control settings the first control report read saw: the
aquaero's preset, control source and power limits, handing its channels
back to their firmware controllers, and the Quadro's duty) exists but is
**not** called, because it would undo the `fallback_pwm` write. If spike
(3) shows the firmware controller is the better state after exit, calling
it is a one-line change in `__main__.py` (§8 items 33, 76).

### Watchdog layering, and the network outside the cooling path

Four watchdogs, innermost first. Each one catches what the one inside it
cannot, and each fires later than the one inside it — that ordering is the
whole design, because the cheap recovery has to get its chance before the
expensive one.

| # | Watchdog | Catches | Cannot catch | Fires after | Set in |
|---|----------|---------|--------------|-------------|--------|
| 1 | the daemon's own sensor gate (§3) | a sensor path that went quiet or implausible | anything that stops the process | `fallback_hold_s`, then the ramp | `config.yaml` |
| 2 | systemd service watchdog | a tick that stops sending `WATCHDOG=1` — a blocked read, a wedged control endpoint, a publisher hook that hangs | a kernel or systemd hang; the daemon being killed outright | `WatchdogSec=45` | `deploy/aqua-bridge.service` |
| 3 | SoC hardware watchdog (BCM2835) | a kernel or systemd hang: PID 1 stops pinging `/dev/watchdog` and the board resets | anything userspace can still act on — it only ever resets the board | `RuntimeWatchdogSec=60 s` | `deploy/install-board-watchdogs.sh` |
| 4 | the aquaero's software-sensor alarm | **nothing writing the heartbeat at all** — daemon dead, board dead, USB gone, power gone | nothing above it; it is the last line | 30 s of silence on `softN` (set on the device, §8 items 33, 84) | the controller itself |

Layer 4 is the last line and is **independent of this board**: it lives in
the aquaero's own firmware, runs off its own clock, and takes every output
to 100 % when the heartbeat stops. Everything above it is an attempt to
never get there. That is why 3 sits above 2: a daemon restart is the cheap
recovery and has to get its chance before a board reset.

**Both of them spend layer 4, and the arithmetic says so.** Layer 4 fires
after 30 s of silence on `soft1`. Layers 2 and 3 fire after 45 s and 60 s.
Neither can therefore beat the controller, and neither is meant to:

- Layer 3 spends it on the boot alone. This board takes 28 s from power to
  its first controller write (measured below), which is already the whole
  window, and a shutdown comes before that. A reset means the alarm.
- Layer 2 spends it *before* it even fires, whenever the daemon is the one
  writing the heartbeat. `WATCHDOG=1` goes out at the end of a tick, and the
  heartbeat goes out inside `apply()` on the same tick, after the duty work and
  only when that succeeded (Track B in §3). So "no `WATCHDOG=1` for 45 s" is
  also "no heartbeat for at least 45 s": at 30 s `soft1` has already fallen
  back to its 90.00 °C, the alarm has already selected profile 2, and every
  output is already at 100 % — fifteen seconds before systemd sends the signal.
  The restart ends the alarm; it does not avoid it.

`WatchdogSec` cannot be lowered under the 30 s to change that. It has to stay
above the bound `check_watchdog` computes, and that bound is 18.5 s for
`config.example-das.yaml` as it ships but **31.5 s** with the Quadro on its own
USB port — already past the controller's 30 s, with no room for the publishers
on top. A watchdog that fires inside the controller's window would have to fire
inside a legitimate slow tick, which is worse: it would kill a daemon that is
still doing its job. 45 s is the right value and the alarm is its accepted cost.

The one configuration in which a watchdog kill is invisible to the fans is the
one the board runs **today**: `heartbeat_sensor: 0` in both example configs,
with a separate `aqua-heartbeat.service` writing `soft1` every 2 s. That
service is not this daemon and a stalled tick does not stop it, so the alarm
never fires and the fans hold their last commanded duty until the daemon is
back. That changes the moment `heartbeat_sensor: 1` is set, which is what the
DAS example says to do once the daemon's heartbeat has run against the
hardware (§8 item 93) — so the paragraph above, not this one, is the case to
design for.

**A slow tick degrades to a restart, not to silence.** `WATCHDOG=1` goes out
at the end of every tick whose controller work did not raise; two pings
further apart than `WatchdogSec` and systemd kills the daemon.
`WatchdogSignal` is left at its `SIGABRT` default on purpose: `SIGTERM` would
run the stop path, which writes `fallback_pwm` through the very sink whose
slowness tripped the watchdog, block until `TimeoutStopSec` and be `SIGKILL`ed
anyway — later, no cleaner, and jolting the fans to `fallback_pwm` on every
timeout as well.

**What happens to the fans during that restart**, with the daemon writing the
heartbeat, in order:

| t | what |
|---|------|
| 0 s | the last tick that finished: duties written, heartbeat written |
| 30 s | `soft1` falls back to 90.00 °C, the alarm selects profile 2, **every output 100 %** |
| 45 s | systemd sends `SIGABRT`; the process dies at once and writes nothing (no `fallback_pwm` jump — only the *clean* stop path writes that, §9) |
| ~53 s | `RestartSec=5` plus about 3 s of start: the first tick has no `last_cmd`, so `resolve_prev` takes `prev` from the duty it reads off the device — the alarm's 100 % — and rate-limits down from there. That tick's `apply()` writes the duties and then the heartbeat |
| ~55 s | the resumed heartbeat clears the alarm (~2 s, measured) and profile 1 is reloaded |
| +`duty_mismatch_s` | the profile switch reloaded the **saved** profile, so every duty written live is gone and the outputs sit at that profile's preset (20 % on the owner's controller). The duty verification sees the mismatch, re-reads the control report, notices the profile change and writes every channel again ("The active profile" above bounds this edge) |

So a watchdog restart is not invisible to the fans: it is about 25 s at 100 %,
then a few seconds at the saved preset, then the daemon's own command again.
Loud first and quiet second, which is the right order — but not "the fans never
moved", and nothing here reduces cooling below what a working daemon would ask
for except that last bounded dip, which the duty verification is there to end.

`StartLimitIntervalSec=0` closes the one hole that would be real silence:
systemd's default rate limit (5 starts in 10 s) parks a unit in `failed`, and
a parked unit never writes the controllers or the heartbeat again.

**Sizes, argued from this board** (Zero 2 W, kernel 6.18, 2026-09-18):

- `step()` for the DAS MPC against the DAS plant: p50 50 ms, p99 82 ms, max
  87 ms over 60 ticks (`tools/bench_step.py`) — the `mpc.budget_ms` 250 /
  `budget_alarm_ms` 350 in `config.example-das.yaml` are 2.9× the p99, as that
  file says.
- A cold start of the daemon: 1.4 s to import numpy, PyYAML and `aqua_bridge`,
  2.9 s from the command line to the first applied command with
  `--source sim --sim-plant das`. That plus `RestartSec=5` is layer 2's
  *recovery*, the ~8 s after the kill — not its cost, which starts with the
  45 s of silence that tripped it.
- The bound `check_watchdog` computes from `config.example-das.yaml` —
  `mpc.dt` + `budget_alarm_ms` + the controllers' worst-case I/O — is **18.5 s**
  as the example ships (the Quadro on aquabus) and **31.5 s** with the Quadro
  on its own USB port. `WatchdogSec=45` leaves 26.5 s and 13.6 s of margin for
  what that bound explicitly does not count, the publishers and the recorder;
  their largest single item on the loop thread is one `vcgencmd get_throttled`
  at `host_health.vcgencmd_timeout_s` = 2 s, at most once a minute
  (`tests/test_deploy.py` checks the margin against both).
- A healthy boot: kernel at monotonic 10.4 s (firmware before that), the
  heartbeat service started at 27.0 s and its first write to the aquaero at
  **27.7 s**. `RuntimeWatchdogSec=60 s` is one service-watchdog period plus
  margin above `WatchdogSec=45`, and systemd then pings every 30 s — far above
  anything a tick does, and short enough that a hung board does not stay hung.
  `RebootWatchdogSec=120 s` bounds a hung shutdown.
- The journal is persistent on this board (`Storage=persistent`). Left at
  systemd's defaults `SystemMaxUse` is 10 % of the filesystem — about 1.5 GB of
  the owner's 15 GB card — so `install-board-watchdogs.sh` caps it at 200 MB,
  16 MB per file, 30 days, `SyncIntervalSec=5m`.

**The network is outside the cooling path.** The owner's rule: switching off
the home router must never degrade cooling. Traced, path by path, on
2026-09-18:

- **The unit no longer waits for the network.** `Wants=`/`After=network-online.target`
  used to be in `deploy/aqua-bridge.service`. With nothing to associate with,
  `NetworkManager-wait-online` spends its full 30 s default (`nm-online`'s
  documented timeout; 1.0 s on a healthy boot here) and the daemon's first
  write to the controllers is that much later — one whole aquaero window, spent
  on a router. Removed; `tests/test_deploy.py` now fails if any directive in
  that unit names the network.
- **MQTT.** `MqttService.start` calls `connect_async` + `loop_start`, never
  `connect`: paho's network thread owns DNS, the TCP connect and the
  reconnects (`reconnect_delay_set(1, 60)`), so a broker that is down at boot
  or dies mid-run costs nothing on the loop thread. `on_tick` returns
  immediately while `client.connected` is false, and paho's `publish()` under
  `loop_start()` only appends to a deque and pokes a non-blocking socketpair —
  it never writes the broker socket from the caller's thread. Every exception
  is caught and counted in `publish_errors`.
- **Home Assistant discovery** is `client.publish(qos=1)` per entity on the
  same path, sent from `on_tick` after a (re)connect or a `manual` crossing.
  Same deque, same non-blocking enqueue.
- **The HTTPS API** runs on its own asyncio loop in a daemon thread.
  `HttpService.start` blocks the main thread for at most 10 s *once*, before
  the loop starts, and it is network-independent: the default `http.bind` is
  `0.0.0.0`, so nothing is resolved and nothing is dialled. A bind failure is
  logged and the daemon runs on without the API. (Keep `http.bind` an address
  and not a hostname: aiohttp would resolve a name there, and with no DNS that
  would spend the 10 s.)
- **Nothing in the daemon does a DNS lookup or a blocking network call on the
  loop thread.** The only `socket` use outside the publishers is
  `sdnotify.py`'s `AF_UNIX` datagram to `$NOTIFY_SOCKET`; the only subprocess
  is `vcgencmd`, bounded by a documented key.
- **The order inside `Loop.tick` is what makes this hold.** `on_tick` — where
  every publisher hangs its hook — runs *after* `sink.apply()` and *after*
  `notifier.watchdog()`. A publisher that hangs therefore cannot hold back the
  command this tick already put on the fans, and cannot swallow this tick's
  ping. It does delay the next tick, and that is exactly what layer 2 is for:
  the daemon goes quiet, systemd restarts it, and the fans hold meanwhile.
  `tests/test_loop.py::test_the_publishers_only_ever_see_a_tick_that_has_already_reached_the_fans`
  pins the order.
- **SMART arrives over the network and its absence cannot lower a fan.** A
  reading older than `bays.smart_max_age_s` (300 s) is simply not there — never
  substituted, never carried forward. The bay is then estimated from its
  proximal 1-Wire sensor, which is wired to this board, with a wider variance;
  the DAS solvers work to `limit - k·sigma`, so a wider sigma can only raise
  duty. A sigma past `sigma_fault_c` is a zone fault: hold, then ramp high.
  Losing the network makes the fans work harder, never less.
- **Recovery never escalates.** `deploy/aqua-net-recover.sh` (a `oneshot` unit
  behind a 5 min timer) probes the gateway its own interface was handed — no
  hardcoded address — and, after two failed checks, re-associates the Wi-Fi
  interface with `nmcli device disconnect`/`connect`. That is all it may do:
  no reboot in any form, no `systemctl`, nothing that touches
  `aqua-bridge.service`, `aqua-heartbeat.service` or a controller. After three
  fruitless re-associations it logs one line and then says nothing at all until
  a probe succeeds, so a router that is simply switched off costs one journal
  line — not a bounce loop, and not a line every timer period either. Three
  details make that true rather than merely intended: the counter moves
  *before* the `nmcli` pair, so a re-association that hangs until systemd kills
  the unit still counts as the attempt it was; each `nmcli` carries an explicit
  `--wait` (`AQUA_NET_NMCLI_WAIT_S`, 20 s) because `device connect`'s own
  default is 90 s, longer than any sane start timeout; and `device disconnect`
  blocks autoconnect until a manual activation (`nmcli(1)`), so autoconnect is
  restored *between* the disconnect and the connect — a run killed in the
  middle then leaves NetworkManager retrying on its own instead of leaving the
  board off the network until somebody logs in locally. It exists
  because of the outage below; it is not part of cooling and cannot become part
  of it. A gateway watchdog that *reboots* is forbidden outright: with the
  router off it is a reboot loop, and every reboot is a heartbeat gap and a jolt
  on the fans.

**The outage that shaped this (2026-09-17).** The board dropped off the
network for hours. It kept running and kept feeding the aquaero's software
sensor the whole time, so the fans never left the quiet profile — the layering
worked, and the network's absence was invisible to the drives. The cause was
Wi-Fi power save on the BCM43430 (`brcmf_cfg80211_set_power_mgmt: power save
enabled` in the kernel log at boot, `power save disabled` when it was turned
off by hand 19 minutes later). Power save is now off through a NetworkManager
drop-in rather than a connection profile — the profile carries the SSID, which
stays out of this repository, and a profile written later by the imager
inherits the drop-in. The journal was made persistent at the same time, which
is what the size cap above is for.

### USB spike results (2026-09-14)

Since 2026-09-15 the daemon no longer uses the hwmon driver described
here: it reads and writes both controllers over hidraw (§8.1), because of
the `pwmK` cost measured below. This section stays as the record of the
spike; the HID layouts it led to are in §3 Track B.

Temporary setup: the aquaero 6 XT and the Quadro each on its own USB port
behind a powered hub, one fan and one thermistor on each, nothing on
aquabus; kernel `6.18.39+rpt-rpi-v6`, driver from
`deploy/install-aquacomputer-dkms.sh` (§9). Questions (2)–(4) above stay
open: they need the Quadro on aquabus.

- **Driver.** Not in the Raspberry Pi OS kernel
  (`CONFIG_SENSORS_AQUACOMPUTER_D5NEXT` is not set): both controllers bind
  to `hid-generic` and no hwmon device appears. Built with DKMS the driver
  binds both. Unpatched, every `pwmK` write failed with `EAGAIN` and a
  `dma_map_phys` kernel warning, after the control report itself had been
  sent: the driver sends its follow-up report from static module data,
  which is not DMA-mappable on the Zero W. The DKMS package carries a fix
  (§9).
- **Attributes.** hwmon names `aquaero` and `quadro`. aquaero:
  `temp1..8` sensors, `temp9..16` virtual sensors, `temp17..20` calculated
  virtual sensors, `fan1..4` plus flow `fan5..6`, `pwm1..4`, per fan
  `inN` / `currN` / `powerN`, and `tempK_offset` for the 8 sensors. Quadro:
  `temp1..4` sensors, `temp5..20` virtual sensors, `fan1..4` plus flow
  `fan5`, `pwm1..4`. Units as §3 Track B assumes (millidegrees, rpm,
  0..255). An input with nothing connected fails its read with `ENODATA`,
  which the adapter already turns into `None`. The daemon's config names
  are no longer these numbers (§3 Track B, §8 item 85): the driver's
  "virtual sensors" are the software sensors `softN`, its "calculated virtual
  sensors" the aquaero's virtual sensors `virtN`, and the flow sensors
  `flowN`.
- **No `pwmK_enable`** on either device: the sysfs adapter's `apply()`
  had nothing to switch and its `release()` nothing to restore.
- **A `pwmK` write reconfigures the aquaero channel.** The driver points
  the channel's control source at its manual preset and sets its minimum
  power to 0 % and maximum to 100 %. The firmware controller that drove
  the channel before is no longer assigned, nothing in hwmon assigns it
  again, and after the daemon exits the channel holds its last preset.
  Reading `pwmK` on the aquaero returns that preset, not the effective
  output: it read 0 while the fan ran at 1082 rpm from its firmware
  controller. On the Quadro a write only sets the channel's manual value.
  Save both control reports before the first write (HID feature reports
  `0x0b`, 2707 bytes, and `0x03`, 961 bytes): written back followed by the
  follow-up report, they restored the configuration byte for byte.
- **Output duty in the status report** (verified 2026-09-15 over hidraw):
  each aquaero fan block carries the duty the output actually drives at
  offset `+0x02`, in 1/100 % (1412 on the channel commanded to 14.12 %,
  10000 on the others, which ran their firmware controllers at 100 %).
  The driver does not decode this field. The Quadro reports its duty at
  `+0x00`. `obs.pwm` is this value on both controllers (§3 Track B).
- **Cost.** `tempK_input` and `fanK_input` come from the cached status
  report. Every `pwmK` read fetches the whole control report, and every
  write fetches, patches and sends it, with the driver's 200 ms spacing
  between control operations (both devices):

  | Operation | Median time |
  |---|---|
  | `fanK_input` or `tempK_input` read | 1 ms |
  | `pwmK` read | 210 ms |
  | `pwmK` write | 420 ms |

  The sysfs adapter read every `pwmK` and wrote every channel on every
  tick: with 8 outputs that is about 5 s per tick, the whole `dt` of the
  DAS example (§8 item 74, done by moving to hidraw).
- **The outputs run in PWM mode.** The output voltage stays at 12.1 V from
  5 % to 100 %; 0 % switches the output off. The aquaero reports 0 mA and
  0 W in this mode; the Quadro reports current and power.
- **PWM to rpm**, one test fan per controller, 10 s settle per step, up
  from 0 % and back down:

  | PWM | aquaero up | aquaero down | Quadro up | Quadro down |
  |---|---|---|---|---|
  | 0 % | 0 | 0 | 0 | 0 |
  | 5 % | 0 | 0 | 694, unstable | 569, unstable |
  | 10 % | 0 | 0 | 167 | 136 |
  | 15 % | 0 | 125 | 165 | 251 |
  | 20 % | 0 | 185 | 186 | 271 |
  | 25 % | 250 | 256 | 302 | 391 |
  | 30 % | 304 | 316 | 322 | 424 |
  | 40 % | 410 | 423 | 452 | 550 |
  | 50 % | 502 | 513 | 565 | 581 |
  | 60 % | 605 | 623 | 675 | 693 |
  | 70 % | 709 | 727 | 704 | 806 |
  | 80 % | 820 | 828 | 840 | 931 |
  | 90 % | 926 | 939 | 968 | 1066 |
  | 100 % | 1078 | 1078 | 1087 | 1087 |

  The aquaero fan starts only at 25 %, keeps turning down to 14 %
  (120 rpm) and stops at 13 %. The Quadro fan keeps turning at 9 %
  (about 123 rpm after a minute); at 5–8 % its speed jumps between 0 and
  860 rpm, faster than at 10 %. Up and down differ by up to 100 rpm on the
  Quadro: 10 s may be too short, or the firmware ramps; not checked
  (§8 item 75).

### hidraw check (2026-09-15)

The branch with the hidraw adapter on the Pi, both controllers on their own
USB ports, the DKMS module removed (§8 item 82), run as a non-root user in
`plugdev`.

- **Discovery and decoding.** `tools/aquacomputer_probe.py` listed both
  controllers (aquaero on USB interface 2, Quadro on interface 1) and
  decoded every status and control report field; `pytest -m hardware`
  passed. `HID_UNIQ` equals the serial in the status report on both
  devices (the adapter now checks that when `serial:` is configured).
- **Timing** (medians):

  | Operation | Time |
  |---|---|
  | status report read (`read()`) | 2 ms |
  | first open, waiting for the first status report | 0.1–0.9 s |
  | `apply()` with every duty already held (no SET) | about 1 ms |
  | `apply()` with a changed duty | 9–13 ms |
  | aquaero SET + secondary report | 6.5 ms + 0.8 ms |
  | Quadro SET + secondary report | 2.9 ms + 0.8 ms |
  | consecutive changed writes on one controller, old 200 ms gap | about 205 ms |
  | one SET with all 4 outputs: aquaero / Quadro | 16 ms / 26 ms |

  The status report showed a new duty in the next report (≤ 1 s); fan rpm
  then took 3–6 s to settle. Two control report GETs 30 s apart with no
  write in between were byte-identical on both devices, so writing from
  the cached report does not revert fields the firmware changes by itself.
- **Gap between writes.** Back-to-back writes (SET + secondary report) on
  the aquaero failed with `EPIPE` at a gap of 0 ms (18 of 20) and 25 ms (15
  of 30), and never at 50, 75, 100 or 150 ms (30 each). The Quadro wrote 20
  of 20 at 0 ms. Owner decision: `ctrl_gap_ms` defaults to 100 ms on the
  aquaero and 0 on the Quadro (§8.1).
- **One SET, all outputs.** One SET wrote all four outputs of each
  controller (read back from the control report), and `release()` restored
  both control reports byte for byte. Not every output followed in the
  status report: aquaero outputs 1 (no fan) and 2 (fan) went to 40 % and
  30 %, outputs 3 and 4 (no fan) stayed at 100 %; Quadro output 3 (fan)
  went to 30 %, outputs 1, 2 and 4 (no fan) stayed at 100 %. The four Quadro
  channel regions (0x55 bytes from the duty offset − 2) are identical apart
  from the duty.
- **aquaero output mode.** The aquaero controller blocks differ: the word at
  block +0x0E is `0x0502` on outputs 1 and 2 and `0x0501` on outputs 3 and 4
  (the word at +0x02 is 1000 on 1–2 and 1600 on 3–4, not interpreted). The
  +0x0E word is the output mode: output 4 (block `0x248`) at `0x0501`
  followed the duty as a voltage (27 % → 3.3–3.6 V, 66 % → 8.06 V, current
  and power reported). Changing only that word to `0x0502` (one GET, patch,
  SET + secondary; the control report then differed from the saved copy
  only at `0x257`) made the output PWM: 12.07 V at 30 % and 50 %, the status
  duty equal to the command in the next report, rpm following. So the low
  byte `0x01` is DC voltage mode and `0x02` PWM; the high byte (`0x05` on
  every output) is not interpreted. In DC mode the output did not follow
  low commands (0–20 % commanded, 27–30 % and 3.3–3.7 V reported) and lagged
  a changed command by several seconds. The aquaero reports current and
  power only in DC mode (0 in PWM mode). Outputs 3 and 4 that stayed at
  100 % were in DC mode without a load; why a DC output without a load
  reports 100 % is unknown, and the earlier guess of a firmware start boost
  is withdrawn for the aquaero. The Quadro outputs without a fan also
  stayed at 100 %; the Quadro's mode field is not identified. The adapter
  decodes the aquaero mode and warns about commanded outputs not in PWM
  mode, but does not change it (§8 item 81).
- **Two more fans, PWM to rpm (2026-09-15).** Fans on aquaero outputs 1
  and 4, both in PWM mode (output 4 switched from DC first), swept together
  over hidraw, 10 s settle per step, up from 0 % and back down:

  | PWM | output 1 up | output 1 down | output 4 up | output 4 down |
  |---|---|---|---|---|
  | 0 % | 0 | 0 | 0 | 0 |
  | 5 % | 0 | 0 | 0 | 0 |
  | 10 % | 0 | 0 | 0 | 0 |
  | 15 % | 0 | 354 | 0 | 375 |
  | 20 % | 0 | 354 | 0 | 375 |
  | 25 % | 357 | 354 | 374 | 375 |
  | 30 % | 416 | 430 | 447 | 452 |
  | 40 % | 608 | 606 | 646 | 642 |
  | 50 % | 780 | 785 | 823 | 834 |
  | 60 % | 964 | 953 | 1024 | 1016 |
  | 70 % | 1127 | 1129 | 1205 | 1196 |
  | 80 % | 1305 | 1312 | 1388 | 1398 |
  | 90 % | 1476 | 1496 | 1591 | 1590 |
  | 100 % | 1733 | 1733 | 1857 | 1857 |

  Both fans start at 25 %, stop at 13 %, and hold the same speed from 25 %
  down to 14 % (about 355 and 371 rpm), so those duties differ in margin
  only, not in speed or noise. They were left at 25 %, where a stopped fan
  starts again. Unlike the first test fan on output 2 (120 rpm at 14 %),
  these fans keep a minimum speed of their own. A 1 % search held both
  steady at 14 % for a minute. Up and down agree within about 20 rpm except
  at 90 % on output 1 (1476 / 1496).
- **Output 4 in DC mode, for comparison.** Swept before the mode switch, it
  did not follow the command below about 30 %: commanded 0–20 % gave 26–30 %
  duty, 3.2–3.7 V and 320–970 rpm with large swings; commanded 100 % gave
  63.7 % duty, 7.77 V and 1571 rpm. Duty verification flagged it on every
  step of the minimum search, as intended.
- **Power cycle (§8 item 77).** With 12 V and USB removed from both
  controllers for 30 s, every written setting survived: both control
  reports were byte-identical before and after, the Quadro's power-cycle
  count went from 12 to 13 and the aquaero's u32 at status `0x11` restarted
  (39478 to 32). Control-report writes are stored in non-volatile memory
  when the save report follows them, as it did then; a SET alone is not
  kept (see "Apply without saving" below).
- **Quadro on aquabus (§8 items 32, 34).** With the Quadro on the aquaero's
  aquabus high-speed port (its USB still connected), the Quadro ignored its
  own fan settings: all four outputs ran at 100 % although its control
  report held 9.02 % on output 3. The aquaero's status report gained the
  Quadro's data, matching what the Quadro reports over its own USB:

  | Quadro | aquaero status report | Value |
  |---|---|---|
  | sensor 2 | aquabus temperature slot 2 (`bus2`, `0x77`) | 24.04 °C (Quadro: 24.03 °C) |
  | outputs 1–4 | fan blocks 5–8 (`0x197`, `0x1A3`, `0x1AF`, `0x1BB`), same layout as fans 1–4 | output 3: 1107 rpm, 100 %, 12.10 V, 20 mA |
  | flow | third flow slot (`flow3`, `0xFD`) | 0 |

  Before the aquabus connection these slots read `0x7FFF` (temperatures and
  flow) and rpm `0xFFFF` (fans). The aquaero's control report has blocks
  for fans 5–8 at `0x25C + 20k` and presets beyond 4. Writing aquaero fan 7
  like outputs 1–4 (preset 7 at `0x568` = 902, block `0x284`: min 0, max
  10000, source `0x62`; one SET plus the secondary report) changed only
  those five bytes, and the Quadro's output 3 followed: status duty 9.02 %
  on both devices at once, 1109 rpm down to 120–136 rpm, steady for a
  minute. The Quadro's PWM is writable through the aquaero. The adapter
  supports this since §8 item 85: the aquaero entry commands `pwm5..pwm8`
  and reads `fan5..fan8` and `bus1..bus8`; an output whose fan block reads
  rpm `0xFFFF` (nothing on aquabus) fails the read instead of reporting a
  duty. In the aquaero's control report the aquabus blocks 5–7 read mode
  word `0x0500` (low byte 0, not interpreted) and block 8 is unconfigured
  (source `0xFFFF`, mode `0x0000`); writing block 8 is not verified.
- **Apply without saving (§8 item 84).** A SET of the aquaero's control
  report with preset 1 changed from 25 % to 30 %, **without** report 6,
  took effect in the next status report (duty 30 %, fan 1 speeding up) and
  read back as 30 %. After a power cycle of the controllers (aquaero
  uptime counter at 176 s) the control report was byte-identical to the
  copy saved before the change, preset 1 at 25 % again. Report 6 saves;
  an administrator on the vendor's forum describes report 6 as the one
  that "controls the device, lock unlock, reset", to be sent only
  deliberately. The Quadro's report after a SET
  (`02 00 00 00 02 00 00 00 00 34 C6`) is byte-identical to the Farbwerk 360's
  documented "save permanently" report (aquacontrol `PROTOCOL.md`); for the
  Quadro that is not verified. Software sensor 1 (report `0x07`, `soft1`)
  took 33.33 °C in the next status report (slot `0x85`), held it for its
  300 s timeout and fell back to 40.00 °C; the control report did not
  change. Since §8 item 86 the adapter sends no save report on a write.
- **Software-sensor heartbeat and profiles (§8 item 84).** The aquaero's
  eight software temperature sensors are written with HID **output** report
  `0x07`, 17 bytes: the report id, then eight `u16` big-endian values in
  1/100 °C, `0x7FFF` for "no data" (a slot left at `0x7FFF` keeps the value the
  device already has, so one sensor can be written without blanking the other
  seven). Their settings live in the control report from `0x177`, five bytes
  per sensor: enabled (1 byte), fallback temperature (`u16`, 1/100 °C), timeout
  (`u16`, s). The values show up in the status report at `0x85 + 2i`
  (`soft1..8`); writing one changes nothing in the control report.
  **Byte `0x06` of the control report is the active profile** (0 = profile 1,
  1 = profile 2; it read 0 in every capture taken before the owner configured
  profiles).
  The owner's configuration (2026-09-15, as it stands): software sensor 1
  enabled, timeout 30 s, fallback 90.00 °C; a temperature alarm on that sensor
  selects profile 2 and alarm level 0 selects profile 1; in both profiles all
  eight outputs follow preset 1 (source `0x5C`) with minimum 0 % and maximum
  100 %, profile 1 holding preset 1 at 20 % and profile 2 at 100 % (the earlier
  capture under item 86 had 35 % / 100 % limits and preset 1 at 25 %; the owner
  has changed them since). Measured: a heartbeat of 20.00 °C written every 2 s
  keeps profile 1; 30 s after the last write the sensor falls back to 90.00 °C
  and within 2 s every output — the Quadro's on aquabus included — runs at
  100 %; a resumed heartbeat brings profile 1 back within 2 s, with the control
  report byte-identical to before. A live, unsaved preset written before the
  alarm is **gone** after the round trip: the profile switch reloads the saved
  profile, so the daemon must write its duties again after one (it watches
  `0x06`, §3 Track B). With nothing writing the heartbeat the aquaero goes to
  profile 2 by itself 30 s after any silence, a power cycle included: that is
  the hardware watchdog of items 33 and 84.
- **Live write through the aquaero to an aquabus fan (§8 item 85).** Preset
  7 set to 20 % without the save report showed duty 2000 in the aquaero's
  status report within 1–2 reports, and the Quadro's own report showed 20 %
  and 225 rpm; set back to 9.02 %, the aquaero's control report equalled the
  one read at the start. The rpm of an aquabus fan in the aquaero's status
  report lags the Quadro's own report by several seconds (aquabus polling);
  the duty shows within 1–2 reports.
- **Quadro `ctrl_gap_ms` of 0, and why it stays 0 (2026-09-17).** On a board
  fast enough to issue a control-report GET and SET almost back to back, the
  first hidraw control-report write to the Quadro over its own USB port
  failed and wedged its control endpoint until a physical replug -- the same
  failure the "Supported topology" note above records for a Quadro reachable
  both on aquabus and on its own USB at once: `SET feature report 0x03:
  [Errno 110] Connection timed out`, then `GET ...: [Errno 32] Broken pipe`
  on every attempt after, a USB-level reset that completed but left the
  kernel unable to reconfigure the device (`error -32`), and only a physical
  replug recovered it. Status reports and the fan the Quadro was already
  driving kept working throughout. This is not evidence that `ctrl_gap_ms:
  0` is wrong for the Quadro: the attempt was against the unsupported
  two-access-path setup (§8 item 106), not against a Quadro on its own USB
  as the daemon's sole path to it, and a gap scan run correctly needs a
  Quadro reachable only that one way. It is not a problem in the supported
  topology (owner decision 2026-09-16: one controlling controller, slaves
  behind it -- the daemon never writes the Quadro's control report when the
  Quadro is a slave on the aquaero's aquabus, so this failure mode cannot
  occur there). It would be one if a Quadro were ever driven directly over
  its own USB as a controlling device, which is exactly the config shape
  §8 item 106's startup check does not yet refuse. So the Quadro's timing
  defaults are unchanged, and the action this observation points at is
  already tracked: land item 106's startup check before a Quadro-on-its-own-
  USB config ships, and when one does, re-run the gap scan (§2 "Gap between
  writes") against the Quadro alone before trusting `ctrl_gap_ms: 0` on that
  path.

### aquabus fields checked against the live devices (2026-09-17)

The layout verification items 35 and 89 asked for, in the final wiring: the
aquaero 6 XT on USB, the Quadro a slave on its aquabus with its own USB **not**
connected (§2 "Supported topology", owner decision 2026-09-16). Read-only
throughout -- 98 status reports over two runs (8 and 90 consecutive reports) and
one control-report GET, no SET, no save report, no duty touched, the heartbeat
service left running. The aquaero was on firmware 2104, running profile 1, with
all eight outputs pointed at preset 1 at 20.00 %, one thermistor on its sensor
6, the Quadro's thermistor on its own sensor 2, and a fan on the Quadro's output
3 (the aquaero's aquabus block 7) turning at 255 rpm throughout. Fixtures:
`tests/fixtures/aquacomputer/aquaero-status-aquabus-block7-power.bin`,
`-block7-no-power.bin` (one second apart) and
`aquaero-ctrl-aquabus-all-on-preset1.bin`, serial bytes zeroed; no operator
names are in any of them.

What every decoder assumption now stands at:

| assumption in `hw/aquacomputer.py` | verdict | evidence |
|---|---|---|
| status report: input id `0x01`, 903 bytes, ~1/s | confirmed | 90 reports in 88.6 s, all 903 bytes |
| serial `u16` pair at `0x07`, firmware at `0x0B` | confirmed | equals `HID_UNIQ`; firmware 2104 |
| `temp1..8` at `0x65`, 1/100 °C signed | confirmed | `temp6` 22.93–23.06 °C live, the other seven `0x7FFF` |
| `bus1..8` at `0x75` | confirmed for slot 2 | `bus2` 23.64–23.69 °C = the Quadro's sensor 2; slots 1, 3, 4 read `0x7FFF` because the Quadro has no thermistor there, so the claimed 1:1 sensor→slot map is **unobservable** for 1, 3 and 4 |
| `soft1..8` at `0x85` | confirmed | `soft1` = 20.00 °C, the heartbeat service's value; `soft2..8` = 50.00 °C, their configured fallback (all eight sensors are enabled now; a *disabled* one reads `0x7FFF`, `aquaero-status-no-aquabus.bin`) |
| software-sensor settings in the control report from `0x177`, 5 bytes each: enabled `u8`, fallback `s16` centi-°C, timeout `u16` s | **confirmed against five captured control reports** (item 113) | `aquaero-ctrl-aquabus-all-on-preset1.bin` reads all eight enabled, `soft1` at 30 s / 90.00 °C — the owner's watchdog, the one the heartbeat service feeds and whose alarm drives every output to 100 % — and `soft2..8` at 300 s / 50.00 °C, exactly the steady value their status report shows. The four earlier captures read `soft1` at 300 s / 40.00 °C (item 84's experiment: a value held for exactly 300 s, then 40.00 °C) with `soft3..8` **disabled**, which is exactly which slots those status reports show as `0x7FFF`. So the settings explain every `softN` reading in every capture, in both directions |
| `virt1..4` at `0x95` | confirmed as a group | all four `0x7FFF`; no virtual sensor is configured, so a populated one stays **unobservable** |
| `0x7FFF` = not connected, in every temperature group | confirmed | 18 of the 28 slots held it in all 90 reports; the 10 that carried a value are `temp6`, `bus2` and `soft1..soft8` (pinned against the fixtures in `tests/test_hw_aquacomputer.py`) |
| fan blocks at `0x167 + 12k`, rpm +0, duty +2, voltage +4 | confirmed, blocks 1-8 | rpm agrees with the fans that turn; duty 2000 on all eight = preset 1's 20.00 %; voltage 12.04–12.10 V |
| rpm `0xFFFF` = no device behind the block | confirmed, and narrower than it reads | it means *no device on aquabus at all*: with the Quadro present, its outputs with no fan read 0 rpm, not `0xFFFF` |
| fan block current +6 and power +8 | **contradicted as a per-report reading**, and the cause corrected on 2026-09-18 | see below, and "The current and power of a fan block are a sample inside the PWM cycle" |
| fan block voltage +4 = *that output's* 12 V rail | confirmed on blocks 1-4, **contradicted on the aquabus blocks 5-8** | block 7 read 12.10 V (the Quadro's rail) in a measuring report and 12.09 V (the aquaero's own) one second later at an unchanged duty, and the outputs with no fan read 0.00 V then 12.09 V; nothing in a single report tells the two apart, so `DeviceKind.reports_rail` is False for 5-8 and the adapter publishes no rail for them |
| the `u16` at `+0x0A` of a fan block = current in mA (item 89's guess) | **contradicted, and still unidentified** | it read 26 while the same block's current field read 6 mA; over the run it tracked current and power but matched neither (26↔6 mA/7 cW, 22↔5/6, 15↔4/4, 11↔3/3, 3↔1/1). The captures suggest a *duty-weighted* reading, but they hold one point that can test it: 26 × 20 % = 5.2 against 6 mA, 13 % low (at 100 % duty the product is the raw value itself and tests nothing — item 114 below). It is decoded raw as `FanStatus.unidentified_raw`, for `tools/aquabus_watch.py` only, and published nowhere |
| flow `flow1..3` at `0xF9`, `flow3` from aquabus | confirmed | all three read 0 with the Quadro present; `flow3` read `0x7FFF` without it (2026-09-15), so `0x7FFF` is the absent-slot sentinel and 0 is "present, nothing connected" |
| control report: feature id `0x0B`, 2707 bytes, no checksum | confirmed | one GET, 2707 bytes |
| active profile = control report byte `0x06`, 0-based | confirmed | byte 0 = profile 1, matching the profile the heartbeat holds |
| per channel: duty at preset `0x55C + 2k` | confirmed | preset 1 = 2000, and every output's status duty is 2000 |
| control source at block `0x20C + 20k` `+0x10`, preset id `0x5C + k` | confirmed for k = 0 only | every block reads source `0x5C` in the owner's profile, so only channel 0 follows *its own* preset; `0x5C + k` for k = 6 stands on the 2026-09-15 write to output 7, and k = 1..5, 7 stay **unobservable** without a write |
| min power `+0x04` = 0, max power `+0x06` = 100 % | confirmed as fields | both read 0 and 10000 on all eight blocks; that a *write* of them takes effect rests on the 2026-09-15 experiment, not on this read |
| mode word `+0x0E`, low byte `0x01` DC / `0x02` PWM | confirmed on outputs 1-4 | all four read `0x0002` and drive PWM |
| the aquabus blocks' mode word means nothing (item 89) | **confirmed, and the earlier `0x0500` was not a constant** | one Quadro's four identical PWM outputs read `0x0000` (block 5) and `0x0002` (blocks 6-8) in the same report, and `0x0500` on blocks 5-7 in the 2026-09-15 capture. The word is whatever the aquaero last stored, not a description of the bus device's output |
| an unconfigured block (source `0xFFFF`, mode 0) | **unobservable in this wiring** | block 8 is configured now (source `0x5C`, mode `0x0002`), so item 89's "write block 8 with a fan on it" cannot be answered read-only |
| byte `0x1A` of the control report (item 92) | **contradicts the obvious guess** | it read `0x00` with the Quadro present on aquabus; item 92 saw it go `0x01` → `0x00` when the Quadro *left* the bus, so it is not "an aquabus device is present" |
| udev / permissions (item 35's last open point) | confirmed | `/dev/hidraw*` are `crw-rw---- root:plugdev` from the shipped rule, the service unit carries `SupplementaryGroups=plugdev`, the probe read both reports as that non-root user, and the heartbeat service writes output report `0x07` as the same user every 5 s (`soft1` held 20.00 °C in all 90 reports) -- readable and writable, proven without this task writing anything |

**The aquabus blocks' electrical fields are not a per-report reading.** Over 90
consecutive reports the aquaero filled blocks 5-8 with the Quadro's own
measurements in 23 of them and with substitutes in the other 67. In a report
that carries measurements the fan on block 7 reads 12.10 V and 5–6 mA / 0.06–0.07 W
and the three outputs with no fan read **0.00 V**; in the rest all four read the
rail voltage (12.09–12.10 V) and block 7 reads **0 mA / 0 W although its fan is
turning at 255 rpm**. The refresh is not atomic either: one report had current 6
mA with power 0, another had blocks 5 and 6 refreshed and block 8 not. So on the
aquaero neither its own outputs (PWM mode, always 0 mA) nor its aquabus outputs
give a current a health rule may judge, and `DeviceKind.reports_power` is False
for all eight.

> **Corrected 2026-09-18, and left standing as the record.** The paragraph below reads
> the 23-in-90 as the aquaero's aquabus poll interval. It is not one: that count is
> the share of reports whose electrical sample fell in the on phase of a 20 % duty,
> and it follows the duty (16 of 16 at 60 %). `AQUABUS_REFRESH_REPORTS`,
> `AQUABUS_REFRESH_S`, `DeviceKind.aquabus_refresh_reports` / `aquabus_refresh_s` and
> `device_health.aquabus.refresh_reports` are **withdrawn**. See "The current and
> power of a fan block are a sample inside the PWM cycle" below.

**That interval is now a number the code carries**: 23 measuring reports in 90
over 88.6 s is a *mean* of one in 3.9 reports, 3.85 s, which
`aquacomputer.AQUABUS_REFRESH_REPORTS` (4) and `AQUABUS_REFRESH_S` (4.0) round to
and `DeviceKind` carries next to `aquabus_outputs_report_power`, as a hardware
fact and not a tunable (§8 item 115). It is a mean and not the longest gap: the
refresh is per block and not atomic, and the maximum gap and the per-block spread
are still unmeasured, so no key and no rule is bounded by this number — it is
reported (`tools/aquabus_watch.py`, `device_health.aquabus.refresh_reports`) and
that is all. Only the electrical fields follow it: the
speed and the output duty of an aquabus block are in **every** report — block 7
read 255 rpm and duty 2000 in all 90 — which is what makes the absence of a bus
device judgeable at all (§8 item 92) and why no rule may key on a voltage or a
current there (§8 item 116).

**The voltage field of an aquabus block is not that output's rail either.** It
is the bus device's rail in the measuring reports and the aquaero's own in the
rest — 12.10 V and 12.09 V one second apart on block 7 at an unchanged duty —
and a single report does not say which. Published as the output's rail it would
be a placeholder standing in for a measurement in about three ticks out of four,
and the rail rule would be *worse* than off: a bus device whose rail really
sagged would be judged 8 V in one report and the aquaero's 12.09 V in the next,
which restarts the rule's timer about every second, so `rail_fault_s` would
never be reached and the dead rail would never be reported. `DeviceKind`
therefore carries `aquabus_outputs_report_rail` (False on the aquaero) next to
`aquabus_outputs_report_power`, `fan_readings` publishes `voltage_v` as `None`
with `rail_reported` false for those outputs, and the rail rule is honestly off
for an aquaero's aquabus outputs (§3 "Fan and device health"): a sagging rail
behind a bus device is **not detected**, and it would take a controller that
reports its own outputs' rails to detect it. The aquaero's own outputs 1–4 are
unaffected and are judged as before. An empty aquabus slot reading 0.00 V in a
measuring report is what the rule's "0.0 V is not judged" clause is for.

**The gap is accepted** (item 117, 2026-09-17). Closing it needs a
controller that reports its own outputs' rails to this daemon — a Quadro on
its own USB — which the supported topology does not have (owner decision
2026-09-16), and the aquaero's own report offers no substitute signal: the
refresh is not even atomic per report (one capture had blocks 5 and 6
refreshed and block 8 not), so no per-report or per-block marker says which
reading a block is carrying. So the daemon states the gap instead of
closing it: every channel's published verdict carries `rail_monitored`,
`power_monitored` and an `unmonitored` mapping of rule → reason, so an empty
`problems` list on an aquabus output can never be read as "the rail is
fine". What stays covered is the aquaero's own outputs 1–4, and with them
any sag common to the whole 12 V supply; what is not is a rail local to the
bus device.

**And the same substitution is why absence is judged on the speed field
alone** (items 90, 116, checked offline against the three captures). Lined
up, they say it plainly. With **nothing on aquabus**
(`aquaero-status.bin`, `aquaero-status-no-aquabus.bin`) blocks 5–8 read rpm
`0xFFFF` and 0.00 V. With the **Quadro present**, the measuring report
`-block7-power.bin` reads its three outputs with no fan at **0.00 V** —
byte for byte what an empty slot reads — and at 0 rpm, while
`-block7-no-power.bin`, one second later at an unchanged duty with nothing
having moved on the bus, reads those same three outputs at 12.09 V. So
0.00 V is neither necessary nor sufficient for absence, and a rule keyed on
it would call a present device absent in one report and present in the next.
The speed field carries no substitute in any of the three: `0xFFFF` when the
bus is empty, 0 rpm for a present output with no fan, 255 rpm for the one
with a fan, identically in the measuring and the non-measuring report.
`FanStatus.present` therefore reads that field and only that field, every
absence judgement in the project goes through it, and **no confirmation
window is added** — the sentinel already outlasts the refresh interval, so
a present bus device is never judged absent, and an absent one is judged
absent on the first report that shows `0xFFFF`. What the captures cannot
say is *when* the aquaero starts showing it, or whether a bus hiccup can
show it for a single report: all three come from a healthy, uninterrupted
bus, and the absent→present transition itself is untested hardware work
(item 96, which needs the Quadro physically off aquabus). So the no-window
decision rests on the sentinel being invariant, not on a measured latency;
a spurious single report would cost a false "no device on aquabus behind
qdN" and a None reading for that tick, never a wrong number.

What this wiring cannot answer, and what it would take:

- whether writing a controller block with **no control source** makes its output
  follow. It needs a block deliberately left unconfigured *and* a fan on that
  output, i.e. a write the owner must authorise; until then the adapter leaves
  that one channel out of its writes and reports it, while every other channel of
  the same controller is written as usual (item 89).
- the preset id `0x5C + k` for k = 1..5 and 7, and that writing min / max power
  takes effect: both need a control-report SET.
- the aquabus sensor→slot map for the Quadro's sensors 1, 3 and 4: it needs
  thermistors on them.
- whether an aquabus fan's rpm lag matters for stall detection (item 75): the lag
  shows only after a duty change, which is a write.
- the Quadro's own status report cross-check: its USB is not connected in the
  supported topology, so `bus2` cannot be compared against the Quadro's own
  reading any more.

### The bus in three questions (items 114, 115, 92; 2026-09-17)

Read-only throughout, and this round **without the board**: the Pi was
unreachable for the whole of this work (no route to its host, retried across the
session), so what follows is what the
captures already in the repository can carry, plus the code and the measurement
tool the next hardware session runs. What still needs the board is named at the
end of each part and in §8.

> **Corrected 2026-09-18, and left standing as the record.** The whole subsection
> below rests on the premise corrected further down, in "The current and power of a
> fan block are a sample inside the PWM cycle": that the 23-in-90 is the aquaero's
> aquabus poll interval. Three things in it are **superseded**, and all three were
> written honestly from a single duty's worth of read-only evidence:
> - *the interval itself*, and with it every symbol the paragraph below points at.
>   `AQUABUS_REFRESH_REPORTS`, `AQUABUS_REFRESH_S`,
>   `DeviceKind.aquabus_refresh_reports` / `aquabus_refresh_s` and
>   `device_health.aquabus.refresh_reports` no longer exist; nothing was bounded by
>   them. Superseded with it: "the refresh is not atomic" — the four blocks
>   alternate together, and the split reading came from a classifier that mixed the
>   two effects.
> - *item 114's `raw × duty` suspicion*, which read the current field as an average
>   over the report. It is one sample, so the two numbers are coordinates of one
>   instant and the arithmetic below has no meaning. The field stays unidentified.
> - *"Until that is done, `aquabus_outputs_report_power` stays False"*, which reads
>   as though finding a poll-rate setting would make it True. It would not: a poll
>   rate cannot make a current sampled inside the PWM cycle judgeable from one
>   report, at any duty.
>
> What is **not** superseded: item 92's rule, which reads the speed field, and the
> question item 115 was really about — at what interval the aquaero polls its
> aquabus *behind* the speed and duty fields, which nothing has measured.

**The refresh interval (item 115).** Measured from the 90-report run: 23 reports
carried the bus device's own electrical values, 67 carried substitutes, over 88.6
s — one refresh in 3.9 reports, 3.85 s, at a report cadence of 0.98 s. The
refresh is *not* atomic (one report had blocks 5 and 6 refreshed and block 8 not,
another had a current with a zero power), so it is a per-block interval, not a
report-level one. The 3.85 s is therefore a mean over 23 refreshes, not the
longest gap between two of them, which this run cannot give. The interval is
recorded where a caller can act on it — `DeviceKind.aquabus_refresh_reports` /
`aquabus_refresh_s`, `device_health.aquabus.refresh_reports`, and the watch tool's
own report — but it bounds **nothing**: item 92's `bus_absent_s` is deliberately
not floored by it, because presence is read from the speed field, which every
report carries, so no skipped poll can look like a departure at any window. What
the run cannot say — and what the next hardware session must
measure with `tools/aquabus_watch.py --reports 600 --raw` — is whether the
interval is *fixed or drifts*, whether it differs per block, and whether it moves
with the report rate. Whether it is a **device setting** cannot be answered
read-only at all: no field of the 2707-byte control report is known to carry an
aquabus poll rate, and finding one means changing that setting in the aquaero's
own menu and diffing a control report captured before against one captured after
— an owner action on the device, not a write from the daemon. Until that is done,
`aquabus_outputs_report_power` stays False.

**The unidentified `u16` at `+0x0A` (item 114).** What it is *not*: not the
current (26 against 6 mA in the same block of the same report), not the power (7
cW there), not a constant, and nothing at all on the aquaero's own blocks 1–4,
where it reads 0 in every capture. It is refreshed with the electrical group — 0
in a report that refreshed nothing — so it is part of what the bus device sends.
What the captures suggest, and how little they support it: the field times the
block's own output duty lands *near* the current field. The captures hold exactly
one point that can test that, since at 100 % duty the product is the raw value
itself (27 against 27 mA) and says nothing about duty — and at that one point it
misses: 26 × 20 % = 5.2 against a measured 6 mA, 13 % low. The 20 % run's further
pairs (22↔5, 15↔4, 11↔3, 3↔1) are all at that same duty, so they constrain a
slope and not a duty relation; under `raw × duty` each of them sits high, by 0.4
to 1.0 mA, every one in the same direction. A straight line fitted to the five
pairs does fit them to ±0.3 mA, but it has a slope of 0.211 and an intercept of
0.55 mA — it is not `field × duty`. So the reading "a current measured over the
output's on-time, of which the current field is the duty average" stays a
**suspicion**: it has one informative point, and that one misses. A second duty
is a duty change, which is a write. So it stays unidentified: decoded raw as
`FanStatus.unidentified_raw` so `tools/aquabus_watch.py` can show it, published
by nothing, judged by nothing. Item 89's other suggestion — look at the aquaero's
own outputs in DC mode, where they do report current — needs a mode change, which
is also a write. The decision the owner has to make is therefore: allow one
duty-and-mode experiment on the bench (and item 114 can be answered), or write
the field off for good.

**The device that leaves the bus (item 92).** On 2026-09-15 the Quadro left
aquabus with the aquaero running and nothing noticed. That is now a health
signal. A device answers on aquabus while at least one of the blocks 5–8 has a
device behind it, or (§8 item 130) the aquabus flow slot carries data
(`aquabus_present`, the speed field and, on the aquaero, the flow field — fields
every report carries); every block reading `0xFFFF` and the flow slot reading
"no data" is an empty bus. The consequences are split by how much they cost:

- **at once, from the first such report**: every logical name bound to one of the
  aquabus temperature slots `bus1..8` reads `None` in the observation. Those
  slots keep serving the last value they read — 24.12 °C an hour after the Quadro
  was unplugged — and nothing in the report marks them old, so a frozen number
  would reach the solver as a measurement. A missing one makes its zone untrusted,
  which holds and then raises: a lost bus device is less evidence, not less heat,
  and nothing about this rule ever lowers a duty.
- **after `bus_absent_s`** (10 s by default; > 0 and floored by nothing else, see
  the refresh interval above): the *report* — one error line, an entry in
  `device_health`'s `problems`, and the `aquabus` block of the device health
  carrying `state`, `present`, `seen`, `absent_s`, `lost` and the temperature
  names gone missing. `state` is the single field a page line or a binary sensor
  can show: `present`, `empty`, `lost`, `never_seen` or `unknown` — and `lost` is
  True only for a device that answered and then stopped, never for the empty bus
  of a plain aquaero, which is `never_seen`. What the window keeps out is a device
  re-enumerating (item 90's transient `0xFFFF`); a skipped aquabus poll never
  could reach it, since presence is read from the speed field.

A bus device with **no fan outputs** (a sensor-only slave) used to be
indistinguishable from an empty bus on the fan blocks alone — its `busN` slots
would have read as missing for ever, a zone with no temperature on a healthy
system (§8 item 130's audit). The aquabus flow slot closes that: confirmed
against the live devices to carry the same two-state "no data" / "present" shape
without needing any fan behind it, so a sensor-only slave, or a Quadro on
aquabus whose fans this daemon does not command, is judged the same as one
whose outputs are bound. The config model still refuses a `busN` binding on a
kind with **neither** witness — no bound aquabus output and no aquabus flow
slot (`aquabus_binding_problem`) — which is dead in the supported topology (the
aquaero always has the flow witness) and stays only for a future controller
kind that adds aquabus temperature slots without one.

What still needs the board: run `tools/aquabus_watch.py --seconds 900`
against the healthy bus and confirm it reports no empty stretch at all (the rule
quiet on a healthy system), and repeat the departure with the daemon running once
the owner is willing to unplug the Quadro again.

### The current and power of a fan block are a sample inside the PWM cycle (2026-09-18)

A measurement on the owner's hardware replaces the cause recorded above for the
CURRENT and POWER fields of an aquabus block. One fan on the Quadro's output 3
(the aquaero's aquabus block 7), its duty driven live, nothing saved:

| commanded duty | rpm | reports with a non-zero current | values |
|---|---|---|---|
| 25 % | 311 | 4 of 14 | 3, 6 mA |
| 60 % | 720–1022 | 16 of 16 | 1–22 mA |
| 100 % | 311 → 1016 | 18 of 20 | 16–44 mA |

The share of reports carrying a number moves with the **duty**, which no fixed
interval can do: 4 of 14 and the earlier 9 of 32 are 29 % and 28 % at a 25 %
duty, and 16 of 16 at 60 % is what no fixed one-in-four poll can produce. The
relation is monotone and *not* a share equal to the duty — 16 of 16 is well above
60 % and 18 of 20 is below 100 %, and three points cannot say more than that. The
current is sampled instantaneously inside the PWM cycle, so at a low duty most
samples land in the off phase and read zero, and at full duty the values become
plausible for a 12 V fan at last — tens of milliamps, where the one-in-four
samples had suggested single digits.

Read-only confirmation on the same board, at the duty the earlier run used
(2026-09-18, 45 consecutive status reports over 44.0 s, the aquaero on firmware
2104, all eight outputs at 20.00 %, the fan on block 7 turning at 251–253 rpm
throughout; no feature report fetched for this run, nothing written, the
heartbeat service left alone):

- **11 of 45** reports carried a non-zero current on block 7 — 24 %, against
  25.6 % in the 90-report run at the same duty, and against a duty of 20 %.
- **The whole electrical group alternates together, across blocks.** Block 7's
  current was non-zero in exactly the 11 reports in which block 5 (an output with
  no fan) read 0.00 V, and in no other: 11 and 34 with nothing off the diagonal.
  So it is one sampling instant for all four blocks of the bus device, whose four
  identical outputs run the same duty in the same phase — not a per-block refresh,
  and the earlier "the refresh is not atomic" reading came from a classifier that
  mixed the two effects.
- **The voltage field goes with it**, which is why nothing changes in what the
  code does with it: in a sampling report block 7 read the Quadro's rail and the
  three outputs with no fan read 0.00 V; in the other 34 all four read the
  aquaero's own. A single report still does not say which, so
  `aquabus_outputs_report_rail` stays False and no rail is published for those
  outputs. Only the *cause* in the text was wrong.
  **And for this field the cause is still open**, which is worth saying plainly:
  an output with no fan draws no current in either phase, so block 5's 0.00 V is
  not itself a phase of a PWM cycle, and "the aquaero does not put the bus
  device's values in every report" — the premise being withdrawn for the current —
  accounts for the voltage as well as in-cycle sampling does. The 20 % run cannot
  separate them, because at that duty the two accounts predict the same share.
  The run that would: **the no-fan block's 0.00 V share at a 60 % duty**, where
  in-cycle sampling has it rise with the duty exactly as the current's share does
  and a relay account has it stay near a quarter. `tools/aquabus_watch.py` already
  prints that share per block; the duty change is a write, so it is the owner's
  call, and it is on item 115's list. Nothing in the code turns on the answer —
  the rail is refused either way — but the record should not claim the mechanism
  it has not measured.
- **The aquaero's own outputs are a different, stronger case**: three fans turning
  at 350, 174 and 375 rpm at a 20 % duty read **0 mA in all 45 reports**. The
  aquaero measures no current on its own outputs at all.

What changed in the code, and why:

- `AQUABUS_REFRESH_REPORTS` (4), `AQUABUS_REFRESH_S` (4.0),
  `DeviceKind.aquabus_refresh_reports` / `aquabus_refresh_s` and the
  `device_health.aquabus.refresh_reports` key are **withdrawn**, not re-rounded.
  The number measured the duty of the run, so there is nothing to round: no
  aquabus poll interval has been measured on a field that is not duty-sampled.
  Nothing was bounded by it — `bus_absent_s` was deliberately not floored by it,
  and that reasoning stands and is now better founded.
- `QUADRO.own_outputs_report_power` becomes **False**. The numbers above are the
  Quadro's own measurement of its own output, relayed by the aquaero, so the
  sampling is the Quadro's: a Quadro on its own USB would report the same field
  the same way. It has not been watched over that path, and the safe direction is
  not to judge. `DeviceKind.reports_power` is now False for every output of both
  kinds, and the two reasons behind that one answer are kept apart in
  `DeviceKind.no_power_reason`: the aquaero measures nothing, a bus device samples.
- Item 114's `u16` at `+0x0A`: the suspicion that it is a current over the
  output's on-time *of which the current field is the duty average* is
  **withdrawn**. It rested on the current field being an average over the report;
  it is one sample, so a pair of the two fields is two coordinates of one instant
  and fitting a duty relation to them means nothing. The 45-report run adds a pair
  no scaling of the current survives at all: one report read 2 mA with the field
  at 0, next to nine that read 6 mA with 26 and one that read 1 mA with 3.
- Item 92's bus-departure rule is **unchanged**: presence is read from the speed
  field, which is in every report (251–253 rpm and duty 2000 in all 45), and takes
  no part in the sampling. What changes is the sentence that justified it.
- `tools/aquabus_watch.py` now measures the right thing: the share of reports
  carrying a sample **against that block's duty**, and whether the blocks are
  sampled together. A run at one duty says nothing on its own; two runs at two
  duties say everything, and the tool prints the duty next to the share so they
  can be compared.

**One good consequence for the fan work.** The same measurement that takes the
current away as a general health signal hands it back for one specific use: at a
high duty it reads in nearly every report (16 of 16 at 60 %, 18 of 20 at 100 %),
and a spin-up kick drives the channel high by definition. So during a kick a
non-zero current *is* evidence that the rotor turns — the only such evidence
this installation has for an output with no tachometer. It is bounded by the same
measurement: it can only ever apply to an output **behind a bus device**, because
the aquaero's own outputs report 0 mA in every report at every duty. That is
written up as a proposed item, not built here.

The earlier text is left standing with a correction marked above it. It was
written honestly from what was then measurable — a single duty, read-only — and
the record should show what it took to find it out.

### The controller's own start boost (2026-09-18)

The daemon's spin-up kick only helps while the daemon runs, and the case that
motivates it — power comes back, the controller holds its saved preset, the fans
do not start — happens before any board is up. The aquaero's firmware has a
per-output start boost (a duty held for a time when an output goes from stopped
to running), so the question is whether this project can read it, publish it, and
possibly write it.

**The answer for now is: it cannot be picked out read-only.** The controller
block is where the rest of an output's settings live, and six of its ten `u16`
are undecoded — but nothing in a read-only capture says which of them is the
boost, or even that the boost is there at all. What the captures give:

- The controller-block array is **twelve blocks**, not eight: `0x20C + 20k` for
  `k` = 0..11, ending at `0x2FB`. Blocks 1–4 are the aquaero's own outputs and
  5–8 the Quadro's four on aquabus — established by writing outputs 1–4 and
  output 7 and watching the fans follow; that 9–12 are four more aquabus outputs
  is an inference from their position, not something a capture shows. What the captures do
  show is that they are controller blocks: 9–12 read, in every capture, byte for
  byte what block 8 read while it was unconfigured in the 2026-09-15 capture:
  `01 C2 07 D0 0D AC 27 10 27 10 00 02 00 02 00 00 FF FF 03 E8`. That is the
  firmware's default for an output nothing is assigned to, and it is the reference
  a changed block can be diffed against.
- Four of the block's ten `u16` are decoded and unchanged: minimum power `+0x04`,
  maximum power `+0x06`, the mode word `+0x0E` and the control source `+0x10`.
  The other six are **not identified**: `+0x00`, `+0x02`, `+0x08`, `+0x0A`,
  `+0x0C`, `+0x12`.

| word | never-configured block | owner's outputs, 2026-09-15 | owner's outputs, 2026-09-17 and 2026-09-18 |
|---|---|---|---|
| `+0x00` | 450 | 500, 500, 500, 500, 450, 300, 500 | 100 on output 1, 450 on 2–8 |
| `+0x02` | 2000 | 1000, 1000, 1600, 1600, 2800, 2000, 1500 | 2000 on all eight |
| `+0x08` | 10000 | 5000 on all seven configured | 10000 on all eight |
| `+0x0A` | 2 | 2 | 2 |
| `+0x0C` | 2 | 2 | 2 |
| `+0x12` | 1000 | 1000 | 1000 |

What that does and does not allow:

- `+0x08` is a per-output field in centi-percent, like the two power limits next
  to it: 100.00 % in every unconfigured block, and 50.00 % on the seven outputs
  the owner had configured before reconfiguring them. That is the shape of a
  start-boost power — and equally the shape of any other per-output percentage the
  aquaero keeps. **Nothing in a read-only capture chooses between them.**
- `+0x0A` and `+0x0C` both read 2 in every block of every capture. A boost
  duration in seconds and a tachometer's pulses per revolution are both plausible
  2s, and neither can be told from the other.
- Two of the six do differ *between outputs*, and they are the ones a boost
  cannot be read out of. `+0x00` reads 100 on output 1 and 450 on the rest today,
  and read 500, 450 and 300 across the seven outputs the owner had configured by
  hand in the 2026-09-15 capture; `+0x02` read 1000, 1600, 2800, 2000 and 1500
  across those same seven. They look like a per-fan rpm pair — a plausible
  start-up or alarm speed, not a duty and a duration — and nothing in any capture
  pairs either of them with a boost setting read off the device's own menu. The
  other four (`+0x08`, `+0x0A`, `+0x0C`, `+0x12`) hold one value across all eight
  outputs, so there is nothing in them to compare at all.

So the honest outcome is the plain one: **the evidence does not support decoding
the start boost, and the code stays honest.**
Nothing here names a boost. What the code does instead is make the next capture
cheap: the six words are read into `ChannelState.undecoded` (raw, no unit, no
name — the same treatment as the fan block's `+0x0A`), printed by
`tools/aquacomputer_probe.py` under each output, published by nothing, judged by
nothing and written by nothing.

**What would settle it**, and only the owner can produce it: set the start boost
to two clearly different values on two outputs in the aquaero's own menu — say
output 1 at 100 % for 5 s and output 2 at 40 % for 1 s — and capture a control
report. Two of the six words will then differ between those two blocks in the way
the two *known* settings differ, and that is the decode. What every capture so far
lacks is exactly that: two of the words do differ between outputs, and no capture
says what the owner set to make them differ. A capture before and after, on one
output, would do as well.

**Writing it is the owner's decision, and no writer is implemented.** Not because
the write is hard — it is the same control-report SET the daemon already does —
but because it would be a write of a field this project cannot name. If the layout
is settled later, this is what making the boost survive a power cycle costs:

- The boost has to be **saved**, not written live. A SET alone is gone at the next
  power cycle (§8 item 84), which is exactly the case the boost exists for. Saving
  means the save report (`DeviceKind.save_report`, feature report 6), which this
  daemon sends **never** (§8 item 86).
- The risk is not the boost: it is everything else in the report. A save persists
  the *whole* control report into the controller's memory, including whatever the
  daemon last wrote — its presets, its control sources, its 0 % / 100 % power
  limits — over the configuration the owner set by hand on the device. An
  administrator on the vendor's forum describes report 6 as the one that "controls
  the device, lock unlock, reset", to be sent only deliberately.
- So if it is ever built: behind an explicit opt-in key that is off by default, as
  a one-shot commissioning action in `tools/aquacomputer_commission.py` rather
  than anything the daemon does on a tick, with the control report captured to a
  file before and after, and only with the owner's own configuration already in
  the report being saved. The daemon's own writes must never be part of a saved
  report.
- And it would only be worth doing at all if a fan measurably fails to start from
  the preset the controller holds. There is a hint that it would: the sweep of
  2026-09-15 found both test fans starting only at 25 % while holding 355–375 rpm
  all the way down to 14 % (§2, "USB spike results"), and on 2026-09-18 three fans
  were turning at a 20 % duty (350, 174 and 375 rpm) — running only because
  something had already spun them up. A controller that comes back holding 20 % is
  exactly the case the boost exists for. But that is a hint, not the observation:
  the observation is one bench test — hold an output at its preset, stop the fan,
  and see whether it starts again — and it needs the owner, because stopping a fan
  by hand is not something a daemon can do.


---

## 3. Architecture: two independent tracks

Hardware **must not** import control code; it may import only the
contract (`model.py`). Control **must not** know USB, sysfs, Digole, HTTP
or MQTT. `tests/test_hw_imports.py` checks the import rule statically.

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
| `stuck_pwm_net` | required | net PWM move that must show up in T; in `(0, 1]`. DAS: a zoned sensor's evidence is its zone's relative airflow instead, `stuck_airflow_net` |
| `stuck_sibling_dT_c` | required | net move of another temperature, °C; > 0. DAS: only the siblings of `stuck_params` (same zone and role; a proximal sensor's own bay) |
| `stuck_pwm_lag_fraction` | `0.25` | fraction of the Stuck window a PWM (legacy) or airflow (DAS) move must be old before it counts as evidence (`stuck_pwm_lag`); in `[0, 0.5]`. The default reproduces the previous hardcoded `stuck_ticks // 4` bit for bit. The lag skips the newest part of the window, so the move is only ever measured over the oldest `1 − fraction` of it: raising the fraction is strictly more conservative (less of the window carries evidence), and a lag above half a window would leave less window to measure over than it skips, which is why the range stops at `0.5` |
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
| `budget_ms` | `250.0` | runtime alarm (`control/loop.py`) and CI/Pi bench gate: `step()` wall time past this logs a warning, ms; > 0 and < `budget_alarm_ms` |
| `budget_alarm_ms` | `350.0` | as above, logs an error instead, ms; > 0 |
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
| `zones.trust_rule` | `strict` | `strict` (sensor groups) \| `sigma` (the estimator's σ, §3 per-zone trust) |
| `zones.fault_coupling` | `declared` | `declared` (a fault reaches the coupled zones' channels) \| `none` |
| `zones.sigma_floor_hold_s` / `sigma_floor_growth_c` / `sigma_floor_release_per_min` | 1800 / 1.0 / 0.0025 | ≥ 0 s / > 0 °C / > 0 PWM per minute; `trust_rule: sigma` only: the soft sigma floor holds at most this long, or until the σ of every lost group has grown this much, then falls at this rate (§3 per-zone trust) |
| `noise.exponent` / `weight_noise` / `band_hysteresis` | 5 / 1.0 / 0.02 | `[3, 7]` / ≥ 0 / ≥ 0 |
| `estimator.k_sigma` | 2.0 | `[0, 4]`; margin `= k_sigma * sigma` |
| `estimator.sigma_fault_c` / `sigma_air_fault_c` | 4.0 / 2.0 | > 0, °C; with `trust_rule: sigma` a zone is untrusted while a constrained bay's drive σ or its air σ is above them; `sigma_fault_c` must then exceed `sigma_uncalibrated_c` |
| `estimator.sigma_uncalibrated_c` | 1.5 | > 0, °C; the σ floor of a bay without an accepted SMART calibration |
| `estimator.air_blind_fault_s` | 900 | ≥ 0 s; with `trust_rule: sigma` a zone is untrusted once its air node has had no trusted `zone_air` reading for this long (the air σ barely grows, §3 per-zone trust and §8 item 70) |
| `estimator.q_t_air` / `q_d_air` / `q_t_drive` / `q_t_sensor` / `q_heat` / `q_offset` | 1e-4 / 4e-7 / 1e-5 / 1e-4 / 4e-7 / 1e-4 | > 0; process noise per tick. `q_offset` is a proximal placement offset's, in both layouts: the offset *state* of a further member with one node per bay, and the offset half of that member's *learned map* with a node per sensor (§3 *A node per proximal sensor*) — a placement drifts either way, and a filter that cannot follow it puts the drift into the shared drive estimate |
| `estimator.p0_t_air` / `p0_d_air` / `p0_t_drive` / `p0_t_sensor` / `p0_heat` | 0.25 / 2.5e-3 / 0.1 / 0.1 / 2.5e-5 | > 0; the initial variance of those same states |
| `estimator.sensor_noise_c` | 0.03 | ≥ 0 |
| `estimator.proximal_offset_c` | 3.0 | ≥ 0, °C; prior σ of the offset between two proximal sensors of one bay (§3 estimator, §8 item 67). It means a different thing in each layout `proximal_slope_spread` selects: with one node per bay it is the prior of the offset *state* each further member carries and 0 fuses them all on one node; with a node per sensor it is the prior of the offset half of that member's *learned map*, where 0 would pin the learned difference at 0 for ever — so `proximal_offset_c: 0` together with `proximal_slope_spread > 0` is a config error, not a third layout |
| `estimator.proximal_slope_spread` | 0.0 | `[0, 0.5]`, dimensionless; prior σ of the *slope* difference between two proximal sensors of one bay. 0 (the default) keeps one sensor node per bay with a constant offset per further member (item 67); above 0 every member gets its own node, its own `tau_s` and its own learned map (§3 *A node per proximal sensor*, §8 item 101) — same state size, same step cost. The learned map is held inside this spread and inside the bounds an accepted calibration has, projected along the combination `Δs·rise + Δb` its rows identify, and its offset half is a random walk with `q_offset` |
| `estimator.proximal_gap_sigmas` | 3.0 | ≥ 0, dimensionless; how many prior σ of `Δs·rise + Δb` the learned gap between two proximal sensors of one bay may reach before the per-member `placement` verdict reports `over`. The box widens with this tick's drive-to-air rise, as the disagreement does. A diagnostic in both layouts and nothing acts on it, because only one of them can tell the two halves apart: `learns: offset+slope` with a node per sensor, `learns: offset` fused (§3 estimator, §8 item 125) |
| `estimator.smart_max_age_s` / `smart_reject_c` | 300 / 8.0 | ≥ `dt` / > 0 |
| `estimator.occupied_dT_c` / `empty_dT_c` / `empty_confirm_s` | 2.0 / 0.7 / 300 | `occupied_dT_c > empty_dT_c > 0`; `empty_confirm_s ≥ 2 * dt` |
| `estimator.occupancy_hold_s` | 30 | `0 ≤ occupancy_hold_s ≤ smart_max_age_s`, s; a blind bay keeps its occupancy this long (0: no debounce). It is also the window in which a blind `empty` bay is unconstrained, so it is bounded by the estimator's other tolerance for absent per-bay evidence |
| `estimator.reset_drive_var` | 25 | > 0, °C²; the drive variance a bay restarts from when a drive (possibly) arrived |
| `estimator.jump_min_c` / `jump_sigmas` | 0.5 / 6.0 | > 0 / > 0; the fast-swap rule's thresholds on a proximal innovation |
| `estimator.bay_settle_s` | 600 | ≥ 0 |
| `estimator.bay_settle_max_s` | 1800 | ≥ `bay_settle_s`, s; the most settling exemption one bay may draw from `trust_rule: sigma` before it has to run this long with neither a window nor a σ over `sigma_fault_c` (§3 per-zone trust, §8 item 69). It is **also** the budget the statistical half of the fast-swap rule spends to reset a bay's thermal block (§8 item 124), so 0 grants neither: no settling exemption, and `mpc.model_reset_on_swap` then acts on occupancy crossings alone |
| `estimator.bay_uncertain_var_c2` | 1.0 | > 0, °C²; the DAS MPC's model checks skip a bay whose drive variance `σ² − σ_cal²` is over this, for `bay_settle_s` afterwards. A *level*, so it covers a bay just swapped and a bay nobody is reading alike; the `sigma` trust rule does not excuse it (§3 *One owner, two exemptions*, §8 item 100) |
| `estimator.bay_cal_step_c` | 0.05 | > 0, °C; the same exemption when a bay's `σ_cal` floor moves by more than this — an accepted or expired map re-maps the drive estimate |
| `estimator.calibration_max_age_days` | 30 | > 0; days, the running daemon's clock, SMART and handheld calibrations alike |
| `estimator.manual_calibration_max_age_days` | 7 | > 0; wall-clock days, how old a **stored** handheld calibration may be when `model.json` is loaded and still come back (item 104). Shorter than `calibration_max_age_days` on purpose: nothing watched the bay during the outage and a handheld map is keyed by bay, not by serial. Past it the stored entry is dropped, not restored at a reduced weight |
| `estimator.calibrate_min_c` / `calibrate_max_c` | 5.0 / 80.0 | °C; `calibrate_min_c < calibrate_max_c`; the range `POST /api/calibrate` accepts for a handheld drive reading (item 23), narrowed at submit time to the gate's `[temp_min_c, temp_max_c]` — a config that narrows the gate below these defaults is valid and never blocks the daemon's start |
| `estimator.associate_window_s` / `associate_min_corr` / `associate_margin` | 3600 / 0.8 / 0.15 | ≥ 600 / `(0, 1)` / `(0, 1)` |
| `estimator.associate_drop_corr` / `associate_drop_checks` | 0.3 / 3 | `0 < associate_drop_corr < associate_min_corr` / a whole number ≥ 1; a correlation pair re-scored below that on this many consecutive evaluations is dropped |

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
| `model_converged_bays_frac` | 1.0 | `(0, 1]`; the fraction of a zone's **occupied bays** that must have informed themselves for the zone to converge, rounded up and never under one. 1.0 (the default) is the all-or-nothing rule term for term. The zone's air block is never substitutable at any fraction, a zone that converged on fewer publishes `partial: true` and `uninformed`, and `model_freeze` never holds such a zone as `frozen` (§8 item 119) |
| `model_partial_k_sigmas` | 1.0 | ≥ 0; the DAS MPC plans each bay its zone converged **without** at `max(k − model_partial_k_sigmas·se(k), k's lower bound)` — the bay's own lower confidence bound, published as `k_used` with the σ it spent as `k_se_used`. Under-estimating `k` makes the solver believe airflow helps that bay less than it does, so it runs the fans higher, which is the safe side. 0 plans the fitted value; inert at `model_converged_bays_frac: 1.0` (§8 item 119) |
| `model_max_pred_err_c` | 1.0 | > 0 |
| `model_use_rpm` | `false` | airflow from the tach for identification, normalised by `fan_models.<m>.rpm_max` (needs `topology`) |
| `model_reset_on_swap` | `true` | a hot-swapped bay's identified `g0`, `k`, `q_s` start over from the prior (§8 item 12; inert in legacy mode and in a `frozen` zone) |
| `model_freeze` | `false` | a converged zone is held frozen and stops adapting online (needs `topology`) |
| `model_split_channels` | `false` | a fan group carries a per-channel split `Es.<z>.<G>.<ch>` (needs `topology`) |
| `fan_curve_online` | `false` | fit each fan model's PWM → RPM curve online (needs `topology`) |
| `fan_curve_settle_s` | 30 | ≥ 0, s: how long a duty must hold before its `(pwm, rpm)` pair is sampled |
| `fan_curve_refit_s` | 600 | > 0, s: shortest interval between two fits per fan model |
| `fan_curve_max_rmse_frac` | 0.05 | `(0, 1]`: a fit with a worse relative RMSE is refused and the curve in force stays |
| `fan_curve_max_age_s` | 3600 | ≥ `fan_curve_refit_s`, s: an accepted fit nothing re-confirms for this long goes stale and its readers fall back to the configured curve |
| `mpc_pred_dt_s` | 30 | > 0; ≥ `dt` with `topology` |
| `mpc_blocks` | `[]` | ints ≥ 1 summing to `horizon`; `[]` = `[1, 1, 2, 4, 6, 6]` cut to `horizon` |
| `mpc_every_ticks` | 1 | int ≥ 1 (DAS example 2) |
| `rho_soft` / `rho_hard` | 40 / 4000 | > 0, `rho_hard ≥ rho_soft` |
| `solver_outer_max` | 4 | int ≥ 1 |
| `model_max_drift_c_per_min` | 0.5 | > 0, °C/min: the drift the drives' own rate does not explain |
| `model_return_factor` | 0.5 | `(0, 1]`: the validity gate's numeric limits, bar the air disturbance's, are scaled by this for the MPC to return from the model fallback |
| `model_return_dwell_s` | 300 | ≥ 0, s: how long the scaled checks must pass continuously (and since the fallback began) |
| `model_drift_rate_tau_s` | 120 | > 0, s: low-pass time constant both the drives' observed rate and the model's own rate go through |
| `model_drift_dwell_s` | 120 | ≥ 0, s: how long the drift or the air-disturbance check must keep failing before the entry faults the model (a leaky dwell: a passing tick does not restart it, a passing spell this long does) |
| `model_max_air_dist_c_per_min` | 8.0 | > 0, °C/min: the zone-air disturbance's move away from its slow level (§8 items 66, 98) |
| `model_air_dist_tau_s` | 900 | > 0, s: time constant of that slow level, and how long it must have run before the check has a reference to report a move against |
| `model_accept_prior` | `false` | the DAS MPC may act on a model that has not converged (needs `topology`) |
| `model_store_interval_s` | 600 | > 0 |
| `model_store_max_age_days` | 30 | > 0 |
| `model_reconfirm_s` | 3600 | ≥ 0 |
| `ident_enabled` | `false` | an explicit start may run an experiment (needs `topology`) |
| `ident_amplitude` | 0.15 | `(0, 0.3]` |
| `ident_levels` | `above` | `above` \| `symmetric` |
| `ident_amplitude_mode` | `fixed` | `fixed` \| `headroom`. `fixed` spends `ident_amplitude` on every channel whatever room it has (what shipped before). `headroom` spends the smallest amplitude in `(0, ident_amplitude]` whose `rel_swing` reaches `ident_pe_aim·√PE_MIN` from where the solver parked that channel, falls back to `ident_amplitude` when even that falls short (so it never excites less than `fixed`), and cuts either level at the rail it would leave — never further than `ident_amplitude` from the base in either direction. `band:<ch>` then refuses the starts whose remaining swing cannot reach the PE floor. Default `fixed`: it pays under `symmetric` and costs a zone under `above` (§3, §8 items 110, 120) |
| `ident_pe_aim` | 2.0 | > 0; how many times `√PE_MIN` of relative airflow swing `ident_amplitude_mode: headroom` sizes a channel's telegraph for. Item 110's measured aim; inert under `fixed` (§8 item 120) |
| `ident_replan` | `true` | the levels follow the live solver demand every tick (§3, §8.4 item 52); `false` freezes them at the base of the start (the Zero W behaviour) |
| `ident_parallel` | `false` | `true`: one experiment drives every channel of the target's own zones at once, each on its own code — the zone-wide excitation the model's PE monitor needs to reach `converged`. Louder while it runs on two of three seeds (−0.007 to +0.025 mean PWM over 16 h since item 112; it was +0.036 to +0.050 while a finished experiment still left its level on the fans), and under `ident_levels: symmetric` the whole zone, not one group of it, may sit `ident_amplitude` under the solver's command on a tick (§3, §8.4 item 102) |
| `ident_require_excitable` | `false` | `true`: `check_start` refuses a start whose channel cannot reach the model's PE bound from where the solver parked it (`not_excitable:<ch>`), the way `band:` and `saturated:` refuse. Off by default: such a run still informs the `E` split and the bays' `g0`/`k`, and a zone-wide start is widened to channels the solver parks high, so refusing gives up the zones that do converge (§3, §8 item 110) |
| `ident_hold_s` | `[60, 120, 180]` | non-empty, positive; each ≥ `5 * dt` when enabled |
| `ident_max_duration_s` | 1800 | `(0, 7200]` |
| `ident_settle_s` | 600 | ≥ 0; ≥ `confirm_s` when enabled |
| `ident_start_band_c` / `ident_max_over_c` | 1.0 / 3.0 | > 0 |
| `ident_abort_below_limit_c` | 1.0 | > 0, °C: the absolute abort below the limit in force on a bay |
| `ident_settle_resume_max_gap_s` | 300 | ≥ 0, s: the longest daemon outage after which `model.json`'s settle timers still count; 0 always starts them over |
| `ident_seed` | 1 | int ≥ 0 |
| `stuck_airflow_net` | 0.15 | `(0, 1]`: net move of a zone's relative airflow (block means over the Stuck window, below) that is Stuck evidence for the zone's sensors |
| `stuck_air_oppose_c` | 0.3 | > 0, °C: a zone-air move against that airflow move by more than this voids it for the zone's `drive_proximal` sensors |
| `stuck_air_oppose_max_c` | 3.0 | > `stuck_air_oppose_c`, °C: an opposing zone-air move larger than this no longer voids the airflow evidence |
| `stuck_zone_air_dT_c` | 1.5 | > 0, °C: a zone-air move larger than this is Stuck evidence for a `drive_proximal` reading of the zone once another of the zone's proximal readings moved with it by more than `stuck_sibling_dT_c` |

`budget_ms` / `budget_alarm_ms` (250 ms / 350 ms default, re-derived on the
Zero 2 W against a measured DAS MPC step p99 of 85.96 ms at `dt = 5 s` in the
DAS example, §8 item 73) gate both the benchmark (`tools/bench_step.py`,
`tests/test_bench_budget.py`) and the runtime alarm (`control/loop.py`, §4.3
"Loop / glue"); both read the config, never a hardcoded literal. They move
together: `budget_ms` must stay strictly below `budget_alarm_ms`, so raising
one to or past the other is a rejected config, not a silent clamp. Both log
lines name the key that was exceeded and the keys an operator can change --
including `mpc_every_ticks` where the DAS MPC is the solver, which is the only
place that key does anything.

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
  ticks, band, decimation, evidence channels with their airflow weights,
  siblings and zone-air witnesses of one sensor);
  `window_ticks` (length of `MpcState.window`: `stuck_ticks` in legacy
  mode, the longest undecimated sensor window, ≥ 3, in DAS mode);
  `bay_class(bay)`, `bay_limit(bay)`, `bay_comfort(bay)`; `blocks()`

Changing `dt` must not silently change the °C/s slew limit. The config is
rejected if `confirm_s > fallback_hold_s`: a real Jump would hit
hold/ramp-high before the new level confirms. Do not “accept the fan
spike” as default.

The rest of `config.yaml` (`config.py`, `AppConfig`): `mpc` is required
and typed; `mqtt`, `host`, `xt6`, `http`, `digole`, `onewire` go to their
owners as plain dicts (each must be a mapping or absent); `aquacomputer`
is the one section shaped as a list (one mapping per controller; a
leftover `hwmon:` section is a `ConfigError` naming the rename); unknown
top-level sections are kept in `AppConfig.extra`, visible but not fatal.
`xt6`, `aquacomputer` and `onewire` are read by Track B (below), `http` by §6,
`mqtt` and `host` by §7. Read from `extra`: `record_path`,
`record_max_bytes`, `record_backup_count` (the tick recorder, §3 Glue)
and `sim.das` (`{topology?, preset?, seed?}`, the DAS simulator for
`--sim-plant das`). `digole`, `onewire.enabled`, `onewire.buses` and
`xt6.prefer` are parsed but no code acts on them (item 57: `onewire.enabled`
and `digole.enabled` are still type-checked -- a non-boolean value is a
config mistake worth naming even where it decides nothing).

Channel names are logical (`xt1`, `qd2`, …; `radiator` in legacy mode).
Mapping onto a controller's `pwmN` lives only in the hardware adapter.

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
   - 3b. DAS: sensor confirmation (below).
   - 3b'. DAS, first tick only: apply a model loaded from the store
     (`control/persist.py`).
   - 3b''. DAS with `fan_curve_online`: the online fan-curve fit
     (`control/fancurve.py`), whose accepted curves go into the store's
     `fan_curves` section and into the model's airflow.
   - 3c. DAS: the estimator (`control/estimator.py`), every tick, fault
     ticks included, on this tick's gate-trusted, confirmed temperatures;
     an estimator error faults the zones with constrained bays (reason
     `solver`) when the solver regulates on estimates.
   - 3d. Zone trust (`control/zones.py`); legacy mode is one implicit zone
     whose verdict is exactly the whole-tick gate verdict. It runs after the
     estimator, whose inputs depend on no verdict, so `trust_rule: sigma`
     reads this tick's σ.
4. Fault bookkeeping per zone (timer, streak).
   - 4b. DAS, `trust_rule: sigma`: the soft sigma floor (§3 per-zone
     trust) on the reach of every solver-driven zone with a lost sensor
     group.
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
- `sigma`: the estimator's uncertainty replaces the zone-air and bay
  groups. For every bay of the zone declared `occupied: true` or `auto`
  that the estimator does not report `empty` this tick, the bay has an
  estimate with `σ ≤ estimator.sigma_fault_c`, the zone's air estimate
  is initialised with `σ_air ≤ estimator.sigma_air_fault_c`, and the zone's
  air node has had a trusted `zone_air` reading within
  `estimator.air_blind_fault_s` (`air_blind_s`); the setpoint
  groups stay required. A lost sensor is not a fault by itself: the
  estimator predicts the unobserved node, its σ grows, the margin `k·σ`
  widens and the fans rise; the zone faults once a σ passes its threshold
  (observability loss), and then holds and ramps high like any zone
  fault. Two things the σ alone gets wrong (items 69 and 70):
  a bay the estimator reports `settling` **and** `observed` carries no σ
  check, because within `bay_settle_s` of a fast-swap jump or an occupancy
  change the filter widens it on purpose — that is the filter *following* a
  swap, not losing sight of it, and the margin carries the widening either
  way; a bay without a trusted proximal member this tick is never exempt, so
  a blind bay still faults at once. The exemption is bounded in wall-clock,
  not by the σ coming back: a bay's σ falls back within a tick or two of every
  jump, so one bay's windows may suspend the check for at most
  `bay_settle_max_s` in total, until the bay has run that long with neither a
  window nor a σ over `sigma_fault_c`. A swap spends one window or two; a
  sensor that keeps jumping spends the budget and then faults its zone on every
  tick it is over, as it did before item 69. This exemption is *not* the DAS MPC
  validity gate's: that one reads `model_exempt` and also covers a bay whose σ is
  simply too wide to score a model against — which is the observability loss the
  rule here exists to catch, so the two lists differ on purpose (§3 *One owner,
  two exemptions*, §8 item 100). And `sigma_air_fault_c` cannot decide at
  all, because the air node is observed by more than the zone-air sensors:
  every proximal sensor reads `(1 − s)·T_a` beside its drive, and the inlet
  and the fan command pin the rest. `air_blind_s` against
  `air_blind_fault_s` (reason `sigma:zone_air_blind`) is the decidable form
  of the same question. **Soft sigma floor** (`zones.advance_sigma_floor`): the estimator
  cannot show the heat a lost sensor would have shown and `k·σ` grows slowly,
  so without a floor the DAS MPC, which had followed the measured warming,
  lowers the fans within the first minutes of the loss (PWM −0.07 below the
  level before it, 19 % less zone airflow than the run with the sensor, closed
  loop) and over 65 minutes costs the bay 2.3–2.4 °C of true margin. (The drop
  used to be −0.04 on the tick of the loss itself; §8 item 19's occupancy
  debounce keeps the blind bay's state for `estimator.occupancy_hold_s`, so the
  estimate changes a few ticks later. The worst drop and what the floor costs
  are unchanged.) When a zone-air or bay
  group of a zone (a `strict` group the estimator replaces) has no
  gate-trusted, confirmed member, an episode opens for the zone: every channel
  of its reach keeps a floor at `prev` of that tick (the command before the
  loss; a further group lost during an episode reopens it at the higher of
  `prev` and the floor in force). The floor **holds** until
  `zones.sigma_floor_hold_s` (1800 s, counted in ticks of `dt`) has passed or
  the σ of every lost group (the bay's drive σ, the zone's air σ) has grown by
  `zones.sigma_floor_growth_c` (1.0 °C, so `k_sigma` × 1.0 = 2 °C of extra
  margin now carries the uncertainty), whichever comes first; it is then
  **released**, lowered by `zones.sigma_floor_release_per_min` (0.0025 PWM per
  minute) until it reaches `pwm_min` and is gone. A floor therefore ends at
  most `sigma_floor_hold_s + 60 · (pwm_max − pwm_min) / sigma_floor_release_per_min`
  seconds after its episode opened. The episode closes when the groups are held
  again; it keeps advancing while its zone is in fault, but only an eligible
  zone's floor applies, and a tick without an estimator update keeps it as it
  is. The solver's demand is raised to the floor
  (`diagnostics["sigma_floor_channels"]`, per zone `diagnostics["sigma_floor"]`:
  lost groups, phase, time held, σ growth, floor). The floor is a level, not
  `prev` of every tick: the hard floor it replaces (at or above `prev` on every
  tick while the sensor was lost) turned the DAS MPC's swings into a ratchet,
  32.5 dB against 27.1 dB without a floor until the zone faulted. Measured on
  `sim/das.py` (example config, busy bays, b02's only proximal sensor lost at
  10 minutes for 65 minutes, DAS MPC, `basic` seeds 1–5): the σ growth ends the
  hold 14–15 minutes after the loss (13–18 over every nightly case); no drive over its limit; the true margin
  lost against the run with the sensor stays within `k_sigma` × the bay's σ
  growth (0.00 °C beyond it; 0.06 °C beyond it without a floor, in the first
  minutes) and at most 1.03 °C (0.48 °C until the zone's σ fault) against
  2.33–2.39 °C without a floor; the zone's command falls rather than rises
  before that fault; mean noise until the fault 27.0–27.5 dB against
  26.0–26.6 dB without a floor. With b13's sensor lost (DAS MPC) the soft floor
  loses up to 0.33 °C (0.08 °C without a floor) at +0.1 dB; the PI-like DAS
  never asks for less, so the floor changes nothing; on the saturated `rich`
  runs no margin is lost and the noise is within
  0.1 dB of no floor, or lower. The floor only delays the loss: with the
  sensor lost until the floor has released (DAS MPC, b02 and b13 on `basic`
  seeds 1–5, b02 on `rich` 0–2, 365 minutes of loss; the floor released
  184–206 minutes after the loss, 337 on `rich`) the true margin lost
  converges to that without a floor
  (2.40–2.43 °C on b02 against 2.40–2.42 °C, up to 0.88 °C on b13 against
  0.75 °C), still 0.00 °C beyond `k_sigma` × the σ growth, with no drive over
  its limit; the zone faults on σ and returns in between (250–360 fault ticks
  of 4380 on `basic`). Bounds asserted
  nightly (`tests/test_sigma_trust.py`): on every tick 0.05 °C beyond `k·σ`
  growth; 1.25 °C in total over the 65 minutes, and until the floor has
  released at most the larger of 1.25 °C and the loss without a floor, plus
  0.05 °C; 1.5 dB above no floor; no command rise beyond the run without a
  floor plus 0.05.
  The verdict reads this tick's σ (step 3d runs after the
  estimator); a zone in fault returns after `confirm_ticks` with σ within
  its thresholds, without waiting for confirmed members of the air and bay
  groups (a confirming sensor is not fused, so σ already carries it). On a
  tick with an estimator fault `strict` applies; `diagnostics["trust_rule"]`
  is the rule that ran. Switching rules is config only;
  `sigma_fault_c` must exceed the uncalibrated floor `sigma_uncalibrated_c`
  (1.5 °C) under `sigma`.
  Measured on `sim/das.py` (example config, `basic` physics with sensor
  noise, busy bays): 2 % per-tick dropouts on every DS18B20 fault no zone
  in 75 minutes under `sigma` against about 200 zone-fault episodes under
  `strict`, with both DAS solvers and no drive over its limit; the same
  holds on `rich` with the example's two redundant proximal pairs present
  (seeds 0–5, item 67). A bay's only
  proximal sensor lost for good passes 4 °C about 44 minutes later
  (PI-like DAS; the fans rose from 0.60 to 0.82 meanwhile); every sensor
  of a zone lost faults it on the blind air clock 15 minutes later, before
  the drives' σ would at about 17. The air σ itself reaches 0.25 °C
  65 minutes into that loss while the true air error grows 0.13 °C, and with
  only the zone-air sensor lost it reaches 0.07 °C while every drive σ stays
  at the uncalibrated floor — so before item 70 nothing ever faulted a zone
  that had lost its air sensors alone, though `strict` faults it at once. A
  hot swap puts the bay's σ above 4 °C for a tick (the fast-swap rule on
  removal, the `reset_drive_var` 25 °C² insert variance) with the bay still
  observed, so the zone no longer faults for it (item 69); the insert still
  raises the zone's fans (+0.31 PI-like DAS, +0.64 DAS MPC, against +0.22
  and +0.65 with the fault). Lose that bay's sensor right after the insert
  and the widened σ faults the zone on the first blind tick.

**Sensor confirmation** (DAS, `zones.advance_confirmation`). A sensor whose
present value the gate rejects (`range`, `slew`, `stuck`: a Jump, a Spike,
a frozen reading leaving its band) is *confirming* until it has been
gate-trusted on `confirm_ticks` consecutive time-valid ticks, the same
count a zone needs to leave a fault. A dropout (`missing`, `null`,
`non_finite`) starts nothing, since the value that returns is gated against
the last good one -- unless there is no last good one to gate it against: a
name whose slew check passed with neither `last_good_obs` nor a previous raw
value to compare against, while `last_good_obs` itself already exists (the
run is past its own cold start), is `no_reference` (a sensor missing since
boot, item 61); such a first reading has no evidence behind it either, so it
starts confirming too, on the tick it first appears. The run's genuine first
tick (`last_good_obs is None`) is exempt and stays bumpless. A dropout or a
time fault while confirming restarts the count. A confirming sensor is not
fused by the estimator, not in the
solver's `temps`, not written into `last_good_obs` and not seen by the
thermal identification. For zone trust it counts as a trusted group member
only while its zone is already in fault, so a sole member costs
`confirm_ticks` once (the zone's own confirmation runs beside it); a
fault-free zone with a group whose only trusted members are confirming
faults, and a zone in fault returns only when every required group also has
a confirmed trusted member (under `sigma`: every setpoint group). A Jump on a redundant member (a second proximal
sensor on a bay, a second zone-air sensor) or on a sensor outside every
group (an inlet, an exhaust) therefore stays out of the estimator until it
confirms, its group stays trusted through the other members, and the zone
does not fault. The counts are `solver_memory["sensor_confirm"]` (sensor →
consecutive trusted ticks, confirming sensors only) and
`diagnostics["sensor_confirm"]`; legacy mode has neither.

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
     sample `max(0, min(floor(stuck_ticks * stuck_pwm_lag_fraction),
     stuck_ticks - 2))` ticks **before** the newest (`stuck_pwm_lag`;
     `stuck_pwm_lag_fraction` default `0.25`, a quarter window, in
     `[0, 0.5]`), **or**
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
  update. DAS: a Jump on a sensor whose zone keeps trusting its group
  (a redundant member, an inlet) confirms per sensor over the same
  `confirm_ticks` before the estimator fuses it (sensor confirmation,
  above).
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
- the PWM evidence is the zone's **relative airflow**, not one channel:
  `Qn = Σ w · φ(u)` over the channels of the sensor's zone, with
  `φ(u) = clip((u − deadband) / (1 − deadband), 0, 1) ^ exponent` from
  `fan_models` — always the configured entry, never the online fit
  (item 107): this is a sensor-validity rule, it is derived from the config
  once at load into `StuckParams.airflow`, and it needs only the *sign and
  size of a move* in relative airflow, which a refitted dead band would
  perturb for no gain — and `w` the channel's
  `fans.<ch>.count` split evenly over the zones that list it, normalised
  to a sum of 1. It counts when the mean `Qn` of the `B` samples that end
  at the lagged sample (`L = stuck_pwm_lag`, `stuck_pwm_lag_fraction`
  of the window, default a quarter, before the newest) differs from the
  mean of the first `B` samples by more than
  `stuck_airflow_net` (`stuck_pwm_net` stays the legacy per-channel rule).
  The block is `B = max(1, min(L, (m − L) / 2))` of the window's `m`
  samples, so the two blocks never overlap or coincide however wide the
  lag is (`B = L` at the default and at every fraction up to `1/3`).
  Block means, not the oldest sample: a drive does not answer a fan dip
  of a tick or two. An inlet without a zone has none: the fans do not move
  the inlet;
- for a `drive_proximal` sensor that airflow move is **no evidence** when
  every zone-air sensor of its zone with a plausible path over the window
  moved against it by more than `stuck_air_oppose_c` **and by at most
  `stuck_air_oppose_max_c`** (warmer air after more airflow, cooler after
  less): a proximal reading mixes the zone air with the drive-to-air
  difference, which airflow moves the other way. The upper bound is the
  cancellation's own limit — an airflow move shifts the drive-to-air
  difference by a few °C at most, so a larger air swing must reach the
  reading whatever the fans did, and without the bound an arbitrarily large
  air move kept a dead sensor trusted for as long as it lasted (§8 item 59).
  A zone-air sensor without a plausible path (dropout, Spike) does not vote;
  one that did not move (a frozen one included) keeps the evidence;
- for a `drive_proximal` sensor whose zone airflow was computable but gave
  no evidence — it stayed **inside** `stuck_airflow_net` (fans pinned at
  `pwm_max`, a slow trim: nothing to measure), or it moved and the zone air
  excused it by the rule above — the evidence is instead a zone-air sensor
  of its zone whose plausible net move over the window exceeds
  `stuck_zone_air_dT_c` **while another `drive_proximal` reading of the same
  zone, of any bay, moved plausibly by more than `stuck_sibling_dT_c` over
  that window**. At constant airflow the drive-to-air difference moves only
  with the bay's own power, so an air move that much larger than the band
  the reading sits in had to reach a healthy sensor; one zone-air sensor is
  enough, and the direction does not matter (§8 item 58). The second half is
  what makes the air move *real*: without it the rule cannot tell "the air
  moved and this one reading did not follow" from "this zone-air sensor is
  drifting and every reading of the zone is correctly still", and a single
  lying air sensor would brand every proximal reading of its zone Stuck and
  fault the zone (§4.4). The other bay's reading is the corroboration and not
  evidence on its own: a neighbour's drive heat never reaches this sensor
  (the sibling rule below), the zone air they share does. Because this check
  also runs after an excused airflow move, the excuse is effective only up to
  whichever of `stuck_air_oppose_max_c` / `stuck_zone_air_dT_c` is met first:
  a zone whose fans happen to move must not hide a dead sensor that a still
  zone catches;
- the sibling evidence counts only other sensors of the same zone **and**
  role, and for a `drive_proximal` sensor only those of its own **bay**:
  another bay's reading follows that bay's drive heat, which does not
  reach this sensor.

Why (§8 item 3). With every zone channel's own net PWM move and every
proximal sensor of the zone as evidence, the `rich` simulator with the
PI-like DAS solver flagged a healthy sensor in 37 and faulted a healthy
zone in 34 of 260 75-minute runs (seeds 0–259). An idle bay's DS18B20
sat inside its band for half an hour while a neighbouring bay's activity
burst moved that bay's reading by 1–2 °C (31 of the 37), or while the
zone's fans moved and the reading had a physical reason to stay: a warming
inlet against more airflow, the solver moving a zone's channels apart (one
up, a shared one down), a shared single-fan output carrying a small share
of the zone's air, or, with the DAS MPC, a short fan dip caught as the
oldest sample. With the rules above no healthy sensor was flagged in 1,040
PI-like DAS runs (seeds 0–1039) and 320 DAS MPC runs (seeds 0–319) of
2.5 hours each; the smallest zone-air opposition that voided a move there
was 0.61 °C, and an airflow threshold of 0.1 brought three false flags
back where 0.125 did not. Rejected: the estimator's predicted change for
the bay (its proximal channel is closed on the very sensor under test, an
open-loop prediction over the window integrates the zone model every tick
on the Zero W, and the prior sensor map, `β = 0.3` against 0.15–0.5 on the
simulator, mispredicts exactly these cancellations); weighing other bays'
siblings by role (the only part of another bay's reading this sensor
shares is the zone air, which the zone-air sensor measures directly).

**Detection time.** A reading frozen from `t0` is flagged at the latest
at `max(t0 + stuck_s, t1 + stuck_s / 2)` plus two decimation intervals
(`2 · stuck_decimate · dt`) once its zone's relative airflow has stepped
by more than `stuck_airflow_net` at `t1 ≥ t0 + stuck_s * stuck_pwm_lag_fraction`
and stayed there, and not before the step is `stuck_pwm_lag_fraction` of a
window old (default a quarter); a step less than that fraction after the
freeze counts only by the share of the lag block that precedes it
(`tests/test_gate.py`). A same-bay sibling
that moves plausibly by more than `stuck_sibling_dT_c` flags it as well.
A reading frozen while the airflow stays inside `stuck_airflow_net` is
flagged once a zone-air sensor of its zone has moved past
`stuck_zone_air_dT_c` within one window and another bay's proximal reading
of that zone has followed it (§8 item 58), and one whose airflow move was
opposed is flagged as soon as that opposition passes
`stuck_air_oppose_max_c`, or passes `stuck_zone_air_dT_c` with the same
corroboration (§8 item 59).

**Coverage on the simulator** (§8 items 3, 58). Each of the example
config's 17 proximal readings frozen 20 minutes into a 2.5-hour `rich`
PI-like DAS run, seeds 0–2 (51 runs, `tests/test_stuck_sim.py`): 17 of 51
flagged with the airflow move as the only zoned evidence, **28 of 51**
with the zone-air rule as well (per seed 6 → 17, 7 → 7, 4 → 4). The gain
is seed 0's, whose drawn inlet drifts far enough that every zone's air
crosses `stuck_zone_air_dT_c` inside a window; seed 1's zone air does move
but stays **under** 1.5 °C within a window, so the rule misses it on margin
rather than on flatness, and seed 2's gains nothing. Corroboration costs
nothing here: the same 28 are flagged with and without it, because on a
real air move every other bay of the zone follows. The global rule before
item 3 flagged all 51, at the price of the false zone faults above. False
flags: **none**, in those 51 runs (no sensor but the frozen one) and in 256
healthy 2.5-hour PI-like runs (seeds 0–255) and 120 DAS MPC runs (seeds
0–119); a zone-air sensor drifting 0.3 °C/min for 45 minutes on an
otherwise healthy enclosure flags no proximal reading and faults no zone
(`tests/test_stuck_sim.py`). The defaults come from the same runs: on a
healthy reading sitting inside its band the largest zone-air move at
steady airflow was 1.20 °C, so `stuck_zone_air_dT_c` is 1.5; the largest
opposing move that voided an airflow move was 1.19 °C, so
`stuck_air_oppose_max_c` is 3.0. Lowering `stuck_zone_air_dT_c` to 1.25
would flag 37 of the 51 (seed 1 alone goes 7 → 16: all of its windows sit
in that 1.25–1.5 °C band) but leaves no margin over that 1.20 °C.
Rejected as evidence: SMART diverging from the proximal reading — the
serial-to-bay map is identified online by the estimator, so the gate's
verdict would depend on an identification that has its own confirmation
rules, and the simulator's SMART view is not exercised by these runs.

Long windows are **decimated**: a sensor whose window has `n` ticks keeps
one sample every `stuck_decimate` ticks (default `max(1, n // 60)`, so
1800 s at `dt = 5` keeps 60 samples, one every 6 ticks) in
`solver_memory["stuck_slow"]`, fed from the previous tick's dense sample
(which carries the applied command). The check interpolates nothing; a
sibling's or zone-air sensor's single step is bounded by
`k × dT_max_tick`. A reading's excursion shorter than `k` ticks can go
unseen by the band check, which can only flag Stuck more readily (its
zone faults, cooling rises), never less. A malformed stored
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
`rpm` input of the channel's `xt6.fans` or `aquacomputer[].fans` entry
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
`unknown_keys`, `stuck`, `stuck_latch`; DAS: when `name` is confirming
(`sensor_confirm`) and the gate's own raw check passed this tick,
`per_temp[name]` is `false`, `reasons[name]` is `["confirming"]` and the
`gate.trusted` summary is recomputed over that `per_temp`, so this outward
copy never claims a value is trusted when the estimator and the solver do not
use it and no key of it contradicts another -- item 63; a name the gate itself
rejected keeps the gate's own reason, and the internal verdict `zones.py`
reads is unaffected. The
top-level `trusted` is a different question -- whether the gate itself
accepted this tick -- and stays the gate's own verdict), `time` (`status`, `dt_obs`,
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
(status, error, per zone air estimate, SMART counters and the separate
manual-calibration counters of item 23, and per zone `airflow_curve`:
which fan curve produced its airflow); `noise`
(`db_index` from the fans' speed, `db_index_cmd` at the new command, per
channel rpm and its source, the `u0` behind the index and the `curve` it
came from); `fan_curves` (with `fan_curve_online`: `readers` — which
reader follows the fit and which stays on the configured curve, item 107 —
and per fan model the curve in force, its `source`, `stale` and the ages
of the fit and of its newest sample); `thermal` (with `model_shadow`: status,
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
`[T_a, d_a, T_d…, T_s…, q…, c…]` for every bay of the zone (the layout never
changes with occupancy):

```text
C_a dT_a/dt   = Σ_j g_j (T_d,j − T_a) − (Q_z + leak)(T_a − T_in) + Σ_z' κ (T_a,z' − T_a) + C_a d_a
C_d dT_d/dt   = C_d q_j − g_j (T_d,j − T_a)                g_j = g0 + k·Qn_z
τ_s dT_s/dt   = s_j T_d,j + (1 − s_j) T_a + b_j − T_s,j    s_j = 1 − β_j
dd_a/dt = 0,  dq_j/dt = 0                                  (integrating disturbances)
dc_i/dt = 0                                                (placement offsets, random walk)
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
`φ = clip((u − deadband)/(1 − deadband), 0, 1)^exponent` at `u = prev`,
from the **curve in force** — the online fit's `deadband`/`exponent` where
there is one, else `fan_models` (item 107; the estimator and the thermal
model must plan on the same air);
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
sensor variance on a proximal innovation above `jump_min_c` (0.5 °C) and
`jump_sigmas` (6) σ, so a pulled-and-replaced drive widens its margin at once.
The innovations of a bay's proximal members are taken against the sensor node
before any of them updates it, and the members have to agree: the rule fires
only when at least one passes the test and no other member's innovation is
past `jump_min_c` the other way. One bay holds one drive, so a swap moves
every sensor of the bay the same way; two redundant sensors at different
placements disagree in opposite directions for as long as they sit there, and
that standing disagreement used to re-arm the rule every tick and lock the
bay's SMART out of the calibration for good (§8 item 17).

**Placement offsets** (item 67). A bay's sensor node is anchored on its first
proximal member that is not `redundant`; every further member reads that node
**plus its own offset** `c_i` (measurement row `H = e_Ts + e_c`), a random walk
(`estimator.q_offset`) with the prior `N(0, estimator.proximal_offset_c²)`.
Two sensors on one bay sit at different placements: they see different fractions
of the drive (`β`) and carry different offsets, so on the `rich` simulator they
read 3–5 °C apart under load. Fused on one node that disagreement stayed in the
innovations, where the fast-swap rule read it as a swap on every tick and the
bay's σ sat at 4–7 °C (up to 10). The offset is what the disagreement is, and
the jump test now judges each reading against its own innovation variance, which
for a member with an offset carries that offset's uncertainty: an unconverged
placement cannot look like a swap, while a swap — which moves the drive and so
every member together — still fires the rule. The offset keeps its variance
through the inflation: the drive moved, the placement did not. The **bay-level**
statistic that publishes `swapped` (§8 item 12) is the mean of those same
innovations, so it carries the offsets the same way: the prediction of the mean
reading is the node plus the mean of the members' offsets, and its variance
carries the offsets' (§8 item 109). Against the bare node it would hold half the
pair's disagreement — a gap proportional to the drive-to-air rise — and none of
its uncertainty.

**One event, one rate limit, one record** (§8 items 123, 124). The two halves of
the rule agree on what an event is: a bay-level step now draws the per-sensor
rule's own consequences — the drive variance widened by the very statistic that
said so, the settling window opened, that tick's SMART held back, the correlation
pair dropped, the placement row of item 101 not absorbed — so a block the model
throws away is never scored against a drive estimate the filter still calls
confident. And what reaches `thermal.update` is `swap_reset`, the verdict at one
event per `estimator.bay_settle_s` while the bay has `estimator.bay_settle_max_s`
of settling budget left: **the same budget, with the same clean-run recovery,
that the `sigma` trust exemption spends**. Exhausting it keeps the (possibly
stale) fit rather than throwing it away again — resetting without a limit is what
left a zone at the prior for a whole run before item 109, while a stale fit is
scored by the MPC's own prediction-error gate, and a bay out of budget is no
longer exempt from that gate. An occupancy crossing is *not* rate-limited: it is
a debounced fact (item 19), not a statistic, and refusing one would model an
arriving drive with the coefficients of the drive that left. Every bay publishes
`swap_reset`, `swap_count`, `swap_last_s`, `swap_reason` and `swap_held`
(verdicts the budget refused), and the thermal block carries `resets` /
`last_reset_s` **across** the reset — the one thing a reset does not throw away.
A bay with one
proximal sensor has no offset state at all, so a config without redundant
proximal pairs keeps the arithmetic it had; `proximal_offset_c: 0` fuses every
member on one node, as before the key existed. Measured on `sim/das.py`
(`rich`, example config, 75 minutes, both DAS solvers, seeds 0–5, redundant
pairs present): b03 and b10 keep σ at the uncalibrated floor 1.50 °C (max 1.53)
and no zone faults, against σ medians up to 5.8 °C and 657–912 zone-fault ticks
of 900 on seeds 0–3 before.

**A node per proximal sensor** (item 101, `estimator.proximal_slope_spread`,
default 0 = off). The offset above is a *constant*, and two placements do not
differ by a constant: they see different fractions of the drive, so their
disagreement is `Δs·(T̂_d − T̂_a) + Δb` and moves with the drive-to-air rise. With
`proximal_slope_spread > 0` every proximal member of a bay is **its own node**
`T_s,i` with its own lag `sensors.<name>.tau_s` and its own map `s_i = s + Δs_i`,
`b_i = b + Δb_i` (clipped to the same bounds an accepted calibration is), and its
measurement is a plain scalar update on that node. The offset block of item 67
becomes the extra sensor nodes, so **the state is exactly the same size** and the
measurement is cheaper (a scalar update instead of the rank-two one). `(Δs_i,
Δb_i)` is a two-parameter RLS, prior `(0, 0)` at
`diag(proximal_slope_spread², proximal_offset_c²)`, row `z_i − z_anchor` against
the filter's own rise with variance `R_i + R_anchor + CAL_ROW_VAR`, skipped on a
jump tick. **No SMART is needed**: both members see the same drive and the same
air, so their difference identifies the placement even where the absolute map
cannot be learnt. The member's own innovation variance carries `h P hᵀ` of that
map, so an unconverged placement still cannot look like a swap. The bay's own
node stays the anchor's: `t_sensor`, the occupancy ΔT, the association series and
the SMART calibration all read it, so the model store's `cal` schema is unchanged.

**Which layout learns what, and what follows** (§8 item 125). The per-sensor
layout fits both halves of `Δs·rise + Δb`; the fused layout has **one** state for
the pair, and that state is the whole disagreement *at the current rise*, so it
absorbs `Δs·rise` as drift and has no slope to separate it from. The fused layout
therefore cannot learn the *shape* of a placement difference, and nothing will
make it: giving the pair a slope **is** the per-sensor layout, at the same state
size and a cheaper measurement. What both layouts can carry is the number itself,
so every bay with a redundant pair publishes a `placement` verdict per further
member — `gap_c`, `bound_c` (the box the config's own priors allow at this tick's
rise, `estimator.proximal_gap_sigmas·√((Δs spread·rise)² + proximal_offset_c²)`,
so it widens with the load as the disagreement does), `over`, `evidence`, and
`layout` with `learns` (`offset` | `offset+slope`). On the per-sensor layout an
`over` is a placement the config does not allow — a sensor coming loose, fouling
or ageing; on the fused one it may equally be a load that layout cannot resolve,
and the verdict says which case it is instead of letting a reader take the two
for the same evidence. Nothing acts on it: it is a diagnostic, not a gate.

Two things keep that RLS honest, and without either it is *worse* than the offset
state it replaces. **`Δb` is a random walk** (`estimator.q_offset`, the same key
and the same per-tick variance the fused layout gives its offset state): a
placement drifts — fouling, a sensor working loose, a thermistor ageing — and a
plain RLS has no forgetting, so `P` collapses, the learned map freezes and the
disagreement the member keeps bringing has nowhere left to go but the drive both
members share. Measured on the two-plateau fixture with the redundant member
sliding 6 °C away over 900 ticks: with the random walk the drive estimate lands
within 0.4 °C of truth on both layouts; without it the per-sensor layout is
1.4–2.4 °C **under** truth with σ still at the uncalibrated floor 1.50 °C, i.e. a
failure that reduces cooling. `Δs` gets none: a placement's geometry does not
drift. **`(Δs, Δb)` is bounded**, because a row identifies only the *combination*
`Δs·rise + Δb` — the pair may walk along `(1, −rise)` for ever and still fit every
reading it has ever taken, and under regulation the rise barely moves, so that is
what it does (measured unbounded on the same fixture: `Δs` +2.5 against a prior
spread of 0.25, `Δb` −51 °C). The bound is `proximal_slope_spread` on `Δs` and,
on both, whatever keeps `s_i`, `b_i` inside the bounds an accepted calibration has;
a pair that leaves the box is slid back **along that same null direction**, so the
member keeps predicting what the evidence says it reads and only the *split*
between slope and offset moves. Truncating the two coordinates one at a time
instead throws the combination away with them, and the clipped map then bears no
relation to either the prior or the evidence.

What it buys, measured. While both members report, the fused offset is *measured*
every tick and follows a slope nearly as well as a map does (after a change of
rise: 0.08 °C out against 0.02 for the per-sensor layout on `tests/test_estimator.py`'s
two-plateau fixture). What a constant offset cannot do is **predict**: a member that
goes quiet leaves its offset at the rise it last saw, so when the load moves under it
and it comes back, the reading is a jump — σ 4.67 °C, `settling: jump`, against σ at
the uncalibrated floor 1.50 °C and no jump with a node of its own. That is item 67's
leftover, and the reason the pairs still cost the `sigma` trust rule an exemption.
On the closed loop the difference is not visible. Measured over the whole 4500 s
`rich` run (`config.example-das.yaml`, declared serials, calibrated MPC, the first
300 s discarded), seeds 0, 1, 2, 3, 5 and 7: every bay's drive-estimate rms agrees
between the two layouts to 0.01 °C on five of the six seeds, and the exception is
seed 0, where b10 (a redundant pair) goes 3.40 → 3.56 °C and b09 (a single-sensor
bay in the same zone, reached through the shared air) 3.62 → 3.90 °C. The worst
under-estimate over those seeds is unchanged to 0.02 °C. **The horizon matters**:
over 900 ticks the two layouts look identical to 0.01 °C everywhere, and an
unbounded `(Δs, Δb)` needs longer than that to leave its box — the 47 % rms
regression the unbounded version showed on seed 7 is what the bound above removes.
The reason accuracy does not move either way is identifiability, not the
representation: the drive-to-air rise barely moves under regulation, so `Δs` and
`Δb` are collinear and the RLS recovers about half the true slope on one seed and
the wrong sign on another — the *combination* `Δs·rise + Δb` is right either way,
which is why the estimate does not move, and why the bound projects along it. Step
cost, development machine, 400 ticks, `tools/bench_step.py --sim-plant das`, two
repeats, with the bound and the random walk above in: DAS MPC step p99 3.08/3.26 ms
fused against 3.45/3.10 per sensor, solve p99 3.19/3.31 against 3.48/3.13 — the same
within run-to-run noise, as the equal state size predicts; PI-like DAS p99 1.12/1.10
against 1.60/1.51, a real but small cost (the extra nodes and the RLS row, ~0.4 ms)
in the path that has nothing else to do. Both are two orders of magnitude inside the
250 ms budget. **The default is 0**
because turning it on for the example's two redundant pairs moves the four DAS
goldens, which is item 99's decision.

**Per-bay initialisation** (item 69). A bay with no trusted proximal member when
its zone starts is *not* initialised: its node holds the prior until the first
trusted reading arrives, which then seeds `T_s`, `T_d`, `q` and the bay's offsets
exactly as the zone start would have. Without that the returning reading met a
node sitting at the zone air and tripped the fast-swap rule (a zone fault on
`rich` seed 5 with 2 % dropouts). A bay is seeded **once**: a sensor that returns
after a *later* loss is judged like any other reading, because the drive may have
been changed while nothing was watching.

Output per constrained bay: `t = T̂_d`, `σ = sqrt(P_dd + σ_cal²)`, margin
`k_sigma·σ`, `soft = limit − comfort − k·σ`, `hard = limit − k·σ`,
`q_w`; per zone air estimate and drift. `σ_cal` is `sigma_uncalibrated_c`
(1.5 °C) uncalibrated and
`max(0.5, EW-RMS residual)` calibrated; the filter cannot shrink it. This
is the honest cost of not touching the drives: without SMART the absolute
sensor-to-drive offset is a prior, and it decides how loud the fans run.
Per bay the block also carries `observed` (a trusted proximal member this tick),
`seeded`, `offsets_c` (the disagreement the filter carries per further proximal
member), `proximal_map` and the **settling
block**; per zone `air_blind_s`, the wall-clock time since a
trusted `zone_air` reading was fused (a tick gap counts in full, capped at the
filter's 3600 s prediction horizon). `trust_rule: sigma` reads `observed`,
`settling` and `air_blind_s` (§3 per-zone trust); `seeded`, `offsets_c` and
`proximal_map` are diagnostics. `proximal_map` carries, per further proximal
member (item 101): `slope_delta` / `offset_delta_c`, the learned difference from
the bay's own map; `slope` / `offset_c`, **the map the filter actually predicts
that member with** — the difference added to the bay's map and clipped to the
bounds an accepted calibration has; `clipped`, whether that clip bit; and
`samples`. The effective pair is published beside the learned one because they can
part company, and a reader working out why a bay under-reads must see the numbers
the estimator is running rather than the ones it wishes it were.

**One owner, two exemptions** (item 100). "This bay is settling" used to mean two
different things in two places: the estimator's `settling` flag and the DAS MPC
solver's own inference from the published σ. The estimator is the truth — it is the
one that widened the bay, and the only one that knows why — so it now marks every
reason and publishes both verdicts, each with the reason and the seconds left:

| key | what it excuses | reasons | bounded by |
|---|---|---|---|
| `settling`, `settling_reason`, `settling_until_s`, `settling_spent_s`, `settling_budget_s` | the `sigma` trust rule's per-bay σ check (§3 per-zone trust), and only while the bay is `observed` | `jump` (the fast-swap rule), `occupancy` (a change into or out of `empty`) | `bay_settle_s` per window, `bay_settle_max_s` in total |
| `model_exempt`, `model_exempt_reason`, `model_exempt_until_s` | the DAS MPC validity gate's prediction-error, drift and air-disturbance checks | `occupancy` (any change), `uncertain` (`σ² − σ_cal² > bay_uncertain_var_c2`), `calibration` (`σ_cal` moved by more than `bay_cal_step_c`) | `bay_settle_s`; no budget, no `observed` |

The two lists differ on purpose. The trust rule asks *"did the filter widen this bay
deliberately?"* — an event, and one a jumpy sensor must not be able to claim for ever,
hence the budget. The model gate asks *"is this bay tight enough to score a model
against?"* — a level, which a bay nobody is reading fails just as surely as one that
has just been swapped, and which the trust rule must **not** excuse (that is the
observability loss it exists to catch). A jump reaches the model gate through
`uncertain`, because the jump is what put the variance there.
`control/solver_das.py::_settling_bays` is now one read of `model_exempt`.

One behaviour changed with the ownership, and it is the intended direction. The
marks used to live in the *solver's* memory (`mem["settle"]`, `mem["cal"]`), which
`_fresh_memory()` cleared on every `initialise()` — so a bumpless transfer (a zone
back from a fault, a solver fault and its retry) put a bay that was still settling
straight back into the prediction-error, drift and air-disturbance checks. They now
live with the estimator and survive that restart: a fresh solver memory is evidence
about the solver, not about whether the bay settled. Everything else reproduces the
old set tick for tick, which is why the four DAS goldens do not move.

**Occupancy** (`topology.bays.<b>.occupied`, runtime `POST /api/bay`):
`true` is always `occupied`, `false` always `empty`; `auto` runs a machine
on `ΔT = T̂_s − T̂_a` and the heat the filter attributes to the drive:

- `unknown` at start and after `occupancy_hold_s` (30 s) without any trusted
  proximal member of the bay (a redundant member keeps the state); a shorter
  dropout keeps the state and the pending counts (item 19), `0` is the
  undebounced rule;
- `unknown → occupied` at once when `ΔT > occupied_dT_c` or a SMART sample
  of the bay's associated serial arrives;
- `occupied | unknown → empty` after `empty_confirm_s` with
  `ΔT < empty_dT_c`, heat < 1 W and no fresh SMART from an associated
  serial, every one of them on ticks with a trusted zone-air sensor;
- `empty → occupied` when `ΔT > occupied_dT_c` on 3 consecutive ticks,
  with the drive state reset to the sensor and a `reset_drive_var` (25 °C²)
  variance.

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
a starting point but `σ_cal` returns to `sigma_uncalibrated_c` until 20 fresh samples
confirm it again. SMART is never a gate input and cannot fault a zone.

**Manual calibration (item 23).** Where no SMART agent can reach the drives,
`POST /api/calibrate {bay, drive_temp_c}` carries one reading from a handheld
thermometer. It is **keyed by the bay**, not by a serial — the operator names
the bay, so there is nothing to associate — and is otherwise the same sample as
a SMART one: the same `smart_reject_c` rejection, the same RLS row, the same
acceptance rule (**20 fresh samples** with slope variance < 0.01 — so a single
handheld reading changes no estimate; the bay stays uncalibrated at
`σ_cal` 1.5 °C until twenty are in, and `POST /api/calibrate` answers with the
count so far), the same measurement of `T_d` at `R = 1 °C²` once accepted, the
same presence evidence for the occupancy machine, and the same expiry after
`calibration_max_age_days`. It is counted in its own `manual_used` /
`manual_rejected` totals, never in the SMART pair, so those stay a diagnostic of
the SMART path alone (an enclosure whose SMART agent has died shows no SMART
traffic, whatever the operator measures by hand). The reading reaches `step` as
`obs.inputs["calibration"]` (`{bay: {temp_c, ts}}`, the supervisor's
`TickPlan.calibrations`), so `step` stays a pure function of `(obs, cfg,
state)`. Its entry lives in the estimator's `manual` memory, never in `cal`
(which the model store owns per serial), so a swapped drive's SMART calibration
and the bay's manual one can never be confused; a bay's associated serial's
calibration wins once it is one the filter would use (accepted at least once, or
accepted now) — a SMART entry still collecting its first twenty samples leaves
the manual map in force instead of dropping the bay back to the prior. The
refusals are in §6 *Control*.

**Manual calibrations survive a restart (item 104).** They ride in the model
store's own per-bay `manual_calibration` section, next to the per-serial
`calibration` one, and their rule is stricter than the SMART one, because a
stale calibration silently trusted is worse than none. Every doubtful case is a
**drop**, which puts the bay back on the prior map at `σ_cal`
`sigma_uncalibrated_c` — more margin, more cooling, never less:

- **old** — an entry whose last accepted reading is further back than
  `estimator.manual_calibration_max_age_days` (7 days by default, wall clock,
  the outage included), or whose age is unknown or negative (a Pi whose clock is
  behind the file before NTP), is dropped outright, not restored with its fresh
  count reset the way a SMART entry is: nothing but the operator can refresh a
  handheld map, so an un-freshened one would sit there for ever showing
  `cal manual` with no evidence behind it. The window is shorter than
  `calibration_max_age_days` because the daemon was not watching and a handheld
  map is keyed by *bay*: a drive swapped during the outage would inherit the
  previous drive's map with nothing to notice.
- **swapped** — each stored entry carries the bay's declaration as it stood when
  the file was written (`occupied`, `class`, `serial` — exactly the policy the
  store's fingerprint leaves out on purpose). Any of the three different in the
  running config drops the entry, naming which: the owner has said this bay holds
  a different drive. While the daemon runs, only the *evidenced* half of the
  estimator's own swap rule (item 12 — the bay's occupancy crossing `empty` in
  either direction) drops that bay's manual entry on the tick it fires, restored
  or measured in this run alike. That is the case where a drive demonstrably left
  or arrived, so the new drive legitimately has no map: the bay falls back to the
  prior at `sigma_uncalibrated_c` and the owner measures the new drive when he
  wants it calibrated. A reading the supervisor is still offering is re-absorbed
  on the next tick, but it is re-absorbed as *one* sample into a brand-new entry
  (`n = 1`, `fresh = 1`), not as the old map: the bay stays uncalibrated until
  twenty fresh hand readings accept it again, which is the point — the map that
  was dropped described the drive that left. The other half of the rule — a bare
  proximal step, with no occupancy crossing — does **not** drop the entry; it
  inflates it (`inflate: 2.0` / `confirm: 20`). One tick's innovation against the
  predicted node is raised by a spin-up, an I/O burst or a fan step as readily as
  by a swap, the identical event leaves a SMART entry untouched, and twenty
  readings the owner took by hand are not worth deleting on evidence that thin.
  Inflation is the conservative direction anyway: larger `σ_cal`, more cooling.
- **reconfigured** — a file written for another *structure* is already refused
  whole by the store's fingerprint; the per-entry declaration covers the policy
  the fingerprint leaves out.
- **always provisional** — a restored entry always comes back with
  `inflate: 2.0` and `confirm: 20`, out of a `fresh` file as much as out of a
  `stale` one. A SMART calibration may be trusted at face value out of a fresh
  file because it is keyed by serial and the SMART feed re-associates it; a
  handheld map has no feed, so its `σ_cal` stays doubled until twenty fresh hand
  readings of that bay confirm it — indefinitely without them. `GET /api/model`
  and `diagnostics["store"]["sections"]["manual_calibration"]` show how many came
  back, and the warnings name every entry that did not.

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
or a serial silent for `smart_max_age_s`; a drop forgets the serial's history,
so the pair must win (3) again over a full window. (5) **Re-checked**: an
accepted pair keeps recording its series and is re-scored against its own bay at
every evaluation; `associate_drop_checks` consecutive scores below
`associate_drop_corr` drop it. Keeping a pair asks less than choosing one (on the
truth simulator a correct pair scores 0.71–0.97 and dips to 0.37 through a quiet
window, a wrong one has a median of −0.27 to +0.24). Acceptance itself forgets the
serial's history too, so the first re-check scores a *fresh* window rather than the
one that accepted the pair. Until a correlated pair has passed one re-check its
calibration is **not used**, however many samples it has: the first window is the
one the re-check cannot judge yet, which costs an undeclared serial about two
windows before its map is used. A declared serial
takes part in none of this. Until associated, a serial's
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

**Per-channel split** (`model_split_channels`, item 13). Regulation moves a
group's channels together, so only `E_zG` is identifiable from it; an
experiment's single-channel phases move them apart. With the switch on each
multi-channel group carries one `Es.<zone>.<group>.<channel>` per channel
beyond the first, whose regressor `φ_ch − φ_zG` is exactly zero under common
motion — at the prior split the model is the shared-`E` model term for term.
The split redistributes the group's coefficient and never changes its total
(`E_ch = E_zG w_ch/W + Es_ch − (w_ch/W) Σ Es`), the convergence rules still
read the group coefficients alone, and `GET /api/model` reports
`e_per_channel` per zone. After every window — and when a stored memory is
read — the split is projected: it is scaled back by the largest factor in
[0, 1] that keeps every `E_ch ≥ 0`, because `Q_zG = Σ E_ch φ_ch` exactly and
a fan never cools less than nothing. The projection keeps the split's
direction and its total, and a feasible split (every split at its prior
included) is left untouched, so a noisy single-channel phase can mis-split a
group but can never reverse a zone's modelled airflow or flatten its
Jacobian.

| Key | Unit | Bounds | Prior | Identified from |
|-----|------|--------|-------|-----------------|
| `E.<zone>.<group>` | W/K | [0, 200] | 33/fan × count / zones listing (0.1× coupled) | air-node RLS with fan excitation; weak cross-zone E ridged |
| `Es.<zone>.<group>.<ch>` | W/K | [−200, 200] | 0 (`model_split_channels`) | air-node RLS, single-channel experiment phases only (ridge to 0) |
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
| `u0.<model>`, `n.<model>` | – | [0, 0.5), [0.5, 1.5] | `fan_models` deadband / exponent | config; `tools/fit_fans.py` offline; the online fit with `fan_curve_online` (item 14) |

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
exception (memory reset to the prior), `frozen` for a zone whose
coefficients are held (never moves them, goes `suspect` by the converged
rule): one loaded converged from a fresh store file, or — with
`model_freeze: true` — one that reaches the converged rule at all, and
one already `converged` when the switch is turned on (at its next closing
window, so the switch needs neither a restart nor a store file), so a
model the owner considers finished stops adapting online. The switch
acts as soon as the converged rule holds; it has no dwell of its own. The switch
freezes a good model only: a frozen zone whose prediction error goes bad
still becomes `suspect` and learns again, and a stale file's hold counts
`frozen` as re-confirmed. `converged` needs enough excited
windows, `pe_min` above its floor, every in-zone `E` and every `k` with a
relative standard error below `model_converged_rel_se`, and `pred_err_c
< model_max_pred_err_c`. The model's status is the least advanced zone's;
`stale` while a stale store file re-confirms (below).

**Partial convergence** (`model_converged_bays_frac`, default 1.0 = the rule
above, §8 item 119). Below 1.0 the zone's **air block** must still pass whole,
but only that fraction of its occupied bays need to — rounded up, never fewer
than one, since a bay block is the only evidence there is about the bay the
solver plans for. The model is not allowed to treat the rest as informed: the DAS
MPC reads each such bay's `k` at `max(k − model_partial_k_sigmas·se(k), 0.05)`,
its own lower confidence bound, which is the safe side of the asymmetry (an
under-estimated `k` makes the solver believe airflow helps that bay less than it
does, so it runs the fans higher). The zone publishes `partial: true` and
`uninformed`, `blocked` keeps naming those bays — it is empty exactly when every
block of the zone has informed itself, not merely when the gate accepts the zone
— every bay publishes the `k_used` the solver plans with beside its fitted
`theta` and the σ that haircut spent as `k_se_used` (which `se` will not publish
on a block that has closed no window, item 111, so the derating stays
reproducible from the diagnostics), and `model_freeze` never enters or holds a
partial zone as `frozen`.

*Has this block informed itself?* is a **latched** question wherever it is asked
after the fact. Two of the three terms are facts about the fit — `n` only grows,
and a relative standard error is a property of the covariance — but `pe` is an
EWMA over the last `PE_WINDOWS` regression windows and decays back to nothing
about an hour after the last identification experiment at the shipped
`model_window_s`, which is the state a finished enclosure is in nearly all of the
time. So the excitation half is latched at the window that first satisfies it and
cleared only by a hot-swap reset. Without the latch every block of every
converged zone would read as uninformed again as soon as the fans went quiet:
`blocked` would fill up on a healthy model, `partial` would never clear,
`model_freeze` could never fire and every `k` of every zone would be haircut for
ever. The *entering* decision still reads every term live, so when a zone
converges is unchanged. Measured on
`sim/das.py` with group experiments, realistic quantisation, lags and
placement: per-bay `k` within 7–16 % and in-zone `E` within 9–22 %
across 8 seeds. `E` is sensitive to relative zone-air/inlet sensor
offsets (0.1 °C moves it 15–35 %, because a zone's air rise is only
0.3–1 °C); `k` is not.

**Fan curve** (`control/fancurve.py`, `fan_curve_online`). `u0.<m>` and
`n.<m>` are config by default. With the switch on, the daemon collects
settled `(pwm, rpm)` pairs — a duty that has held for
`fan_curve_settle_s` — into PWM bins per fan model (bounded: a full bin
becomes an exponential mean, so a replaced fan is followed), refits every
`fan_curve_refit_s` over the same `deadband × exponent` grid
`tools/fit_fans.py` searches with `rpm_max` in closed form, and accepts a
fit only with enough bins over enough PWM span and a relative RMSE at
most `fan_curve_max_rmse_frac`. Until then the configured curve stays.
Accepted curves live in the store's `fan_curves` section, so they survive
a restart. One fit per fan model, not per channel: a tach-less output is
covered by the curve of its own model.

An accepted fit **goes stale** after `fan_curve_max_age_s` without being
re-confirmed (default 3600 s, validated `≥ fan_curve_refit_s`). Two stamps
confirm it, and either one running out is enough: the tick that accepted the
fit (a refused refit leaves it alone, so a tachometer that lies badly enough
that no refit passes again ages out) and the tick of the newest **tachometer
reading** of that fan model (the bins keep their history, so a dead
tachometer would otherwise let the fit be re-accepted from year-old data for
ever). The second stamp counts every finite reading, *not* only a settled
`(pwm, rpm)` sample: the DAS QP moves the duty on nearly every solve, so
settled samples can stop arriving for hours with the hardware perfectly
healthy, and a controller that is simply regulating must not lose the curve
its live tachometers keep confirming. Past that age the fit stops being
published: its entry leaves `solver_memory["fan_curves"]` — or, when the
model store's seed carried a curve for that model, falls **back to the
stored one**, which the store's own age rule judged at load, rather than
being deleted from the section `model.json` saves — and its readers are on
whatever is in force again.

**Who reads the curve** (item 107; the table lives in code as
`fancurve.READERS` and is reported in the diagnostics, so the running
daemon says which curve produced which number):

| Reader | Curve | Why |
|--------|-------|-----|
| thermal model `u0.<m>`, `n.<m>` (`thermal.model_params`) | fit | it plans on the air a fan moves |
| DAS MPC prediction (`solver_das`) | fit | the same parameters through the prediction |
| estimator airflow `Q_z`, `Qn_z` (`estimator._airflow`) | fit | an estimator that disagrees with the model about the airflow is read by the prediction-error guard as a bad model: a fallback, safe but louder than it needs to be |
| noise objective `u0` (`noise.channel_deadband`, read by `surrogate`, `_level`, `rpm_model` and `noise_diagnostics`) | fit | it changes the objective, not the safety: with a real dead band of 0.25 and a configured 0.1 the surrogate charges for noise over a band where the fan does not turn, and one `u0` across model, estimator and objective keeps them consistent. `channel_power` and `noise_db` read no `u0` at all — they take the speed fraction the caller computed, so the curve behind them is the curve behind that fraction |
| noise `noise_db_at_max` | config | a datasheet figure at `rpm_max` that a PWM → RPM fit says nothing about |
| noise `rpm_max` (tach normalisation, `rpm_cmd`) | config | a fitted `rpm_max` normalises away the loss of speed it was fitted to |
| `model_use_rpm` (`thermal._channel_phi`) | config | same: the fit supplies the shape, the commissioned `rpm_max` the reference |
| fan health rpm and power rules (`health.py`) | config | the rule judges a fan against its commissioned figures; a curve fitted to those same tachometer readings would follow a fan that slows down and the deviation would never show. A safety floor derived from the configured curve does not drift with a fit |
| gate Stuck airflow evidence (`StuckParams.airflow`) | config | a sensor-validity rule, derived from the config once at load |

`GET /api/state`'s `fan_curves` diagnostics carry that table (`readers`)
and, per fan model, the curve in force, its source (`fit` | `store` |
`config`), whether an accepted fit has gone `stale`, its `age_s` and the
`sample_age_s` of the newest tachometer reading behind it; the `noise`
diagnostics carry the `u0` each channel's index was computed with and the
`curve` it came from, and each estimator zone carries `airflow_curve`
(`fit` | `config` | `mixed`, the last one for a zone whose channels are of
several fan models and only some of them have a fitted curve).

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
`model_max_pred_err_c`; the **drift** ≤ `model_max_drift_c_per_min`; the
**air disturbance** ≤ `model_max_air_dist_c_per_min`. Only bays with a
row (an estimate, a trusted zone, not empty) are predicted and scored, so
a drive in a faulted zone is not evidence about a model the solver never
plans for it (§8 item 11). Bays the estimator reports `model_exempt` are
left out of the last three checks, and its `model_exempt_reason` says
which of `occupancy`, `uncertain` and `calibration` it is (§3 *One owner,
two exemptions*, §8 item 100). A failure switches to the **PI-like DAS form on the same
estimates**: `mode` stays `auto`, `diagnostics.solver_diag.model` says
`active: pi_das` and why. The MPC returns only after the checks pass at
`model_return_factor` (0.5) times their numeric limits for
`model_return_dwell_s` (300 s) — every limit but the air disturbance's,
which is a move, not a level.

The **drift** is relative: `max_j |m_j − r_j|`, with `m_j` the model's own
rate `dT_d,j/dt` at the estimate and `r_j` the observed rate of the
estimator's drive `j` (its tick-to-tick difference low-passed with
`model_drift_rate_tau_s`, 120 s, starting at 0 for a fresh track). On the
**entry** `m_j` goes through that same low-pass, starting at the model's
raw rate for a bay without a filter state, so a fresh track checks the
plain drift. A load step warms the
drives at 0.26–0.36 °C/min, which a sound model predicts; the plain drift
had to fall below 0.25 °C/min before the return's dwell started and held
the fallback for up to 29 minutes on the truth simulator, and it tripped
the *entry* twice over: on the drives' physical warm-up right after
`bay_settle_s` (three of eight `rich` seeds, 0.50–0.52 °C/min, §8 item 65)
and on the MPC's own quieter move right after a return (a return at
2110 s, a re-entry at 2150 s on every preset and seed, §8 item 64). Both
sides through one filter removes both: a command move enters the model's
rate and the drives' alike, and physical warming the model predicts
cancels. A model whose equilibrium is wrong keeps its drift while the
drives settle and still enters the fallback and stays there (bay gains
0.3×, 0.5×, 2× and 3×: caught 170–420 s after the load step, against
0–860 s before). Nearer the edge the residual is what limits the gate,
not the dwell: bay gains 1.5× hold the drift at 0.49–0.54 °C/min against
the limit 0.5 and are caught 220–530 s after the step wherever they reach
it (`basic` z0, most `rich` zones and seeds), while on `basic` z2 a 1.5×
error puts the drift over the limit on two ticks of a whole run and the
gate does not see it at all — it is also 10.8 °C from any drive limit. The **return** keeps the form §8 item 10 measured — the
model's rate as it is against the filtered observed rate — because while
the fallback regulates, the model's rate is not answering a move of its
own, and filtering it there only lags the return (540 s against the
documented 320 s bound on several `rich` seeds). `checks` reports the
entry's drift, the return's (`drift_return_c_per_min`) and the plain one
(`drift_abs_c_per_min`).

The **air disturbance** is the one piece of evidence about the fan gains
`E` (§8 item 66). `E` acts on the air node alone, and the estimator's
per-zone air disturbance `d_air` re-balances that node at any one
operating point, so airflow the enclosure no longer has — a blocked
filter, a dust mat; the tachometers read the same — leaves the drive
rows' prediction error and drift where they were (measured: 0.14 °C and
0.20 °C/min against limits 1.0 and 0.5 at a third of the airflow). What it
cannot hide is the move, so the check is `max_z |d_air,z −
d_air,z(slow)|` over the zones of the constrained bays, against the same
disturbances through a second low-pass `model_air_dist_tau_s` (900 s). A
level is a reference only once it has run for that time constant: a fresh
track, and one a gap of over an hour has made stale, report
`air_dist_c_per_min: null` until then rather than a move of zero against
a level snapped to whatever the disturbance is now. A clock stepped back
keeps every level and its age — only the filter refuses to advance — so a
step of the wall clock cannot re-reference the check to a disturbance
that has already moved.

Both are rates the plant itself moves, so on the **entry** they fault the
model only after failing for `model_drift_dwell_s` (120 s); until then
the MPC keeps acting and `reason` names the failing check. The dwell is
**leaky**: a passing tick does not restart it, and only a passing spell
as long as the dwell itself does. A moderate parameter error holds its
residual just over the limit and sensor noise dips it under every few
ticks, so a dwell that had to be contiguous would restart for ever and
never fault the model at all — measured with bay gains 1.5× on `basic`
seed 2, where the drift was over its limit on 112 ticks across 18 minutes
and never for 120 s together. A switch either way clears the dwell: it
belongs to the entry, not to the fallback. The
structural checks (status, parameters, eigenvalues, gain) and the
prediction error still fault the model on the tick they fail. Every switch is
bumpless. The fallback regulates every drive at its soft target with the
same margins, so a model fallback changes loudness, not safety. Without
`model_accept_prior` nothing can act before a model has converged, which
needs experiments (or a fresh store file).

**Bumpless (DAS MPC):** when a channel starts being driven (no integrator
entry) or the active model switches, the solver records `bias = prev −
demand` so the first output equals `prev`; the offset decays with 15 s
when more cooling is wanted and 60 s when less. `integrator` holds the
first block per driven channel.

**Budget at `dt = 5 s`.** Hard gate per tick 250 ms (5 % of `dt`; raised from
500 ms to 600 ms by the owner after the Zero W measured a DAS MPC p99 of
507–552 ms, then 609–615 ms after items 3, 8, 9 and 10 -- item 73;
**re-derived to 250 ms on the Zero 2 W, item 73, 2026-09-17**: measured
DAS MPC step p99 85.96 ms, max 92.61 ms, over 600 ticks against
`config.example-das.yaml`, `tools/bench_step.py --sim-plant das` --
2.9× the measured p99 and 2.7× the measured max, so a regression that
tripled the solver's cost would still trip the gate. Not chased lower:
at 200 ms (2.3× the measured p99) the first garbage-collection pause on a
loaded board would false-alarm), alarm 350 ms (a 1.4× warn-to-alarm
ratio, up from the old pair's 1.25×; §8 items 73, 95). Measured with
`tools/bench_step.py` on the development machine (`--sim-plant das` for
the DAS rows) and, for the Zero W and Zero 2 W columns, on the Pi itself.
The Zero W column is measured, not extrapolated; so is the Zero 2 W
column, on a Raspberry Pi Zero 2 W (4 cores at 1.0 GHz, 64-bit trixie,
kernel 6.18, no throttling -- `get_throttled` 0x0 throughout, board
temperature 41.9→49.4 °C over the run). The factor between the
development machine and the Zero W is **not** the single 100× the plan
first assumed: the legacy MPC scales by about 100× (0.3–0.4 ms against
34.5 ms), the DAS MPC by about 180× (3.3–3.6 ms against 609–615 ms) and
the PI-like DAS form by about 220× (1.5 ms against 322 ms). The more of a
step is small numpy calls and Python bookkeeping rather than the few
large matrix operations the ×100 was measured on, the worse the Zero W
does — so an extrapolation from this machine is a lower bound on the Pi,
never a promise. The Zero 2 W's four cores at 1.0 GHz cut the Zero W's
single 1.0 GHz core's numbers by roughly 7×, well past what core count
alone would predict for this single-threaded step -- the newer core is
also simply faster per cycle.

| Configuration | Step p99 (dev machine) | Zero W | Zero 2 W | Verdict |
|---------------|------------------------|--------|----------|---------|
| Legacy MPC, 2 temps × 2 channels, `dt = 2` | 0.3–0.4 ms | 34.5 ms measured (×100) | 7.14 ms measured | legacy reference |
| DAS MPC, 25 sensors, 15 bays, 8 channels, N = 20 × 30 s, blocks `[1, 1, 2, 4, 6, 6]`, estimator every tick | 2.8–2.9 ms after item 73, 3.3–3.4 before (9–11× the legacy MPC) | 609–615 ms measured before item 73 (×180) | **85.96 ms measured, max 92.61 ms** (item 73, 2026-09-17); solve_p99_ms 87.22 ms over 300 solve ticks; model_active_fraction 1.0; modes auto 562 / saturated 38 / degraded 0 / fallback 0 | under the 250 ms gate (2.9× headroom); item 73 done |
| PI-like DAS form + estimator, same layout | ~1.5 ms | 322 ms measured (×220) | 25.36 ms mean, 26.93 ms p99, max 27.56 ms measured | always available |

Where the DAS MPC's step goes after item 73 (development machine,
`config.example-das.yaml` against the DAS truth plant, 240 ticks with 20
discarded, per-phase wall time on a **solve** tick; timing wrappers add a
few per cent to every figure): the SQP with its box QPs 0.73 ms, the
estimator 0.64 ms, building the penalised QP and the noise surrogate
0.21 ms, the prediction 0.28 ms (of which the linearisation rebuild
0.15 ms — it misses the memo on about half the solve ticks, 51 % over 380
ticks and 65 % over the 220 measured here, and a bigger memo does not
help: the command keeps landing on new quantisation points, so the miss
rate is the same at cache sizes 8 through 256), the operating point and
the model checks 0.17 ms, the solver's
own bookkeeping 0.28 ms, the sensor gate 0.10 ms, the `json.dumps` guard
on the solver memory and the diagnostics 0.10 ms, the rest of `step`
0.20 ms. Cutting the first two means changing the arithmetic, which would
move the goldens; whether that regeneration is worth it is §8 item 95.
Item 67 added two states to the estimator's filter (one placement offset per
proximal sensor of a bay beyond the first: the example has two, in two of its
four zones). Measured the same way on the development machine, 400 ticks,
two repeats each: DAS MPC mean 1.93–1.95 ms before against 1.97–1.99 after and
a solve-tick p99 of 3.41–3.44 ms before against 3.34–3.41 after; PI-like DAS
1.03 against 1.04–1.06 mean. The Zero W column is not re-measured: the
owner has since moved to the Zero 2 W, and item 73's re-measurement above
is against that board.

CI checks the ratio (DAS MPC p99 ≤ 12× legacy MPC p99 in the same
process, `tests/test_bench_budget.py`); the absolute `mpc.budget_ms` gate
runs only on the Pi (marker `pi`). If the Pi still measures worse, the
owner's fallback (§8.1, 2026-09-14) is config, not a code change:
`budget_ms: 1000.0` **with** `budget_alarm_ms: 1250.0` (the model rejects
a `budget_ms` that is not below the alarm) and `mpc_every_ticks: 3`.
Solving every third tick lowers the mean step and the CPU share, not the
p99: while solve ticks are more than 1 % of ticks the p99 over all ticks
is a solve tick, so the raised `budget_ms` is what covers it. Further
reductions, in order: `mpc_every_ticks: 4`, fewer or longer blocks with a
shorter horizon, a Zero 2 W.

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
`thermal` (the thermal memory, which `tools/fit_model.py --store-out` can
write straight from an offline fit, item 15), `fan_curves`, `calibration` (per bay and
serial, with wall-clock `last_sample_wall` and `expires_wall` so expiry
survives a reboot), `manual_calibration` (per bay, item 104: the handheld
maps of `POST /api/calibrate`, with `last_sample_wall` and the bay's
declaration at the snapshot instead of an expiry of their own — their
window is the *loading* config's `estimator.manual_calibration_max_age_days`,
and the whole rule for what comes back is in §3 *Manual calibrations
survive a restart*), `bays` (last occupancy, class, serial,
association; report only) and `ident_settle` (the experiments' settle
timers as **seconds already settled** per zone, so they need no clock
conversion: the outage is subtracted from them on load). `ModelPersister`
(a loop `on_tick` observer)
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
siblings hold the group's base. The sequence is a two-level random
telegraph: levels `u_base` and `u_base + ident_amplitude` (`above`,
never less cooling than the solver asks for) or `u_base ±
ident_amplitude` (`symmetric`), holds drawn from `ident_hold_s` by a
seeded LFSR, `ident_max_duration_s` split over the phases. The levels are
ordinary supervisor overrides, so `compose` rate-limits and clamps them
and fallback beats them.

**A rise in the solver's demand wins over the experiment's plan, always**
(`ident_replan: true`, item 52). Every tick each channel's base is raised
to the solver's want for it (`diagnostics["target_pwm"]`, before the rate
limit and the clamp) and its two levels are re-derived around the new base
and clamped into `[pwm_min, pwm_max]`. That want is **not** free of the
experiment's own influence — the DAS MPC's objective carries
`weight_dpwm·‖u₀ − prev‖²` and `prev` is what was last on the fan, so
following it raw would ratchet (measured on the DAS example, all else
equal: `prev` 0.40 → want 0.398, `prev` 1.00 → 0.962). The echo of the
experiment's own level therefore comes off first,
`wanted = demand − max(0, min(prev, own) − base)`, with `prev` from
`diagnostics["prev_pwm"]` and `own` the override the experiment itself had
on that tick: a fan above the base because the experiment put it there is
an echo and comes off, a fan above the base because the solver's own floor
lifted it there is demand and stays. The base **rises at once and falls
slowly**: a fall is taken only at a level switch, by at most `d_pwm_max`,
and never below the base of the start, so a hold keeps its level while a
spent excursion is released within a hold or two instead of pinning the
channel and its held siblings at its peak for the rest of
`ident_max_duration_s`. A channel whose demand or `prev` is missing this
tick keeps its base. The schedule (phases and switch times) still comes
from the seed once, at the start. Because a re-planned level can only reach
the fans on the next tick, `compose` also floors an experiment channel's
own override with the solver's command for the tick
(`max(override, mpc_cmd.pwm[ch] − dip)`, `dip` 0 under `above` and
`ident_amplitude` under `symmetric`, both read off the **running**
experiment's own plan so a config rebuilt under it cannot take the floor
away), so no tick of an experiment puts less on a fan than the controller
would have. The aborts of a tick are decided on that tick's estimates, with
the same thresholds in the same order, *before* the next tick's levels are
re-planned: the re-plan cannot weaken the decision itself, and a re-planned
level is never below the frozen plan's for the same tick. Across ticks it
does move the aborts, which is the point — a channel followed up cools its
own bay, so an excursion the frozen plan would have ended on the envelope
runs to completion; fewer envelope aborts are the intended outcome, not a
weakened rule. What it costs the identification: the levels move with the
demand, so the step sizes change (a rise of `d` inside a hold makes the
next switch `A ∓ d` instead of `A`, and a level clamped at `pwm_max`
squeezes the step — under `above` with the demand at `pwm_max` the
excitation stops until the demand falls back). The base drift is slow next
to the 60–180 s holds, so the regressors keep the content the fit lives on
(§8.4 item 52 has the sim numbers and the measured per-tick cost).
`ident_replan: false` keeps the levels frozen at the base of the start,
which is what ran on the Zero W, and costs exactly what it cost then.

**A whole zone at once, on independent codes** (`ident_parallel`, default
`false`; item 102). One group at a time is the wrong unit for the *model*. The
thermal model's air block per zone regresses one airflow regressor per fan group
of that zone, and `converged` asks the smallest eigenvalue of their information
matrix to pass `PE_MIN`: every group of the zone has to move, independently of
the others, inside the PE monitor's ~30-window memory. Telegraphing one group
while the solver carries the rest fills one direction of that matrix at a time,
and the others only as far as the solver happens to move them: a round robin over
*every* channel does push `pe_min` past `PE_MIN` now and then — peaks of
0.15–0.22 against the 0.05 bound — but it does not hold it there, and over three
seeds it latches `converged` on no zone of twelve (it latched one, on a
transient, before item 112 changed where the fans park between experiments).
That is why no closed-loop scenario had ever been shown to converge (§8.4 item
102 has the eigenvalues and the per-seed numbers, item 112 what moved them).
With
`ident_parallel: true` the target's channel set grows to every channel of the
zones that **list** the target's channels (`ZoneLayout.zone_channels`; the
coupled zones are served, checked and aborted on as before, but not excited), the
experiment is one phase over all of them, and each channel gets its own telegraph
— its own start level and its own hold draws, from an LFSR seeded per channel
from `ident_seed`. Nothing else moves: the same two levels around the same
anchor, `above` still never below the solver's command, the same preconditions
over the larger channel set (so a start is refused when any of those channels is
saturated or would leave the band), the same envelope and abort list on the same
served zones, and `compose` still floors, rate-limits and clamps. Under `above`
the cost is noise, not temperature: `G` channels sit a step above the anchor for
about half the run instead of one — measured on the DAS example against a round
robin over every channel of the plant, **−0.007 to +0.025 mean PWM** over a 16 h
run. `above` never commands less cooling than the solver **on the tick**, which
is a per-tick statement and not a claim about two 16 h trajectories: the
zone-wide arm ended with the larger worst true margin on every seed before item
112, and 0.27 °C below the round robin on one seed after it (4.026 against 4.296;
both arms keep ≥ 4 °C and violate nothing — §8 items 102, 112). Per zone identified the
fan-seconds are arithmetic-neutral (one zone-wide experiment does the work of the
`G` sequential ones its zone needs, in a `G`-times shorter window), but that is
arithmetic and not what the A/B measured: both arms there run the same 16 h of
experiment. Under `symmetric` the accepted worst case widens with the key:
`compose` floors each experiment channel at its own solver command minus
`ident_amplitude`, and a coded phase can have *every* channel of the zone at its
low level on the same tick (10 % of the ticks of a three-channel phase on the
scenario's knobs, 20 % of a two-channel one), where the old schedule could only
ever dip one group of it. The envelope, the abort list and the floor itself are
unchanged; `above`, the default, gives up no cooling at all.

**How far a channel can move the PE monitor from where it sits** (item 110).
`pe_min` is a **relative** measure — the monitor normalises each fan regressor by
its own running mean — so `converged` wants a relative airflow variation of
`√PE_MIN` = 0.224 in the least excited direction. A 50/50 telegraph at base `u`
with dead band `d` reaches `A / (2(u − d) + A)` under `above`, i.e. it needs
`A ≥ 0.576 (u − d)`: a channel parked past ≈ 0.62 PWM cannot clear the bound at
any amplitude the `(0, 0.3]` cap allows, and one parked at `pwm_min` clears it
with 0.06 while the schedule spends whatever `ident_amplitude` says.

**The normalisation stays.** It is what makes the monitor read as *information*
rather than as PWM: a fan near `pwm_max` really does move proportionally less
extra air per unit of PWM, and dividing by something else would relabel an
uninformative channel, not inform it. What changed is that the arithmetic is no
longer invisible. `ident.excitation` publishes it per channel — `rel_swing`, the
`pe_reach` it implies, whether that clears `PE_MIN` at the configured
`ident_amplitude` (`excitable`) and whether any amplitude up to the `0.3` cap
would (`excitable_at_cap`, so the owner is told which remedy applies: a larger
amplitude, or waiting for the solver to park the channel lower) — in
`snapshot().extra["experiment"]` (`excitation`, `unexcitable`) and in a warning
on the start log line, and the thermal model publishes the measured half:
`pe_diag`, the **diagonal** of the very matrix whose smallest eigenvalue is
`pe_min`, one entry per strong group. The eigenvalue can never exceed the
smallest diagonal, so a group whose own entry sits at or under `PE_MIN` shuts its
zone's gate by itself and `pe_diag` names it. `ident_require_excitable` (default
`false`) turns the prediction into a refusal, `not_excitable:<ch>`.

**Sizing the telegraph to that headroom** (`ident_amplitude_mode`, item 120;
`fixed` by default = every channel spends `ident_amplitude`). In `headroom` a
channel's step is the **smallest** amplitude in `(0, ident_amplitude]` whose
`rel_swing` reaches `ident_pe_aim·√PE_MIN` from where it is parked — falling back
to `ident_amplitude` when even that falls short, so no channel is excited *less*
than `fixed` excites it — and the two levels are placed **inside**
`[pwm_min, pwm_max]` with the room the channel really has on each side. A level
the band will not take is **cut** at the rail, never slid across the base:
`ident_amplitude` bounds the excursion in *both* directions, because it is the
owner's ceiling on how much PWM one experiment may add (`above`) or give up
(`symmetric`) and `IDENT_AMPLITUDE_MAX` is that ceiling's own cap. Sliding a
symmetric pair up off `pwm_min` would keep its whole `2A` swing, but it would buy
that swing with up to `2·ident_amplitude` *above* the solver's command — the
loudest the enclosure gets, spent on the channel the solver had parked at the
rail because it wanted quiet there — so the cut costs swing instead and
`excitable` reports the short one. Since the levels are then inside the band by
construction, `band:<ch>` asks the question it was standing in for of what the
cut left: it refuses a start whose placement was cut **and** whose remaining
swing cannot reach the aim, so a channel with no room to move at all is still
refused (it would hold the schedule and every sibling channel for
`ident_max_duration_s` and inform the model of nothing) while a channel parked
near a rail with room to say something runs. It is
for `symmetric`, where the amplitude is cooling *given up*: since item 112 the
solver parks the channels near `pwm_min` between runs, which is exactly where a
symmetric pair does not fit, so `band:` refuses most of the programme (13 / 4 / 8
of 16 starts over 16 h on the three `rich` seeds at the shipped
`ident_amplitude: 0.15`, against 16 / 11 / 15 sized to the headroom), and the dip
per experiment falls by 3 / 28 / 39 % while the enclosure's own mean PWM moves by
at most ±0.004 either way. Under `above` the amplitude is cooling
*added* and the solver takes it straight back, so sizing down buys 0.002–0.008
mean PWM and costs a zone — item 110's own judgement, which is why the default is
`fixed`.

**The published reach is an upper bound**, not a prediction. The monitor never
sees the telegraph's two levels: it accumulates one sin²-weighted mean of the
airflow regressor per `model_window_s` block, so a regression window reads a
level only while it sits inside one hold, and a window that spans a switch
averages the two — which can only move it toward their mean. `rel_swing` is
therefore what the monitor reads exactly when every hold of `ident_hold_s` is at
least `model_window_s` long, and more than it reads otherwise;
`holds_cover_window` in the same status says which case the schedule is in. So
`excitable` is a **necessary** condition: `false` is a proof that the channel
cannot inform its zone from this base, `true` is permission for the schedule to
try. `ident_require_excitable` refuses on that bound, so it never blocks a start
that could have worked — with holds shorter than a window it can let one through
that still falls short, and `pe_diag` is what says so afterwards. The shipped
`config.example-das.yaml` is in that case (`ident_hold_s: [60, 120, 180]` against
`model_window_s: 120`); `tests/test_ident_converge_sim.py` lengthens the holds
past the window for exactly this reason. The reach is
computed from the commissioned `fan_models` curve, not from a fitted one
(`fancurve.READERS["ident_excitation"] = "config"`): `check_start` runs on the
supervisor's side, where a fit is not in hand, and the number is a plan for a
start rather than an input to the estimator, the solver or a health rule — the
measured answer is `pe_diag`, which follows whatever the model used.

**Why a zone is still learning** (items 110, 111). Every zone of the thermal
summary carries `blocked`: the parts of the `converged` rule it still fails, in
the rule's own words — `windows:<zone|bay>`, `pe:<zone|bay>`,
`rel_se:<coefficient>` and `pred_err`. It comes from the same function the status
machine decides with — and, since the review of items 110–112, from the same
occupancy rule (`thermal._bay_occupied`), so the published list and the decision
cannot drift apart on their input either — and it is empty exactly for a
`converged` or `frozen` zone. Each bay also publishes `se`, the absolute standard
error beside `rel_se`: `rel_se(k)` is `se / |k|`, so a bay whose airflow
sensitivity is genuinely small fails the relative gate on a fit no worse than its
neighbours' (§8 item 111 has the numbers). `se` is `null` per coefficient
wherever `rel_se` is — a block that has closed no window carries the prior's own
initial variance in `P`, and publishing its square root would hand the owner a
prior dressed as a measurement, on exactly the bays (b03, b10) item 111 asks them
to read `se` for.

**Start preconditions**, each refused with a
named reason: control mode `auto` without human overrides
(`control_mode`), last command `auto` (`mode:<m>`), no saturation, band
or stall on the target (`saturated:`, `band:`, `no_command:`,
`fan_stall:`), with `ident_require_excitable` a target channel whose telegraph
can reach the thermal model's PE bound from where the solver parked it
(`not_excitable:`, above), every served zone (listing a target channel, plus
`coupled_to`) trusted and fault-free for `ident_settle_s` (`settle:`),
no served zone with a sensor group that has no trusted, confirmed
member (`sensor_lost:`, from `diagnostics["sigma_floor"]`: under
`trust_rule: sigma` such a zone is still trusted, so nothing else in the
list sees the loss), no bay there `unknown` or mid-transition
(`bay_unknown:`, `bay_transition:`), no calibration in its first 20 samples
(`calibrating:`), and every drive within `ident_start_band_c` of its soft
target (`start_band:`). **Envelope** on the estimates, every tick, for
every constrained bay of the served zones: `T̂_d ≤ soft +
ident_max_over_c` (3 °C, owner-accepted), `T̂_d ≤ hard` always, and
`T̂_d + k·σ < limit − ident_abort_below_limit_c` (the absolute abort per
drive; there is no `ident_abort_temp_c`). **`k·σ` is counted once:** `soft`
and `hard` already subtract it, so only the absolute rule, measured against
the raw limit, adds it back. `T̂_d ≤ hard` therefore *is* `T̂_d + k·σ ≤
limit`. **Which rule binds:** on `T̂_d + k·σ` the envelope fires above
`limit − comfort_c + ident_max_over_c`, the absolute abort at `limit −
ident_abort_below_limit_c` and `hard` above `limit`, so the absolute abort
is always stricter than `hard`, and the *soft envelope* binds first whenever
`ident_max_over_c < comfort_c − ident_abort_below_limit_c` — the case for
every class of `config.example-das.yaml`: the envelope fires at `T̂_d + k·σ`
of 48 / 58 / 63 °C for hdd / ssd_sata / nvme against 49 / 64 / 69 °C for the
absolute abort. (The absolute abort binds instead where the comfort band is
narrow, e.g. the `quiet` preset on hdd.) So item 53 *did* move the binding
abort, up by `k·σ`, on the envelope — that `k·σ` was the double count — while
the absolute backstop is bit-identical to before it. **Abort list:**
any fallback tick or tick without a solver command, any `degraded` tick,
two failed applies, a clock running backwards or jumping past the duration,
the envelope, an untrusted served zone, a lost sensor group in a served
zone, an `unknown` bay, a stalled experiment fan, `stop`, and any human
intent. **The end of an experiment — completed or aborted — returns the channels
to the solver as it found them** (item 112): they are *not* put into
`TickPlan.released`. The experiment's levels are an override applied after
`mpc.step`, so the solver ran on every one of its ticks and its integrator
already holds the command it would have given without one. Releasing them drops
that entry and re-initialises the solver bumplessly at *the PWM on the fan*, so a
channel released on its high level was handed that level as the solver's own
starting point, stayed there until the integral wound it back down, and a later
start took the inflated command as its base. Nothing steps: the rate limit still
moves the fan at most `d_pwm_max` per tick back to the solver's own demand, and
under `ident_levels: symmetric` — where the old release could hand the solver the
*low* level, below what it wanted — the correction is now upward. A **human**
override is still released bumplessly: there the fan was parked by somebody
outside the loop. The control mode stays
`auto` during an experiment; its status is in
`snapshot().extra["experiment"]`. A restart never resumes an experiment,
but the settle timers do survive one: they ride in `model.json` as seconds
already settled, the outage is subtracted from them on load, and a zone
spends what is left only when it is trusted and fault-free again (nothing
unobserved is ever counted as settled; `ident_settle_resume_max_gap_s`
caps the outage). A start that arrives between `plan_tick` and
`record_tick` arms offset 0 one tick further out, because the tick whose
plan the loop already holds cannot carry the overrides.

On the truth simulator (`tests/test_ident_sim.py`), with
`config.example-das.yaml`, the PI-like DAS solver and every bay at 35 %
activity: `basic` seeds 1–5 keep a worst true margin of 10.73–10.74 °C with
the experiment against 10.72–10.73 °C without it, zero limit violations, the
excitation visible on the fans (max PWM 0.527 against 0.501); `rich` seeds
3–5 keep 9.81 / 10.06 / 8.32 °C from the start on, the same with and without,
zero violations, and `rich` seeds 1–2 are drawn saturated and the start is
refused. A settled enclosure never comes near the envelope, so the same file
also drives it there: with the room warming 20 °C/h, `basic` seeds 1–5 abort
on `envelope:<bay>` 12–13 minutes into the experiment at `T̂_d` 0.00–0.04 °C
past `soft + ident_max_over_c`, with `T̂_d + k·σ` 48.0 °C against the absolute
abort at 49.0 °C (so the soft envelope is what stopped the run, as above),
zero limit violations and 4.69 °C of worst true margin, on `ident_levels:
above` and `symmetric` alike — `symmetric` commands the low level throughout
and aborts a minute earlier for it. Experiments are needed where PI-DAS acts:
a model converges only with them.

### Track A — core (dev machine / CI, no Pi)

- Safety core: `control/gate.py`, `control/zones.py`, `control/mpc.py`,
  `control/solver_pi.py`, `control/solver_mpc.py`, `sim/plant.py`,
  `tests/test_gate.py`, `tests/test_zones.py`, `tests/test_mpc_*.py`
- DAS core: `control/estimates.py`, `control/estimator.py`,
  `control/associate.py`, `control/thermal.py`, `control/noise.py`,
  `control/solver_das.py`, `control/persist.py`, `control/fancurve.py`,
  `control/ident.py`, `control/spinup.py`,
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

- `hw/aquacomputer.py` (pure, stdlib only, no I/O): the aquaero 5/6 and
  Quadro HID report layouts. `decode_status(kind, data)` returns a
  `StatusReport` (serial, firmware, temperatures by input name in °C or
  `None`, per output rpm, output duty, voltage, current and power and
  whether a device is behind it, flow, the Quadro's power-cycle count).
  Control-report helpers: a channel's commanded duty and, on the aquaero,
  whether it follows its preset, whether it is an aquabus output and
  whether its block is unconfigured (`channel_state`, `channel_holds`),
  the profile the controller runs (`active_profile`, byte `0x06`, 1-based),
  `patch_duties`, `finalize_control_report` (the Quadro's checksum),
  `capture_channel` / `restore_channel`. `software_sensor_report(kind,
  {number: degC})` builds the HID **output** report that sets the aquaero's
  software sensors; every slot it is not given carries `0x7FFF` ("no data"),
  which leaves the device's own value for that sensor alone. A wrong id,
  length or checksum raises `ReportError`, and so does a sensor number or a
  temperature the report cannot carry. Protocol constants, not tunables (big-endian,
  offsets count from the report id, sizes include it):

  | | aquaero (USB `0c70:f001`, interface 2) | Quadro (`0c70:f00d`, interface 1) |
  |---|---|---|
  | status report | input id `0x01`, 903 bytes, about once per second | input id `0x01`, 220 bytes |
  | temperatures (1/100 °C, `0x7FFF` = none) | physical sensors `temp1..8` at `0x65`, aquabus temperature slots `bus1..8` at `0x75`, software sensors `soft1..8` at `0x85`, virtual sensors `virt1..4` at `0x95` | physical sensors `temp1..4` at `0x34`, software sensors `soft1..16` at `0x3C` |
  | output blocks (`fanN` tachometer, `pwmN` output) | `0x167 + 12k`, k 0..7 (1–4 its own outputs, 5–8 a device on its aquabus; rpm `0xFFFF` = no device): rpm +0, duty +2, voltage +4, current +6, power +8 | `0x70 0x7D 0x8A 0x97`: duty +0, voltage +2, current +4, power +6, rpm +8 |
  | flow | `flow1..3` at `0xF9` (`flow3` from aquabus; `0x7FFF` = no data) | `flow1` at `0x6E` |
  | identity | serial `u16` pair at `0x07`, firmware at `0x0B` | serial at `0x03`, firmware at `0x0D`, power cycles `u32` at `0x18` |
  | control report | feature id `0x0B`, 2707 bytes, no checksum | feature id `0x03`, 961 bytes, CRC-16/USB over `[1, size − 2)` stored in the last two bytes |
  | duty of output `k` (1/100 %) | k 0..7: preset `0x55C + 2k` = duty; control source (block `0x20C + 20k` + `0x10`) = `0x5C + k`; min power (+`0x04`) = 0; max power (+`0x06`) = 100 % | `[0x37 0x8C 0xE1 0x136][k]` |
  | save report (only `save()` sends it) | feature report `06 00 02 00 00 00 00` (verified) | feature report `02 00 00 00 02 00 00 00 00 34 C6` (not verified) |
  | software sensors | **output** report `0x07`, 17 bytes: eight `u16` 1/100 °C, `0x7FFF` = no data (settings in the control report from `0x177`, 5 bytes each: enabled, fallback, timeout) | not known |
  | active profile | control report byte `0x06`, 0-based | none |

  Units: duty 1/100 %, voltage 1/100 V, current mA, power 1/100 W. Config
  names: `pwmN` an output, `fanN` the tachometer of output N, `flowN` a
  flow sensor, and the temperature groups above. They are no longer the
  Linux driver's hwmon attribute numbers, which ran the temperatures in one
  sequence (aquaero `temp9..16` software, `temp17..20` virtual; Quadro
  `temp5..20` software) and numbered flow as `fanN` after the fans (§8 item
  85); the Quadro's 16 software sensors are named after the aquaero's,
  unverified. Temperatures decode as signed 16-bit values (the driver
  decodes them unsigned, so −1 °C would read 655 °C). The layouts are
  checked against captured reports and the driver's readings in
  `tests/fixtures/aquacomputer/` (§2 "USB spike results", "hidraw check");
  a duty write patched into the captured firmware reports reproduces the
  driver's write byte for byte, and one on output 7 the aquaero's control
  report with the Quadro on aquabus. The aquaero output mode word at block
  +0x0E (low byte `0x01` DC voltage, `0x02` PWM, §2 "hidraw check") is
  decoded (`output_mode`, `ChannelState.mode`) and never written; on the
  aquabus blocks 5–8 it is read but marked **uninterpreted**, and its name is
  `unknown` whatever the low byte holds, because one Quadro's four identical
  PWM outputs were measured reading `0x0000` and `0x0002` in one report and
  `0x0500` in an earlier one (§2, 2026-09-17). The Quadro's mode field is not
  known. `reports_power` is False for every aquaero output, its aquabus ones
  included: those blocks carry the bus device's current in about one report in
  four and 0 mA in the rest, so no single report's figure is a measurement
  (§2, 2026-09-17; §8 item 89). The `u16` at +0x0A of a fan block is not
  identified and is not decoded. A control report SET takes effect in the next status report and
  does not survive a power cycle; the save report stores the configuration
  (§2 "Apply without saving", §8 item 84). The official software and the
  driver send it after every write; the adapter does not (§8 item 86).
- `hw/hidraw.py`: discovery and transport. Every `hidrawN` under
  `/sys/class/hidraw` is described by its `device/uevent` (`HID_ID`
  vendor and product, the trailing `inputN` of `HID_PHYS` as the USB
  interface, `HID_UNIQ` as the serial; a line without `=` is skipped).
  A device is selected by kind and optional serial, never by its
  `hidrawN` number, which changes on re-plug; the aquaero's interfaces 0
  and 1 (keyboard, mouse) never match. None found raises
  `DeviceUnavailable`, several without a serial `AmbiguousDevice` naming
  the serials found. `HidrawTransport` opens `/dev/hidrawN` non-blocking,
  drains input reports, waits with `select`, and sends `HIDIOCGFEATURE`
  / `HIDIOCSFEATURE` (`buf[0]` = report id). `write_report` sends a HID
  *output* report with `write` (the software-sensor heartbeat). It is handed to
  the kernel, **not acknowledged by the device**: on an interface with an
  interrupt OUT endpoint usbhid queues the URB and `write` returns at once
  (only without one does the kernel fall back to a synchronous SET_REPORT
  control transfer), and a full usbhid output queue drops the report. So the
  returned byte count means "accepted for sending", never "delivered" — what
  proves delivery is reading the sensor back out of a status report (§8 item
  93) — and it is budgeted as one device operation all the same. An errno
  meaning the node is
  gone raises `DeviceUnavailable`, any other failed feature or output report
  `FeatureReportError`. The adapter only needs the `HidTransport`
  protocol, so tests run it against a fake controller.
- `hw/aquacomputer_adapter.py`: `AquacomputerAdapter(binding, clock=,
  sleep=, opener=)` is one controller with `read() -> PlantObservation`,
  `apply(MpcCommand)`, `release()` and `save()`. Nothing is opened at
  construction.
  - `read()` drains the input reports and uses the newest status report:
    `obs.temps` in °C (`None` where nothing is connected), `obs.rpm` from
    `fanN`, `obs.pwm` in `[0, 1]` from the status report's **output
    duty** on both kinds (what the device drives, not the cached
    command). A report counts as received when the queue is drained, so
    its age is known to one read (one tick). The kernel's hidraw queue
    holds 63 reports (`HIDRAW_BUFFER_SIZE` 64) and drops new ones while
    full: a drain that returns a full queue may be minutes old, so those
    reports are discarded (no status time, no duty evidence) and the queue
    is read once more. No status report for longer than
    `status_max_age_s` raises `DeviceUnavailable` (the loop's
    blank-observation fallback ramps the fans up). After an open, only
    the first `read()` waits up to `status_max_age_s` for a report; a
    device that stays silent then raises at once on every later read
    instead of blocking each tick. A vanished node closes the device and
    raises `DeviceUnavailable`; the next call discovers it again (a
    re-plug may bring a new `hidrawN`). With `serial:` configured, a
    first status report carrying another serial closes the device and
    raises, naming both; `apply()` then raises too, without opening the
    node, until a status report carries the configured serial (a device
    that is merely silent is not affected). A configured aquabus output or
    bound aquabus tachometer (aquaero 5–8) whose fan block reads rpm
    `0xFFFF` has no device behind it (nothing on the aquaero's aquabus):
    that channel's `rpm` and `pwm` are `None` in the observation,
    `absent_channels` lists it, one error is logged when that list changes
    (an info line when it empties) — once per state change, not once per
    tick — and such a slot is no evidence for duty verification. Everything
    else the controller reports comes through, its temperatures and its own
    outputs included. **Only a controller with no usable status report is
    unavailable** (a vanished node, the wrong serial, or nothing within
    `status_max_age_s`): faulting the whole device for an empty slot blinded
    its healthy half and kept the loop in fallback for as long as the slot
    stayed empty (§8 item 90). A fan the daemon cannot command is still safe
    through the plant: the zones it served lose cooling, their temperatures
    rise and the remaining fans ramp — the same path that covers a fan that
    has simply died. An adapter whose *every* commanded output is absent says
    so in that one error line and is still not unavailable, since its
    temperatures are exactly what makes the fans that are left ramp. The
    check uses the newest status report alone, with no confirmation over time
    (a single transient `0xFFFF` costs that channel one tick of `None`, not a
    fallback tick for the whole composite); the aquaero's
    own outputs 1–4 are never checked. **Every** aquabus block reading `0xFFFF`
    at once (and, on the aquaero, the aquabus flow slot reading "no data" too, §8
    item 130) says more: no device answers on that aquabus (§8 item 92). Then each
    logical name bound to an aquabus temperature slot `bus1..8` reads `None` from
    that first report — those slots keep serving the last value they read, which
    nothing in the report marks as old — while the rest of the observation is
    untouched; and after `bus_absent_s` of such reports the loss is reported (one
    error line, `device_health.aquabus`, a `problems` entry). Nothing here lowers
    a duty: the missing reading makes its zone untrusted, which holds and then
    raises. A bus device with no fan outputs can still be told from an empty bus
    on the aquaero, since its aquabus flow slot is a second, output-independent
    witness (§8 item 130); a kind with neither that witness nor a bound aquabus
    output still has binding a `busN` refused as a config error (Track B below).
    `ts` comes from the
    injected monotonic clock. `last_status` keeps the newest decoded
    report for tools and diagnostics; voltage, current and power are not in
    the observation (§8 item 79).
  - **Control report cache.** `apply()` opens the node without waiting
    for a status report, so the fallback ramp and the stop write reach a
    device whose status reports stopped. The control report is read (GET)
    once after opening and again after every invalidation, never on a
    normal tick. `apply()` needs a finite value in `[0, 1]` for every
    configured channel (else `ValueError`, nothing sent; no silent
    clamping) and commands `round(pwm × 10000)`. A write patches every
    channel with a pending change into the cached report and sends
    **one** SET, without the save report: it takes effect at once and is
    not stored in the controller's memory (§8 items 84, 86). Before every
    GET, SET and save report the adapter waits `ctrl_gap_ms` after the end
    of the previous control operation, failed ones included, as the Linux
    driver does (Linux commit 56b930dc added its 200 ms after seeing
    `EPIPE`; the aquaero's 100 ms were measured with the save report after
    every SET, §8 item 87). A failed operation invalidates the cache and is
    retried from a fresh GET up to `ctrl_retries` times (at most 5), then
    raises `DeviceUnavailable`. No control operation starts once
    `ctrl_budget_s` of this `apply()` is spent (`DeviceUnavailable`): every
    usbhid control transfer can block for its 5 s timeout. The cache counts
    as written only when the SET succeeded; after a failed SET, or a budget
    spent before it, the next write sends every configured channel (a
    failed SET may or may not have reached the device). A Quadro control
    report whose checksum does not match is a failed read. An aquabus output
    with no device behind it is written with the others and `apply()` does
    not raise for it (`read()` reports that channel as `None` and lists it in
    `absent_channels`, §8 item 90): the loop rate limits the fallback ramp
    against the last command whose `apply()` succeeded, so an `apply()` that
    raised after its SET went out would keep every fan of the composite below
    `fallback_pwm` while the slot stays empty (and would hold back `READY=1`
    and report the stop write as failed).
  - **Software-sensor heartbeat** (§8 items 33, 84), off by default
    (`heartbeat_sensor: 0`). With a sensor configured, every `apply()`
    writes `heartbeat_value_c` into that aquaero software sensor with output
    report `0x07` — once per tick, **after** the duty work and only when it
    succeeded, so once per `apply()` however often the control write is
    retried. Only that sensor is written; the other seven slots carry
    `0x7FFF`, so the device keeps its own values for them. It is the daemon's
    half of the controller's own watchdog: the sensor is enabled on the device
    with a timeout and a high fallback temperature, and an alarm on it selects
    a safe profile, so a daemon or a Pi that stops writing lets the controller
    take over (§2 "Software-sensor heartbeat and profiles": 30 s, then every
    output at 100 %).
    **The order is the point.** The failure mode only this watchdog covers is
    a daemon that is alive but cannot write — a live node whose every control
    operation fails, where the loop logs "apply failed" and keeps ticking. A
    heartbeat sent before the duties would hold the watchdog shut through
    exactly that, leaving the fans at the last duty that did go out. So it
    goes out behind an `apply()` that reached the device; a tick that did not
    sends none and the controller falls back on its own. An `apply()` with
    nothing to send (no duty changed) still counts as commanding — the cadence
    is one heartbeat per tick, not one per SET.
    The heartbeat is sequenced like any other control operation — it waits
    `ctrl_gap_ms` and runs inside `ctrl_budget_s` — so the worst case per tick
    is unchanged; running last, it is the operation a spent budget drops.
    A failed heartbeat never fails the tick: it is logged once per state
    change (error when it starts failing, info when it goes out again), and
    what it costs is exactly the fallback the controller is configured for,
    which is the safe direction. Nothing is written to a device whose serial
    does not match, the heartbeat included. Only the aquaero has a known
    software-sensor report, so `heartbeat_sensor` on a `quadro` entry is a
    startup `ConfigError`. `heartbeat_on` / `heartbeat_ok` expose the state
    (publishing it is §8 item 83); `heartbeat_ok: true` means the report was
    accepted for sending, not that the controller saw it (`write_report`
    queues an output report, §2), so reading the sensor back from a status
    report is what would prove delivery (§8 item 93).
    **`heartbeat_sensor: 0` is right only while no software sensor is enabled
    on the controller.** A sensor enabled there with a timeout and an alarm
    that nothing writes falls back one timeout after boot and the alarm
    selects its profile for good: the watchdog has fired and can never fire
    again, and live writes keep working, so nothing in the journal says so.
    Both example files show the key at its default, with that written next to
    it; on the owner's aquaero (`soft1` enabled since 2026-09-15) the value to
    run is `1`, after item 93.
  - **Profile changes** (§8 item 84). Byte `0x06` of the aquaero's control
    report is the profile it runs (`active_profile`, 1-based). A switch — the
    alarm above, or the controller's own panel — reloads the **saved**
    profile, so every duty the daemon wrote live is gone. Every fresh control
    report is therefore compared: a changed byte logs one line naming both
    profiles and makes the next write send every configured channel. **No
    control read is added per tick**: the report is the one the periodic
    `ctrl_refresh_s` read or an invalidation already fetches. So a switch is
    noticed within `duty_mismatch_s` plus a tick (about 5–10 s at the
    defaults) whenever the reloaded profile drives an output differently than
    the daemon last wrote — the duty verification sees that and re-reads the
    report — and otherwise at the latest after `ctrl_refresh_s` (60 s).
    **Both edges matter, and byte `0x06` bounds neither.** On the *alarm-set*
    edge the reloaded profile is the safe one (on the owner's controller every
    output at 100 %), so a late rewrite only postpones the daemon taking the
    fans back down. On the *alarm-clear* edge — the one a daemon restart after
    an alarm goes through: its first heartbeat clears the alarm, the controller
    reloads saved profile 1 and every output drops to what that profile holds
    (20 % on the owner's controller) while the daemon still believes it
    commands what it last wrote. That direction **reduces cooling**, and what
    bounds it is the duty verification, not the profile byte: the reloaded duty
    differs from the written one, so within `duty_mismatch_s` plus a tick
    (about 5–10 s) the cache is invalidated, the report re-read, the profile
    change logged and every configured channel written again. The
    `ctrl_refresh_s` worst case is reached only when the reloaded profile
    happens to drive the outputs exactly as the daemon last wrote them — when
    there is nothing to lose by waiting. In between, `obs.pwm` still reports
    what the outputs really drive (it comes from the status report), so the
    solver and the gate see the drop; only `applied_cmd` names the duty the
    daemon wrote.
  - **Write limiting**, off by default. It was introduced against memory
    wear while every SET was followed by the save report and so stored in
    the controller's non-volatile memory (§8 item 77). A SET without the
    save report changes nothing that survives a power cycle (§8 item 84;
    whether it still writes the memory internally cannot be observed), so
    both keys default to 0 and every change is written (§8 item 86). The
    keys stay to limit USB traffic (one 2707-byte aquaero SET per changed
    tick). A channel's duty is written at once when it **rises** above the
    duty the device holds, or when the channel does not follow a duty at
    all (an aquaero channel on a firmware controller): cooling never waits.
    A **fall** is written only when it is at least `write_deadband` below
    the written duty **and** at least `write_min_interval_s` has passed
    since this device's last write. Every write carries every channel with
    a pending change, rises and pending falls alike. A deferred fall is not
    an error: `obs.pwm` keeps coming from the status report, so the
    controller sees the duty the fans actually get.
  - **No slower fan during a fault.** A deferred fall is recorded by the
    loop as applied, so the fallback hold and ramp start from the lower
    command while the device still holds the higher duty. With
    `cmd.mode` FALLBACK or DEGRADED the adapter therefore never sends a
    channel below the duty the device holds: `max(command, held)`, where
    held is the duty in the cached control report (read first when the
    cache is invalid; a failed read takes the failure path as usual) and,
    for an aquaero channel on a firmware controller, the newest status
    report's output duty since the open, or 100 % before any report. It
    applies to every write in those modes, forced rewrites included, so a
    pending fall can neither ride along with another channel's rise nor
    mature after `write_min_interval_s` during a fault. The stop write of
    `fallback_pwm` is a FALLBACK command, so it never lowers a fan either.
    In AUTO and SATURATED mode falls are written as the rule above says.
  - **Keeping the cache honest.** Speed, output duty, voltage, current and
    power arrive in every status report, so drift of the fans themselves
    is visible without any control read. A one-time read misses a
    configuration changed behind the daemon's back (front panel,
    aquasuite, liquidctl, a controller reset), hence three rules:
    (a) *duty verification*, per channel — the reference is the duty
    written to (or read back from) the device, not the raw command. A
    channel's mismatch timer starts at the first status report received
    after that channel's own last change that differs by more than
    `duty_mismatch_tolerance`; it is cleared only by an agreeing report or
    a write that changes that channel's duty (writes to other channels do
    not restart it), and fires when it has lasted longer than
    `duty_mismatch_s`: the cache is invalidated, a warning names device,
    channel, written and reported duty, and the next `apply()` reads the
    report again and rewrites every configured channel. A channel that
    mismatches again after that rewrite is logged **once** as an error,
    listed in `stuck_channels`, and not rewritten for the mismatch again
    until the device reports its duty (writes on a changed command
    continue) — no rewrite loop, no warning every few seconds (§8 item 81;
    publishing it is item 83). The aquaero output mode (block +0x0E: PWM
    or DC voltage) is only reported: one warning per open for every
    commanded output of the aquaero's own (1–4) not in PWM mode and none
    for an aquabus output (5–8, mode word not interpreted); the adapter
    never writes the mode. A commanded channel whose controller block has
    **no control source** (`0xFFFF`) is left out of the write instead of
    being written blind, because nothing on the device drives that output
    and no capture shows that writing the block the way a configured one is
    written would change that (§8 item 89). The refusal is **per channel**:
    `apply()` writes every other channel of that controller in the same SET
    and sends the software-sensor heartbeat as usual, and `read()` is
    unaffected (it never adopts a control report), so one unconfigured block
    cannot take a whole controller — or the heartbeat that keeps the
    aquaero's watchdog quiet — down with it. The channel is logged once per
    open as an error, and stays in `unconfigured_channels` and in the
    `device_health` problem list, telling the owner to give the output a
    source in the controller's own software or drop it from the config, for
    as long as the block reads that way. A stuck channel's error
    carries the adapter's `stuck_hint` when it returns one
    (`CompositeSource` sets it on a Quadro, below);
    (b) *power cycles* — a change of the Quadro's power-cycle count from
    the value seen at open invalidates the same way (logged);
    (c) *periodic refresh* — every `ctrl_refresh_s` (0 disables) `apply()`
    reads the control report again first; a channel that no longer holds
    its duty is logged and written again, other changes in the report are
    adopted.
  - `release()` restores, for every channel this adapter has written, the
    fields captured by the first GET after the daemon started (aquaero:
    preset, control source, minimum and maximum power; Quadro: duty) with
    one live SET, not saved; a no-op when nothing was written. Not called
    at exit (§2; §8 items 33, 76). A power cycle of the controller restores
    its saved configuration anyway.
  - `save()` sends the save report once (gap and budget as for any control
    operation, not retried; a failure raises `DeviceUnavailable`): the
    controller stores the configuration it holds at that moment, which it
    then comes back with after a power cycle. The daemon never calls it. It
    is the commissioning step for the saved safe configuration of §8 item
    84 (set that configuration, then save once); nothing calls it yet (§8
    item 88). Verified to persist on the aquaero; on the Quadro the report
    is only known to match the Farbwerk 360's save report.
  - **Worst case per tick.** One `read()` plus one `apply()` of a device
    can block for `status_max_age_s` (the first read after an open) +
    `ctrl_budget_s` + one 5 s control transfer that started just before the
    budget ran out + `ctrl_gap_ms` (the retry after that failure sleeps the
    gap before it finds the budget spent): 13.1 s for the aquaero and 13 s
    for the Quadro at their defaults. Write limiting does not enter the
    bound; with it off, a tick whose duties changed costs one SET per
    controller (§2 "hidraw check": 9–13 ms per `apply()` measured with the
    save report after the SET; item 80). `WATCHDOG=1` goes out at the end of a
    tick and the loop then sleeps until the next one, so two pings can be
    `mpc.dt` + `mpc.budget_alarm_ms` (the step time bound the config
    states; a slower step is logged, not interrupted) + the sum over all
    devices apart: 5 + 0.75 + 13.1 ≈ 19 s for `config.example-das.yaml`
    (the Quadro on the aquaero's aquabus), 5 + 0.75 + 26.1 ≈ 32 s with the
    Quadro on its own USB port as a second device.
    `build_io` reads the systemd watchdog period from `$WATCHDOG_USEC` and
    refuses (exit 2) a configuration whose bound is not below it (§9).
- **The device entry**, `xt6:` or one `aquacomputer:` list entry
  (`parse_device_section`; `build_adapter_from_config(xt6,
  channels=mpc.channels, temps=mpc.temps)` for `--source xt6`):

  ```yaml
  xt6:
    device: aquaero               # required: aquaero | quadro
    serial: "12345-54321"         # optional; required when several of one kind are attached
    fans:
      radiator: {pwm: pwm1, rpm: fan1}
      intake:   {pwm: pwm2}       # rpm optional
      rear:     {pwm: pwm5, rpm: fan5}  # aquaero 5-8: a Quadro on its aquabus
    temp_map:
      coolant: temp1              # physical sensor
      rear_air: bus2              # the Quadro's sensor 2 on aquabus
    status_max_age_s: 3.0         # optional timing keys, defaults shown
    ctrl_gap_ms: 100              # per kind: aquaero 100, quadro 0
    ctrl_retries: 1
    ctrl_budget_s: 5.0
    ctrl_refresh_s: 60.0
    duty_mismatch_tolerance: 100
    duty_mismatch_s: 5.0
    write_min_interval_s: 0.0
    write_deadband: 0
    bus_absent_s: 10.0            # how long every aquabus block must read "no device"
                                  # before the bus device is reported lost (the report
                                  # only: a bound busN reads as missing at once)
    heartbeat_sensor: 0           # softN written on every tick that commanded the duties;
                                  # 0 = off, and right only while no softN is enabled there
    heartbeat_value_c: 20.0
  ```

  | Key | Default | Valid | Meaning |
  |---|---|---|---|
  | `status_max_age_s` | 3.0 | finite, > 0 | a newest status report older than this is no observation (`DeviceUnavailable`); an open waits this long for the first report |
  | `ctrl_gap_ms` | aquaero 100, Quadro 0 | finite, ≥ 0 | wait after any control operation (failed ones included) before the next GET or SET, ms |
  | `ctrl_retries` | 1 | integer 0..5 | retries of a failed control operation, each from a fresh GET |
  | `ctrl_budget_s` | 5.0 | finite, > 0 | no control operation starts once this much of one `apply()` is spent |
  | `ctrl_refresh_s` | 60.0 | finite, ≥ 0 | periodic control report read in `apply()`; 0 disables |
  | `duty_mismatch_tolerance` | 100 | integer 0..10000 | 1/100 %: a status duty further from the written duty is a mismatch |
  | `duty_mismatch_s` | 5.0 | finite, > 0 | a mismatch lasting longer re-reads the report and rewrites every channel |
  | `write_min_interval_s` | 0.0 | finite, ≥ 0 | a falling duty is written at most this long after the device's last write (rises at once); 0 writes every fall; only limits USB traffic since writes are not saved (§8 item 86) |
  | `write_deadband` | 0 | integer 0..10000 | 1/100 %: a falling duty is written only this far below the written one; 0 writes every fall (§8 item 86) |
  | `bus_absent_s` | 10.0 | finite, > 0 | how long every aquabus fan block must read "no device" (rpm `0xFFFF`), over received reports, before the controller reports the device on its aquabus as lost (§8 item 92). It bounds the *report* only — a bound `busN` reads as missing from the first such report — and what it buys is the blip: a device re-enumerating shows one or two reports of `0xFFFF` (§8 item 90). Nothing about the aquabus floors it (§8 item 115, as corrected 2026-09-18): an aquabus block's electrical fields are one sample taken inside the PWM cycle and read 0 mA at a low duty with the fan turning, while presence is read from the speed field every report carries, so no report can look like a departure at any window |
  | `heartbeat_sensor` | 0 (off) | integer 0..8, aquaero only | the software sensor `softN` every `apply()` writes, after its duty work and only when that succeeded, as the heartbeat of the controller's own watchdog; 0 only while no software sensor is enabled on the device (§8 item 84) |
  | `heartbeat_value_c` | 20.0 | finite, −327.68..327.66 °C | the temperature the heartbeat writes; it must stay below the alarm the controller has on that sensor |

  The defaults live once, in `AquacomputerTiming`, and the one that
  depends on the device kind (`ctrl_gap_ms`, owner decision 2026-09-15)
  in `KIND_TIMING_DEFAULTS` next to it; both example configs show every
  key at its default for their device (`tests/test_model_config.py`
  checks that). What only the kind can bound — a `heartbeat_sensor` above
  the kind's software sensors, or any on a Quadro, whose software-sensor
  report is unknown — is checked with the kind, in `check_kind`, both when
  the config is parsed and when a `DeviceBinding` is built in code. Each fan is **one** `fans` entry that names the channel once and
  carries both inputs, so a PWM output and its tachometer cannot drift
  apart into two differently spelt channels. Only `pwm` and `rpm` are
  allowed inside an entry (a typo such as `rmp:` is rejected); `pwm` must
  be one of the kind's outputs (aquaero `pwm1..8`, 5–8 a Quadro on its
  aquabus; Quadro `pwm1..4`), `rpm` one of its tachometers (aquaero
  `fan1..8`, Quadro `fan1..4`), `temp_map` values one of its temperature
  inputs (aquaero `temp1..8`, `bus1..8`, `virt1..4`; Quadro `temp1..4`),
  and two names on one input are rejected. A `softN` **software** sensor is
  not on that list and cannot be bound anywhere: it holds what a host last
  wrote and its configured fallback for ever after, which no status report
  tells apart from a measurement, so both `parse_device_section` and
  `DeviceBinding.__post_init__` refuse it — see "Fan and device health"
  below and §8 item 113 for what is published instead. A `busN` **may** be
  bound since §8 item 92. While no device answers on that aquabus the
  slot reads as missing instead of as the value the controller keeps
  serving for it, so a frozen temperature can no longer reach the solver
  as an ordinary reading — and presence needs evidence the config model
  can find without a live report: on the aquaero, its aquabus flow slot
  (§8 item 130), confirmed to carry data whether or not the bus device
  behind it has any fan output at all, so a sensor-only slave (a Farbwerk
  360) or a Quadro on aquabus whose fans this daemon does not command is
  in scope the same as one whose outputs are bound. A kind with **no**
  such witness still has the config model enforce the older condition —
  the same entry must also bind one of that device's aquabus outputs
  (`pwm5..8`, or a `fanN` of one of them), or the entry is rejected, since
  presence would otherwise be judged from the aquabus *fan* blocks alone,
  which cannot tell a bus device with no fan outputs from an empty bus.
  There is no such kind today (the aquaero is the only one with aquabus
  temperature slots, and it always has the flow witness). A
  name of the hwmon driver's numbering that means another input now is
  rejected with the new name: aquaero `temp17..20` (`virt1..4`) and the
  Quadro's flow sensor `fan5`, which is not a tachometer. The hwmon names
  of the software sensors — aquaero `temp9..16`, Quadro `temp5..20` — are
  rejected saying which `softN` they used to mean **and** that the modern
  name is refused too, since renaming them would only move the same
  unbindable input. Any tachometer
  may be bound to any output; the aquaero's hwmon `fan5`/`fan6` were its
  flow sensors and are its aquabus tachometers now, so an old config that
  bound flow as `rpm` fails its reads with "no device behind fan5" unless a
  Quadro is on aquabus. Flow sensors (`flowN`) cannot be bound anywhere in
  the config — not as `pwm`, not as `rpm`, not in `temp_map` — and naming
  one is rejected with a message saying so and that an aquaero's hwmon
  `fan5`/`fan6` mean aquabus tachometers here (owner decision 2026-09-16,
  §8.1; §8 item 91). Flow is still decoded, shown by
  `tools/aquacomputer_probe.py` and published with the device health
  (`device_health.devices[].flows`), just never in `PlantObservation`.
  Unknown keys are rejected too, so a misspelt timing key cannot fall
  back to its default silently; `xt6.prefer` is accepted and ignored.
  Keys of the hwmon era are rejected with a hint: `hwmon_name` (use
  `device:`), `root` (devices are discovered by USB id; use `serial:`),
  `name` (renamed to `device`), and the older `map` / `fan_map` (use
  `fans`). For `xt6:` the `fans` keys must **equal** `mpc.channels` and
  the `temp_map` keys must **equal** `mpc.temps`, or the daemon exits
  with a `ConfigError` (code 2) before the loop starts. Without that
  check a channel missing from the map is silently never written, and a
  temperature missing from or extra in `temp_map` keeps the gate in
  permanent fallback with no visible error. `--source xt6` is this single
  device.
- `hw/sources.py` (`--source composite`, the DAS source):
  `CompositeSource` merges any number of `AquacomputerAdapter`s and an
  optional 1-Wire source into one `PlantObservation` and puts the SMART
  inbox's snapshot into `obs.inputs["smart"]`; `apply()` hands the whole
  command to every device, each adapter commanding its own channels with
  at most one SET. Both visit **every** device even after one raised, so
  every hidraw queue is drained and the fallback ramp and stop write reach
  every healthy controller; the failures are then raised as one
  `DeviceUnavailable` naming each failed device, chained to the first
  (the observation of that tick is still lost).
  `build_composite_from_config(aquacomputer_section=, xt6_section=, onewire_section=, channels=, temps=, dt=, smart=)`:

  ```yaml
  aquacomputer:               # a list, one entry per controller
    - device: aquaero
      fans: {xt1: {pwm: pwm1, rpm: fan1}, ...}
      temp_map: {inlet_a: temp1, air_z0: temp2, ...}
      # the Quadro on the aquaero's aquabus: its outputs are this entry's
      # pwm5..pwm8 / fan5..fan8, e.g. qd1: {pwm: pwm5, rpm: fan5}
    - device: quadro          # or: the Quadro on its own USB port
      fans: {qd1: {pwm: pwm1, rpm: fan1}, ...}
      temp_map: {}            # bind Quadro inputs once they are connected
  onewire:
    sensors: {prox_b01: 28-0316a27a0aff, ...}   # logical name -> ROM id
    resolution_bits: 12       # 9..12
    max_age_s: 7.5            # default 1.5 * dt
    root: /sys/bus/w1/devices # default
  ```

  An `xt6:` section is still accepted as one more device. Every
  `mpc.temps` name must be bound exactly once across all `temp_map`s and
  `onewire.sensors`, every channel exactly once across all `fans` maps,
  and two entries of one kind need distinct serials (both would otherwise
  open and command the same controller), or the daemon exits 2 before the
  loop starts. So does a config that commands aquaero outputs `pwm5..pwm8`
  together with a quadro entry that commands outputs and has no `serial:`:
  that entry opens whichever Quadro is attached, possibly the one on
  aquabus, which ignores writes over its USB (a quadro entry with
  `fans: {}` that only reads sensors is allowed). Which Quadro sits on
  aquabus cannot be read before the devices are opened, so a commanding
  quadro entry with a serial is refused too, as a second, physically
  distinct Quadro on its own USB port, i.e. a second, independent
  controller (item 106). At runtime a Quadro
  channel found stuck, next to an aquaero whose status report shows a device
  on its aquabus, is logged with that explanation: the Quadro is probably
  on aquabus and must be commanded through the aquaero. A config that
  still has a `hwmon:` section exits 2 with a message naming the rename to
  `aquacomputer:` and `device:`, and so does one whose summed worst case
  per tick is not below the systemd watchdog (`watchdog_s=`, see above). A
  ROM id missing from every bus at start is a warning, not fatal: a sensor may
  be unplugged with its drive.
- **Fan and device health** (`health.py`, items 79 and 83). Every status
  report already carries, per output, rpm, the output duty the device
  drives, the 12 V rail voltage, current and power. Two paths carry them
  out of `hw/`, chosen so neither can change what the solver sees:

  - the measurements ride the observation.
    `AquacomputerAdapter.fan_readings()` returns
    `{channel: {device, output, tach, rpm, duty, voltage_v, current_ma,
    power_w, power_reported, rail_reported, not_measured, aquabus}}` —
    `not_measured` is `{rule: why this output's reading cannot feed it}`,
    written where the hardware knows the reason and carried to the health
    rules so a verdict can name the rules it did not run (item 117);
    `current_ma` and
    `power_w` are `None`, not a number, wherever `power_reported` is false,
    and `voltage_v` is `None` wherever `rail_reported` is false (the
    aquaero's aquabus outputs, whose voltage field holds the aquaero's own
    rail in every report that does not carry the bus device's electrical
    sample — most of them at a low duty), so a placeholder can never be read
    as a measurement (§8 item 89); `rpm` from
    the channel's *bound*
    tachometer (`fans.<ch>.rpm`, named in `tach`), the one `obs.rpm` and a
    `tools/fit_fans.py` fit describe, which the config may deliberately put
    on another block than the output; the electrical fields from the
    output's own block — and `read()` puts it in
    `PlantObservation.inputs["fans"]`, which `CompositeSource` merges
    across controllers. `inputs` is exogenous non-gated data: it never
    enters the sensor gate, never faults anything and never reaches
    `mpc.step`'s arithmetic, so **no golden changes**. The recorder writes
    them as the record's `fans` key (a new key of schema version 1: every
    reader takes fields by name with a default, and an older recording, or
    a source with no such readings, simply has `{}`).
  - the controller's own state does not, because a missing aquabus device
    makes `read()` raise and the tick that most needs the diagnosis would
    carry nothing. `AquacomputerAdapter.device_health()` /
    `CompositeSource.device_health()` answer from the last status report
    and the cached control report even while the device is gone: per
    controller `stuck_channels`, `absent_channels`, `not_pwm_channels`
    (commanded own outputs in DC or an unknown mode),
    `unconfigured_channels`, the flow sensors `flowN`, `aquabus` (§8 item 92:
    `state` — `unknown`, `present`, `empty`, `lost` or `never_seen`, the one
    field a single line can show — plus `present`, `seen`, `absent_s`, `lost`
    (a device that answered and then stopped, never an empty bus nothing was
    ever on) and `temps_missing`), serial, firmware,
    power cycles, status age, and `active_profile` when a later change
    publishes one (read defensively; absent until then). `problems` is the
    human-readable list, empty exactly when nothing is wrong. Two more
    complete item 83's list: `heartbeat` (`on`, `ok`, `sensor`, `value_c` —
    the software-sensor heartbeat of item 84, which was on the adapter for
    diagnostics only until now) and `software_sensors`, one entry per
    `softN` slot of the controller with `enabled`, `fallback_c`,
    `timeout_s`, `written_by_daemon`, `reading_c` and `reads_fallback`
    (item 113, below). The settings come from the control report and are
    **kept once decoded**: `null` means no control report has been read
    yet, `[]` means a device with no software sensors this daemon knows
    how to read (the Quadro), and neither the invalidation of the cached
    report after a duty mismatch nor a close takes the slots away — they
    are the operator's configuration of the controller, and publishing
    them only while a report happens to be cached would make them, and
    the disabled-`heartbeat_sensor` problem that reads them, flap.

  `health.HealthMonitor` is a `Loop.on_tick` observer (like the recorder
  and the MQTT publisher): it never raises, only reads, and never changes a
  duty. It applies three sustained rules and hands the result to
  `Supervisor.set_device_health`, whence `/api/state`, `/api/health`, the
  MQTT state blob and the page show it:

  - **rpm against the fitted curve.** Expected speed is
    `rpm_max * phi(duty, deadband, exponent)` from the channel's
    `mpc.fan_models` entry — the curve `tools/fit_fans.py` fits from a
    recording, so a fit goes straight into `fan_models`. Always that entry,
    never the online fit of item 14, however good it is (item 107): this
    rule judges a fan against its commissioned figures, and a curve fitted
    online to the very tachometer readings being judged would follow a fan
    that slows down, so the deviation would never show. The speed judged
    is the channel's *bound* tachometer (`fans.<ch>.rpm`), which the config
    may deliberately put on another block than the output, and which is the
    one the curve was fitted from. A deviation is a speed further than
    `rpm_tolerance_frac * rpm_max` outside the duty band below, held
    `rpm_fault_s`.
  - **the 12 V rail.** Outside `[rail_min_v, rail_max_v]` for
    `rail_fault_s`. A block reading 0.0 V is not judged: that is an
    aquaero's empty aquabus slot — an aquabus output with nothing connected,
    on a bus device that is present, reads 0.00 V in the reports that carry
    that device's own measurements (§2, 2026-09-17) — not a dead rail. The
    rule is off altogether for an aquaero's **aquabus** outputs, whose
    voltage field is the aquaero's own rail in about three reports out of
    four and is therefore published as unknown (§2, 2026-09-17): a rail
    sagging behind a bus device is not detected, and judging the substituted
    value would have been worse than not judging at all. **That gap is
    accepted and declared** (item 117): closing it needs a controller that
    reports its own outputs' rails to this daemon — a Quadro on its own
    USB — which the supported topology does not have (owner decision
    2026-09-16), and the aquaero's report carries no substitute signal,
    because the refresh is not even atomic per report (one capture had
    blocks 5 and 6 refreshed and block 8 not), so nothing says which
    reading a block is carrying. What the daemon does instead is say so:
    every channel's verdict carries `rpm_monitored`, `rail_monitored` and
    `power_monitored` with an `unmonitored` mapping of rule → reason, so an
    empty `problems` is never readable as coverage the daemon does not
    have. **All three rules declare themselves, and a flag means the rule
    ran** — not that the device measures the field: a rule needs the
    measurement *and* the configuration it is judged against, so an output
    that reports power with no `power_w_at_max` behind it, a channel with
    no fitted curve, and a rail of exactly 0.00 V (which this rule refuses
    to interpret, below) are all `*_monitored` false with the reason. What
    the flags do not carry is the two transient conditions of the
    duty-dependent rules — the `settle_s` window still filling after a gap,
    and a duty below `min_duty` — which hold a rule off for a tick or two
    rather than for the channel. What stays
    covered is the aquaero's own outputs 1–4, and with them any sag common
    to the whole 12 V supply; what is not is a rail local to the bus
    device. This rule does not
    depend on the duty, so it is judged at every duty and a duty move never
    restarts it: a rail that sags while the solver modulates is exactly the
    case worth catching.
  - **power against the duty.** Only where the device reports a current a
    single report may be judged on — which no supported output does (§2,
    2026-09-18). An aquaero reports 0 mA and 0 W for its *own* outputs 1–4
    in PWM mode however fast the fan turns, and a bus device's outputs
    (its aquabus blocks 5–8, and a Quadro's own four) sample the current
    inside the PWM cycle, so at a low duty most reports read 0 mA with the
    fan turning. Absence of current is therefore no fault anywhere, and
    `power_reported` says so per output. Expected power is
    `count * power_w_at_max * phi(duty) ** power_exponent` over the same
    band; `fan_models.<m>.power_w_at_max` has no default (the figure
    depends on the fan), so without it this rule is simply off for that
    model — and it is unset in both example configs until item 94's
    measurement, so a measured power is **not** coverage on its own:
    `power_monitored` is true only when the device reports the power *and*
    the channel's model says what to expect, and the missing key is named
    in `unmonitored` otherwise.

  Nothing is judged below `min_duty` (inside and just above the deadband
  the curve says little). An aquabus fan's rpm in the aquaero's status
  report lags the Quadro's own report by several seconds, so the two
  duty-dependent rules judge a reading not against the duty of its own tick
  but against the **band** the output duty spanned over the last
  `settle_s` — `[min duty, max duty]` of that window mapped through the fan
  curve. A step widens the band while the tachometer catches up and it
  narrows back to a point after `settle_s` of steady duty; a duty that
  keeps moving is judged against a wider band rather than never judged at
  all. Nothing is judged before `settle_s` of live readings has
  accumulated, and every window of a channel starts again whenever it
  misses a tick (the read raised, the aquabus slot went away): wall time
  that passed while nothing was measured is not evidence of a deviation.
  Repeated problems are logged at most once
  per channel per `log_interval_s`. Every threshold is a `fan_health:` key
  with one default declared once in `health.FanHealthConfig`, validated
  there (an unknown key or a bad value is a `ConfigError`, exit 2, checked
  before anything is opened) and shown in both example configs.

  **A `softN` software sensor is not a reading, and cannot be bound**
  (item 113). The aquaero's eight software temperature slots hold whatever
  a host last wrote into them with output report `0x07`, and, once that
  host has been quiet for the slot's configured timeout, its configured
  fallback — for ever, as a steady number that never goes stale and never
  reads `0x7FFF`. A *disabled* slot is the only one a status report marks.
  So a `softN` name in `temp_map` is refused, on both kinds, in
  `parse_device_section` and again in `DeviceBinding` itself, and nothing
  reaches the estimator, the recorder or a health rule through it. The
  daemon writes at most one software sensor itself (`heartbeat_sensor`),
  and that one carries `heartbeat_value_c`, its own constant, so it is
  refused too. What a human gets instead is the slots themselves:
  `aquacomputer.software_sensor_settings()` decodes the five bytes per
  sensor the control report holds from `0x177` — enabled, fallback
  temperature, timeout — and `device_health.devices[].software_sensors` publishes
  each slot with its reading and `reads_fallback`, which is true exactly
  when the reading *is* the configured fallback (an exact centi-degC
  comparison: both sides are the same field, so no tolerance and no config
  key). `tools/aquacomputer_probe.py` prints the same three numbers under
  the status report's `softN` values. One `problems` line comes out of
  this: a `heartbeat_sensor` the controller has **disabled**, which the
  daemon cannot see any other way — it writes the slot every tick, the
  write succeeds, and the sensor the alarm watches stays `0x7FFF`, so the
  watchdog of item 84 cannot fire. An enabled slot nothing feeds is not a
  problem line: it is a fact about the controller's configuration, and on
  the owner's board seven of eight are in that state.

  **The controller block's undecoded words** (§2, "The controller's own start
  boost"). Four of the ten `u16` of an aquaero controller block are decoded —
  minimum power, maximum power, the mode word and the control source — and six
  are not: `+0x00`, `+0x02`, `+0x08`, `+0x0A`, `+0x0C`, `+0x12`
  (`aquacomputer.CONTROL_BLOCK_UNDECODED`). They are read raw into
  `ChannelState.undecoded` and printed by `tools/aquacomputer_probe.py` under
  each output, exactly like the fan block's `+0x0A`: no name, no unit,
  published by nothing, judged by nothing and written by nothing — a duty write
  touches the preset, the source and the two power limits and nothing else in
  the block. The firmware's per-output **start boost** is one of the six and a
  read-only capture cannot say which: four of them (`+0x08`, `+0x0A`, `+0x0C`,
  `+0x12`) hold one value across all eight outputs, and the two that do differ
  between outputs (`+0x00`, `+0x02`) differ in no way a boost setting is known to
  explain. Printing them is what makes the capture that settles it a diff.

  **Absence is judged on the speed field and on nothing else** (items 90,
  116). An aquabus block whose electrical sample fell in the off phase of
  its output's PWM cycle looks exactly like an empty slot, so the rule that
  separates them matters: it is `rpm == 0xFFFF` (`FanStatus.present`), the
  one field that takes no part in the sampling. The voltage cannot do it in
  either direction — an aquabus output with no fan reads 0.00 V in a
  sampling report, which is byte for byte what a slot with no device on the
  bus reads, and the same present output reads 12.09 V one second later —
  so a rule keyed on it would call a present device absent in one report and
  present in the next. Nothing in the project keys on it, and no confirmation
  window is added either: the sentinel is invariant across the sampling, so a present
  bus device is never judged absent, and an absent one is judged absent on
  the first report that shows `0xFFFF` — how soon the aquaero shows it,
  and whether a bus hiccup can show it for one report only, no capture
  says (item 96). Pinned on the three captures in
  `tests/test_hw_aquacomputer.py`.
- **Spin-up kick** (`control/spinup.py`, item 75). A fan's *starting* duty is
  higher than its running duty — the owner's aquaero test fan stops at 13 %
  and starts again only at 25 % — so a channel can sit at a healthy duty on a
  healthy rail, with the duty read back matching the duty written, while the
  rotor stands still and the enclosure is silently uncooled there. Only the
  tachometer knows. The owner met exactly that after a restart (2026-09-17).
  The answer is a **supervised sequence, never a blind boost**:

  1. *Detect.* A channel whose commanded duty has stayed at or above its stall
     duty and whose tachometer has read at or below `min_rpm` for `confirm_s`
     of live, uninterrupted readings. A tick with no live reading (the read
     failed, the write failed, the aquabus slot went away) starts every window
     again — the same rule the health monitor follows.
  2. *Kick.* Raise that channel to its own `kick_duty` for `kick_s`. The kick
     is a **floor, never a level**: `compose` takes `max(what the solver or a
     human asked for, the floor)` and then applies the same `d_pwm_max` rate
     limit and the same `[pwm_min, pwm_max]` clamp as any other command. A
     solver that already wants more is untouched, so a kick can reduce cooling
     nowhere and `mpc.step` is not touched at all.
  3. *Verify.* Watch the tachometer for `verify_s`. Motion ends the sequence;
     no motion starts the next attempt, and after `max_attempts` the channel is
     a **failed fan**: a `problems` line naming it, and (with
     `failed_channel_floor`) a floor under every other channel of the zones it
     served, at the duty they carried when the failure was declared, so the
     zone cannot *lose* cooling because one of its fans died. The floor is a
     hold, not a computed compensation: how much more air the zone needs is
     something only the measured temperatures know, and they raise the
     siblings through the solver like any other heat. A failed channel gets one
     more sequence every `retry_s`, so a fan replaced while the daemon runs is
     picked up without a restart and a dead one costs at most `max_attempts`
     kicks per interval instead of for ever — and that retry runs *under* the
     floor, which keeps the level recorded at the **first** failure rather than
     being re-recorded, so repeated retries across a hot spell cannot ratchet
     the siblings upward.

     Three bounds keep a hold from outliving its reason. It is never recorded
     above `failed_channel_floor_max` (0.6), because a failure declared in the
     middle of a hot spell would otherwise pin the enclosure at that duty for as
     long as the fan stays dead, and above the cap the temperatures hold the
     siblings up on their own anyway. It is never carried by a channel whose own
     fan is declared dead — a rotor that does not turn moves no air at any duty,
     so holding it high is noise and nothing else. And it is lifted only by the
     tachometer: a *declared* failure is retracted after `clear_s` of live
     readings above `min_rpm`, never by one sample, because a dead rotor
     windmilled by the air of its siblings reads a handful of rpm — and never by
     a gap, which is not evidence of anything, least of all that a fan came
     back. The alarm stands through the ticks of a retry for the same reason:
     the daemon trying again is not the fan working again.
  4. *At start too.* Every commanded channel is under the rule from the first
     tick; the same confirmation window covers the seconds a healthy fan needs
     to spin up. Nothing is boosted on start without evidence.

  **The kick duty is per output, not one global number.** Measured on the
  owner's hardware 2026-09-18: the same fan model reads **174 rpm on an
  aquaero output and 255 rpm on an output behind the aquabus device at the
  same nominal 20 %** (244 against 311 at 25 %) — 32–47 % apart at exactly the
  duties where starting happens. The duty-to-rpm mapping belongs to the
  output, so one global kick would be wasteful noise on the stronger output
  and too weak on the other, and the one that failed to start is the weaker
  one, the aquaero's own. `spin_up.channels.<ch>.kick_duty` carries it, with
  `spin_up.kick_duty` as the documented fallback; `config.example-das.yaml`
  ships 0.5 for `xt1..xt4` and 0.35 for `qd1..qd4`. Those are duties chosen
  with margin over the measured start behaviour, **not a fitted curve** — two
  points per output fit nothing — and `tools/fit_fans.py` measuring each
  output's real start duty is what replaces them (item 75).

  **Three things that look alike must not be confused, and the config is what
  tells them apart**: an output with no fan on it
  (`spin_up.channels.<ch>.fan: false`), an output whose fan drives no
  tachometer (`tachometer: false`, and equally a channel with no `rpm:` in its
  `aquacomputer:` binding, which reports no speed at all), and a fan that
  should be turning and is not. The first two are **never kicked and never
  alarmed**; their verdict carries `monitored` false with the reason, exactly
  like the health rules that declare what they do not cover (item 117), so a
  channel nothing watches can never read as a fan found healthy. A channel
  with no fitted curve (`mpc.fans` / `mpc.fan_models`, which need
  `mpc.topology`) is off for the same reason: without a curve nothing can say
  what "implausibly low" means for that output. **A legacy config therefore
  gets no kick at all and its command path is bit-identical.**

  Where it sits: the tracker is the supervisor's — `record_tick` advances it
  on the tick's own facts (the composed command's duty, `obs.rpm`, whether the
  tick was live), `plan_tick` puts the floors in force into `TickPlan`, and
  `compose` applies them last, in every mode, because they only raise. That is
  also why a `fallback` command is no longer returned untouched while a floor
  is in force; with no floor (every ordinary tick) `compose` behaves exactly as
  before. Everything in `control/spinup.py` is pure and clock-free: the clock
  is `obs.ts`. The verdict is published through `health.HealthMonitor`
  (`fans.<ch>.spin_up`, and the failed fans in the payload's `problems`), so
  `/api/state`, `/api/health`, the MQTT state blob and the page all show it.
  Every threshold and timing is a `spin_up:` key with one default declared once
  in `control.spinup.SpinUpConfig`, validated there and — for the per-channel
  names and the reachability of the kick duty, which need `mpc` —
  in `validate_spin_up`, both at startup before anything is opened. In full, the
  section is `enabled` (false switches the whole rule off, and every channel's
  verdict says so), `stall_duty` (0.15), `kick_duty` (0.5), `kick_s` (30 s),
  `confirm_s` (30 s), `verify_s` (30 s), `max_attempts` (3), `min_rpm` (60),
  `retry_s` (900 s), `clear_s` (30 s), `failed_channel_floor` (true),
  `failed_channel_floor_max` (0.6), `log_interval_s` (300 s — one line per
  channel and transition at most that often, like `fan_health.log_interval_s`)
  and `channels`; per output, `channels.<ch>` carries `fan`, `tachometer`,
  `stall_duty` and `kick_duty`, the last two defaulting to the section's.
  `validate_spin_up` refuses a `kick_s` that does not cover the `d_pwm_max`
  ramp from `pwm_min` up to the largest kick duty plus one tick: a
  kick the fans can never reach does not fail quietly, it condemns the fan for a
  kick it never got. A floor that raises a channel a running identification
  experiment drives aborts that experiment (`spin_up:<channel>`), because the
  experiment plans its levels from the solver's demand and cannot see the floor
  `compose` applies after it.

  **Noise cost of a kick**, measured on `config.example-das.yaml` at the shipped
  defaults (`control/noise.py`'s energetic index, `noise.exponent: 5`, ten fans
  on eight outputs; the index is relative — `noise_db_at_max: 30` is a
  placeholder until item 94 measures the fans):

  | enclosure at | index | kick `xt1` (2 fans) to 0.50 | kick `xt3` (1 fan) to 0.50 | kick `qd1` (1 fan) to 0.35 |
  |---|---|---|---|---|
  | 0.20 (`pwm_min`) | −7.7 dB | 15.4 dB (**+23.1**) | 12.4 dB (**+20.1**) | 2.6 dB (**+10.3**) |
  | 0.30 | 7.3 dB | 15.9 dB (**+8.6**) | 13.5 dB (**+6.1**) | 8.2 dB (**+0.8**) |
  | 0.50 | 22.4 dB | 22.4 dB (**+0.0**) | 22.4 dB (**+0.0**) | 22.4 dB (**+0.0**) |

  The cost is paid for at most `kick_s` (30 s) per attempt and at most
  `max_attempts` (3) attempts per sequence, so the worst case is 90 s of an
  audible rise against a channel that is moving no air at all — and, once the
  fan is declared dead, one such sequence per `retry_s` (900 s) and no more,
  which is 10 % of the time at the shipped defaults. It is zero whenever the
  solver already commands more than the kick, which is the whole reason the kick
  is a floor. The quiet-enclosure figures are the expensive
  ones, and the aquabus outputs' lower kick duty is most of why: the same
  measurement that made the duty per output also made the kick cheaper on the
  outputs that need less of it.

- **The board itself** (`health.py`, item 103). The Raspberry Pi the daemon
  runs on is a **health signal, never a model input** (owner decision,
  §8.1, 2026-09-16): about a watt against the drives' tens of watts, and
  its reading is dominated by its own self-heating, which moves with CPU
  load. It is not a solver input, not a zone air sensor and not a model
  node. `hostinfo.read_throttled()` adds the board's throttling state to the
  host metrics, decoded into `under_voltage`, `freq_capped`, `throttled` and
  `soft_temp_limit`, each both `_now` and `_since_boot`, `null` when nothing
  reads it and never an exception. **Three sources, best first, because no
  single one is present everywhere** — measured on the owner's board, a
  Raspberry Pi Zero 2 W on kernel 6.18:

  1. the firmware driver's sysfs attribute
     `/sys/devices/platform/soc/soc:firmware/get_throttled` — the whole word
     for a file read. **Absent on that kernel**: the `soc:firmware` platform
     device is there and carries no such attribute, and nothing under `/sys`
     is named for throttling at all. It stays first for the kernels that do
     expose it, but it is not the normal case;
  2. `vcgencmd get_throttled` — the whole word, and on that kernel **the only
     source of it**. A process: 3.3 ms median, 3.8 ms p95 there. Rate limited
     by `hostinfo.ThrottledReader` to one run per
     `host_health.vcgencmd_interval_s`, with the last word served in between
     and tagged with its age;
  3. the `rpi_volt` hwmon device's `in0_lcrit_alarm` — the **under-voltage
     condition only**, 0.17 ms median / 0.23 ms p95, found by the device's
     `name` and never by its index, which is not stable across boots (on that
     board `hwmon0` is the `cpu_thermal` zone the temperature already comes
     from and `hwmon1` is `rpi_volt`, today).

  A source that reads less than the whole word **says so**: the conditions it
  could not see stay `null`, `unknown` names them and `partial` flags the
  reading, and the `now` / `since_boot` summaries are `null` rather than
  `false` while an unread condition could be the one in force. An unknown bit
  is never reported as an absent one, so a partial source can raise the
  throttling rule but never silence it. A board with none of the three
  degrades to `null` and warns about nothing.

  `health.HostHealth` then applies five
  rules on the same `on_tick` observer as the fans', and the verdict rides
  the same payload as `device_health.host` (§6, §7):

  - **the board is hot** — its temperature above `temp_limit_c` for
    `temp_fault_s`. Default 75 °C: the Pi caps its ARM clock around 80 °C
    and hard-throttles around 85, so the warning arrives before the board
    starts defending itself, and 75 is far above the 47.2 °C the owner's
    Zero 2 W reads at idle. Default 120 s: a build, an update or a log
    rotation heats the SoC for tens of seconds; two minutes above the limit
    is placement or airflow, not a burst of work.
  - **the board is throttling now** — any `_now` bit. Reported the tick it
    is seen, with no window and no config key: the firmware has already
    latched the condition, so a sustained rule would only delay a fact. The
    `_since_boot` half is published next to it and never warns on its own —
    an under-voltage during boot is history, not a live problem. It fires on
    whatever source the board has, the `rpi_volt` under-voltage bit on its
    own included, and the message says which source it was, how old the word
    is where it came from the cache, and which conditions that source could
    not read.
  - **the board diverges from the enclosure air** — its temperature further
    than `divergence_c` from the mean of the air reference for
    `divergence_fault_s`, **and only while the CPU is idle**
    (`load1 <= idle_load1_max`). Default 40 °C, a deliberately coarse
    backstop: an un-heatsinked Zero 2 W idles roughly 20–25 °C above the air
    around it — the owner's reads 47.2 °C at idle, which against room air is
    already a ~25 °C divergence on a perfectly healthy board — so anything
    near that figure would stand permanently tripped, and 40 clears the
    self-heating floor while still catching a board in hot exhaust or an air
    sensor that stopped tracking. Narrow it only from a *measured*
    board-vs-air delta with the board in its finished place. It is judged on
    the *absolute* difference, since air reading above the board is evidence
    too. Default 900 s: the enclosure's
    air moves in minutes, so a quarter of an hour of continuous idle
    divergence is not a transient. Default `idle_load1_max` 0.5: four
    cores, so below 0.5 the daemon's own tick is the only load and the
    SoC's self-heating is at its floor; above it the reading says more
    about the CPU than about the air. **This rule is a hint, not a
    verdict**: it says that either the air sensors or the board's placement
    deserve a look, never which of the two is wrong.
  - **the card is low on space** — free space on `host_health.disk_path`
    (default `"/"`) below `disk_free_min_gb` for `disk_free_fault_s` ("the
    disk nobody watches", §8 proposed items). `hostinfo` has always
    collected `disk_used_pct`/`disk_free_gb` and both were always
    published — an MQTT sensor, a row on the page — but nothing watched
    them: the card fills silently and the recorder, the model store and
    the journal all start failing at once. On the owner's board that card
    is a 15 GB SD (11 GB free today), and it is also the failure mode of a
    worn-out card. `record_path` and `--model-store` are not pinned to
    `/`, so an operator who puts either on a different mount must point
    `disk_path` there too, or this rule (and the read-only one) watches a
    filesystem nobody writes to while the one that matters fills or dies
    unwatched. The rule is meant to warn **while there is still room to
    act**, not once a write has already failed, so the default is argued
    from what actually writes to the default `disk_path` rather than a
    round number: the recorder's own worst case is its file plus every
    rotated backup, each up to `record_max_bytes` — 20 MB × 6 files =
    120 MB at the recorder's own defaults (item 79); the model store's
    measured floor is 1.4 KB and stays well under a megabyte even richer
    (item 48) — under 125 MB together, unconditionally.
    `deploy/install-board-watchdogs.sh` also caps the journal at
    `JOURNAL_MAX_USE` (200 MB), but only in the persistent-journal case
    (§9 "Board hardening" above has why that is not guaranteed on a stock
    image — that script assumes the journal is already persistent and
    does not itself create `/var/log/journal`); that cap is margin on top
    of the 125 MB figure, not a term the default depends on.
    `disk_free_min_gb` (2 GB) leaves more than ten times the unconditional
    figure as room to act. Default `disk_free_fault_s` (60 s) is a short
    debounce against one noisy `statvfs` sample, not a filter for anything
    transient — a filling card is a slow, roughly monotonic trend, not a
    spike the way a rail sag is. **This rule is a hint, not a verdict**,
    for the same reason the divergence rule is (below): once it starts, a
    filling card can sit below the threshold for days or weeks, and a
    daemon-wide flag latched for that whole span would be
    indistinguishable from an aquabus device that went missing sometime
    during it.
  - **the filesystem is read-only** — `hostinfo.read_mount_ro()` said so
    for `disk_path`: the kernel's own `/proc/mounts`, not a write probe.
    On an SD card this is usually the first visible sign of a dying
    card — an I/O error trips the filesystem's `errors=remount-ro` and
    every write after that fails — loudly, not silently: `Recorder.on_tick`
    logs an exception on every failed write it catches, and the model
    store's atomic write logs a warning on every failed attempt too, once
    per `model_store_interval_s` for as long as the condition lasts.
    Chosen over a write probe because the failure this rule exists to
    catch is already a fact in `/proc/mounts` the instant it happens — the
    kernel updates that table synchronously on every mount change, so
    there is no staleness a poll interval would need to cover — and a
    write probe would add both a write and an `fsync`, on every check, to
    the very card this rule is protecting. Reported the tick it is seen,
    like throttling now: no window, since the kernel has already
    remounted the filesystem and a sustained rule would only delay a fact
    everyone downstream needs immediately. An unknown reading (the mount
    table unreadable, no matching entry) never fires this rule — only a
    confirmed `True` does; never guessed either way, the same rule the
    throttling word follows. Unlike the free-space rule this clears the
    moment the mount is writable again, so it stays a *fact*.

  The first two rules and the read-only rule report a *fact*, and their
  problems join the daemon's one `problems` list (so `/api/health` goes
  not-ok and Home Assistant's `Controller problem` turns on); the
  divergence *hint* and the free-space *hint* do not — each shows on
  `device_health.host` and on the board's own `Board problem` sensor
  alone, because a hint that flips the daemon-wide flag is
  indistinguishable from an aquabus device that has gone missing.

  The air reference is `air_temps`, empty by default, which means every
  `zone_air` sensor, else every `inlet` sensor, else every configured
  temperature (a legacy config declares no roles); a name that is not in
  `mpc.temps` is a startup `ConfigError` (exit 2), checked in `main()`
  before anything is opened, not only where the rules are built. Problems
  are logged at most once per `log_interval_s` (300 s). Every threshold is
  a `host_health:` key with one default declared once in
  `health.HostHealthConfig`, validated there, shown in both example
  configs — including the two that govern the one source that starts a
  process, `vcgencmd_interval_s` (60 s) and `vcgencmd_timeout_s` (2 s).

  **Every reader gets that source, the control loop's included.** `on_tick`
  runs on the loop thread, and a fork/exec there does stretch the tick — and
  the watchdog ping behind it — without showing in the step budget, which is
  measured before the tick's observers run. The alternative, though, is
  worse: with no sysfs attribute on this kernel, a tick reader without
  `vcgencmd` sees `throttled: null` forever and the "throttling now" rule can
  never fire on the one thread that matters, while the HTTP and MQTT readers
  quietly do see it. So the cost is spent knowingly and bounded by two
  documented keys instead: one 3.3 ms fork per `vcgencmd_interval_s` is
  0.07 % of the one `dt = 5 s` tick it lands on, 1.3 % of `mpc.budget_ms`
  (250, alarm 350), and nothing on the other eleven ticks of that minute;
  `vcgencmd_timeout_s` guards the one failure that could cost more — a
  VideoCore mailbox that never answers — and a failed poll drops the stale
  word, falls through to the hwmon bit and still waits out the interval, so
  a board where the call hangs pays one timeout a minute and not one a tick.
  `health.host_metrics_reader(..., subprocess_fallback=False)` builds a
  file-only reader for a caller that wants no process at all. Nothing here
  can change a duty: the board's numbers never enter `PlantObservation` or
  the `diagnostics` the solver reads.
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
- `tools/aquacomputer_probe.py` (Pi, bring-up; replaces `sensors`): lists
  every discovered aquaero and Quadro (kind, serial, USB interface, node),
  then prints each one's status report (temperatures by group under their
  config names; rpm, duty, voltage, current and power per output, the
  aquaero's outputs 5–8 marked aquabus and "no device" without one; a field
  the kind does not measure for that output is printed but marked as not
  measured — current and power on the aquaero's own outputs, and on its
  aquabus outputs the voltage too; flow, "no data" for an absent slot;
  the Quadro's power cycles) and each output's control-report duty with the
  aquaero's control source, power limits and output mode (PWM or DC
  voltage; not interpreted on aquabus outputs; a block with no control
  source marked, which the daemon leaves out of its writes).
  Read-only: it never sends a SET or the save report. `--device`, `--serial`,
  `--timeout` (default: the `status_max_age_s` default).
- Unit tests against captured HID reports, a **fake controller**
  (`tests/aquacomputer_fakes.py`), a fake hidraw sysfs tree and a **fake
  w1 tree** in a temp directory (CI, no Pi). `pytest.mark.hardware` only
  for the live device; `tests/conftest.py` skips those tests when no
  aquaero hidraw node (`0c70:f001`, interface 2) exists.
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
`Notifier`; it knows nothing about USB, sysfs, HTTP or MQTT. Policies:

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
  to): nothing the broken tick computed is committed, an emergency
  command ramps every channel toward `max(last applied, fallback_pwm)` at
  `d_pwm_max`, and `WATCHDOG=1` is **withheld** so systemd restarts the
  process after `WatchdogSec`. The applied-command mirror *does* follow
  that ramp (below), because it moved the fans.
- **Applied-command feedback:** after every tick `state.last_cmd` and
  the newest window sample mirror the applied command (overrides,
  failures, **and the emergency ramp**), so the next tick's rate limit is
  always measured from what is on the fans and never from a command they
  no longer carry — otherwise a fan the emergency walked toward
  `fallback_pwm` comes back down in one step. Channels released from a
  manual override have their integrator entry dropped for one tick, so
  `step` re-initialises bumplessly from what is on the fan; an
  identification experiment's channels are not released (§3 *Active
  identification experiments*, §8 item 112).
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
`--source xt6|composite|sim` (default `xt6`: the one controller of
`xt6:`; `composite`: the DAS composite of `aquacomputer:` devices and
`onewire:`; `sim`: a simulated plant
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
`--source composite`, `POST /api/in/smart` and the MQTT `in/smart` topic.

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
    aqua-bridge-das.conf     # drop-in: ExecStart= --source composite (§9, §10, install-pi.sh --das)
    aqua-bridge-smart-agent.service  # systemd *user* unit example for the PC
    99-aquacomputer.rules    # udev: hidraw plugdev 0660 (the daemon), usb, optional driver's hwmon pwm (§9)
    host-usb.sh              # dwc2 host overlay, idempotent (§10)
    install-pi.sh            # provisioning, self-signed HTTPS certificate, --das (§10)
    install-aquacomputer-dkms.sh  # optional aquacomputer_d5next hwmon driver via DKMS, not run by install-pi.sh (§9)
    dkms/aquacomputer_d5next/     # dkms.conf template, Makefile, driver patches (§9)
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
    control/fancurve.py      # online PWM -> RPM fit per fan model (pure)
    control/ident.py         # identification experiments on fan groups (pure)
    control/spinup.py        # spin-up kick for a fan commanded but not turning (pure)
    control/intents.py       # intents, ControlSurface, ControlSnapshot, payloads
    control/supervisor.py    # control mode, overrides, setpoints, limits, bays, presets, experiments, compose
    control/loop.py          # read -> step -> compose -> apply -> watchdog
    hw/aquacomputer.py       # aquaero / Quadro HID report layouts (pure)
    hw/hidraw.py             # hidraw discovery, input reports, feature report ioctls
    hw/aquacomputer_adapter.py  # AquacomputerAdapter (read/apply/release/save), device entry config
    hw/sources.py            # composite of several controllers + 1-Wire + SMART inputs
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
    aquacomputer_probe.py    # read-only: attached controllers, status and control reports (Pi)
    smart_agent.py           # SMART over MQTT (PC)
    fit_model.py             # offline zoned model fit from recordings (--store-out: a store file)
    fit_fans.py              # PWM -> RPM curve per fan model (offline; online: fan_curve_online)
    replay.py                # replay recordings through the thermal model
    http_user.py             # create or update an HTTPS API user (Pi, as root)
    ha_check.py              # read-only MQTT / Home Assistant check against the broker (Pi)
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
    test_stuck_sim.py        # Stuck evidence on sim/das (sweeps: nightly)
    test_pi_das.py
    test_thermal_model.py
    test_thermal_split.py    # per-channel split of a fan group's E (item 13)
    test_thermal_ident.py    # identifiability on sim/das (sweeps: nightly)
    test_solver_das.py
    test_noise_regression.py # MPC noise vs the quietest uniform curve (sweeps: nightly)
    test_bench_budget.py     # relative step budget; absolute budget on the Pi
    test_modelstore.py
    test_fancurve.py         # online PWM -> RPM fit, its acceptance rules and the store
    test_ident_experiment.py
    test_ident_sim.py        # experiments against the DAS truth plant
    test_ident_replan_sim.py  # re-planned vs frozen levels on sim/das (nightly)
    test_ident_converge_sim.py  # closed-loop convergence, zone-wide vs one group (nightly)
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
    test_ha_check.py         # ha_check.py against what MqttClient publishes (no broker)
    test_mqtt_live.py        # marker mqtt_live: the publisher against a real broker
    test_inputs_smart.py
    test_smart_agent.py
    test_hostinfo.py
    test_publishers_runtime.py
    fixtures/aquacomputer/   # captured status and control reports, the driver's readings
    aquacomputer_fakes.py    # fake clock, sleep and controller for the hidraw adapter
    test_hw_aquacomputer.py  # report layouts against the captured reports
    test_hw_hidraw.py        # discovery on a fake sysfs tree, ioctl numbers, transport
    test_hw_aquacomputer_adapter.py  # fake controller in CI; live device with pytest.mark.hardware
    test_hw_aquacomputer_config.py   # the device entry, timing keys, hwmon-era hints
    test_hw_imports.py       # static no-control-import check
    test_hw_sources.py
    test_aquacomputer_probe.py
    test_hw_onewire.py       # fake w1 tree
    test_w1_commission.py
    test_deploy.py           # units, udev rules, install script, shellcheck
```

CI on GitHub runs every test except `hardware` and `nightly` on every PR
and push (§12), `fuzzy` and `slow` included; the nightly job adds the
`nightly` sweeps. Hypothesis example counts are capped per profile, not
unbounded. Live device tests run only on the Pi. Fake-controller tests
run everywhere.

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
`aquaero_hidraw` (the live aquaero's hidraw node; skips when absent). Gate suites use a `gcfg` fixture
parametrised over `median3`. `tests/das_fixtures.py` builds a small zoned
config (`das_mapping`, `das_cfg`) and observations (`das_obs`,
`default_temps`) for the zone, gate and fallback suites.

**Markers** (`--strict-markers`): `hardware` (live aquaero; auto-skipped
when no aquaero hidraw node, USB `0c70:f001` interface 2, exists), `fuzzy`
(Hypothesis), `slow`
(long closed-loop runs, subprocess SIGTERM), `nightly` (heavy sweeps and
long simulations: excluded from PR and `main` runs, run by the nightly
job), `solver_cases(*cases)` (restricts `solver_kind`), `pi` (only on the
Raspberry Pi, e.g. the absolute step budget; skipped elsewhere),
`mqtt_live` (the publisher against a **real** broker: needs both
`$AQUA_BRIDGE_MQTT_TEST_HOST` naming one — with `_PORT`, `_USERNAME` and
`_PASSWORD` completing the set — and `-m mqtt_live` on the command line, so
CI stays offline and a variable left exported in a shell cannot make an
ordinary suite run connect; §8 item 21). PR and
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
  `assert_command_safe`); under `trust_rule: sigma` only the setpoint
  groups count, since a lost drive or air sensor faults through σ, which
  needs history

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
  `mpc.budget_log_interval_s` naming the exceedances since the last line,
  the key that was exceeded and the keys an operator can change (raise both
  budgets; with the DAS MPC also `mpc.mpc_every_ticks`, §8 item 73);
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
| DS18B20 plateau (DAS) | a proximal DS18B20 on one 1/16 °C code for minutes while its zone's fans move | stays trusted: its window is `stuck_s` 1800 s decimated, its band `1.5 × 0.0625 °C`, its evidence only its zone's relative airflow and same-bay siblings (`tests/test_gate.py`) |
| Idle bay beside a busy bay (DAS) | an idle bay's DS18B20 on one code for longer than `stuck_s` while a neighbouring bay's reading climbs 3 °C and the fans answer a little; zone air warming against more airflow; zone channels moved apart; a one-tick fan dip | stays trusted, no zone fault (`tests/test_gate.py`; `rich` truth-sim runs in `tests/test_stuck_sim.py`, §3 Stuck sizing) |
| Frozen sensor (DAS) | a sensor frozen for its whole (decimated) window while its zone's relative airflow moved net by more than `stuck_airflow_net` | Stuck within `max(t0 + stuck_s, t1 + stuck_s / 2)` plus two decimation intervals, DS18B20 and thermistor alike; **only its zone** faults (hold, then high on its reach), other zones keep regulating (`degraded`); a redundant member's flag faults nothing; a frozen value hidden behind median3 glitches is still flagged (`tests/test_gate.py`, `tests/test_stuck_sim.py`) |
| Frozen proximal, fans pinned (DAS) | a proximal reading frozen while the zone's airflow never moves past `stuck_airflow_net` (at `pwm_max`, or a rate limit that cannot cross a window) and the ambient steps, so the zone air rises and the zone's other bays follow it | Stuck once a zone-air sensor of its zone has moved past `stuck_zone_air_dT_c` within one window **and** another proximal reading of the zone has moved by more than `stuck_sibling_dT_c`; no other sensor of the run is flagged; only its zone faults (§8 item 58, `tests/test_gate.py`, `tests/test_stuck_sim.py`) |
| Frozen proximal, air swinging against the fans (DAS) | the zone air moves against an airflow move by more than `stuck_air_oppose_max_c` | the opposition no longer excuses the reading: Stuck, since a swing that large cannot be cancelled by the drive-to-air difference (§8 item 59, `tests/test_gate.py`) |
| Drift on a zone-air sensor (DAS) | one `zone_air` sensor biased +0.3 °C/min for 45 minutes while the enclosure is healthy and every proximal reading of the zone is correctly still | **no** proximal reading of that zone is Stuck and no zone faults: the zone-air evidence needs another bay's reading to have followed the air, and a lying air sensor moves alone. The drifting sensor itself is followed like any Drift, and flagged by its own rules only (§8 item 58, `tests/test_gate.py`, `tests/test_stuck_sim.py`) |
| Lie in one zone (DAS) | any row above on a sensor of one zone | only that zone (and its declared neighbours' channels) under fallback policy; a Flicker there never resets another zone's streak; a dropout inside a redundant group is no fault (`tests/test_mpc_zone_fallback.py`) |
| Jump on a redundant member (DAS) | a second proximal sensor on a bay, a second zone-air sensor or an inlet steps +15 °C and stays | no zone fault; the member is excluded from the estimator, the solver and `last_good_obs` until its `confirm_ticks`-th trusted tick, then fused; a sole member still confirms in `confirm_ticks`; losing the confirmed member while the other confirms faults the zone (`tests/test_sensor_confirm.py`) |
| Swapped proximal sensors (DAS) | ROM ids of two bays exchanged | a Jump on both at onset (their zones hold), confirmed like any Jump; caught at commissioning, not by the gate (§3) |

The **sensor gate** is explicit (`control/gate.py`) and uses the
trusted-tick rule of §3. Tests target that gate (`tests/test_gate.py`,
and the gate-level cases here) **and** the full `step`. They include:
Jump must not stay in fallback forever; Stuck must flag when **net** PWM
(DAS: the zone's relative airflow, unless the zone air moved against it
for a proximal sensor) or a sibling sensor moves, and must **not** flag at
equilibrium with
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

**No control import.** Protocol, transport and adapter must not wait for
the board.

- `tests/test_hw_aquacomputer.py` — the report layouts against captured
  reports (`tests/fixtures/aquacomputer/`, serial bytes zeroed): both
  status reports decode to the Linux driver's hwmon readings
  (temperatures within 20 m°C, the driver having taken the next report
  for one input; everything else exact) and to the aquaero's output duty
  field; signed temperatures; a wrong id, length or checksum is
  rejected; `crc16_usb` on both Quadro control reports; patching the
  firmware reports reproduces the driver's writes byte for byte (aquaero
  channel 2 to 14.12 %, Quadro channel 3 to 9.02 %) and capture /
  restore undo them; the aquaero output mode word (outputs 1–2 PWM, 3–4
  DC voltage in the firmware fixture, aquabus blocks 5–7 `0x0500`, block 8
  unconfigured; none on the Quadro); the temperature group names; the
  captures with and without the Quadro on aquabus (fan blocks 5–8 with and
  without a device, the aquabus slots, software sensors 1–2 at their
  fallback, flow 3) and the output 7 write patched into the aquabus control
  report reproducing the hardware's report byte for byte; the save reports;
  the software-sensor output report (one sensor set, the other seven
  `0x7FFF`, the slot it lands in read back as `softN`, a bad sensor number or
  a temperature the field cannot carry, none for the Quadro) and the
  active-profile byte (profile 1 in every capture, every raw value, none for
  the Quadro); Hypothesis round trips of patched duties. The 2026-09-17
  verification captures (§2 "aquabus fields checked against the live devices")
  pin what that read found: the aquabus blocks' electrical fields in both of
  their states one second apart (so neither the current nor the voltage is that
  output's own), the aquaero's own blocks reporting no current whatever the
  fan does, the live temperature groups with 10 of 28 slots populated and the
  rest at the `0x7FFF` sentinel, the live control report with every block on
  preset 1, and the aquabus mode word read but never interpreted. The
  2026-09-18 pass adds the controller block itself (§2 "The controller's own
  start boost"): twelve blocks, not eight, with the spare four byte-identical
  to the block 8 that was caught unconfigured and the twenty bytes after the
  last one *not* another block; the six undecoded `u16` read raw for every
  channel and none for the Quadro; that four of them hold one value across all
  eight outputs and the two that differ (`+0x00`, `+0x02`) hold values nothing
  connects to a boost, which is why none can be named; that a duty write
  touches none of them; and that no kind carries an aquabus refresh interval
  any more (item 115, corrected).
- `tests/test_hw_hidraw.py` — discovery on a fake sysfs tree (interface
  selection, serial selection, ambiguity naming the serials, not found,
  uevent lines without `=`), the ioctl request numbers, draining reports
  over a datagram socket pair, end of file, errno classification, open
  errors (a permission error names the udev rule), and the output report
  `write_report` (the bytes arrive, a short write and the errno
  classification, a closed transport).
- `tests/test_hw_aquacomputer_adapter.py` — `AquacomputerAdapter` against
  a fake controller (`tests/aquacomputer_fakes.py`) with an injected
  clock and sleep: read mapping (the output duty as `obs.pwm`), the
  newest report wins, a stale status report, a silent device (only the
  first read waits, `apply()` still writes), a full kernel queue treated
  as stale and read again, the configured serial against the status
  report and a rejected serial blocking writes (a silent device still
  takes them), a re-plug with a new node reads the control report again, an
  unchanged command sends nothing, a changed command sends one SET with the
  driver's bytes and no save report, the per-kind gap timed from every
  operation (failed ones included), retries and `ctrl_budget_s` then
  `DeviceUnavailable`, a failed SET or a budget spent before it rewrites
  every channel, `save()` sending exactly one save report (not retried)
  and nothing else the adapter does sending one, write limiting (rises at
  once, falls after the interval and outside the deadband, one write
  carrying every pending change, the default zeros writing everything), no
  channel lowered in FALLBACK or DEGRADED mode (a
  rise carrying a deferred fall, a matured fall, a forced rewrite, the
  rewrite after a failed write, a channel on a firmware controller) while
  AUTO still falls, per-channel duty verification (a brief mismatch,
  one within tolerance or one without a new report does not fire; another
  channel changing every tick does not hold it off; a change of the
  channel itself restarts it), the rewrite-then-stuck escalation and its
  hint, the aquaero output mode warning (none for aquabus outputs), a
  commanded block with no control source left out of the write (the other
  channels of the same controller still written in one SET, the block's own
  fields untouched, the heartbeat still sent, one error per open, the channel
  in `unconfigured_channels` and in `device_health`; `control_snapshot()`
  showing it instead of raising; `release()` still reporting success; a block
  nobody commands no problem at all), aquabus outputs 5–8 (read, the output
  7 write with the hardware's bytes, all eight in one SET, an output or
  tachometer with no device behind it reported as `None` while the rest of
  the controller still reads and every channel is still written,
  `absent_channels` and its one error per state change, every commanded
  output absent named in that line, rpm `0xFFFF` on an own output not
  treated as absent, no duty evidence from an empty slot), the
  software-sensor heartbeat (the report's bytes and the configured sensor
  only, one per `apply()` after the duty work and none per retry, off by
  default, the gap before it, a failed write logged once and not failing the
  tick, none from a tick whose control write failed or whose budget the duties
  spent, a budget too short for the heartbeat dropping it and not the tick,
  none to a device with the wrong serial) and profile changes (a changed
  byte `0x06` rewriting every channel with one log line, found by the duty
  verification or by the periodic refresh, no extra control read per tick,
  none on the Quadro), a Quadro power cycle,
  the periodic refresh
  and `ctrl_refresh_s: 0`, `release()` restoring the captured bytes,
  rejected NaN / out-of-range / missing channel with nothing sent. The live device test is `pytest.mark.hardware`
  (`test_live_read_and_reapply_what_the_device_holds`: reads a status
  report, reads the control report right before re-applying the duty of
  every channel that already follows its preset, and asserts that no SET
  went out); it is skipped when no aquaero hidraw node exists or while the
  `aqua-bridge` service is active.
- `tests/test_hw_aquacomputer_config.py` — the device entry: kinds,
  serial, input ranges per kind, `fans` / `temp_map` errors, keys equal
  to `mpc.channels` / `mpc.temps`, every timing key's validation, the
  per-kind `ctrl_gap_ms` default, the ping interval bound against the
  watchdog (a timed-out retry within it, the unit's `WatchdogSec` against
  the DAS example's `dt` and step bound, and with the aquaero and a Quadro
  on its own USB port), every input name of both kinds, the hints for
  `hwmon_name`, `root`, `name`, `map` and `fan_map` and for the hwmon
  driver's input numbering, the flow sensor hint, any aquaero tachometer on
  any output, write limiting off by default, the heartbeat keys (off by
  default, the sensor against the kind's software sensors, a Quadro entry
  refused, a temperature the report cannot carry, both example configs
  showing them at their defaults).
  driver's input numbering, the flow sensor hint in all three places a name
  can appear (`fans.<ch>.pwm`, `fans.<ch>.rpm`, `temp_map`) naming the
  aquaero's aquabus `fan5`/`fan6`, any aquaero tachometer on
  any output, write limiting off by default.
- `tests/test_health.py` — the `fan_health:` keys and their one default,
  every rejection; the fitted curve's expected rpm (`None` for a legacy
  config); each rule firing only after its own window and clearing again; a
  duty step never firing while the band still covers it (the aquabus lag)
  and a duty that moves every tick still judged, for the rail at once and
  for rpm against the widened band; a gap in the readings restarting every
  window; nothing judged
  below `min_duty` (the captured 9.02 % duty / 128 rpm aquabus report);
  0 V never a rail fault; 0 W never a fault on an aquaero's own output but a
  fault on one that reports power; no aquaero output giving the power rule a
  number to judge, shown by running the rule over the captured readings and by
  the same readings firing it once `power_reported` is forced on; an aquabus
  output's rail never judged however the measuring and substituting reports
  alternate, while an own output's is (§8 item 89); the power rule off without
  `power_w_at_max` and following the fan law and `fans.<ch>.count` with one;
  `on_tick` publishing both halves, never raising on a broken result, source
  or publisher, and rate-limiting its log. The captured reports supply the
  numbers: the aquaero's own outputs at 0 mA / 0 W, the Quadro's aquabus
  fan 7 at 27 mA / 0.32 W / 1105 rpm, every rail inside the default window.
  The board's own rules (§8 item 103): the `host_health:` keys and their one
  default, every rejection (`air_temps` included); the air reference and its
  `zone_air` → `inlet` → every temperature default; the hot-board and
  divergence rules firing only after their own window and clearing again;
  throttling reported the tick it is seen and the `_since_boot` half never
  warning alone; a busy CPU gating the divergence rule and restarting its
  window; a missing temperature, load average or air reading judging
  nothing; the owner's own idle board (47.2 degC) against room air staying
  clear of the shipped `divergence_c`; the divergence hint leaving the
  payload's `ok` true while the board's own goes false, and a hot board
  joining the daemon's problem list; `on_tick` publishing the board's
  verdict next to the fans', surviving a host reader that raises, and
  leaving the observation untouched. The disk rules ("the disk nobody
  watches", §8 proposed items): `disk_path`'s validation and the free-space
  default's arithmetic cross-checked against the recorder's and the
  journal's real bounds; the free-space rule firing only after
  `disk_free_fault_s` and clearing, and staying a *hint* (`faults == []`,
  `hints` carries it) so it never joins the daemon-wide list even while
  it fires; a missing or unreadable `disk_free_gb` never firing, degrading
  to unknown; `disk_used_pct` riding along in the verdict; the read-only
  rule firing the tick it is seen and clearing, as a *fact*; an
  unknown or non-boolean `read_only` value never firing; both rules
  naming `disk_path` in their messages, and the read-only message saying
  the recorder and the model store fail loudly, not silently; both disk
  rules firing together with the right one in each list; both still
  publishing their numbers with `host_health.enabled: false`; `on_tick`
  publishing both and only the read-only fact joining the daemon-wide
  `problems`; `host_metrics_reader` threading `disk_path` into
  `collect_hostinfo`.
- `tests/test_hw_sources.py` — `CompositeSource` over two fake
  controllers and a fake 1-Wire source: merged reads, each channel
  written to its own device with one SET each, a failing device (first or
  last in the list) raises after every other device was still read or
  written, naming all failed devices, `xt6:` plus an `aquacomputer:` list, timing keys per
  device, distinct serials for one kind, every name bound exactly once
  (exit 2 otherwise), aquaero `pwm5..pwm8` together with a commanding Quadro
  entry refused either way, with or without a serial (a sensors-only Quadro
  entry allowed, item 106), the stuck Quadro hint only next to
  an aquaero reporting an aquabus device, the loop over an aquaero whose
  aquabus slot empties (the loop keeps controlling the healthy half with no
  read error, that channel's `rpm` / `pwm` `None`, one error line, and the
  channel back when the device returns), every commanded channel absent
  still reading its temperatures, and a start with an empty slot sending
  `READY=1`, a missing ROM
  at start does not block, `inputs["smart"]` only with an inbox.
  aquabus slot empties (the fallback ramp reaches `fallback_pwm`, the stop
  write succeeds) and a start with an empty slot sending `READY=1`, a missing ROM
  at start does not block, `inputs["smart"]` only with an inbox,
  `inputs["fans"]` merged across controllers and `device_health()` merging
  both controllers' problems and still answering after a failed read.
- `tests/test_hw_imports.py` — `hw/` never imports `control`, and
  `hw/aquacomputer.py` imports no I/O module.
- `tests/test_hw_onewire.py` — a fake `w1_bus_master*` tree: discovery,
  the trigger/poll/read cycle, sensors of another bus ignored, CRC
  failure and garbage → `None` and counted, a bulk read that never
  completes or a failing trigger → `None` without raising, resolution
  written once, stale samples → `None`, missing ROMs.
- `tests/test_w1_commission.py` — `--list`, `--identify` ranking by
  warming rate, `--check` building the daemon's composite.
- `tests/test_aquacomputer_probe.py` — the probe on a fake sysfs tree and
  fake controllers: listing, decoded output including the temperature
  groups, the aquaero output mode, the active profile, aquabus outputs with
  and without a device and the unconfigured block, filters, failures
  reported, no SET, output report or save report sent.

These never replace §4.1–4.6.

### 4.8 HTTP tests

`tests/test_http_api.py`. See also §6. A stub `ControlSurface`; no
hardware. `/api/state` and `/api/health` bodies equal
`ControlSnapshot.state_payload()` / `health_payload()` with exactly the
documented keys. Auto mode: `POST /api/pwm` → 409. Malformed JSON, body
not an object, missing field, unknown channel, PWM outside `[0, 1]` or
outside `[pwm_min, pwm_max]` → 400. Unknown `/api/...` route → 404.
The DAS routes are tested in `tests/test_das_intents.py` (`POST
/api/limit` and `/api/bay` on DAS and legacy configs, fuzzed bodies never
5xx and never exceed a configured limit, `GET /api/estimate`, `/api/bays`,
`/api/model`, 404 on legacy), `POST /api/ident` 200 / 400 / 409 with
the reasons (here), `POST /api/calibrate` (here and in
`tests/test_supervisor.py` / `tests/test_estimator.py`: the happy path
through the real `Supervisor`, every refusal with its reason — unknown bay,
out of range, legacy config, `no_tick`, `empty:<bay>`, `untrusted:<zone>` —
`401` without credentials and `429` under the uncached-check limit, fuzzed
bodies never 5xx, and that one manual reading produces byte-identical
estimator state to the same value arriving as SMART) and
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
deleted file; the global limit on uncached checks with an injected clock
(a flood from many addresses admits `auth_verify_max` derivations per
sliding window and gets `429` with the window's `Retry-After` for the
rest, charges no failure to the refused addresses and never delays a
cached login; `0` disables the window; `auth_verify_pending_max` refuses
while checks are queued and a slot is released even when a check raises;
a backing-off client takes no slot; a queued check for credentials cached
meanwhile runs no PBKDF2; refusals are logged once per window); the failure bookkeeping's work per request under a flood of malformed headers from many addresses, and its pruning against a full scan (property test); the worker thread's raised nice value (Linux)
and the no-op elsewhere; over HTTP, the checks run on the worker thread,
the flood gets `429` and a cached login `200`; TLS context refusals. Over the wire, with a certificate
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
- `tests/test_loop.py` — §4.3 loop cases, READY/WATCHDOG/STOPPING, the
  applied-command mirror after an emergency ramp (the next healthy tick
  rate-limits from the fans, not from the command they no longer carry,
  while the window sample of the last good tick is left exactly as it was),
  overrides and bumpless release through the loop, an override on frozen
  temperatures tripping Stuck, run scheduling, the `on_tick` hook, the
  step budget alarm (injected clock: tracking, warn/error thresholds,
  log rate limiting, the raised thresholds of the §8 item 73 fallback, the
  config keys both lines name, `/api/health` and the MQTT state blob), sim
  closed loops (`slow`).
- `tests/test_main.py` — CLI parsing, exit codes, `--once`, sim wiring
  (`--sim-plant basic|rich|das`), xt6 map mismatch → exit 2, `--source
  composite` wiring and a missing binding → exit 2, `--source hwmon`
  rejected, a config of the hwmon era → exit 2 naming the replacement
  keys, the hardware worst case per tick against `$WATCHDOG_USEC` → exit 2,
  `--record` and
  `record_path`, `--model-store` and `STATE_DIRECTORY` (legacy: ignored /
  exit 2), publisher wiring and start failures, the
  SIGTERM stop path in-process, a SIGTERM delivered inside a stderr write,
  a second SIGTERM during shutdown, and a real subprocess (`slow`). Item 103:
  a `host_health.air_temps` typo exits 2 before anything opens; the tick's
  host reader runs `vcgencmd` once per `host_health.vcgencmd_interval_s` and
  not once per tick, honours `vcgencmd_timeout_s` and leaves the tick's
  `throttled` non-null; the publishers' reader is built from the same keys.
  The run itself is faked, so no test starts a process.
- `tests/test_sdnotify.py` — address resolution, the watchdog period
  from `$WATCHDOG_USEC` / `$WATCHDOG_PID`, no-op without
  `$NOTIFY_SOCKET`, failures return `False`, a real `AF_UNIX` datagram
  socket (skipped where binding one is denied).
- `tests/test_mqtt_ha.py` — topics, Discovery entities (the
  `device_problem` and `host_problem` binary sensors in both modes, their
  templates and their attributes topics), PWM numbers only
  in manual and deleted on leaving it, state payload, command parsing
  never raises, `on_message` never raises, `cmd/ident` and a retained
  `start` that never starts an experiment, `cmd/calibrate` under and
  without `mqtt.allow_calibrate` (item 105: no topic and no intent
  without it, so the refusal never reaches the supervisor or the
  estimator; a retained reading ignored; the opt-in refused without
  `mqtt.username`; no Discovery entity either way),
  `add_topic_handler` /
  `topic_matches`; and the whole Discovery contract item 21 confirms
  live: the exact entity table per config and control mode (component,
  unit, device class, state class), every payload's config topic, unique
  id, state topic, device block and availability trio, only numbers
  carrying a command topic and every one of them subscribed, their range
  and step, a setpoint / limit / manual-PWM command reaching the
  supervisor on the entity's own command topic, a SMART message reaching
  the inbox, the qos and retain flags of every publish, the last will,
  and `online → Discovery → state → offline` under the configured
  `node_id` / `discovery_prefix`. No broker. DAS entities and topics:
  `tests/test_das_intents.py`.
- `tests/test_ha_check.py` — `tools/ha_check.py` against exactly what
  `MqttClient` publishes (fake paho): a clean checklist, a missing
  entity, a stale retained PWM number, a config payload from another
  build, a missing or `offline` availability topic, no state topic, SMART
  ages, a retained command; a template's `| default(...)` read as a value
  and not as "unknown" (a DAS with `model_shadow: false`); `--send`
  naming the command, saying what outlives it and doing nothing without a
  typed `yes`; credentials never printed, the broker host only under
  `--verbose`, never the daemon's client id; a negative `--wait` refused
  as an argument error. Its broker layer against a fake paho client: a
  CONNACK that never comes and one that refuses, a refused SUBACK, and
  that neither is charged to the collection window. No broker.
- `tests/test_mqtt_live.py` (`mqtt_live`) — the publisher against a real
  broker when `$AQUA_BRIDGE_MQTT_TEST_HOST` names one **and** `-m
  mqtt_live` selects it: retained
  Discovery and state, a command from another client reaching the
  supervisor, `in/smart` reaching the inbox, `ha_check` clean, a clean
  stop leaving a retained `offline`. Its own `node_id`
  (`aqua-bridge-test-<pid>`); every retained message it leaves is deleted
  afterwards, and a cleanup that cannot reach the broker prints the
  topics it left rather than raising over the real failure.
- `tests/test_inputs_smart.py`, `tests/test_smart_agent.py` — inbox
  staleness by receipt time and rejection of malformed payloads; the
  agent's discovery, `smartctl -j` fixtures including `-n standby`, never
  raising.
- `tests/test_recorder.py`, `tests/test_tools_fit_replay.py` — record
  shape (DAS and legacy, the `fans` key from `inputs["fans"]` and `{}`
  without it, a recording made before it still replaying), rotation,
  never raising; `fit_model.py`,
  `fit_fans.py` and `replay.py` end to end on synthetic `sim/das.py`
  recordings.
- `tests/test_publishers_runtime.py` — `HttpService` reporting a start
  failure (serving over TLS and the refusals: `tests/test_http_auth.py`); `MqttService` with a fake client: nothing
  while disconnected, Discovery per connect and on the manual boundary,
  host refresh interval, swallowed publish errors.
- `tests/test_hostinfo.py` — every host metric against fake procfs /
  sysfs files, `None` on missing or malformed input; the board's throttling
  state (§8 item 103): every bit of both halves; the three sources in the
  order the chain tries them — the sysfs attribute preferred, `vcgencmd`
  where it is absent, the `rpi_volt` hwmon alarm where both are (found by
  `name` across several hwmon directories, with `rpi_volt` deliberately not
  at index 1), and `None` where none of them reads; a partial reading
  claiming nothing it did not read, and its summaries `None` rather than
  false; `ThrottledReader`'s cadence counting the runner's calls against a
  fake clock (one a minute over ten minutes of 5 s ticks), the cached word
  reused between polls with its age, and a failed poll dropping the word,
  falling through to the hwmon bit and not being retried before the interval
  is out. The runner is always a callable the test owns — no test starts a
  `vcgencmd` process. `read_mount_ro` ("the disk nobody watches", §8
  proposed items): the kernel's own
  read-only report against a fake `/proc/mounts`; a confirmed read-write
  mount is `False`, not `None`; the longest matching prefix wins so a
  sub-mount is judged by its own entry; two entries for one mount point
  (a stacked mount, not a remount — `/proc/mounts` is generated fresh
  from the live table, so a remount never adds a line) and the later one
  wins; no matching entry, or an unreadable mounts file, is `None`; a
  short or garbage line is skipped, not raised; an escaped mount point
  (`\040` etc.) still matches the real path instead of silently falling
  back to the root entry; an embedded NUL byte in the path (the
  reachable failure of `os.path.realpath`, not an `OSError` — that one is
  swallowed by `realpath(strict=False)` on POSIX) degrades to unknown,
  not raised.
- `tests/test_deploy.py` — unit file (`Type=notify`, `NotifyAccess=main`,
  `Restart=always`, watchdog, no `ExecStop=`, venv `ExecStart`,
  `TimeoutStartSec >= WatchdogSec`), udev rules against the unit’s
  groups (hidraw read/write, usb, the optional driver's hwmon pwm group
  write), install script triggers udev for hidraw and does not run the
  DKMS script, `dkms` / `curl` / `patch` not in the package list, the DKMS
  script naming its packages and stopping early without them, install
  script creates the self-signed certificate at the
  `http:` default paths only when absent and creates no users, `bash -n` and `shellcheck` on both scripts (shellcheck skipped
  when not installed; GitHub runners have it), `StateDirectory=` for the
  model store, the SMART agent unit being a user unit that
  `install-pi.sh` does not install, and the journald cap ("the disk
  nobody watches", §8 proposed items): `JOURNAL_MAX_USE` a documented
  variable at the top of the script, actually installed as both
  `SystemMaxUse=` and `RuntimeMaxUse=` in the drop-in.

### 4.10 DAS suites (estimator, model, solver, experiments)

Everything runs against `sim/das.py`, the DAS truth plant, or small zoned
configs; no hardware. "PR" runs in every PR and `main` run; "nightly"
tests carry the `nightly` marker.

| File | Proves | Where |
|------|--------|-------|
| `tests/test_zones.py` | DAS config parsing, defaults and rejections; `strict` trust per group (`sigma` without an estimator update); closure `F*` with `declared` / `none`; per-zone timers and confirmation; `degraded` vs `fallback`; legacy = one implicit zone; per-channel `compose`; per-role Stuck sizing; the DEGRADED banner and health field | PR |
| `tests/test_mpc_zone_fallback.py` | a fault in zone A never lowers any channel of its reach below `prev` (hold, then `max(prev, fallback_pwm)`); channels outside keep regulating; the solver request never carries faulted-zone sensors and healthy commands do not depend on their values; per-zone recovery is bumpless; Flicker in one zone never resets another; a dropout in a redundant group is no fault; solver faults; legacy `mpc` turns a zone fault into whole fallback | PR (one sweep nightly) |
| `tests/test_sensor_confirm.py` | sensor confirmation (§3): a jumping redundant member (proximal, zone air, inlet) is not fused until it confirms and the zone does not fault, the estimates of the DAS example config match a run without the member until then; a real level change is fused after `confirm_ticks`; restart on a new jump or a dropout; a sole member costs `confirm_ticks` once; a sensor missing since boot confirms its first reading too, including while every zone is blind and on a time-faulted tick (item 61); a zone in fault waits for a confirmed member in every group; the outward `diagnostics["gate"]` agrees with itself about a confirming sensor -- `per_temp` `false`, `confirming` in `reasons`, `trusted` recomputed -- even on a tick its raw reading passes cleanly (item 63); a property over random jumps, dropouts and time faults (median3 on and off) that the counts follow the gate's raw verdict and no confirming sensor reaches the estimator, the solver or `last_good_obs`; malformed memory; JSON and determinism; legacy keeps no state | PR |
| `tests/test_sigma_trust.py` | `trust_rule: sigma` (§3, §8 item 8): thresholds inclusive, empty and undeclared bays, an uninitialised zone, a sigma that is not a number, time faults, unknown keys and setpoint groups still fault, `strict` ignores the estimator; the `sigma_fault_c` floor; switching rules by config only; the verdict reads this tick's σ across a crossing; an estimator fault applies `strict`; a zone in fault returns without its lost sensor only under `sigma`; on the truth sim 2 % DS18B20 dropouts fault far fewer zones than `strict` with no violation (both DAS solvers), a replay without a bay's only proximal sensor never lowers its zone's airflow beyond 2 % and raises it within ten minutes (PI-like DAS), losing either member of a redundant pair changes almost nothing, a bay's or a zone's sensors lost for good fault the zone (on the drive σ, or on the blind air clock of item 70) and it holds, then ramps high; the example's redundant pairs fault no zone on `rich` with their σ at the floor (item 67); a hot swap no longer faults its zone but a swapped bay that goes blind faults at once, and a flapping proximal sensor spends `bay_settle_max_s` and then faults its zone on every jump again (item 69); a zone that loses only its air sensor faults on `air_blind_fault_s`, and never at all with that key set wide (item 70); the soft sigma floor (§8 item 68): it holds the command before the loss, ends on the σ growth or the hold time, falls at its rate, reopens on a further lost group, keeps its episode on an estimator fault, starts over from malformed memory, its config keys; on the DAS MPC without a bay's only sensor the no-floor run reproduces the drop (−0.04, about 21 % less airflow) and the soft floor holds, then releases | PR: `basic`; nightly: dropout sweep on `basic` and `rich`, the redundant pairs on `rich` seeds 0–5 and both solvers, a sensor lost for good on both presets and solvers, the soft floor's margin, noise and no-ratchet bounds on `basic` seeds 1–5 and `rich` 0–2, both solvers, two sensors, and its margin bounds on the DAS MPC until the floor has released (`basic` b02 and b13, `rich` b02) |
| `tests/test_das_core.py` | the core invariants, closed loops and DAS goldens for `pi_das` and `mpc_das` (§4.2) | PR |
| `tests/test_pi_das.py`, `tests/test_estimates.py`, `tests/test_das_config.py` | the margin-deficit PI (served zones, unconstrained channels, fixed channels, occupancy), the estimates block and prior map, `noise` / `limit_c` / served-zone config | PR |
| `tests/test_estimator.py` | exact discretisation and Joseph form (random sequences keep P symmetric PSD); first tick and constant readings; σ grows while a bay is unobserved and shrinks back; redundant members; placement offsets (a constant disagreement is an offset and not a swap, `proximal_offset_c: 0` reproduces item 67, one sensor carries no offset state, a swap still widens a bay with two, either member keeps the bay observed); a node per proximal sensor (item 101: the slope between two placements learned from the two readings alone, the same state size, a config with one sensor per bay bit-identical either way, a member that returns after the load moved a swap only on the fused node, both members / one absent / one lying the other way / a hot swap on both layouts, a member that *drifts* followed by its own map on both layouts instead of biasing the shared drive, the learned map bounded so it cannot run along the combination its rows identify, and the diagnostics publishing the map the filter predicts with); the two notions of settling (item 100: the reason and the seconds left of each exemption, an occupancy change and a calibration step named, and a bay nobody reads out of the model checks but still faulting its zone); the fast-swap rule needs a bay's proximal members to agree, so a standing disagreement between a redundant pair never locks the bay's SMART out while a step both members see together still fires it (§8 item 17); the bay-level `swapped` report of a redundant pair (§8 item 109: a standing gap is not a swap on any tick, a step both members of a disagreeing pair take still is, and on the `rich` truth sim over five seeds neither redundant pair is ever reported swapped while a drive replaced in place by one 12 °C hotter is caught on b03 and b10 no later than on the single-sensor b02, each bay silent on every tick before its own swap so that a standing alarm cannot pass for a detection); per-bay seeding (a bay missing on the first tick is seeded by its first reading, a sensor returning after a later loss still widens its bay), `settling` expiring and its wall-clock budget (repeated jumps at four cadences spend `bay_settle_max_s` and stop exempting, a clean run earns it back, an empty bay spends nothing, 0 grants none), `air_blind_s` and a tick gap counted in full; the occupancy machine incl. "never empty while zone air is unobserved"; SMART calibration acceptance, rejection, serial change and expiry after `calibration_max_age_days`; a guessed serial never relaxes a class; the manual-calibration half of item 104 (a bay whose occupancy crosses `empty` drops its manual entry on that tick while a bay the rule did not fire on keeps its own, and a bare proximal step inflates the entry instead of deleting it, `inflate` / `confirm` set and `σ_cal` doubled); `restore_manual_calibration` never raising over a malformed section and raising only for a legacy config, and a live manual entry winning over a stored one; determinism, JSON round trip, malformed memory; the swap event (§8 items 123, 124: on a one-sensor bay the bay-level verdict and the per-sensor rule call one threshold with the same `nu` and the same variance on every tick — watched through that function, so a change to either half's arithmetic fails here — a bay-level step widens the bay the way a jump does even where no member crossed, the model reset spends the trust exemption's own budget and stops once it is gone while the verdict goes on firing, a refused event is counted once per `bay_settle_s` rather than once per tick, a bay out of budget loses the model gate's `uncertain` exemption so its stale fit is scored rather than excused, an occupancy crossing is never rate-limited however often it comes, and a reset bay's count / last time / reason are published and survive the reset); the placement verdict (§8 item 125: the layout and what it separates named per member, the gap read against the prior box at this tick's rise, a pair outside the box reported and a wide box not) | PR |
| `tests/test_associate.py` | detrended correlation, greedy assignment with margins, confirmation, full-window history, drops on silence / jump / empty, a declared serial wins; on the truth sim the right bays are found and indistinguishable bays refused | PR |
| `tests/test_stuck_sim.py` | Stuck evidence on the truth sim (§3 Stuck sizing, §8 items 3 and 58): healthy `rich` runs flag no sensor and fault no zone; a frozen DS18B20 or thermistor proximal reading is flagged once its zone's airflow moves, or, with the fans held by the rate limit, once its zone air has risen past `stuck_zone_air_dT_c` and another bay has followed it; a zone-air sensor drifting on a healthy enclosure flags no proximal reading; only a bay's last proximal sensor faults its zone | PR: 2 seeds, 3 frozen runs, 1 drifting-air run; nightly: 48 seeds, 2.5-hour runs of both DAS solvers, and the per-seed coverage sweep (each proximal reading frozen in turn) |
| `tests/test_hotswap.py` | slow swap, quick swap (never through `empty`, margin widens, class follows the new drive) and empty-at-boot on the truth sim: no limit violation, no zone fault, fans rise, constraints removed on empty | PR |
| `tests/test_thermal_model.py` | structure, fan groups, parameter table, Jacobians vs finite differences, `eig` vs matrix exponential and the Euler fallback, windows, lag correction, RLS safeguards, status machine, never raising in `step`, shadow never changes the command; the diagnostics that say why a zone is still learning (items 110, 111): `pe_diag` is the diagonal of the matrix `pe_min` is the smallest eigenvalue of (so `pe_min ≤ min(pe_diag)`) and reads 0 on a block with no windows, `blocked` names exactly the gates the `converged` rule fails and is empty exactly for a `frozen` zone and for a `converged` one that converged on every bay (§8 item 119), and a bay's `se` is its `rel_se` times the coefficient and `null` wherever `rel_se` is, including on a block that has closed no window; partial convergence (§8 item 119: `min_informed_bays` rounds up and never falls under one, the air block is never substitutable, a zone converges on the bays that informed themselves and publishes `partial` / `uninformed` / a non-empty `blocked`, those bays reach the solver at their own lower confidence bound and an informed one at its fitted value, the σ that bound spent published beside it as `k_se_used` where `se` will not publish it, `model_freeze` never freezes a half-informed zone, the default 1.0 is the old rule with nothing derated, and the informedness latch: a converged model whose `pe` has decayed to nothing still reads `blocked: []` / `uninformed: []` / `partial: false` and is not derated at any fraction, a block that never informed itself still is, the latch is written by a closing window and cleared by a hot-swap reset, and a store from before it existed derives it once) | PR |
| `tests/test_thermal_split.py` | the per-channel split (item 13): only a multi-channel group gets keys, the prior split reproduces the shared-`E` prediction and Jacobians bit for bit, a split redistributes without changing the group total, the Jacobian of a split group against finite differences, a whole-group experiment leaves the split at 0 while single-channel phases find a 60 % difference, a store file written with the switch the other way is converted (and a corrupt coefficient on that path drops the thermal section alone, not the seed), and a split that would make a channel cool less than nothing is projected back so the zone's airflow and its Jacobian stay usable | PR |
| `tests/test_thermal_ident.py` | identifiability with group experiments on the truth sim: in-zone `E` within 15 % (PR seed), per-bay `k` within 25 %, `leak` / `κ` at prior, convergence; regulation only never converges; the seed sweep (bound 25 %), sensor offsets, the rich preset | PR: 4 cases; nightly: sweeps |
| `tests/test_solver_das.py` | active-piece SQP vs a projected-gradient reference, monotone objective, iteration cap, forbidden-band snap and hysteresis, zero-order hold, prediction vs thermal Jacobians, noise index and surrogate, bumpless offset, fixed channels, validity gate and model fallback with dwell (the entry dwell, the model rate through the drives' filter, the air-disturbance check, the prediction guard on eligible rows only), a clock stepped back, horizon and block extremes, the estimator's own exemption read rather than re-derived (a real fast-swap jump driven through `mpc.step` reaching the gate as `uncertain`, and the set surviving a solver restart, §8 item 100) | PR |
| `tests/test_model_fallback_sim.py` | the validity gate against the truth plant: the observed drive rate (ramp, restarts, memory), a sound model returns within `model_return_dwell_s + 2 · mpc_every_ticks · dt` and does not re-enter (§8 items 10, 64), a model with wrong bay gains is caught and held, a healthy enclosure never reaches the fallback (§8 item 65), a fouling jump to 0.15× airflow does (§8 item 66) and the same run without it does not, every drive within its limit and bumpless switches; the plain drift holds the same step | PR: 1 return, 1 broken model, 3 healthy seeds and the fouling pair per preset; nightly: zones × seeds on `basic` and `rich`, more broken gains, 8 healthy seeds and 4 fouling seeds per preset |
| `tests/test_noise_regression.py` | calibrated DAS MPC noise ≤ 0.8× the quietest uniform curve at equal or better worst true margin (measured 0.31–0.48×); uncalibrated bound 2.0 (0.30–0.69×); rich preset bound 1.25 (up to 1.18×); on `rich` every bay calibrates and the estimate follows the drive-*reported* temperature to 2.5 °C rms and never reads more than 1.0 °C *below* it (§8 item 17); MPC vs PI-DAS reported, not asserted (PI-DAS has not settled within the window on several seeds) | PR: 2 seeds; nightly: 8-seed sweeps |
| `tests/test_bench_budget.py` | DAS MPC step p99 ≤ 12× (named constant) the legacy MPC p99, the 75th percentile of several interleaved, warm-up-discarded repeats (§8 item 6); `bench_step.py` runs both DAS solvers; absolute p99 ≤ `mpc.budget_ms` only on a recognised Pi board (`platform.machine()` in `PI_MACHINES` -- armv6l Zero W, aarch64 Zero 2 W; a skip on any other board names what it saw, §8 item 108); the Zero W fallback of §8 item 73 (`budget_ms` 1000 with `budget_alarm_ms` 1250 loads, `budget_ms` 1000 alone is rejected, `mpc_every_ticks: 3` solves a third of the ticks and a solve tick is the expensive one) | PR / Pi |
| `tests/test_modelstore.py` | config keys, store path (CLI, env, legacy), fingerprint covers structure not policy, corrupt / truncated / wrong-schema / wrong-fingerprint files → prior, fresh vs stale by age (a clock behind the file is stale), fresh loads `frozen` and the MPC acts at once, stale holds until `model_reconfirm_s` with the prediction error in bounds, calibration keyed by serial and inflated when stale, a save does not reset a stale hold, atomic writes, malformed seeds never raise; a `tools/fit_model.py` report in the store's place loads as a model, ages like a store file, drops a model of another structure and is refused outright when its `store_fingerprint` is another config's or missing (item 15); the settle timers round-trip through the file as seconds already settled minus the daemon's outage, drop on a long outage, a stale file or an unknown age, a malformed section is dropped with a warning, and the persister asks the supervisor for them before a save (item 20); the handheld calibrations round-trip per bay, are always restored provisional, are kept at the staleness window's boundary and dropped one second past it, dropped for an unknown or negative age with the warning naming which of the two causes it was (no saved sample time against a wall clock behind the file), dropped when the bay's declared `occupied` / `class` / `serial` has changed, and land in the same estimator memory as the SMART ones (item 104) | PR |
| `tests/test_fancurve.py` | the online PWM → RPM fit (item 14): a swept fan is identified per fan model, one duty or a ramping command is never enough, a noisy tachometer is refused by the residual, the bins stay bounded and follow a fan that changes, a malformed memory or a changed channel → fan-model map starts over, `curve_pair` falls back to the config for anything unusable, `step` publishes the fit into the store's `fan_curves` and reports it, the curve round-trips through `model.json`, and `model_use_rpm` keeps the configured `rpm_max` as its reference so a worn fan still reads as less air. Item 107, the readers: every reader has a decision in `READERS`, the estimator's airflow and the noise objective's `u0` follow a fitted curve and fall back without one, the noise diagnostics name the curve behind the index, fan health judges a fan at half speed a deviation whatever is fitted, the Stuck airflow evidence and the identification's excitation reach stay on the configured curve, a fit nothing re-confirms goes stale and every reader falls back, one that keeps being re-confirmed does not, and a curve from the store stays in force until a fit replaces it | PR |
| `tests/test_ident_experiment.py` | config rules; groups, targets and served zones; the seeded two-level sequence; every precondition with its reason; the envelope at its threshold; the aborts (human intent, stop, fallback, every zone in fault, emergency command, apply failures, a frozen sensor); the re-planned levels of item 52 (the echo of the experiment's own level taken out of the want and no ratchet over a run of high ticks, a fan the solver's own floor lifted still read as demand, the anchor rising at once and falling only at a switch and never below the frozen base, a held sibling with it, the band, `symmetric` dipping at most the amplitude below the live demand, the same aborts on the same tick with and without re-planning, a re-planned level never below the frozen one, the dip and the floor read off the running experiment's own plan, the demand copied only on its ticks, and through the loop a warming zone followed instead of held back, a spent load released instead of pinning the fans, and an experiment ending later than the frozen one and never earlier — against `ident_replan: false`, which still holds it back); bumpless release; overrides through `compose` and fallback beating them; a restart never resumes a run but the settle timers come back from the store and a start between `plan_tick` and `record_tick` keeps its tick (§8 item 20); `k·σ` counted once and the absolute abort from `ident_abort_below_limit_c` (items 53, 54); a lost sensor group blocks a start and aborts a run (item 72); the excitation reach of item 110 (`rel_swing` matches the plan's closed form and the fan's own curve, falls with the base, rises with the amplitude and is read off the clamped pair; `excitation` / `unexcitable` / `holds_cover_window` in the status; `pe_bound` is the constant and not `pe_min`; `excitable` at the configured amplitude against `excitable_at_cap` at the `0.3` cap, which separates "raise the amplitude" from "only a lower park reaches it"; the published reach met by the monitor's own `pe_diag` when a window sits inside a hold and missed when it straddles a switch; `ident_require_excitable` refusing with `not_excitable:<ch>` and a channel the band already refuses getting one reason, not two); sizing the telegraph to the headroom (§8 item 120: the keys' defaults and validation, `fixed` reproducing the old levels to the digit, `headroom` never exciting less than `fixed`, neither level ever further from the base than `ident_amplitude` in either direction on either `ident_levels`, a start `fixed` refuses for the band running sized to the room, `band:` still refusing a channel whose cut left no swing while the mid-band starts `fixed` allows still run, the per-channel planned dip, and `excitation` reading the cap around the plan's own base); the release of item 112 (the experiment's channels are never put into `released`, the solver keeps its integrator through a completion and through every human intent, and a run stopped on its high level hands back the solver's own level while the fan walks down at `d_pwm_max`); legacy refuses | PR |
| `tests/test_ident_sim.py` | an experiment on the truth plant never takes a drive over a limit and never leaves the enclosure hotter than the same seed without one, with the excitation visible on the fans; a drawn enclosure that is already saturated refuses the start; and, with the room warming 20 °C/h so the envelope actually binds, the run aborts on `envelope:<bay>` with `T̂_d + k·σ` still below the absolute abort and no drive over its limit from the abort on, on `ident_levels: above` and `symmetric` (§8 item 53) | PR: `basic` seed 1, both `ident_levels`; nightly: `basic` and `rich` seeds 1–5, envelope sweep `basic` seeds 1–5 × both `ident_levels` |
| `tests/test_ident_replan_sim.py` | re-planned against frozen experiment levels on the truth sim (`rich`, the real loop and supervisor), both solvers and three seeds: no tick below the solver's own command (and the frozen plan does hold one back), no anchor more than `ident_amplitude` above the demand the frozen arm saw (the ratchet guard, which the pre-review `_replan` fails on the MPC arm), and the fit no worse than frozen beyond a loose margin | nightly |
| `tests/test_ident_converge_sim.py` | closed-loop identification on the daemon's own path, 16 h on the truth sim (`rich`, the real loop and supervisor), three seeds. Item 102: a zone-wide `ident_parallel: true` schedule against a one-channel-at-a-time round robin over *all eight* channels on the same plant — the measured converged zones per seed on both arms (z0+z3 / z0+z2 / z0+z3 against none / none / z0), `pe_min` and `excited_windows` on the converged zones held to the values the runs reached rather than to the rule's own thresholds, which of the round robin's converged zones still hold `PE_MIN` at the end of the run rather than latching from a burst, the zones that close no window at all named per seed (none since §8 item 109), zero limit violations in either arm, both arms' true margin above a floor and the mean-PWM gap bounded both ways. Item 112: no channel is left with a floor above `pwm_min` after an experiment ends, and both the enclosure's mean PWM and qd1's own stay under what the old release spent (the channel mean is what separates the two releases on the seeds where qd1 already reached `pwm_min`). Item 111: the same schedule at 36 h — the zones that converge with more windows (6 of 12 at 16 h to 11 of 12 at 36 h since §8 item 109's fix), no zone blocked by an `E`'s relative standard error, the one bay that still blocks a zone (b15, seed 3) named, and `se(k)` under 0.12 on every bay. Item 119: the same schedule at `model_converged_bays_frac` 0.75 and 0.5 — the zones each fraction converges per seed and the bays each converged zone was converged *without*, that a lower fraction never converges fewer zones than a higher one, that such a zone publishes `partial` and keeps naming those bays in `blocked`, and that every one of them is planned at its own `k_used` lower bound while an informed bay keeps its fitted value; and at 36 h that `0.5` closes the one zone the whole-zone rule never reaches (z3, seed 3, blocked on b15). Item 120: `ident_levels: symmetric` at the shipped amplitude over 12 h, `fixed` against `headroom` — the starts the `band:` precondition refuses (11 / 4 / 7 of 12 against none), that no start is refused for the band once the telegraph is sized to the headroom, and that no tick ever gives up more than `ident_amplitude` under either | nightly |
| `tests/test_sim_das.py` | the truth plant: energy balance through transients and hot swap, steady state, more airflow never warms anything, dead band and exponent, quantisation per sensor type, lags, SMART cadence, determinism per seed | PR |
| `tests/test_spinup.py` | the spin-up kick's section and its sequence (§8 item 75), driving the supervisor the way the loop does with the tachometer scripted: every key, every rejection, the per-output kick duty and its clamp, the `mpc`-aware checks (an unknown channel, a `kick_s` shorter than a tick), both example configs at their defaults; confirm → kick → verify → the next kick → failed; a fan that starts on the first kick and one that needs two; the failed fan's message and its siblings' floor holding while the solver asks for less, with an unrelated zone's channel left alone; the kick as a no-op while the solver commands more; an output with no fan, a fan with no tachometer, a channel with no tachometer bound, a duty below the stall duty and a legacy config all off with their reason; a gap starting the window again; the floor surviving both a gap during a retry and one windmilled reading, and going only after `clear_s`; a second dead fan neither ratcheting the floor nor being pinned by it, and a hold recorded in a hot spell capped; a `kick_s` too short for the `d_pwm_max` ramp refused; the floor applied under `fallback` too, pinned against the same run with the rule off, and absent from the diagnostics when nothing is in force; the log lines; a run with the rule on never below the same run with it off; the measured noise cost of a kick (§3 "Spin-up kick"); the health payload carrying the verdict and the failed fans, and a bad key or a channel typo exiting 2 before anything opens | PR |
| `tests/test_spinup_sim.py` | the same rule through the real `Loop` against the DAS truth plant, where the rotor really does not turn (the simulator's `start_duty`): a stalled fan starting on the first kick and staying turning at `pwm_min` afterwards, a `kick_s` too short for the `d_pwm_max` ramp refused rather than condemning the fan, a seized fan declared failed with its zone's other channels floored, an output with no fan and a fan with no tach wire never kicked, a kick that changes no command while the solver is above it, the command never lower than the same run with the rule off and never outside `[pwm_min, pwm_max]` or past `d_pwm_max`, a healthy enclosure never kicked, and `start_duty: 0` reproducing the plant bit for bit | PR |

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
| `auth_verify_max` | `6` | uncached password checks (one PBKDF2 each) admitted per `auth_verify_window_s`, across every client address; beyond it a request that needs one gets `429`; `0` = no window limit |
| `auth_verify_window_s` | `60` | the sliding window of `auth_verify_max`, seconds (> 0) |
| `auth_verify_pending_max` | `2` | uncached checks admitted and unfinished at once (one runs, the rest wait); beyond it `429` (≥ 1) |
| `auth_verify_nice` | `10` | added to the nice value of the one thread that runs the checks (Linux; 0–19, capped at 19; `0` = the control loop's priority) |

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
  count. Recording a failure costs amortised constant time however many
  addresses have failed (expired addresses are dropped oldest first), so
  malformed headers from many addresses cannot turn the bookkeeping into
  quadratic work on the event loop. The page's 2 s poll is answered from the cache (on the event
  loop, never queued behind a password check).
- **Cost of an uncached check and its global limit:** one PBKDF2-HMAC-SHA256
  at 100000 iterations takes **1.23 s** on the Zero W (median of 5,
  1.23–1.55 s; 10000: 0.13 s, 25000: 0.31 s, 50000: 0.64 s, 200000:
  2.60 s; Python 3.13.5, armv6l, 2026-09-14). The per-client backoff does
  not bound clients on many source addresses (easy with IPv6), so the
  uncached checks are also limited globally: at most `auth_verify_max`
  start within any `auth_verify_window_s` and at most
  `auth_verify_pending_max` are admitted and unfinished at once, whatever
  the address. A request that needs a check beyond either gets `429` with
  `Retry-After` (the time until the oldest check leaves the window, or
  the queue times the last check's duration), no derivation and no failure
  charged to its address; a client in its backoff takes no slot; a cached
  login is answered as usual. The refusals are logged at most once per
  window. The defaults (6 per 60 s) bound a flood to about 7.4 s of PBKDF2
  per minute (12 % of the core). A legitimate uncached login during a
  flood can get `429` too (nothing tells it apart before the hash); the
  browser's retry succeeds once a slot frees, and a login cached before
  the flood is unaffected. Admitted checks run one at a time on the
  authenticator's own worker thread, whose nice value is raised by
  `auth_verify_nice` (Linux applies it to that thread only), so the
  scheduler favours the control loop. Measured on the Zero W with the DAS
  MPC (`tools/bench_step.py`'s DAS loop, 90 ticks per case) while a
  thread submitted wrong passwords from new addresses as fast as admitted:
  no flood p50/p99 231/584 ms; unlimited at nice 0 508/1075 ms; unlimited
  at nice 10 241/675 ms; defaults 247/617 ms.
- **Recommended iteration count on the Zero W:** the default `100000`.
  With the cache a user pays 1.2 s once per `auth_cache_s` per browser.
  A flood's CPU is `auth_verify_max` checks per window times the cost of
  one, so it scales with the count; at 100000 the defaults keep it near
  12 % of the core at a lowered priority, and a lower count would mainly
  weaken the hashes if the credentials file leaked. Above 200000 (over
  2.5 s per login and per admitted flood check) lower `auth_verify_max`
  in proportion; that is not recommended on this core.
- **Startup refusal:** `start_publishers` parses the whole `http:` section
  through `HttpSettings.from_section` before deciding whether to start
  (item 57: this covers `enabled` itself too, so `enabled: "true"` — a
  string, not a bool — is refused by name instead of comparing unequal to
  `True` and reading as off with no log line). With `enabled: true`, the
  service then loads the certificate and key and the credentials file (at
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
| `GET` | `/api/zones` | DAS: `{"zones": {zone: {…}}, "zones_in_fault": […], "degraded": bool}`; legacy: 404 |
| `GET` | `/api/model` | DAS: `{"thermal", "parameters", "calibration", "manual_calibrations", "store", "experiment"}`; legacy: 404 |
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
- `device_health` — what the fan- and device-health monitor last found
  (§3 "Fan and device health", §8 items 79 and 83): `devices` (per
  controller `label`, `device`, `serial`, `firmware`, `power_cycles`,
  `open`, `status_age_s`, `stuck_channels`, `absent_channels`,
  `not_pwm_channels`, `unconfigured_channels`, `flows`, `heartbeat`,
  `software_sensors` (§8 items 83, 113 — `null` until a control report has
  been decoded, `[]` on a device with no software sensors this daemon
  reads), `aquabus` (the state of the controller's aquabus, §8 item 92),
  and `active_profile` once one is published), `fans` (per channel `duty`,
  `rpm`, `voltage_v`, `current_ma`, `power_w`, `expected_rpm`,
  `expected_power_w`, `rpm_monitored`, `rail_monitored`,
  `power_monitored`, `unmonitored` — the rules that did not run for that
  output and why, §8 item 117; a flag is true only when that rule actually
  ran, which needs the measurement *and* the configuration it is judged
  against — `spin_up` (the spin-up kick's verdict for that output, §8 item
  75: `state` — `off` | `idle` | `turning` | `confirming` | `kicking` |
  `verifying` | `failed` — with `monitored`, `failed`, `attempts`, the last
  `rpm` and `duty`, and, where the rule is off for that output, the
  `reason`; a failed fan's line is in the top-level `problems` too), and
  `problems`), `host` (§8 item 103, extended for "the disk nobody
  watches": the board's own
  `cpu_temp_c`, the `air_c` reference it is compared against with the
  `air_temps` it was averaged from, the signed `divergence_c`, `load1`,
  `idle`, the `throttled` reading, the card's own
  `disk_free_gb`/`disk_used_pct` and `read_only`, and this board's own
  `faults`, `hints`, `problems` and `ok`), `problems` and `ok`. Only the
  board's *facts* — it is hot, it is throttling now, the filesystem is
  read-only — are in the top-level `problems` list `/api/health` shows;
  the divergence *hint* and the free-space *hint* stay in
  `device_health.host` (§3 "The board itself"). Empty with `ok`
  true before the first tick and with a source that has none (the
  simulator)
- DAS only: `limits` (`{"classes": {class: limit_c}, "bays": {bay:
  limit_c}}`, the limits in force) and `bays` (`{bay: {zone, occupied,
  class, serial}}`, the declarations in force). A legacy payload keeps
  its shape.
- `host` — host machine metrics (`aqua_bridge.hostinfo.collect_hostinfo`:
  `cpu_temp_c`, `load1`, `load5`, `load15`, `mem_used_pct`, `mem_total_kb`,
  `disk_used_pct`, `disk_free_gb`, `wifi_rssi_dbm`, `uptime_s`,
  `throttled` — the board's throttling state, §8 item 103, the one nested
  value: the decoded word with its `source` and, for a cached `vcgencmd`
  word, its `age_s`, or a `partial` reading whose `unknown` conditions are
  `null`; and `read_only` — whether the filesystem `disk_used_pct` and
  `disk_free_gb` describe is mounted read-only, from `/proc/mounts`
  (`hostinfo.read_mount_ro`), `null` when that cannot be determined;
  `null` per key when unreadable), refreshed at most every `host.interval_s` seconds
  through a cache the HTTP app owns (`publishers/http.py`, item 25); present
  in both modes

`/api/zones`: per zone `trusted`, `reasons`, `fault`, `fault_reason`,
`fault_since_ts`, `fault_elapsed_s`, `fault_ticks`, `trusted_streak`,
`in_closure`, `channels`, `channels_under_fallback` (the zone's own
`channels` that are currently held or ramped high, whether because this
zone is in fault or because a coupled zone's fault reaches a channel the
two zones share) and `policy` (`solver` | `hold` | `ramp_high` |
`coupled`, `diagnostics["zones"]` verbatim plus the derived
`channels_under_fallback`); `zones_in_fault` and `degraded` mirror
`cmd.diagnostics.zones_in_fault` and `/api/health`'s `solver ==
"degraded"` (item 22).

`/api/estimate`: per constrained bay `t_c`, `sigma_c`, `margin_c`
(`k·σ`), `soft_c`, `hard_c`, `limit_c`, `limit_margin_c` (`hard_c −
t_c`), `occupancy`, `class`, `zone`, `zone_trusted`, `calibrated`,
`source`, `q_w`; plus the estimator summary (status, per zone air
estimate, SMART counters). `/api/model`: `thermal` is
`diagnostics["thermal"]` (status, prediction error, per zone and bay the
coefficients with relative standard errors, plus per zone `pe_diag` — the
per-group diagonal beside `pe_min` — and `blocked`, the parts of the
`converged` rule the zone still fails, and per bay `se`, the absolute standard
error beside `rel_se` and `null` wherever `rel_se` is (§3, §8 items 110, 111);
`{"status": "off"}` without
`model_shadow`), `parameters` the static table of §3 (unit, bounds,
prior, identified from), `calibration` per bay (serial, calibrated,
`sigma_cal_c`, `calibration_source` — `smart` | `manual` | `null` — and the
calibration details), `manual_calibrations` the handheld readings still being
offered to the estimator (`{bay: {temp_c, ts}}`, item 23), `store` what the
model store loaded
(`{"source": "off"}` without a store), `experiment` the identification
experiment's status (running, target, phase, level, the base at the start,
the anchor the levels are drawn around now (`plan_base`) with `levels` and
`replan`, `excitation` per channel and `unexcitable` — how far each channel's
telegraph can move the model's PE monitor from where it sits (`rel_swing`,
`pe_reach`, `pe_bound`, `excitable`, `pe_reach_at_cap`, `excitable_at_cap`; the
constant bound is `pe_bound`, never `pe_min`, which is the thermal summary's own
*measured* eigenvalue) and the channels that cannot clear that bound at the
configured `ident_amplitude` — `holds_cover_window`, whether the schedule can
deliver that reach at all or a regression window averages two levels and the
monitor reads less (§3, §8 item 110), elapsed and remaining
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
- `device_health` — `{ok, problems}`, the short form of `/api/state`'s
  `device_health`: the pair a Home Assistant problem sensor needs (§8
  item 83). `ok` is true with an empty `problems`

JSON is the API. HTML is a thin, view-only client (the browser asks for
the basic-auth credentials once and sends them with every poll): it polls
`/api/state` and `/api/health` every 2 s (no websockets until 2W) and shows
Overview, Temps, Fans, Controllers, MPC and Host from those two — Controllers
being `/api/state`'s `device_health`: per controller the status age with any
stuck, absent, non-PWM or unconfigured outputs, the flow sensors, the aquabus
line (§8 item 129: "a device answers" / "lost `<n>` s ago" / "empty" / "never
connected" / "unknown", from `aquabus.state`) and the
active profile when one is published, then per channel rpm against the fitted
curve with rail voltage, current and power (a channel with a drift is marked
DRIFT), then every problem line. In DAS mode (`"bays" in
state`) it also polls `/api/estimate`, `/api/bays`, `/api/zones` and
`/api/model` and shows Drives (per-bay estimate joined with the declared
occupancy and class, and where that bay's sensor-to-drive map comes from —
`cal smart`, `cal manual` or `uncal`), Zones (trust, fault and which channels are
held or ramped) and Model (thermal identification status, model store, noise
index, experiment running, the calibrated bays by source and the handheld
readings still pending, item 23; plus, per zone, its identification status with
any `blocked` reasons and `pe_diag` values, and an "Unexcitable channels" line
from the running or last experiment's `unexcitable`, §8 item 121); those three
sections stay hidden on a legacy config.
The FAULT banner shows when `solver` is `fallback` or `fault`, or when a
poll fails; a **DEGRADED** banner names the zones from
`cmd.diagnostics.zones_in_fault` when `solver` is `degraded`. Controls are
JSON-only for now. The Host section reads `/api/state`'s `host` key (items
22, 24, 25).

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
| `POST` | `/api/calibrate` | `{ "bay": "b03", "drive_temp_c": 41.5 }` | DAS: one handheld drive reading for one bay (the stand-in for SMART); legacy: 400 |
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
- **`/api/calibrate`** (DAS, item 23): one drive temperature measured with a
  handheld thermometer, for an enclosure whose drives no SMART agent can read.
  Same authentication, same rate limit and same intent path as every other
  command. `400` for a malformed body, an unknown bay, a `drive_temp_c` outside
  `[estimator.calibrate_min_c, estimator.calibrate_max_c]` narrowed to the gate's
  `[temp_min_c, temp_max_c]`, or a legacy config;
  `409` when the reading would be meaningless or unsafe, the error naming which
  bay and why: `no_tick` (no estimate yet — the estimator has to have run once),
  `empty:<bay>` (the estimator calls the bay empty, so there is no drive a
  temperature could describe), `untrusted:<zone>` (the bay's zone is untrusted
  or in fault, and a map fitted to a reading the gate does not believe would
  bias every later estimate of that bay, which is what decides how hard the fans
  run). The three are read against the last completed tick, and re-read on every
  tick that offers the reading: one taken while the zone was trusted is held back,
  never absorbed, if the gate has stopped trusting the zone (or the bay has gone
  empty) by the time a tick would fold it in. An accepted reading is offered to
  the estimator for `estimator.smart_max_age_s` and reaches it exactly like a
  SMART sample of that bay (§3) — folded in once, by sample time, exactly as a
  repeated SMART reading is. **One reading is not a calibration:** the map is
  accepted at 20 fresh samples (§3), so the 200 body carries
  `{"ok": true, "calibration": {bay, calibrated, calibration_source,
  fresh_samples, samples_required}}` — the counts as of the last tick, this
  reading landing on the next one. It is visible in `GET /api/model` under
  `manual_calibrations` while it is offered, and as the bay's `calibration` with
  `calibration_source: "manual"` in `/api/bays` and `/api/model`, and on the page
  in Drives (`cal manual`) and Model. A second reading for the same bay replaces
  the first. Like every other accepted intent it aborts a running identification
  experiment — but a *refused* one does not: the bay, the range and the three
  refusals are checked before the experiment is touched. An accepted map
  survives a restart through the model store (item 104, §3 *Manual
  calibrations survive a restart*). The MQTT counterpart
  `{node_id}/cmd/calibrate/<bay>` exists but is **off** unless
  `mqtt.allow_calibrate` is set — this route has authentication of its own,
  MQTT has only the broker's ACL (item 105, §7).
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
`discovery_prefix` (`homeassistant`), `node_id` (`aqua-bridge`),
`allow_calibrate` (`false`; item 105, *Manual calibration over MQTT* below).
`host:` —
`interval_s` (5): how often host metrics are refreshed. `start_publishers`
parses the section through `validate_mqtt_section` before deciding
whether to connect (item 57): a wrong-typed scalar anywhere in it,
`enabled` included, is a named, logged setup error, never a silent read
as off or a bare `ValueError` from an ad hoc `int()`/`str()`.

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
- inbound, DAS mode and **off by default**: `{node_id}/cmd/calibrate/<bay>`
  (a raw number, °C — one handheld drive reading, the MQTT counterpart of
  `POST /api/calibrate`, item 105). Only with `mqtt.allow_calibrate: true`;
  see *Manual calibration over MQTT* below.
- inbound SMART: `{node_id}/in/smart/<serial>`, retained JSON `{serial,
  model, temp_c, ts_wall}` from `tools/smart_agent.py`, into the
  `SmartInbox` on the same broker connection
  (`MqttClient.add_topic_handler`)

Discovery config topics are
`{discovery_prefix}/{component}/{node_id}/{object_id}/config`, retained,
qos 1; `unique_id` is `{node_id}_{object_id}`; every entity carries the
availability topic and one device block. Entities:

- sensors `host_cpu_temp_c`, `host_load1`, `host_mem_used_pct`,
  `host_disk_used_pct`, `host_disk_free_gb`, `host_wifi_rssi_dbm`,
  `host_uptime_s`
- sensor `temp_<temp>` per `mpc.temps`; `rpm_<channel>` and
  `pwm_<channel>` (commanded PWM in %) per channel
- binary sensor `device_problem` (`device_class: problem`, diagnostic,
  §8 item 83): on whenever `health.device_health.ok` is false, that is
  whenever any controller reports a stuck output, an aquabus slot with no
  device behind it, a commanded output not in PWM mode or an unconfigured
  controller block, or whenever a fan has drifted from its fitted curve,
  its rail has sagged or its power is out of line with its duty. Its
  `json_attributes` are the whole `device_health` blob from the same
  retained state topic, so the per-controller and per-channel detail (and
  the flow sensors) is one tap away without an entity per output.
- binary sensor `host_problem` (`device_class: problem`, diagnostic,
  §8 item 103): on whenever `device_health.host.ok` is false, that is
  whenever the board has been above `host_health.temp_limit_c` for
  `temp_fault_s`, is throttling now, has sat further than `divergence_c`
  from the enclosure air for `divergence_fault_s` (only while its CPU is
  idle), the card at `host_health.disk_path` has been below
  `disk_free_min_gb` free for `disk_free_fault_s`, or that card's
  filesystem has gone read-only. Its `json_attributes` are the
  `device_health.host` blob: the board's temperature, the air reference,
  the load average, the decoded `get_throttled` word, and the card's own
  `disk_free_gb`/`disk_used_pct`/`read_only` (`disk_free_gb` also has its
  own sensor, `host_disk_free_gb`, above). The board — and the card it
  runs from — get an entity of their own so a hot Pi or a filling SD card
  is not read as a controller fault. Only the *facts* — hot, throttling
  now, read-only — also join the one `health.device_health.problems` list
  behind `device_problem`; the divergence hint and the free-space hint
  turn on `host_problem` alone, so `device_problem` keeps meaning
  "something is broken" — the free-space rule is a hint rather than a
  fact for the same reason divergence is: it can sit true for days once a
  card starts filling, and latching the daemon-wide flag for that long
  would be indistinguishable from a missing aquabus device.
- binary sensor `aquabus_problem` (`device_class: problem`, diagnostic,
  §8 items 92, 114, 115, 129): on whenever some controller's
  `device_health.devices[].aquabus.lost` is true — a device that had
  answered has now been missing for that controller's `bus_absent_s`. A
  bus that has never had anything on it (`aquabus.state: "never_seen"`,
  the normal state of an aquaero with nothing on its aquabus) or one
  reading empty for less than `bus_absent_s` never turns this on. Its
  `json_attributes` are the `aquabus` block of every controller in
  `device_health.devices` (`state`, `present`, `seen`, `absent_s`,
  `lost`, `temps_missing`).
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
- DAS mode: sensor `unexcitable_channels` (§8 item 121, `excitation()`):
  state is `extra.experiment.unexcitable` joined with `, `, or `none`
  when every channel can clear the model's PE bound at `ident_amplitude`
  from where it sits; `json_attributes` the full per-channel `excitation`
  mapping (`rel_swing`, `pe_reach`, `pe_bound`, `excitable`,
  `pe_reach_at_cap`, `excitable_at_cap`). A value, not a fault: no
  `device_class`.
- DAS mode, per zone: sensor `zone_status_<zone>` (`diagnostics.zones.
  <zone>.policy`: `solver` while trusted and fault-free, `hold` /
  `ramp_high` while its own fault holds or ramps its channels, `coupled`
  while it only carries a coupled zone's channels under fallback; `off`
  before the first DAS tick) (item 22); and sensor `model_block_<zone>`
  (§8 items 110, 111, 121): state is `diagnostics["thermal"]["zones"]
  [<zone>]["blocked"]` joined with `, `, or `none` once the zone has
  nothing left to wait on (`windows:<block>`, `pe:<block>`,
  `rel_se:<coefficient>`, `pred_err`, naming the gate and the group or
  bay it names); `json_attributes` the whole per-zone thermal summary
  (`pe_diag` included). Also a value: an unconverged zone is not a
  fault, so this is a `sensor`, never a `binary_sensor` with
  `device_class: problem`.

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
`{node_id}/in/smart/#` to the accounts that need it.

**Manual calibration over MQTT is off by default (item 105).** There *is* a
`{node_id}/cmd/calibrate/<bay>` topic (a raw number in °C, the counterpart of
`POST /api/calibrate`), but `mqtt.allow_calibrate` defaults to `false` and
nothing accepts one until the owner sets it: the topic is not in the
subscription list, and the parser refuses it a second time, returning no intent
at all, so a message that arrives anyway (a wildcard subscription, a replayed
session) never reaches the supervisor or the estimator. **Why it is off when
`cmd/limit` and `cmd/bay` are on**, given all three have only the broker's ACL
behind them: a limit or a declared occupancy is *policy* — a number the gate
re-clamps every tick, visible in Home Assistant as the entity that carries it,
bounded by the safety core. A calibration is not policy but *measurement*: it
fits the bay's sensor-to-drive map, so a wrong reading biases every later
estimate of that bay, and an estimate biased low makes the controller run the
fans slower than the drive needs. It is the one inbound payload that can quietly
reduce cooling, so it asks for a deliberate `true` in `config.yaml` rather than
riding in on the ACL by default. Its guards when it *is* on:

- `mqtt.allow_calibrate: true`, checked like every other scalar of the section
  (item 57), and refused as a named setup error unless **`mqtt.username`** is set
  with it: an anonymous connection cannot be given an ACL of its own, so "the
  broker's ACL" would be no guard at all. Read that guard for what it is — *a
  configured username is required*, checked once when the config is parsed. It is
  **not** a check that the connection is authenticated: nothing inspects the
  CONNACK or the broker's ACL, and a broker running `allow_anonymous true`, or one
  that takes the username without checking the password, accepts the same
  calibration. The opt-in plus a named account is a speed bump the owner sees at
  startup; enforcing the authentication is the broker's job.
- a **retained** calibration is ignored and logged, exactly as a retained
  experiment `start` is — and for a sharper reason: a retained number is
  redelivered on every reconnect and would be refitted into the bay's map on
  every restart, which is the failure item 104 is about.
- **no Discovery entity.** Fifteen more `number` entities would be fifteen more
  things Home Assistant republishes on its own restart — the retained-message
  problem in another costume. The topic is for a script or an automation
  publishing one live reading, not for a slider.

Past the parser it is the ordinary `Calibrate` path: the supervisor's own checks
(the bay, the range, `no_tick` / `empty:<bay>` / `untrusted:<zone>`) apply
unchanged, and a refused reading still costs no running identification run.

Everything above is pinned by tests with a fake client
(`tests/test_mqtt_ha.py`, §4.9): the entity table, every payload's fixed
keys, the command topics and the intents they produce, the SMART route,
the qos and retain flags, the availability transitions. What needs a
broker is `tools/ha_check.py` (a read-only checklist run on the Pi
against the real broker, §10 step 12) and `tests/test_mqtt_live.py`
(marker `mqtt_live`); the run against the owner's Home Assistant is §8
item 21.

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
  `tests/test_bench_budget.py`; `pytest -m pi` passed on the Zero W at the
  time, where the DAS MPC measured p99 507–552 ms — it measured 609–615 ms
  after items 3, 8, 9 and 10, which is item 73).
- Live MQTT and Home Assistant checks use the owner's Home Assistant
  broker; its host name is in `private.md` (item 21).
- An experiment holding back a cooling increase was deferred to the
  Zero 2 W upgrade (item 52); done 2026-09-16, the levels are re-planned
  from the live solver demand every tick (`ident_replan`, §3), at a
  measured ≈ 10 µs per tick here and only while an experiment runs — small
  enough that the board is unlikely to be what decides it, which the
  Zero 2 W run confirms (item 52).

Owner decisions (2026-09-14, later the same day):

- Item 73: optimise the DAS step on the Zero W; if that does not bring
  the DAS MPC under `mpc.budget_ms` 600, raise the budget to 1000 ms.
  The DAS MPC solves every third tick (`mpc_every_ticks: 3`).
- Item 68: replace the hard sigma floor with a soft floor.
- Item 56: no token cookie. A verified login is cached for
  `http.auth_cache_s` (300 s by default), so PBKDF2 runs once per cache
  period per credential, not per request; the page's 2 s poll reads the
  cache.

Owner decision (2026-09-15):

- The daemon reads and writes the aquaero and the Quadro through
  **hidraw**; the `aquacomputer_d5next` kernel module is not used.
  Through hwmon every `pwmK` read cost 210 ms and every write 420 ms,
  about 5 s per tick with 8 outputs; over hidraw a read takes the
  unsolicited status report (about 1 ms) and one feature report writes
  every channel of a controller (item 74, §3 Track B). The DKMS package
  stays in `deploy/` for later; `install-pi.sh` no longer runs it (§9).
- `ctrl_gap_ms` (the wait between control operations on one controller)
  stays a per-device config key; its default depends on the device kind:
  **100 ms for the aquaero, 0 for the Quadro**. Measured on the Pi:
  back-to-back aquaero writes failed with `EPIPE` at 0 ms (18 of 20) and
  25 ms (15 of 30), none at 50, 75, 100 or 150 ms (30 each); the Quadro
  wrote 20 of 20 at 0 ms (§2 "hidraw check").
- Item 21 (live MQTT and Home Assistant check) is deferred: it does not
  block production. Item 78 (sending the driver fix upstream) is deferred
  too.

Owner decision (2026-09-16):

- **Item 21 is no longer deferred** (it was deferred on 2026-09-15, above).
  The live MQTT and Home Assistant check runs against the owner's Home
  Assistant broker (host in `private.md`). Everything that does not need
  the broker is done offline first — the Discovery contract in
  `tests/test_mqtt_ha.py`, the read-only `tools/ha_check.py`, and the
  opt-in `mqtt_live` suite — so the live run is a short confirmation
  (item 21's remaining text is the commands and what to look at).
- **Flow stays out of `PlantObservation`** (item 91). The DAS has no
  coolant loop, so a flow reading steers nothing and would only add a
  field every consumer must ignore. Flow is still decoded
  (`hw/aquacomputer.py`: aquaero `flow1..3`, Quadro `flow1`), shown by
  `tools/aquacomputer_probe.py`, and published next to the other device
  health as `device_health.devices[].flows` (a raw value, or `null` for
  `0x7FFF`) — never in the observation, so it never reaches the gate or
  the solver. No config key binds it: a `flowN` name anywhere in a device
  entry (`fans.<ch>.pwm`, `fans.<ch>.rpm`, `temp_map`) is rejected with a
  message saying flow sensors cannot be bound and that an aquaero's
  hwmon-era `fan5`/`fan6` are aquabus tachometers now (a Quadro's outputs
  1–2 on its aquabus), not the flow sensors the driver called by those
  names. If a coolant layout ever comes back, it gets its own decision;
  `inputs` is the place it would go, not `temps`/`rpm`.
- **One controlling controller, slaves behind it.** The daemon supports
  exactly one controlling controller — today the aquaero — with slave
  devices, the Quadro among them, attached to it over aquabus. Two
  independent controllers are not supported yet, though they might be
  later. A controller reachable over two access paths at once (aquabus
  and its own USB, both live) is an out-of-design mode: reaching an
  aquabus-attached Quadro over its own USB too proved unstable (§2
  "Supported topology"), and the daemon does not need it in that form.
  Hidraw support for the Quadro's own USB port stays in the code and is
  simply unused in this installation (item 106).

- **The Pi's own board is a health signal, never a model input**
  (item 103). On the owner's Zero 2 W (64-bit trixie, kernel 6.18.50-v8)
  `/sys/class/thermal/thermal_zone0` is `cpu-thermal` and read 47.2 °C at
  idle, and `vcgencmd get_throttled` returned `0x0`. Measured there since:
  the firmware's `get_throttled` **sysfs attribute does not exist** on that
  kernel (the `soc:firmware` platform device carries none, and `find /sys
  -iname '*throttl*'` finds nothing), so `vcgencmd` — 3.3 ms median, 3.8 ms
  p95 per run — is the only source of the whole word there; `hwmon1` is
  `rpi_volt` with the single attribute `in0_lcrit_alarm`, the under-voltage
  condition alone, at 0.17 ms median / 0.23 ms p95. The board is **not**
  part of the thermal model: about a watt against the drives' tens of
  watts, and its reading is dominated by its own self-heating, which moves
  with CPU load. It must never become a solver input, a zone air sensor or
  a model node. It **is** a health signal, for two reasons: the board
  throttles around 80 °C, and it is the first thing to notice if the Pi
  sits in hot exhaust; and at idle its temperature should track the
  enclosure air, so a large, sustained divergence is evidence that
  something is wrong with the air sensors or with the board's placement —
  a hint about where to look, never a verdict about which of the two.

### 8.2 Open — no DAS hardware needed (dev machine, CI, the Pi, the PC)

11. **Done** (2026-09-16): the one-step prediction is made for, and scored
    against, only the bays that have a row on that tick (an estimate, a
    trusted zone, not empty), so a drive in a faulted zone is no longer
    evidence about a model the solver never plans for it (§3, validity gate
    and model fallback).
12. **Done** (2026-09-16): a hot-swapped bay's identified coefficients start
    over from the prior. The estimator reports `swapped: true` for one tick on a
    bay whose occupancy crosses the `empty` boundary in either direction or whose
    **mean innovation** over its trusted proximal members passes the fast-swap
    rule's own thresholds (`jump_min_c`, `jump_sigmas`) — each member predicted
    where the filter says *it* sits, its node and, on the fused layout, its own
    placement offset, against the variance of that mean (item 109);
    `step` hands those bays to `thermal.update(reset_bays=...)`, which with
    `model_reset_on_swap` (new flat key, default `true`) resets the bay's `g0`,
    `k`, `q_s`, covariance, counters, PE monitor and window in progress, plus the
    zone air block's window (it anchors on that bay's heat through those very
    coefficients). The zone's *status* is left alone: demoting it would park the
    DAS MPC in its PI-like fallback until the next experiment, while the bay
    simply relearns like a new one. A `frozen` zone is skipped — it never moves
    its coefficients, so a reset there would strand the bay at the prior.
    The consequence of leaving the status alone, for the record: the zone keeps
    its `converged` badge, its `conv` level and its exponentially-weighted
    `pred_err` from before the swap, so the validity gate (§3, `check_model`)
    accepts the zone while that one bay's block is back at the generic prior,
    for the few windows it takes the prediction error to climb. Cooling is not
    reduced by it — the estimator's own `reset_drive_var` and jump inflation
    make the swapped bay's margin dominate — but the diagnostics claim more
    confidence than the model has until the bay is re-identified.

    The **mean** is the point, not any one sensor. The per-sensor fast-swap test
    is sequential, so with a redundant pair — two sensors at different placements,
    disagreeing with the bay's one sensor node — one of them jumps on almost every
    tick under fan excitation (measured: 5689 of 5760 ticks on b10, 5759 on b03,
    the same pathology §3 records for the `rich` preset), and resetting on that
    would leave such a bay permanently unidentifiable. A swap moves every member
    of the bay together and survives the mean; a disagreement between placements
    cancels in it. The per-sensor rule itself is unchanged: it still inflates the
    variances and drops a correlation association.

    *Corrected by item 109 (2026-09-17):* "the mean of a disagreeing pair sits
    where the node already is" was wrong by half the disagreement. The mean
    reading of a pair is the node **plus the mean of the members' placement
    offsets**, so comparing it with the bare node left `Δs·rise/2 + Δb/2` in the
    statistic and none of the offsets' variance under it, and a pair further
    apart than `2·jump_min_c` was `swapped` on every tick — which is what
    item 102's dead zones were.

    Measured on `sim/das.py` (the identifiability harness of
    `tests/test_thermal_ident.py`: group experiments, `basic` physics, 8 h, the
    drive in b06 swapped at 4 h for one with `k = 1.2` and `g0 = 0.6` against
    the 0.5/0.3 that came out, seeds 2 and 3): the old behaviour ends at
    `k = 0.57–0.63`, 47–52 % below the new drive's truth, because the window
    forgetting has to walk the old drive's value across; the reset ends at
    `k = 1.12–1.18`, within 2–6 %, from 119 windows in the 4 h after the swap.
    No other bay is touched (`k.b05` moves by less than 0.01 between the two
    runs), and one swap produced exactly one reset.
13. **Done** (2026-09-16): `mpc.model_split_channels` (needs `topology`,
    default `false`). A fan group of more than one channel in a zone carries
    one extra air-block coefficient per channel beyond the first,
    `Es.<zone>.<group>.<channel>`, in the same regression:
    `Q_zG = E_zG φ_zG + Σ_{ch ≠ ref} Es_zGch (φ_ch − φ_zG)`. The extra
    regressor is **exactly zero** while the group's channels hold the same
    duty, so regulation and a whole-group phase leave the shared-`E` model
    term for term (the prediction and the Jacobians are bit-identical at the
    prior split) and only an experiment's single-channel phases move it.
    What it learns is a redistribution: the implied per-channel
    `E_ch = E_zG w_ch/W + Es_ch − (w_ch/W) Σ Es` sums to `E_zG` whatever the
    split, so the group as a whole never changes. The convergence rules read
    the group coefficients alone (`Es` is not a gain for `rel_se`;
    excitation and the PE monitor still see `φ_zG`), so the switch cannot
    stop a model converging. `GET /api/model` reports `e_per_channel` per
    zone, and a store file written with the switch the other way round is
    converted rather than dropped (shared keys keep their value, variance
    and covariances; a new key starts at its prior; a corrupt coefficient
    on that path raises `ValueError` like every other malformed field, so
    it costs the thermal section and not the whole seed). After every
    window, and when a stored memory is read, the split is projected: it is
    scaled back by the largest factor in [0, 1] that leaves every `E_ch ≥
    0`, since `Q_zG = Σ E_ch φ_ch` exactly and a fan never cools less than
    nothing — direction and total kept, a feasible split untouched, so a
    mis-split group can never reverse a zone's modelled airflow or flatten
    its Jacobian row. Measured on the truth
    simulator with a group whose channels differ by 60 % (84 W/K against
    26, a ratio of 3.23 where the count-weighted prior says 2.0): after
    whole-group phases and then each channel alone, 78.6 / 28.6, ratio 2.75,
    group total 107 against 110 (`tests/test_thermal_split.py`).
14. **Done** (2026-09-16): `mpc.fan_curve_online` (needs `topology`,
    default `false`) fits each fan model's PWM → RPM curve online
    (`control/fancurve.py`, pure). Settled pairs only: a channel
    contributes `(pwm, rpm)` once its commanded duty has held for
    `fan_curve_settle_s`, into PWM bins per fan model that cap at a bounded
    exponential mean, so a worn or replaced fan is followed rather than
    outvoted by a year of old samples. Every `fan_curve_refit_s` the same
    grid over `deadband × exponent` `tools/fit_fans.py` uses (one
    definition, shared) is searched with `rpm_max` in closed form, and the
    fit is accepted only with enough bins over enough PWM span and a
    relative RMSE at most `fan_curve_max_rmse_frac` — otherwise the
    previous curve, or the configured one, stays, so a stalled or lying
    tachometer widens the residual instead of moving the model. Accepted
    curves go into `solver_memory["fan_curves"]`, the store section that
    was written and validated but never produced or read, and from there
    the thermal model's identification and the DAS MPC's prediction plan on
    them (`u0.<m>`, `n.<m>` in §3's table) and they survive a restart. The
    fitted `rpm_max` is reported only: `model_use_rpm` keeps normalising the
    tachometer by the commissioned `fan_models.<m>.rpm_max`, because a
    `rpm_max` fitted to those same readings would make `φ` self-normalising
    and hide a fan that has lost speed.
    `GET /api/state`'s `fan_curves` diagnostics name the curve in force per
    model and where it came from (`fit` | `store` | `config`). Which other
    readers follow the fit, and which stay on the configured curve on
    purpose, is item 107 and §3's *Fan curve* table.
15. **Done** (2026-09-16): `modelstore.document_from_fit` converts a
    `tools/fit_model.py` report (`kind: aqua_bridge.thermal_model`, whose
    `memory` is already the thermal memory) into a store document: the fit
    becomes `thermal`, `fan_curves` / `calibration` / `bays` are empty (a
    fit of a recording knows nothing about the calibrations or the bay view
    of the machine that loads it) and `saved_wall` is the report's
    `generated_at`, so the fresh / stale rule measures the age of the *fit*.
    `tools/fit_model.py --store-out PATH` writes it, and `load` also accepts
    the report itself in the store's place. The report carries
    `store_fingerprint`, the store's own structure fingerprint of the config
    it was fitted against, and that becomes the document's fingerprint — so
    a report of another machine's structure (a different `dt`, fan count,
    sensor role or zone inlet, none of which the thermal model's own
    fingerprint covers) is refused exactly as a store file is, and a report
    from a tool older than that field is refused too (re-run the fit). The
    one thing it warns about: the persister replaces the file with a store
    document at its first save, so keep the report elsewhere.
16. **Done** (2026-09-16): `mpc.model_freeze` (needs `topology`, default
    `false`). With it on, a zone that reaches the `converged` rule is
    entered `frozen` instead, and a zone that is already `converged` when
    the switch is turned on freezes at its next closing window — so the
    switch acts on a model that has already converged without a restart or
    a store file, which is the case it exists for. Online identification
    never moves a frozen zone; the
    DAS MPC's validity gate accepts `frozen` exactly as it accepts
    `converged`, and a stale store file's hold counts `frozen` as
    re-confirmed. It freezes a good model only: a frozen zone whose
    prediction error crosses the `suspect` rule still becomes `suspect` and
    learns again, so the switch cannot hold a model the data has
    contradicted (§3 *Identification*).
17. **Done** (2026-09-16): the fast-swap rule now needs a bay's proximal
    members to agree before it fires (§3, *DAS thermal model and estimator*),
    so a bay with a redundant pair calibrates. Estimator accuracy on the
    `rich` sim preset. Measured over seeds 1–8 in the noise scenario's
    window, `tests/test_noise_regression.py`:

    | | uncalibrated bays | worst per-bay error | noise ratio |
    |---|---|---|---|
    | before | 2 of 16 on 7 of 8 seeds | 5.2–18.5 °C | 0.43–1.30 |
    | after | none, on every seed | 0.27–7.6 °C | 0.29–1.18 |

    The cause was not the calibration but the fast-swap rule: the example's
    two bays with a redundant proximal pair (`b03`, `b10`) draw different
    placements on `rich`, their readings disagree by a couple of degrees for
    the whole run, one of them tripped the rule on nearly every tick, and
    `jumped[bay]` gates SMART out of the calibration — so those bays never
    calibrated and their estimates sat on the prior map, 6–8 °C off, which is
    what the MPC then over-cooled against. With the agreement test every bay
    calibrates on every seed. `RICH_BOUND` drops 1.35 → 1.25 and a nightly
    test asserts that every bay calibrates.

    What is left is **not** an accuracy defect. The residual error against
    the physical drive node is the preset's per-drive SMART offset
    (±2 °C, `sim/das.py`): the estimator is calibrated to what the drive
    *reports*, which is the only temperature the daemon can see and the one
    the limit is written against, so that offset is invisible to any
    controller. Against the drive-reported temperature the rms error is
    0.12–1.97 °C over the seeds. The one outlier inside that (bay `b10`,
    up to 3 °C on two seeds, over-estimating, i.e. the conservative
    direction) is the single sensor node per bay: two proximal sensors at
    different placements cannot both be represented → item 101. The nightly
    accuracy test bounds both directions, not one rms: 2.5 °C rms over the
    bays (which `b10`'s +7.62 °C on seed 2 fits inside) plus a signed
    per-bay floor of −1.0 °C, because an *under*-estimate is the direction
    that regulates a drive hotter than the daemon believes (measured worst:
    −0.27 °C). `RICH_BOUND` is asserted on six of the eight seeds: seeds 2
    and 5 skip it because `b10`'s over-estimate drives their hot zone to
    `u* = 1.0`, where the uniform-curve ratio is 1.0 by construction; their
    margins are still asserted, and their estimates by the accuracy test.
18. **Done** (2026-09-16): a correlation pair now has to keep proving itself.
    An association is a claim about correlation, so the claim is re-tested with
    the statistic that made it: the bay series and the serial's SMART history go
    on being recorded after the pair is accepted, every evaluation (60 s)
    re-scores the pair against its own bay, and `associate_drop_checks` (3, new
    key) consecutive scores below `associate_drop_corr` (0.3, new key) end it.
    Keeping a pair deliberately asks less than choosing one: measured on the
    truth simulator a correct pair scores 0.71–0.97 over the window and dips to
    0.37 through a quiet one, while a wrong pair's median is −0.27 to +0.24.
    Every drop — re-check, hot swap, jump, silence — now also forgets the
    serial's SMART history, so the pair has to win the acceptance rule again
    over a fresh window, and so does *acceptance itself*: the window that made
    the pair does not get to sit its own first re-check, which therefore scores
    a fresh window. And until a correlated pair has passed one re-check its
    calibration is **not used** (`calibrated` stays false, `σ_cal` stays at
    `sigma_uncalibrated_c`, and `calibration.accepted_once` reads false, so the
    thermal identification will not convert a row with that map either): the
    first window is exactly the one the re-check cannot judge yet. The cost is
    that an undeclared serial takes about two windows, not one, before its
    calibration is used. A `ver` flag is per pair, not per bay: a declaration,
    and every drop, clears it. A declared serial takes part in none of this — it is the
    owner's statement, and its band stays `smart_reject_c`.

    A tighter absolute band for a correlated serial was tried first and dropped:
    it cannot tell the two apart, because an uncalibrated bay's estimate carries
    the prior map's own offset, so a *correct* pair's SMART is several °C away
    too (`smart_reject_corr_c: 3` cost 11 of 12 correct associations on the
    example config).

    Measured on `sim/das.py` (example config, `basic` physics with sensor noise,
    SMART with activity bursts, three bays given a neighbouring drive's serial
    as if the correlator had guessed wrong, two correct pairs as a control,
    2000 ticks, seeds 9 and 11): without the re-check 5 of 6 wrong pairs were
    held for the whole run, fed 121–205 rows into the bay's RLS and **every one
    of them reached an accepted calibration on the wrong drive**. With it all
    six were dropped at 3425–3725 s (the first evaluation with a full window)
    after 53–60 rows, and only one of the six ever had its calibration used —
    which it lost at the drop, the bay's map reverting to the prior in the same
    tick. Neither control pair was dropped in any run, and the declared-serial
    calibration run of `tests/test_estimator.py` is unchanged (12+ of 15 bays
    accepted, estimates within 1 °C).

19. **Done** (2026-09-16): occupancy debounce. A bay whose proximal members
    all go untrusted keeps its occupancy state, its `pending_empty_s` and its
    `pending_occupied_ticks` until the blindness has lasted
    `estimator.occupancy_hold_s` (new key, 30 s = 6 ticks at `dt = 5`); only
    then does it fall back to `unknown` with the `reset_drive_var` insert
    variance, exactly as before. The count runs on every blind tick and
    restarts when any member reports again, so `occupancy_hold_s: 0` is the
    old rule bit-for-bit. The per-bay diagnostics show `pending_unknown_s`,
    which is a *pending* transition: a bay that has already reached `unknown`
    counts nothing and reports 0 however long the blindness lasts.
    and `control/ident.py` counts it as a pending transition, so an
    experiment does not start into a dropout. A zone with no filter state at
    all is still `unknown` at once. Measured on `sim/das.py` (example config,
    `basic` physics with sensor noise, 2 % per-tick dropouts on every
    DS18B20, three bays pulled at t = 60 s, 900 ticks, seeds 1-3, both DAS
    solvers): those bays leave `empty` 4-9 times per run without the
    debounce and 0 times with it (once on `pi` seed 3, a dropout longer than
    the hold), which is 879-1225 ticks of a bay wrongly constrained at the
    insert variance against 0-105. Nothing else moves: 0 limit violations
    either way, the same hottest drive to 0.01 °C, and the mean noise index
    equal or 0.16 dB lower. It is a loudness fix, as the item said.
20. **Done** (2026-09-16): the settle timers ride in `model.json` as
    seconds already settled (`ident_settle`), `persist.apply_seed` subtracts
    the daemon's outage from them and drops them when the file is stale, its
    age unknown or the outage longer than the new
    `ident_settle_resume_max_gap_s` (the warning names which of the three it
    was), and a zone spends what is left only when
    it is trusted and fault-free again, minus the time that took — so no
    second the daemon did not observe is counted as settled (§5).
    `build_model_store` takes the supervisor-backed `ident_settle` callable
    as a parameter and hands it to `ModelPersister`, so a caller other than
    `main` cannot end up with a persister that writes the section empty.
    `plan_tick`
    marks the plan pending and `record_tick` clears it, so a start between
    the two arms offset 0 one tick further out. A restart still never resumes
    a running experiment, and `bay_settle_s` deliberately still runs again
    (occupancy restarts `unknown`: a drive may have moved while the daemon
    was down).

21. Live MQTT and Home Assistant check against the owner's Home Assistant
    broker (host and credentials in `private.md`). Needs the Pi, the broker
    and Home Assistant, not the DAS. No longer deferred (owner, 2026-09-16,
    §8.1). Everything that does not need them is done: the Discovery
    contract is pinned in `tests/test_mqtt_ha.py` (§4.9), `tools/ha_check.py`
    checks a live broker read-only against the daemon's own config, and
    `pytest -m mqtt_live` runs the publisher against a real broker. What is
    left is a confirmation.

    **On the Pi**, as the service user, from `/opt/aqua-bridge`, with the
    daemon running (`--config /etc/aqua-bridge/config.yaml` throughout):

    - `.venv/bin/python tools/ha_check.py --config /etc/aqua-bridge/config.yaml`
      — the checklist. Exit 0 and "0 problem(s)" is the whole claim: every
      expected entity announced and retained, the availability topic
      `online`, the state blob carrying what every template reads. Add
      `--verbose` for every entity with its value, `--strict` to fail on the
      warnings too. `--verbose` also prints the broker host in the header:
      that host lives in `private.md`, so paste the plain run's output, not
      the verbose one, into the PR or an issue.
    - `AQUA_BRIDGE_MQTT_TEST_HOST=<broker> AQUA_BRIDGE_MQTT_TEST_USERNAME=<user>
      AQUA_BRIDGE_MQTT_TEST_PASSWORD=<password> .venv/bin/python -m pytest -m
      mqtt_live -q` — the publisher end to end against that broker. Both
      signals are needed: the variable **and** `-m mqtt_live`, so a variable
      left exported in a shell cannot make an ordinary suite run open a
      session on the live broker. It uses its own `node_id`
      (`aqua-bridge-test-<pid>`) and deletes its retained messages
      afterwards, so it is safe to run next to the live daemon; if it cannot
      delete them it prints the topics it left, to clear with
      `mosquitto_pub -r -n -t <topic>`.
    - the three commands, each of which names its topic and payload, says
      what stands after the message, and waits for a typed `yes`:
      `tools/ha_check.py ... --send cmd/mode manual`, then
      `--send cmd/pwm/<channel> <duty>`, then `--send cmd/mode auto`. Each
      prints the state topic again afterwards, so the effect is visible
      without Home Assistant. **Both of the first two stand until something
      clears them**: manual mode takes the solver off the fans, and the PWM
      override replaces the solver's duty on that channel
      (`Supervisor.compose` only refuses one in fallback). So pick `<duty>`
      at or **above** that channel's current duty — the same run prints it,
      `cmd.pwm.<channel>` under "home assistant would show" — and if the
      sequence is interrupted, a declined prompt or a dropped ssh session
      included, `--send cmd/mode auto` puts it back and clears every
      override.

    **In Home Assistant**: one device `aqua-bridge` (MQTT integration → the
    device page). Confirm the entity list of §7 — the host sensors, a
    temperature per `mpc.temps`, RPM and PWM per channel, `Controller
    problem` as a diagnostic, the setpoint numbers (legacy) or the
    `limit_<class>` numbers and the per-bay drive sensors (DAS) — and that
    they have values, not "unknown". Move a limit or a setpoint number in the
    UI and watch `/api/state` (or the next `ha_check` run) follow within a
    tick. Put the daemon in manual and confirm the `pwm_cmd_<channel>`
    numbers appear, then set it back to auto and confirm they disappear
    rather than linger. Tap `Controller problem` and confirm its attributes
    carry the whole `device_health` blob (item 83). With the PC's SMART agent
    running, confirm the drive temperatures arrive through the broker
    (`ha_check` lists the serials and their age). Finally `sudo systemctl
    stop aqua-bridge`: every entity must go unavailable within seconds (the
    LWT), and come back on start.

    **What a failure would mean.** Entities missing altogether: the broker
    account may not be allowed to publish retained messages under
    `{discovery_prefix}/#`, or Home Assistant's MQTT integration uses another
    discovery prefix than `mqtt.discovery_prefix`. Every entity missing *and*
    nothing on `{node_id}/status`, while Home Assistant shows the device
    working: that is `ha_check`'s own account, not the daemon — the tool
    subscribes with the `mqtt:` credentials, and a publish-only ACL leaves it
    subscribed to nothing. It says so (`the broker refused the subscription
    to …`) and exits 2; grant that account read on `{node_id}/#` and
    `{discovery_prefix}/#`. Entities present but permanently unavailable: the
    availability topic is not readable by Home Assistant's account, or the
    daemon never connected (`/api/health`'s `mqtt_connected`). Entities
    available but "unknown": the state blob and a template disagree —
    `ha_check` names the missing path, and that is a bug here, not a Home
    Assistant setting. Not this, though: a path the state blob does not carry
    but whose template has a `| default(...)` is listed separately as reading
    its default and is neither a problem nor a warning — `model_status` is
    `off` on a healthy daemon with `model_shadow: false`, and the per-bay
    sensors read their default until the first DAS tick. "Announced but not
    expected": an entity in Home Assistant that this config does not have.
    The daemon deletes the *other control modes'* configs of the config it is
    running, so this is either another build or an entity dropped from an
    older `config.yaml` (a bay, zone, channel, temp or drive class) whose
    retained config nothing deletes; confirm it really is gone from
    `config.yaml`, then clear it with
    `mosquitto_pub -r -n -t <the config topic ha_check printed>`. A number
    that moves in the UI and changes nothing: either the broker drops the
    command (an ACL on `{node_id}/cmd/#`) or the supervisor refused it (the
    journal logs the rejected intent, like HTTP's 409). No SMART: the agent's
    `--node-id` or its broker account, not the daemon (§10 step 13).

22. **Done:** `GET /api/zones` (§6 View) and the HA entity
    `zone_status_<zone>` (§7), both reusing `diagnostics["zones"]`.
23. **Done** (2026-09-16): `POST /api/calibrate {bay, drive_temp_c}` —
    calibration with a handheld thermometer when SMART is absent. Same
    authentication, rate limit and intent path as every other command
    (`Calibrate` in `control/intents.py`, `Supervisor._check_calibrate`); the
    bay and the range `estimator.calibrate_min_c` / `calibrate_max_c`
    (narrowed to the gate's absolute range) are validated, and the reading is
    refused with the reason named when it would be meaningless or unsafe
    (`no_tick`, `empty:<bay>`, `untrusted:<zone>`, §6 *Control*) — all of it
    before a running identification experiment is touched, so a refused
    reading costs no experiment, and re-checked on every tick that offers the
    reading. An accepted reading rides to `step` as
    `obs.inputs["calibration"]` and reaches the estimator exactly like a
    SMART sample of that bay, keyed by bay instead of by serial (§3
    *Manual calibration*): one of the 20 fresh samples an accepted map needs,
    counted in its own `manual_used` / `manual_rejected` totals, and it does
    not displace a bay's SMART calibration until that one is accepted.
    Visible in `GET /api/model` (`manual_calibrations`,
    `calibration_source`), in `GET /api/bays`, in the 200 body's
    `calibration` progress and on the page (Drives `cal manual`, Model).
    Persistence is item 104 (**done**) and the MQTT counterpart item 105
    (**done**, off by default).
24. **Done:** the HTML page shows drive estimates, bays, zone and model
    status (§6 View, `publishers/static/index.html`), reusing the existing
    `/api/estimate`, `/api/bays`, `/api/model` views plus the new
    `/api/zones`.
25. **Done:** `/api/state` now carries a `host` key (§6 View); the HTML
    Host section reads it instead of showing dashes.
26. **Done:** `test (latest)` reached 10–11 min on slow runners against the
    11-minute guideline; `tools/ci_pytest_shards.py` now runs several
    pytest processes concurrently, each on a disjoint deterministic slice
    of `tests/test_*.py`, inside both `test` matrix jobs (§12).
27. **Done** (2026-09-16): the sine's turning points never held a
    plateau close to `stuck_s`. `tests/test_gate.py` now holds `prox_a2`
    (bay a2's only sensor, so the airflow branch is its Stuck rule's
    only evidence path) at one exact code for the whole 1100-tick run,
    several times `stuck_s`, while zone za's fans step from 0.2 to 0.9
    and stay there — a real net airflow move over every later window.
    Two runs of that scenario: with za's air warming against the rising
    airflow by more than `stuck_air_oppose_c` the reading is spared for
    the whole run, and with za's air flat it *is* branded Stuck, from
    one tick on and then continuously. So the plateau run is spared by
    the designed protection (`_air_opposes`), not by the absence of
    evidence, and the pair would fail if the Stuck rule were deleted.
28. **Done** (2026-09-16): `tools/bench_step.py --sim-plant das` takes
    `--sim-preset` (`basic`, default, or `rich`, item 17) and reports the
    preset it actually ran in `plant.preset`, instead of the literal
    `"basic"` regardless of what ran.
29. **Done** (2026-09-16): the stale docstrings, checked against the code
    and rewritten. `noise.py` named `tools/fit_fans.py` a later milestone;
    it exists and fits `rpm(u)` per fan model from a recording today (its
    output is still copied into `fan_models` by hand -- wiring it into the
    model store automatically, `fan_curves`, stays item 14). `thermal.py`
    said the fan group's shared `E` split per channel waits on
    experiments (later milestone); `control/ident.py`'s active
    identification experiments exist and already run each channel of a
    group alone in turn, so the data to split `E` exists -- the split
    itself is still open (item 13), and the docstring now says so instead
    of naming experiments as unbuilt. `FanSpec.forbidden_pwm` said the
    solver that honours it was a later milestone; the DAS solver
    (`control/solver_das.snap_bands`) has honoured it since that solver
    shipped, on both its PI-like and MPC paths.
30. **Done:** README carries a condensed, numbered bring-up checklist
    matching §10, with every command and flag checked against
    `deploy/install-pi.sh`, `deploy/host-usb.sh` and the `tools/*.py`
    `--help` output (README.md).
53. **Done** (2026-09-16): the soft and hard rules read `T̂` and only the
    absolute abort, measured against the raw limit, adds `k·σ` back (§5).
    What moved and what did not: the *soft envelope* is the rule that binds
    first under the shipped defaults, and it moved up by `k·σ` — that `k·σ`
    was the double count the item is about; the absolute abort
    (`limit − ident_abort_below_limit_c`) is bit-identical to before and
    still stops an experiment before `hard`. On the truth simulator an
    experiment keeps the same worst true margin as the same seed without one
    and never takes a drive over a limit; a run driven onto the envelope by a
    rising inlet aborts with `envelope:` and takes no drive over its limit
    after the abort either, on `ident_levels: above` and `symmetric` alike
    (`tests/test_ident_sim.py`, numbers in §5). The module
    docstring no longer defends the old formulas. Experiments count `k·σ`
    twice: `control/ident.py` checks `T̂ + k·σ` against `soft` and `hard`,
    which already subtract `k·σ`. Now that PI-like DAS settles at `soft`, a
    settled enclosure is refused by the start band (`ident_start_band_c`) and
    sits on the `ident_max_over_c` abort edge.
54. **Done** (2026-09-16): `mpc.ident_abort_below_limit_c` (1.0 °C, > 0,
    §3). `control/ident.py` hardcoded `ABORT_BELOW_LIMIT_C = 1.0` (the
    absolute abort margin below the limit).
55. **Done:** `test_noise_sweep_basic_preset` now asserts zero true
    limit violations over the whole run for the calibrated PI-like DAS
    result too (MPC calibrated/uncalibrated already had the check).
56. **Done:** a global limit on uncached password checks (`http.auth_verify_max` per `http.auth_verify_window_s`, `http.auth_verify_pending_max`) answers 429 beyond it, the checks run on one worker thread at a raised nice value (`http.auth_verify_nice`), and one check measured 1.23 s on the Zero W, where 100000 iterations stays the recommended count (§6). HTTPS auth cost on the Zero W: one PBKDF2 check at
    `http.hash_iterations: 100000` takes 1.15–1.4 s on the single core.
    Legitimate use pays it once per `http.auth_cache_s` per credential
    (owner: acceptable, no cookie needed). Remaining risk: every failed
    or uncached attempt runs PBKDF2, serialised by one lock; clients on
    many source addresses (easy with IPv6) could keep the core busy.
    Bound the global rate of uncached checks.
57. **Done:** `start_publishers` parses `http:` and `mqtt:` through
    `HttpSettings.from_section` and `validate_mqtt_section` before deciding
    whether to start, so `enabled: "true"` (a string) or any other mistyped
    scalar is a named error in the journal instead of a silent "off".
    `onewire.enabled` is type-checked at config load; `digole.enabled` has
    no owner module yet and only logs a warning.
58. **Done** (2026-09-16): a `drive_proximal` reading frozen while its
    zone's airflow gives no evidence (it stayed inside `stuck_airflow_net`,
    or it moved and the zone air excused it) is now flagged once a zone-air
    sensor of its zone has moved past the new `mpc.stuck_zone_air_dT_c`
    (default 1.5 °C) over one window **and** another `drive_proximal`
    reading of the same zone, of any bay, has moved with it by more than
    `stuck_sibling_dT_c` — at constant airflow the drive-to-air difference
    moves only with the bay's own power, so an air move that much larger
    than the band had to reach the sensor (§3). The corroboration is what
    separates a real air move from a drifting zone-air sensor: without it
    one lying air sensor branded every healthy proximal reading of its zone
    Stuck and faulted the zone (§4.4, `tests/test_stuck_sim.py`). Coverage
    on the `rich` sim, each of the 17 proximal readings frozen 20 minutes
    into a 2.5-hour PI-like run, seeds 0–2: **28 of 51** against 17 of 51
    before — the same 28 with and without the corroboration, since on a real
    air move the zone's other bays follow — with no healthy sensor flagged
    in those runs, in 256 healthy PI-like runs or in 120 DAS MPC runs. The
    default sits above the largest zone-air move seen at steady airflow on
    a healthy in-band reading (1.20 °C) in those 256 runs. The original
    gap (21 of 48) was measured on a different seed grid; the numbers here
    are the ones `tests/test_stuck_sim.py` reproduces. SMART divergence was
    rejected as evidence: the serial-to-bay map is identified online by the
    estimator, so the gate's verdict would depend on that identification.
59. **Done** (2026-09-16): `mpc.stuck_air_oppose_c` now has an upper
    bound, `mpc.stuck_air_oppose_max_c` (default 3.0 °C, validated
    `> stuck_air_oppose_c`): an airflow move shifts the drive-to-air
    difference by a few °C at most, so a zone-air swing larger than that
    cannot excuse a reading that did not move at all, and the airflow
    evidence stands. The default is above the largest opposing move that
    voided an airflow move on a healthy reading (1.19 °C) in the 256
    healthy runs above. An excused airflow move also falls through to the
    item 58 check, so the excuse in fact ends at whichever of
    `stuck_air_oppose_max_c` / `stuck_zone_air_dT_c` is met first: a zone
    whose fans happen to move must not hide a dead sensor that the same air
    swing would expose in a zone whose fans sit still.
60. **Done** (2026-09-16): the Stuck rule's quarter-window lag is now
    `mpc.stuck_pwm_lag_fraction` (§3 field table), a fraction of the
    window in `[0, 0.5]`, default `0.25`; `control/gate.stuck_pwm_lag`
    takes it as a required parameter instead of a literal `// 4`, both
    call sites (the legacy per-channel PWM lag and the DAS
    `_airflow_move` block lag) pass `cfg.stuck_pwm_lag_fraction`, and
    `floor(n * 0.25) == n // 4` for every window length the config
    allows, so the legacy and default-DAS paths stay bit for bit
    (`tests/test_gate.py`, `tests/test_stuck_sim.py`). The key may not
    switch the rule off: `_airflow_move` caps its block at
    `max(1, min(L, (m - L) / 2))` so the two blocks never coincide (a
    move of exactly `0.0` at `L = m // 2`) or run off the start
    (`None`), the range stops at half a window because a wider lag would
    leave less window to measure the move over than it skips, and a
    parametrised test flags a frozen, sibling-less reading at every
    fraction the config accepts.
61. **Done** (2026-09-16): a name whose slew check (gate rule 2) passed
    with neither a `last_good_obs` value nor a previous raw one to
    compare against, on any tick but the run's genuine first, is now
    `GateResult.no_reference`; `zones.advance_confirmation` starts such
    a name confirming on the same tick, exactly like a fresh rejection,
    instead of fusing it on trust alone (`tests/test_sensor_confirm.py`).
    "Past its own cold start" is a window, previous raw temps or a
    `last_good_obs` — not `last_good_obs` alone: while every zone is in
    fault nothing is fused, so it stays `None` for the whole outage, and
    a sensor returning then (a 1-Wire bus that was not up at boot, a
    DS18B20 reading its 85 °C power-on value) is exactly the one that
    needs vetting. Like the rejection branch it does not depend on the
    time status either, so one badly timed tick cannot be the tick a
    sensor slips in on. The genuine first tick — no window, no last raw
    temps, nothing trusted — is unaffected and stays bumpless.
62. **Done** (2026-09-16): `record_from_tick` now skips the names of
    `diagnostics["sensor_confirm"]` when building `trusted_temps`, the
    same exclusion `mpc.step` itself applies before fusing a sensor
    (`tests/test_recorder.py`).
63. **Done** (2026-09-16): the outward copy of `diagnostics["gate"]`
    (`control/mpc.py`, DAS only) now reads `false` in `per_temp` for a
    name in `diagnostics["sensor_confirm"]` even when the gate's own raw
    check passed this tick, so HTTP and MQTT no longer see a sensor as
    trusted while the estimator and the solver ignore it. The copy
    changes as a whole and not in one key: `reasons[name]` becomes
    `["confirming"]` (`zones.REASON_CONFIRMING`, the wording the zone
    reasons already use) and `gate.trusted` is recomputed over the
    overridden `per_temp`, so a consumer that renders "untrusted because
    …" or reads the summary cannot still be shown the old picture. Only a
    name the gate itself accepted is overridden; one it rejected already
    reads `false` with the gate's own reason. The top-level
    `diagnostics["trusted"]` keeps its own meaning (the gate's verdict on
    this tick, §4.7) and the internal verdict `zones.py` reads for zone
    trust and fault closure is untouched (`tests/test_sensor_confirm.py`).
64. **Done** (2026-09-16): the entry checks the same relative drift the
    return does, and the model's own rate now goes through the same
    low-pass as the drives' observed rate (`model_drift_rate_tau_s`), so a
    command move enters both sides alike instead of only the model's; a
    drift also has to keep failing for the new `model_drift_dwell_s`
    (120 s) before it faults the model (§3, validity gate and model
    fallback). On the load-step scenario of item 10 the sequence was
    `pi_das` at 1800 s, `mpc` at 2110 s, `pi_das` again at 2150 s, `mpc` at
    2570 s on every preset and seed; it is now two switches, the fallback
    and the return, on `basic` and `rich` seeds 1–3. The dwell leaks: a
    passing tick does not restart it and only a passing spell as long as
    the dwell does, because a residual just over its limit dips under it
    on sensor noise every few ticks. With a contiguous dwell bay gains
    1.5× — the band between a sound model and the sweep's 2× — escaped the
    gate on three of four `basic` zone/seed combinations while the drift
    was over its limit on up to 176 ticks; they are now caught 220–530 s
    after the load step wherever the drift reaches its limit at all
    (§3 for the band it cannot reach).
65. **Done** (2026-09-16): the same change. A healthy idle enclosure ran
    3600 s on both presets, seeds 1–8: three `rich` seeds used to enter the
    fallback on the drives' warm-up right after `bay_settle_s`
    (0.50–0.52 °C/min against the limit 0.5, at 600 s, 770 s and 1130 s,
    60–70 ticks of fallback each), and none does now. Loaded and
    hot-swapped runs (4800 s, seeds 1–4, both presets) enter none either.
66. **Done** (2026-09-16): the gate has air-node evidence now — the
    estimator's per-zone air disturbance against its own slow level
    (`model_max_air_dist_c_per_min` 8.0 °C/min, `model_air_dist_tau_s`
    900 s, under the same `model_drift_dwell_s` as the drift; §3, validity
    gate and model fallback). `E` acts on the air node alone, and that
    disturbance re-balances the node at any one operating point, so no
    residual at a settled operating point can see a steady airflow error —
    at a third of the airflow the drive rows' prediction error stayed at
    0.14 °C (limit 1.0) and their drift at 0.20 °C/min (limit 0.5). What the
    disturbance cannot hide is its own move when the airflow changes under
    the model. Measured on the truth simulator, 8 seeds × (idle, loaded,
    hot-swapped) per preset, the airflow scaled at 1800 s of a 4800 s run:
    with the limit at 8.0 the gate enters the fallback on **24/24 runs of
    both presets at 0.15× airflow and below**, 8/24 (`basic`) and 18/24
    (`rich`) at 0.25×, 0/24 and 11/24 at 0.3×, 0/24 and 3/24 at 0.5× — and
    on **0/48 healthy runs**. The limit cannot go lower without faulting a
    healthy model: at 6.0 it catches 0.25× everywhere but faults 6/24
    healthy `rich` runs, whose drawn physics leave the prior's air node as
    wrong as a 2× airflow error. **The headroom is thin, and deliberately
    so:** on the healthy runs the check reports at most 0.77 °C/min on
    `basic` and 7.44 °C/min on `rich` (seed 6; idle 3600 s, seeds 1–8 of
    both presets, and 0.49/7.11 on the loaded 4800 s runs), so the
    shipped 8.0 sits just above the `rich` preset's own distribution and
    another draw could reach it. Two things keep that from being item 65
    again: the limit is under the same leaky 120 s dwell, so a peak has to
    hold, and no healthy run of the 32 measured puts a single tick over
    it. A level is also a reference only after `model_air_dist_tau_s`
    (§3), which is what the highest healthy peaks were: with the level
    snapped to a fresh track's first value, `rich` seed 5 reported 7.95.
    Letting the check report but never fault while the status is `prior`
    or `learning` would remove the risk and every catch above with it:
    each of those runs is a prior-model run (`model_accept_prior`, §13
    stage 3), which is what the enclosure runs on until identification has
    happened, so the check faults on a prior model too.
    On the real enclosure the model is fitted, so the healthy floor is far
    below the `rich` preset's and the limit can come down — item 98.
67. **Done** (2026-09-16): the estimator carries a **placement offset** per
    proximal sensor of a bay beyond the first (`estimator.proximal_offset_c`,
    `q_offset`), so their disagreement is an offset it estimates rather than a
    swap the fast-swap rule keeps seeing (§3 estimator, "Placement offsets").
    Two proximal sensors on one bay at different
    placements: the estimator fused both into one sensor node, their
    disagreement tripped the fast-swap rule every tick and the bay's σ
    stayed at 4–7 °C, which faulted healthy zones on the `rich` sim.
    `trust_rule: sigma` was not usable with the example's redundant pairs
    until this was fixed.

    Measured on `sim/das.py`, example config, `rich` preset, 75 minutes,
    busy bays, redundant pairs present, before → after (zone-fault episodes
    and ticks of 900; σ of b03 / b10 as median, max):

    | seed | PI-like DAS before | after | DAS MPC before | after |
    |------|--------------------|-------|----------------|-------|
    | 0 | 2 / 912, σ 2.13/5.47, 5.31/6.87 | 0 / 0 | 2 / 912 | 0 / 0 |
    | 1 | 1 / 899, σ 1.50/1.53, 5.82/9.97 | 0 / 0 | 1 / 899 | 0 / 0 |
    | 2 | 15 / 733, σ 3.92/4.88, 3.72/4.89 | 0 / 0 | 13 / 657 | 0 / 0 |
    | 3 | 1 / 899, σ 1.50/1.53, 4.38/5.74 | 0 / 0 | 0 / 0 | 0 / 0 |
    | 4 | 1 / 3, σ 3.07/4.30, 1.50/1.91 | 0 / 0 | 0 / 0 | 0 / 0 |
    | 5 | 0 / 0, σ 1.50/1.53, 2.28/2.63 | 0 / 0 | 0 / 0 | 0 / 0 |

    After the change every seed has σ 1.50 median and 1.53 max on both bays
    (the uncalibrated floor), no drive over its limit, and the estimated
    offsets sit on the drawn placement difference (4.87 °C on b10, seed 1).
    The example's goldens moved with it: item 99.

    **Residual** (found completing item 17, 2026-09-16): item 17's agreement
    test removes the *standing* half of this — a settled redundant pair no
    longer re-arms the fast-swap rule — but not the *dropout* half: when one
    member of a pair drops out and returns, the returning reading is the
    bay's only trusted member that tick, the rule fires on it anyway, and σ
    jumps. Measured with `REDUNDANT = ()` in `tests/test_sigma_trust.py` (the
    `rich` masking removed): 9 of 123 tests still fail, all in the dropout
    sweep. The masking stays in place until item 101 gives every proximal
    sensor its own node.
68. **Done:** the hard sigma floor is a soft floor that holds the command before the loss until `zones.sigma_floor_hold_s` or a `zones.sigma_floor_growth_c` σ growth, then falls at `zones.sigma_floor_release_per_min` (§3 per-zone trust). Soft sigma floor (owner decision 2026-09-14). Today a zone with a
    lost sensor group keeps its fans at or above `prev`, which ratchets
    the DAS MPC's fans up until the zone faults when a bay's only sensor
    is lost for good (32.5 dB against 27.1 dB without the floor). Replace
    it with a soft floor whose thresholds are config keys, for example one
    that ends after σ has grown by a set amount.
69. **Done** (2026-09-16): the `sigma` rule suspends a bay's σ check while the
    estimator reports it `settling` **and** `observed`, and a bay with no trusted
    proximal member when its zone starts is seeded by its first reading
    (§3 estimator "Per-bay initialisation", §3 per-zone trust). Under `sigma` a
    hot swap faulted its zone for 1 tick plus
    `confirm_ticks` (the estimator's deliberate variance inflation), and
    a sensor missing on the estimator's first tick tripped the fast-swap
    rule when it returned.

    The exemption is bounded on the wall clock, by
    `estimator.bay_settle_max_s` (1800 s): one bay's windows may suspend the
    σ check for that long in total, until the bay has run that long with
    neither a window nor a σ over `sigma_fault_c`. Bounding it on the σ
    instead did not work — a bay's σ falls back within a tick or two of every
    jump, so a window could re-open at every cadence but one jump per tick,
    and a sensor that kept jumping kept its zone exempt for ever. Measured on
    `sim/das.py` (example config, 75 min, `basic` seed 1, checked against
    `rich` seed 0, both DAS solvers): b06's proximal sensor stepping +3.5 °C
    (inside the gate's slew limit, so every value is trusted) every 60 s
    faulted z1 75 times / 229 ticks with no exemption, **0 / 0** bounded on
    the σ alone, and 35 / 107 with the budget — the same at every cadence
    tried (120 s: 38 / 114, 0 / 0, 18 / 56; 240 s: 19 / 57, 0 / 0, 9 / 27;
    600 s: 8 / 24, 0 / 0, 4 / 12; 1200 s: 4 / 12, 0 / 0, 1 / 3). The exemption
    totals at most `bay_settle_max_s + bay_settle_s` = 2400 s over the whole
    run — 2395 s of it at the 60 s cadence, where the first fault lands at
    2400 s, and 1820 s at the 1200 s cadence, where it lands on the fourth
    jump — after which the zone faults as it did before this item. No run had
    a drive over its limit.

    The swap: `sim/das.py`, example config, b06's drive pulled at 300 s and a
    warm one inserted at 1200 s, seed 31 — 2 fault episodes / 6 fault ticks
    before, 0 / 0 after, with the bay's peak σ unchanged at 5.22 °C and the
    zone's fans rising by +0.31 (PI-like DAS, was +0.22 with the fault) and
    +0.64 (DAS MPC, was +0.65) after the insert. Every channel stays on
    `solver` where it used to `hold`. Safety: lose b06's sensor two ticks
    after that insert and the zone faults on the first blind tick, on
    `sigma:bay:b06`. The first tick: `rich` seed 5 with 2 % dropouts faulted
    z0 at 5–15 s before (1 episode, 3 ticks), none after; every other seed of
    the sweep was and stays 0.
70. **Done** (2026-09-16): `estimator.sigma_air_fault_c` cannot decide and the
    numbers say it should not, so the rule that decides is the time the air
    node has run unmeasured: `estimator.air_blind_fault_s` (900 s), reason
    `sigma:zone_air_blind` (§3 per-zone trust). With every
    sensor of a zone lost the air σ stayed below 0.35 °C, and only drive
    σ faulted a blind zone (after about 17 min, or 44 min for a bay's
    only sensor).

    Why the variance cannot: the air node is observed by more than the
    zone-air sensors. Every proximal sensor reads `(1 − s)·T_a` beside its
    drive, and the inlet sensor and the fan command pin the rest, so the blind
    air estimate stays good and its variance stays small — honestly so.
    Measured on `sim/das.py` (example config, z0's sensors dropped at 300 s,
    75 minutes, PI-like DAS): with **every** sensor of z0 lost, σ_air runs
    0.014 → 0.247 °C over 65 minutes while the true air error stays at
    0.001–0.070 °C (`basic` seed 1) or grows 0.446 → 0.576 °C (`rich` seed 0,
    where the standing 0.45 °C is the drawn sensor offset the estimate had
    already inherited — the *growth* while blind is 0.13 °C). With **only**
    the zone-air sensor lost, σ_air reaches 0.068 °C, the true error 0.053 °C,
    and every bay σ stays at the uncalibrated floor 1.501 °C for the whole
    run — so under `sigma` nothing ever faulted that zone, while `strict`
    faults it at once. Even a disturbance the model does not know (z0's fans
    fouling to 0.4× over the run) leaves σ_air at 0.044 °C and the air error
    at 0.47 °C, because the proximal sensors carry the zone.

    With the clock: the zone faults 15.0 min after either loss
    (`basic` seed 1, both solvers), holds and ramps to `fallback_pwm`, no
    drive over its limit. It is never quieter than before: the fully blind
    zone used to fault at 16.8 min on the drive σ, and the air-only loss never.
    `air_blind_fault_s: 0` faults on the first blind tick (as `strict` does);
    a large value leaves the drive σ to decide, which is the behaviour before
    this item. `air_blind_s` is wall-clock: a gap in the ticks counts in full
    (capped at the filter's 3600 s prediction horizon), not at the occupancy
    horizon's 3·`dt`, which would have under-counted a gap by up to 240× and
    delayed the fault — and the air σ cannot make that up, which is this
    item's own premise.
71. **Done** (2026-09-16): the hardcoded estimator tunables are config keys
    of the `estimator` section with one documented default each in
    `ESTIMATOR_DEFAULTS`, validated in the config model and spelled out in
    `config.example-das.yaml`: `reset_drive_var` (25 °C²), `jump_min_c`
    (0.5 °C), `jump_sigmas` (6), `p0_t_air` (0.25), `p0_d_air` (2.5e-3),
    `p0_t_drive` (0.1), `p0_t_sensor` (0.1), `p0_heat` (2.5e-5) and
    `sigma_uncalibrated_c` (1.5 °C). `SIGMA_UNCALIBRATED_C` is gone from
    `model.py`; `control/estimates.sigma_uncalibrated_c(cfg)` reads the key
    (a legacy config without an `estimator` section falls back to the same
    default), and the `zones.trust_rule: sigma` rule compares
    `sigma_fault_c` with the config's value rather than a literal. The
    module priors of the physical model (`C_a`, `leak`, `κ`, `E`, `g0`, `k`)
    stay constants: they are the thermal model's, not the operator's.
    At the defaults the behaviour is unchanged. One deliberate exception, a
    single bit wide: `p0_d_air` was spelled `0.05 ** 2` in code and is
    `0.0025` as a config number, which is the next float. The four DAS
    goldens then reproduce with `max |Δpwm| = 1.4e-12` (PI-like DAS
    1.2e-15), identical temperatures, modes and faulted zones — inside
    the goldens' `1e-6` tolerance, so the files are untouched. Every other
    key reproduces its trajectory bit-for-bit (checked by running the four
    scenarios with the pre-item-71 spellings: `MISMATCHES 0`).
72. **Done** (2026-09-16): `sensor_lost:<zone>` is a start precondition
    and an abort reason — a served zone with a zone-air or bay group that has
    no trusted, confirmed member this tick, read from
    `diagnostics["sigma_floor"]` (§5). Under `strict` such a zone is in fault
    and `settle` / `degraded` catch it first, as before. Under `sigma`, an
    identification experiment could start in a zone that has a lost sensor,
    because the zone is still trusted.
73. DAS step budget on the Zero W after items 3, 8, 9 and 10. Measured
    on the Pi: DAS MPC p99 609–615 ms (solve ticks 643 ms) against
    `mpc.budget_ms` 600, so `pytest -m pi` fails again (it was
    507–552 ms); PI-like DAS p99 322 ms. Owner decision 2026-09-14:
    optimise; if the DAS MPC still misses 600 ms, raise `mpc.budget_ms`
    to 1000; solve every third tick (`mpc_every_ticks: 3`). Note that the
    p99 over all ticks is set by the solve ticks while they are more than
    1 % of ticks, so solving less often lowers the mean, not the p99.

    **Optimised (2026-09-16), still open.** Four changes, all bit-identical
    (the closed loop dumps every tick's mode, command and diagnostics as
    `float.hex` and matches the same run on the previous revision across
    nine scenarios: DAS MPC and PI-like DAS, `mpc_every_ticks` 1/2/3, two
    seeds, `median3` on and off, and the legacy MPC and PI on the RC plant;
    the golden files are untouched):

    - the gate sanitises only the window samples a check reads (the
      pre-filter needs the newest three; the Stuck band check stops at the
      first sample outside the band), and takes the exact-`float` fast path;
    - the estimator memoises its config-derived structure by config
      identity (as `thermal._derived` does) and drops the `numpy.outer`
      wrappers and a temporary from the Joseph update;
    - `model._is_real`, called for every value of every observation and
      command, answers exact `float` / `int` before the `numbers.Real` ABC;
    - `thermal.state_jacobian` gives the DAS MPC's linearisation `A` and
      the airflow gradients from one airflow evaluation instead of three,
      and stops building an input matrix, a derivative vector and an affine
      term it then discarded.

    Development machine, `config.example-das.yaml` against the DAS truth
    plant, 240 ticks with 20 discarded as warm-up, gc paused, 5 repeats per
    measurement and three rounds alternating before/after so machine state
    cannot favour one side; medians of the per-repeat statistics: median
    step 1.886 → 1.575 ms (−16 %), p99 3.381 → 2.876 ms (−15 %), p99 over
    the solve ticks 3.398 → 2.892 ms (−15 %). At the Zero W's measured ×180
    for this step that is roughly 520 ms p99 — under the 600 ms gate, but
    by 13 %, on an extrapolation that §4 says is a lower bound. It has to
    be measured, not assumed. Where the remaining time goes is in §4
    "Budget at `dt = 5 s`": the SQP, its box QPs and the estimator's Kalman
    update are most of it, and cutting those means changing the arithmetic,
    which would move the goldens — item 95 carries that decision, so closing
    this item does not lose it.

    The owner's fallback is documented config, not a new default:
    `mpc.budget_ms: 1000.0` **with** `mpc.budget_alarm_ms: 1250.0` (the
    config model rejects a `budget_ms` that is not strictly below the
    alarm, so raising one alone fails to load) and `mpc_every_ticks: 3`.
    `config.example-das.yaml` names each of the three values in the comment
    on the live key it replaces: the edit is *changing a value*, not
    uncommenting a line. A second copy of a key is not an override — YAML
    keeps only the last one — so since this item `load_config` refuses a key
    written twice instead of silently dropping one
    (`config._UniqueKeyLoader`); `config.example.yaml` is the legacy layout
    with no DAS MPC and points at the DAS file. Both budget log lines now
    name the key that was exceeded and the keys to change
    (`mpc.mpc_every_ticks` only where the DAS MPC is the solver).
    `tests/test_bench_budget.py` reads the three numbers out of
    `config.example-das.yaml`'s own comments, writes each over its live
    value and loads the file — so a fallback documented in a form that does
    not take effect fails there and not on the Pi — and checks that
    `budget_ms: 1000` alone is rejected, that `mpc_every_ticks: 3` really
    solves on a third of the ticks against the truth plant and that a solve
    tick is the one that runs the SQP; `tests/test_loop.py` checks the
    raised thresholds and the two log lines.

    **On the Pi (main session).** Re-measure the DAS MPC p99 with
    `tools/bench_step.py --sim-plant das` and run `pytest
    tests/test_bench_budget.py -m pi`; if the p99 is still over 600 ms,
    put the three fallback keys into the Pi's config and re-run. Then run
    the runtime alarm against the **DAS MPC** for at least 20 minutes
    (`budget_warn_count` / `budget_alarm_count` in the health payload; the
    earlier 20-minute run used the PI-like DAS form) and record the
    numbers here.

    **Re-measured on a Raspberry Pi Zero 2 W (2026-09-17), the owner's
    faster board.** 4 cores at 1.0 GHz, 64-bit trixie, kernel 6.18, no
    throttling during the run (`get_throttled` 0x0 throughout; board
    temperature 41.9 → 49.4 °C). `tools/bench_step.py --sim-plant das`,
    `config.example-das.yaml`, `dt = 5 s`, 600 ticks: DAS MPC `step()`
    mean 51.08 ms, p50 49.42 ms, **p99 85.96 ms**, max 92.61 ms;
    `solve_p99_ms` 87.22 ms over the 300 solve ticks; `model_active_fraction`
    1.0 (the MPC drove the fans on every tick, so this times the MPC path,
    not the PI-like fallback); modes `auto` 562 / `saturated` 38 /
    `degraded` 0 / `fallback` 0. PI-like DAS `step()` mean 25.36 ms, p99
    26.93 ms, max 27.56 ms. Legacy config (`dt = 2 s`): MPC p99 7.14 ms, PI
    p99 1.79 ms. For comparison, the single-core Zero W these budgets were
    set for had an MPC step p99 of 609–615 ms.

    Against the shipped `budget_ms` 600 / `budget_alarm_ms` 750 the
    measured DAS MPC p99 is **7.0× and 8.7× under** -- so far below that a
    regression tripling the solver's own cost would still pass unnoticed.
    **Re-derived: `budget_ms` 250, `budget_alarm_ms` 350** -- 2.9× the
    measured p99 and 2.7× the measured max, an alarm 1.4× above the
    warning (the old pair's ratio was 1.25×). What else shares the 5 s
    tick and could still make `step()` late without the solver itself
    regressing: the sensor gate and the estimator update inside `step`
    (already in the p99 above), the hidraw I/O and 1-Wire bulk read the
    loop does *outside* `step` (§2 "hidraw check", not gated by
    `budget_ms` at all), and GC pauses on a loaded board. That last one
    sets the floor: not chased below roughly 200 ms, because 200 ms is
    only 2.3× the measured p99 and the first garbage-collection pause on a
    board under load would false-alarm at that margin. Changed in the
    config model's default (`MpcConfig.budget_ms` / `budget_alarm_ms`),
    both example configs, and everywhere else in this document that quoted
    600 / 750 as the shipped value (§3 "Budget at `dt = 5 s`" and its key
    table, §6, `health.py`'s `vcgencmd` percentage). `mpc_every_ticks`
    stays 2 in both example configs -- nowhere near needed. The Zero W
    fallback (`budget_ms: 1000.0` / `budget_alarm_ms: 1250.0` /
    `mpc_every_ticks: 3`, documented on the live keys of
    `config.example-das.yaml`) is untouched: it is for a board this one is
    not, and stays exactly as it was for whichever board needs it next.

    **Still open:** the runtime alarm has not been run live against the
    DAS MPC for 20 minutes on the Zero 2 W (`budget_warn_count` /
    `budget_alarm_count` in the health payload) -- only the bench tool was
    run. With 2.9× headroom over the measured p99 that run is expected to
    show zero warnings, but it has not been performed and the counts are
    not recorded here.
79. **Done** (2026-09-17, audited item by item against what this item
    asks). Every status report carries each output's rpm, output duty,
    voltage (the 12 V rail) and current and power (the Quadro reports both;
    the aquaero reports 0 in PWM mode).
    - *Record them.* `AquacomputerAdapter.fan_readings()` hands them out
      per commanded channel, `read()` puts them in
      `PlantObservation.inputs["fans"]` — exogenous, non-gated data that
      never reaches the sensor gate or `mpc.step`, so no golden moved —
      and `recorder.py` writes them as the record's `fans` key.
    - *Publish them.* `HealthMonitor` merges each tick's verdicts into the
      payload `Supervisor.set_device_health` takes, whence `/api/state`'s
      `device_health.fans`, the MQTT state blob, the Home Assistant
      `device_problem` sensor's attributes and the page.
    - *Warn on drift.* All three rules are implemented and sustained
      (`health.py`, §3 "Fan and device health"): rpm against the
      `mpc.fan_models` curve `tools/fit_fans.py` fits, the 12 V rail
      window, and power against the duty through the fan law. Each judges
      against the `settle_s` duty band and holds for its own `*_fault_s`;
      every threshold is a `fan_health:` key with one default in
      `FanHealthConfig`, validated there and shown in both example configs.
    What this item does **not** cover, and where the remainder lives, so
    "Done" is not read for more than it is: **current is never judged on
    its own** — on the aquaero neither its own outputs (0 mA in PWM mode)
    nor its aquabus blocks (a sample taken inside the PWM cycle, which at a
    low duty reads 0 mA with the fan turning — the cause corrected
    2026-09-18, §2) give a number a rule may act on, so `power_reported` is
    false for all eight, and for a Quadro's own four as well, and both
    fields publish as `None`; **the rail rule cannot fire for an
    aquaero's aquabus outputs**, which is item 117, accepted and now
    declared per channel in the verdict; and **the thresholds are still the
    wide guesses**, with `fan_models.<m>.power_w_at_max` unset in both
    example configs (so the power rule is off until it is measured) and the
    DAS example's placeholder `rpm_max` not yet the real curve — that is
    item 94, which needs the Pi and the fans and is the owner's. The last
    of those is declared, not just written here: a channel whose model has
    no `power_w_at_max` publishes `power_monitored` false with that as the
    reason, so a controller that *does* measure power (a Quadro on its own
    USB) cannot publish an unjudged 0 W as a rule that passed (item 117).
84. **Done** (2026-09-16): fan control without wearing the controllers'
    memory (item 77), with the controller's own watchdog behind it. The
    adapter writes a **software-sensor heartbeat** and watches the **active
    profile**; the device-side half (the sensor's timeout and fallback, the
    alarm action, the saved profiles) is the owner's configuration on the
    aquaero, and the commissioning tool for it is item 88.
    - `heartbeat_sensor` (0 = off, the default; aquaero `softN` 1..8) and
      `heartbeat_value_c` (20.0 °C) are per-device config keys. Every
      `apply()` writes that one sensor with HID output report `0x07` **after**
      the duty work and only when it succeeded — once per `apply()`, however
      often the control write is retried — and leaves the other seven slots at
      `0x7FFF` ("no data"), so the device keeps its own values for them. The
      order is the safety property: a heartbeat ahead of the duties would hold
      the watchdog shut through the one failure it uniquely covers, a daemon
      alive on a live node whose every control operation fails; a tick that
      could not command a duty now sends none and the controller falls back on
      its own. An `apply()` with nothing to send still counts as commanding,
      so the cadence stays one per tick. The heartbeat waits
      `ctrl_gap_ms` and runs inside `ctrl_budget_s` like any control
      operation, so the worst case per tick and the watchdog bound are
      unchanged, and running last it is what a spent budget drops; a failed
      write never fails the tick and is logged once per state change. Nothing
      is written to a device whose serial does not match, so the controller
      takes over then too. The Quadro's software-sensor report is unknown, so
      `heartbeat_sensor` on a `quadro` entry is a startup `ConfigError`.
      `heartbeat_sensor: 0` is right only while **no** software sensor is
      enabled on the controller: an enabled sensor nothing writes falls back
      one timeout after boot and its alarm selects that profile for good, so
      the watchdog has fired and cannot fire again while live writes keep
      working. Both example files show the key at its default with that said
      next to it; `1` is the value for the owner's aquaero, after item 93.
    - Byte `0x06` of the aquaero's control report is the active profile
      (`active_profile`, 1-based). Every fresh control report is compared
      against the last one: a change logs one line naming both profiles and
      makes the next write send every configured channel, because the switch
      reloads the **saved** profile and every live duty is gone with it. No
      control read is added per tick: the report is the one `ctrl_refresh_s`
      or an invalidation already fetches, so a switch is noticed within
      `duty_mismatch_s` plus a tick when the reloaded profile drives an
      output differently (the duty verification sees it) and at the latest
      after `ctrl_refresh_s` = 60 s otherwise. Both edges, and byte `0x06`
      bounds neither: on the alarm-**set** edge the reloaded profile is the
      safe one (every output at 100 %), so the delay only postpones the daemon
      taking the fans back down; on the alarm-**clear** edge — a daemon
      restarted after an alarm — the reloaded profile is the quiet one (20 %
      on the owner's controller), which reduces cooling, and what bounds that
      is the duty verification (the reloaded duty differs from the written
      one), about 5–10 s. The 60 s worst case is reached only when the
      reloaded profile drives the outputs exactly as the daemon last wrote
      them (§3 Track B).
    - `heartbeat_on`, `heartbeat_ok` and `active_profile` are on the adapter
      for diagnostics; publishing them is item 83.
      `tools/aquacomputer_probe.py` prints the active profile.
    Fake-transport tests for the report bytes and cadence, the configured
    sensor only, a failed write, a tick that could not command the duties
    sending no heartbeat, a profile change forcing the rewrite, no
    extra control read per tick and the config keys (§4.7). Not yet run
    against the hardware: item 93. Verified on the Pi
    (2026-09-15, §2 "Apply without saving", "Software-sensor heartbeat and
    profiles"):
    - A control report SET **without** the short report 6 that follows it
      (`06 00 02 00 00 00 00`, the "secondary report" the adapter sent after
      every SET until item 86) takes
      effect at once and does **not** survive a power cycle: the controller
      comes back with the configuration saved before. Report 6 saves. The
      same pattern is documented for the Farbwerk 360, whose save report is
      byte-identical to the report the Quadro was sent after every SET; for
      the Quadro this is not verified. Whether a SET without the save still writes the
      memory cannot be observed; nothing persisted.
    - The aquaero's eight software temperature sensors are set by HID
      output report `0x07` (17 bytes: the id, then eight u16 big-endian
      values in 1/100 °C, `0x7FFF` for no data; as in aerotools-ng). Their settings
      sit in the control report from `0x177`, 5 bytes each: enabled (1
      byte), fallback temperature (u16, 1/100 °C), timeout (u16, s). They
      show in the status report at `0x85 + 2i`, the software sensors
      `soft1..8` of `hw/aquacomputer.py` (the Linux driver's virtual sensors
      1–8, renamed by item 86). A value written once to the enabled sensor
      1 appeared in the next status report, held for exactly its 300 s
      timeout and then fell back to 40.00 °C. The write did not change the
      control report. A disabled sensor shows no value.
    - The aquaero has only four curve controllers (manual §12.1), so mapping
      software sensors onto duties covers at most four independent outputs,
      not the eight of an aquaero plus a Quadro.
    Answered since: a profile switch **does** override live presets, on the
    Quadro's aquabus outputs too, and the switch back leaves the saved
    profile running, so the daemon rewrites its duties after one; a live
    write through the aquaero to an aquabus fan behaves like any other
    (§2 "Live write through the aquaero to an aquabus fan"). The adapter
    sends no save report on a write (item 86) and
    `AquacomputerAdapter.save()` stays the commissioning step that stores the
    safe configuration once (item 88). The four curve controllers are not
    needed for this: the heartbeat drives an alarm, not a curve.
    **Blocks item 42** only until the daemon's heartbeat runs on the hardware
    (item 93) and the safe configuration is commissioned (item 88).
86. **Done** (2026-09-15): a write is one control report SET without the
    save report, so it takes effect at once and is not stored in the
    controllers' memory (items 77, 84); `AquacomputerAdapter.save()` sends the
    save report once, for commissioning the saved safe configuration, and
    the daemon never calls it (on the Quadro the save is unverified). The
    SET alone is the write for the gap, budget and retries: the gap is timed
    before every GET, SET and save report, and a failed SET or a budget
    spent before it makes the next write send every configured channel.
    `release()` restores with a live SET. `write_min_interval_s` and
    `write_deadband` default to 0, so every change is written; the keys
    remain to limit USB traffic, and the FALLBACK/DEGRADED floor (no channel
    below what the device holds) is unchanged (§3 Track B). The status slots
    at `0x85` are the software sensors `soft1..8` and those at `0x95` the
    virtual sensors `virt1..4` in `hw/aquacomputer.py`, the probe, the
    config and the docs. Tests with captured reports and the fake controller
    (§4.7). Follow-ups: items 87, 88. The adapter sent the save report (the
    "secondary report") after every SET, so every write that changed a duty
    was saved to the controller's memory.
    Hardware watchdog verified (2026-09-15, owner's profiles): software
    sensor 1 enabled with a 30 s timeout and a 90 °C fallback; a temperature
    alarm on it selects profile 2, alarm level 0 selects profile 1. In both
    profiles every output 1–8 (the Quadro on aquabus included) follows preset
    1 with minimum 35 % and maximum 100 %; profile 1 holds preset 1 at 25 %
    (every output at 51.25 %: the aquaero scales a controller value into
    [min, max]), profile 2 at 100 %. With a heartbeat of 20.00 °C written
    every 2 s nothing changed; 30 s after the last write the sensor fell
    back to 90 °C and within 2 s every output went to 100 %. A resumed
    heartbeat brought profile 1 back within 2 s, with the control report
    byte-identical to before. Byte `0x06` of the control report reads the
    active profile (0 = profile 1, 1 = profile 2; 0 in every earlier
    capture). A live, unsaved preset (30 %, output 54.5 %) was overridden by
    the alarm and did not come back with profile 1: the switch reloads the
    saved profile, so the daemon must rewrite its duties after a profile
    change (watch `0x06`). With no daemon writing the heartbeat, the aquaero
    now goes to profile 2 on its own 30 s after any silence, including after
    a power cycle.
85. **Done** (2026-09-15): the aquaero entry commands outputs `pwm1..pwm8` and
    reads tachometers `fan1..fan8`, 5–8 being a Quadro on its aquabus (fan
    blocks `0x167 + 12k`, control blocks `0x20C + 20k`, presets `0x55C + 2k`
    with id `0x5C + k`, the same write as outputs 1–4), the aquabus
    temperature slots `bus1..bus8` (`0x75`) and `flow1..flow3`. Config names
    are by group (aquaero `tempN`, `busN`, `softN`, `virtN`; Quadro
    `temp1..4`, `soft1..16`; `fanN`, `flowN`, `pwmN`), and a name of the hwmon
    driver's numbering that means another input now (aquaero `temp9..20`,
    Quadro `temp5..20` and its flow sensor `fan5`) is rejected with the new
    name (§3 Track B); flow sensors cannot be bound (item 91). A configured
    aquabus output or tachometer whose fan block reads rpm `0xFFFF` (nothing
    on aquabus) fails `read()` naming the channel and is listed in
    `absent_channels`; `apply()` writes it and does not raise, so the
    fallback ramp is not held back (item 90). A config that commands
    `pwm5..pwm8` and the outputs of a commanding quadro entry is refused at
    startup either way, with or without `serial:` (item 106: one with a
    serial is a second, physically distinct Quadro on its own USB port, i.e.
    a second, independent controller); a stuck Quadro channel next to an
    aquaero that reports an aquabus device is logged as probably on aquabus,
    to be commanded through the aquaero. Aquabus blocks (mode word `0x0500`)
    get no "not PWM" warning; the unconfigured block 8 (source `0xFFFF`,
    mode 0) is written the same way with one warning that this is unverified
    (item 89). `config.example-das.yaml` commands the Quadro through the
    aquaero. Tests with captures with and without the Quadro on aquabus
    (§4.7). Follow-ups: items 89, 90, 91.
88. **Done** (2026-09-16): a commissioning command for the saved
    configuration (item 84), `tools/aquacomputer_commission.py`: it opens
    the one configured controller named by `--device` (`--serial` narrows
    several entries of one kind), prints every output's duty, source and
    limits and the active profile -- exactly what `save()` is about to
    store, since `save()` persists whatever the control report holds *now*
    and this tool changes nothing about it first -- and, only with `--save`
    and a typed confirmation (the flag alone never saves), calls
    `AquacomputerAdapter.save()` once. What is saved is what was shown: the
    device stays open for the whole run and the control report is read once
    more right before the save, since the controller can change while the
    prompt waits (the heartbeat times out, the alarm selects another
    profile, and a profile switch reloads *that* profile's saved settings;
    aquasuite; the front panel). A report that no longer matches the one
    printed aborts the run with exit 6 and saves nothing. Refuses unless
    `systemctl is-active <unit>` (default `aqua-bridge.service`) answers
    `inactive` or `failed`; every other answer counts as running --
    `active`, `activating`, `reloading`, `deactivating` (a `systemctl stop`
    returns while the daemon's own `fallback_pwm` write and its `release()`
    SET are still in flight) and anything undeterminable (a missing
    `systemctl`, a timeout, a blank or unrecognised answer). Names the
    Quadro's save report as unverified (item 86) before asking to send it.
    `AquacomputerAdapter.control_snapshot()` (new: a control-report GET
    that adopts the result like `apply()` does, writing nothing) is what
    reads the report, both times. The per-output line is formatted by
    `hw.aquacomputer.format_channel_state()` (new, with
    `format_percent()`), shared with `tools/aquacomputer_probe.py` so the
    two tools cannot drift apart. Fake-transport tests only
    (`tests/test_aquacomputer_commission.py`,
    `tests/test_hw_aquacomputer_adapter.py`); confirming the Quadro's save
    persists over a power cycle on the hardware is item 96.
90. **Done** (2026-09-16): an aquabus output or tachometer with nothing
    behind it (rpm `0xFFFF`) faults **only its own channel**. `read()`
    returns the observation with that channel's `rpm` and `pwm` as `None`;
    every other channel, every temperature and the whole write path are
    unaffected, and `absent_channels` lists the channels, with one error
    logged when that list changes (an info line when it empties) instead of
    a read failure every tick. Before, every `read()` of that controller
    raised, so the whole composite ran in the fallback and the aquaero's own
    thermistors went blind for as long as the slot stayed empty — which is
    what the Quadro dropping off aquabus does (item 92). What still makes a
    controller unavailable is **only a missing status report** (a vanished
    node, the wrong serial, or nothing within `status_max_age_s`): that is
    when nothing it reports can be trusted. An adapter whose *every*
    commanded output is absent says so in that one error line and is still
    not unavailable, because its temperatures are exactly what makes the
    fans that are left ramp. A missing fan stays safe through the plant, not
    through a fault: the zones it served lose cooling, their temperatures
    rise and the remaining fans ramp (§3 Track B) — the same path as a fan
    that has simply died, and one the `sigma` gate and the MPC already
    handle. `obs.pwm` `None` also keeps the solver from taking a dead slot
    as `prev` (`obs_pwm_usable`), and `obs.rpm` `None` is neither evidence
    for nor against a stall. No confirmation over time was added: a single
    transient `0xFFFF` while the Quadro re-enumerates now costs that channel
    one tick of `None` instead of a fallback tick for the composite, so a
    config key for it would only add a tunable nobody needs. `apply()` still
    writes an absent output with the others and does not raise for it (an
    `apply()` that raised after its SET would freeze the loop's rate-limited
    fallback ramp). Publishing `absent_channels` stays item 83; the frozen
    `busN` temperatures of an absent device stay item 92 — and they got more
    urgent here, because the read that used to raise was also what kept them
    out of the solver, so §3 Track B and the README now say not to bind a
    `busN` of a device that can leave aquabus until item 92 lands.
83. **Done** (2026-09-17; the list was complete but for one pair, closed
    here). `AquacomputerAdapter.device_health()` and
    `CompositeSource.device_health()` publish `stuck_channels` (item 81),
    `not_pwm_channels`, `absent_channels` (items 85, 90),
    `unconfigured_channels` (item 89), the flow sensors (item 91), serial,
    firmware, power cycles, status age, `active_profile` (item 84) and a
    `problems` list, and `HealthMonitor` merges them into what
    `Supervisor.set_device_health` takes — so they reach `/api/state`
    (`device_health.devices[]`), `/api/health` (`ok` and `problems`), the
    MQTT state blob and the Home Assistant `device_problem` sensor, whose
    attributes carry the whole blob. An absent channel is no longer a read
    failure, so the health endpoint is the way to see it.
    The audit found **`heartbeat_on` and `heartbeat_ok` still unpublished**
    — on the adapter for diagnostics and in no payload anywhere. They are
    now `device_health.heartbeat` (`on`, `ok`, `sensor`, `value_c`), and a
    heartbeat whose last write failed is a `problems` line, since the
    controller's own timeout is then running with nothing resetting it. The
    other state the daemon could not see is a `heartbeat_sensor` the
    controller has **disabled** — the write succeeds every tick and the
    sensor the alarm watches stays `0x7FFF` — which item 113's
    software-sensor settings make visible and which is a `problems` line
    too.
91. **Done** (2026-09-16): owner decision, §8.1 — flow stays out of
    `PlantObservation`. It is still decoded, `tools/aquacomputer_probe.py`
    shows it, and it is published with the device health as
    `device_health.devices[].flows`. A `flowN` name in a device entry's
    `fans.<ch>.pwm`, `fans.<ch>.rpm` or `temp_map` is rejected with a message
    saying flow sensors cannot be bound and that an aquaero's hwmon-era
    `fan5`/`fan6` are aquabus tachometers now.
94. Tune the fan-health thresholds on the real enclosure (item 79). The
    `fan_health:` defaults are deliberately wide guesses; nothing but the
    rail window has been checked against hardware (every captured report sits
    inside 11–13 V). Needs the Pi and the fans, so the **main session**:
    - record a run that sweeps each channel over its duty range, fit it with
      `tools/fit_fans.py` and put the result in `mpc.fan_models` — the rpm
      rule is only as good as that curve, and with the DAS example's
      placeholder `rpm_max: 1500` it would fire on healthy fans (the online
      fit of item 14 feeds the thermal model, not `fan_health`, so the
      configured curve still has to be right here);
    - measure each fan model's power at full speed and set
      `fan_models.<m>.power_w_at_max` (the power rule is off without it).
      The one measurement so far is the Quadro's aquabus fan 7: 27 mA and
      0.32 W at 100 % duty, 1105 rpm — small enough that `power_min_w` 0.2
      leaves little headroom, so check whether the controllers' 0.01 W
      quantisation makes the rule usable at all on these fans or whether it
      should key on current instead;
    - watch how long the aquabus rpm actually lags after a duty step and set
      `settle_s` from that (15 s is a guess from "several seconds"). It is now
      the width of the duty band the rpm and power rules judge against, so too
      large a value only makes them blunt, never false;
    - confirm no rule fires over a quiet day before narrowing
      `rpm_tolerance_frac` or the `*_fault_s` windows.
95. Numerics of the DAS MPC solve tick (from item 73; the breakdown is in
    §4 "Budget at `dt = 5 s`"). On the development machine the SQP with its
    box QPs (0.73 ms) and the estimator's Kalman update (0.64 ms) are about
    half of a 2.9 ms solve tick, and they are what is left after item 73's
    bit-identical optimisations. Every way found to make them cheaper —
    reordering the accumulations, other dtypes, vectorising the Joseph
    update across bays — changes the floating-point arithmetic and moves
    `tests/golden/*`, which item 73's brief forbids without asking, so it
    stopped there. **Owner decision:** is the remaining time worth
    regenerating the goldens? If yes, the work is the numerics change, the
    regenerated goldens, and a re-run of item 73's bit-identity harness
    against the *new* baseline (it can then only prove that the two
    revisions agree, not that the controller is unchanged, so the golden
    diff has to be read). If no, close this item and leave both phases as
    they are.

    **On the Zero 2 W (2026-09-17), what item 73's re-measurement adds.**
    `tools/bench_step.py --sim-plant das` against `config.example-das.yaml`,
    600 ticks, reports `solve_p99_ms` **87.22 ms** over the 300 solve ticks
    (against the DAS MPC's own overall step p99 of 85.96 ms -- the solve
    ticks are the ones near that p99, as expected with `mpc_every_ticks: 2`)
    and the tick-mode counts: `auto` 562, `saturated` 38, `degraded` 0,
    `fallback` 0, with `model_active_fraction` 1.0 -- the run never left the
    MPC path, so every number above times the MPC, not the PI-like
    fallback. That is the whole breakdown the bench tool gives on the Pi:
    one p99 over the solve ticks and the mode counts, not the per-phase
    split (SQP and its box QPs, the estimator's Kalman update, the
    prediction, the operating-point and model checks, ...) that only the
    development-machine profiling above has. Getting that same per-phase
    split on the Pi is not a numerics change and would not move the
    goldens, but nothing in `bench_step.py` or `control/solver_das.py`
    currently times the sub-phases -- it is unmeasured, not merely
    un-transcribed here. The owner decision above (regenerate the goldens
    for cheaper arithmetic) is unaffected by this and still open.

    **`bench_step.py --profile-phases` added (2026-09-17); board run
    below (2026-09-18).** The bench tool now takes `--profile-phases`
    (`--sim-plant das`, `mpc` solver only) and adds a `phases` breakdown to
    its `mpc` result: `sqp_box_qp_ms` (`solver_das.solve_penalty_qp`, the
    SQP with its box QPs), `estimator_kalman_ms` (`estimator.update`),
    `gate_ms` (`mpc.evaluate_gate`) and `bookkeeping_ms` (the residual --
    everything else `_tick` does: prediction, model checks, disturbance
    filtering, band snapping, the bumpless offset, the `json.dumps` guards,
    ...), each with mean/p99/max/n over the solve ticks, matching
    `solve_p99_ms`'s own denominator. It measures by temporarily
    monkeypatching those three call sites' own module bindings for the run
    and restoring them after -- `control/mpc.py` and `control/solver_das.py`
    are untouched, so this cannot move a golden or change what a tick
    returns, only how the wall time inside it is reported (`tools/bench_step.py`
    module docstring, `tests/test_bench_budget.py`). A dev-machine sanity
    run (`--ticks 60`, 30 solve ticks) shows the wrapper actually reaching
    every named phase and the phases summing to the tick's own time, e.g.
    (dev machine, not the Pi, not comparable to the numbers above which use
    different call boundaries): `sqp_box_qp_ms` mean 0.48 ms,
    `estimator_kalman_ms` mean 0.71 ms, `gate_ms` mean 0.04 ms,
    `bookkeeping_ms` mean 0.96 ms, against a 2.6 ms solve-tick max.

    **Board run (2026-09-18).** The earlier no-route failure was a
    network-reachability gap in a different work environment, not this
    one or the board: `ssh` reaches the board this time, and `/opt/aqua-
    bridge` there is a git-free copy of the same revision this ran
    from (`sha256sum` of `tools/bench_step.py`, `tools/bench_model_
    store.py` and this file all match). `PYTHONPATH=/opt/aqua-bridge/src
    .venv/bin/python tools/bench_step.py --sim-plant das --ticks 600
    --solver mpc --profile-phases` ran clean (`/sys/class/thermal/
    thermal_zone0/temp` 42.93 → 48.31 °C, `vcgencmd get_throttled` `0x0`
    before and after -- not a throttled run) and reports, over the 300
    solve ticks: `sqp_box_qp_ms` mean 25.80 ms / p99 35.92 ms / max
    38.69 ms, `estimator_kalman_ms` mean 22.37 ms / p99 24.49 ms / max
    25.23 ms, `gate_ms` mean 1.84 ms / p99 3.02 ms / max 3.11 ms,
    `bookkeeping_ms` mean 30.36 ms / p99 34.36 ms / max 43.22 ms, against
    `solve_p99_ms` **94.70 ms** (`p99_ms` 91.64 ms over all 600 ticks,
    `mean_ms` 55.91 ms, `modes` auto 559 / saturated 41 / degraded 0 /
    fallback 0, `model_active_fraction` 1.0 -- a fresh run of the same kind
    (clean, non-throttled, MPC-only) as item 73's re-measurement above, not
    a reproduction of it: this run's numbers sit about 8-9 % above item
    73's across the board (`solve_p99_ms` 94.70 ms vs 87.22 ms, `p99_ms`
    91.64 ms vs 85.96 ms, `mean_ms` 55.91 ms vs 51.08 ms, `modes` auto 559 /
    saturated 41 vs auto 562 / saturated 38), which is ordinary run-to-run
    variance on this board, not a regression or a conflict between this
    entry and §4's table -- read the two as separate measurements of the
    same board, not restatements of each other). A same-session run without
    `--profile-phases` gives `solve_p99_ms` 91.69 ms, so the wrapper's
    own overhead on this board is about 3 ms (~3 %), matching the module
    docstring's own caution that `--profile-phases` "costs a little
    overhead of its own".

    The four phases add up to the whole: `bookkeeping_ms` is defined as
    the tick's own elapsed time minus the other three (module
    docstring), so the identity holds by construction every solve tick
    unless that subtraction would go negative, in which case the code
    clamps it to zero and the identity breaks. Checked directly on this
    board (same config and seed, 300 solve ticks, phase wrappers active
    throughout): the clamp never triggered, and the mean of the four
    phases (49.60 ms of named phases + 30.21 ms bookkeeping = 79.80 ms)
    matches the mean elapsed time over those same solve ticks (79.80 ms)
    exactly; the closest any tick came to the clamp was a 24.51 ms
    margin, nowhere near zero. The one place the numbers above do *not*
    sum cleanly is the p99 column -- the four phase p99s add to
    97.79 ms, about 3.1 ms over `solve_p99_ms`'s 94.70 ms -- but that gap
    is a percentile artefact, not lost time: each phase's p99 is that
    phase's own worst tick, taken independently, and the tick worst for
    `bookkeeping_ms` need not be the tick worst for `sqp_box_qp_ms`, so
    their p99s do not have to add to the p99 of the sum. Nothing is
    unaccounted for.

    This settles the tooling half of item 95: the per-phase split works
    on the board, on real hardware, exactly as the dev-machine sanity run
    said it would. The owner decision above (whether the remaining
    solve-tick time is worth regenerating the goldens) is unaffected by
    this and still open.
98. Tune `model_max_air_dist_c_per_min` on the real enclosure (item 66).
    The shipped 8.0 °C/min is set above what the `rich` truth simulator's
    *drawn* physics produce on a healthy enclosure, where the prior's air
    node is already as wrong as a 2× airflow error; with a fitted model
    (§13 stage 3 onward) the healthy air disturbance should sit far lower
    and the limit can come down, which is what buys detection between
    0.25× and 0.5× airflow. It is also the tuning the shipped default
    needs rather than merely deserves: on the simulator the healthy
    ceiling is 7.44 °C/min (`rich` seed 6) against the limit 8.0, about
    7 % of margin, held only by the 120 s dwell (item 66). The work is on
    the running enclosure, not the
    hardware bench: record `solver_diag.model.checks.air_dist_c_per_min`
    over a quiet week with a converged model (ignoring the first
    `model_air_dist_tau_s` after every restart, where it reads `null`),
    take its ceiling, and set the
    limit a factor above it — a factor of two over a fitted model's
    ceiling should land far below 8.0; then check a real fouling event (a
    filter deliberately blocked) enters the fallback. `model_air_dist_tau_s`
    (900 s) sets how slowly the reference level follows, so a genuine slow
    drift of the enclosure is not a fault: lengthen it only if a real
    seasonal drift trips the check.
99. The four DAS goldens moved with item 67 and were regenerated on its
    branch, in one commit of its own so it can be dropped
    (`tests/golden/das_regulation.*`, `das_hotswap.*`). The example config
    declares two redundant proximal pairs (b03, b10), so a filter that carries
    a placement offset for the second member of each cannot leave those
    trajectories where they were. The legacy goldens
    (`regulation_noise_disturbance.*`, `setpoint_steps.*`) are untouched and
    bit-identical: they run on a config without `topology`, which has no
    estimator. How far the DAS four moved, over 480 ticks each: max |Δpwm|
    8.9e-4 (`das_regulation.pi_das`), 4.8e-4 (`das_hotswap.pi_das`), 6.2e-2
    (`das_regulation.mpc_das`), 3.1e-2 (`das_hotswap.mpc_das`); max |Δtemp|
    0.0625 °C (one DS18B20 LSB) on all four; `mode` differs on 20 of 1920
    ticks, all of them `auto` ↔ `saturated` in `das_regulation.mpc_das`;
    `zones_in_fault` is identical on every tick of all four and no run has a
    drive over its limit. **Owner decision:** keep the regenerated
    trajectories, or drop that commit and leave the four
    `test_das_core.py::test_golden_trajectory` cases failing until item 67 is
    reworked to leave the example alone (which would mean shipping
    `proximal_offset_c: 0` in `config.example-das.yaml`, i.e. not fixing
    item 67 for the example's own redundant pairs).

    **Where that commit is:** `522a39d0de08ef3c4ce985f10d18c1a17086de54`,
    "goldens: regenerate the four DAS trajectories (drop this commit to
    keep the old ones)", 2026-09-16, directly after item 67's estimator
    commit on `main`. `git show --stat` on it touches exactly the four
    files above and nothing else — no code, no other golden, no other doc
    — so `git revert` of that one commit is the whole "drop" side of the
    owner decision above; it has not been reverted, so the four DAS
    goldens the tree ships today are the regenerated ones and
    `test_das_core.py::test_golden_trajectory` passes against them. That
    is this document recording where the commit sits, not the owner
    decision above being resolved — nothing here answers it.

    **Before ever regenerating `tests/golden/das_*.json` again**, a reader
    should: confirm the change is isolated to `tests/golden/*` in its own
    commit exactly the way this one is, never folded into the code change
    that moved the goldens, so either can be dropped without the other;
    confirm the two legacy goldens (`regulation_noise_disturbance.*`,
    `setpoint_steps.*`) are still untouched and bit-identical — if a
    change moves those too it reached the legacy, no-`topology` path and
    breaks the safety contract's bit-identity requirement (§2) regardless
    of how reasonable the DAS numbers look; and read the move itself —
    `max |Δpwm|`, `max |Δtemp|`, and every tick where `mode` or
    `zones_in_fault` differs (`GOLDEN_PWM_ATOL` 1e-6, `GOLDEN_TEMP_ATOL`
    1e-4, `AQUA_BRIDGE_REGEN_GOLDEN=1` to produce the new file,
    `tests/test_das_core.py`) — against the specific code change that
    caused it, the way this item's own numbers above are read against
    item 67's added offset states. A move that cannot be explained that
    way by the change that supposedly caused it is a bug to fix, not a
    golden to accept. **The rule stays what item 73's brief already
    set: goldens are regenerated only on an owner decision, in an
    isolated commit of their own, never to make a failing test pass.**
100. **Done** (2026-09-17): the two notions are untangled, and they were not one
    rule wearing two hats — they were two questions. The estimator is the owner
    of both: it marks every reason a bay is not itself (`jump`, `occupancy`,
    `uncertain`, `calibration`) and publishes two verdicts, each with its reason
    and the seconds it has left (§3 *One owner, two exemptions*). `settling` is
    the `sigma` trust rule's exemption and answers *"did the filter widen this
    bay deliberately?"* — an event, still budgeted by `bay_settle_max_s` and
    still granted only to an `observed` bay. `model_exempt` is the DAS MPC
    validity gate's and answers *"is this bay tight enough to score a model
    against?"* — a level, which a bay nobody is reading fails as surely as one
    just swapped, and which the trust rule must **not** excuse, since that is the
    observability loss it exists to catch. So a jump reaches the model gate
    through `uncertain` rather than as an event, and the solver's
    `_settling_bays` is now one read of the published flag; its
    `SETTLE_DRIVE_VAR_C2` and `SETTLE_CAL_STEP_C` became the config keys
    `estimator.bay_uncertain_var_c2` (1.0) and `estimator.bay_cal_step_c` (0.05).
    The published set is the same one the solver used to infer, tick for tick, so
    **the four DAS goldens do not move** — the part item 99 was blocking (the
    model gate honouring the `jump` *event* instead of the `uncertain` *level*)
    is the part that is not needed: the level is the better question for that
    gate, and the event is the better question for the trust rule. One behaviour
    does change, in the intended direction: the marks lived in the solver's memory,
    which `initialise()` cleared, so a bumpless transfer used to drop up to
    `bay_settle_s` of them and put a still-settling bay back into the checks. They
    live with the estimator now and survive a solver restart.

107. **Done** (2026-09-17): every reader of the fan-curve data was found
    (grep for `fan_models` and `fan_curves`) and decided one way or the
    other, in §3's *Fan curve* table, in `fancurve.READERS` and in
    `diagnostics["fan_curves"]["readers"]`, which reports that table so
    nobody has to guess which curve produced a number. **Now follow the
    fit**, next to the thermal model and the MPC's prediction of item 14:
    the estimator's airflow `Q_z` / `Qn_z` (`estimator._airflow`, which took
    `cfg` alone and now takes the curves), so the estimator and the model
    cannot disagree about the air a fan moves and have the prediction-error
    guard answer the difference with a fallback — safe, but louder than it
    needs to be; and the noise model's `u0`
    (`noise.channel_deadband`, read by `surrogate`, `_level`, `rpm_model`
    and `noise_diagnostics`; `channel_power` and `noise_db` read no `u0`,
    they take the fraction their caller computed), because it changes the
    objective, not the safety, and a surrogate charging for noise over a
    band where the fan does not turn sends the solver to the wrong command.
    **Deliberately stay on the configured curve:** the fan
    health rpm and power rules (a curve fitted to the same tachometer
    readings would follow a fan that slows down and the deviation would
    never show — a floor derived from the configured curve must not drift
    with a fit), the gate's Stuck airflow evidence (a sensor-validity rule,
    derived from the config once at load), `noise_db_at_max` (a datasheet
    figure the fit says nothing about) and every `rpm_max` normalisation
    (`model_use_rpm`, the noise diagnostics' tach branch), for item 14's
    own reason. A fit that nothing re-confirms for `fan_curve_max_age_s`
    (new key, default 3600 s, validated `≥ fan_curve_refit_s`) goes stale:
    it leaves `solver_memory["fan_curves"]` and every reader falls back to
    what was in force before it — the model store's seeded curve when there
    was one (the stale rule abandons this run's fit; it never deletes the
    stored curve from the section `model.json` saves), else `fan_models`.
    The two stamps that re-confirm a fit are the tick that accepted it and
    the tick of the newest **tachometer reading** of that fan model, not of
    the newest settled `(pwm, rpm)` sample: the DAS QP moves the duty on
    nearly every solve, so a settled-sample rule would flip a healthy
    controller config → fit → config on an hours timescale while the
    tachometers were reporting all along. The noise model's shape is
    unchanged — `r(u)` is still linear in the dead-band-shifted duty, only
    `u0` moves — so no golden moves and the legacy path is untouched.
101. **Done, off by default** (2026-09-17): the estimator can now hold a sensor
    node and a map per proximal *sensor* rather than per bay
    (`estimator.proximal_slope_spread`, default 0 — §3 *A node per proximal
    sensor*). The representation defect was real: two placements differ in the
    fraction `β` of the drive they see, so their disagreement is
    `Δs·(T̂_d − T̂_a) + Δb` and a constant offset (item 67) cannot carry the first
    term. Each member now gets its own node with its own `tau_s` and its own map
    `s + Δs_i`, `b + Δb_i`, learned by a two-parameter RLS from the two readings
    alone — no SMART, since both members see the same drive and the same air.
    That RLS needs two things to be no worse than the offset state it replaces,
    and both are in it: `Δb` is a **random walk** with the same `estimator.q_offset`
    the fused offset state uses, so a placement that drifts (fouling, a loosening
    sensor, a thermistor ageing) is followed by the member's own map instead of
    biasing the drive both members share; and `(Δs, Δb)` is **bounded**, because a
    row identifies only the combination `Δs·rise + Δb` and the pair would otherwise
    walk along that null direction for ever (measured unbounded on the two-plateau
    fixture: `Δs` +2.5 against a prior spread of 0.25, `Δb` −51 °C, and a drive
    estimate 2.4 °C *under* truth with σ still at the uncalibrated floor). A pair
    that leaves its box is slid back along that same null direction, so the member
    keeps predicting what the evidence says it reads.
    The offset block becomes the extra nodes, so the **state size and the step
    cost are unchanged** (measured: DAS MPC step p99 3.10–3.45 ms against
    3.08–3.26 ms fused, two repeats of `tools/bench_step.py --sim-plant das
    --ticks 400`; the PI-like DAS path pays about 0.4 ms for the extra nodes and
    the RLS row, which is still two orders of magnitude inside the budget); the
    bay's own node stays the anchor's, so `cal`, the occupancy machine and the
    association are untouched and the model store needs no migration.
    Three things it does **not** do, which is the reason for the default:
    (a) turning it on for the example's two redundant pairs moves the four DAS
    goldens — item 99's decision;
    (b) on the closed loop it changes almost nothing. Measured over the whole
    4500 s `rich` run, seeds 0, 1, 2, 3, 5 and 7, first 300 s discarded: every
    bay's drive-estimate rms agrees between the two layouts to 0.01 °C on five of
    the six seeds, and on seed 0 b10 goes 3.40 → 3.56 °C and b09 3.62 → 3.90 °C.
    The horizon matters — over 900 ticks the two look identical everywhere, which
    is not long enough for the bound above to have anything to do. The premise
    about the accuracy is also stale: measured today, b03 and b10 are not the
    worst bays on any seed tried — the worst per-tick error belongs to
    single-sensor bays (b05 +6.54, b06 +6.83 °C during the warm-up);
    (c) the slope is **not identifiable from regulation alone**: the drive-to-air
    rise barely moves, so `Δs` and `Δb` are collinear and the RLS recovers about
    half the true slope on `rich` seed 2 and the wrong sign on seed 0. The
    *combination* `Δs·rise + Δb` is right on every seed, which is exactly why the
    estimate does not move — the same excitation problem the fan gains `E` have.
    What it does fix is the part item 67 left: a member that goes quiet leaves a
    constant offset at the rise it last saw, so when the load moves under it and
    it comes back the reading is a swap (σ 4.67 °C, `settling: jump`); with a node
    of its own the member is predicted through the same drive and air the anchor
    sees and σ stays at the uncalibrated floor 1.50 °C
    (`tests/test_estimator.py::test_a_member_that_returns_after_the_load_moved_is_a_swap_only_on_the_fused_node`).
    That is what keeps the pairs costing the `sigma` trust rule an exemption.
102. **Done** (2026-09-17): a closed-loop scenario in which the fit converges,
    and the reason none did before. *What it was:* every convergence result
    came from `tests/test_thermal_ident.py`, which drives the outputs open
    loop (fixed levels 0.35/0.8, independent sequences on every channel at
    once) and never runs `control/ident.py`, while `ident.py` was measured
    only where the fit does not converge.

    **Diagnosis** (DAS example, `rich` preset, real `Loop` + `Supervisor` +
    `control/ident.py`, PI-DAS, 4–24 h per run). Of the four conditions in the
    `converged` rule only two ever fail: `excited_windows` reaches 110 of 119
    within 4 h, the air block's `rel_se(E)` is 0.03–0.15 against the 0.25
    bound and `pred_err_c` 0.05–0.10 against 1.0 — the air block's `pe_min`
    and the bays' `rel_se(k)` are the whole of it. Per hypothesis:

    - *conditioning, not aborts, not the window, not the siblings.* In a
      three-group zone the group regressors correlate 0.91–0.98 and the
      smallest eigenvalue of the normalised covariance is 0.002–0.05 against
      diagonals of 0.06–0.17: one direction is excited, not three. Aborts are
      not the cause (0–1 per 12–24 h run, and the baseline 4 h run aborts
      nothing at all, completing 7 of 8 starts with the experiment running
      96 % of ticks). The measure is not computed on the wrong signal: the
      per-window and per-tick eigenvalues agree to under 10 %. Holding a
      zone's siblings, the fix the module docstring applies within a group,
      does not do it either — replayed on the measured regressors it moves z1
      from 0.0165 to 0.0379 and makes z3 *worse*.
    - *regulation alone is exactly zero.* The rank-1 common-mode fit of the
      measured regressors — what the solver's own motion contributes — scores
      `pe_min` 0.0000 on every zone.
    - *sequential vs simultaneous, same amplitude, same background.* Replayed
      on that background: one channel per slot peaks at 0.008–0.030 and never
      clears `PE_MIN`; an independent code per channel peaks at 0.09–0.11 and
      holds above it in 61–68 % of windows. A factor of about twelve, from
      simultaneity alone. (That is the replay, on one fixed background. In the
      closed loop a sequential round robin does clear `PE_MIN` in bursts — the
      scenario below has those numbers — but does not hold it.)
    - *why the amplitude also matters.* `pe_min` is a **relative** measure: it
      wants a relative airflow std of √`PE_MIN` = 0.224 in the least-excited
      direction. For a 50/50 telegraph at base `u` with deadband `d`, `above`
      gives `A / (2 (u − d + A/2))` and `symmetric` `A / (u − d)`, so `above`
      needs `A ≥ 0.576 (u − d)` — above the `ident_amplitude` cap of 0.3 for
      any channel parked past ≈ 0.62 PWM. Measured on the example with
      regulation only, the channels park between 0.21 and 0.94 PWM, which asks
      0.06–0.49; the one channel that asks more than the cap is also the one
      the `saturated:` precondition already refuses to excite.

    **What changed:** one config key, `ident_parallel` (default `false`, so
    the old schedule is byte-identical). With it an experiment drives every
    channel of the target's own zones at once, each on its own code (§3). No
    change to the levels, the preconditions, the envelope, the abort list or
    `compose`.

    **The scenario and what it reached** (`tests/test_ident_converge_sim.py`,
    nightly): `ident_parallel: true`, `ident_amplitude: 0.25`,
    `ident_hold_s: [240, 360, 480]` (longer than `model_window_s`, so a window
    sits inside one hold), `ident_max_duration_s: 3600`
    (`PE_WINDOWS × model_window_s`, the monitor's own memory — a shorter
    experiment can never dominate it), starts on `xt1, xt3, xt2, xt4` (one per
    zone), 16 h, `rich` seeds 2/3/4. **The control arm is a round robin over all
    eight channels** (`xt1, qd1, xt3, qd3, xt2, qd2, xt4, qd4`), the schedule a
    daemon cycling every fan group would actually run: the same 16 starts of the
    same length and the same amplitude, one channel each instead of a zone. Both
    arms run on the same plant and the same seed.

    (Everything from here on was measured **before item 112**, which changed
    where the fans park between experiments and moved every one of these
    numbers; item 112 has the after. The reasoning is unaffected, the
    arithmetic is not. Re-measured **after item 109**'s fix, which was
    separately holding z0 down on seeds 3 and 4; seed 2 is unchanged to every
    digit.)

    Zones converged, zone-wide: **z0+z3 / z0+z1+z2 / z0+z3** (seeds 2 / 3 / 4),
    with `pe_min` peaking at 0.12–0.34 / 0.25–0.35 / 0.30–0.36 and still
    0.08–0.30 at the end of the run, 443–478 excited windows and `pred_err_c`
    0.05–0.08. One channel at a time: **nothing / z1 / z0**. That is the
    honest comparison and it is narrower than "nothing converges
    sequentially": the round robin does push `pe_min` past `PE_MIN` (peaks
    0.156 on z0 seed 2, 0.421 on z0 seed 3, 0.413 on z0 seed 4, all above the
    0.05 bound); on seed 3 that was enough for z1 to latch `converged` — with
    its live `pe_min` back at 0.0076 by the end, i.e. from a transient the
    monitor no longer holds — and on seed 4 it *holds* the bound on z0, 0.208
    at the end of the run. **z0 is the one zone of the example with two fan
    groups** (`xt1, qd1`) rather than three, so it has one direction fewer to
    separate and is the zone a round robin can do most for — the only zone it
    converges and holds. What the zone-wide excitation buys is a zone more
    per seed on all three seeds, and the bound *held* on every zone it
    converges. Restricting the control arm to the four aquaero channels (the
    four targets the zone-wide arm starts on) converges nothing on any seed,
    but that measures channel coverage, not simultaneity, and is not the arm
    the test runs.

    Zero limit violations in either arm; worst true margin 4.59 / 4.89 / 4.53 °C
    zone-wide against 4.22 / 4.29 / 4.25 °C for the round robin and 4.07–4.30 °C
    with no experiment at all — `above` only ever adds cooling. The fit: per-bay
    `k` rms error 0.10–0.18 against 0.22–0.29 with no experiment.

    **What it costs.** Noise, and only while it runs: mean PWM over the 16 h
    +0.050 / +0.037 / +0.036 (seeds 2 / 3 / 4) against the eight-channel round
    robin, on the same starts and the same amplitude. Per *zone identified* the
    fan-seconds are arithmetic-neutral — one zone-wide experiment does the work
    of the `G` sequential ones its zone needs, in a `G`-times shorter window —
    but that is arithmetic, not this A/B: both arms here run the same 16 h of
    experiment, and the mean-PWM gap above is what that costs. Against *no*
    experiment the cost depends on how hard the enclosure is already working —
    on a quiet seed +0.045 mean PWM and about +13 dB of mean fan noise, on a
    warm one −0.6 dB and +0.011 mean PWM, because the extra cooling an `above`
    telegraph supplies comes back off the solver's own demand. Under
    `ident_levels: symmetric` there is a second cost, in cooling rather than
    noise: a coded phase can have every channel of the zone at its low level on
    the same tick (10 % of the ticks of a three-channel phase), so the whole
    zone may sit `ident_amplitude` under the solver's command where the old
    schedule could only dip one group of it (§3).

    **What is not reached, and why.** The *model* status (the least advanced
    zone) is still `learning` in every run: no seed gets all four zones. Two
    reasons, both measured and neither an excitation problem. (a) A channel
    parked past ≈ 0.62 PWM cannot clear `PE_MIN` under `above` at any allowed
    amplitude (the arithmetic above); `symmetric` can, at the owner-accepted
    dip, but the band check refuses it once the base is within
    `ident_amplitude` of either rail. (b) *Was* on `rich` seeds 3 and 4 bay
    b03 — one of the two bays with a redundant second proximal sensor — reported
    `swapped` on 2788 and 2391 of 2880 ticks, so `model_reset_on_swap` reset its
    block and its zone's air accumulator on nearly every tick and z0 closed 0
    and 15 regression windows where the other zones closed about a hundred,
    which no excitation could fix. **Fixed by item 109**; with it z0 converges
    on all three seeds and the numbers above are the re-measured ones. Reason
    (a) stands, and no seed gets all four zones.

103. **Done** (2026-09-16): the Pi's own temperature and throttling as a
    health signal (owner decision above, §8.1). `hostinfo.read_throttled()`
    reads the board's throttling state from whichever of three sources this
    kernel has — the sysfs attribute
    `/sys/devices/platform/soc/soc:firmware/get_throttled` first, then
    `vcgencmd get_throttled` (short-circuited on `PATH`, so no process is
    started on a machine that is not a Pi), then the `rpi_volt` hwmon
    device's `in0_lcrit_alarm`, which is the under-voltage condition alone
    and is reported as exactly that; `null` when none reads and never an
    exception, like the rest of `hostinfo`. `decode_throttled()` turns a
    whole word into named booleans: `under_voltage`, `freq_capped`,
    `throttled`, `soft_temp_limit`, each `_now` and `_since_boot`, plus the
    `now` / `since_boot` summaries, the raw word and its hex; a source that
    saw less leaves what it did not read `null`, names it in `unknown`, sets
    `partial` and summarises to `null` rather than `false`, so an unknown bit
    is never read as an absent one. Every reading carries its `source` and,
    for a `vcgencmd` word served from the cache, its `age_s`. It rides
    `collect_hostinfo()` as the `throttled`
    key, so `/api/state`'s `host` and the MQTT state blob carry it
    (item 25). `health.HostHealth` adds the three rules and the
    `host_health:` keys described in §3 ("The board itself"), runs on the
    same `on_tick` observer as the fan health with a `CachedHostInfo` on
    `host.interval_s`, and publishes its verdict
    as `device_health.host` — so `/api/state`, `/api/health`, the MQTT state
    blob and the page show it the way item 83 shows device health, plus a
    Home Assistant binary sensor `host_problem` of its own (`device_class:
    problem`, diagnostic) whose attributes are the host blob. Only the two
    rules that report a fact put their problems in the daemon-wide
    `problems` list; the divergence hint does not, so it never makes
    `/api/health` not-ok or turns on `device_problem`. `air_temps` is
    cross-checked against `mpc.temps` in `main()` before anything is opened,
    so a typo is exit 2 and not a traceback with the hardware open. The board stays **out** of `PlantObservation` and out of
    the `diagnostics` the solver reads: no golden changes, and a failure
    here cannot reduce cooling (§2). Tests: every bit of the throttled word
    in both halves, an unreadable source, a malformed word, a missing
    binary and a raising runner (no test shells out to `vcgencmd`); the
    hwmon source found by `name` among several hwmon directories and not by
    index, alone as the whole chain, and claiming nothing it did not read;
    the `vcgencmd` cadence counted against a fake clock and a fake runner,
    the cached word reused with its age, and a failed poll falling through
    to the hwmon bit without a retry before the interval is out; the rule
    firing from the under-voltage bit alone and staying silent when that one
    bit is clear; each
    rule firing and clearing against a fake clock; the idle gate switching
    the divergence rule off and restarting its window; the air reference and
    its default; the publishers' payloads and the Discovery entity.
    Follow-up: the defaults are reasoned, not measured — item 94's kind of
    tuning applies here too once the board sits in the finished enclosure.
    `divergence_c` in particular ships as a coarse backstop (40 °C) because a
    healthy idle Zero 2 W is already 20–25 °C from room air; it means little
    until someone measures the board-vs-air delta where the Pi actually sits,
    and only then should it be narrowed. **The source question is now
    answered** (measured on the board, 2026-09-16, §8.1): on 64-bit trixie
    with kernel 6.18.50-v8 the firmware sysfs attribute is **absent** and
    `vcgencmd` is the only source of the whole word, so the control tick runs
    it too — once per `host_health.vcgencmd_interval_s` (60 s; one run is
    3.3 ms median, 0.07 % of the tick it lands on), with
    `vcgencmd_timeout_s` bounding the one call that could cost more. The
    `rpi_volt` hwmon alarm covers under-voltage on a board that has no
    `vcgencmd` either, and only `hostinfo.THROTTLED_SYSFS` changes if the
    attribute lives elsewhere on some other kernel. Still open: on a board
    whose only source is that process, "throttling now" can be up to one
    `vcgencmd_interval_s` late — acceptable for a health signal (the firmware
    latches the `_since_boot` half regardless, and the message carries the
    word's age), and lowering the key trades forks for latency. A cheap way
    to do better would be to escalate: the 0.17 ms hwmon bit could be read
    on every tick and an under-voltage it sees force a word refresh ahead of
    the cadence. Not done — it complicates the chain for the one condition of
    the four the hwmon source already reports on its own.
104. **Done** (2026-09-17): a manual calibration (item 23) survives a restart.
    Its RLS entry rides in the model store's own per-bay
    `manual_calibration` section, next to the per-serial `calibration` one
    (a handheld reading has no serial to key it by), with its sample time
    converted to wall clock and re-based on the new controller clock the way
    a SMART entry's is. The rule for what comes back is stricter than the
    SMART one and lives in one place (`estimator.restore_manual_calibration`,
    §3 *Manual calibrations survive a restart*), because a stale calibration
    silently trusted is worse than none — every doubtful case is a **drop**,
    which returns the bay to the prior map at `sigma_uncalibrated_c`, i.e.
    more margin and more cooling: (a) **old** — past
    `estimator.manual_calibration_max_age_days` (new `estimator` key,
    default 7 wall-clock days, the outage included), or with an age that is
    unknown or negative, it is dropped outright rather than restored with its
    fresh count reset the way a SMART entry is, because nothing but the
    operator can refresh a handheld map; the window is shorter than
    `calibration_max_age_days` because the daemon was not watching and the map
    is keyed by bay, not by serial; (b) **swapped** — each entry stores the
    bay's declaration at the snapshot (`occupied`, `class`, `serial`, exactly
    the policy the store fingerprint leaves out) and any difference in the
    running config drops it by name, while the *evidenced* half of the running
    daemon's own swap rule (item 12: the bay's occupancy crossing `empty` in
    either direction) drops that bay's manual entry on the tick it fires,
    restored or measured in this run alike — the other half, a bare fast-swap
    proximal step with no occupancy crossing, only *inflates* the entry
    (`inflate: 2.0` / `confirm: 20`), because one tick's innovation is raised by
    a spin-up or a fan step as readily as by a swap and the identical event
    leaves a SMART entry alone; (c) **always provisional** — a restored entry always comes back
    `inflate: 2.0` / `confirm: 20`, out of a `fresh` file as much as a `stale`
    one, since a bay-keyed map has no feed that could re-associate it after a
    swap the daemon slept through. Visible as
    `diagnostics["store"]["sections"]["manual_calibration"]`, with a named
    warning for every entry that did not come back.
105. **Done** (2026-09-17), and the answer is **off by default**. There is now
    an MQTT counterpart of `POST /api/calibrate` —
    `{node_id}/cmd/calibrate/<bay>`, a raw number in °C — behind the explicit
    opt-in `mqtt.allow_calibrate` (new `mqtt` key, default `false`). Without
    it the topic is not subscribed *and* the parser refuses it, returning no
    intent at all, so a message that arrives anyway never reaches the
    supervisor or the estimator. Why it is off when `cmd/limit` and `cmd/bay`
    are on, all three having only the broker's ACL behind them: a limit or a
    declared occupancy is policy the gate re-clamps every tick and the safety
    core bounds, while a calibration is a *measurement* — it fits the bay's
    sensor-to-drive map, and a map biased low makes the controller run the
    fans slower than the drive needs. It is the one inbound payload that can
    quietly reduce cooling, so it asks for a deliberate `true` in
    `config.yaml`. Turning it on also requires `mqtt.username` (an anonymous
    connection cannot be given an ACL of its own; refused as a named setup
    error at startup, not per message — the check is that a username is
    *configured*, not that the connection is authenticated: nothing inspects the
    CONNACK or the ACL, and enforcing the account is the broker's job), ignores a
    **retained** reading the way
    a retained experiment `start` is ignored (it would be refitted into the map
    on every reconnect and every restart — item 104's failure over the wire),
    and publishes **no** Discovery entity: fifteen more `number` entities whose
    state Home Assistant republishes on its own restart is the retained-message
    problem in another costume. §7 *Manual calibration over MQTT*.
106. **Done** (2026-09-17): nothing at startup checked a config against the
    one-controlling-controller shape (owner decision 2026-09-16, §8.1, §2
    "Supported topology"). `hw/sources.py::_check_quadro_commanded_once`
    refused a commanding quadro entry without a `serial:` when the
    aquaero already commanded aquabus outputs 5–8 (both would reach the
    same Quadro), but a commanding quadro entry *with* a `serial:` — a
    second, physically distinct Quadro on its own USB port, i.e. a second
    controlling controller — was only logged as a warning. That warning
    is now the same `ConfigError` as the unidentified case, naming both
    clashing entries and the one supported shape: exactly one controlling
    device, with any slave devices (a Quadro on its aquaero's aquabus,
    for instance) hanging off it. Raised in the same early,
    before-any-device-is-opened block as the composite source's other
    config errors (`build_composite_from_config`, ahead of
    `check_watchdog`), so it fails clean at startup. A single controller
    alone (aquaero or Quadro) and an aquaero with a Quadro genuinely
    slaved to it over aquabus (no separate commanding `quadro:` entry)
    both still build, as do `config.example.yaml` and
    `config.example-das.yaml` unchanged; so does an aquaero and a Quadro
    that command their own, non-aquabus outputs independently — two
    physically distinct devices with no ambiguity between them, which
    this item's narrower scope (the aquabus-Quadro ambiguity that can
    silently write nothing, and its identified twin that is a second
    controller either way) does not reach. Whether that last shape should
    also be refused, to fully match "two independent controllers are not
    supported yet" in the general case, is the owner's call — see
    PROPOSED ITEMS.
108. **Done** (2026-09-17): `tests/test_bench_budget.py::test_das_mpc_step_p99_within_budget_ms_on_the_pi`
    was guarded by `@pytest.mark.skipif(platform.machine() != "armv6l", ...)`,
    so `pytest -m pi` silently skipped it on the owner's current board: the
    move to the Zero 2 W (items 50, 51) made `platform.machine()`
    `aarch64`, not `armv6l`. It is the one test that checks the absolute
    `mpc.budget_ms` gate on real Pi hardware, so nothing in `-m pi` caught
    a regression there any more; item 73's own re-measurement already
    worked around this by using `tools/bench_step.py` directly rather than
    `pytest -m pi`, which is evidence, not a fix.

    The check is now `platform.system() == "Linux"` and `platform.machine()`
    in `PI_MACHINES` (`tests/test_bench_budget.py`), the set of boards this
    project has actually measured the gate on: `armv6l` (the Zero W the
    budget was set for) and `aarch64` (the Zero 2 W, the owner's current
    board, items 50, 51, 73, 95). Chosen over a provisioning-set
    environment variable: an architecture check needs nothing from the
    owner to work — `pytest -m pi` on the real board just runs — where an
    env var the Pi's own run sets would be one more manual step to
    remember on every `-m pi` invocation, i.e. the same kind of easy-to-forget
    precondition this item is about. Either mechanism goes stale the next
    time the board's `platform.machine()` changes; the difference is what
    happens then. `PI_SKIP_REASON` names the exact `platform.system()` and
    `platform.machine()` seen and what `PI_MACHINES` expects, so a skip on
    an unrecognised board prints why in the pytest output instead of
    reporting nothing — a silent pass is no longer possible, whether or not
    the owner remembers to widen the list first. `tests/test_mpc_nominal.py`
    had a tolerance comment with the same `armv6l` assumption; reworded to
    point at `PI_MACHINES` instead of naming one architecture. The §4 test
    table's row for this test no longer says "only on `armv6l`".

    Not done: no new test exercises `PI_SKIP_REASON`'s wording or the
    `ON_PI` computation itself (it would need monkeypatching `platform`
    before the module import, which is more machinery than a two-line,
    reviewable check justifies) — the fix is proven by running the DAS
    suite on both an x86_64 dev machine (skips, reason visible) and relying
    on the owner's next `-m pi` run to confirm it collects and runs there.

109. **Done** (2026-09-17): a bay with a redundant proximal sensor was reported
    `swapped` on almost every tick, and then its zone could never converge. On
    `sim/das.py` `rich` seeds 3 and 4 (item 102's scenario) bay b03 — two
    proximal sensors, a DS18B20 and a thermistor at different placements —
    tripped the bay-level swap test on 2788 and 2391 of 2880 ticks, and
    `model_reset_on_swap` reset its block and its zone's air accumulator with
    it: z0 closed 0 and 15 regression windows where every other zone closed
    about a hundred, and nothing anywhere said so.

    **What it was: the statistic, not the threshold.** The test compares the
    bay's *mean* trusted proximal reading with what the filter predicts for it
    (item 12). On the fused layout the prediction used was the bay's bare sensor
    node — the **anchor's** prediction — while the mean reading is the node plus
    the *mean of the members' placement offsets*. Two placements differ by
    `Δs·(T_d − T_a) + Δb` (§3, item 101), so half that gap stood permanently
    inside the statistic while none of the offsets' variance stood under it, and
    the gap *grows with the load*, because it is proportional to the drive-to-air
    rise. Any pair further apart than `2·jump_min_c` = 1 °C was therefore a swap
    on every tick. The per-sensor branch item 101 added already did this right,
    which is why the defect was invisible there. Neither the per-sensor
    fast-swap rule nor its agreement test (item 17) was involved: they were
    correct, and are unchanged.

    **The fix** is to build the bay statistic out of the *same per-member
    innovations the rule already computes*, on either layout: `nu_bay` is their
    mean and its variance is the variance of that mean — every cross term of the
    node/offset block of `P` plus the members' measurement noise. **No threshold
    moved, no key was added, and nothing was made quieter on a two-sensor bay.**
    A bay with one proximal member is one member, so its statistic is
    arithmetically the old one; the goldens do not move and the `rich` seeds
    whose pairs never tripped the rule *in the closed loop* (2 and 5) reproduce
    to every digit there.
    The three options the item floated were all rejected: a higher threshold
    treats a bug in the statistic as noise; rate-limiting the reset would hide a
    drive genuinely swapped twice; and reporting a bay that resets this often
    would have documented the defect instead of removing it.

    **False positives, measured.** Closed loop (PI-DAS, `rich`, 4 h = 2880
    ticks, no experiment), `swapped` ticks on b03: **2788 → 0** (seed 3),
    **2391 → 0** (seed 4), **712 → 0** (seed 7); b10 2 → 0 (seed 3). z0's
    excited windows over the same runs: 0 → 109, 15 → 100, 24 → 105. In *this*
    loop seeds 2 and 5 never tripped the rule at all, and they close identical
    windows in every zone before and after — the control that shows the change
    reaches only the bays whose pair actually crossed the thresholds. Open loop
    (`tests/test_estimator.py`, the fans stepped every 120 ticks, 1200 ticks,
    seeds 2/3/4/5/7) the swing is larger and so is the gap, and it is *no longer
    those two seeds that are quiet*: one or both redundant pairs were reported on
    **1073–1199 of 1200 ticks** on every one of the five, seeds 2 and 5 included
    (seed 2, pre-fix: b03 1198, b10 1198), and on none after. A seed is a clean
    control only in the harness it was measured in.

    **Detection, measured — the other direction.** A drive replaced in place at
    3600 s of a closed-loop run by one `Δ` degrees from the one that left, the
    latency being the first tick reporting `swapped` (`dt` = 5 s; steps ±0.5 to
    ±15 °C on b02, b03, b09 and b10, seeds 3 and 4). At **|Δ| ≥ 3 °C every bay
    of either kind is caught on the first or second tick, before and after, and
    not one case is slower after than before** — several are much faster, b10's
    negative steps on seed 3 going from 6 ticks to 1. The single-sensor bays b02
    and b09 are **identical in every one of the 68 cases measured**, which is the
    arithmetic and not a coincidence. b03, which had no detection at all before —
    it was alarming on every tick — now catches every step down to ±2 °C on the
    first tick, ahead of b02, which misses −2 °C on both seeds before *and*
    after.

    The cost is below 3 °C and it is on b10. Of its eight cases under 3 °C, four
    go from caught to missed (+2.5, −2.0 and −2.5 on seed 3, −2.0 on seed 4) and
    one from missed to caught (+2.5 on seed 4); +3.0 on seed 4 also goes from
    missed to caught, and +2.0 on seed 3 goes from 1 tick to 2, i.e. 5 s to 10 s.
    That band was already sign- and seed-dependent on single-sensor bays, so it
    is inside what the rule promises rather than a new blind spot — and what now
    stands under the statistic is the filter's own placement uncertainty, which
    was always there, not a margin anyone chose.

    **What it unblocks:** item 102's dead zones. Re-measured over its 16 h
    scenario, z0 now converges on all three seeds zone-wide (was one of three),
    no zone is dead on any seed, and on seed 4 the round-robin control arm
    converges z0 too and *holds* the bound there — z0 is the one zone of the
    example with two fan groups rather than three. Those numbers are in item 102
    and in `tests/test_ident_converge_sim.py`.

    Not done: `swapped` is still a per-tick verdict with no memory, so a bay
    that genuinely flaps still resets its block on each event; the estimator's
    own `bay_settle_max_s` budget bounds the *trust* exemption but nothing
    bounds the model resets. See the proposed items.

110. **Done** (2026-09-17), and the honest answer is *say so*, not renormalise.
    `pe_min` is a relative measure: the monitor normalises each fan regressor by
    its own running mean, so `converged` wants a relative airflow variation of
    `√PE_MIN` = 0.224 in the least excited direction, and under `above` a 50/50
    telegraph at base `u` with dead band `d` reaches `A / (2(u − d) + A)` — it
    needs `A ≥ 0.576 (u − d)`, past the `(0, 0.3]` cap for any channel parked
    beyond ≈ 0.62 PWM.

    **The normalisation stays.** It is what makes the monitor read as
    *information* rather than as PWM: a fan near `pwm_max` really does move
    proportionally less extra air per unit of PWM, so dividing by something else
    would relabel an uninformative channel, not inform it. What changed is that
    the arithmetic is no longer invisible:

    - `ident.excitation(cfg, levels)` gives per channel `rel_swing`, the
      `pe_reach` it implies and whether that clears `PE_MIN`, computed on the
      channel's commissioned curve with the monitor's own scale floor. `pe_reach`
      is the **most** a group of that one channel could report on `pe_diag`: the
      monitor accumulates one sin²-weighted mean of the regressor per
      `model_window_s` block, so it reads the telegraph's own levels only while a
      regression window sits inside one hold, and a window that spans a switch
      averages the two. `holds_cover_window` (`min(ident_hold_s) ≥
      model_window_s`) rides beside it and says whether the bound is tight; the
      shipped DAS example is not in that case, `tests/test_ident_converge_sim.py`
      is. `excitable` is read at the **configured** `ident_amplitude` and
      `excitable_at_cap` at the `0.3` cap, so the owner is told which remedy
      applies — a larger amplitude, or a lower park — instead of only that the
      channel is blind. The constant bound is published as `pe_bound`, not
      `pe_min`: that name is already the thermal summary's *measured* smallest
      eigenvalue, and both structures reach `/api/model` and MQTT. It rides in
      `snapshot().extra["experiment"]` as `excitation` / `unexcitable` and in a
      `WARNING` on the start log line naming the channels that cannot get there —
      logged when that answer *changes*, not on every start, since on a busy
      enclosure most starts have one (the item 108 precedent).
    - The thermal summary gains `pe_diag` per zone: the **diagonal** of the very
      matrix whose smallest eigenvalue is `pe_min`, one entry per strong group.
      `pe_min ≤ min(pe_diag)` always, so a group whose own entry sits at or under
      `PE_MIN` shuts its zone's gate by itself, and this is what names it. On the
      three `rich` seeds the binding group is visible: seed 2 z2 reads
      `{xt3: 0.286, qd2: 0.071, qd3: 0.558}` with `pe_min` 0.047 — qd2, parked
      mid-range, is the whole of it.
    - Every zone also carries `blocked`, below (item 111).

    `mpc.ident_require_excitable` (bool, default `false`) turns the prediction
    into a refusal, `not_excitable:<ch>`, the way `band:` and `saturated:`
    refuse (§3's start-precondition list has it). It refuses on the upper bound
    above, so it never blocks a start that could have worked; with holds shorter
    than `model_window_s` it can let one through that still falls short, and
    `pe_diag` is what says so afterwards. **Off by default deliberately, and measured:** on this example a
    zone-wide start is widened to channels the solver parks high, so with the key
    on almost no start is allowed and the zones that do converge today would not.
    An experiment that cannot finish the air block's PE gate still informs the
    `E` split and the bays' `g0` / `k`.

    **Not kept, and why** (the "make the requirement depend on the headroom"
    option): a per-channel amplitude, sized to the smallest step that reaches a
    multiple of `√PE_MIN` at that channel's own base, with `ident_amplitude` as
    the cap. Implemented and measured over 16 h on `rich` seeds 2/3/4 against the
    flat 0.25: at an aim of 2·`√PE_MIN` the mean amplitude falls to 0.18–0.22 and
    the enclosure's mean PWM falls by **0.004** (0.3563 against 0.3587 on seed 2,
    0.2932 against 0.2969 on seed 4); at 1.5 it is 0.3549 and at 1.0 it converges
    **nothing** on any seed. Under `above` the extra cooling an experiment
    supplies comes straight back off the solver's own demand (item 102 measured
    the same effect), so spending less on the telegraph buys almost no quiet — a
    config key that costs zones and saves 0.004 PWM is not worth carrying. The
    arithmetic itself survives as `ident.rel_swing`, which `excitation` reports.
    Worth revisiting only under `ident_levels: symmetric`, where the amplitude is
    cooling *given up* rather than added; that is in the proposed items.

111. **Done** (2026-09-17), and the answer is **observations, not excitation** —
    with the number of them measured. The bays' `rel_se(k)` is the second gate,
    and after item 112's release fix it is very nearly the *only* one: over three
    `rich` seeds at 16 h the air block's `rel_se(E)` never fails (0.011–0.052
    against the 0.25 bound) and 10 of the 12 zone-seeds are held by
    `rel_se:k.<bay>` alone.

    **What those bays lack.** Not separation from their neighbours: the
    correlation between a bay's own `g0` and `k` in the fitted covariance is
    −0.05 to +0.01 on every bay and every seed. Not excitation that misses the
    bay: the proximal block's own `Qn_z` regressor reads `pe_min` 0.06–0.26,
    against the air block's 0.08–0.21 on the same runs. What is left is
    precision. `rel_se(k)` is `se(k) / |k|`; across 45 bay-seeds `se(k)` lands in
    0.04–0.22 whatever the bay, while the fitted `k` spans 0.22–0.75. **The gate
    fails on the bays with the least airflow sensitivity to measure, on a fit no
    worse than their neighbours'** — the pass/fail line runs along `|k| ≳ 4·se`,
    which is the bound written out. Each bay now publishes `se` beside `rel_se`
    so that is readable.

    **What it would take: time, and about twice what a 16 h run gives.** `se(k)`
    falls as 1/√(excited windows). Running the *same* schedule for 36 h instead
    of 16 h closes 900–1000 excited windows per bay instead of 390–450, drops
    `se(k)` from 0.10–0.16 to 0.07–0.11, and takes the converged zones from **3
    of 12 to 7 of 12** (seed 2: z0, z1, z3; seed 3: z1; seed 4: z1, z2, z3).
    Amplitude cannot substitute: matching that gain needs about 5× the `Qn_z`
    variance, a relative zone-flow swing of 0.55–1.1 against the 0.24–0.51
    measured, which no `ident_amplitude` in `(0, 0.3]` produces.

    **Of the five zones that still do not converge at 36 h, four are item 109,
    not identification.** b03 and b10 — the two bays with a redundant second
    proximal sensor — are reported `swapped` on nearly every tick and close 0–22
    windows in 36 h, so they block z0 on seeds 3 and 4 and z2 on seeds 2 and 3
    whatever the excitation. The fifth is b15 on seed 3, whose fitted `k` is
    0.215: less than half the prior, so even `se` 0.109 is 50 % of it.

    **So the convergence rule is left alone.** The all-or-nothing question the
    item raised — letting a zone converge on its air block plus the bays that
    have informed themselves, the rest on the prior — is a policy decision about
    acting on a partly identified model, and it is not needed to reach
    `converged`: waiting is enough on 7 of 12, and on 4 of the remaining 5 a
    partial rule would only paper over item 109. It is in the proposed items with
    these numbers, for the owner to decide.

    What ships is the visibility. Every zone of the thermal summary carries
    `blocked`: the parts of the `converged` rule it still fails, in the rule's own
    words — `windows:<zone|bay>`, `pe:<zone|bay>`, `rel_se:<coefficient>` and
    `pred_err`. It comes from the same function the status machine decides with,
    so the published list and the decision cannot drift apart, and it is empty
    exactly for a `converged` or `frozen` zone. A zone that has sat in `learning`
    for a day now says which bay it is waiting on.

112. **Done** (2026-09-17): the release hands the solver its own level, not the
    fan's. The supervisor used to put a finished experiment's channels into
    `TickPlan.released`, which drops their integrator entries and makes `step`
    re-initialise the solver bumplessly at *the PWM on the fan* — the
    experiment's own level. A channel released high was handed that level as the
    solver's starting point and stayed there until the integral wound it down,
    and a later start took the inflated command as its base.

    The fix is to stop releasing them. The experiment's levels are an override
    applied *after* `mpc.step`, so the solver ran on every one of its ticks and
    the integrator kept at the release is its own state, advanced throughout and
    never re-seeded from the fan; keeping it returns control at the solver's own
    demand rather than at the experiment's level. *Near* it, not bit-exact, under
    the DAS MPC: its first block is solved with `weight_dpwm·‖u₀ − prev‖²` and
    `prev` is the PWM in force, which during the experiment is the experiment's
    own level, so the stored core keeps a small residual pull toward it — in the
    safe direction, more cooling than the counterfactual — and `_replay_core`
    hands the same plan back for up to `mpc_every_ticks` ticks after the release.
    Raising either knob makes that residue larger; at the shipped values the
    measurements below show the ratchet gone. On the PI-DAS branch there is no
    such term and the integrator is exact. Nothing steps: `step` still
    rate-limits against the PWM on the fan, so the walk back is `d_pwm_max` per
    tick like any other move. Under `ident_levels: symmetric` — where the old
    release could hand the solver the *low* level, below what it wanted — the
    correction is now upward. A **human** override is still released bumplessly:
    there the fan was parked by somebody outside the loop.

    **Measured** on the closed loop (`rich`, zone-wide schedule, 16 h, the old
    release reinstated beside the new one):

    | seed | mean PWM | qd1 mean PWM | qd1's lowest command | worst true margin |
    |---|---|---|---|---|
    | 2 | 0.4064 → 0.3587 | 0.790 → 0.492 | 0.326 → 0.200 (`pwm_min`) | 4.586 → 4.133 °C |
    | 3 | 0.2758 → 0.2636 | 0.362 → 0.299 | 0.200 → 0.200 | 4.893 → 4.510 °C |
    | 4 | 0.3142 → 0.2969 | 0.374 → 0.304 | 0.200 → 0.200 | 4.528 → 4.026 °C |

    Zero limit violations either way. The enclosure is quieter and warmer by
    about the same amount, and it lands back on the 4.07–4.30 °C these seeds keep
    with **no experiment at all** (item 102) — which is the point: the fans were
    elevated *after* the experiments, not during them. On seed 2 qd1 never came
    below 0.326 PWM in 16 h with the old release; it now reaches `pwm_min`. There
    is a second effect, in the identification's favour: the channels park lower,
    where a fixed 0.25 step is a larger *relative* airflow swing (item 110), so
    `pe_min` on the converged zones roughly doubles (seed 2: 0.083 → 0.216 on z0,
    0.125 → 0.205 on z3). It also moves the item 102 A/B: on seed 3 z1 now falls
    just short on one bay's `rel_se(k)` instead of latching, and on seed 4 the
    zone-wide arm's worst margin ends 0.27 °C *below* the round robin's, so
    `tests/test_ident_converge_sim.py` no longer asserts a cross-arm ordering of
    the margin (`above` never commands less cooling than the solver **on the
    tick**, which is a per-tick statement, not a claim about two 16 h
    trajectories). Both arms keep ≥ 4 °C and violate nothing.

    "Ending every phase on its low level", the item's own suggestion, was not
    taken: it would fix the completion path only, leave every abort as it was,
    and under `symmetric` the low level is *below* the anchor, which is the wrong
    thing to hand over. The integrator is the thing that was wrong.

    **A defect this uncovered, fixed here** (`control/loop.py`): on a tick where
    `step` or `compose` raises, the loop deliberately leaves its state untouched
    — but it also left `state.last_cmd` behind, while the emergency ramp *did*
    move the fans. The next healthy tick then measured `d_pwm_max` from a command
    the fans no longer carried, so it could move them by up to two steps at once
    (and after a long emergency run, by however far the ramp had walked toward
    `fallback_pwm`). `state.last_cmd` now follows the emergency ramp; nothing the
    broken tick *computed* is committed, which is what "state untouched" is for.
    **Only** `last_cmd`: a broken tick pushes no window sample, so the newest one
    belongs to the last *good* tick, and writing a later command into it (as the
    first version of this fix did, through `_with_applied`) would back-date the
    whole airflow excursion into the command history `gate._stuck` reads — where
    `stuck_pwm_lag_fraction` exists precisely to require that the drives have had
    time to answer a move before a flat proximal sensor is called stuck. The old experiment release happened to mask this on the
    experiment path (it pinned the solver at the same stale value), which is how
    it surfaced.
119. **Done** (2026-09-18), and the answer is *yes, on the bays that informed
    themselves, but never as if the rest had*. Two config keys.
    `mpc.model_converged_bays_frac` (default **1.0**, the all-or-nothing rule,
    bit-identical to what came before): a zone reaches `converged` when its **air
    block** has informed itself and at least this fraction of its occupied bays
    has, rounded up and never fewer than one. The air block is not substitutable
    at any fraction — it is the one block every bay of the zone is planned
    through — and neither is the last bay: a zone may not converge on the air
    block alone.

    **What the model may do with the bays that have not**, which is the safety
    half. Item 119 named the asymmetry and it decides this: an *under*-estimated
    `k` makes the DAS MPC believe airflow helps that bay less than it does, so it
    runs the fans higher — safe; an over-estimated one runs them lower. So the
    solver reads such a bay's `k` at its own lower confidence bound,
    `max(k − model_partial_k_sigmas · se(k), 0.05)` (`thermal.current_model`,
    second new key, default 1.0). Nothing else moves: `g0` is not a gain of the
    rule, and the air block had to pass whole. The fitted value stays in `theta`
    and the number the solver plans with is published beside it as `k_used`, from
    the same function, so the diagnostics cannot say one thing while the solver
    plans another — together with `k_se_used`, the σ that haircut was measured in,
    because on a block that has closed no window that σ is the prior's own and
    `se` refuses to publish it as a measurement (item 111). Without it the number
    driving the fans would not be reconstructable from anything published.

    **How a reader tells the two apart.** The zone publishes `partial: true` and
    `uninformed` (the bays it converged without, in topology order), and
    `blocked` — which used to be empty for any `converged` zone — keeps naming
    them in the rule's own words (`rel_se:k.b10`). `blocked` is now empty exactly
    when every block of the zone has informed itself, not merely when the gate
    accepts the zone. And `model_freeze` never enters or holds a partial zone as
    `frozen`: the switch means *this model is finished*, which a half-informed one
    is not.

    **"Has informed itself" is latched.** Asked after the fact, the question is
    about the fit and not about this minute's fan motion. `n` only grows and a
    relative standard error is a property of the covariance, but `pe` is an EWMA
    over the last `PE_WINDOWS` regression windows and decays back to nothing about
    an hour after the last experiment at the shipped `model_window_s` — the state
    a finished enclosure is in nearly all of the time. So the excitation half is
    latched at the window that first satisfies it and cleared only by a hot-swap
    reset (a block that was thrown away has informed nothing), and a model store
    written before the latch existed derives it once from what the block does say.
    The *entering* decision still reads every term live, so when a zone converges
    is unchanged. Without the latch the feature inverted on a quiet enclosure:
    `blocked` filled up on a healthy converged model at the shipped default 1.0,
    `partial` never cleared, `model_freeze` could never fire, and below 1.0 every
    bay of every zone was haircut for ever instead of the one or two the zone
    really converged without. The closed-loop harness hid it exactly, because it
    submits a new `Ident("start")` on every tick no experiment is running, so
    excitation never stops there and `pe` never decays.

    **Measured** on item 102's closed-loop scenario (`tests/test_ident_converge_sim.py`,
    `rich` seeds 2/3/4, zone-wide schedule, PI-DAS). Zones converged, per seed:

    | | 16 h | 36 h |
    |---|---|---|
    | `1.0` (today) | z0+z3 / z0+z2 / z0+z3 — **6 of 12** | **11 of 12** |
    | `0.75` | z0+z2+z3 / z0+z1+z2 / z0+z1+z3 — **9 of 12** | 11 of 12 |
    | `0.5` | z0+z2+z3 / z0+z1+z2 / z0+z1+z2+z3 — **10 of 12** | **12 of 12** |

    The bays each zone was converged without are named in the test: seed 2 z2
    without b10, seeds 3 and 4 z1 without b05, and at 0.5 seed 4 z2 without b11
    and b12. At 36 h the one hold-out of item 111 — z3 on seed 3, blocked on b15,
    whose fitted `k` is 0.215 — converges at 0.5 and not at 0.75, because b15 is
    one of that zone's *three* bays and 2/3 = 0.667.

    **What it costs, measured where it can cost anything.** Under PI-DAS nothing
    moves: mean PWM and the worst true margin are identical to the digit at every
    fraction, because the PI-like solver never reads the thermal model. The cost
    is on the **DAS MPC**, and it is the cost of *acting on the model sooner* — the
    model's status is its least advanced zone, so converging the stragglers is
    what lets the MPC act at all. Same scenario, `solver: mpc`, 36 h:

    | seed | mean PWM at 1.0 | at 0.5 | MPC ticks of 25920, 1.0 -> 0.5 |
    |---|---|---|---|
    | 2 | 0.3165 | 0.3656 | 370 -> 11146 |
    | 3 | 0.2553 | 0.2904 | 0 -> 6754 |
    | 4 | 0.2976 | 0.3566 | 5266 -> 20074 |

    +0.035 to +0.059 mean PWM, and **the derating is almost none of it**: the same
    runs with `model_partial_k_sigmas: 0` (partial convergence, no lower bound)
    read 0.3647 / 0.2885 / 0.3514, so the safety haircut itself costs **+0.0009 /
    +0.0019 / +0.0052** mean PWM. Worst true margin 4.133 / 4.510 / 4.026 degC in
    every arm, zero limit violations everywhere. That the DAS MPC is the louder
    of the two solvers on this simulated enclosure is a separate observation about
    the example's weights, not about this item; it is in the proposed items.

    **Left at 1.0** all the same. The evidence says the rule works and says what
    it costs; which fraction an enclosure should run is the owner's call on a real
    one, and shipping a default that changes when the MPC starts acting is not a
    decision to take from three simulated seeds.
120. **Done** (2026-09-18): `mpc.ident_amplitude_mode` (`fixed` | `headroom`,
    default `fixed` = what shipped) and `mpc.ident_pe_aim` (default 2.0, the aim
    item 110 measured at). In `headroom` a channel's telegraph is the **smallest**
    amplitude in `(0, ident_amplitude]` whose own `rel_swing` reaches
    `ident_pe_aim · sqrt(PE_MIN)` from where that channel is parked — falling back
    to `ident_amplitude` when even that falls short, so no channel is ever excited
    *less* than `fixed` excites it — and the two levels are **placed inside
    `[pwm_min, pwm_max]`** with the room the channel really has on each side.

    Both directions, and **bounded in both**. A level the band will not take is
    *cut* at the rail, never slid across the base: `ident_amplitude` is the owner's
    ceiling on how much PWM one experiment may add (`above`) or give up
    (`symmetric`), so no level is ever further from the base than the amplitude
    asked for, either way. Sliding a symmetric pair up off `pwm_min` would keep its
    whole `2A` swing, but it would buy that swing with up to `2 · ident_amplitude`
    *above* the solver's command — the loudest the enclosure gets, spent on the
    channel the solver had parked at the rail because it wanted quiet there, and
    the project's objective is the least modelled fan noise at the same margin. So
    the cut costs swing instead and `excitable` reports the short one (item 110's
    machinery, unchanged).

    Because the levels are then inside the band by construction, `band:<ch>` asks
    the question it was standing in for of what the cut left: a start is refused
    when the placement **was** cut *and* the swing that survived cannot reach the
    aim. Both halves are load-bearing. Refusing every cut pair would make
    `headroom` the stricter mode, which it is not; refusing on the aim alone would
    refuse the mid-band starts `fixed` allows; refusing neither would let a channel
    parked on `pwm_max` run a telegraph of zero swing, hold the schedule and every
    sibling channel for `ident_max_duration_s` and inform the model of nothing —
    with `ident_require_excitable` (off by default) the only thing left to stop it.

    **Why this matters now, measured.** Since item 112 stopped a finished
    experiment leaving its level on the fans, the solver parks the channels near
    `pwm_min` between runs — which is exactly where a symmetric pair does not fit,
    so the `band:` precondition refuses the start. On item 102's scenario at the
    shipped `ident_amplitude: 0.15` over 16 h, `ident_levels: symmetric` takes
    **13 / 4 / 8** of the 16 starts offered (seeds 2/3/4); sized to the headroom it
    takes **16 / 11 / 15**. Band refusals fall from 1858 / 8393 / 5535 to
    175 / 3339 / 586 — not to zero, because under `headroom` the refusal is no
    longer "a level leaves the band" but "the cut left no usable swing", and the
    channels the solver parks *high* still have none. At the harness's own
    `ident_amplitude: 0.25` the comparison is **2 / 0 / 2** against
    **16 / 16 / 16**, with 9971 / 11453 / 9983 refusals against 3 / 0 / 0 — under
    `symmetric` at that amplitude the experiment programme is effectively dead
    today.

    **What the dip costs at the shipped defaults.** 16 h, `ident_amplitude: 0.15`,
    `symmetric`, mean PWM given up per tick against the solver's own command:

    | seed | `fixed` | `headroom` | per start |
    |---|---|---|---|
    | 2 | 0.00668 over 13 starts | 0.00802 over 16 | 0.000514 -> 0.000501 |
    | 3 | 0.00113 over 4 starts | 0.00224 over 11 | 0.000284 -> 0.000204 |
    | 4 | 0.00582 over 8 starts | 0.00664 over 15 | 0.000727 -> 0.000443 |

    **3 % / 28 % / 39 % less cooling given up per experiment**, while a quarter to
    nearly three times as many experiments run. The deepest dip any single tick
    took falls where the sizing bites: 0.1500 on all three seeds under `fixed`
    against 0.1500 / 0.1331 / 0.1496. The **enclosure's** own mean PWM over the run
    — the noise the added excursion produces, not just the cooling given up —
    reads 0.3651 / 0.2360 / 0.2789 under `fixed` against 0.3616 / 0.2397 / 0.2822:
    within ±0.004 either way, so the whole trade is inside the noise of the
    scenario. Worst true margin 4.161 / 4.287 / 4.379 degC against
    4.171 / 4.316 / 4.148, no limit crossed either way.

    **The cut costs some of that, and the cost is the point.** With the pair slid
    up off `pwm_min` instead of cut, the same 16 h reads 16 / 16 / 16 starts and
    0.000449 / 0.000048 / 0.000210 per start — better on both counts, bought with
    up to `2 · ident_amplitude` above the solver's command. The cap is the owner's
    and the objective is the least fan noise at the same margin, so the swing is
    what gives way.

    **Under `above` the default stays `fixed`, and the measurement says why.**
    16 h at `ident_amplitude: 0.25`: `headroom` reads mean PWM 0.3563 / 0.2552 /
    0.2932 against 0.3587 / 0.2636 / 0.2969 — 0.002 to 0.008 quieter — while
    converging z0+z3 / **nothing** / z0+z3 against z0+z3 / z0+z2 / z0+z3. A zone
    for 0.005 PWM is item 110's own judgement restated with the key in hand:
    under `above` the amplitude is cooling *added* and the solver takes it
    straight back, so there is nothing there to save. Under `symmetric` it is
    cooling *given up*, and that is the path the key is for.
121. **Done** (2026-09-18): the page's Model panel gets one line per zone
    (`<zone> model`: status, `-- blocked: <reasons>` when the zone is not
    `converged`/`frozen`, `(pe <group>=<value>, …)` from `pe_diag` once the
    PE monitor has closed two windows, `(pe not yet measured, <n> windows)`
    before that — `pe_diag` reads 0 for every group until then, and 0 is the
    reading a group at `PE_MIN` gets, so it must not print like a measurement)
    and one "Unexcitable channels" line from `experiment.unexcitable`,
    or `not evaluated (no experiment)` while `experiment.excitation` is
    still empty — the normal state of a daemon with no identification
    experiment under way, which must not read the same as a channel an
    experiment actually cleared (`publishers/static/index.html`,
    `renderModel`). Home Assistant gets a `sensor` per zone,
    `model_block_<zone>` (state: `blocked` joined, or `none`;
    `json_attributes` the whole per-zone `diagnostics["thermal"]
    ["zones"][<zone>]` block, `pe_diag` included), and one `sensor`
    `unexcitable_channels` (state: `unexcitable` joined, `none` once
    `excitation` is non-empty and nothing is unexcitable, or `not evaluated`
    while `excitation` is still empty; attributes the per-channel
    `excitation` mapping) — values, not a `device_class: problem` binary
    sensor: a zone still `learning` is not a fault, only a reading about the
    model (`publishers/mqtt_ha.py`, `_sensor_with_attributes`). Nothing here
    reaches `PlantObservation` or the solver's `diagnostics`; both keys were
    already published, this only puts them where a person looks.
122. **Done** (2026-09-18): the nightly cost and the third seed.
    `test_ident_converge_sim.py` was 9 cases at ~12.5 min: the 36 h
    item-111 case ~60-136 s per seed depending on contention and the 16 h
    A/B two runs per seed similarly. (Re-measured after item 109's fix:
    with b03 no longer reported swapped, seed 3's zone-wide arm now
    converges z0 and z2 too, so all three seeds separate the two arms at
    16 h, not just two of three as first measured — the original line was
    written against the pre-item-109 numbers.) Re-measured for this item,
    isolated: the 36 h case ~67 s/seed, the 16 h A/B pair ~59 s/seed, the
    single 16 h release-check ~32 s/seed — the full 9-case module 821 s
    (13 min 41 s) under this machine's own contention (three other
    branches' suites at once), each of the three tests' slowest instance
    54-137 s against 32-67 s isolated, confirming item 126's point that
    contention roughly doubles wall clock here rather than singling out
    one test.
    **Decision**: the 36 h case now runs one seed (3) instead of three,
    dropping the module from 9 cases to 7. Seed 3 is the one with a real
    `blocked_bays` entry (b15) to exercise; seeds 2 and 4 fully converge
    with no blocked bay at 36 h and mainly re-confirm what seed 3 already
    demonstrates (the mechanism: the air block never blocks, `se(k)` falls
    with excited windows). **What is lost**: nightly no longer re-verifies
    that seeds 2 and 4 also reach full, unblocked convergence at 36 h —
    that stays a one-time, dated measurement in the test's own
    `MEASURED_LONG` rather than a nightly-checked fact, and a regression
    specific to seed 2 or seed 4 at 36 h (not at 16 h, not on seed 3) would
    go unnoticed until someone re-runs those seeds by hand. The 16 h tests
    keep all three seeds — they are the primary evidence for items 102 and
    112 and are not, individually, the expensive case. See the module
    docstring's "Nightly cost" section (`tests/test_ident_converge_sim.py`)
    for the full accounting.
123. **Done** (2026-09-18): the reset leaves a record, in both places that know
    something about it. The estimator publishes per bay, beside the unchanged
    per-tick `swapped`: `swap_reset` (this tick's verdict *as an event* — the only
    thing that reaches `thermal.update`, item 124), `swap_count`, `swap_last_s`,
    `swap_reason` (`jump` | `occupancy`, the estimator being the one that knows
    which, item 100) and `swap_held` (events the rate limit refused for want of
    budget — counted at the cadence an accepted event would have had, one per
    `bay_settle_s`, so it is a count of events and not of ticks and a threshold set
    against it does not depend on `dt`). The thermal block carries its own
    `resets` / `last_reset_s`, and they
    are the **one thing a reset carries over**: `g0`, `k`, `q_s`, the covariance,
    the counters, the PE monitor and the window in progress all start at the
    prior, the count does not. Both ride to `/api/state` and `/api/model` with the
    other per-bay diagnostics.

    Why the thermal side counts separately rather than reading the estimator's:
    `model_reset_on_swap` can be off and a `frozen` zone is skipped, so "how often
    this bay was reported swapped" and "how often this block was thrown away" are
    different numbers, and a reader who is trying to explain a zone that closes no
    window needs the second.

    The rate above which this should become a health finding rather than a
    diagnostic is *not* decided here, and after item 124 the urgency is gone: the
    reset now spends the settling budget, so the pathology item 109 found — a bay
    resetting on nearly every tick, taking its zone's air accumulator with it —
    cannot recur at any rate. What the record adds is that it would be visible
    afterwards. The health rule is in the proposed items.

    **Measured**: nothing moves. Re-run over item 102's 16 h and 36 h scenarios on
    `rich` seeds 2/3/4, both arms, every converged zone, every mean PWM and every
    worst margin reproduces the pre-change numbers to the digit — because since
    item 109 no bay of that example is ever reported swapped, so there is no event
    to record. That is the honest measurement of a diagnostic: on a healthy
    enclosure it costs nothing and says nothing.
124. **Done** (2026-09-18): they agree now, on both halves, and a test fails if
    they stop.

    **The definition.** Since item 109 the two rules read the *same per-member
    innovations at the same thresholds*; what they did not share was the
    consequence. Two ways they could part, and both are closed:

    - on a bay with **one** proximal member the two statistics are arithmetically
      the same two numbers (the mean of one thing is that thing, and the variance
      of that mean is its own), so they cannot disagree. That is the case where
      the agreement is provable rather than a judgement call, which is why it is
      the one pinned — and it is pinned on the **arithmetic**: both halves now
      call one threshold function, and the test watches its arguments and asserts
      that on a one-member bay the two calls of a tick carry the same `nu`, the
      same variance and the same verdict. Asserting that the settling window
      follows the verdict could not have failed on divergence, since the
      production code draws that consequence *from* the verdict;
    - on a bay with a **redundant pair** the mean can cross while no single member
      does. That used to throw the thermal block away while the estimator went on
      calling the bay's drive estimate confident — a model scored against a
      reading nothing had widened. A bay-level step now *implies* the per-sensor
      consequences: the drive variance is widened by the very statistic that said
      so (through the bay's own map), the settling window opens, the tick's SMART
      is held back, a correlation pair is dropped, and the placement row of item
      101 is not absorbed on a tick the drive itself moved.

    **The rate limit.** `swapped` keeps its per-tick meaning and every consequence
    the estimator draws from it. What `mpc.step` hands to `thermal.update` is now
    `swap_reset`: the same verdict at **one event per `estimator.bay_settle_s`**
    while the bay has `estimator.bay_settle_max_s` of settling budget left — the
    same budget, with the same clean-run recovery, that `jumped` spends for the
    `sigma` trust exemption. A swap moves the readings for as long as the filter
    takes to follow them; that is one event and one reset, which is what item 12
    measured anyway ("one swap produced exactly one reset"). `bay_settle_max_s: 0`
    now grants neither exemption nor reset — with it, `mpc.model_reset_on_swap`
    acts on occupancy crossings alone — and the config comment, the key table row
    and the `EstimatorSpec` docstring all say so, since the key is named for the
    trust exemption and this is a second, safety-relevant meaning for it.

    **Exhausting it keeps the fit, and that is the louder of the two.** Resetting
    without a limit is what left z0 at the prior for a whole run in item 109; a
    stale fit, by contrast, is *judged* — the DAS MPC's prediction-error gate
    scores it and falls back to `solver_pi` when it is wrong. For that to be the
    answer and not a hope, a bay whose verdict the budget refused has the model
    gate's `uncertain` exemption **withheld** while the refused verdict stands:
    the bay-level widening the verdict itself applies is exactly what pushes
    `σ² − σ_cal²` over `bay_uncertain_var_c2`, so without this the bay would be
    neither reset nor scored and the fallback would rest on a gate that never
    looked. With the mark withheld the stale fit really is scored. The record
    (item 123) counts the refusals in `swap_held`, at the cadence an accepted
    event would have had — one per `bay_settle_s`, not one per tick the verdict
    stands, so the number is a count of events and its rate does not depend on
    `dt` — and "this bay stopped resetting" is a number rather than a silence.

    **The occupancy half is deliberately not rate-limited.** A crossing of the
    `empty` boundary is not a statistic but a declared or debounced fact with its
    own rate limit (item 19's `empty_confirm_s`, three consecutive ticks, the
    owner's flag). Refusing one would model an arriving drive with the
    coefficients of the drive that left, which is the whole of item 12: two
    crossings in opposite directions inside one settling window are two events,
    and a test asserts every one of them is honoured.

    **Measured**: nothing moves on the healthy example — item 102's 16 h and 36 h
    scenarios reproduce to the digit on all three `rich` seeds, since no bay there
    is ever reported swapped. The rate limit is a bound on a pathology, not a
    change to the ordinary path.
125. **Done** (2026-09-18), and the answer to "does the fused layout learn it
    too" is **no, it cannot, and the code and the docs now say so** rather than
    leaving the two layouts looking alike.

    **What follows for the fused layout.** Two placements on one bay disagree by
    `ds·(T_d − T_a) + db`. The per-sensor layout (item 101) fits both halves: `ds`
    has no process noise, because a placement's geometry does not drift, and `db`
    is a random walk, so a sensor that moves is followed. The fused layout (item
    67) has **one** state for the pair, and that state is the whole disagreement
    *at the current rise* — it absorbs `ds·rise` as drift and has no slope to
    separate it from. So the fused layout cannot learn the *shape* of a placement
    difference, and there is nothing to add that would let it: giving the pair a
    slope **is** the per-sensor layout, at the same state size and a cheaper
    measurement (item 101's own finding). The estimator's docstring said "two
    sensors sit differently and carry different offsets", which reads as though
    the offset were the whole story; it now states the limitation where the layout
    is described, and the config comment for `proximal_slope_spread` and
    `proximal_offset_c` does too.

    **The consequence that ships** is the signal both layouts *can* carry, which
    item 125 asked for: every bay with a redundant pair publishes `placement` per
    further member — `gap_c` (the learned disagreement now), `bound_c` (the box the
    config's own priors allow it **at this tick's drive-to-air rise**,
    `proximal_gap_sigmas · sqrt((slope_spread·rise)² + offset_c²)`, so it widens
    with the load exactly as the disagreement does), `over`, `evidence` (rows the
    per-sensor map has taken), and — the point — `layout` with `learns`
    (`offset` | `offset+slope`). One new key,
    `estimator.proximal_gap_sigmas` (default 3.0, `>= 0`).

    What a reader may conclude from `over` therefore depends on the layout, and
    the verdict says which: with a node per sensor the slope and the offset are
    separated, so a gap outside the box is a placement the config does not allow —
    a sensor coming loose, fouling or ageing; with one fused node the gap may
    equally be a load that layout cannot resolve. Publishing one flag for both and
    letting a health rule read it would have been the third way to imply they know
    the same thing, which is what the item is against.

    Not done: nothing *acts* on `over` — it is a diagnostic, not a health finding
    and not a gate. On the fused layout it cannot be one without the false
    positives above, and on the per-sensor layout that turn is item 99's decision,
    since `proximal_slope_spread > 0` moves the four DAS goldens. See the proposed
    items.
126. **Done** (2026-09-18): the relative step-budget gate loses its
    margin on a shared machine (item 109).
    `test_das_mpc_step_p99_within_the_relative_budget` divides the DAS
    MPC's p99 by the legacy MPC's, measured back to back in one process,
    and asks for at most 12x; the method notes were written against "a
    runner hiccup during one repeat", which the warm-up and the 75th
    percentile do handle. Sustained contention is a different failure: it
    loads the DAS side harder than the legacy one. Measured independently
    on items 109 and 110/111/112's own PRs (and again during this merge):
    the gate fails intermittently under an 8-way parallel
    `ci_pytest_shards.py` run and passes alone every time, while the
    ratio's own distribution does not move (10.4–10.6 on one measurement,
    10.57–10.62 on another). A fourth independent measurement (item 95's
    PR): the same test flaked once under a host load average over 160 on
    32 cores from concurrent workflow runs, and passed reliably both
    alone and in a single-process (`--shards 1`) full run on the same
    host — a single-process run sidesteps the contention this item is
    about, at the cost of the wall-clock time sharding buys.
    **Fixed by measuring what the gate means**: both sides now time with
    `time.process_time()` (CPU time) instead of `time.perf_counter()`
    (wall clock). Wall clock counts a repeat's *entire* elapsed span,
    including any stretch this process spent preempted while a runner
    running four branches' suites at once served someone else's turn; CPU
    time only counts cycles this process actually used, so a preemption
    costs it nothing on either side. Demonstrated on the machine this item
    was fixed on, itself running three other branches' suites throughout:
    five independent single-repeat samples read `perf_counter` ratios of
    3.1x–39.3x — both under and over the 12x budget, on the very same
    code, back to back — against `process_time` ratios of 9.6x–11.7x, in
    the same band the item's own prior, less-contended measurements
    report (10.4x–10.6x, 10.57x–10.62x). The gate itself, run repeatedly
    on that same busy machine after the fix, passed every time. The
    absolute Pi-only gate (`test_das_mpc_step_p99_within_budget_ms_on_the_pi`)
    keeps wall clock: there the question really is elapsed time against
    `dt` on hardware that runs this suite alone, not a ratio between two
    solvers on a shared runner, so `perf_counter` measures exactly what
    that gate means. `_timed`/`das_mpc_times`/`legacy_mpc_times` take an
    optional `clock` (default `time.perf_counter`, so the absolute gate is
    unaffected) rather than hardcoding one, since the two gates now
    legitimately want different clocks for the same helper
    (`tests/test_bench_budget.py`, both docstrings state the choice and
    why).
127. **Done** (2026-09-18): the recorder keeps `rail_reported` next to
    `power_reported` on every `fans.<channel>` entry (`recorder.py`,
    `_fan_readings`), and `rail_known`/`power_known` read a fan entry's
    flag as `False` — unknown, not measured — whether the flag is
    explicitly `false` or the key is missing entirely (a recording made
    before this item). `tools/fit_fans.py` carries the same distinction
    into its output: each model's `channels_with_rail_reported` names a
    channel only where some record's `fans.<ch>.rail_reported` was `true`,
    read through `rail_known`, so a reader of the written-out
    `fan_curves.json` can tell which channels' `voltage_v` in the source
    recordings meant something; it plays no part in the curve fit itself
    (only `pwm`/`rpm` do).
129. **Done** (2026-09-18): the page's Controllers section gets one line
    per controller that actually has an aquabus, `<label> aquabus`, from
    `device_health.devices[].aquabus.state` ("a device answers" / "lost
    `<absent_s>` s ago" / "empty" / "never connected" / "not read yet"); a
    controller kind with no aquabus at all (a `quadro`) has no such row —
    its `state` reads `"unknown"` forever, which is not the same thing as
    an aquaero that has simply not been read yet, and must not print like
    it (`publishers/static/index.html`, `renderDevices`). Home Assistant
    gets `aquabus_problem` (`device_class: problem`, diagnostic, both
    modes), keyed on the daemon's own `device_health.aquabus_lost` scalar
    (`aqua_bridge.health._aquabus_lost`): true only where some controller's
    aquabus is both `lost` (a device that answered has now been missing for
    `bus_absent_s`) *and* `bound` (something of that controller's own
    config — an aquabus output, tachometer or temperature slot — actually
    reads the bus) — the same gate `device_health`'s own `problems` list
    already applies before it reports a lost bus, since aquabus presence is
    read from the status report alone and can go `lost` on a controller
    with nothing of its own bound behind it. A bus with `state:
    "never_seen"` (a healthy aquaero with an empty bus) or `"empty"` never
    turns it on either. Its attributes are `{"devices": [...], "aquabus":
    [...]}` — each controller's `label` and `aquabus` block (now including
    `bound`), index for index — a JSON object, because Home Assistant's
    MQTT attributes mixin discards anything else (`publishers/mqtt_ha.py`,
    `health.py`).
133. **Done** (2026-09-18): `tools/bench_model_store.py`'s warm-up loop
    never got `calibration` or `fan_curves` populated against the shipped
    `config.example-das.yaml` (`fan_curve_online: false`, and the DAS
    truth-plant closed loop doesn't accumulate an accepted SMART
    calibration in a modest number of ticks), so its dev-machine size
    sanity check (1424 bytes, item 48) was close to the store's floor
    rather than a representative steady-state size.
    **Diagnosed further than "a modest number of ticks", to two separate
    causes.** First, `calibration`: none of the shipped examples declare
    a bay's serial (§3 "no SES backplane"), and the truth simulator reads
    `serial` straight off the topology dict
    (`aqua_bridge.sim.das.DriveSpec`) — with none, `_schedule_smart` never
    schedules a sample for that bay at all, so the closed loop carried
    **zero** SMART traffic however long it ran, confirmed by running it
    six simulated hours and reading `estimator.smart` back empty.
    `fan_curve_online: false` was the second, obvious cause, but forcing
    it on alone was not enough: a closed loop that is always actively
    regulating practically never holds a duty within
    `control.fancurve.SETTLE_TOL` (1e-6) for `fan_curve_settle_s`
    straight — sensor noise alone moves the MPC/PI output by more than
    that on almost every tick. Measured directly: six simulated hours of
    ordinary closed-loop operation left the fit's bin accumulator with
    exactly **one** usable bin (the `pwm_max` clamp, the one duty that
    ever holds bit-for-bit), never the four bins across a 0.25 span a fit
    needs — not a "modest number of ticks" problem, a structural one
    about what a continuously-regulating closed loop's own duty does.
    **Fix, in `tools/bench_step.py` and `tools/bench_model_store.py`**:
    `das_plant(..., inject_smart_serials=True)` gives the *truth* plant
    (never `cfg.topology`, which stays exactly as loaded) a synthetic
    serial per occupied bay with none, so the estimator has SMART traffic
    to correlate and calibrate against — the same way a real enclosure's
    unlabelled drives do (off by default, so `tools/bench_step.py`'s own
    solver-timing benchmark is unaffected). The warm-up runs against
    `dataclasses.replace(cfg, fan_curve_online=True)`, and appends a
    short, separate dwell scan after the closed loop
    (`_fan_curve_dwell_scan`): holds every channel at six fixed duties
    spanning `[pwm_min, pwm_max]`, each long enough to settle and clear a
    whole `fan_curve_refit_s` window (2x the interval, since the closed
    loop above leaves that clock at an unknown phase), feeding the *same*
    `control.fancurve.update` the daemon's own `step()` calls — a
    commissioning-style sweep through the real online-fit code, not a
    fabricated curve. `--warm-ticks` defaults to 4320 (6 h simulated,
    ~9-15 s wall): measured on this change, 11 of 15 bays reach an
    accepted calibration and the one fan model reaches an accepted curve,
    and the document grows from the 1424-byte floor to 4774 bytes — over
    3x, and no longer close to empty. The report gains three fields so a
    reader is never left guessing: `fan_curve_online_forced` (true
    whenever `--config` itself leaves the flag off), `calibrated_bays`
    and `fan_curve_fit_accepted` (which models, `[]` if none). Tested at
    a smaller, faster scale too (`--warm-ticks 900`, ~2 s wall): at least
    one bay calibrated and the one model's curve accepted, asserted in
    `tests/test_bench_model_store.py::test_warm_up_populates_calibration_and_fan_curves`,
    so a warm-up that regressed back to the item's original, unpopulated
    state would fail a test, not merely read smaller in a report nobody
    is diffing.
    Not done: the aquaero board itself was never involved (this whole
    item is a dev-machine tool), and what a *sensor-only* bus device (no
    fan outputs, unrelated to this item's `fan_curve_online`) does to
    SMART or fan-curve accumulation is not this item's question.
134. **Done** (2026-09-18): board watchdogs, and the network proved to be
    outside the cooling path, argued from the owner's board rather than
    copied (§2 *Watchdog layering, and the network outside the cooling
    path*, §9 *Board hardening*, §10 step 6b). `deploy/install-board-
    watchdogs.sh` (idempotent, `--check`, every value a documented
    variable at the top) installs the SoC watchdog
    (`RuntimeWatchdogSec=60s`, `RebootWatchdogSec=120s`), journald caps
    for the now-persistent journal (`SystemMaxUse=200M`,
    `SystemMaxFileSize=16M`, `MaxRetentionSec=30day`,
    `SyncIntervalSec=5m` — the default would have been 10 % of the card),
    Wi-Fi power save off through a NetworkManager drop-in rather than the
    SSID-carrying connection profile, and `deploy/aqua-net-recover.
    {sh,service,timer}`, which may only re-associate the interface: no
    reboot, no `systemctl`, no controller, and it gives up after three
    fruitless re-associations so a router that is off costs one journal
    line and then silence rather than a bounce loop. `deploy/aqua-
    bridge.service` lost `Wants=`/`After=network-online.target` (with the
    router off, `NetworkManager-wait-online` spent its full 30 s default
    before the daemon's first write to the controllers — a whole
    aquaero software-sensor window, spent on a router) and gained
    `StartLimitIntervalSec=0` (systemd's default rate limit parks the
    unit in `failed`, the one state where nothing writes the controllers
    or the heartbeat again) and `RestartSec=5`; `WatchdogSignal` stays
    `SIGABRT`, with the fan behaviour of a kill written down (about 25 s
    at 100 % from the alarm, then the saved profile's preset until the
    duty-mismatch rewrite). Values argued from measurement, not copied:
    `step()` p50 50 ms / p99 82 ms / max 87 ms over 60 ticks against the
    DAS plant, a healthy boot reaching the first controller write at
    27.7 s, and the `check_watchdog` bound of 18.5 s (31.5 s with the
    Quadro on its own USB port) — so SoC 60 s > service 45 s > worst-case
    tick 18.5 s > healthy tick, under the aquaero's own 30 s
    software-sensor timeout. Every publisher path into the tick was
    traced and pinned: `on_tick` runs after `sink.apply()` and after
    `notifier.watchdog()`, so a hung publisher (MQTT, HTTPS, Home
    Assistant discovery) can delay only the next tick, never this tick's
    command or its ping; SMART absence widens a zone's σ and so can only
    raise duty, never lower it. `tests/test_deploy.py` (17 new cases) and
    one in `tests/test_loop.py` pin the unit never waiting on the
    network, the rate-limit/margin/knob values, and the recovery
    script's re-association, give-up and silence behaviour against
    `nmcli`/`ip`/`ping` stubs.
139. **Done** (2026-09-18): free space and a read-only filesystem as a
    health signal, the shape item 103 established for the board's own
    temperature and throttling extended to "the disk nobody watches"
    (§3). `hostinfo` had always collected `disk_used_pct`/`disk_free_gb`
    and both were always published — an MQTT sensor, a row on the
    page — but nothing judged them. Two new `host_health` rules, both
    published in the health payload, MQTT's `host_problem` sensor and
    its attributes, and the page, and both kept out of
    `PlantObservation` and the solver's `diagnostics`: a free-space hint
    (`disk_free_min_gb`, default 2 GB; `disk_free_fault_s`, default
    60 s) and a read-only-filesystem fault, detected from the kernel's
    own `/proc/mounts` (`hostinfo.read_mount_ro()`) rather than a write
    probe, reported the tick it is seen like "throttling now" since the
    kernel has already remounted by the time this reads it. Both judge
    a new `disk_path` config key (default `"/"`) rather than a
    hardcoded root, since `record_path` and `--model-store` are not
    pinned to `/` and an operator who moves either without moving
    `disk_path` would have these rules watch a filesystem nobody writes
    to. `disk_free_min_gb`'s default is argued from what actually
    writes to the card, not a round number: the recorder's own worst
    case (120 MB, item 79) plus the model store's measured floor (item
    48) is under 125 MB, unconditionally; `deploy/install-board-
    watchdogs.sh`'s `JOURNAL_MAX_USE` (item 134) is margin on top of
    that, not a term the default depends on, since that script assumes
    the journal is already persistent rather than making it so. The
    free-space rule is a **hint**, not a fault, for the same reason the
    divergence rule (item 103) is: a filling card can sit below the
    threshold for days, and latching the daemon-wide `problems` list
    for that long would mask an unrelated device fault. An unreadable
    `statvfs` or mount table degrades both rules to unknown, never to a
    fault and never to "fine". `deploy/install-pi.sh` does **not**
    install a second journal cap: an earlier version of this change did
    (its own drop-in, `SystemMaxUse` only), and it was withdrawn in
    review in favor of the board-hardening script's, which already
    covers the same setting plus `SystemMaxFileSize`, `MaxRetentionSec`
    and `SyncIntervalSec` from one set of knobs — a board running both
    scripts must not end up with two drop-ins governing one journal
    from two different defaults.
140. **The DAS MPC is the louder of the two solvers on the shipped example**
    (item 119). Measured while item 119's cost was being separated: over 36 h
    on `rich` seeds 2/3/4 the enclosure's mean PWM is 0.3656 / 0.2904 / 0.3566
    with the MPC acting for 11146 / 6754 / 20074 of 25920 ticks, against
    0.3165 / 0.2553 / 0.2976 with it acting for 370 / 0 / 5266 — and the worst
    true margin is identical to the digit (4.133 / 4.510 / 4.026 degC) either
    way. So on this simulated enclosure the model-based solver buys no margin
    for about +0.05 mean PWM. Worth deciding whether that is the example's own
    weights (`weight_dpwm`, `rho_soft`), the prediction horizon, or the model,
    because it is the objective of the whole milestone that the MPC be
    *quieter* at the same margin.
141. **A health finding on the reset rate** (items 123, 124). Item 123 left it
    open and item 124 made it non-urgent, but the numbers are now there to
    write a rule on: a bay whose `swap_held` is climbing is one the estimator
    keeps calling swapped and the budget keeps refusing, which is a sensor or
    a placement problem and not a drive. The counter is now one per
    `bay_settle_s` of a standing verdict, so a threshold set against it does
    not depend on `dt`. Worth deciding a threshold (per hour? per
    `bay_settle_max_s` of clean time?) and whether it belongs in `health.py`
    beside the fan and board rules or in the estimator's own diagnostics.
142. **Acting on item 125's `placement` verdict** (item 125). The signal
    ships as a diagnostic. On the per-sensor layout an `over` is a placement
    the config does not allow, and a sensor coming loose is exactly the
    failure that biases a whole bay's estimate toward safety or away from it
    depending on the sign. Turning it into a gate input (drop the member,
    keep the anchor) or a health finding needs the per-sensor layout on,
    which is item 99's golden decision.
143. **The replay path never reconstructs `reset_bays`** (item 123).
    `recorder.thermal_inputs` builds `occupancy`, `classes` and `maps` from a
    record's `bays` block but not the swap verdict, so `tools/replay.py` and
    `tools/fit_model.py` re-learn a bay the live run reset — the fit they
    produce is not the fit the daemon had. The record now carries
    `swap_reset` per bay (item 123), so the gap is one line to close; worth
    deciding whether a replay *should* reproduce the resets or deliberately
    fit across them, since the second is what a hold-out pass wants.
144. **A monkeypatched stub that no longer matches its function is
    invisible.** `test_model_fallback_sim` replaced `thermal.current_model`
    with a two-positional-arg stub; adding a keyword argument turned every
    call into a `TypeError` that `solver_das` catches as "a thermal memory
    that cannot be read", and the whole suite went on passing while testing
    the fallback path instead of the fallback *decision*. Only the
    assertions about *which* tick switched caught it. Worth deciding whether
    the repo wants a small conftest helper that monkeypatches through
    `inspect.signature`, so a stub that has drifted fails loudly instead of
    silently widening an `except Exception`.
145. **`bay_settle_max_s` is one key with two meanings** (items 123, 124). It
    bounds the `sigma` trust exemption and, since item 124, the statistical
    half of `model_reset_on_swap` — so `bay_settle_max_s: 0`, a documented and
    supported way to switch the trust exemption off, now also stops a drive
    replaced in place from ever having its coefficients thrown away. The
    review pass documented the coupling in all three places an owner would
    look. Worth deciding whether the two budgets should be one key at all, or
    whether the reset wants its own, since a key named for one rule quietly
    gating another is what makes that setting a trap.
146. **A genuine sensor-only aquabus device has never been captured**
    (item 130). A Farbwerk 360, or any bus slave with zero fan outputs, has
    never actually been on the owner's bus; item 130's flow-slot witness is
    inferred to generalise from the Quadro's own flow header, not measured
    against a second device kind. Worth a bench session if one becomes
    available.
147. **`bench_model_store.py`'s dwell scan and SMART-serial injection are
    dev-machine-only conveniences** (item 133). They have no bearing on and
    are not exercised by anything that runs on the Pi. Worth a one-line note
    in PROJECT.md section 4 pointing future readers at item 133 if the bench
    tool's warm-up ever needs revisiting again.

### 8.3 Open — needs the DAS hardware

31. USB host: `dtoverlay=dwc2,dr_mode=host` (`deploy/host-usb.sh`), powered
    hub.
32. **Done** (2026-09-15): yes, the Quadro's PWM is writable through the
    aquaero over aquabus (§2 "Quadro on aquabus"). The Quadro stays on
    aquabus; the adapter commands it as the aquaero's outputs 5–8 since
    item 85.
33. Spike: does the XT6 revert after the Pi stops writing? If not, software
    sensor plus firmware timeout (§2); then decide whether `release()` runs
    at exit. Since item 86 written duties are not saved: a power cycle of
    the controllers brings back their saved configuration (item 84). But
    while they stay powered, a Pi that stops writing leaves the fans at the
    last written duty, possibly low, until the daemon runs again — unless
    the software-sensor heartbeat of item 84 is configured, which is the
    answer: about 30 s after the last write the aquaero's alarm selects the
    safe profile and every output runs at 100 %, verified on the hardware
    (§2 "Software-sensor heartbeat and profiles"). It covers the daemon or
    the Pi stopping **and** a daemon that runs but can no longer write (the
    heartbeat goes out only behind an `apply()` that reached the device, §3
    Track B). What it does not cover is a write the kernel accepts and the
    device never sees: `write_report` queues an output report rather than
    waiting for an acknowledgement, so `heartbeat_ok` means "accepted", and
    reading the sensor back is the check item 93 adds. What is left of this
    item is the owner's decision whether `release()` runs at exit, and
    running the daemon's own heartbeat on the hardware (item 93).
34. **Done** (2026-09-16): a Quadro temperature input carries a reading in
    the *aquaero's* status report — the Quadro's sensors 1–4 appear in the
    aquaero's aquabus temperature slots `bus1..bus4` (§2 "Quadro on
    aquabus"); with one thermistor on sensor 2, only `bus2` read, 23.28 °C
    on the run behind the 2026-09-16 topology decision (§2 "Supported
    topology", §8.1), matching the earlier 24.04 °C capture. They reach the
    daemon through the aquaero, not a second hidraw path to the Quadro
    itself, and the config binds them as `busN` (item 85). Which of the
    remaining three inputs will be used is decided when the sensors are
    wired. Still open: whether the Quadro's 16 slots at `0x3C`, named
    software sensors `soft1..16` after the aquaero's, are software sensors.
35. Confirm the HID report layout of `hw/aquacomputer.py` on the real
    devices in their final wiring (the Quadro on aquabus or on its own
    USB port, every fan and sensor connected): the status report fields
    of every input and output, the control-report duty fields of every
    channel, and that the udev rule makes `/dev/hidrawN` readable and
    writable for the service user. Answered for the wiring that ships
    (§2 "aquabus fields checked against the live devices", 2026-09-17, on
    top of "USB spike results" and "hidraw check"): in the final topology --
    the aquaero on USB, the Quadro a slave on its aquabus -- every status
    report group, every fan block 1–8, the flow slots, the identity fields,
    the active-profile byte and every control-report field of all eight
    channels were read and compared against the table in §3 Track B, and the
    udev rule is confirmed (`/dev/hidraw*` `root:plugdev` 0660, the service
    unit in `plugdev`, the heartbeat service writing report `0x07` as that
    user every 5 s). What stays **unobservable without a write the owner must
    authorise**: the preset id `0x5C + k` for k = 1–5 and 7 (only channel 0
    and, from the 2026-09-15 experiment, channel 6 are seen), that writing
    min / max power takes effect, and a populated virtual sensor. What stays
    unobservable without more hardware: the aquabus sensor→slot map for the
    Quadro's sensors 1, 3 and 4 (no thermistors on them), and a sub-zero
    temperature (decoded signed by design, not observed).
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

    **Tool added (2026-09-17); board run below settles the size question,
    not the write-latency one (2026-09-18).**
    `tools/bench_model_store.py` builds one realistic snapshot -- a DAS
    closed loop of `--warm-ticks` steps against `config.example-das.yaml`
    (`--sim-preset rich` by default), so the estimator's calibration, the
    thermal model and whatever else a running daemon actually accumulates
    are in `solver_memory` before anything is timed -- then serialises it
    exactly as `ModelPersister.save` does (`modelstore.build_document`,
    then `json.dumps(doc, allow_nan=False, separators=(",", ":"))`) and
    times `modelstore.write_atomic` (open, write, `fsync`, `os.replace`,
    `fsync` of the directory -- the same call the daemon makes) against
    `--path` for `--repeats` repeats, reporting the size in bytes, min /
    mean / median / p99 / max write time, the spread over the repeats, and
    that p99 as a fraction of `dt` and of `mpc.model_store_interval_s`.
    Refuses anything but a path under a scratch directory
    (`tempfile.gettempdir()` or `/tmp`) before running anything, and
    deletes its own scratch file afterwards unless `--keep` is given
    (`tests/test_bench_model_store.py`).

    Dev-machine sanity run only (not the Pi's SD card, not what this item
    asks for): `--warm-ticks 600 --repeats 20` against `config.example-das.yaml`
    on the development machine's filesystem -- `size_bytes` **1424**, `min_ms`
    2.84, `median_ms` 2.85, `mean_ms` 2.96, `p99_ms` 4.46, `max_ms` 4.46,
    `spread_ms` 1.62 over the 20 writes. The 1424-byte size is this run's
    document, not a representative one: `config.example-das.yaml` ships
    `fan_curve_online: false`, so `fan_curves` stays empty, and the DAS
    truth-plant closed loop here never accumulated a SMART calibration
    a real drive's agent would, so `calibration` and `bays` are empty too;
    the tool passes `ident_settle=None` (no identification experiment
    runs in this warm-up loop), so that section is unconditionally empty
    on every run of this tool as well -- what's actually in the 1424
    bytes is close to the store's floor (`fingerprint`, plus four empty
    sections: `fan_curves`, `calibration`, `bays`, `ident_settle`), not
    its steady state on a populated enclosure. The size and the write
    time both need the Pi run to mean anything for this item: SSD-class
    dev-machine storage says nothing about a microSD card's write and fsync latency,
    and only a board running against the real topology (drives reporting
    SMART, calibrations accepted, fan curves fit if enabled) reaches the
    store's real size.

    **Board run (2026-09-18).** The earlier no-route failure was a
    network-reachability gap in a different work environment, not this
    one; `ssh` reaches the board this time and `/opt/aqua-bridge` there
    is a git-free copy of the same revision this ran from (`sha256sum`
    of `tools/bench_model_store.py`, `tools/bench_step.py` and this file
    all match). Writing only under `/tmp` on the board, never over the
    daemon's own store: `PYTHONPATH=/opt/aqua-bridge/src .venv/bin/python
    tools/bench_model_store.py --warm-ticks 1200 --sim-preset rich
    --repeats 30 --path /tmp/model-store-bench.json` ran clean
    (`/sys/class/thermal/thermal_zone0/temp` 46.16 → 50.46 °C across this
    and item 95's board run together, `vcgencmd get_throttled` `0x0`
    throughout -- not a throttled run) and reports `size_bytes` **1424**
    -- the same floor as the dev-machine run above, even at 1200 ticks
    and `--sim-preset rich`: item 133's caveat (the warm-up loop never
    gets `fan_curves` or `calibration` populated) is now confirmed on the
    board itself, not only argued from the shipped config, and no
    sibling branch has landed that changes it -- and `min_ms` 0.243,
    `median_ms` 0.250, `mean_ms` 0.273, `p99_ms` **0.710**, `max_ms`
    0.710, `spread_ms` 0.467 over the 30 writes. Against `dt = 5 s` and
    `mpc.model_store_interval_s = 600 s` that p99 gives
    `p99_fraction_of_dt` 0.000142 (about 0.014 % of the tick) and
    `p99_fraction_of_model_store_interval` 0.0000012 -- both computed from
    the RAM write-and-`fsync` latency measured here, not the microSD
    card's, so neither fraction is the item's real answer yet either.

    Those write numbers answer the size question and nothing else:
    `findmnt /tmp` on this board shows `/tmp` is `tmpfs` -- RAM-backed,
    and a separate mount from the SD card the daemon's own store and
    `/opt` both live on (`/dev/mmcblk0p2`, `ext4`, confirmed with
    `findmnt /opt`) -- and `tempfile.gettempdir()` there is `/tmp`
    itself, so the tool's own scratch-root check (module docstring,
    *Safety*) accepts exactly the mount that defeats this item's
    purpose. A sub-millisecond `p99_ms` an order of magnitude faster
    than the dev machine's own SSD-class 4.46 ms is itself the tell:
    this measured RAM write-and-`fsync` latency, not a microSD card's,
    and nothing in this run touched the card at all. The run this item
    asked for was scoped to write only under `/tmp` on the board -- never
    elsewhere, never as root, never near the daemon's own store -- so a
    disk-backed path was not attempted even though one exists on the
    same filesystem as `/opt` (`findmnt /var/tmp` resolves to the root
    `ext4`, checked read-only, nothing written there). The size half of
    this item is now settled; the write-and-`fsync`-latency half is not,
    and still needs either a scratch path the owner points at the SD
    card explicitly, or the tool itself flagging a resolved path that
    turns out to be `tmpfs` so a future run cannot repeat this same
    silent miss (PROPOSED ITEMS).
49. Digole: protocol, pages (Overview, Drives, Zones/Fans, Model, Host),
    touch, hit-test.
75. **The kick is done; the measured duties behind it are not** (2026-09-18).
    A fan below its stall duty stops and starts again only at a higher duty
    (aquaero test fan: stops at 13 %, starts at 25 %), and a fan can speed up in
    a low-duty band (Quadro test fan: up to 860 rpm at 5–8 %). `mpc.pwm_min` is
    one global value.

    **Shipped** (`control/spinup.py`, §3 "Spin-up kick"): the start kick this
    item asked for, as a supervised sequence rather than a boost — detect a
    channel commanded at or above its stall duty whose tachometer reads at or
    below `spin_up.min_rpm` for `confirm_s` of live readings, kick it to its own
    `kick_duty` for `kick_s`, verify by the tachometer for `verify_s`, and after
    `max_attempts` call it a failed fan with a `problems` line and a floor under
    its siblings so the zone cannot lose cooling. Every commanded channel is
    under the rule from the first tick, which is the restart case the owner hit
    on 2026-09-17. The kick is a floor applied in `Supervisor.compose` through
    the same `d_pwm_max` and clamp as any command, so it can reduce cooling
    nowhere; `mpc.step` is untouched and a legacy config gets no kick at all.
    An output with no fan (`spin_up.channels.<ch>.fan: false`), a fan with no
    tachometer (`tachometer: false`, or no `rpm:` in the `aquacomputer:`
    binding) and a channel with no fitted curve are never kicked and never
    alarmed, and each says so in its verdict.

    **What the review changed** (same item, before the merge): the declaration
    of a failed fan is now a fact of its own rather than a state string, so the
    alarm and the sibling floor survive a dropped reading and the ticks of a
    `retry_s` retry, and are retracted only after `clear_s` of readings above
    `min_rpm` — one windmilled sample used to clear both. The hold a failure
    records is capped at `failed_channel_floor_max` and is never carried by a
    channel whose own fan is declared dead, so two failures across a hot spell
    cannot leave the enclosure pinned at 90 % for ever. `validate_spin_up`
    refuses a `kick_s` that cannot cover the `d_pwm_max` ramp up to the kick
    duty, which used to be named in the message and not enforced — a fan would
    have been condemned for a kick it never got. A kick that raises a channel a
    running identification experiment drives now aborts that experiment
    (`spin_up:<channel>`) instead of silently making its excitation something
    other than what it planned. And `build_health_monitor` asks whether the rule
    can cover any channel at all rather than whether the section is enabled, so
    a legacy deployment that turned both health sections off does not get an
    observer built behind it to publish "off" verdicts.

    **The duty-to-rpm mapping is the output's, not the fan model's** — measured
    2026-09-18 on the owner's hardware: the same fan model reads 174 rpm on an
    aquaero output and 255 rpm on an output behind the aquabus device at the
    same nominal 20 % (244 against 311 at 25 %). So `kick_duty` is per channel,
    with a documented fallback; `config.example-das.yaml` ships 0.5 for the
    aquaero's own outputs (the weaker mapping, and the ones that failed to
    start) and 0.35 for the aquabus ones.

    **Still open.** Those are duties chosen with margin over the measured start
    behaviour, not measured start duties: two points per output fit nothing, and
    `spin_up.stall_duty` is still one number for every channel with a per-channel
    override nothing has measured a value for. What closes it is
    `tools/fit_fans.py` finding each output's stall duty, start duty and unstable
    band from a recording, after which `stall_duty` and `kick_duty` come from
    measurements per output instead of from margin. Open question for the Quadro:
    whether its jump to 860 rpm at 5–8 % is a firmware start boost; if so, it may
    restart a stalled fan without daemon code, and the kick would be a no-op
    there. (For the aquaero that guess is withdrawn: the outputs that stayed
    at 100 % were in DC voltage mode, §2 "hidraw check".)
76. What the aquaero channels hold after exit: each commanded channel
    keeps its manual preset (the stop write leaves `fallback_pwm`).
    `AquacomputerAdapter.release()` now restores the captured firmware
    assignment over HID (the aquaero's preset, control source and power
    limits, and the Quadro's duty, as the first control report read after
    the daemon started saw them), but it is not called at exit. Decide
    together with item 33 whether that stays or `release()` runs at exit.
    Note that after a daemon restart the first read sees the previous
    run's presets, not the firmware controllers, so a restore at exit also
    needs the capture kept across restarts (for example in the state
    directory). Since item 86 neither the stop write nor `release()` is
    saved: after a power cycle a controller runs its saved configuration
    (item 84), so what the channels hold after exit matters only until then.
77. **Done** (2026-09-15): control-report writes are stored in
    non-volatile memory on both controllers. The owner removed 12 V and USB
    from both for 30 s. The Quadro's power-cycle count went from 12 to 13,
    and the aquaero status report's u32 at `0x11` went from 39478 to 32 (it
    restarts at power-on; likely seconds since power-on, not otherwise
    decoded). Both control reports read back byte-identical to the copies
    taken just before, and the fans came back at the written duties (aquaero
    outputs 1, 2, 4 at 25 %, 14.12 %, 25 % on their presets, output 4 still in
    PWM mode; Quadro output 3 at 9.02 %). Those writes were a SET followed by
    the save report (report 6 on the aquaero); a SET without it does not
    persist (item 84). The adapter sent it after every SET, so every write
    was saved and the write limiting of §3 Track B was required, until item
    86 stopped saving: writes are live, write limiting is off by default,
    and after a power loss the controllers come back with their saved
    configuration, not the last written duty (items 33, 84). The daemon side
    of item 84 is done; what is left before item 42 is the hardware run of
    item 93 and the commissioning tool of item 88.
78. Send the driver fix in `deploy/dkms/aquacomputer_d5next/` upstream
    (linux-hwmon), then drop the patch once a Raspberry Pi OS kernel
    carries it. Only relevant if the DKMS driver path is revived: the
    daemon uses hidraw (§8.1, 2026-09-15).
80. **Done** (2026-09-17, on the Zero 2 W): timed
    `AquacomputerAdapter.read()`, the full hidraw round trip of one daemon
    tick, with both controllers configured. Measured 2026-09-15 (§2
    "hidraw check"): status read 2 ms, first open 0.1–0.9 s, `apply()` 1 ms
    without a write and 9–13 ms with one, one SET with all four outputs
    16 ms (aquaero) / 26 ms (Quadro); the gap scan that set `ctrl_gap_ms`
    to 100 ms (aquaero) and 0 (Quadro), all with the save report after
    every SET (item 87 measures without it). Measured 2026-09-17 (min /
    median / p95 ms, Zero 2 W, no save report since item 86):

    | Operation | min | median | p95 |
    |---|---|---|---|
    | aquaero status read + decode | 0.70 | 1.23 | 1.28 |
    | Quadro status read + decode | 0.55 | 0.88 | 0.90 |
    | `AquacomputerAdapter.read()`, aquaero | 1.95 | 3.05 | 3.12 |
    | `AquacomputerAdapter.read()`, Quadro | 1.03 | 1.67 | 1.70 |
    | `AquacomputerAdapter.read()`, **both devices together** | 2.99 | **4.73** | **4.82** |

    That last row is the I/O of one daemon tick with both controllers
    configured -- the question this item asked. On the old Zero W the
    same operations cost about 2 ms, 1 ms and 9–26 ms (§2 "hidraw check");
    the Zero 2 W is roughly 2–5× faster on hidraw I/O, well short of the
    ~7× seen on the pure-CPU `step()` path (item 73), which fits: hidraw
    I/O is bounded by the USB control-transfer round trip, not by core
    speed. Still open, and not part of this item's own question: the
    1-Wire buses' timing on this board, a periodic refresh tick, the
    reopen after a re-plug, and how often a control read fails in long
    operation (about one in 60 in the hwmon spike) -- none of those were
    measured here.
81. Outputs that do not follow the written duty. On the Pi (§2 "hidraw
    check") one SET wrote all outputs, but aquaero outputs 3 and 4 and
    Quadro outputs 1, 2 and 4, all without a fan, kept reporting 100 %. The
    aquaero part is explained: the word at controller block +0x0E is the
    output mode (low byte `0x01` DC voltage, `0x02` PWM); outputs 3 and 4
    were in DC mode. `hw/aquacomputer.py` decodes it, the probe prints it,
    and the adapter logs one warning per open for every commanded aquaero
    output of its own not in PWM mode (an aquabus output's mode word is read
    uninterpreted: it took three different values on identical outputs,
    §2 2026-09-17, items 85 and 89); it does not set the mode.
    Duty verification is escalated: a channel that still reports another
    duty after one rewrite
    is logged once as an error, listed in `stuck_channels` and not
    rewritten for the mismatch again until the device reports its duty
    (writes on a changed command continue). Open: whether the config
    declares a mode per output and the adapter sets it; the Quadro's mode
    field (its four channel regions are identical apart from the duty); why
    a DC output without a load reports 100 %. A Quadro whose outputs stay at
    100 % because it sits on the aquaero's aquabus is named as such in the
    stuck error when the aquaero reports the aquabus device (item 85).
87. **Done** (2026-09-17, on the Zero 2 W): the aquaero's `ctrl_gap_ms` of
    100 ms comes from back-to-back writes that were a SET followed by the
    save report (§2 "hidraw check": `EPIPE` at 0 and 25 ms). Repeated with
    SETs alone (item 86, no save report), min / median / p95 ms:
    `apply()` with a changed duty, the gap already waited out: 3.68 / 4.22
    / 4.97; the first write of a back-to-back pair: 3.66 / 3.74 / 5.97;
    the **second** write of that pair, issued right after the first with
    no deliberate wait: **103.17 / 103.33 / 105.59**. That confirms the
    SET itself costs about 3.7 ms and `ctrl_gap_ms`'s 100 ms is the entire
    remaining cost of a second write -- not the bus, not the CPU (a
    Zero 2 W core is markedly faster than the Zero W's and the number did
    not move). So the aquaero still needs the gap even with the save
    report gone (the thing item 86 removed was never what the gap was
    paying for), and the default stays 100 ms -- no new default to decide.
    This also answers this item's second ask: `apply()` with a changed
    duty, gap already waited out, is 3.68–4.97 ms, well under the 9–13 ms
    that included the save report (item 80). Nothing else this item asked
    for is open.
89. Aquabus details the adapter writes or decodes without verification.
    Checked read-only against the live devices on 2026-09-17 (§2 "aquabus
    fields checked against the live devices"), and the code made honest where
    a reading could not be justified:
    - **the mode word of the aquabus blocks** — answered, and the earlier
      `0x0500` turns out not to be a constant: one Quadro's four identical
      PWM outputs read `0x0000` on block 5 and `0x0002` on blocks 6–8 in one
      report. The word says nothing about a bus device's output, so
      `output_mode` now marks an aquabus block **uninterpreted** and names it
      `unknown` whatever its low byte holds.
    - **the `u16` at `+0x0A` of a fan block** — the guess that it equals the
      current in mA is contradicted (26 there against 6 mA in the current
      field). It tracks current and power without matching either. Left
      unidentified and undecoded.
    - **the aquabus blocks' current and power** — contradicted as a
      per-report reading: over 90 consecutive reports the aquaero carried the
      bus device's measurements in 23 and substitutes in 67, so a fan turning
      at 255 rpm reported 0 mA in three reports out of four.
      `DeviceKind.reports_power` is now False for the aquaero's aquabus
      outputs as well as its own, and `fan_readings` publishes `current_ma`
      and `power_w` as `None` rather than a 0.0 the health rules could act
      on.
    - **the aquabus blocks' voltage** — contradicted the same way, and found
      in review of this item's own change: the field holds the bus device's
      rail in the measuring reports and the aquaero's own in the rest
      (12.10 V and 12.09 V on block 7 one second apart at an unchanged
      duty), so it is not that output's rail. `DeviceKind` now carries
      `aquabus_outputs_report_rail` (False on the aquaero), `fan_readings`
      publishes `voltage_v` as `None` with `rail_reported` false there, and
      the rail rule is off for those outputs rather than judging — and
      resetting its own timer on — a substituted value (§2, 2026-09-17).
    - **writing the unconfigured control block** — still unobservable: block
      8 is configured in this wiring (source `0x5C`), so nothing can be
      learned read-only. Instead of writing it blind, the adapter now leaves
      a commanded channel whose block has no control source **out of its
      writes**, logs it once per open and reports it in
      `unconfigured_channels` and `device_health`. The refusal is per
      channel by design: every other channel of that controller is written
      in the same SET and the heartbeat still goes out, because taking a
      whole controller to the aquaero's watchdog fallback over one
      unconfigured block would raise every fan on it to 100 % indefinitely.
      To answer the question for real the owner must deliberately leave a
      block unconfigured with a fan on that output and allow a write.
    - **the rpm lag of an aquabus fan (item 75)** — still open: the lag shows
      only after a duty change, which is a write, so this wiring cannot
      measure it.

92. **Detection shipped** (2026-09-17), the rest still open. The daemon now
    sees a bus device disappear and says so, and — the part that was a safety
    hole — no longer lets the frozen `busN` temperatures of a departed device
    reach the solver as readings. A device answers on aquabus while at least
    one of the aquaero's blocks 5–8 has a device behind it
    (`aquacomputer.aquabus_present`, the speed field only, which every report
    carries); every block reading `0xFFFF` is an empty bus. From the **first**
    such report every logical name bound to a `bus1..8` slot reads `None` in
    the observation, so its zone goes untrusted and holds and then raises —
    nothing in this rule ever lowers a duty, because a lost bus device is less
    evidence, not less heat. The **report** waits for the new `bus_absent_s`
    (10.0 s, > 0, floored by nothing else): one error line naming what went
    missing, an entry in `device_health`'s `problems`, and
    `device_health.aquabus` with `state`, `present`, `seen`, `absent_s`, `lost`
    and `temps_missing` (the `refresh_reports` key this item shipped was removed
    on 2026-09-18 with the number behind it, item 115; nothing keyed on it). `state` (`unknown`, `present`,
    `empty`, `lost`, `never_seen`) is the field a one-line consumer shows, and
    `lost` is True only for a device that answered and then stopped — the empty
    bus of a plain aquaero is `never_seen`, not a fault. `seen` separates "the
    Quadro left" from "nothing has ever answered here", and the window keeps a
    re-enumeration blip (item 90) out of the report. Nothing about the aquabus
    floors it: an aquabus block's electrical fields are one sample taken inside
    the PWM cycle and read 0 mA at a low duty with the fan turning (item 115, as
    corrected 2026-09-18), while the speed field presence is read from is in
    every report, so no report can look like a departure at any window. The
    conclusion is the one this item shipped; only the sentence that justified it
    has changed, and it is now better founded, since what was taken for a poll
    interval was never one. Tests cover the departure on one adapter
    and through the loop (a bound `bus2`, where no tick after the loss commands
    any channel below what it commanded before it, ending at `fallback_pwm`),
    a report whose electrical sample missed never being read as an absence, a
    single transient
    `0xFFFF`, a controller with nothing bound on the bus, a bus that was never
    there, and a reopen of the node (which does not restart the clock on an
    empty bus, so `lost` and `absent_s` stay one pair). §3 Track B and the
    README no longer forbid binding a `busN` — at the time this item shipped,
    they required the same entry to bind one of that device's aquabus outputs
    too, because a bus device with **no fan outputs** was indistinguishable
    from an empty bus on the fan blocks alone, so its slots would read as
    missing for ever and that shape was refused rather than warned about.
    **Superseded by item 130** (2026-09-18): the aquabus flow slot is a second
    witness that needs no fan output on the bus device at all, so on the
    aquaero that condition is no longer enforced — see item 130 for the
    evidence and what still falls back to it. Not done on the hardware: the board
    was unreachable all session, so the rule has not been shown quiet on the
    real bus over a long
    run (`tools/aquabus_watch.py --seconds 900`), and the departure has not
    been repeated with the daemon running. Still open besides, unchanged:
    why the menu misbehaves with the Quadro on aquabus, what control-report
    byte `0x1A` means, and the live write to outputs 5–8 once the link is back.
    The original report:
    The aquaero lost the Quadro on aquabus without a restart (2026-09-15,
    between 21:14 and 22:21; aquaero uptime counter 81 min, Quadro power
    cycles unchanged, its USB still connected). The aquaero's fans 5–8 then
    read rpm `0xFFFF` and 0 V and flow 3 `0x7FFF`, but its aquabus
    temperature `bus2` stayed at 24.12 °C while the Quadro reported 23.66 °C
    over its own USB: a lost aquabus device leaves its temperatures frozen,
    not missing. In the same window byte `0x1A` of the aquaero's control
    report changed from `0x01` to `0x00` (meaning unknown). The Quadro kept
    its own saved duties. Cause: the owner unplugged the Quadro from aquabus
    because the aquaero's menu misbehaved while it was connected. More than
    an hour later `bus2` still read 24.12 °C. To do: find out why the menu
    misbehaves with the Quadro on aquabus (firmware versions, bus speed,
    address) and whether the Quadro stays on aquabus or on its own USB;
    what `0x1A` means -- it read `0x00` on 2026-09-17 *with* the Quadro
    present on aquabus (§2 "aquabus fields checked against the live
    devices"), so it is not a flag for an aquabus device being present; treat `busN` temperatures as
    missing while the device behind them is absent (for the Quadro: its fan
    slots 5–8 read `0xFFFF`), or refuse to bind them without such a check —
    item 90 now judges that absence per slot, so the evidence a `busN`
    temperature would need is already decoded. **This got more urgent with
    item 90**: until then a commanded aquabus output reading `0xFFFF` made
    `read()` raise, which kept the frozen `busN` values out of the solver as a
    side effect; now the observation comes through with only that channel's
    `rpm`/`pwm` nulled, so a bound `busN` of a departed device reaches the
    gate as an ordinary reading and its zone would hold its fans while the
    real temperature climbs (the Stuck rule catches it only once some channel
    moves). No shipped example binds a `busN` — they bind `temp1..temp8` —
    and §3 Track B and the README now say not to bind one until this item
    lands, but that is a warning, not a check;
    once the link is back, repeat the adapter's live write to outputs 5–8 on
    the hardware (PR for items 85 and 86 was checked on the aquaero's own
    outputs only: no save report, live writes, the fault floor).

93. On the hardware (the main session, the Pi and the controllers): run the
    daemon's own software-sensor heartbeat and profile handling of item 84
    against the aquaero. Confirm that the adapter's output report `0x07`
    keeps profile 1 while the daemon ticks (the sensor reading 20.00 °C in
    `soft1`, the other seven untouched); that stopping the daemon lets the
    alarm select profile 2 within about 30 s and every output run at 100 %;
    that when it comes back the profile-change line is logged once and every
    configured channel is written again within `ctrl_refresh_s`; and that a
    heartbeat write costs no more than a few milliseconds on the Pi (it is
    inside `ctrl_budget_s`, but the measured cost belongs next to the other
    timings in §2). Confirm the **negative** half too, which is the point of
    the ordering: with the daemon running and its writes failing (pull the
    aquaero's USB after `read()` has an observation, or make the SET fail),
    no heartbeat goes out and the alarm fires on schedule.
    **Read the heartbeat back**: `write_report` only hands the report to the
    kernel (§2), so `heartbeat_ok` is "accepted", not "delivered". Compare
    `soft1` in the status report against `heartbeat_value_c` on the hardware;
    if the sensor can be bound or read cheaply every tick, make that read-back
    drive `heartbeat_ok` so a queued-but-lost heartbeat is visible instead of
    showing a healthy watchdog while the controller runs down its timeout.
    Item 113's `software_sensor_settings()` decode makes this cheaper than
    it was: the control report now gives the slot's timeout and fallback
    directly, so the read-back can say not only "the value came through"
    but "the value is fresher than the timeout and is not the fallback" —
    folded in here rather than carried as its own item (proposed by item
    113's PR).
    Then set `heartbeat_sensor: 1` on the owner's deployment and decide
    whether it goes into `config.example-das.yaml` as well, against the
    convention that both example files show every timing key at its default
    (`tests/test_model_config.py::test_example_configs_show_every_timing_key_at_its_default`).
    Until then that file ships 0 with the cost of 0 written next to it:
    `soft1` is enabled on that aquaero, so with nothing writing it the alarm
    selects profile 2 about 30 s after boot and the watchdog is spent — live
    writes keep working, so only the profile byte shows it. While
    the Quadro is off aquabus (item 92), also confirm item 90 on the
    hardware: the aquaero's own thermistors and outputs keep working with
    `pwm5..pwm8` absent, one error line instead of a read failure per tick,
    and the channels back when the Quadro returns.

96. **Not applicable in the supported topology** (owner decision
    2026-09-16, §8.1, §2 "Supported topology"). Item 88's hardware half was
    going to confirm that the Quadro's own save report (identical to the
    Farbwerk 360's) persists a configuration over a power cycle, the way
    report 6 does on the aquaero (verified 2026-09-15, item 86), by running
    `tools/aquacomputer_commission.py --device quadro --save` against the
    real Quadro. That needs the Quadro reachable on its own USB while it
    stays wired to the aquaero's aquabus too — the exact two-access-path
    setup that proved unstable (§2) — so it does not run against the
    supported topology, and this item is not carried out. It would apply
    again only if a Quadro on its own USB port, independent of the
    aquaero, became a supported second controller.

97. Tune `mpc.stuck_zone_air_dT_c` on the real enclosure (item 58). Its
    default, 1.5 °C, is the simulator's number: it clears the largest
    zone-air move (1.20 °C) seen at a steady airflow on a healthy proximal
    reading inside its band over 256 `rich` 2.5-hour runs. On the hardware
    the quantity to measure is the same — with the recorder running, take
    every window in which a proximal reading stayed inside its
    `stuck_eps_c` band while its zone's relative airflow stayed inside
    `stuck_airflow_net`, and look at how far that zone's air moved. The key
    should sit above the largest such move with room to spare; if the real
    spread is much tighter than the simulator's, lowering it catches a dead
    sensor sooner (on the sim, 1.25 raised the coverage from 28 of 51 frozen
    readings to 37, almost all of it on one seed whose air moves sit in the
    1.25–1.5 °C band — the margin is what this measurement decides). Same
    measurement for `mpc.stuck_air_oppose_max_c` (item 59, default 3.0 °C
    against 1.19 °C on the sim), from the windows where the zone air moved
    against the airflow. Both keys only ever *add* evidence, and only where
    another bay's reading followed the air, so a wrong value costs coverage,
    not false faults.

113. **Done** (2026-09-17): both halves — the daemon refuses to bind any
    `softN` **and** publishes every slot for what it is. The refusal is
    total, not limited to the slots the daemon does not write: a `softN`
    holds whatever a host last wrote into it and, once that host has been
    quiet for the slot's configured timeout, its configured fallback for
    ever — a steady number that never goes stale and never reads `0x7FFF`,
    which the status report cannot separate from a measurement in either
    direction (a *disabled* slot is the only state it marks). The one slot
    the daemon does write carries `heartbeat_value_c`, its own constant,
    which is no better as a temperature. So `parse_device_section` refuses
    a `softN` in `temp_map` with the reason, and `DeviceBinding.__post_init__`
    refuses it again so no path below the config parser can build one; the
    hwmon-era hint that used to say "use `soft1`" now says why `soft1` is
    refused too, and no message offers a `softN` as a choice of input any
    more. Nothing `softN` reaches the estimator, the recorder or a health
    rule.
    **This is a breaking config change with an upgrade action**: a
    `temp_map` that binds a `softN` parsed yesterday and exits 2 today, so
    the daemon does not start and the controller runs on its own saved
    preset until someone reads the journal. Fail-safe, but an outage if it
    is a surprise — so the README's config step, both example configs and
    this item all say it: repoint such a binding (a physical `tempN`, or a
    `busN` of a device that cannot leave the bus) **before** upgrading. On
    the owner's board no binding is affected (`soft1` is the heartbeat slot
    the daemon writes, not a bound input), but the board was unreachable
    while this was written and that could not be checked live.
    What a human gets instead: `aquacomputer.software_sensor_settings()`
    decodes the five bytes per sensor the control report holds from `0x177`
    (enabled `u8`, fallback `s16` centi-°C, timeout `u16` s) and
    `device_health.devices[].software_sensors` publishes each slot with
    `written_by_daemon`, its current `reading_c` and `reads_fallback` — the
    reading *is* the configured fallback, compared exactly at centi-°C
    (same field on both sides, so no tolerance and no config key).
    `tools/aquacomputer_probe.py` prints the same three numbers, and the
    status listing says under `softN` that a value there is not a
    measurement. One `problems` line comes of it, the state the daemon
    cannot see any other way: a `heartbeat_sensor` the controller has
    **disabled**, where the write succeeds every tick and the sensor the
    alarm watches stays `0x7FFF`, so item 84's watchdog cannot fire. An
    enabled slot nothing feeds is published, not a problem — on the owner's
    board seven of eight are in that state and a problem line there would
    leave `/api/health` permanently not-ok.
    Verified offline against five captured control reports (§2, table row
    "software-sensor settings"): the settings explain every `softN` reading
    in every status capture, in both directions — `soft1` at 30 s / 90.00 °C
    is the owner's watchdog reading the heartbeat's 20.00 °C, `soft2..8` at
    300 s / 50.00 °C read exactly 50.00 °C, and the earlier captures'
    disabled `soft3..8` are exactly the slots those reports show as
    `0x7FFF`. No hardware access was needed and none was used for this.

114. **The duty-average suspicion is withdrawn; the field is still
    unidentified** (corrected 2026-09-18). The reading below — "a current
    measured over the output's on-time, of which the current field is the duty
    average" — rested on the current field being an average over the report. It
    is not: it is one sample taken inside the PWM cycle (§2, 2026-09-18), so a
    pair of `+0x0A` and the current field is two coordinates of one instant and
    the fit below has no meaning, not merely thin support. The 45-report run adds
    a pair no scaling of the current survives: one report read 2 mA with the
    field at 0, next to nine that read 6 mA with 26 and one that read 1 mA with
    3. The field stays decoded raw, published by nothing and judged by nothing.
    What would now identify it is a run at a **high** duty, where the current
    field reads in every report and both numbers are steady — which is a duty
    change, i.e. still a write, and still the owner's call.

    *The entry this corrects, left as the record —* **Read-only evidence
    gathered, the answer needs a write** (2026-09-17;
    §2 "The bus in three questions"). What the `u16` at `+0x0A` of a fan block
    is *not*: not the current (26 against the same block's 6 mA in the same
    report), not the power (7 cW there), not a constant, and nothing at all on
    the aquaero's own blocks 1–4, where every capture reads 0. It is refreshed
    with the electrical group (0 in a report that refreshed nothing), so it
    comes from the bus device. What the captures suggest is that the field times
    that block's own output duty lands near the current field — and how thin
    that is, stated plainly: of the two captured duties only one can test it,
    since at 100 % duty `raw × duty` is the raw value itself (27 against 27 mA)
    and tests nothing, and at the one informative point it misses — 26 × 20 % =
    5.2 against a measured 6 mA, 13 % low. The 20 % run's further pairs (22↔5,
    15↔4, 11↔3, 3↔1) are all at that same duty: they constrain a slope, not a
    duty relation, and under `raw × duty` each sits 0.4 to 1.0 mA high, every
    one in the same direction. A line fitted to the five pairs does fit them to
    ±0.3 mA, but with a slope of 0.211 and an intercept of 0.55 mA — that line
    is not `field × duty`. So "a current measured over the output's on-time, of
    which the current field is the duty average" is a suspicion with one
    informative point that misses, not a finding; a second duty is a duty
    change, i.e. a write. The same goes for item 89's suggestion of looking at
    the aquaero's own outputs in DC mode (a mode change is a write). So it is decoded raw as
    `FanStatus.unidentified_raw` for `tools/aquabus_watch.py`, published by
    nothing and judged by nothing, and the field stays unidentified until the
    owner decides between one bench experiment (three duties on an aquabus
    output, and one output in DC mode) and writing it off for good. The
    experiment is what the decision needs, not more read-only watching.

115. **The interval is withdrawn: what was measured was the duty**
    (corrected 2026-09-18; §2, "The current and power of a fan block are a sample
    inside the PWM cycle"). The 23-in-90 below counted the reports whose aquabus
    block held a non-zero current, and that count follows the output's **duty**,
    not any bus poll: 4 of 14 at 25 %, 16 of 16 at 60 %, 18 of 20 at 100 % on the
    owner's hardware, and 11 of 45 at 20 % on a read-only run the same day. A
    fixed one-in-four poll cannot give 16 of 16. So `AQUABUS_REFRESH_REPORTS`,
    `AQUABUS_REFRESH_S`, `DeviceKind.aquabus_refresh_reports` /
    `aquabus_refresh_s` and `device_health.aquabus.refresh_reports` are removed
    rather than re-rounded — there is no number to round, because no bus poll
    interval has been measured on a field that is not duty-sampled. Nothing was
    bounded by them. The 45-report run also corrects the reading in the other
    direction: the four aquabus blocks are sampled **together** (block 7's
    current was non-zero in exactly the 11 reports where block 5 read 0.00 V, and
    in no other), so "the refresh is not atomic" was an artifact of the same
    mixed classifier. What is still open is the question this item was really
    about, now asked of a field that can answer it: **at what interval does the
    aquaero poll its aquabus behind the speed and duty fields?** Those two are in
    every report and may simply be held between polls, so a run at a duty where
    the current is steady (60 % or more, where it reads in every report) is what
    would show a poll gap at all. `tools/aquabus_watch.py` now prints the share
    of sampling reports against the block's duty, which is the measurement; a run
    at one duty says nothing on its own. The same run answers a second thing the
    45-report run could not: **what the voltage field's alternation is.** An
    output with no fan draws no current in either phase, so its 0.00 V is not a
    PWM phase, and a report that simply does not carry the bus device's values
    explains that field as well as in-cycle sampling does. At 60 % the two part
    company — in-cycle sampling has the no-fan block's 0.00 V share rise with the
    duty like the current's, a relay account has it stay near a quarter — and
    `tools/aquabus_watch.py` prints that share per block already. Nothing in the
    code turns on the answer (the rail is refused either way, §2), so this is a
    record to correct, not a rule to enable. Whether a poll rate is a **device
    setting** is unchanged and still needs the owner's menu and a before/after
    control report — and a setting found there would not make the current
    judgeable (item 132).

    *The entry this corrects, left as the record —* **Interval measured and
    recorded, its cause still open** (2026-09-17).
    From the 90-report run: 23 reports carried the bus device's own electrical
    values and 67 carried substitutes over 88.6 s — one refresh in 3.9 reports,
    3.85 s, at a report cadence of 0.98 s — and the refresh is per block, not
    per report (one report refreshed blocks 5 and 6 but not 8, another carried
    a current with a zero power). The 3.85 s is a **mean** over 23 refreshes and
    not the longest gap, which this run cannot give. That is now
    `aquacomputer.AQUABUS_REFRESH_REPORTS` (4) and `AQUABUS_REFRESH_S` (4.0) on
    `DeviceKind`, where a caller reads it: `tools/aquabus_watch.py` reports it
    and `device_health.aquabus.refresh_reports` publishes it. It bounds no key:
    item 92's `bus_absent_s` is deliberately not floored by it, because presence
    is read from the speed field every report carries, so a skipped poll cannot
    look like a departure at any window.
    Only the electrical fields follow the interval — speed and output duty are
    in every report, which is what makes item 92 judgeable and item 116's
    warning about keying on a voltage concrete. What is **not** answered: the
    board was unreachable all session, so nothing new was measured on the
    hardware. Whether the interval is fixed or drifts, whether it differs per
    block, what the longest gap actually is and whether any of it moves with the
    report rate is one run of the new
    `tools/aquabus_watch.py --reports 600 --raw` away (read-only, no feature
    report at all). Whether it is a **device setting** the owner could raise
    cannot be answered read-only: no field of the 2707-byte control report is
    known to carry an aquabus poll rate, and finding one means changing that
    setting in the aquaero's own menu and diffing a control report captured
    before against one captured after — an owner action on the device, not a
    write from the daemon. Until then `aquabus_outputs_report_power` stays
    False.

116. **Done** (2026-09-17): the rule that separates the two is rpm
    `0xFFFF`, it is already the only one used, and it needs no time window.
    Checked against the three captures rather than by reading the code
    alone (§2, "the same substitution is why absence is judged on the speed
    field alone"): with nothing on aquabus, blocks 5-8 read rpm `0xFFFF`
    **and** 0.00 V; with the Quadro present, its three outputs with no fan
    read 0.00 V in the measuring report — byte for byte what an empty slot
    reads — and 12.09 V one second later at an unchanged duty. So 0.00 V is
    neither necessary nor sufficient for absence and would judge one slot
    both ways within a second, while the speed field carries no substitute
    at all: `0xFFFF` for an empty bus, 0 rpm for a present output with no
    fan, the fan's speed otherwise, identically in both reports. Because
    the sentinel is invariant across the refresh, **no confirmation window
    is added** — a present bus device is never judged absent, and an absent
    one is judged absent on the first report that shows `0xFFFF`, which is
    the promptness a window would have cost. What the captures do *not*
    show is the absent→present transition itself: all three come from a
    healthy, uninterrupted bus, so how soon the aquaero starts showing
    `0xFFFF`, and whether a single dropped aquabus poll can show it for one
    report, is untested and stays item 96's hardware work. That is why the
    decision is stated as "the sentinel is invariant", not as a measured
    latency; a spurious single report would cost a false `absent_channels`
    line and a `None` reading for that tick, never a wrong number.
    The audit found no rule keyed on the voltage, current or power: every
    absence judgement in the project goes through `FanStatus.present`
    (`_empty_slot` in the adapter, and `fan_readings` through it). What is
    added is the pin and the reason, so it cannot regress silently:
    `FanStatus.present` says in its docstring that the speed field is the
    only one that answers this and why, and
    `test_only_the_speed_field_says_whether_a_bus_device_is_there` holds the
    three captures against each other. The `fan_health` rail rule's "0.0 V
    is not judged" clause is not an absence rule and is unchanged; it is
    what keeps a *present* empty output from reading as a dead rail.

117. **Done** (2026-09-17): **the gap is accepted, and the code no longer
    claims coverage it does not have.** Closing it needs a controller that
    reports its own outputs' rails to this daemon — a Quadro on its own USB
    — which the supported topology does not have (owner decision
    2026-09-16, §2), and the aquaero's report holds no substitute signal.
    Three candidates were looked at and all three fail on what the captures
    show:
    - *a per-report marker for "this report carries the bus device's
      measurements".* An aquabus output with no fan reads 0.00 V in a
      measuring report, which would identify one — but only on a
      controller that happens to have an unused aquabus output, so the
      coverage would vanish the day the owner plugs in a fourth fan. And
      the refresh is not even atomic per report: one capture had current
      6 mA with power 0, another had blocks 5 and 6 refreshed and block 8
      not, so there is no per-report flag to find and not even a reliable
      per-block one.
    - *a low reading over a window.* The substitute is the aquaero's own
      healthy rail, so a reading below `rail_min_v` could only come from
      the bus device — which is a real signal, but it turns a sustained
      rule into a single-sample one, and nothing in a read-only capture of
      a **healthy** rail can show what a sagging one actually puts in that
      field. Proposed as an item rather than shipped on reasoning alone.
    - *the aquaero's own rail as a proxy.* It is already judged on blocks
      1-4, and it covers a sag common to the whole 12 V supply — but the
      bus device has its own power input, so it is not evidence about that
      device's rail and must not be presented as such.
    So the decision is the second branch of this item, and what it adds is
    the declaration. Each channel's published verdict now carries
    `rpm_monitored`, `rail_monitored`, `power_monitored` and an
    `unmonitored` mapping of rule → why it did not run, the reason written
    by the hardware adapter (which knows why the field is not that output's
    own measurement) and carried through `fan_readings`'s `not_measured`
    into `HealthMonitor.check_channel`. An empty `problems` list on an
    aquabus output is therefore no longer readable as "the rail is fine",
    in `/api/state`, the MQTT attributes or the page.
    **A flag says the rule ran, not that the device measures the field** —
    the distinction the first version of this change got wrong. A rule
    needs the measurement *and* the configuration it is judged against, so
    all four of these publish as not monitored, each with its own reason:
    an output whose field the controller substitutes (the aquabus rail, the
    aquaero's own 0 mA / 0 W); an output that reports power on a model with
    no `power_w_at_max`, which is the state of **both example configs**
    until item 94 — a seized fan drawing 0 mA at full duty would otherwise
    publish as a power rule that ran and passed; a channel with no fitted
    curve in `mpc.fans` / `mpc.fan_models`, where the rpm rule is off (a
    legacy config, or a channel left out); and a rail of exactly 0.00 V,
    the one rail reading this daemon refuses to interpret, which was a
    silent skip before. The value is still published where there is one —
    0.00 V is a reading — and the page prints it with "rail not monitored"
    beside it rather than as a number something checked. `health.py`'s module
    docstring, §2 and §3 state the same thing in words, naming what stays
    covered (the aquaero's own outputs 1-4, and any sag common to the whole
    supply) and what does not (a rail local to the bus device).

118. **Done** (2026-09-17): the `dev` extra now depends on `http` and
    `mqtt` (`dev = [..., "aqua-bridge[http,mqtt]"]`, a self-referential
    extra, `pyproject.toml`), so `pip install -e ".[dev]"` alone pulls
    `aiohttp` and `paho-mqtt` — the same package set the existing
    `pip install -e ".[dev,http,mqtt]"` install lines already asked for
    (§11), now guaranteed rather than merely documented. Verified with
    `pip install --dry-run -e ".[dev]"` in a clean venv: `aiohttp` and
    `paho-mqtt` are pulled in. Before this, a clean venv installed only
    with `.[dev]` (missing the `http`/`mqtt` extras the full suite in §4
    needs) hit two different failure shapes depending on the test: a
    bare top-level `import aiohttp` (`tests/test_http_api.py`,
    `tests/test_http_auth.py`, ...) is a collection error, loud but not
    named as an extras problem, while `pytest.importorskip("paho.mqtt.client")`
    (`tests/test_mqtt_ha.py`, `tests/test_ha_check.py`,
    `tests/test_mqtt_live.py`) is a silent skip — the item 108 shape. Fixing
    it at the install boundary (nothing that asks for `dev` can end up
    without `http`/`mqtt`) closes both shapes at once rather than papering
    over either test file individually.
128. A sagging aquabus rail from the readings below the window (item
    117's audit). Item 117 accepts that a rail behind a bus device is
    undetected, on the grounds that no per-report marker says which
    reading an aquabus block carries. One asymmetry survives that
    argument: the aquaero's substitute is *its own rail*, which is
    itself judged on blocks 1-4, so a reading strictly between 0 V and
    `fan_health.rail_min_v` cannot be the substitute and must be the bus
    device's own measurement. A rule could fire on "this block read
    below the window at least N times in the last W seconds, while the
    aquaero's own rail stayed inside it", keeping a sustained-evidence
    shape at about a quarter of the sample rate. Not shipped because
    nothing read-only shows what a *sagging* bus rail actually puts in
    that field — the captures only ever show a healthy one — so the
    rule would be coverage claimed from reasoning. To settle it the
    owner would have to run a Quadro output on a deliberately sagging
    supply, or accept the rule on the reasoning. If taken, `N` and `W`
    are two new `fan_health:` keys with defaults derived from the
    measured refresh rate (23 measuring reports in 90).
130. **Done** (2026-09-18): a bus device with no fan outputs (item 92's
    audit). Item 92 judged an aquabus device present from the fan blocks
    5-8 alone, so a sensor-only slave (a Farbwerk 360, a Quadro with no
    fans) would read as an empty bus and its `busN` slots would read as
    missing for ever — safe, but blind — and the config model refused
    the binding outright, which also shut out a legitimate shape: a
    Quadro on aquabus whose fans this daemon does not command.
    **In scope, and the flow slot is the evidence.** The aquaero's third
    flow slot (`flow3`, `0xFD`) is already confirmed against the live
    devices (§2, "aquabus fields checked against the live devices"):
    `0x7FFF` ("no data") with nothing on aquabus, 0 ("present, nothing
    connected") with the Quadro on it, in every capture that has been
    taken. It needs no fan behind any block — the aquaero relays it
    whether or not the bus device drives anything — so it is exactly the
    witness item 130 asked for, already gathered rather than a new bench
    session. `aquabus_present` now reads it as a second, independent
    check after the fan blocks (`DeviceKind.aquabus_flow_index`, 3 on the
    aquaero, `None` on the Quadro, which has no aquabus of its own to
    judge): present if either says so, absent only when both do.
    `aquabus_binding_problem` drops its requirement to bind one of the
    device's aquabus outputs on a kind with that witness, so a `busN` may
    now be bound on the aquaero with nothing else in the same entry — the
    rule still refuses a kind with **neither** the flow witness nor a
    bound output, which is dead in the supported topology (the aquaero
    is the only kind with aquabus temperature slots, and it always has
    the flow witness) and stays only for a future controller kind that
    adds one without the other.
    A departing device was already confirmed to blank the flow slot at
    the same time as the fan blocks (the original 2026-09-15 report:
    "fans 5–8 then read rpm `0xFFFF` and 0 V and flow 3 `0x7FFF`"), so the
    new witness goes stale exactly when the old one does — nothing here
    changes when a departure is reported, only whether a fan-less device
    is ever seen as present at all. Tests: the flow slot's two states
    pinned against the same fixtures item 92's own test uses
    (`test_aquabus_presence_also_reads_the_flow_slot_for_a_device_with_no_fan_outputs`),
    a synthetic "every fan block absent, flow slot present" report reading
    `aquabus_present` `True`, a kind with `aquabus_flow_index` cleared
    falling back to the old fan-blocks-only behaviour on the same report,
    and `DeviceBinding`'s own construction test updated for both directions
    (a bare `busN` on the aquaero now accepted, a synthetic no-witness kind
    still refusing it). The three fake "bus device gone" fixtures
    (`tests/test_hw_aquacomputer_adapter.py`, `tests/test_hw_sources.py`)
    now blank the flow slot too, matching the original report instead of
    only the fan blocks — without that they would have read a departed
    device as still present through the new witness alone, silently
    contradicting the item 92 capture they are built from.
    Not done: a sensor-only device (no fan outputs on the bus at all) has
    never actually been on the owner's bus, so the flow slot's behaviour
    behind one — as opposed to behind the Quadro, whose own flow header is
    what has been captured — is inferred from the aquaero relaying
    whatever answers, not measured against a second device kind.
131. The one experiment that names `+0x0A` (item 114; **restated
    2026-09-18**, because what it asked for is no longer a question that
    can be answered). It asked for the `field x duty ~ current` line to be
    checked against a third and fourth point. That line is withdrawn: the
    current field is one instantaneous sample and not an average, so a
    pair of the two fields is two coordinates of one instant and fitting a
    duty relation to them means nothing (item 114). What is left, and is
    still worth one bench session with writes allowed: run the output at
    **60 % or more**, where the current field reads in nearly every report
    and both numbers are steady, and see whether `+0x0A` holds any fixed
    relation to the current there at all; then put one of the aquaero's
    own outputs in DC mode and see whether the field stops reading 0. A
    low-duty run cannot answer it, whatever the fit. Either it is named,
    or item 114 is closed as unidentified for good.
132. Whether the aquabus poll rate is a device setting (item 115;
    **restated 2026-09-18**). Owner action on the aquaero's own menu, not
    a daemon write: capture the control report, change the aquabus setting
    on the device, capture it again, diff the 2707 bytes. What a field
    turning up would and would not buy: it would say the poll rate is
    configurable, and it would be worth having next to the run that
    measures the poll interval behind the speed and duty fields. It would
    **not** make an aquabus output's current a health signal, and
    `aquabus_outputs_report_power` must stay False whatever the setting
    reads: the current is sampled inside the PWM cycle, so at a low duty
    most reports read 0 mA with the fan turning, and no poll rate changes
    that.
135. A fan with no tachometer is invisible to the spin-up rule (item
    75); the owner's idea is to watch its **current** instead, and the
    evidence says what would have to be settled first. A stalled rotor
    draws differently from a turning one, so an output's current could
    in principle stand in for a tachometer the fan does not have — but
    32 status reports over 31.7 s with the Quadro's own USB unplugged
    (aquabus path alone) show three problems: the aquaero's own outputs
    report 0 mA / 0 W in every report, so no current-based rule can
    ever cover a fan wired to the controller itself, where three of the
    owner's four fans are — the device does not measure the field at
    all, not a gap to close (§3 "Fan and device health",
    `power_reported`); an output behind the aquabus device carries a
    non-zero current in only 9 of the 32 reports, the same roughly
    one-in-four refresh item 115 describes, and unplugging the slave's
    USB changed nothing, so it is the controller's own bus polling; and
    the values themselves are implausible as they stand — 1, 2, 4 and
    6 mA for a 12 V fan turning at about 310 rpm, where tens of
    milliamps would be expected, so the field's scale is not settled
    (item 114 is about exactly that field family). Not shippable yet:
    first settle the current field's **scale** (item 114's bench
    experiment, or writing the field off), then find a **sampling
    window that outlasts the refresh pattern** — a single report proves
    nothing when three in four carry a substitute, so the rule would
    have to accumulate over several refresh intervals (item 115
    measured a 3.85 s mean, not the longest gap). Any such rule could
    only ever cover outputs behind a bus device, never the aquaero's
    own, so at most a quarter of the owner's enclosure, and would have
    to declare that in its `monitored` flag like every other rule that
    does not cover what it looks at.
136. The capture that decodes the aquaero's start boost (owner action;
    §2 "The controller's own start boost"). The firmware's per-output
    start boost is in the control report and cannot be picked out
    read-only: four of a controller block's six undecoded `u16`
    (`+0x08`, `+0x0A`, `+0x0C`, `+0x12`) hold one value across all
    eight of the owner's outputs, so there is no difference between
    outputs to read there, and the two that differ (`+0x00`, `+0x02`)
    differ in a way nothing connects to a boost, with no capture saying
    what was set to make them differ. What settles it is one capture
    only the owner can produce: set the start boost to two clearly
    different values on two outputs in the aquaero's own menu — say
    output 1 at 100 % for 5 s and output 2 at 40 % for 1 s — and
    capture a control report; a before/after capture on one output does
    as well. The words that differ, and how, are the decode. The
    groundwork is done: the six words are read into
    `ChannelState.undecoded` and printed by `tools/aquacomputer_probe.py`
    under each output, and the firmware's default for an unassigned
    output is pinned from blocks 9-12 as the third reference point.
    Once decoded: name the fields, publish them next to the power
    limits in the probe and the device health, and keep them read-only
    — writing a boost needs the save report, which persists the whole
    control report over the owner's own configuration, and that
    decision is separate (§2, "The controller's own start boost").
137. Current as evidence that a tach-less rotor turns, bounded to a
    spin-up kick only (item 75, item 135). The current field of an
    output behind a bus device is a PWM-cycle sample, so it is useless
    at a low duty — but at a high one it reads in nearly every report
    (16 of 16 at 60 %, 18 of 20 at 100 %), and a spin-up kick drives the
    channel high by definition. So during a kick, and only then, a
    non-zero current is evidence that the rotor turns: the only such
    evidence this installation has for an output whose tachometer is
    missing, unbound or on a splitter's second fan. Bounded by the same
    measurement item 135 is bounded by: it can only ever apply to an
    output behind a bus device, since the aquaero's own outputs report
    0 mA in every report at every duty, so the rule must be off for
    those rather than silently passing. Shape: after a kick of a
    tach-less output, if every report during the kick read 0 mA, report
    it as a channel whose rotor cannot be confirmed — never as a fault
    that changes a duty, and never outside the kick window. Needs one
    bench run to size it (how many reports of the kick, what a stalled
    rotor's current actually reads) before any threshold is written
    down, and every number it uses is a config key with one documented
    default.
138. Does a fan actually fail to start from the preset the controller
    holds (owner action)? The whole case for a start boost rests on it,
    and nothing has measured it. The hint is there: the 2026-09-15
    sweep found both test fans starting only at 25 % while holding
    355-375 rpm all the way down to 14 %, and on 2026-09-18 three fans
    were turning at a 20 % duty only because something had already spun
    them up. The observation is one bench test, and it needs the owner
    because stopping a fan by hand is not something a daemon can do:
    hold an output at the duty the controller would come back with,
    stop the fan, and see whether it starts again. If it does, the
    firmware boost is not worth a write on this hardware and item 136
    can close; if it does not, the saved preset is a cold-start hazard
    on every power return and item 136's decode becomes worth the
    owner's time.

### 8.4 Open — Zero 2 W upgrade

50. **Done** (2026-09-16): the owner moved the controller itself from the
    Zero W to a Raspberry Pi Zero 2 W and installed the daemon there from
    this repository (`deploy/install-pi.sh --user <user> --das`) with the
    same DAS config the Zero W ran, `config.example-das.yaml` — no key
    added, removed or renamed for the move. What the new board changes is
    the step-budget gate, re-measured and re-derived on it (`budget_ms`
    600 → 250, `budget_alarm_ms` 750 → 350; §8 items 73, 95, 2026-09-17);
    every other config key, the HTTP/MQTT surface and the safety contract
    (§2) are exactly what they were on the Zero W. Run on a Zero 2 W with
    the same config.
51. **Done** (2026-09-16): the Zero 2 W runs 64-bit Raspberry Pi OS —
    trixie, kernel 6.18, aarch64, 4 cores at 1.0 GHz — in place of the Zero
    W's 32-bit `armv6l` userland on a single 1.0 GHz core; nothing in
    `src/aqua_bridge` is conditioned on word size, endianness or core count,
    and no request, response or config-schema shape in `model.py`, the HTTP
    API (§6) or MQTT/HA (§7) changed for the move — item 50's re-derived
    step budget is a config value, not an API change. `deploy/install-pi.sh`
    ran unmodified against this board: it still installs
    `deploy/packages-rpi.txt`'s package *names* through `apt-get`, Debian's
    own package resolution picked the `arm64` build of each one, and the
    script's own guard against `pip` fetching or building numpy from PyPI
    (§9) did not fire — so the apt-package venv strategy §9 documents for
    the Zero W needed no change either. §9's "Versions on Trixie" note
    still names only what was verified on the Zero W; it has not been
    re-run with `apt-cache policy` on the Zero 2 W to confirm the same
    package versions there. 64-bit Lite if needed, with no API change.
52. **Done** (2026-09-16; corrected the same day after review): the levels
    are re-planned from the live solver demand every tick
    (`mpc.ident_replan`, default `true`; §3 "Active identification
    experiments"). Each tick every channel's base is raised to the solver's
    want (`diagnostics["target_pwm"]`), its levels are re-derived around it
    and clamped into the band, and because a re-planned level can only reach
    the fans on the next tick, `compose` floors an experiment channel's
    override with the solver's command for the tick. The rule: **a rise in
    demand wins over the experiment's plan, always** — no tick of an
    experiment puts less on a fan than the controller would have
    (`symmetric`: at most the owner-accepted `ident_amplitude` below it).

    Two things the first version of this got wrong, both found in review and
    both fixed here:

    - *The want is not free of the experiment.* The DAS MPC's objective
      carries `weight_dpwm·‖u₀ − prev‖²` and `prev` is what was last put on
      the fan, so during an experiment the want follows the experiment's own
      level (measured on the DAS example, all else equal: `prev` 0.40 → want
      0.398, `prev` 1.00 → 0.962, a slope near 1). Following it raw ratcheted:
      the high level raised the want, the want raised the base, the base
      raised the high level. On the closed loop with `solver: mpc` the base
      ran **0.400 PWM (seed 1) and 0.198 (seed 2) above anything the frozen
      arm ever wanted**, seed 1 pinned at `pwm_max`, so the fans were loud and
      the excitation was squeezed out — on the very configuration
      identification exists to enable. It went unseen because the numbers were
      taken with `solver: pi`, whose demand does not depend on the last
      command at all. The echo now comes off the want first,
      `wanted = demand − max(0, min(prev, own) − base)` (`prev` from
      `diagnostics["prev_pwm"]`, `own` the override the experiment itself had
      on that tick), which leaves a fan the solver's own floor lifted reading
      as demand. Same runs after the fix: **+0.000 and +0.102**, inside one
      `ident_amplitude`.
    - *A monotone base pinned a spent excursion.* The base only ever rose, and
      under `above` the low level **is** the base, so one transient peak held
      the channel and every sibling held with it at that peak for the rest of
      `ident_max_duration_s` — against the noise objective this enclosure is
      built around. The base now falls again: only at a level switch (so a
      hold keeps its excitation), by at most `d_pwm_max`, and never below the
      base of the start.

    The abort decision of a tick is taken on that tick's estimates, with the
    same thresholds in the same order, *before* the next tick's levels are
    re-planned, so the re-plan cannot weaken the decision itself, and a
    re-planned level is never below the frozen plan's for the same tick.
    Across ticks the aborts do move, and that is the point of the item: a
    channel followed up cools its own bay, so an excursion the frozen plan
    would have ended on the envelope runs to completion. Fewer envelope
    aborts are the intended outcome, not a weakened rule.

    *What it was:* with `above` levels the low level was the solver's base
    frozen at start, so a solver that later wanted more cooling on that
    channel was held back until the +3 °C envelope tripped. Deferred to the
    Zero 2 W (owner, 2026-09-14) for the per-tick compute.

    *Cost per tick* (dev machine, arm64, Python 3.14, DAS example: 8
    channels, 15 bays, 4 zones; `facts_from_tick` + `advance` + `compose`,
    6 interleaved repeats × 2000 calls, median): 32.8 µs before this item,
    32.5 µs with `ident_replan: false` and 42.6 µs with `true` — **+10 µs per
    tick**, of which 5.6 µs is the two diagnostics maps copied into
    `TickFacts`, 2.6 µs the re-plan itself and 1.9 µs the floor in `compose`.
    It is paid only on the ticks of an experiment that re-plans: with
    `ident_replan: false`, and on every tick without a running experiment,
    the maps are not copied at all and the cost is what it was before the
    item (−0.3 µs, i.e. nothing, measured). Only the levels are re-derived,
    never the schedule, which is why it stays this small. Scaling it to a
    Zero W by the factor its DAS MPC step differs by (3.25 ms p99 here
    against 609–615 ms there, item 73, ≈ 190×) gives ≈ 2 ms per tick, which
    is a crude upper bound — that factor is set by the numpy work in the
    step, and interpreter bookkeeping does not scale the same way. Against
    `budget_ms` 600 it is small either way, so the default is `true` on every
    board; `false` is there to get the frozen plan back, not to save time.
    The board figure is an estimate until the Zero 2 W run below.

    *Identification* (`tests/test_ident_replan_sim.py`, the committed nightly
    A/B: `sim/das.py` `rich` preset, `k_sigma: 1`, the real loop and
    supervisor with experiments started one channel at a time; the table
    below is that harness at 8 h, seeds 1–4, paired frozen against
    re-planned, both solvers). Both arms start the same 64 experiments and
    end them the same way (`pi`: 8 aborts each, the same reason on the same
    bay; `mpc`: none). What changes is the cooling the experiment withheld:
    **channel-ticks below the solver's own command 1162 (`pi`) and 5426
    (`mpc`) → 0**. The fit is the same within the seed spread — relative
    error of the in-zone `E`, rms over the parameters, median over the seeds:

    | solver | `E` rms frozen → re-planned | worst per-seed | `k` rms frozen → re-planned | worst per-seed |
    | --- | --- | --- | --- | --- |
    | `pi` | 0.357 → 0.375 | +0.037 | 0.179 → 0.179 | +0.049 |
    | `mpc` | 0.354 → 0.343 | −0.003 (every seed better) | 0.160 → 0.156 | +0.016 |

    Neither arm reaches `converged` in this closed-loop scenario (`pe_min`
    stays near zero: with the solver moving the other channels of a zone, one
    experiment at a time does not excite the zoned regressors) — that is the
    scenario, not this item. Item 102 has since measured why and given the
    closed loop a scenario that does converge (`ident_parallel`,
    `tests/test_ident_converge_sim.py`); this A/B is unchanged, still runs one
    group at a time, and still passes.

    *Main session, on the Zero 2 W:* run one experiment on the real
    enclosure with `ident_replan: true` and confirm (a) an experiment on a
    warming bay follows the solver up instead of aborting on the envelope,
    (b) `GET /api/model`'s `experiment.plan_base` rises with the demand, falls
    back after an excursion and never goes below `experiment.base`, and (c)
    the per-tick cost stays inside the step budget (`tools/bench_step.py`,
    `step_ms_max` and the budget counters in `/api/health`, with an
    experiment running) — the ≈ 2 ms above is a scaled estimate, not a board
    measurement.

### 8.5 Done

#### Finished from §8.2 (2026-09-14)

1. **Done:** the PI-like DAS error is now `max(t̂ − soft)`, counting `k·σ` once inside `soft`. PI-like DAS: count `k·σ` once. The per-channel error becomes
   `max(t̂ − soft)` with `soft = limit − comfort − k·σ` (today `k·σ` is also
   added to `t̂`). Regenerate only the DAS goldens `das_*.pi_das.json`; keep
   every per-zone invariant.
2. **Done:** `tests/test_hw_xt6.py::test_live_read_and_writeback` no
   longer writes `0.0` to a channel whose PWM read returns `None`; such
   channels are excluded from the write-back and reported, and the test
   skips with a clear reason if none is readable. **Must land before item 36.**
   (That test went with the sysfs adapter; its hidraw successor,
   `tests/test_hw_aquacomputer_adapter.py::test_live_read_and_reapply_what_the_device_holds`,
   re-applies only duties the device already holds.)
3. **Done:** a zoned sensor's Stuck evidence is now its zone's relative airflow (`stuck_airflow_net`, void for a proximal sensor when the zone air moved against it by more than `stuck_air_oppose_c`) and a proximal sensor's siblings are on its own bay (§3). False zone fault on a healthy enclosure: the Stuck rule's sibling
   evidence faults a zone when an idle bay's DS18B20 stays inside its
   1.5-LSB band for `stuck_s` while a sibling's activity changes and the
   controller compensates (about one in three 75-minute `sim/das.py`
   runs). Fix the evidence rule; regression: long DAS sim runs with zero
   false zone faults.
4. **Done:** the API and page are served only over HTTPS with basic auth (§6), with `tools/http_user.py` for users and a self-signed certificate from `install-pi.sh`.
   HTTPS with basic auth for the API and the page: certificate and key
   paths in `http:`, hashed credentials in a root-owned 0640 file under
   `/etc/aqua-bridge`, every route authenticated, plain HTTP refused.
   Document that the MQTT counterparts (`cmd/bay`, `cmd/limit`,
   `cmd/ident`) rely on broker authentication.
5. **Done:** Runtime budget alarm: `control/loop.py` logs a
   rate-limited warning past `mpc.budget_ms` and error past
   `mpc.budget_alarm_ms`, both now config keys, with `step_ms_last` /
   `step_ms_max` / exceedance counters in `/api/health` and the MQTT state
   blob.
6. **Done:** The relative budget gate in CI is thin (DAS MPC p99
   9–11× the legacy MPC against 12×); `tests/test_bench_budget.py` now
   interleaves, discards a warm-up repeat and gates on the 75th percentile
   of several per-repeat ratios instead of one min/min pair.
7. **Done:** `install-pi.sh --das` installs `config.example-das.yaml` and the `deploy/aqua-bridge-das.conf` systemd drop-in (`ExecStart=` with `--source hwmon`, since 2026-09-15 `--source composite`); without `--das` the legacy path is unchanged.
8. **Done:** `zones.trust_rule: sigma` now trusts a zone on this tick's estimator σ (`sigma_fault_c`, `sigma_air_fault_c`) instead of its drive and air sensor groups, so a lost sensor widens the margin and only a σ past its threshold faults the zone (§3 per-zone trust). `zones.trust_rule: sigma` (zone trust from the estimator's σ); the
   config accepts it, `strict` applies today.
9. **Done:** a sensor whose value the gate rejects now confirms over
   `confirm_ticks` on its own (§3 Sensor confirmation) and stays out of the
   estimator until then, while its group stays trusted through the others.
   A Jump on a redundant group member was accepted after one tick without
   `confirm_ticks`.
10. **Done:** the return from the model fallback checks the drift relative to the drives' observed rate, with `model_return_factor`, `model_return_dwell_s` and `model_drift_rate_tau_s` as config keys, so a sound model returns about 5 minutes after a load step (§3, validity gate and model fallback). The drift check's hysteresis can hold the PI-DAS model fallback for
    tens of minutes after a load step (return threshold 0.25 °C/min against
    0.26–0.29 °C/min physical transients); tune it.

#### Finished from §8.3 (2026-09-15)

74. **Done:** the daemon reads and writes over hidraw (§8.1, 2026-09-15):
    `read()` uses the newest unsolicited status report and reports its
    output duty, and `apply()` sends one control report SET per
    controller, only on a tick whose duties changed, with no control
    report read on a normal tick (§3 Track B). Timing a full tick on the
    Pi is item 80. hwmon PWM cost (§2 "USB spike results"): a `pwmK` read
    takes 210 ms and a write 420 ms, and `Xt6Adapter` reads every `pwmK`
    and writes every channel on every tick, about 5 s per tick with 8
    outputs. Stop reading `pwmK` back on every tick (the original item
    proposed reporting the last commanded value; the adapter reports the
    status report's output duty instead), write only channels whose raw
    value changed (and all of them again after a re-plug), and time a full
    tick on the Pi.
82. **Done:** the DKMS module was removed from the spike Pi on 2026-09-15
    (`dkms remove --all`, source deleted, `dkms` package purged, module
    unloaded); every Aqua Computer HID interface is on `hid-generic`. The
    Pi that ran the USB spike still has the DKMS `aquacomputer_d5next`
    module installed, which binds both controllers. The hidraw nodes stay
    and the daemon works, but anything that reads a `pwmN` attribute
    issues a control report read of its own, outside the daemon's
    `ctrl_gap_ms`. Remove it (`sudo dkms remove aquacomputer_d5next
    --all`, reboot) before item 42 and confirm the daemon runs with
    `hid-generic`.

#### Docs / repo

- [x] Create the GitHub repo (`gh`, §12)
- [x] `pyproject.toml` (ruff, pytest), `config.example.yaml`
- [x] CI: ruff + pytest for track A (invariants, nominal, failures, sensor lies, fuzzy, closed-loop; fake hardware; no live USB)
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
- [x] Active identification experiments per fan group with the +3 °C envelope on estimates, `POST /api/ident`, MQTT `cmd/ident`, HA `ident_running` (`control/ident.py`); levels re-planned from the live solver demand every tick, a rise in demand always winning over the plan (`ident_replan`, §8.4 item 52); optionally a whole zone at once on independent codes, which is what the fit needs to converge in the closed loop (`ident_parallel`, §8.4 item 102)
- [x] Runtime drive limits: `POST /api/limit`, MQTT `cmd/limit/...`, HA `limit_<class>`; DAS presets (comfort band, noise weight)

#### Track B — hardware (Pi USB; fake sysfs anywhere)

- [x] `hw/xt6.py` read/apply, udev `0c70`, sysfs root injectable (replaced 2026-09-15)
- [x] `hw/xt6.py`: `pwmK_enable` re-checked on every apply (re-plug) (replaced 2026-09-15)
- [x] hidraw adapter (2026-09-15, replaces the hwmon sysfs adapter): `hw/aquacomputer.py` report layouts against captured reports, `hw/hidraw.py` discovery and feature reports, `hw/aquacomputer_adapter.py` (status-report reads, one SET per controller on a changed tick, duty verification, power cycles, periodic refresh, `release()`), `aquacomputer:` config with timing keys, `--source composite`, `tools/aquacomputer_probe.py`
- [x] Startup rejection: `xt6.fans` keys == `mpc.channels`, `xt6.temp_map` keys == `mpc.temps`
- [x] `xt6.fans`: one entry per fan with `pwm` and optional `rpm`; legacy `map` / `fan_map` rejected
- [x] `test_hw_map.py` / fake hwmon in CI (replaced by the fake controller, 2026-09-15)
- [x] `hw/sources.py`: several hwmon devices + 1-Wire, every name bound exactly once (`--source hwmon`; since 2026-09-15 controllers over hidraw, `--source composite`)
- [x] systemd unit: `Type=notify`, no ordering on `network-online.target`
      (removed 2026-09-18, §2: the network is outside the cooling path),
      `Restart=always`, `RestartSec`, `StartLimitIntervalSec=0`, `WatchdogSec`, `TimeoutStartSec`,
      `ExecStart=/opt/aqua-bridge/.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml`,
      SIGTERM stop path writes `fallback_pwm` (no `ExecStop=`), `StateDirectory=aqua-bridge`
- [x] udev rule for hwmon `pwm*` group write (`plugdev`; kept for the optional driver)
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
- [x] Tests: HTTP talks to the command sink, not the hardware; Auto rejects raw PWM; fuzz JSON → 4xx

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
| lm-sensors | `sensors` (the controllers are read with `tools/aquacomputer_probe.py`) |
| openssl | self-signed HTTPS certificate (`install-pi.sh`) |
| python3-hid python3-usb liquidctl | liquidctl and ad-hoc USB checks (the daemon's hidraw code needs only the standard library) |
| python3-numpy python3-yaml | controller (runtime deps) |
| python3-aiohttp python3-paho-mqtt | HTTP, MQTT |
| python3-pytest python3-hypothesis | tests on the Pi |

`dkms`, `curl` and `patch` are not in the list: only the optional
`aquacomputer_d5next` DKMS package needs them (below).

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

### HID access

The daemon opens the aquaero's and the Quadro's `/dev/hidrawN` read/write
(`hw/hidraw.py`, §3 Track B); no kernel driver beyond `hid-generic` is
needed. The kernel creates hidraw nodes `root:root 0600`: the udev rule
(below) gives the `0c70` nodes to `plugdev` with mode `0660`, and the unit
puts the service user in `plugdev` (`SupplementaryGroups=`). Check with
`ls -l /dev/hidraw*` and `tools/aquacomputer_probe.py` as the service
user. A permission error there, or a start-timeout loop whose journal
says `cannot open /dev/hidrawN: ... Permission denied`, means the rule has
not applied: re-plug, or `sudo udevadm trigger --action=add
--subsystem-match=hidraw`.

### Kernel module `aquacomputer_d5next` (DKMS, optional)

**Optional, and not run by `install-pi.sh`:** the daemon uses hidraw
(§8.1, 2026-09-15). The package stays in `deploy/` for experiments through
hwmon and in case the driver path is revived. Raspberry Pi OS kernels are
built without `CONFIG_SENSORS_AQUACOMPUTER_D5NEXT`, so the aquaero and the
Quadro bind to `hid-generic` and have no hwmon device. To build the
driver anyway, install `dkms patch curl linux-headers-rpi-v6` (the script
stops early with that hint when `dkms` or `patch`, or `curl` without
`--source`, is missing) and run `deploy/install-aquacomputer-dkms.sh`. The
script:

1. derives the stable tag from the kernel release
   (`6.18.39+rpt-rpi-v6` → `v6.18.39`; `--tag` overrides it) and
   downloads `drivers/hwmon/aquacomputer_d5next.c` from the kernel.org
   stable tree (`--source FILE` uses a local copy);
2. applies `deploy/dkms/aquacomputer_d5next/*.patch` in name order; a
   patch already contained in the source is skipped, one that does not
   apply stops the script;
3. removes other versions of the module from DKMS, installs the source
   with `Makefile` and `dkms.conf` as
   `/usr/src/aquacomputer_d5next-<upstream>-aqb<PATCH_LEVEL>` and runs
   `dkms install`; the module goes to `updates/dkms` and loads by its
   HID alias at boot;
4. loads the module when none is loaded. It never unloads a loaded one
   (the daemon may be using it) and says when a reboot is needed.

It is idempotent, and a kernel with the driver built in (`=y`) is left
alone with a warning. Bump `PATCH_LEVEL` in the script when a patch is
added or changed. After a kernel upgrade DKMS rebuilds the old source for
the new kernel; run the script with `--kernel <new release>` before
rebooting so the source matches the kernel.

The patch (`0001-send-secondary-ctrl-report-from-heap.patch`): after
every control write the driver sends a short follow-up report from static
module data. USB transfer buffers must be DMA-mappable and module data is
not on the Zero W, so every `pwmK` write failed with `EAGAIN` and a
`dma_map_phys` warning (§2 "USB spike results"). The patch sends a heap
copy made once per device. Mainline still has the bug (2026-09-14; §8
item 78).

Check: `sudo dkms status` lists the module as installed, and
`cat /sys/class/hwmon/hwmon*/name` shows `aquaero` (and `quadro` on its
own USB port). With the module loaded the hidraw nodes stay (the driver
connects hidraw too) and the daemon still works, but anything that reads
a `pwmN` attribute issues a control report read of its own, outside the
daemon's `ctrl_gap_ms` (§8 item 82).

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
Documentation=file:///opt/aqua-bridge/PROJECT.md
# No Wants=/After=network-online.target: §2, the network is outside the cooling path
StartLimitIntervalSec=0

[Service]
Type=notify
NotifyAccess=main
# User= is a placeholder; install-pi.sh writes the real service account
User=aqua
SupplementaryGroups=dialout plugdev
ExecStart=/opt/aqua-bridge/.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml
StateDirectory=aqua-bridge
Restart=always
RestartSec=5
WatchdogSec=45
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

The unit's `ExecStart=` above has no `--source`, i.e. `xt6` (the single
controller of `xt6:` over hidraw, §3): that is the legacy install path.
The DAS install path (`--source composite`, the composite `aquacomputer:`
+ `onewire:` source) adds `deploy/aqua-bridge-das.conf` as a systemd
drop-in, `/etc/systemd/system/aqua-bridge.service.d/das.conf`, changing
only `ExecStart=` (an empty `ExecStart=` line clears the base unit's
before the real one is set — repeated-directive semantics,
`systemd.unit(5)`). `install-pi.sh --das` installs it (§10) and
`tests/test_deploy.py` checks that the drop-in's `ExecStart=` is the
base unit's plus exactly `--source composite`, nothing else.

(The file itself carries the reasoning as comments.) **Nothing here waits
for the network**, on purpose: `Wants=`/`After=network-online.target` were
removed on 2026-09-18 (§2, "the network is outside the cooling path") —
with nothing to associate with, `NetworkManager-wait-online` spends its
full 30 s default and the daemon's first write to the controllers is that
much later, which is the aquaero's whole software-sensor window spent on a
router. Do not put them back; `tests/test_deploy.py` fails if any directive
in the unit names the network.
`StartLimitIntervalSec=0` (in `[Unit]`, where it has lived since systemd
229) turns off systemd's default start rate limit. That default — 5 starts
in 10 s, then the unit is parked in `failed` — is the one way this unit can
go silent: parked means nothing writes the controllers and nothing writes
the heartbeat again. `RestartSec=5` is what makes "never park" affordable:
a daemon that exits immediately restarts once every five seconds instead of
ten times a second, and 5 s is far inside the aquaero's 30 s.
`Type=notify` is required or `READY=1` is ignored. `NotifyAccess=main`
means the **main** process is Python: `ExecStart=` is the venv
interpreter and `-m aqua_bridge` directly — no `sh -c`, no wrapper
script. `%h` would be the *manager*’s home (`/root` for a system unit),
not `User=`’s, hence the fixed install dir `/opt/aqua-bridge`, owned by
the service user.

`plugdev` is the group the udev rule gives read/write access to the
controllers' `/dev/hidrawN` nodes (the kernel creates them `root:root
0600`); without it the daemon cannot open them. `dialout` is for the
Digole UART.

**Watchdog against blocked ticks.** Two `WATCHDOG=1` pings further apart
than `WatchdogSec=` get the daemon killed without its `fallback_pwm` stop
write. The loop pings at the end of a tick and sleeps until the next, so
after a normal tick the next ping can come `mpc.dt` + `mpc.budget_alarm_ms`
+ the controllers' worst-case I/O later, each controller at most
`status_max_age_s` + `ctrl_budget_s` + one 5 s usbhid control transfer +
`ctrl_gap_ms` (§3 Track B). For `config.example-das.yaml` with the aquaero
and the Quadro each on its own USB port at their defaults that is
5 + 0.75 + 13.1 + 13 ≈ 32 s, hence `WatchdogSec=45` (it was 30); with the
Quadro on the aquaero's aquabus, as the example now has it, one controller:
≈ 19 s. `build_io` reads the period from
`$WATCHDOG_USEC` (only set under systemd) and exits 2 when that bound is not
below it; raise `WatchdogSec=` or lower those keys, and keep a margin for
the publishers and the recorder, which the bound does not count.
`tests/test_hw_aquacomputer_config.py` checks the unit against the DAS
example and the two-controller alternative, and `tests/test_deploy.py`
checks the *margin*: the bound is 18.5 s as the example ships and 31.5 s
with the Quadro on its own port, so 45 s leaves 26.5 s and 13.6 s for the
publishers and the recorder, whose largest single item on the loop thread
is one `vcgencmd get_throttled` at `host_health.vcgencmd_timeout_s` (2 s,
at most once a minute). `TimeoutStartSec=120` stays above `WatchdogSec`.

`WatchdogSignal` stays at its `SIGABRT` default, and what that means for
the fans is the timeline in §2 ("A slow tick degrades to a restart, not to
silence"): nothing is written, so this is the one path that does *not* jump
the fans to `fallback_pwm` (that is the clean stop below) — but it is not a
free restart either. `WatchdogSec=45` is above the aquaero's own 30 s, and it
has to be, because the bound it must clear is already 31.5 s in the
two-controller layout. So whenever the daemon is the one writing the heartbeat
(`heartbeat_sensor`), a watchdog kill means the alarm fired fifteen seconds
before the kill: every fan at 100 %, then, after the restart, a few seconds at
the saved profile's preset while the duty verification catches up, then the
daemon's command again. With `heartbeat_sensor: 0` and a separate service
feeding `soft1`, as the board runs today, the stall never reaches the
controller and the fans simply hold.

`READY=1` waits for the first applied command, so a device that is
absent at boot (USB not enumerated, hidraw permissions wrong) shows up as
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
that — two writers to the controllers at once.

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

### Board hardening: SoC watchdog, journald, Wi-Fi

`deploy/install-board-watchdogs.sh` — idempotent, run by the owner with
`sudo`, `--check` reports without writing anything, `--no-net-recover`
leaves the Wi-Fi timer out. It installs layer 3 of §2 and the two board
settings the 2026-09-17 outage produced, and it touches
`aqua-bridge.service`, the controllers and `config.yaml` **not at all**
(`install-pi.sh` owns the unit; the script only *reads* it, to print the
layering). Every number is a variable at the top, overridable from the
environment (`sudo SOC_WATCHDOG_SEC=90 deploy/install-board-watchdogs.sh`):

| Variable | Default | What it writes |
|----------|---------|----------------|
| `SOC_WATCHDOG_SEC` | `60` | `RuntimeWatchdogSec=` in `/etc/systemd/system.conf.d/10-aqua-watchdog.conf` |
| `REBOOT_WATCHDOG_SEC` | `120` | `RebootWatchdogSec=` in the same file |
| `JOURNAL_MAX_USE` | `200M` | `SystemMaxUse=` in `/etc/systemd/journald.conf.d/20-aqua-journal-limits.conf` |
| `JOURNAL_MAX_FILE_SIZE` | `16M` | `SystemMaxFileSize=` in the same file |
| `JOURNAL_MAX_RETENTION` | `30day` | `MaxRetentionSec=` in the same file |
| `JOURNAL_SYNC_INTERVAL` | `5m` | `SyncIntervalSec=` in the same file |
| `WIFI_POWERSAVE` | `off` | `wifi.powersave = 2` in `/etc/NetworkManager/conf.d/10-aqua-wifi-powersave.conf` (`keep`: install nothing) |
| `NET_RECOVER_INTERVAL` | `5min` | `OnBootSec=`/`OnUnitActiveSec=` in `aqua-net-recover.timer` |

`--no-net-recover` is an off switch, not a skipped step: it stops and disables
`aqua-net-recover.timer` and deletes the three installed files. Skipping alone
would leave a previously enabled timer running the copy of the script installed
back then, with the thresholds it had back then — and a re-run with the flag
would look like it had turned the recovery off while it had not.

Notes that only show up on a real board:

- **No overlay is needed for the watchdog.** `bcm2835_wdt` is in the base
  Raspberry Pi device tree: `/dev/watchdog` (`Broadcom BCM2835 Watchdog
  timer`) is there on a stock trixie image with nothing added to
  `config.txt`. The script refuses to install if it is absent rather than
  writing an overlay it cannot verify.
- **Raspberry Pi OS already enables it**, in
  `/usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf`
  (`RuntimeWatchdogSec=1m`, `RebootWatchdogSec=2m`). The drop-in above is in
  `/etc`, which wins, so the board runs the value §2 argues for and it does
  not move when `raspberrypi-sys-mods` is upgraded. The numbers happen to
  agree today; the point is that they are now ours.
- **`RuntimeWatchdogSec` is only read when PID 1 re-executes**, so the script
  runs `systemctl daemon-reexec`. That restarts no service and does not
  interrupt the control loop.
- **NetworkManager is reloaded, never restarted** (a restart drops the active
  connection, and the script has to be safe over ssh). `wifi.powersave` applies
  from the next activation of the interface — a reconnect or a reboot.
- `install-pi.sh` does not call this script and this script does not call
  `install-pi.sh`; run both, in either order.

`deploy/aqua-net-recover.{sh,service,timer}` are the network-recovery unit
described in §2. The script is installed to
`/usr/local/lib/aqua-bridge/aqua-net-recover.sh`; its own knobs
(`AQUA_NET_IFACE`, `AQUA_NET_FAIL_CHECKS`, `AQUA_NET_MAX_BOUNCES`,
`AQUA_NET_PING_COUNT`, `AQUA_NET_PING_DEADLINE_S`, `AQUA_NET_NMCLI_WAIT_S`,
`AQUA_NET_STATE_DIR`) are
at the top of the file with their reasoning; the unit passes none of them, so
an override is `systemctl edit aqua-net-recover.service`. The unit's
`TimeoutStartSec=90` is twice the worst case those defaults allow
(`2·AQUA_NET_NMCLI_WAIT_S + AQUA_NET_PING_DEADLINE_S` = 45 s) and is set
explicitly because systemd's default (1 min 30 s here) is *below* `nmcli`'s own
default wait for `device connect` alone. Counters live under
`RuntimeDirectory=` (tmpfs: they are meaningless across a reboot and must not
wear the card). `--dry-run` reports what one check would do and changes
nothing. `tests/test_deploy.py` runs the script against `nmcli`/`ip`/`ping`
stubs and fails if it ever re-associates more than `AQUA_NET_MAX_BOUNCES`
times, pings without a gateway, or touches a disconnected interface that
NetworkManager is already retrying. The stubs cover the failing branches too,
which are the ones the guards exist for: an `nmcli` that exits non-zero, one
slow enough to be killed mid-run, and the silence after the give-up.

**The free-space rule's journal cap is this script's, not a second one.**
`health.HostHealthConfig.disk_free_min_gb`'s default (§3, "the card is low
on space") is argued in part from a journal cap, and the cap it means is
this one — `JOURNAL_MAX_USE` above, installed by this script. An earlier
version of the disk-free-rule change installed its own second drop-in from
`install-pi.sh` (`/etc/systemd/journald.conf.d/aqua-bridge.conf`,
`SystemMaxUse` only); resolved in favor of this script's, since it already
covers the same setting plus `SystemMaxFileSize`, `MaxRetentionSec` and
`SyncIntervalSec` from one set of knobs, and a board running both scripts
must not end up with two drop-ins governing one journal from two different
defaults. `install-pi.sh` does not touch the journal. Whether this
script's own drop-in ever lands on the card is itself conditional:
journald's `Storage=auto` (the Raspberry Pi OS default) keeps the
journal *persistent* under `/var/log/journal` only when that directory
exists, and neither this script nor `install-pi.sh` creates it, so on a
stock image the journal lives *volatile* under `/run/log/journal`
(tmpfs) and `SystemMaxUse=` bounds nothing on the SD card until an
operator or a later image makes the journal persistent — which is
exactly why `health.HostHealthConfig.disk_free_min_gb`'s default treats
this cap as margin on top of the recorder-plus-model-store arithmetic,
never a term it depends on.

### udev

`deploy/99-aquacomputer.rules` → `/etc/udev/rules.d/` (`install-pi.sh`
reloads the rules and re-triggers `hidraw` and the `0c70` USB devices so
an attached device gets them without a re-plug):

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="0c70", MODE="0660", GROUP="plugdev"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0c70", MODE="0660", GROUP="plugdev"
ACTION=="add", SUBSYSTEM=="hwmon", ATTRS{idVendor}=="0c70", RUN+="/bin/sh -c 'for f in /sys%p/pwm*; do [ -e $f ] && chgrp plugdev $f && chmod g+w $f; done'"
```

The `hidraw` rule is the path the daemon reads and writes (group
`plugdev`, mode `0660`, *HID access* above); the `usb` rule serves
liquidctl and `lsusb -v`. The `hwmon` rule only matters for the optional
`aquacomputer_d5next` driver (not installed by `install-pi.sh`): it hands
the driver's `pwm*` attributes to `plugdev` with group write. Unverified
on a real device (§8 item 35).

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
   installs the apt packages (no kernel driver is built: the daemon uses
   hidraw, and the optional DKMS package is not run, §9), creates the
   install dir and the
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
   `--das`, the `deploy/aqua-bridge-das.conf` drop-in (`--source composite`,
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
   `mpc.sensors` / `mpc.topology` for the enclosure, the `aquacomputer:`
   devices (`device: aquaero` or `quadro`, `serial:` when several of one
   kind are attached, `fans` with `{pwm: pwmN, rpm: fanN}` per output (the
   Quadro on aquabus: the aquaero's `pwm5..pwm8` / `fan5..fan8`),
   `temp_map` per input actually present (`tempN` physical sensors; `busN`
   are the Quadro's sensors on aquabus — do not bind one until §8 item 92
   lands: the slot keeps its last value when the Quadro leaves the bus), the
   timing keys at their defaults
   unless §8 item 80 says otherwise; `heartbeat_sensor` matching the software
   sensor enabled on the controller, since 0 next to an enabled sensor spends
   its watchdog one timeout after boot, §8 item 84), `onewire.sensors`
   (step 9), MQTT
   host and credentials (the MQTT command topics rely on the broker's
   authentication, §7), `http.enabled` / `mqtt.enabled`. Every name must
   be bound exactly once (the daemon exits 2 otherwise). Legacy mode:
   `xt6.device`, `xt6.fans` and `xt6.temp_map` with exactly the keys of
   `mpc.channels` / `mpc.temps`. A config from before the hidraw adapter
   (`hwmon:`, `xt6.hwmon_name`) exits 2 with a message naming the
   replacement keys. Always pass `--config /etc/aqua-bridge/config.yaml`.
   **HTTPS users** (when `http.enabled: true`): `sudo
   /opt/aqua-bridge/.venv/bin/python /opt/aqua-bridge/tools/http_user.py
   --config /etc/aqua-bridge/config.yaml --group USER <name>` prompts for
   the password and writes `/etc/aqua-bridge/http-users` mode 640
   `root:USER`; rerun it to change a password or add a user (no restart).
   Without a certificate, key or user the API stays off and the journal
   says why; the fans are controlled regardless.
6b. Board hardening, once per card and independent of the hardware steps:
   `sudo deploy/install-board-watchdogs.sh` (`--check` first to see what it
   would change). SoC watchdog, journald caps, Wi-Fi power save off, and the
   Wi-Fi re-association timer — §9 *Board hardening*, §2 *Watchdog layering*.
   It enables `aqua-net-recover.timer` and nothing else; it never enables or
   starts `aqua-bridge`.
7. USB: dwc2 host, powered hub, XT6 on USB; the Quadro on aquabus (its PWM
   is writable through the aquaero, §2), or on its own USB port.
8. `lsusb`, then `.venv/bin/python tools/aquacomputer_probe.py` as the
   service user (read-only; no driver build and no `sensors`): each
   aquaero and Quadro with serial, USB interface and `/dev/hidrawN`, its
   temperatures, outputs (rpm, duty, voltage, current, power) and control
   settings — the spike questions of §2 (which Quadro temperature inputs
   exist, firmware revert) and the input names (`pwmN`, `fanN`, `tempN`,
   `busN`, `softN`, `virtN`) and `serial:` for the config; with the Quadro
   on aquabus its outputs show as the aquaero's `pwm5..pwm8`. A permission
   error means the hidraw udev rule has not applied (`ls -l /dev/hidraw*` must show group
   `plugdev` with read/write, §9 *HID access*). Run
   `.venv/bin/python -m pytest -m hardware` as the service user. Warm each mapped thermistor and watch the right `obs.temps` key
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
    `.venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml --source composite --once`
    (legacy: without `--source`, which defaults to `xt6`) reads every
    source, prints the observation and the command, applies it, and ends
    with the stop write: the fans are left at `fallback_pwm`, each aquaero
    channel assigned to its manual preset (§2).
11. DAS: the unit's `ExecStart=` has no `--source`, i.e. `xt6`. `--das`
    at step 5 already installed the `--source composite` drop-in
    (`deploy/aqua-bridge-das.conf`, §9); confirm with `systemctl cat
    aqua-bridge` (its `ExecStart=` lines show the override). Without
    `--das`, add it by hand: `sudo systemctl edit aqua-bridge` — an
    empty `ExecStart=` line, then the full `ExecStart=` line with
    `--source composite` — then `sudo systemctl daemon-reload` and `sudo
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
    enabled. **MQTT and Home Assistant**: `.venv/bin/python
    tools/ha_check.py --config /etc/aqua-bridge/config.yaml` (read-only; it
    exits non-zero when an expected entity is missing) prints which Discovery
    entities the broker holds, whether the availability and state topics carry
    what the code publishes, and what Home Assistant would show for each
    entity; `--verbose` lists them all (and the broker host, which belongs
    in `private.md` — the plain run is the one to paste anywhere), `--send`
    is the only way it ever publishes and it asks first, naming what stands
    after the message. Then the device page in Home Assistant
    (§8 item 21 says what to look at and what a failure would mean).
    Digole later.
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
# --store-out also writes the fit as a model store file the daemon loads (--model-store)
.venv/bin/python tools/fit_model.py --config config.example-das.yaml --topology \
  --out /tmp/model.json --store-out /tmp/model-store.json /tmp/rec.jsonl
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

The DAS bench on the Pi is the budget check of §3: p99 ≤ `mpc.budget_ms`
at `dt = 5 s` with `mpc_every_ticks: 2`, else apply the owner's fallback
(`budget_ms: 1000.0`, `budget_alarm_ms: 1250.0`, `mpc_every_ticks: 3`;
§8 item 73) and then the reductions listed there.

Rerun `deploy/install-pi.sh --user USER` on the Pi when `pyproject.toml`,
the unit or the udev rule changed. Or `git pull` from GitHub on the Pi.
Avoid committing from a Zero W (slow card, easy to mix up branches).

Running the daemon by hand on the Pi (stop the service first: two
processes would both write the controllers):

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
three PRs — `mpc-core` (contract, gate, PI and MPC, CI), `hw-xt6` (the
hwmon adapter, since replaced by the hidraw adapter), `glue` (loop,
supervisor, entry point, HTTP, MQTT, deploy) —
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
| `test (latest)` | push, PR, dispatch | Python 3.14, `pip install -e ".[dev,http,mqtt]"`, `HYPOTHESIS_PROFILE=ci`, `tools/ci_pytest_shards.py -- -m "not hardware and not nightly" --durations=15` | 15 min |
| `test (pi-parity)` | push, PR, dispatch | Python 3.13 with the Pi’s apt versions pinned (numpy 2.2.4, pyyaml 6.0.2, pytest 8.3.5, hypothesis 6.130.5, aiohttp 3.11.16, paho-mqtt 2.1.0), `pip install -e . --no-deps`, same sharded pytest | 15 min |
| `nightly-fuzz (latest, pi-parity)` | schedule, dispatch | same installs, `HYPOTHESIS_PROFILE=nightly` (randomized, 1000 examples), `tools/ci_pytest_shards.py -- -m "not hardware" --durations=15`: everything the PR jobs run plus the `nightly` sweeps | 90 min |

**Sharded PR test job (§8 item 26).** `tools/ci_pytest_shards.py` replaces
the single `pytest` invocation in `test (latest)` and `test (pi-parity)`:
it lists `tests/test_*.py`, bin-packs them onto `os.cpu_count()` shards
(greedy longest-processing-time-first, weighted by each file's line count
plus a flat bonus per `@given` — a property test runs many examples
through a closed-loop sim, so line count alone badly under-weighted
`test_mpc_fuzzy.py`; both are cheap stand-ins, no timing history is kept
or needed), and runs one plain `pytest <forwarded args> <shard's files>`
subprocess per shard concurrently, no plugin (`pytest-xdist` included).
It waits for every shard, prints each shard's file list up front and its
wall time at the end, and exits non-zero if any shard failed — the job
stays red exactly when an unsharded run would have. The assignment is
deterministic (same files, same weights, same buckets every run), so a
shard's failure reproduces locally with `pytest` on the same file list.
The suites needed no isolation changes: every shared fixture already
hands out a private `tempfile.mkdtemp()` directory or an OS-assigned port
(`aiohttp` `TestServer`, `AF_UNIX` sockets in short per-test temp dirs),
golden regeneration is opt-in only via `AQUA_BRIDGE_REGEN_GOLDEN=1` (never
set in CI), the `ci` Hypothesis profile runs `derandomize=True`, under
which Hypothesis does not use its on-disk example database, and
`tests/test_bench_budget.py`'s relative step-time gate was already
written to tolerate a slow shared runner (§12 budget gate paragraph
below) — so concurrent shards never touch the same file or destabilise
each other's timing. `nightly-fuzz` shards the same way: unsharded it took
51 min on 2026-09-16 and hit the old 60-minute limit on 2026-09-15 (both
matrix jobs killed), which is why it now runs through the same tool with a
90-minute limit as the backstop for a hung run rather than the expected
length.

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
the PR jobs reached 7–11+ min (`test (latest)`, runner speed varies) and
5–10 min (`test (pi-parity)`), at or past the 11-minute guideline (§8 item
26). Sharded (4 shards, `os.cpu_count()` on the GitHub-hosted runner): a
same-day before/after comparison measured `test (latest)` at 11m 13s
before and 6m 11s after (-45%), `test (pi-parity)` at 10m 3s before and
3m 42s after (-63%). A nightly failure prints a `@reproduce_failure`
blob in the log.

---

## 13. Rollout order (with parallelism)

1. **Done.** GitHub repo exists, public, protected `main` (§12).
   Site-specific inventory in `private.md`.
2. **In parallel:** the control core on the dev machine **and**, on the
   Pi, the USB spike (hub, XT6, HID reports, Quadro PWM and inputs, **firmware
   revert**) plus **DS18B20 and sensor commissioning** (`w1-gpio` buses,
   `tools/w1_commission.py --list / --identify / --check`, §10). Neither
   needs the other. Core **done** (legacy PI and MPC, and the whole DAS
   track: zones, estimator, PI-like DAS form, thermal model, DAS MPC,
   store, experiments; CI). Spike and commissioning **open**: the
   aquaero, the Quadro and the sensors are not connected.
3. Hardware adapters once the spike answers the Quadro PWM **and** revert
   questions. **Written**: the controllers over hidraw against captured
   reports (`hw/aquacomputer.py`, `hw/hidraw.py`,
   `hw/aquacomputer_adapter.py`, `hw/sources.py`), 1-Wire against the
   assumed `w1_therm` ABI and fake trees (`hw/onewire.py`); confirm on the
   devices, adjust the bindings or the adapters if the spike disagrees.
4. Glue loop, MQTT. **Code done**; Pi provisioned with `install-pi.sh`,
   sim smoke run and benchmark on the Zero W; service left disabled; no
   live broker yet.
5. HTTP API + HTML (same pages as Digole). **Done** (Host section and
   drive views gap, §8).
6. Hardware bring-up of the DAS: spike, hidraw permissions, `-m hardware`,
   bind every sensor, `--source composite` drop-in, DAS step budget on the
   Pi, enable the service, MQTT/HA live check, SMART agent on the PC.
7. **Model ladder** on the running enclosure, each stage ≥ 24 h unless
   stated, one change at a time:

   | Stage | Config | Decide by | Watch |
   |-------|--------|-----------|-------|
   | 0 commission + record | `solver: pi` (PI-like DAS), `zones.trust_rule: strict`, `record_path` | zero zone faults from sensors, bulk cycle time, every bay `occupied` / `empty` as physically true, no drive over its soft target | `/api/estimate`, `/api/bays`, `/api/health`, journal |
   | 1 offline fit + fan curves | – | `tools/fit_model.py`: `E` per group and per-bay `k` pinned (relative SE under `model_converged_rel_se`), `--store-out` seeding the model store (item 15); `tools/fit_fans.py` RMS < 5 % rpm, copied into `fan_models` (or `fan_curve_online` fitting them live, item 14); spare thermistors moved to the bays the fit ranks tightest | fit and replay reports |
   | 2 SMART calibration (if used) | agent on the PC | most bays `calibrated`, `σ_cal` ≤ 0.7 °C, calibrated estimates within 2 °C of SMART, associations match the physical bays | HA `drive_sigma_*`, `/api/model` calibration, `/api/bays` |
   | 3 shadow + experiments | `model_shadow: true`, `ident_enabled: true`, store on; experiments one group at a time under PI-DAS | every zone `converged`, prediction error < 0.5 °C, no experiment abort on the envelope | HA `model_status`, `model_pred_err_c`, `ident_running`, `/api/model` |
   | 4 MPC | `solver: mpc` | the validity gate keeps `active: mpc` (no model fallbacks), no `degraded`, `noise_db` lower than stage 0 at equal or better `drive_margin_*`, step p99 under budget | `/api/health`, `solver_diag.model` (its `checks`, `air_dist_c_per_min` for §8 item 98), `noise_db`, `drive_margin_*` |
   | 5 restart check | restart the daemon | `model.json` loads `fresh` and the zones `frozen`; after > `model_store_max_age_days` offline it loads `stale` and re-confirms in shadow before the MPC acts | `/api/model` `store`, journal |
   | 6 `trust_rule: sigma` | `zones.trust_rule: sigma` (§3 per-zone trust; not with two proximal sensors on a bay at different placements) | fewer zone faults than `strict`, no violation, the time from a lost sensor to its zone fault acceptable | zone graphs, `diagnostics.zones` reasons, `drive_sigma_*` |

   **Rollback**, one line each: `solver: pi` (PI-like DAS); `model_shadow:
   false`; `ident_enabled: false`; `zones.trust_rule: strict`; delete
   `$STATE_DIRECTORY/model.json` for a clean prior (the calibrations go
   with it). Removing `topology` returns to legacy mode only with a
   legacy-shaped config (setpoints, `xt6:`). Every restart passes the
   `fallback_pwm` stop write (about ten seconds of loud fans, accepted,
   §9).
8. Digole + touch.
9. **Done** (2026-09-16, §8 items 50, 51): Zero 2 W with no API change.
