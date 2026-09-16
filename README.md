# aqua-bridge

[![CI](https://github.com/nikkikom/aqua-bridge/actions/workflows/ci.yml/badge.svg?branch=main&event=push)](https://github.com/nikkikom/aqua-bridge/actions/workflows/ci.yml?query=branch%3Amain+event%3Apush)
[![nightly fuzz](https://img.shields.io/github/actions/workflow/status/nikkikom/aqua-bridge/ci.yml?event=schedule&label=nightly%20fuzz)](https://github.com/nikkikom/aqua-bridge/actions/workflows/ci.yml?query=event%3Aschedule)
![Python 3.13 | 3.14](https://img.shields.io/badge/python-3.13%20%7C%203.14-3776AB?logo=python&logoColor=white)

Quiet fan control for an air-cooled DAS enclosure: a Raspberry Pi keeps
every drive within its temperature limit with the least modelled fan
noise. Up to 15 hot-swap drives in zones, 8–10 fans driven through an Aqua
Computer aquaero 6 XT + Quadro, 24–30 temperature sensors next to the
drives and in the air stream (thermistor inputs and DS18B20 on 1-Wire),
optional SMART from the PC, Digole display, Home Assistant telemetry.

The controller estimates each drive's temperature from nearby sensors,
trusts and falls back per zone (a fault never reduces cooling), and runs
either a PI-like margin regulator or a noise-minimising MPC on an
identified zoned thermal model. Without the hardware everything runs
against a DAS simulator: start from `config.example-das.yaml`.
`config.example.yaml` is the legacy single-setpoint mode, kept bit for bit
as the reference for the safety core.

Full spec, TODO and rationale: **[PROJECT.md](PROJECT.md)**. Development
happens on a desktop or laptop (§11); the board is hardware and runtime
only.

Project language is English only (docs, comments, commit messages, issues).

## Bring up on a fresh Raspberry Pi

Condensed from PROJECT.md §10, which has the full rationale for each
step. There is no project-specific image. Steps 1–6 below have been run
on a Zero W; step 7 onward (USB spike, sensor commissioning, enabling
the service) needs the aquaero and is not yet confirmed on real
hardware.

1. **Flash.** Raspberry Pi Imager → Raspberry Pi OS Lite (32-bit for a
   Zero W; 32- or 64-bit for a Zero 2 W), Trixie. In the imager's
   customisation: a sudo user, SSH enabled, Wi-Fi SSID/password/country,
   timezone.

2. **Overlays.** Add to `/boot/firmware/config.txt` under `[all]`:

   ```text
   dtparam=i2c_arm=on
   dtparam=spi=on
   enable_uart=1
   dtoverlay=disable-bt
   dtoverlay=w1-gpio,gpiopin=4
   dtoverlay=w1-gpio,gpiopin=17
   ```

   (a third `dtoverlay=w1-gpio,gpiopin=27` only if one bus ends up
   carrying more than ~12 DS18B20 sensors). Load the I2C userspace
   module: `echo i2c-dev | sudo tee /etc/modules-load.d/i2c-dev.conf`.
   Drop `console=serial0,115200` from `/boot/firmware/cmdline.txt` if
   present (leaves the UART free for a Digole display). Then, for the
   USB host port (powered hub, aquaero on it):
   `deploy/host-usb.sh` (`--check` reports without changing anything;
   it backs up `config.txt` itself and says whether a reboot is
   needed). Reboot.

3. **Service account.** A dedicated Linux user in groups `sudo, gpio,
   i2c, spi, dialout, plugdev, netdev`, with an SSH public key in its
   `~/.ssh/authorized_keys`; optionally NOPASSWD sudo. This account must
   exist before step 5.

4. **Swap** (Zero W only — 512 MB RAM is tight for apt/pip builds). 1 GB
   via the `dphys-swapfile` package Raspberry Pi OS ships with:

   ```bash
   sudo dphys-swapfile swapoff
   sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=1024/' /etc/dphys-swapfile
   sudo dphys-swapfile setup
   sudo dphys-swapfile swapon
   ```

5. **Code and install.**

   ```bash
   sudo install -d -o <user> -g <user> /opt/aqua-bridge
   rsync -az ./ <user>@<pi-host>:/opt/aqua-bridge/          # or: git clone <fork-url> (no root needed after the chown)
   ssh <user>@<pi-host>
   cd /opt/aqua-bridge
   deploy/install-pi.sh --user <user>            # add --das for a DAS enclosure
   ```

   Idempotent; safe to rerun. It installs the apt packages
   (`deploy/packages-rpi.txt`, §9; no kernel driver is built — the
   daemon talks to the aquaero and the Quadro over hidraw, and the
   optional `deploy/install-aquacomputer-dkms.sh` is not run, PROJECT.md
   §9), creates `/opt/aqua-bridge` and a
   `--system-site-packages` venv, `pip install -e . --no-deps` as the
   service user (aborts if pip tries to fetch/build numpy instead of
   using apt's), installs `config.example.yaml`
   (`config.example-das.yaml` with `--das`) as
   `/etc/aqua-bridge/config.yaml` mode `640 root:<user>` **only if
   absent**, creates a self-signed HTTPS certificate
   `/etc/aqua-bridge/tls/cert.pem` (mode `644 root:root`) with key
   `key.pem` (mode `640 root:<user>`, valid for `--tls-days` days,
   default 3650) **only if neither file exists**, installs the
   udev rule (group `plugdev` read/write on the controllers' hidraw
   nodes) and re-triggers it, and installs the systemd unit with
   `User=` substituted — with `--das`, plus `deploy/aqua-bridge-das.conf`
   as the `aqua-bridge.service.d/das.conf` drop-in (adds `--source
   composite`). It reloads systemd and runs `systemd-analyze verify`, but
   does **not** enable, start, or create any HTTPS user — it prints the
   `tools/http_user.py` command to run next. If `/opt/aqua-bridge` has
   no `pyproject.toml` yet, the pip step is skipped with a warning
   (everything else — config, certificate, udev rule, unit — still
   runs); rerun the same command once the code is in place to pick up
   the pip install.

6. **Config.** Edit `/etc/aqua-bridge/config.yaml`: `mpc.channels` /
   `mpc.temps` / `mpc.sensors` / `mpc.topology` for the enclosure, the
   `aquacomputer:` list (one entry per controller: `device: aquaero` or
   `quadro`, `serial:` when several of one kind are attached, `fans`
   with `{pwm: pwmN, rpm: fanN}` per output (with the Quadro on the
   aquaero's aquabus, its outputs are the aquaero's `pwm5..pwm8` and
   `fan5..fan8`), `temp_map` per input actually present (`tempN`
   physical sensors, `busN` the aquaero's aquabus slots, `softN` software
   and `virtN` virtual sensors); the optional timing keys and the
   software-sensor heartbeat (`heartbeat_sensor`, `heartbeat_value_c`,
   off by default — 0 is right only while no software sensor is enabled
   on the controller, §8 item 84) are shown at their defaults,
   PROJECT.md §3 Track B; do **not** bind a `busN` input of a device that
   can leave the aquaero's aquabus yet: that slot keeps the last value it
   read instead of reading as missing, so the solver would follow a
   frozen temperature (§8 item 92)),
   `onewire.sensors`
   (bound in step 8), the MQTT host and credentials, `http.enabled` /
   `mqtt.enabled`. Every declared name must be bound exactly once — the
   daemon exits 2 otherwise. Legacy (no DAS sections): `xt6.device`,
   `xt6.fans` and `xt6.temp_map` with exactly the keys of
   `mpc.channels` / `mpc.temps`. A config from before the hidraw
   adapter (`hwmon:`, `xt6.hwmon_name`) exits 2 with a message naming
   the replacement keys, and so does an input name of the hwmon
   driver's numbering that now means another input (aquaero `temp9..20`,
   Quadro `temp5..20` and its flow sensor `fan5`), naming the new one.
   The aquaero's hwmon `fan5`/`fan6` were flow sensors and are now the
   tachometers of a Quadro on its aquabus; flow sensors cannot be bound.
   Always pass `--config /etc/aqua-bridge/config.yaml` explicitly.
   Without `--das` at step 5, install the DAS example by hand instead:
   `sudo install -m 640 -o root -g <user> config.example-das.yaml
   /etc/aqua-bridge/config.yaml`.

   **HTTPS users**, when `http.enabled: true`:

   ```bash
   sudo /opt/aqua-bridge/.venv/bin/python /opt/aqua-bridge/tools/http_user.py \
     --config /etc/aqua-bridge/config.yaml --group <user> <name>
   ```

   prompts for the password (never on the command line) and writes
   `/etc/aqua-bridge/http-users` mode `640 root:<user>`; rerun to add a
   user or change a password (no restart needed). Without a certificate,
   key or user the API stays off (logged) and the fans are still
   controlled.

7. **USB hardware.** dwc2 host + powered hub, XT6 on USB; the Quadro on
   aquabus (its PWM is writable through the XT6, §2), or on its own USB
   port. `lsusb`, then, as the service user,
   `.venv/bin/python tools/aquacomputer_probe.py` — read-only: lists
   each aquaero / Quadro (serial, USB interface, `/dev/hidrawN`) with
   its temperatures by group, outputs (rpm, duty, voltage, current,
   power; the aquaero's 5–8 are the Quadro's on aquabus) and control
   settings; pick the input names and `serial:` from it. The daemon's
   writes take effect at once and are not saved in the controllers'
   memory: after a power cycle they run their saved configuration. A
   permission error means the udev rule has not applied
   (`ls -l /dev/hidraw*` must show group `plugdev` with read/write).
   For the aquaero's own watchdog, configure a software sensor on the
   controller (enabled, a timeout of a few tens of seconds, a fallback
   temperature above every alarm threshold) with an alarm that selects a
   safe profile, in aquasuite, then set `heartbeat_sensor` to that
   sensor: the daemon writes it on every tick whose duties reached the
   controller (and on no other), so a daemon or Pi that stops — or one
   that runs but can no longer write — leaves the controller to run the
   safe profile
   (PROJECT.md §2 "Software-sensor heartbeat and profiles", §8 item 84).
   Set the key in the same step as the sensor: a sensor enabled on the
   device with `heartbeat_sensor: 0` falls back and fires the alarm one
   timeout after boot, for good. Then, with the service **stopped**,
   commission the controller's own configuration — the one it falls back
   to — so that a power cycle brings that back. What the tool sees is
   never the daemon's live duties: the daemon restores every control
   field it wrote when it stops (`release()`), and the software-sensor
   heartbeat times out a few tens of seconds later, so the controller is
   running its own saved profile (or the safe profile the alarm selected)
   by the time the tool can read it. Check that what it prints is the
   safe configuration — temperature-driven sources, sane min/max, the
   right profile — and only then save it:

   ```bash
   .venv/bin/python tools/aquacomputer_commission.py \
     --config /etc/aqua-bridge/config.yaml --device aquaero
   ```

   shows every output's duty, source and limits and the active profile
   without writing anything; add `--save` to store it, after a typed
   confirmation and a re-read that aborts when the controller changed
   while you were reading (PROJECT.md §8 item 88; run once per
   controller, so `--device quadro` too when one is configured — its save
   report is not verified to persist, unlike the aquaero's, §8 item 96).
   As the
   service user:
   `HYPOTHESIS_PROFILE=pi .venv/bin/python -m pytest -m hardware`. Warm
   each mapped thermistor and confirm the right `obs.temps` key moves (a
   swapped `temp_map` is invisible to the safety gate).

8. **1-Wire sensor commissioning** (DS18B20, once, before the daemon
   runs, with the `w1-gpio` overlays from step 2 active), from
   `/opt/aqua-bridge`:

   ```bash
   .venv/bin/python tools/w1_commission.py --list                          # every ROM id per bus, current reading
   .venv/bin/python tools/w1_commission.py --identify                      # warm one sensor at a time, ranked by warming rate
   .venv/bin/python tools/w1_commission.py --check --config /etc/aqua-bridge/config.yaml
   ```

   `--identify` is how each ROM id gets a name (`prox_b01`, `inlet_b`,
   …) in `onewire.sensors` — repeat per sensor. `--check` builds the
   exact composite the daemon would (every name bound once, every ROM
   present) and reports the bulk-read cycle time per bus and the CRC
   error rate per sensor over 20 cycles: aim for < 1 % and a cycle under
   `0.4 × dt`.

9. **Diagnostic tick**, as the service user:

   ```bash
   .venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml --source composite --once
   ```

   (legacy: drop `--source`, which defaults to `xt6`). Reads every
   source, prints the observation and the command, applies it once, and
   ends with the stop write: fans left at `fallback_pwm`, each aquaero
   channel assigned to its manual preset (PROJECT.md §2).

10. **DAS unit check.** `--das` at step 5 already installed the
    `--source composite` drop-in; confirm with `systemctl cat aqua-bridge`
    (its `ExecStart=` lines show the override). Without `--das`, add it
    by hand: `sudo systemctl edit aqua-bridge` (an empty `ExecStart=`
    line, then the full line with `--source composite`), then `sudo
    systemctl daemon-reload` and `sudo systemd-analyze verify
    /etc/systemd/system/aqua-bridge.service`.

11. **Enable and verify:**

    ```bash
    sudo systemctl enable --now aqua-bridge
    journalctl -u aqua-bridge -f
    curl -k -u <name> https://<pi-host>:8443/api/health      # -k: self-signed certificate
    curl -k -u <name> https://<pi-host>:8443/api/estimate    # DAS only: every bay occupied/empty as physically true, no zone in fault
    ```

    A start-timeout loop in the journal means the device or its
    permissions are missing. With `mqtt.enabled`, check the broker and
    Home Assistant next:

    ```bash
    .venv/bin/python tools/ha_check.py --config /etc/aqua-bridge/config.yaml
    ```

    Read-only (it publishes only with `--send`, which names the command,
    says what stands after it and asks first) and driven by the same config
    the daemon uses: it lists which Discovery entities the broker holds,
    which expected ones are missing, whether the availability and state
    topics carry what the code publishes and what Home Assistant would show
    for each entity (`--verbose` for all of them, and for the broker host,
    which the plain run keeps out of its output). It exits non-zero when
    something expected is missing. Then the device page in Home Assistant.

    `/api/health`'s `device_health` should be `{"ok": true, "problems":
    []}`; the page's **Controllers** section (and `/api/state`'s
    `device_health`) shows the same thing per controller and per fan —
    stuck or absent outputs, outputs not in PWM mode, the rail voltage,
    current and power, and rpm against the fitted fan curve. In Home
    Assistant it is the `Controller problem` binary sensor. The drift
    thresholds are the `fan_health:` section of `config.yaml`; the rpm rule
    needs an `mpc.fan_models` curve (fit one with `tools/fit_fans.py` from a
    recording) and the power rule needs `fan_models.<m>.power_w_at_max`,
    without which it stays off.

    The Raspberry Pi itself is watched the same way (`host_health:`, §8
    item 97): `/api/state`'s `device_health.host` and the page's **Host**
    section show the board's temperature against the enclosure air, whether
    its CPU was idle when that was judged, and the decoded
    `get_throttled` word (throttling now, and what has occurred since boot);
    in Home Assistant it is the `Board problem` binary sensor. The board is
    a health signal only — it is never a solver input, a zone air sensor or
    a model node — and the divergence rule is a hint about the air sensors
    or the board's placement, not a verdict about either: it shows on the
    board's own verdict and its `Board problem` sensor, and deliberately
    does *not* make `/api/health` not-ok or turn on `Controller problem`,
    which a hot or throttling board (a fact, not a hint) does. Its default
    `divergence_c` is a coarse backstop — an idle Zero 2 W already sits
    20–25 °C above the air around it — so narrow it only from a measured
    board-vs-air delta on the board in its finished place.

12. **Optional SMART agent**, on the PC with the drives attached (not
    the Pi): `python tools/smart_agent.py --mqtt <broker-host>
    --node-id aqua-bridge --interval 60`, or install
    `deploy/aqua-bridge-smart-agent.service` as a user unit (its header
    comment has the steps). `--node-id` must match `mqtt.node_id`.

Zero W is 32-bit only; do not install a desktop. The Pi should be
reachable on the LAN (mDNS `<pi-host>.local` or DHCP name).
