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
   (`deploy/packages-rpi.txt`, §9), builds the `aquacomputer_d5next`
   hwmon driver with DKMS (`deploy/install-aquacomputer-dkms.sh`; the
   Raspberry Pi OS kernel does not include it, PROJECT.md §9), creates `/opt/aqua-bridge` and a
   `--system-site-packages` venv, `pip install -e . --no-deps` as the
   service user (aborts if pip tries to fetch/build numpy instead of
   using apt's), installs `config.example.yaml`
   (`config.example-das.yaml` with `--das`) as
   `/etc/aqua-bridge/config.yaml` mode `640 root:<user>` **only if
   absent**, creates a self-signed HTTPS certificate
   `/etc/aqua-bridge/tls/cert.pem` (mode `644 root:root`) with key
   `key.pem` (mode `640 root:<user>`, valid for `--tls-days` days,
   default 3650) **only if neither file exists**, installs the
   udev rule and re-triggers it, and installs the systemd unit with
   `User=` substituted — with `--das`, plus `deploy/aqua-bridge-das.conf`
   as the `aqua-bridge.service.d/das.conf` drop-in (adds `--source
   hwmon`). It reloads systemd and runs `systemd-analyze verify`, but
   does **not** enable, start, or create any HTTPS user — it prints the
   `tools/http_user.py` command to run next. If `/opt/aqua-bridge` has
   no `pyproject.toml` yet, the pip step is skipped with a warning
   (everything else — config, certificate, udev rule, unit — still
   runs); rerun the same command once the code is in place to pick up
   the pip install.

6. **Config.** Edit `/etc/aqua-bridge/config.yaml`: `mpc.channels` /
   `mpc.temps` / `mpc.sensors` / `mpc.topology` for the enclosure, the
   `hwmon:` section (`fans` with `{pwm: pwmN, rpm: fanN}` per output,
   `temp_map` per thermistor input actually present), `onewire.sensors`
   (bound in step 8), the MQTT host and credentials, `http.enabled` /
   `mqtt.enabled`. Every declared name must be bound exactly once — the
   daemon exits 2 otherwise. Legacy (no DAS sections): `xt6.fans` and
   `xt6.temp_map` with exactly the keys of `mpc.channels` / `mpc.temps`.
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
   aquabus, or its own USB port if its PWM is not writable through the
   XT6 (§2). `lsusb`, `sensors` — the USB spike questions (attribute
   names/units, Quadro PWM writability, firmware revert behaviour).
   `ls -l /sys/class/hwmon/hwmon*/pwm*` must show group `plugdev` with
   write. As the service user:
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
   .venv/bin/python -m aqua_bridge --config /etc/aqua-bridge/config.yaml --source hwmon --once
   ```

   (legacy: drop `--source`, which defaults to `xt6`). Reads every
   source, prints the observation and the command, applies it once, and
   ends with the stop write: fans left at `fallback_pwm` with
   `pwmK_enable` in manual mode.

10. **DAS unit check.** `--das` at step 5 already installed the
    `--source hwmon` drop-in; confirm with `systemctl cat aqua-bridge`
    (its `ExecStart=` lines show the override). Without `--das`, add it
    by hand: `sudo systemctl edit aqua-bridge` (an empty `ExecStart=`
    line, then the full line with `--source hwmon`), then `sudo
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
    permissions are missing. Then check MQTT entities in Home Assistant
    if `mqtt.enabled`.

12. **Optional SMART agent**, on the PC with the drives attached (not
    the Pi): `python tools/smart_agent.py --mqtt <broker-host>
    --node-id aqua-bridge --interval 60`, or install
    `deploy/aqua-bridge-smart-agent.service` as a user unit (its header
    comment has the steps). `--node-id` must match `mqtt.node_id`.

Zero W is 32-bit only; do not install a desktop. The Pi should be
reachable on the LAN (mDNS `<pi-host>.local` or DHCP name).
