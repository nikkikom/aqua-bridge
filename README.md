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

Spec, TODO, Pi packages, new-Pi checklist, GitHub/`gh` setup:
**[PROJECT.md](PROJECT.md)**

Development happens on a desktop or laptop. The board is hardware and runtime only.

Project language is English only (docs, comments, commit messages, issues).
