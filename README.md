# aqua-bridge

[![CI](https://github.com/nikkikom/aqua-bridge/actions/workflows/ci.yml/badge.svg?branch=main&event=push)](https://github.com/nikkikom/aqua-bridge/actions/workflows/ci.yml?query=branch%3Amain+event%3Apush)
[![nightly fuzz](https://img.shields.io/github/actions/workflow/status/nikkikom/aqua-bridge/ci.yml?event=schedule&label=nightly%20fuzz)](https://github.com/nikkikom/aqua-bridge/actions/workflows/ci.yml?query=event%3Aschedule)
![Python 3.13 | 3.14](https://img.shields.io/badge/python-3.13%20%7C%203.14-3776AB?logo=python&logoColor=white)

Quiet fan control for an air-cooled DAS enclosure: a Raspberry Pi keeps
every drive within its temperature limit with the least fan noise.
Up to 15 hot-swap drives in zones, 8–10 fans driven through an Aqua
Computer aquaero 6 XT + Quadro, many temperature sensors next to the
drives and in the air stream, Digole display, Home Assistant telemetry.

The implemented core (sensor gate, safe fallback, PI and MPC solvers,
hwmon adapter, HTTP/MQTT, deploy) still uses example names from an
earlier watercooling draft; the DAS-specific parts are listed in
PROJECT.md §8 Track A2.

Spec, TODO, Pi packages, new-Pi checklist, GitHub/`gh` setup:
**[PROJECT.md](PROJECT.md)**

Development happens on a desktop or laptop. The board is hardware and runtime only.

Project language is English only (docs, comments, commit messages, issues).
