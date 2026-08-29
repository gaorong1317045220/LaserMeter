# Project Instructions

## Project

This is the minimal source-only handoff for an ESP32-S3 laser room-measurement device and its Python PC web/ONNX companion.

## Run

- Firmware: open an ESP-IDF 5.5.2 PowerShell and run `./tools/build.ps1`.
- Flash: run `./tools/flash.ps1 -Port COMx` only with matching hardware.
- PC service: install `requirements-pc.lock.txt`, then run `python pc_app/server.py`.

## Stack

- ESP-IDF 5.5.2, C/C++, LVGL 8.4.0, ESP32-S3.
- Python 3.12, standard-library HTTP/TCP service, OpenCV and ONNX Runtime.
- Browser UI is plain HTML/CSS/JavaScript under `pc_app/static/`.

## Layout and conventions

- `main/` is the firmware component; pin truth is `main/pin_config.h`.
- UI C files are generated from `assets/` by scripts in `tools/`; keep generators and outputs synchronized.
- Vendored firmware dependencies live in `managed_components/`; this release intentionally contains only ESP32-S3 build inputs.
- `door_window_ai/` is runtime-only. Training, dataset, evaluation and test modules are intentionally absent.
- Keep the project root ASCII-only on Windows.
- Never commit `build/`, `sdkconfig`, virtual environments, PC runtime data, datasets or training runs. `Release/` is an intentional generated publication artifact and is rebuilt with `tools/package_release.ps1`.

## Current state

Project-authored software code uses MIT; project-authored documentation and artwork use CC BY-NC-SA 4.0. Third-party components retain their original licenses. The ONNX model is separate and keeps its embedded Ultralytics AGPL-3.0 declaration. `tools/package_release.ps1` produces the tested `Release/` directory, including the offset-0 merged firmware and portable PC app. The 2026-08-29 build/package checks passed; see `BUILD_VERIFICATION.md`. Before publication, the owner still needs to confirm model/material rights, decide the default Wi-Fi credential policy, and add the hardware project link.
