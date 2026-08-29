# 第三方软件及独立授权内容

根目录 `LICENSE` 只授权项目作者拥有版权的自研软件代码，不改变第三方组件、模型、数据或素材的原许可证。各组件目录中的许可证正文和源码文件头是权威文本；发生差异时以原始文本为准。

## ESP32-S3 固件依赖

| 组件 | 已核对版本 | 许可证 | 核对依据 |
|---|---:|---|---|
| ESP-IDF | 5.5.2 | Apache-2.0；个别组件包含各自第三方条款 | ESP-IDF 5.5.2 根目录 `LICENSE` 及各组件许可证；工具链不随源码包附带 |
| LVGL | 8.4.0 | MIT | `managed_components/lvgl__lvgl/LICENCE.txt` |
| esp32-camera | 2.1.7 | Apache-2.0 | `managed_components/espressif__esp32-camera/LICENSE` |
| esp_jpeg | 1.3.1 | Apache-2.0，另含 ChaN TJpgDec 条款 | `managed_components/espressif__esp_jpeg/license.txt` 及 `tjpgd/` 源文件头 |
| esp_new_jpeg | 1.0.2 | ESPRESSIF MIT License（仅许可用于 ESPRESSIF SYSTEMS 产品） | `managed_components/espressif__esp_new_jpeg/LICENSE` |

`esp_new_jpeg` 的文本带有产品范围限制，因此不能简写为标准 SPDX `MIT`。当前目标硬件是 ESP32-S3，属于其许可证描述的 Espressif 产品范围。该组件包含预编译库 `managed_components/espressif__esp_new_jpeg/lib/esp32s3/libesp_new_jpeg.a`，源码包不能从其余文件重新构建该库；再分发时必须保留原许可证。

上述组件内部还包含带独立声明的上游代码，例如 esp32-camera 的 `jpge`、ChaN TJpgDec，以及 LVGL 中的 mpaland/printf、TLSF、LodePNG、qrcodegen、stb、Arm-2D、NXP PXP/VGLite 等。项目没有修改这些第三方声明。分发源码或固件时应保留组件目录中的许可证和源码文件头。

## PC 与 AI 的 Python 依赖

以下版本来自 `requirements-pc.lock.txt`，并已在本项目 Python 3.12 虚拟环境的 wheel 元数据/许可证文件中核对：

| 包 | 版本 | 已核对许可证/声明 |
|---|---:|---|
| flatbuffers | 25.12.19 | wheel 元数据声明 Apache-2.0 |
| numpy | 2.5.2 | BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0；详见 wheel 的 `dist-info/licenses/` |
| onnxruntime | 1.29.0 | MIT；另有 `onnxruntime/ThirdPartyNotices.txt` |
| opencv-python | 5.0.0.93 | wheel 元数据声明 Apache-2.0；其 `LICENSE.txt` 为包装项目 MIT，`LICENSE-3RD-PARTY.txt` 记录 OpenCV/FFmpeg 等内容 |
| packaging | 26.3 | Apache-2.0 OR BSD-2-Clause |
| Pillow | 12.3.0 | MIT-CMU；完整许可证集合见 wheel 的 `dist-info/licenses/LICENSE` |
| protobuf | 7.36.0 | BSD-3-Clause |

Windows 便携发布包使用 PyInstaller 6.20.0 构建。PyInstaller 本身为 GPL-2.0-or-later，并带允许分发包括商业程序在内的特殊 bootloader exception。构建依赖还包括 altgraph 0.17.5（MIT）、pefile 2024.8.26（MIT）、pyinstaller-hooks-contrib 2026.7（标准 hooks 为 GPL-2.0-or-later，打包进入程序的 runtime hooks 为 Apache-2.0）、pywin32-ctypes 0.2.3（BSD-3-Clause）和 setuptools 84.0.0（MIT）。发布脚本会从实际构建环境收集这些包及运行时 wheel 的许可证文件到 `Release/pc/licenses/python/`。

## ONNX 模型

`door_window_ai/models/door_window_outlet_v3/model.onnx` 不是项目自研软件源码，不适用根目录 MIT License。文件 SHA-256 为 `41C2F0C111450B44B4E6C443869C0DAFD81C997123DBD798BC6720B415984A07`，其内嵌元数据声明：

- `author=Ultralytics 8.4.118`
- `license=AGPL-3.0 License (https://ultralytics.com/license)`

项目没有修改或重新授权该模型。公开发布模型或包含它的 AI 可执行程序前，必须由项目所有者确认基础权重、训练数据、AGPL-3.0/商业许可路径以及商业再分发权利。详见 `door_window_ai/models/door_window_outlet_v3/MODEL_LICENSE.md`。

## 图片和项目素材

项目作者自行制作的图片、SVG 和示意图按 `assets/LICENSE.md` 使用 CC BY-NC-SA 4.0。素材目录中如含第三方商标或图片，该许可证不会替代原权利人的授权；其来源仍需项目所有者人工确认。

## 许可证边界结论

- MIT 与上述开源依赖本身不存在直接冲突，但分发者必须分别履行各依赖的保留声明义务。
- `model.onnx` 的 AGPL-3.0/商业授权状态与“完整软件可不受限制商业分发”的目标存在待确认项。
- CC BY-NC-SA 4.0 素材可与 MIT 代码放在同一仓库中分别授权，但包含这些素材的完整固件/程序不能仅按 MIT 理解；商业使用素材需要作者授权。
