# 构建与发布验证

验证日期：2026-08-29

本次只整理许可证、版权声明、构建/发布工具和文档，没有修改测距、P2P、扫描、墙线、DXF、UI、AI 模型、标定参数或 GPIO 定义。

## 验证结果

| 检查项 | 结果 | 证据 |
|---|---|---|
| 固件源码完整性 | 通过 | `main/CMakeLists.txt` 登记的 16 个编译源文件全部存在；完整 ESP-IDF 构建通过 |
| ESP32-S3 固件构建 | 通过 | ESP-IDF 5.5.2、Python 3.12.10；`tools/build.ps1` 完成链接和分区检查 |
| 构建脚本独立性 | 通过 | 在系统默认 Python 3.14 的 PowerShell 中，仅传 `-IdfPath` 即能自动发现现有 ESP-IDF Python 3.12 环境 |
| Python 源码编译 | 通过 | `python -m compileall -q pc_app door_window_ai/src tools` |
| Python 依赖一致性 | 通过 | `pip check` 返回 `No broken requirements found` |
| 源码版 PC 服务 | 通过 | 首页、`calib.html`、`/api/status` 均返回 HTTP 200 |
| 源码版 ONNX 推理 | 通过 | `vision_worker.py` 成功加载 `model.onnx` 并对测试图片返回 `ok: true` |
| Windows PC 打包 | 通过 | PyInstaller 6.20.0 生成便携目录；最终大小约 387.1 MiB |
| 打包版 PC 服务 | 通过 | `Release/pc/LaserMeter.exe` 的首页、标定页和状态 API 均返回 HTTP 200 |
| 打包版 AI worker | 通过 | `Release/pc/vision/vision_worker.exe` 成功加载随包模型并返回 `ok: true` |
| Web 状态回归 | 通过 | `node tools/test_web_state.js` 验证新建清空、旧请求失效、历史扫描替换和手工墙保留 |
| 完整固件合并 | 通过 | Bootloader、分区表、应用程序分别与合并镜像 `0x0`、`0x8000`、`0x10000` 内容逐字节一致 |
| 许可证残留扫描 | 通过 | 自研源码中未发现旧许可证正文或 Apache-2.0 SPDX 残留；第三方 Apache 声明按原样保留 |
| 临时缓存 | 通过 | 验证生成的源码目录 `__pycache__` 已清除，构建/虚拟环境目录由 `.gitignore` 排除 |

## 固件产物

| 文件 | 字节数 | SHA-256 |
|---|---:|---|
| `build/bootloader/bootloader.bin` | 20,992 | `DEFADCC948230277B63B26AA650D893ECF8984DF38E4ECD143B6F04915E29F75` |
| `build/partition_table/partition-table.bin` | 3,072 | `5E71C3C890E04714DAA03B21A0D23953A2DADF9FDF15EEC02B886F7B6E9F8C81` |
| `build/board_self_test.bin` | 2,017,312 | `C05080DE03F368D32F48679528F77245949B4F02417A98DBA8482854AB6BCA4D` |
| `Release/firmware/laser_meter_full.bin` | 2,082,848 | `61545D8FFC32194AC55E9F5B3320DDFF0B6C9265A4645F6FAF8541C378B234D8` |

应用程序占 2 MiB factory 分区约 96%，剩余 `0x137e0` 字节（约 4%）。构建仍然成功，但继续增加字体、位图或功能前必须关注分区容量。

当前应用程序 SHA-256 与此前已烧录到 COM27 并通过启动日志验证的镜像一致，因此本次纯许可证/发布整理没有再次改写设备 Flash。此前启动日志已确认应用从 `0x10000` 正常加载、8 MiB PSRAM 测试通过、OV5640 初始化成功。

## 便捷发布包

- 目录：`Release/`
- 文件数：258
- 总大小：约 390.3 MiB（包含完整许可证清单）
- 全量文件哈希：`Release/SHA256SUMS.txt`
- 完整固件按地址 `0x0` 烧录。
- PC 端为便携目录，必须保留 `_internal/` 和 `vision/`，不能只复制 `LaserMeter.exe`。

## 已知警告与人工验收

1. 固件编译有两个原有警告：`upload_crc` 未使用、`pc_send_file` 未使用；不影响本次构建，但后续可在不改变协议行为的前提下清理。
2. 自动化验证覆盖构建、服务启动、HTTP/API、模型加载、一次推理和合并镜像结构；真实房间中的测距精度、P2P、完整扫描、门窗插座拖放、DXF 尺寸及长时间稳定性仍需在实机上人工走查。
3. `model.onnx` 元数据声明 Ultralytics AGPL-3.0，模型及打包 AI 程序的商业再分发权利需项目所有者确认。
4. 默认配网/演示密码 `12345678` 不是私人密码，但正式产品应评估是否更换。
5. 嘉立创硬件项目链接仍需发布者填写确认；许可证作者署名已确认为 GitHub 用户名 `gaorong1317045220`。
