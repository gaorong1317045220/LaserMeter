# 构建与发布验证

验证日期：2026-09-10

本次版本修复相机页倍率文字不同步和物理按键切换距离单位后回显旧值的问题，并新增厘米(cm)单位。测量、融合、标定、CSV 和历史记录仍使用原有毫米数据，不改变存储格式和计算精度。

## 验证结果

| 检查项 | 结果 | 证据 |
|---|---|---|
| UI 字体覆盖 | 通过 | `tools/check_ui_font_coverage.py`：178 个非 ASCII UI 字形全部存在，新增“厘”字可显示 |
| ESP32-S3 固件构建 | 通过 | ESP-IDF 5.5.2、Python 3.12.10，`tools/build.ps1` 完成编译、链接和分区检查 |
| 单位兼容性 | 通过 | 保留 NVS 值 `mm=0`、`m=1`，新增 `cm=2`；循环顺序为 `mm → cm → m → mm` |
| 显示入口检查 | 通过 | 单点测距、P2P、历史列表、历史详情和设置页统一走单位格式化逻辑 |
| Python 源码编译 | 通过 | `python -m compileall -q pc_app door_window_ai/src tools` |
| Python 依赖一致性 | 通过 | `pip check` 返回 `No broken requirements found` |
| Web 状态回归 | 通过 | `node tools/test_web_state.js` 返回 `WEB_STATE_REGRESSION_OK` |
| Windows PC 打包 | 通过 | PyInstaller 6.20.0 重新生成 `Release/pc/`，总大小约 387.1 MiB |
| 完整固件合并 | 通过 | Bootloader、分区表、应用程序按 `0x0`、`0x8000`、`0x10000` 合并，产物可从 `0x0` 烧录 |
| 发布哈希清单 | 通过 | `Release/SHA256SUMS.txt` 和 `Release/firmware/SHA256SUMS.txt` 已重新生成 |

## 固件产物

| 文件 | 字节数 | SHA-256 |
|---|---:|---|
| `build/bootloader/bootloader.bin` | 20,992 | `DEFADCC948230277B63B26AA650D893ECF8984DF38E4ECD143B6F04915E29F75` |
| `build/partition_table/partition-table.bin` | 3,072 | `5E71C3C890E04714DAA03B21A0D23953A2DADF9FDF15EEC02B886F7B6E9F8C81` |
| `build/board_self_test.bin` | 2,018,096 | `54768516578F834EEA4F09E8305B3946E7FE52B0E5B37A04182102F4F8F03ADD` |
| `Release/firmware/laser_meter_full.bin` | 2,083,632 | `8CC4F8D663CF54C1CCA47577BA8C6C8D3D72C9CC45734D28201C14C6852D5A37` |

应用程序占 2 MiB factory 分区约 96%，剩余 `0x134d0` 字节（约 4%）。构建成功，但继续增加字体、位图或功能时必须关注分区容量。

## 便捷发布包

- 目录：`Release/`
- 文件数：250（包括发布哈希清单）
- 总大小：约 390.1 MiB
- 完整固件：`Release/firmware/laser_meter_full.bin`
- 烧录芯片：ESP32-S3
- 烧录地址：`0x0`
- PC 端必须完整保留 `pc/_internal/` 和 `pc/vision/`，不能只复制 `LaserMeter.exe`

## 已知警告与待人工验收

1. 固件仍有两个原有编译警告：`upload_crc` 已赋值但未使用、`pc_send_file` 已定义但未使用。本次没有改变其协议行为。
2. 最新 UI 改动尚未在真实设备上人工验收。烧录后应检查相机页 `1X ↔ 2X` 画面和文字同步、触屏/物理按键单位循环、重启后的 cm 持久化，以及单点/P2P/历史页面显示。
3. 应用分区只剩约 4%，属于持续关注项。
4. ONNX 模型声明为 Ultralytics AGPL-3.0；商业再分发权利仍需项目所有者确认。
5. 默认演示密码 `12345678` 的正式产品策略和嘉立创硬件项目链接仍待发布者确认。
