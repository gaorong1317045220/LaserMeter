# 构建与发布验证

验证日期：2026-09-11

0.2.2 修复全量烧录、NVS 为空时，PSRAM 栈上的 `pc_link` 任务初始化 Wi-Fi 并写 NVS，触发 Flash Cache 安全断言后循环重启的问题。Wi-Fi 初始化、模式配置、启停和连接现统一由内部 RAM 栈上的 `wifi_ctrl` 任务执行；大型网络/JPEG/SD 处理仍保留在 PSRAM。本次并发加固为控制请求增加完成编号、Wi-Fi 启动状态使用原子变量、STA/AP netif 创建和状态查询加锁，避免超时请求串线及 UI/PC-link/串口同时操作 Wi-Fi。0.2.1 的相机倍率同步、厘米单位和物理按键单位修复保持不变。

## 验证结果

| 检查项 | 结果 | 证据 |
|---|---|---|
| UI 字体覆盖 | 通过 | `tools/check_ui_font_coverage.py`：178 个非 ASCII UI 字形全部存在，新增“厘”字可显示 |
| ESP32-S3 固件构建 | 通过 | ESP-IDF 5.5.2、Python 3.12.10，`tools/build.ps1` 完成编译、链接和分区检查 |
| Wi-Fi Flash 安全路径 | 通过 | `esp_wifi_init/set_config/start/stop/connect` 等控制调用仅位于内部栈 `wifi_ctrl` 分发函数；驱动配置使用 `WIFI_STORAGE_RAM` |
| Wi-Fi 并发路径 | 通过 | 控制任务单实例启动、请求完成 ID 校验、Wi-Fi 状态原子读写、STA/AP netif 创建及扫描/状态查询互斥 |
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
| `build/board_self_test.bin` | 2,020,176 | `CF6DEA4609B0F662D0D1CBFF5B06BBA9D7A1E1621350F5DB5C5EB27B1B3C7868` |
| `Release/firmware/laser_meter_full.bin` | 2,085,712 | `AB8EEA2EDEC9ADAE64BBF66588180B34179A8CA89B780E4A6C5415992692EAA3` |

应用程序占 2 MiB factory 分区约 96%，剩余 `0x13030` 字节（约 4%）。构建成功，但继续增加字体、位图或功能时必须关注分区容量。

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
2. Wi-Fi 栈修复已根据真实设备 Backtrace 定位并通过构建与静态调用路径检查，但修复后的固件仍需在真实设备上“完整擦除 Flash”后验证：启动不再重启、配对 AP 可用、STA 可连接且可上传。
3. 最新 UI 改动仍需检查相机页 `1X ↔ 2X` 画面和文字同步、触屏/物理按键单位循环、重启后的 cm 持久化，以及单点/P2P/历史页面显示。
4. 应用分区只剩约 4%，属于持续关注项。
5. ONNX 模型声明为 Ultralytics AGPL-3.0；商业再分发权利仍需项目所有者确认。
6. 默认演示密码 `12345678` 的正式产品策略和嘉立创硬件项目链接仍待发布者确认。
