# 智能量房仪（LaserMeter）

本仓库是“智能量房仪”的完整软件源码包，包含 ESP32-S3 固件、PC 端浏览器工作台、平面扫描与墙线提取算法、DXF 导出和门窗插座 ONNX 推理运行时。

硬件原理图、PCB、BOM、STEP/STL 等资料不在本软件仓库中，将单独发布到嘉立创开源硬件平台。

硬件项目链接：**待补充：嘉立创开源硬件项目 URL**

## 项目作用

设备通过激光测距、BNO086 姿态和 OV5640 相机采集房间几何及图像信息；PC 端接收设备数据，显示测量点与照片，提取墙线和墙角，辅助布置门、窗、插座并导出接近实际量房图纸的 DXF 文件。

## 已实现功能

- ESP32-S3 开机自检、LCD/LVGL 界面、触摸和实体按键输入。
- 单点测距、P2P 对测、平面扫描，以及测量记录保存、查看、删除和导出。
- OV5640 实时预览、1×/2× 画面、正式拍照与 JPEG 传输。
- BNO086 姿态读取、UART 激光测距模块通信、SD 卡存储。
- Wi-Fi 配网，以及设备与 PC 之间的 TCP 数据链路。
- PC 端 HTTP 工作台、TCP 接收、实时图像、点云和扫描数据解析。
- 中值滤波、RANSAC 墙线/墙角提取、墙体编辑和 DXF 导出。
- ONNX Runtime 门、窗、插座分割与四边形边缘细化。

## 软件架构

```text
ESP32-S3 固件（main/）
  传感器 / 显示 / 存储 / 网络
             │
             ├─ 测距、P2P、平面扫描、测量记录
             └─ TCP / JPEG / 扫描文件
                         │
PC 工作台（pc_app/） <───┘
  server.py
    ├─ scan_analysis.py
    ├─ wall_extraction.py
    ├─ Web 前端（pc_app/static/）
    └─ vision_worker.py ──> door_window_ai/（ONNX runtime）
```

## 源码完整性与目录

| 路径 | 内容 | 复现功能是否需要 |
|---|---|---|
| `main/` | ESP32-S3 固件源码、引脚、UI、标定和生成资源 | 必需 |
| `managed_components/` | 固定版本的 LVGL、相机和 JPEG 组件 | 必需，保留第三方许可证 |
| `pc_app/` | TCP/HTTP 服务、Web 工作台、扫描/墙线/DXF 和 AI worker | 必需 |
| `door_window_ai/src/` | 自研 AI 推理及后处理代码 | AI 功能必需 |
| `door_window_ai/models/` | ONNX 模型、类别和元数据 | 当前 AI 功能必需，模型单独授权 |
| `calibration_capture/` | 相机内参与激光—相机外参结果 | 当前准星投影必需 |
| `assets/`、`tools/` | UI 源素材、生成器、标定、构建、烧录与打包工具 | 从源码完整复现时保留 |
| `requirements-pc.lock.txt` | 已验证的 PC 运行依赖版本 | Python 方式运行必需 |

本包不包含 ESP-IDF 安装环境、Python 安装环境、训练数据和 AI 训练流水线。ESP-IDF 与 Python 依赖由使用者按自行安装。

## ESP32-S3 构建与烧录

要求 Windows、ESP-IDF 5.5.2、Python 3.12，以及 ESP-IDF 安装器提供的完整工具链。建议将仓库放在不含中文和空格的路径。

```powershell
# 在 ESP-IDF 5.5.2 PowerShell 中执行
.\tools\build.ps1

# 如未自动发现 IDF，可显式指定
.\tools\build.ps1 -IdfPath 'D:\Espressif\frameworks\esp-idf-v5.5.2'

# 连接与引脚配置匹配的设备后烧录
.\tools\flash.ps1 -Port COM27
```

将 `COM27` 替换为实际端口。`-EraseFlash` 会清除设备 NVS 中的配网和用户设置，只在确有需要时使用。构建产物位于 `build/`，该目录已被 `.gitignore` 排除。

## PC 端运行

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-pc.lock.txt
.\.venv\Scripts\python.exe pc_app\server.py
```

浏览器访问 `http://127.0.0.1:8000/`。设备数据接收端口为 TCP 8765，首次绑定目标端口为 TCP 8766。也可以使用 `pc_app\start.ps1`、`connect_device.ps1` 等脚本。

源码中保留了首次配网及演示热点使用的默认 SSID/密码（例如 `LASER-METER-SETUP`、`LASER-PC`、`ESP32-CAM-TEST` / `12345678`）。这些不是私人凭据，但公开部署前建议按实际网络安全要求修改。

## 便捷烧录与使用

为降低复刻门槛，发布目录同时提供合并固件和 Windows PC 程序：

```text
Release/
├─ firmware/
│  ├─ laser_meter_full.bin      # 从地址 0x0 烧录的完整 ESP32-S3 镜像
│  └─ SHA256SUMS.txt
├─ pc/
│  ├─ LaserMeter.exe            # 双击启动 PC 工作台
│  ├─ _internal/                # LaserMeter.exe 的运行依赖，不可删除
│  ├─ vision/                   # AI worker 及依赖，不可删除
│  └─ THIRD_PARTY_NOTICES.md
└─ README.md
```

PC 程序采用便携目录方式打包；请整体解压 `pc/`，不能只复制其中的 `.exe`。

维护者可在安装 `requirements-pc.lock.txt` 和 `requirements-build.lock.txt` 后运行以下命令重新生成整个 `Release/`：

```powershell
.\tools\package_release.ps1 -IdfPath 'D:\Espressif\frameworks\esp-idf-v5.5.2'
```

### 使用 ESP LAUNCHPAD 烧录

1. 使用支持数据传输的 USB 线连接设备。
2. 使用 Chrome 或 Edge 打开 [ESP LAUNCHPAD](https://espressif.github.io/esp-launchpad/)。
3. 连接 ESP32-S3 对应串口，选择本地固件 `Release/firmware/laser_meter_full.bin`。
4. 设置 `Chip: ESP32-S3`、`Offset: 0x0`，开始烧录。
5. 等待进度达到 100%；期间不要拔线、断电、关闭浏览器或手动复位。
6. 重启设备，等待屏幕、SD 卡、BNO086、相机和激光测距模块初始化。

完整镜像已经包含 Bootloader、分区表和应用程序，不需要分别烧录三段镜像。

### 启动 PC 程序

整体解压 `Release/pc/` 后双击 `LaserMeter.exe`，然后访问：

```text
http://127.0.0.1:8000/
```

## AI 模块说明

`door_window_ai/src/door_window_ai/` 是项目自研的 ONNX 推理运行时，采用 MIT License。`pc_app/vision_worker.py` 加载模型并完成门、窗、插座分割与轮廓后处理。

`door_window_ai/models/door_window_outlet_v3/model.onnx` 不是软件源码，不能由根目录 MIT 自动重新授权。其内嵌元数据声明 Ultralytics 8.4.118 / AGPL-3.0；发布模型及打包 AI 可执行程序前，项目所有者必须确认基础权重、训练数据及商业再分发权利。详见同目录的 `MODEL_LICENSE.md`。

## 标定说明

- `main/fusion_config.h`：机械安装、激光发射器与三脚架旋转轴偏移；当前值为实测值。
- `main/camera_calibration.h`：相机内参与畸变参数。
- `calibration_capture/`：相机内参和激光—相机外参结果。
- `tools/sync_laser_camera_calibration.py`：构建前将 JSON 标定结果同步到 `main/laser_camera_calibration.h`。

更换机械结构、传感器位置、镜头或激光安装角度后必须重新标定。标定值会影响准星投影、P2P 和房间扫描结果。

## AI 辅助编程说明

本项目软件部分大量采用 Vibe Coding / AI 辅助编程方式开发。代码已经结合实际硬件进行测试，但仍可能存在 Bug、异常处理不足或代码结构需要优化的问题。欢迎 Code Review、Issue、Pull Request 和功能建议。

## 许可证

- 项目自研的软件代码采用 [MIT License](LICENSE)，可自由学习、修改、分发及商业使用。
- 硬件设计、机械结构、README/项目教程以及作者自行制作的项目图片和示意图采用 CC BY-NC-SA 4.0；商业使用需要取得作者授权。
- `managed_components/`、Python 依赖、ONNX 模型及其他第三方材料保持原许可证，不因本项目采用 MIT 而被重新授权。详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

注意：固件和 PC 发布包中包含第三方组件及素材。商业分发完整成品时，除遵守 MIT 外，还必须同时满足所有随附内容的许可证；模型继续遵循其已有授权声明。
