# 智能量房仪便捷发布包

## 固件烧录

使用 Chrome/Edge 打开 <https://espressif.github.io/esp-launchpad/>，连接 ESP32-S3 后选择：

- 固件：`firmware/laser_meter_full.bin`
- 地址：`0x0`
- 芯片：ESP32-S3

等待进度达到 100% 后重新启动设备。烧录期间不要拔线、断电、关闭浏览器或手动复位。

## PC 工作台

完整解压 `pc/` 目录并双击 `LaserMeter.exe`，然后访问 <http://127.0.0.1:8000/>。

`_internal/` 和 `vision/` 是运行依赖，不可删除，也不要只复制 `.exe`。

## 许可证

项目自研软件代码采用 MIT License。第三方组件、ONNX 模型、字体和素材保持各自许可证或授权状态，详见发布目录和源码根目录中的 `THIRD_PARTY_NOTICES.md`。
