# 门窗插座 AI 运行时

本目录只包含智能量房仪 PC 端实际使用的 ONNX 推理链路，不包含训练程序、训练数据、数据清洗/增强、评估、报告或训练日志。

```text
door_window_ai/
├─ models/door_window_outlet_v3/
│  ├─ model.onnx
│  ├─ classes.json
│  ├─ model_metadata.json
│  └─ MODEL_LICENSE.md
├─ src/door_window_ai/
│  ├─ inference/
│  ├─ refinement/
│  └─ schemas/
├─ DATA_LICENSES.md
└─ pyproject.toml
```

实际入口是上级目录的 `pc_app/vision_worker.py`。它会加载内置 ONNX 模型，输出门、窗、插座的检测掩膜，并通过 `quad_edges.py` 提取四边形边缘。

运行依赖统一由项目根目录的 `requirements-pc.lock.txt` 安装。单独安装本包可以使用：

```powershell
python -m pip install -e .
```

本目录中项目自研的 AI runtime 代码采用 MIT License。`model.onnx` 是独立模型资产，不随 runtime 自动获得 MIT 授权；其元数据显示为 Ultralytics 8.4.118 / AGPL-3.0。发布前必须由项目所有者确认模型、基础权重和训练数据的再分发及商业权利，详见 `models/door_window_outlet_v3/MODEL_LICENSE.md`。
