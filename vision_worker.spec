# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_submodules

hiddenimports = ['onnxruntime']
hiddenimports += collect_submodules('door_window_ai')


a = Analysis(
    ['pc_app\\vision_worker.py'],
    pathex=['door_window_ai\\src'],
    binaries=[],
    datas=[('door_window_ai\\models\\door_window_outlet_v3', 'door_window_ai\\models\\door_window_outlet_v3')],
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['ultralytics', 'torch', 'torchvision', 'tensorflow', 'tensorboard', 'matplotlib', 'lap', 'onnx', 'onnxruntime.transformers', 'onnxruntime.tools', 'onnxruntime.quantization', 'tkinter'],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='vision_worker',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='vision_worker',
)
