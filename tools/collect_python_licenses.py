# SPDX-License-Identifier: MIT
"""Collect license files from the Python distributions bundled in the PC release."""

from __future__ import annotations

import argparse
import importlib.metadata
import shutil
from pathlib import Path


PACKAGES = (
    "flatbuffers",
    "numpy",
    "onnxruntime",
    "opencv-python",
    "packaging",
    "Pillow",
    "protobuf",
    "pyinstaller",
    "altgraph",
    "pefile",
    "pyinstaller-hooks-contrib",
    "pywin32-ctypes",
    "setuptools",
)


def is_license_file(relative_path: str) -> bool:
    normalized = relative_path.replace("\\", "/")
    path = Path(normalized)
    name = path.name.lower()
    if "__pycache__" in normalized.lower() or path.suffix.lower() in {".py", ".pyc", ".pyo", ".pyd"}:
        return False
    return (
        "license" in name
        or "copying" in name
        or "notice" in name
        or "/licenses/" in normalized.lower()
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)

    manifest: list[str] = []
    for package in PACKAGES:
        try:
            dist = importlib.metadata.distribution(package)
        except importlib.metadata.PackageNotFoundError:
            manifest.append(f"MISSING: {package}")
            continue

        package_dir = output / f"{dist.metadata['Name']}-{dist.version}"
        copied = 0
        for entry in dist.files or ():
            relative = str(entry)
            if not is_license_file(relative):
                continue
            source = Path(dist.locate_file(entry))
            if not source.is_file():
                continue
            destination = package_dir / Path(relative).name
            if destination.exists():
                destination = package_dir / relative.replace("/", "__").replace("\\", "__")
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
            copied += 1
        manifest.append(f"{dist.metadata['Name']}=={dist.version}: {copied} license file(s)")

    (output / "MANIFEST.txt").write_text("\n".join(manifest) + "\n", encoding="utf-8")
    print("\n".join(manifest))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
