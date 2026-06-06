#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
download_weights.py
-------------------
下载 UFLDv2 的 ONNX 权重到本包 weights/ 目录。

权重来源：PINTO model zoo (324_Ultra-Fast-Lane-Detection-v2)，
该仓库提供已从官方 PyTorch 权重导出的 ONNX 模型，便于用 onnxruntime
直接推理，避免对训练仓库与特定 torch 版本的强依赖。

用法:
  rosrun race_lane_detection download_weights.py            # 默认 res34 culane
  rosrun race_lane_detection download_weights.py --model res18

下载后在 config/params.yaml 中将 onnx_path 设为输出路径并把
detector_backend 设为 "learning" 或 "auto"。
"""

import argparse
import os
import sys

try:
    import rospkg
    _PKG_DIR = rospkg.RosPack().get_path("race_lane_detection")
except Exception:
    _PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

WEIGHTS_DIR = os.path.join(_PKG_DIR, "weights")

# PINTO model zoo 直链(若失效请参考下方仓库页手动获取):
#   https://github.com/PINTO0309/PINTO_model_zoo/tree/main/324_Ultra-Fast-Lane-Detection-v2
PINTO_DOWNLOAD_SH = ("https://github.com/PINTO0309/PINTO_model_zoo/blob/main/"
                     "324_Ultra-Fast-Lane-Detection-v2/download.sh")

MODELS = {
    "curvelanes_res34": "ufldv2_curvelanes_res34_800x1600.onnx",
    "curvelanes_res18": "ufldv2_curvelanes_res18_800x1600.onnx",
    "res18": "ufldv2_culane_res18_320x1600.onnx",
    "res34": "ufldv2_culane_res34_320x1600.onnx",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", choices=list(MODELS.keys()), default="curvelanes_res34")
    ap.add_argument("--url", default="",
                    help="直接提供 onnx 下载直链(覆盖默认)")
    args = ap.parse_args()

    os.makedirs(WEIGHTS_DIR, exist_ok=True)
    fname = MODELS[args.model]
    out_path = os.path.join(WEIGHTS_DIR, fname)

    if os.path.isfile(out_path):
        print("[download_weights] 已存在: %s" % out_path)
        print("[download_weights] 在 params.yaml 设置 onnx_path: %s" % out_path)
        return

    if not args.url:
        print("=" * 72)
        print("未提供直链。PINTO model zoo 使用 Google Drive 分发，建议执行其官方脚本：")
        print("  git clone https://github.com/PINTO0309/PINTO_model_zoo")
        print("  cd PINTO_model_zoo/324_Ultra-Fast-Lane-Detection-v2")
        print("  ./download.sh")
        print("然后将 %s 拷贝到:" % fname)
        print("  %s" % out_path)
        print("脚本说明页: %s" % PINTO_DOWNLOAD_SH)
        print("=" * 72)
        print("或用 --url <直链> 让本脚本直接下载。")
        sys.exit(1)

    try:
        import urllib.request
        print("[download_weights] 下载 %s -> %s" % (args.url, out_path))
        urllib.request.urlretrieve(args.url, out_path)
        print("[download_weights] 完成。在 params.yaml 设置 onnx_path: %s"
              % out_path)
    except Exception as e:
        print("[download_weights] 下载失败: %s" % e)
        sys.exit(1)


if __name__ == "__main__":
    main()
