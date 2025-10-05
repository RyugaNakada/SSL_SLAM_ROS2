#!/usr/bin/env python3
import open3d as o3d
import os

# データセットのダウンロード
dataset_path = o3d.data.L515Bag()
print(f"Dataset downloaded to: {dataset_path.path}")

# ファイル確認
import glob
bag_files = glob.glob(os.path.join(dataset_path.path, "*.bag"))
print(f"Found bag files: {bag_files}")
