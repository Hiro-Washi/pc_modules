#!/usr/bin/env python3

import open3d #ライブラリのインポート
import numpy as np

pcd = open3d.io.read_point_cloud("human-skeleton.ply") #"bun.pcd") #点群ファイルを変数
open3d.visualization.draw_geometries([pcd]) #点群を画像として表示
cps = np.asarray(pcd.points)
open3d.io.write_point_cloud("output.pcd", pcd) #output.pcbという名前の点群ファイルを出力

