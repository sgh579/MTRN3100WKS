# tools/safe_zone.pixel_y
import cv2
import numpy as np
from pathlib import Path
from typing import Tuple, Optional
import math
import os
from tools.User_Configuration import IMAGE_FOLER
from tools.grid_graph import Grid_Graph

def draw_graph_on_image(
    graph,
    image_path: str,
    out_path: str,
    node_radius: int = 8,
    edge_thickness: int = 2,
    node_bgr=(0, 0, 255),   # 红色（注意 OpenCV 使用 BGR）
    edge_bgr=(0, 255, 0),   # 绿色
    show_ids: bool = True,
    font_scale: float = 0.5,
):
    """
    在 image_path 指定的图片上，把 graph 的节点/边画出来并保存到 out_path。
    - graph.nodes: {node_id: Node(...)}，Node.pixel_x/py 为像素坐标（左上角原点）
    - graph.edges: {u: {v: weight, ...}, ...}（无向图：两侧都存）
    """
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {image_path}")

    H, W = img.shape[:2]

    # ---------- 画边（只画一次：u < v） ----------
    for u, nbrs in graph.edges.items():
        u_node = graph.nodes.get(u)
        if u_node is None or u_node.pixel_x is None or u_node.pixel_y is None:
            continue
        p1 = (int(round(u_node.pixel_x)), int(round(u_node.pixel_y)))

        # 可选：跳过超出图像范围的点
        if not (0 <= p1[0] < W and 0 <= p1[1] < H):
            continue

        for v in nbrs.keys():
            if v <= u:  # 避免重复画无向边
                continue
            v_node = graph.nodes.get(v)
            if v_node is None or v_node.pixel_x is None or v_node.pixel_y is None:
                continue
            p2 = (int(round(v_node.pixel_x)), int(round(v_node.pixel_y)))
            if not (0 <= p2[0] < W and 0 <= p2[1] < H):
                continue

            cv2.line(img, p1, p2, edge_bgr, edge_thickness, lineType=cv2.LINE_AA)

    # ---------- 画节点（在边之上） ----------
    for nid, node in graph.nodes.items():
        if node.pixel_x is None or node.pixel_y is None:
            continue
        center = (int(round(node.pixel_x)), int(round(node.pixel_y)))
        if not (0 <= center[0] < W and 0 <= center[1] < H):
            continue

        cv2.circle(img, center, node_radius, node_bgr, thickness=2, lineType=cv2.LINE_AA)

        if show_ids:
            cv2.putText(
                img, str(nid),
                (center[0] + node_radius + 2, center[1] - node_radius - 2),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, node_bgr, 1, cv2.LINE_AA
            )

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    cv2.imwrite(out_path, img)
    return out_path