from __future__ import annotations
import cv2
import numpy as np
from pathlib import Path
from typing import Tuple, Optional
import math
import os
from tools.User_Configuration import IMAGE_FOLER
from tools.grid_graph import Grid_Graph

import os
from pathlib import Path
from typing import Tuple, Optional, Dict
import cv2
import numpy as np
from collections import deque

def bresenham_line(x0: int, y0: int, x1: int, y1: int):
    pts = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        pts.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return pts

def is_line_clear_global(safe_mask_global: np.ndarray, p0_abs, p1_abs) -> bool:
    """
    在整张图里判断两点连线是否完全经过“安全像素”（白色）。
    p0_abs / p1_abs: (pixel_x, pixel_y) 绝对像素坐标
    """
    H, W = safe_mask_global.shape[0:2]
    x0, y0 = int(round(p0_abs[0])), int(round(p0_abs[1]))
    x1, y1 = int(round(p1_abs[0])), int(round(p1_abs[1]))
    # 若端点越界，视为不可连（也可自行裁剪到边界）
    if not (0 <= x0 < W and 0 <= y0 < H and 0 <= x1 < W and 0 <= y1 < H):
        return False

    for x, y in bresenham_line(x0, y0, x1, y1):
        if not (0 <= x < W and 0 <= y < H):
            return False
        if not safe_mask_global[y, x]:
            return False
    return True

# 生成全图安全掩膜：仅 (255,255,255) 视为 True
def build_global_safe_mask(full_img_bgr: np.ndarray) -> np.ndarray:
    # full_img_bgr：与你“合成回原图”的图一样大小的 BGR 图（或任何全图标注图）
    return (full_img_bgr[:, :, 0] == 255) & (full_img_bgr[:, :, 1] == 255) & (full_img_bgr[:, :, 2] == 255)

def draw_graph_on_image(
    graph,
    image_path: str,
    node_radius: int = 8,
    edge_thickness: int = 2,
    node_bgr=(0, 0, 255),   # 红色（注意 OpenCV 使用 BGR）
    edge_bgr=(0, 255, 0),   # 绿色
    show_ids: bool = True,
    font_scale: float = 0.5)->np.ndarray:
    """
    在 image_path 指定的图片上，把 graph 的节点/边画出来并保存到 out_path。
    - graph.nodes: {node_id: Node(...)}，Node.pixel_x/pixel_y 为像素坐标（左上角原点）
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

    return img

def process_continuous_maze(
    graph,                                  # Graph 实例：有 add_node/add_edge/nodes/edges
    in_image_path: str,                      # 输入大图路径（out_path_3）
    out_crop_path: str,                      # 输出裁剪图路径（out_path_6）
    out_unsafe_path: str,                    # 输出 unsafe zone 图路径（out_path_7）
    out_combined_path: str,                  # 输出合成图路径（out_path_8）
    top_left_cell: Tuple[int, int],          # continuous_maze_top_left_cell
    bottom_right_cell: Tuple[int, int],      # continuous_maze_bottom_right_cell
    cell_px: int = 240,                      # 单格像素（你的代码使用 240）
    unsafe_kernel_size: int = 30,            # dilate kernel
    unsafe_iterations: int = 3,              # dilate 次数
    node_margin_px: int = 50,                # 连续区内部留边
    division: int = 20,                      # 网格划分（生成 division×division 个点）
    fully_connect: bool = True,              # 是否完全连接（O(N^2)，仅用于演示/小规模）
    edge_weight: float = 10.0,               # 边权
) -> Dict[str, str]:
    """
    读取 in_image_path → 裁剪 → 膨胀生成 unsafe 区 → 合成回原图；
    在该连续区域生成节点并（可选）完全连接。返回各输出路径。

    返回：
        {
            "cropped": out_crop_path,
            "unsafe": out_unsafe_path,
            "combined": out_combined_path
        }
    """
    os.makedirs(Path(out_crop_path).parent, exist_ok=True)
    os.makedirs(Path(out_unsafe_path).parent, exist_ok=True)
    os.makedirs(Path(out_combined_path).parent, exist_ok=True)

    # 1) 读取图片
    img = cv2.imread(in_image_path, cv2.IMREAD_COLOR)
    if img is None:
        raise IOError(f"无法读取图像: {in_image_path}")

    # 2) 计算裁剪坐标（基于 cell 坐标转像素）
    x1 = top_left_cell[0] * cell_px
    y1 = top_left_cell[1] * cell_px
    x2 = (bottom_right_cell[0] + 1) * cell_px
    y2 = (bottom_right_cell[1] + 1) * cell_px

    H0, W0 = img.shape[:2]
    # 边界钳制
    x1 = max(0, min(W0, x1)); x2 = max(0, min(W0, x2))
    y1 = max(0, min(H0, y1)); y2 = max(0, min(H0, y2))
    if not (x2 > x1 and y2 > y1):
        raise ValueError("裁剪区域无效，请检查 top_left_cell/bottom_right_cell 与 cell_px。")

    # 3) 裁剪并保存
    cropped = img[y1:y2, x1:x2].copy()
    cv2.imwrite(out_crop_path, cropped)

    # 4) 生成 unsafe_zone
    #    你的代码从 out_path_6 再读，这里直接用内存的 cropped
    gray_map = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    # 二值化：根据你的管线使用反阈值
    _, binary_map = cv2.threshold(gray_map, 127, 255, cv2.THRESH_BINARY_INV)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (unsafe_kernel_size, unsafe_kernel_size))
    dilated_map = cv2.dilate(binary_map, kernel, iterations=unsafe_iterations)

    H, W = binary_map.shape
    # 可视化：白底，红色 dilated，黑色 obstacle
    vis_unsafe = np.ones((H, W, 3), dtype=np.uint8) * 255
    dilated_mask = (dilated_map == 255)
    obstacle_mask = (binary_map == 255)
    vis_unsafe[dilated_mask] = [0, 0, 255]   # BGR: 红
    vis_unsafe[obstacle_mask] = [0, 0, 0]    # 黑
    cv2.imwrite(out_unsafe_path, vis_unsafe)

    # 5) 合成回原图并保存
    img_combined = img.copy()
    img_combined[y1:y2, x1:x2] = vis_unsafe
    cv2.imwrite(out_combined_path, img_combined)

    # 6) 在连续区域内部生成节点
    x_min = x1 + node_margin_px
    y_min = y1 + node_margin_px
    x_max = x2 - node_margin_px
    y_max = y2 - node_margin_px

    if division <= 0:
        raise ValueError("division 必须 > 0")

    unit_w = (x_max - x_min) / division
    unit_h = (y_max - y_min) / division  # 原代码用同一 unit，这里按宽高分离更合理

    # 从 1000 开始编号
    for r in range(division):
        for c in range(division):
            nid = 1000 + r * division + c
            pixel_x = int(round(x_min + (c + 0.5) * unit_w))  # 采单元中心点更均匀
            pixel_y = int(round(y_min + (r + 0.5) * unit_h))
            graph.add_node(nid, pixel_x, pixel_y, gx=None, gy=None)

    # 7) 连接边
    if fully_connect:

        gray = cv2.cvtColor(img_combined, cv2.COLOR_BGR2GRAY)
        _, safe_mask_global = cv2.threshold(gray, 254, 255, cv2.THRESH_BINARY)

        continuous_ids = [nid for nid in graph.nodes if nid >= 1000]
        for nid in continuous_ids:
            if nid not in graph.edges:
                graph.edges[nid] = {}

        for i, u in enumerate(continuous_ids):
            u_node = graph.nodes[u]
            p_u = (u_node.pixel_x, u_node.pixel_y)
            for v in continuous_ids[i+1:]:
                if v in graph.edges[u]:
                    continue
                v_node = graph.nodes[v]
                p_v = (v_node.pixel_x, v_node.pixel_y)

                if is_line_clear_global(safe_mask_global, p_u, p_v):
                    graph.add_edge(u, v, edge_weight)

    # 9)
    # id 小于1000的属于grid node
    # 已知一个矩阵范围的点属于continuous nodes的范围        
    # 遍历最靠近这个范围的那一圈grid nodes
    # 对于每一个grid node,找到continuous里面的一个最接近的，中间没有障碍物的continuous进行连接，权重为10
    # safe_mask_global = build_global_safe_mask(img_combined)

    gx1, gy1 = top_left_cell
    gx2, gy2 = bottom_right_cell
    # 构建 (gx,gy) -> grid node id
    grid_index = {}
    for nid, node in graph.nodes.items():
        if nid < 1000 and getattr(node, "gx", None) is not None and getattr(node, "gy", None) is not None:
            grid_index[(node.gx, node.gy)] = nid

    border_cells = []
    # 顶边：在 continuous block 上方一行
    for x in range(gx1, gx2 + 1):
        border_cells.append((x, gy1 - 1))

    # 右边：在 continuous block 右侧一列
    for y in range(gy1, gy2 + 1):
        border_cells.append((gx2 + 1, y))

    # 底边：在 continuous block 下方一行
    for x in range(gx2, gx1 - 1, -1):
        border_cells.append((x, gy2 + 1))

    # 左边：在 continuous block 左侧一列
    for y in range(gy2, gy1 - 1, -1):
        border_cells.append((gx1 - 1, y))

    continuous_ids = [nid for nid in graph.nodes if nid >= 1000]
    for nid in continuous_ids:
        if nid not in graph.edges:
            graph.edges[nid] = {}

    def euclidean(p, q):
        dx, dy = p[0]-q[0], p[1]-q[1]
        return (dx*dx + dy*dy) ** 0.5

    for gx, gy in border_cells:
        print(f"[DEBUG] checking grid node at ({gx}, {gy})")
        gid = graph.find_node_id(gx, gy)
        if gid is None:
            continue
        if gid not in graph.edges:
            graph.edges[gid] = {}

        gnode = graph.nodes[gid]
        gp = (gnode.pixel_x, gnode.pixel_y)
        print(f'gp:{gp}')

        best_cid, best_d = None, float('inf')
        for cid in continuous_ids:
            if cid in graph.edges[gid]:
                continue
            cnode = graph.nodes[cid]
            cp = (cnode.pixel_x, cnode.pixel_y)

            if is_line_clear_global(safe_mask_global, gp, cp):
                d = euclidean(gp, cp)
                if d < best_d:
                    best_d, best_cid = d, cid

        if best_cid is not None:
            graph.add_edge(gid, best_cid, 10.0)
            print(f'connnect {gid} and {best_cid}!')

    return {
        "cropped": out_crop_path,
        "unsafe": out_unsafe_path,
        "combined": out_combined_path,
    }

def continuous_bfs(
    graph,
    unsafe_zone_img_path: np.ndarray,
    start_pos: Tuple[int, int],
    end_pos: Tuple[int, int],
    path_color=(0, 255, 0),      # 绿色
    path_thickness: int = 2,
    show_nodes: bool = False,
    node_radius: int = 4,
    node_color=(0, 0, 255),      # 红色
) -> np.ndarray:
    """
    在 graph 上以 BFS 搜索从起点到终点（均为 grid 坐标）的最短路径（最少跳数）。
    - graph.nodes[nid] 需含属性: pixel_x, py（像素坐标），gx, gy（grid 坐标, 对于 grid nodes）
    - graph.edges[u] 是 dict，key 为邻接点 id
    - unsafe_zone: BGR 图片（不会被当作障碍判断，仅用于画路径）

    返回：在 unsafe_zone 上绘制了路径的图像副本 (np.ndarray)
    """
    unsafe_zone = cv2.imread(unsafe_zone_img_path)
    if unsafe_zone is None or unsafe_zone.ndim != 3:
        raise ValueError("unsafe_zone 必须是 BGR 图像 (H,W,3)")

    # ---------- 1) 建立 (gx,gy) -> grid_node_id 的索引 ----------
    grid_index: Dict[Tuple[int, int], int] = {}
    for nid, node in graph.nodes.items():
        # 约定：grid nodes 的 id < 1000，且具有 gx,gy
        if nid < 1000 and getattr(node, "gx", None) is not None and getattr(node, "gy", None) is not None:
            grid_index[(int(node.gx), int(node.gy))] = nid

    # 起终点的节点 id
    start_id = graph.find_node_id(int(start_pos[0]), int(start_pos[1]))
    end_id   = graph.find_node_id(int(end_pos[0]), int(end_pos[1]))
    if start_id is None or end_id is None:
        raise ValueError(f"无法根据 grid 坐标定位起点或终点：start={start_pos}, end={end_pos}")

    if start_id == end_id:
        # 直接在原图上画一个点并返回
        vis = unsafe_zone.copy()
        s = graph.nodes[start_id]
        cv2.circle(vis, (int(s.pixel_x), int(s.pixel_y)), node_radius, node_color, -1, cv2.LINE_AA)
        return vis

    # ---------- 2) BFS（最少跳数） ----------
    q = deque([start_id])
    visited = {start_id}
    parent: Dict[int, Optional[int]] = {start_id: None}

    found = False
    while q:
        u = q.popleft()
        if u == end_id:
            found = True
            break
        for v in graph.edges.get(u, {}):
            if v not in visited:
                visited.add(v)
                parent[v] = u
                q.append(v)

    # ---------- 3) 复原路径 ----------
    vis = unsafe_zone.copy()
    if not found:
        # 没有路径：直接返回原图（可选：在起终点各画一个标记）
        su = graph.nodes[start_id]; eu = graph.nodes[end_id]
        cv2.circle(vis, (int(su.pixel_x), int(su.pixel_y)), node_radius, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(vis, (int(eu.pixel_x), int(eu.pixel_y)), node_radius, (255, 0, 0), -1, cv2.LINE_AA)
        return vis

    path_ids: List[int] = []
    cur = end_id
    while cur is not None:
        path_ids.append(cur)
        cur = parent.get(cur)
    path_ids.reverse()

    # 将 id → (pixel_x,pixel_y)
    pts = []
    H, W = vis.shape[:2]
    for nid in path_ids:
        n = graph.nodes[nid]
        if n is None or n.pixel_x is None or n.pixel_y is None:
            continue
        x, y = int(round(n.pixel_x)), int(round(n.pixel_y))
        if 0 <= x < W and 0 <= y < H:
            pts.append([x, y])

    if len(pts) >= 2:
        pts_np = np.array(pts, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(vis, [pts_np], isClosed=False, color=path_color, thickness=path_thickness, lineType=cv2.LINE_AA)

    if show_nodes:
        for x, y in pts:
            cv2.circle(vis, (x, y), node_radius, node_color, -1, cv2.LINE_AA)

    # 可选：标记起终点颜色
    s = graph.nodes[start_id]; e = graph.nodes[end_id]
    cv2.circle(vis, (int(s.pixel_x), int(s.pixel_y)), node_radius+1, (0, 255, 255), -1, cv2.LINE_AA)  # 起点黄色
    cv2.circle(vis, (int(e.pixel_x), int(e.pixel_y)), node_radius+1, (255, 0, 0),   -1, cv2.LINE_AA)  # 终点蓝色

    return vis