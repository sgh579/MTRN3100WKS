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

PIXEL2MM_RATIO = 0.75

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
                     “    ”（  ）。
    p0_abs / p1_abs: (pixel_x, pixel_y)       
    """
    H, W = safe_mask_global.shape[0:2]
    x0, y0 = int(round(p0_abs[0])), int(round(p0_abs[1]))
    x1, y1 = int(round(p1_abs[0])), int(round(p1_abs[1]))
    #      ，     （         ）
    if not (0 <= x0 < W and 0 <= y0 < H and 0 <= x1 < W and 0 <= y1 < H):
        return False

    for x, y in bresenham_line(x0, y0, x1, y1):
        if not (0 <= x < W and 0 <= y < H):
            return False
        if not safe_mask_global[y, x]:
            return False
    return True

#         ：  (255,255,255)    True
def build_global_safe_mask(full_img_bgr: np.ndarray) -> np.ndarray:
    # full_img_bgr：  “     ”        BGR  （        ）
    return (full_img_bgr[:, :, 0] == 255) & (full_img_bgr[:, :, 1] == 255) & (full_img_bgr[:, :, 2] == 255)

def draw_graph_on_image(
    graph,
    image_path: str,
    node_radius: int = 8,
    edge_thickness: int = 2,
    node_bgr=(0, 0, 255),   #   （   OpenCV    BGR）
    edge_bgr=(0, 255, 0),   #   
    show_ids: bool = True,
    font_scale: float = 0.5)->np.ndarray:
    """
      image_path       ，  graph    /         out_path。
    - graph.nodes: {node_id: Node(...)}，Node.pixel_x/pixel_y      （     ）
    - graph.edges: {u: {v: weight, ...}, ...}（   ：    ）
    """
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {image_path}")

    H, W = img.shape[:2]

    # ----------   （    ：u < v） ----------
    for u, nbrs in graph.edges.items():
        u_node = graph.nodes.get(u)
        if u_node is None or u_node.pixel_x is None or u_node.pixel_y is None:
            continue
        p1 = (int(round(u_node.pixel_x)), int(round(u_node.pixel_y)))

        #   ：          
        if not (0 <= p1[0] < W and 0 <= p1[1] < H):
            continue

        for v in nbrs.keys():
            if v <= u:  #         
                continue
            v_node = graph.nodes.get(v)
            if v_node is None or v_node.pixel_x is None or v_node.pixel_y is None:
                continue
            p2 = (int(round(v_node.pixel_x)), int(round(v_node.pixel_y)))
            if not (0 <= p2[0] < W and 0 <= p2[1] < H):
                continue

            cv2.line(img, p1, p2, edge_bgr, edge_thickness, lineType=cv2.LINE_AA)

    # ----------    （    ） ----------
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
    graph,                                  # Graph   ：  add_node/add_edge/nodes/edges
    in_image_path: str,                      #       （out_path_3）
    out_crop_path: str,                      #        （out_path_6）
    out_unsafe_path: str,                    #    unsafe zone    （out_path_7）
    out_combined_path: str,                  #        （out_path_8）
    top_left_cell: Tuple[int, int],          # continuous_maze_top_left_cell
    bottom_right_cell: Tuple[int, int],      # continuous_maze_bottom_right_cell
    cell_px: int = 240,                      #     （       240）
    unsafe_kernel_size: int = 30,            # dilate kernel
    unsafe_iterations: int = 3,              # dilate   
    node_margin_px: int = 50,                #        
    division: int = 20,                      #     （   division×division   ）
    fully_connect: bool = True,              #       （O(N^2)，     /   ）
    edge_weight: float = 10.0,               #   
) -> Dict[str, str]:
    """
       in_image_path →    →      unsafe   →      ；
               （  ）    。       。

      ：
        {
            "cropped": out_crop_path,
            "unsafe": out_unsafe_path,
            "combined": out_combined_path
        }
    """
    os.makedirs(Path(out_crop_path).parent, exist_ok=True)
    os.makedirs(Path(out_unsafe_path).parent, exist_ok=True)
    os.makedirs(Path(out_combined_path).parent, exist_ok=True)

    # 1)     
    img = cv2.imread(in_image_path, cv2.IMREAD_COLOR)
    if img is None:
        raise IOError(f"      : {in_image_path}")

    # 2)       （   cell      ）
    x1 = top_left_cell[0] * cell_px
    y1 = top_left_cell[1] * cell_px
    x2 = (bottom_right_cell[0] + 1) * cell_px
    y2 = (bottom_right_cell[1] + 1) * cell_px

    H0, W0 = img.shape[:2]
    #     
    x1 = max(0, min(W0, x1)); x2 = max(0, min(W0, x2))
    y1 = max(0, min(H0, y1)); y2 = max(0, min(H0, y2))
    if not (x2 > x1 and y2 > y1):
        raise ValueError("      ，    top_left_cell/bottom_right_cell   cell_px。")

    # 3)      
    cropped = img[y1:y2, x1:x2].copy()
    cv2.imwrite(out_crop_path, cropped)

    # 4)    unsafe_zone
    #          out_path_6   ，         cropped
    gray_map = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    #    ：           
    _, binary_map = cv2.threshold(gray_map, 127, 255, cv2.THRESH_BINARY_INV)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (unsafe_kernel_size, unsafe_kernel_size))
    dilated_map = cv2.dilate(binary_map, kernel, iterations=unsafe_iterations)

    H, W = binary_map.shape
    #    ：  ，   dilated，   obstacle
    vis_unsafe = np.ones((H, W, 3), dtype=np.uint8) * 255
    dilated_mask = (dilated_map == 255)
    obstacle_mask = (binary_map == 255)
    vis_unsafe[dilated_mask] = [0, 0, 255]   # BGR:  
    vis_unsafe[obstacle_mask] = [0, 0, 0]    #  
    cv2.imwrite(out_unsafe_path, vis_unsafe)

    # 5)         
    img_combined = img.copy()
    img_combined[y1:y2, x1:x2] = vis_unsafe
    cv2.imwrite(out_combined_path, img_combined)

    # 6)            
    x_min = x1 + node_margin_px
    y_min = y1 + node_margin_px
    x_max = x2 - node_margin_px
    y_max = y2 - node_margin_px

    if division <= 0:
        raise ValueError("division    > 0")

    unit_w = (x_max - x_min) / division
    unit_h = (y_max - y_min) / division  #        unit，          

    #   1000     
    for r in range(division):
        for c in range(division):
            nid = 1000 + r * division + c
            pixel_x = int(round(x_min + (c + 0.5) * unit_w))  #          
            pixel_y = int(round(y_min + (r + 0.5) * unit_h))
            graph.add_node(nid, pixel_x, pixel_y, gx=None, gy=None)

    # 7)    
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
    # Nodes with IDs less than 1000 belong to the grid nodes.
    # Given a matrix range, points belong to the range of continuous nodes.
    # Traverse the circle of grid nodes closest to this range.
    # For each grid node, find the closest continuous node with no obstacles in between, and connect it with a weight of 10.
    # safe_mask_global = build_global_safe_mask(img_combined)

    gx1, gy1 = top_left_cell
    gx2, gy2 = bottom_right_cell
    #    (gx,gy) -> grid node id
    grid_index = {}
    for nid, node in graph.nodes.items():
        if nid < 1000 and getattr(node, "gx", None) is not None and getattr(node, "gy", None) is not None:
            grid_index[(node.gx, node.gy)] = nid

    border_cells = []
    #   ：  continuous block     
    for x in range(gx1, gx2 + 1):
        border_cells.append((x, gy1 - 1))

    #   ：  continuous block     
    for y in range(gy1, gy2 + 1):
        border_cells.append((gx2 + 1, y))

    #   ：  continuous block     
    for x in range(gx2, gx1 - 1, -1):
        border_cells.append((x, gy2 + 1))

    #   ：  continuous block     
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
        gid = graph.find_node_id(gx, gy)
        if gid is None:
            continue
        if gid not in graph.edges:
            graph.edges[gid] = {}

        gnode = graph.nodes[gid]
        gp = (gnode.pixel_x, gnode.pixel_y)

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
    path_color=(0, 255, 0),      #   
    path_thickness: int = 2,
    show_nodes: bool = False,
    node_radius: int = 4,
    node_color=(0, 0, 255),      #   
) -> np.ndarray:
    """
      graph    BFS         （   grid   ）     （    ）。
    - graph.nodes[nid]     : pixel_x, py（    ），gx, gy（grid   ,    grid nodes）
    - graph.edges[u]   dict，key      id
    - unsafe_zone: BGR   （         ，      ）

      ：  unsafe_zone             (np.ndarray)
    """
    unsafe_zone = cv2.imread(unsafe_zone_img_path)
    if unsafe_zone is None or unsafe_zone.ndim != 3:
        raise ValueError("unsafe_zone     BGR    (H,W,3)")

    # ---------- 1)    (gx,gy) -> grid_node_id     ----------
    grid_index: Dict[Tuple[int, int], int] = {}
    for nid, node in graph.nodes.items():
        #   ：grid nodes   id < 1000，    gx,gy
        if nid < 1000 and getattr(node, "gx", None) is not None and getattr(node, "gy", None) is not None:
            grid_index[(int(node.gx), int(node.gy))] = nid

    #        id
    start_id = graph.find_node_id(int(start_pos[0]), int(start_pos[1]))
    end_id   = graph.find_node_id(int(end_pos[0]), int(end_pos[1]))
    if start_id is None or end_id is None:
        raise ValueError(f"     grid          ：start={start_pos}, end={end_pos}")

    if start_id == end_id:
        #              
        vis = unsafe_zone.copy()
        s = graph.nodes[start_id]
        cv2.circle(vis, (int(s.pixel_x), int(s.pixel_y)), node_radius, node_color, -1, cv2.LINE_AA)
        return vis

    # ---------- 2) BFS（    ） ----------
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

    # ---------- 3)      ----------
    vis = unsafe_zone.copy()
    if not found:
        #     ：      （  ：          ）
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

    #   id → (pixel_x,pixel_y)
    pts = []
    H, W = vis.shape[:2]
    for nid in path_ids:
        n = graph.nodes[nid]
        if n is None or n.pixel_x is None or n.pixel_y is None:
            continue
        x, y = int(round(n.pixel_x)), int(round(n.pixel_y))
        # print(f'[DEBUG] x, y of nid {nid} : {x}, {y}')
        if 0 <= x < W and 0 <= y < H:
            pts.append([x, y])

    if len(pts) >= 2:
        pts_np = np.array(pts, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(vis, [pts_np], isClosed=False, color=path_color, thickness=path_thickness, lineType=cv2.LINE_AA)

    if show_nodes:
        for x, y in pts:
            cv2.circle(vis, (x, y), node_radius, node_color, -1, cv2.LINE_AA)

    #   ：       
    s = graph.nodes[start_id]; e = graph.nodes[end_id]
    cv2.circle(vis, (int(s.pixel_x), int(s.pixel_y)), node_radius+1, (0, 255, 255), -1, cv2.LINE_AA)  #     
    cv2.circle(vis, (int(e.pixel_x), int(e.pixel_y)), node_radius+1, (255, 0, 0),   -1, cv2.LINE_AA)  #     

    cmd_str = pts2cmd(pts)
    print(f'Final arduino cmd: {cmd_str}')

    return vis

def pts2cmd(pts)->str:
    i = 0
    prev_x = 0
    prev_y = 0
    cmd_list = []
    for waypoint in pts:

        if i == 0:
            prev_x = round(waypoint[0] * PIXEL2MM_RATIO)
            prev_y = round(waypoint[1] * PIXEL2MM_RATIO)
            i += 1
            continue

        x = round(waypoint[0] * PIXEL2MM_RATIO)
        y = round(waypoint[1] * PIXEL2MM_RATIO)
        #   x, y           ，      
        if x == prev_x:
            if y > prev_y:
                cmd_list.append('o180')
            else:
                cmd_list.append('o0')
            cmd_list.append(f'f{abs(y - prev_y)}')
        elif y == prev_y:
            if x > prev_x:
                cmd_list.append('o270')
            else:
                cmd_list.append('o90')            
            cmd_list.append(f'f{abs(x - prev_x)}')
        else:
            dy = x - prev_x
            dx = y - prev_y
            orientaion_rad = math.atan2(dy, dx)
            deg = math.degrees(orientaion_rad) + 180
            cmd_list.append(f'o{round(deg, 1)}')
            cmd_list.append(f'f{round(math.sqrt(dx*dx + dy*dy))}')
        i += 1
        [prev_x, prev_y] = [x, y]

    
    # print(cmd_list)

    #       
    result = []
    last_o = None  #        o    

    for cmd in cmd_list:
        if cmd.startswith("o"):
            angle = cmd[1:]  #      （   ）
            if angle != last_o:   #          
                result.append(cmd)
                last_o = angle
            #       
        else:  # f...
            result.append(cmd)
        
    merged = []
    acc = 0
    for cmd in result:
        if cmd.startswith("f"):
            acc += int(float(cmd[1:]))  #     
        else:  #    o，         f
            if acc != 0:
                merged.append("f" + str(acc))
                acc = 0
            merged.append(cmd)
    #          f    
    if acc != 0:
        merged.append("f" + str(acc))

    cmd_str = '|'.join(merged)
    return cmd_str