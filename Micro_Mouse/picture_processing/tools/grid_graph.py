# tools/safe_zone.py
import cv2
import numpy as np
from pathlib import Path
from typing import Tuple, Optional
import math
import os
from tools.User_Configuration import IMAGE_FOLER

class ThresholdTuner:
    def __init__(self, max_window=(1400, 900), auto_resize=True):
        self.max_w, self.max_h = max_window
        self.auto_resize = auto_resize
        self.img = None
        self.gray = None
        self.scale = 1.0
        self.disp = None
        self.win = "Threshold Tuner"
        self.out_path = "./binary.png"
        #   ：             
        self.current_T = 128
        self.current_invert = 0
        self.current_bw = None

    def _fit(self, img):
        h, w = img.shape[:2]
        if not self.auto_resize:
            return img, 1.0
        s = min(self.max_w / w, self.max_h / h, 1.0)
        if s < 1.0:
            return cv2.resize(img, (int(w*s), int(h*s)), interpolation=cv2.INTER_AREA), s
        return img.copy(), 1.0

    def _update(self, _=None):
        #           
        self.current_T = cv2.getTrackbarPos("Threshold", self.win)   # 0..255
        self.current_invert = cv2.getTrackbarPos("Invert(0/1)", self.win)  # 0/1

        #            
        _, bw = cv2.threshold(self.gray, self.current_T, 255, cv2.THRESH_BINARY)
        if self.current_invert == 1:
            bw = cv2.bitwise_not(bw)

        #   （       ）
        show = bw
        if self.scale < 1.0:
            h, w = bw.shape[:2]
            show = cv2.resize(bw, (int(w*self.scale), int(h*self.scale)), interpolation=cv2.INTER_NEAREST)

        cv2.imshow(self.win, show)
        self.current_bw = bw  #         

    def run(self, image_path: str, out_path: str = "./binary.png"):
        p = Path(image_path)
        if not p.exists():
            raise FileNotFoundError(image_path)
        self.out_path = out_path

        self.img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if self.img is None:
            raise IOError(f"Cannot read image: {image_path}")

        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.disp, self.scale = self._fit(self.img)

        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        #             
        cv2.createTrackbar("Threshold", self.win, self.current_T, 255, self._update)
        cv2.createTrackbar("Invert(0/1)", self.win, self.current_invert, 1, self._update)
        self._update()

        print("Operation: Drag the slider to adjust the threshold; press S to save, Q/ESC to exit.")
        while True:
            k = cv2.waitKey(20) & 0xFF
            if k in (ord('q'), 27):
                break
            if k in (ord('s'), ord('S')):
                cv2.imwrite(self.out_path, self.current_bw)
                print(f"   ：{self.out_path}")

        #       ，    （      self.current_T/self.current_invert）
        final_T = self.current_T
        final_invert = self.current_invert
        final_bw = self.current_bw

        cv2.destroyAllWindows()
        #   ：   、    、    
        return final_bw, final_T, final_invert
    
class SafeZone:
    def __init__(self,
                 expect_size=(2160, 2160),
                 auto_resize=False,
                 invert=False,
                 threshold=160,
                 close_kernel=5, close_iter=1,
                 keep_largest=True,
                 fill_holes=True):
        self.expect_w, self.expect_h = expect_size
        self.auto_resize = auto_resize
        self.invert = invert
        self.threshold = threshold
        self.close_kernel = close_kernel
        self.close_iter = close_iter
        self.keep_largest = keep_largest
        self.fill_holes = fill_holes

    def _check_or_resize(self, img):
        h, w = img.shape[:2]
        if (w, h) == (self.expect_w, self.expect_h):
            return img
        if not self.auto_resize:
            raise ValueError(f"Input size {w}x{h} != expected {self.expect_w}x{self.expect_h}. "
                             f"Enable auto_resize=True to resize automatically.")
        return cv2.resize(img, (self.expect_w, self.expect_h), interpolation=cv2.INTER_AREA)

    def binarize(self, image_path: str) -> np.ndarray:
        p = Path(image_path)
        if not p.exists():
            raise FileNotFoundError(f"Image not found: {image_path}")

        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            raise IOError(f"Failed to read image: {image_path}")
        img = self._check_or_resize(img)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #        （ =  ）
        ttype = cv2.THRESH_BINARY_INV if self.invert else cv2.THRESH_BINARY
        _, bw = cv2.threshold(gray, self.threshold, 255, ttype)

        # 1)    ：   、    
        if self.close_kernel > 1 and self.close_iter > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.close_kernel, self.close_kernel))
            bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, k, iterations=self.close_iter)

        # 2)           （       ）
        if self.keep_largest:
            num, labels, stats, _ = cv2.connectedComponentsWithStats(bw, connectivity=8)
            if num > 1:
                idx = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])  #     0
                bw = np.where(labels == idx, 255, 0).astype(np.uint8)

        # 3)           （       ）
        if self.fill_holes:
            inv = cv2.bitwise_not(bw)                    #  =    /  
            h, w = inv.shape
            mask = np.zeros((h + 2, w + 2), np.uint8)
            cv2.floodFill(inv, mask, (0, 0), 255)       #       “   ”    
            holes = cv2.bitwise_not(inv)                 #     =      
            bw = cv2.bitwise_or(bw, holes)               #        →   
        
        return bw

class DisplayGridOnImg:
    """
Overlays a uniform grid on the image and saves it.
The default grid divides the image into a 9×9 grid (8 vertical lines + 8 horizontal lines).
    """
    def __init__(self,
                 rows: int = 9,
                 cols: int = 9,
                 color: Tuple[int, int, int] = (0, 0, 255),  # BGR   
                 thickness: int = 2,
                 draw_border: bool = False,
                 line_type: int = cv2.LINE_AA):
        assert rows > 1 and cols > 1, "rows/cols    > 1"
        self.rows = rows
        self.cols = cols
        self.color = color
        self.thickness = thickness
        self.draw_border = draw_border
        self.line_type = line_type

    def draw_and_save(self, image_path: str, out_path: Optional[str] = None) -> str:
        """
           image_path，       。
              。
        """
        p = Path(image_path)
        if not p.exists():
            raise FileNotFoundError(f"Image not found: {image_path}")

        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            raise IOError(f"Failed to read image: {image_path}")

        h, w = img.shape[:2]
        out = img.copy()

        #   ：  x = j * (w/cols), j = 1..cols-1
        for j in range(1, self.cols):
            x = round(j * w / self.cols)
            cv2.line(out, (x, 0), (x, h - 1), self.color, self.thickness, self.line_type)

        # Horizontal: present y = i * (h/rows), i = 1..rows-1
        for i in range(1, self.rows):
            y = round(i * h / self.rows)
            cv2.line(out, (0, y), (w - 1, y), self.color, self.thickness, self.line_type)

        # Optional: Draw the border around the edges
        if self.draw_border:
            cv2.rectangle(out, (0, 0), (w - 1, h - 1), self.color, self.thickness, self.line_type)

        if out_path is None:
            out_path = str(p.with_name(f"{p.stem}_grid_{self.rows}x{self.cols}{p.suffix}"))
        cv2.imwrite(out_path, out)
        return out_path
    
class Grid_Graph:
    def __init__(self,
                 image_path: str,
                 rows: int = 9,
                 cols: int = 9,
                 connectivity: int = 4,        # 4 or 8
                 node_radius: int = 4,         #     ；     * node_diameter_scale
                 node_diameter_scale: int = 3, # If the diameter is magnified (=3), the radius will also be *3
                 grid_color: Tuple[int,int,int] = (0, 0, 255),     
                 grid_thickness: int = 2,
                 edge_color: Tuple[int,int,int] = (0, 255, 0),     
                 edge_thickness: int = 2,
                 node_color: Tuple[int,int,int] = (0, 255, 255)):  
        assert rows > 1 and cols > 1, "rows/cols must > 1"
        assert connectivity in (4, 8), "connectivity can only be 4 or 8"
        self.image_path = image_path
        self.rows = rows
        self.cols = cols
        self.connectivity = connectivity
        self.node_radius = node_radius
        self.node_diameter_scale = node_diameter_scale
        self.grid_color = grid_color
        self.grid_thickness = grid_thickness
        self.edge_color = edge_color
        self.edge_thickness = edge_thickness
        self.node_color = node_color

        self.img = None
        self.graph = None
        self._centers = None  # [(id, x, y, gx, gy), ...]

    def _load_image(self):
        p = Path(self.image_path)
        if not p.exists():
            raise FileNotFoundError(f"Image not found: {self.image_path}")
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            raise IOError(f"Failed to read image: {self.image_path}")
        return img

    def _compute_centers(self):
        """Calculate the grid center pixel coordinates and grid index (0-based)"""
        h, w = self.img.shape[:2]
        cell_w = w / self.cols
        cell_h = h / self.rows
        centers = []
        node_id = 0
        for gy in range(self.rows):
            for gx in range(self.cols):
                cx = round((gx + 0.5) * cell_w)
                cy = round((gy + 0.5) * cell_h)
                centers.append((node_id, cx, cy, gx, gy))
                node_id += 1
        return centers

    @staticmethod
    def _dist(x1, y1, x2, y2) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    def build(self) -> "Graph":
        """Create a graph: Create points at the center of the grid; use 4/8 adjacency for edges (weight = pixel distance)"""
        self.img = self._load_image()
        self._centers = self._compute_centers()

        G = Graph()

        # 1) Add Node
        for nid, px, py, gx, gy in self._centers:
            G.add_node(nid, px, py, gx, gy)

        # 2) Add edges (4 adjacent: right, bottom; 8 adjacent plus bottom right, bottom left)
        def nid_of(gx, gy):
            return gy * self.cols + gx

        h, w = self.img.shape[:2]
        cell_w = w / self.cols
        cell_h = h / self.rows

        for gy in range(self.rows):
            for gx in range(self.cols):
                u = nid_of(gx, gy)
                px_u = round((gx + 0.5) * cell_w)
                py_u = round((gy + 0.5) * cell_h)

                #  
                if gx + 1 < self.cols:
                    v = nid_of(gx + 1, gy)
                    px_v = round((gx + 1 + 0.5) * cell_w)
                    py_v = py_u
                    G.add_edge(u, v, self._dist(px_u, py_u, px_v, py_v))

                #  
                if gy + 1 < self.rows:
                    v = nid_of(gx, gy + 1)
                    px_v = px_u
                    py_v = round((gy + 1 + 0.5) * cell_h)
                    G.add_edge(u, v, self._dist(px_u, py_u, px_v, py_v))

                if self.connectivity == 8:
                    #   
                    if gx + 1 < self.cols and gy + 1 < self.rows:
                        v = nid_of(gx + 1, gy + 1)
                        px_v = round((gx + 1 + 0.5) * cell_w)
                        py_v = round((gy + 1 + 0.5) * cell_h)
                        G.add_edge(u, v, self._dist(px_u, py_u, px_v, py_v))
                    #   
                    if gx - 1 >= 0 and gy + 1 < self.rows:
                        v = nid_of(gx - 1, gy + 1)
                        px_v = round((gx - 1 + 0.5) * cell_w)
                        py_v = round((gy + 1 + 0.5) * cell_h)
                        G.add_edge(u, v, self._dist(px_u, py_u, px_v, py_v))

        self.graph = G
        return G

    def render(self) -> np.ndarray:
        """Draw on the original image: red grid lines, green edges, and nodes magnified 3 times the diameter; save"""
        if self.img is None or self.graph is None or self._centers is None:
            raise RuntimeError("Call build() before render().")

        canvas = self.img.copy()
        h, w = canvas.shape[:2]

        # Draw the red grid lines first
        for j in range(1, self.cols):
            x = round(j * w / self.cols)
            cv2.line(canvas, (x, 0), (x, h - 1), self.grid_color, self.grid_thickness, cv2.LINE_AA)
        for i in range(1, self.rows):
            y = round(i * h / self.rows)
            cv2.line(canvas, (0, y), (w - 1, y), self.grid_color, self.grid_thickness, cv2.LINE_AA)

        # Draw the lines between the green nodes (to avoid duplication: only draw u<v)
        drawn = set()
        for u, nbrs in self.graph.edges.items():
            for v in nbrs.keys():
                if u < v and (u, v) not in drawn:
                    pu = self.graph.nodes[u]
                    pv = self.graph.nodes[v]
                    cv2.line(canvas,
                             (int(pu.pixel_x), int(pu.pixel_y)),
                             (int(pv.pixel_x), int(pv.pixel_y)),
                             self.edge_color, self.edge_thickness, cv2.LINE_AA)
                    drawn.add((u, v))

        # Finally, draw the node (solid circle), diameter × 3 → radius × 3
        draw_radius = int(self.node_radius * self.node_diameter_scale)
        for nid, px, py, gx, gy in self._centers:
            cv2.circle(canvas, (int(px), int(py)), draw_radius, self.node_color, -1, cv2.LINE_AA)
            # ← Print grid coordinates (gx, gy) on the saved image
            label = f"({gx},{gy})"
            pos = (int(px) + draw_radius + 4, int(py) - draw_radius - 4)  # Text position: upper right of the dot
            fs = 0.6
            # Draw the black outline first and then the white text to improve readability
            cv2.putText(canvas, label, pos, cv2.FONT_HERSHEY_SIMPLEX, fs, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, label, pos, cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), 1, cv2.LINE_AA)

        return canvas
    def filter_edges_by_mask(self,
                             mask_image_path: str,
                             white_thresh: int = 200,
                             line_thickness: int = 3,
                             require_ratio: float = 1.0,
                             mid_ratio: float = 0.5) -> Tuple[int, int]:
        """
Use a binary/grayscale mask to filter edges: Only pixels along the entire line of edges that are white (≥ white_thresh) are retained.
- white_thresh: Mask grayscale >= this value is considered white.
- line_thickness: Line thickness (in pixels) to check on the mask.
- require_ratio: Permissible white pixel ratio threshold (strictly 1.0 for all white pixels, can be relaxed to 0.98).
Returns: (number of edges to retain, number of edges to remove)
        """
        if self.graph is None or self.img is None:
            raise RuntimeError("Call build() before filter_edges_by_mask().")


        p = Path(mask_image_path)
        if not p.exists():
            raise FileNotFoundError(mask_image_path)
        mask = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise IOError(f"Failed to read mask: {mask_image_path}")

        # Dimension Alignment
        H, W = self.img.shape[:2]
        if mask.shape[:2] != (H, W):
            # Binary/label masks are scaled using nearest neighbors to avoid grayscale interpolation
            mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)

        to_remove = []

        # Only check 4-neighbor edges (left, right, top, and bottom). We may have 8 adjacencies when building; here we only delete edges that belong to the 4-neighbors.
        def is_4_neighbor(u, v):
            nu, nv = self.graph.nodes[u], self.graph.nodes[v]
            dx = abs(nu.grid_x - nv.grid_x)
            dy = abs(nu.grid_y - nv.grid_y)
            return (dx + dy) == 1  # Manhattan distance 1: up and down or left and right

        # Enumerate undirected edges (u < v)
        for u, nbrs in self.graph.edges.items():
            for v in list(nbrs.keys()):
                if u >= v:
                    continue
                if not is_4_neighbor(u, v):
                    continue  # Only process up, down, left and right

                nu = self.graph.nodes[u]
                nv = self.graph.nodes[v]
                x1, y1 = int(nu.pixel_x), int(nu.pixel_y)
                x2, y2 = int(nv.pixel_x), int(nv.pixel_y)

                # Draw this line on an empty mask as a sampling channel
                line_mask = np.zeros((H, W), dtype=np.uint8)

                alpha = float(np.clip(mid_ratio, 1e-6, 1.0))   # 0~1 
                t0 = (1.0 - alpha) / 2.0
                t1 = 1.0 - t0

                mx1 = int(round(x1 + t0 * (x2 - x1)))
                my1 = int(round(y1 + t0 * (y2 - y1)))
                mx2 = int(round(x1 + t1 * (x2 - x1)))
                my2 = int(round(y1 + t1 * (y2 - y1)))

                cv2.line(line_mask, (mx1, my1), (mx2, my2), 255, thickness=line_thickness, lineType=cv2.LINE_8)
                line_vals = mask[line_mask > 0]
                if line_vals.size == 0:
                    # Theoretically, it won't happen; to be safe, treat it as not passing.
                    to_remove.append((u, v))
                    continue

                white_ratio = float(np.mean(line_vals >= white_thresh))
                if white_ratio < require_ratio:
                    to_remove.append((u, v))


        for (u, v) in to_remove:
            self.graph.remove_edge(u, v)

        kept = sum(len(nbrs) for nbrs in self.graph.edges.values()) // 2
        removed = len(to_remove)
        return kept, removed

    def filter_and_render(self,
                          mask_image_path: str,
                          out_path: Optional[str] = None,
                          white_thresh: int = 200,
                          line_thickness: int = 3,
                          require_ratio: float = 1.0,
                          mid_ratio: float = 0.5) -> str:
        """
            ：      ，  render()   。
        """
        self.filter_edges_by_mask(mask_image_path,
                                  white_thresh=white_thresh,
                                  line_thickness=line_thickness,
                                  require_ratio=require_ratio,
                                  mid_ratio=mid_ratio)
        return self.render(out_path=out_path)


class Node:
    def __init__(self, node_id, px, py, gx, gy):
        self.id = node_id
        self.pixel_x = px
        self.pixel_y = py
        self.grid_x = gx
        self.grid_y = gy
        
    
    def get_point_img(self):
        return (self.pixel_x,self.pixel_y)
    
    def get_point_gird(self):
        return (self.grid_x,self.grid_y)
    
    def get_ID(self):
        return self.id

class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = {}
        self.nodes_cnt = 0
        self.edges_cnt = 0

    def add_node(self, node_id, px, py, gx, gy):
        if node_id not in self.nodes:
            self.nodes[node_id] = Node(node_id, px, py, gx, gy)
            self.edges[node_id] = {}
            self.nodes_cnt += 1
        else:
            print(f"Node {node_id} already exists in the graph.")

    def add_edge(self, node_id1, node_id2, weight):
        if node_id1 in self.nodes and node_id2 in self.nodes:
            self.edges[node_id1][node_id2] = weight
            self.edges[node_id2][node_id1] = weight
            self.edges_cnt += 1
        else:
            print(f"One or both nodes {node_id1} and {node_id2} do not exist in the graph.")

    def remove_edge(self, node_id1, node_id2):
        if node_id1 in self.edges and node_id2 in self.edges[node_id1]:
            del self.edges[node_id1][node_id2]
            del self.edges[node_id2][node_id1]
            self.edges_cnt -= 1
        else:
            print(f"Edge between {node_id1} and {node_id2} does not exist.")
    
    def get_nodes(self):
        return list(self.nodes.keys())
    
    def get_edge_weight(self, node_id1, node_id2):
        return self.edges.get(node_id1, {}).get(node_id2, None) 
    
    def find_node_id(self, gx, gy):
        for nid, node in self.nodes.items():
            if node.grid_x == gx and node.grid_y == gy:
                return nid
        return None
    def remove_node(self, node_id)->bool:
        if node_id not in self.nodes:
            print(f"Node {node_id} does not exist in the graph.")
            return False
        self.nodes_cnt -= 1
        
        # Delete all edges connected to the node (including back edges in neighbors)
        neighbors = list(self.edges.get(node_id, {}).keys())
        for nb in neighbors:
            if nb in self.edges and node_id in self.edges[nb]:
                del self.edges[nb][node_id]
                self.edges_cnt -= 1
        # Delete the node's own adjacency list
        if node_id in self.edges:
            del self.edges[node_id]

        # Finally delete the node body
        del self.nodes[node_id]
        return True