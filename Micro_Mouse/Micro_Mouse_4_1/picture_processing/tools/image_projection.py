# image_projection.py
import cv2
import numpy as np
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple, Optional, List

CornerMap = Dict[str, Tuple[int, int]]
ORDER_NAMES = ["top_left", "top_right", "bottom_left", "bottom_right"]

@dataclass
class ROIParams:
    max_window_w: int = 1400
    max_window_h: int = 900
    # Morphological denoising
    open_kernel: int = 3
    open_iter: int = 1
    # Selection strategy when there are multiple black blocks: 'largest' or 'nearest'
    pick: str = "largest"

class CornerFinder:
    def __init__(self, rp: ROIParams = ROIParams()):
        self.rp = rp
        self._img = None           
        self._disp = None          
        self._scale = 1.0
        self._points: CornerMap = {}
        self._next_idx = 0
        self._win = "corner_roi_picker"

        # Current selection status
        self._dragging = False
        self._p0 = None            # (x, y) Display coordinates
        self._p1 = None
        self._roi_rect = None      # (x1, y1, x2, y2) Original image coordinates
        self._roi_center = None    # Original image coordinates
        self._roi_centroid = None  # Identified center of mass (original image coordinates)

    # ---------- utils ----------
    @staticmethod
    def _fit_window(img, max_w, max_h):
        h, w = img.shape[:2]
        scale = min(max_w / w, max_h / h, 1.0)
        if scale < 1.0:
            disp = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        else:
            disp = img.copy()
        return disp, scale

    def _draw(self):
        disp = self._disp.copy()

        # Confirmed points
        for name, (cx, cy) in self._points.items():
            dx, dy = int(cx * self._scale), int(cy * self._scale)
            cv2.circle(disp, (dx, dy), 8, (0, 255, 0), 2)
            cv2.putText(disp, name, (dx + 8, dy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1, cv2.LINE_AA)

        # Current drag rectangle (display coordinates)
        if self._p0 and self._p1:
            x0, y0 = self._p0
            x1, y1 = self._p1
            cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 200, 255), 2)

        # Current ROI centroid
        if self._roi_centroid is not None:
            dx, dy = int(self._roi_centroid[0] * self._scale), int(self._roi_centroid[1] * self._scale)
            cv2.circle(disp, (dx, dy), 10, (0, 0, 255), 2)
            cv2.putText(disp, ORDER_NAMES[self._next_idx], (dx + 10, dy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2, cv2.LINE_AA)

        # Top Tips
        if self._next_idx < 4:
            hint = f"Drag a box around {ORDER_NAMES[self._next_idx]} | Enter=confirm  r=redo  q=quit"
        else:
            hint = "All 4 picked."
        cv2.rectangle(disp, (0, 0), (disp.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(disp, hint, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow(self._win, disp)

    # ---------- core processing on ROI ----------
    def _process_roi_and_get_centroid(self, rect_xyxy: Tuple[int,int,int,int]) -> Optional[Tuple[float,float]]:
        """rect_xyxy is in the original image coordinate system; returns the centroid (cx, cy) of the black connected region"""
        x1, y1, x2, y2 = rect_xyxy
        # Correcting Boundaries
        H, W = self._img.shape[:2]
        x1, x2 = np.clip([x1, x2], 0, W-1)
        y1, y2 = np.clip([y1, y2], 0, H-1)
        if x2 <= x1 + 2 or y2 <= y1 + 2:
            return None

        roi = self._img[y1:y2+1, x1:x2+1]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # Otsu binary + inversion (black → white)
        _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # Slightly open the calculation denoising
        if self.rp.open_kernel > 1 and self.rp.open_iter > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.rp.open_kernel, self.rp.open_kernel))
            bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, k, iterations=self.rp.open_iter)

        # Connected domain
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(bw, connectivity=8)
        if num <= 1:
            return None  # Only background

        # Filter out very small noise (< 10 pixels)
        valid: List[int] = []
        for i in range(1, num):
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= 10:
                valid.append(i)
        if not valid:
            return None

        # Select a strategy
        if self.rp.pick == "nearest":
            cx0 = (x1 + x2) / 2.0
            cy0 = (y1 + y2) / 2.0
            best_i = min(valid, key=lambda i: np.hypot((x1 + centroids[i,0]) - cx0,
                                                       (y1 + centroids[i,1]) - cy0))
        else:  # 'largest'
            best_i = max(valid, key=lambda i: stats[i, cv2.CC_STAT_AREA])

        cx, cy = centroids[best_i]
        #        
        return (x1 + float(cx), y1 + float(cy))

    # ---------- interaction ----------
    def _on_mouse(self, event, x, y, flags, userdata):
        if self._next_idx >= 4:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self._dragging = True
            self._p0 = (x, y)
            self._p1 = (x, y)
            self._roi_centroid = None
            self._roi_rect = None
            self._draw()
        elif event == cv2.EVENT_MOUSEMOVE and self._dragging:
            self._p1 = (x, y)
            self._draw()
        elif event == cv2.EVENT_LBUTTONUP and self._dragging:
            self._dragging = False
            self._p1 = (x, y)
            #          
            x0, y0 = self._p0
            x1, y1 = self._p1
            x0o, y0o = int(round(x0 / self._scale)), int(round(y0 / self._scale))
            x1o, y1o = int(round(x1 / self._scale)), int(round(y1 / self._scale))
            x1o, x2o = sorted([x0o, x1o])
            y1o, y2o = sorted([y0o, y1o])
            self._roi_rect = (x1o, y1o, x2o, y2o)
            self._roi_center = ((x1o + x2o) / 2.0, (y1o + y2o) / 2.0)

            centroid = self._process_roi_and_get_centroid(self._roi_rect)
            if centroid is None:
                print("[INFO] No black region detected in ROI. Try a tighter box.")
            self._roi_centroid = centroid
            self._draw()

    def pick_corners_with_roi(self, file_path: str) -> Optional[CornerMap]:
        p = Path(file_path)
        if not p.exists():
            print(f"[WARN] File not found: {file_path}")
            return None

        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            print(f"[WARN] Cannot read image: {str(p)}")
            return None

        self._img = img
        self._disp, self._scale = self._fit_window(img, self.rp.max_window_w, self.rp.max_window_h)
        self._points.clear()
        self._next_idx = 0
        self._dragging = False
        self._p0 = self._p1 = None
        self._roi_rect = None
        self._roi_centroid = None

        cv2.namedWindow(self._win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self._win, self._on_mouse)

        self._draw()
        while True:
            key = cv2.waitKey(20) & 0xFFFF
            if key in (27, ord('q')):  #   
                cv2.destroyAllWindows()
                return None
            if key in (ord('r'),):     #      
                self._p0 = self._p1 = None
                self._roi_rect = None
                self._roi_centroid = None
                self._draw()
            if key in (13, 10):        # Enter   
                if self._roi_centroid is None:
                    print("[INFO] No centroid yet. Drag a box first.")
                    continue
                name = ORDER_NAMES[self._next_idx]
                self._points[name] = (int(round(self._roi_centroid[0])),
                                      int(round(self._roi_centroid[1])))
                self._next_idx += 1
                #     ，     
                self._p0 = self._p1 = None
                self._roi_rect = None
                self._roi_centroid = None
                self._draw()
                if self._next_idx >= 4:
                    break

            if self._next_idx >= 4:
                break

        #      
        vis = self._img.copy()
        for name, (cx, cy) in self._points.items():
            cv2.circle(vis, (cx, cy), 10, (0, 0, 255), 2)
            cv2.putText(vis, name, (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.imwrite("corner_pick_debug.png", vis)
        cv2.destroyAllWindows()
        return dict(self._points)
    

class Projection:
    def __init__(self, out_size: Tuple[int, int] = (2160, 2160), margin: int = 100):
        self.out_w, self.out_h = out_size
        self.margin = margin

    def _dest_points(self) -> np.ndarray:
        #       （  TL, TR, BL, BR   ）
        tl = (self.margin, self.margin)
        tr = (self.out_w - self.margin, self.margin)
        bl = (self.margin, self.out_h - self.margin)
        br = (self.out_w - self.margin, self.out_h - self.margin)
        print(f'in target image: \n tl: {tl} \n tr: {tr} \n bl: {bl} \n br: {br} \n')
        return np.array([tl, tr, bl, br], dtype=np.float32)

    def warp_from_image(self,
                        image_path: str,
                        corners: CornerMap,
                        out_path: Optional[str] = None) -> str:
        """
        corners: {'top_left':(x,y), 'top_right':(x,y), 'bottom_left':(x,y), 'bottom_right':(x,y)}
               ，        。
        """
        required = ["top_left", "top_right", "bottom_left", "bottom_right"]
        for k in required:
            if k not in corners:
                raise ValueError(f"Missing corner key: {k}")

        img = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(f"Cannot read image: {image_path}")

        #   ：  TL, TR, BL, BR   
        src = np.array([
            corners["top_left"],
            corners["top_right"],
            corners["bottom_left"],
            corners["bottom_right"],
        ], dtype=np.float32)

        dst = self._dest_points()

        #          
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, (self.out_w, self.out_h), flags=cv2.INTER_LINEAR)

        #     
        if out_path is None:
            stem = Path(image_path).stem
            out_path = f"./{stem}_projected_{self.out_w}x{self.out_h}.png"

        cv2.imwrite(out_path, warped)
        return out_path

if __name__ == "__main__":
    CornerF = CornerFinder()
    #         
    corners = CornerF.pick_corners_with_roi("Micro_Mouse\\picture_processing\\MicromouseMazeCamera.jpg")
    if corners is None:
        print("Cancelled or failed.")
    else:
        print(corners)
    proj = Projection(out_size=(2160,2160), margin=100)
    out_file = proj.warp_from_image("Micro_Mouse\\picture_processing\\MicromouseMazeCamera.jpg", corners)
    print("Saved to:", out_file)
