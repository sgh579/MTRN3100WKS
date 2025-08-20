# image_projection.py
import cv2
import numpy as np
from dataclasses import dataclass
from pathlib import Path
import os
from typing import Dict, Tuple, Optional, List
from tools.User_Configuration import IMAGE_FOLER

CornerMap = Dict[str, Tuple[int, int]]
ORDER_NAMES = ["top_left", "top_right", "bottom_left", "bottom_right"]

@dataclass
class ROIParams:
    max_window_w: int = 1400
    max_window_h: int = 900
    #      
    open_kernel: int = 3
    open_iter: int = 1
    #             ：'largest'   'nearest'
    pick: str = "largest"

class CornerFinder:
    def __init__(self, rp: ROIParams = ROIParams()):
        self.rp = rp
        self._img = None           #    BGR
        self._disp = None          #    （  ）
        self._scale = 1.0
        self._points: CornerMap = {}
        self._next_idx = 0
        self._win = "corner_roi_picker"  
    @staticmethod
    def _fit_window(img, max_w, max_h):
        h, w = img.shape[:2]
        scale = min(max_w / w, max_h / h, 1.0)
        if scale < 1.0:
            disp = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        else:
            disp = img.copy()
        return disp, scale
    def pick_corners_by_clicks(self, file_path: str) -> Optional[dict]:
        """
Select four corner points sequentially using mouse clicks.

Operation:
- Left click: Add corner points (order by ORDER_NAMES)
- z / Backspace: Undo the previous corner point selection
- r: Clear and start over
- Enter: Confirm and return if all four points are selected
- q / Esc: Cancel and exit (returns None)

Return: { name: (x, y), ... } — Original image coordinates (pixels)
        """
        p = Path(file_path)
        if not p.exists():
            print(f"[WARN] File not found: {file_path}")
            return None

        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            print(f"[WARN] Cannot read image: {str(p)}")
            return None

        assert hasattr(self, "_win"), "Window name self._win not set."
        assert hasattr(self, "rp") and hasattr(self.rp, "max_window_w") and hasattr(self.rp, "max_window_h"), "Runtime params missing."
        assert len(ORDER_NAMES) >= 4, "ORDER_NAMES must have at least 4 entries."

        #   
        self._img = img
        self._disp, self._scale = self._fit_window(img, self.rp.max_window_w, self.rp.max_window_h)
        H, W = img.shape[:2]
        clicked_pts_disp = []   #        （    ）
        clicked_pts_orig = []   #        （    ）

        # ---      ---
        def _draw():
            disp = self._disp.copy()
            #        
            for i, (dx, dy) in enumerate(clicked_pts_disp):
                cv2.circle(disp, (dx, dy), 8, (0, 0, 255), 2, cv2.LINE_AA)
                label = ORDER_NAMES[i] if i < len(ORDER_NAMES) else str(i + 1)
                cv2.putText(disp, label, (dx + 10, dy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            #     
            tip = "LMB: add | z/Backspace: undo | r: reset | Enter: confirm | q/Esc: quit"
            cv2.putText(disp, tip, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(self._win, disp)

        # ---     ：             ---
        def _on_mouse(event, x, y, flags, userdata):
            if event == cv2.EVENT_LBUTTONDOWN:
                if len(clicked_pts_disp) >= 4:
                    return
                #      ->     
                ox = int(round(x / self._scale))
                oy = int(round(y / self._scale))
                #     
                ox = max(0, min(W - 1, ox))
                oy = max(0, min(H - 1, oy))
                clicked_pts_disp.append((x, y))
                clicked_pts_orig.append((ox, oy))
                _draw()

        # ---      ---
        cv2.namedWindow(self._win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self._win, _on_mouse)

        try:
            _draw()
            while True:
                key = cv2.waitKey(20) & 0xFFFF

                if key in (27, ord('q')):  #   
                    return None

                if key in (ord('z'), 8):   #    (Backspace=8)
                    if clicked_pts_disp:
                        clicked_pts_disp.pop()
                        clicked_pts_orig.pop()
                        _draw()

                if key == ord('r'):        #   
                    clicked_pts_disp.clear()
                    clicked_pts_orig.clear()
                    _draw()

                if key in (13, 10):        # Enter   
                    if len(clicked_pts_orig) < 4:
                        print(f"[INFO] Need 4 points, currently {len(clicked_pts_orig)}.")
                        continue
                    break

                #     ：   4      
                if len(clicked_pts_orig) >= 4:
                    break

            #     （  ORDER_NAMES   ）
            result = {}
            for i in range(4):
                name = ORDER_NAMES[i]
                result[name] = clicked_pts_orig[i]

            #      （     ）
            vis = self._img.copy()
            for i in range(4):
                cx, cy = result[ORDER_NAMES[i]]
                cv2.circle(vis, (cx, cy), 10, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.putText(vis, ORDER_NAMES[i], (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
            return result

        finally:
            cv2.destroyAllWindows()
    

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
        return np.array([tl, tr, bl, br], dtype=np.float32)

    def warp_from_image(self,
                        image_path: str,
                        corners: CornerMap) -> np.ndarray:
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

        return warped

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
