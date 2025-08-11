import cv2
import numpy as np
from tools.image_projection import CornerFinder, Projection
from tools.grid_graph import ThresholdTuner, SafeZone, DisplayGridOnImg, Grid_Graph

if __name__ == '__main__':
    # ============================
    # Block 1 — Pick 4 corner markers (human-in-the-loop)
    # Purpose: Open an interactive window, let the user drag/select ROIs for
    #          TL → TR → BL → BR, and compute the circle centers inside each ROI.
    # Inputs : Raw camera image path
    # Output : corners = {'top_left':(x,y), 'top_right':(x,y),
    #                     'bottom_left':(x,y), 'bottom_right':(x,y)} or None
    # ============================
    CornerF = CornerFinder()
    corners = CornerF.pick_corners_with_roi("Micro_Mouse\\picture_processing\\MicromouseMazeCamera.jpg")
    if corners is None:
        print("Cancelled or failed.")
    else:
        print(corners)

    # ============================
    # Block 2 — Perspective warp (projection)
    # Purpose: Map the original image to a fixed-size top-down view.
    # Config : out_size=(2160,2160), margin=120 ⇒ TL maps to (120,120),
    #          other 3 corners placed symmetrically.
    # Output : A rectified image saved to disk; returns the output path.
    # ============================
    proj = Projection(out_size=(2160,2160), margin=120)
    out_file = proj.warp_from_image("Micro_Mouse\\picture_processing\\MicromouseMazeCamera.jpg", corners)
    print("Saved to:", out_file)

    # ============================
    # Block 3 — Manual threshold tuning (preview only)
    # Purpose: Interactively sweep a global threshold and preview the binary mask.
    # Action : Adjust the slider; press S to save the preview mask as PNG.
    # Note   : This step is for visual tuning; the pipeline below still
    #          produces its own binary using a fixed threshold.
    # ============================
    tuner = ThresholdTuner()  
    bw = tuner.run("MicromouseMazeCamera_projected_2160x2160.png", out_path="./safe_zone_binary_manual.png")

    # ============================
    # Block 4 — Fixed-threshold binarization + morphology
    # Purpose: Produce a clean “white = free space” mask from the rectified image.
    # Config : threshold=160 (global), closing (kernel=5, iter=1),
    #          keep only the largest white component, fill interior holes.
    # Output : Binary mask saved to disk.
    # ============================
    sz = SafeZone(expect_size=(2160,2160), auto_resize=False,
              threshold=160, close_kernel=5, close_iter=1,
              keep_largest=True, fill_holes=True)
    out_file = sz.binarize("MicromouseMazeCamera_projected_2160x2160.png", "./safe_zone_binary.png")

    # ============================
    # Block 5 — Build grid graph and prune edges by mask
    # Purpose: Create a 9×9 node grid graph (node at each cell center),
    #          then remove edges that cross non-white pixels in the mask.
    # Steps  :
    #   (a) build() creates nodes/edges (4-connectivity by default)
    #   (b) filter_edges_by_mask(...) keeps only edges whose middle segment
    #       lies on white (require_ratio=1.0 = strictly white).
    #   (c) render() draws remaining edges (green), grid lines (red), and nodes.
    # Outputs: Kept/removed edge counts in console; final visualization saved.
    # ============================
    gg = Grid_Graph("safe_zone_binary.png", rows=9, cols=9, connectivity=4)
    G = gg.build()
    kept, removed = gg.filter_edges_by_mask("safe_zone_binary.png",
                                            white_thresh=200,      
                                            line_thickness=3,      
                                            require_ratio=1.0)     
    print(f"kept={kept}, removed={removed}")
    save_path = gg.render()   
    print("Saved to:", save_path)

    bfs_graph = gg.graph # this is the graph to be used in bfs
# use BFS to generate a series of motion command to complete task 4.1 in format of extended cmd
# it is expected to be copied into the source code of arduino
# command list example: f18|o90|f18|f18|o0|