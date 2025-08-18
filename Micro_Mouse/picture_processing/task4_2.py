import os
# os.chdir(os.path.dirname(os.path.abspath(__file__)))
import cv2
import numpy as np
from collections import deque
# import networkx as nx
from tools.image_projection import CornerFinder, Projection
from tools.grid_graph import ThresholdTuner, SafeZone, DisplayGridOnImg, Grid_Graph
from tools.BFS_pathfinding import BFSPathfinder, run_pathfinding_example
from tools.User_Configuration import IMAGE_FOLER

THRESHOLD_TUNER_ENABLE_FLAG = True

def main():
    print(' ############# 4.1 starts #############')

    # change to the desired workspace
    wks_path = 'x:/1_Projects/UNSW_MSC/MTRN3100/MTRN3100WKS/'
    os.chdir(wks_path)

    images_folder = IMAGE_FOLER
    # backup_image = "backup_image.jpg"
    backup_image = "captured_image.jpg"
    camera_raw_save_image = "captured_image.jpg"

    raw_image = backup_image

    # use camera 1 to capture one frame and save to local. If camera 1 doesn't exit, use the backup image to do some deugging
    print('trying to open the camera 1')
    cap = cv2.VideoCapture(1)  # Camera ID 1
    if cap.isOpened():
        ret, frame = cap.read()
        cap.release()
        if ret:
            cv2.imwrite(camera_raw_save_image, frame)
            print(f"Captured image from camera 1 and saved to: {camera_raw_save_image}")
            raw_image = camera_raw_save_image
        else:
            print("Camera 1 is open, but failed to read frame. Using original image instead.")
            raw_image = backup_image
    else:
        print("Camera 1 not detected. Using original image instead.")
        raw_image = backup_image
    # ============================
    # Block 1 — Pick 4 corner markers (human-in-the-loop)
    # Purpose: Open an interactive window, let the user drag/select ROIs for
    #          TL → TR → BL → BR, and compute the circle centers inside each ROI.
    # Inputs : Raw camera image path
    # Output : corners = {'top_left':(x,y), 'top_right':(x,y),
    #                     'bottom_left':(x,y), 'bottom_right':(x,y)} or None
    # ============================
    CornerF = CornerFinder()
    corners = CornerF.pick_corners_with_roi(os.path.join(images_folder, raw_image))
    if corners is None:
        print("Cancelled or failed.")
        return
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
    out_file = proj.warp_from_image(os.path.join(images_folder, raw_image), corners)
    print("Saved to:", out_file)

    # ============================
    # Block 3 — Manual threshold tuning (preview only)
    # Purpose: Interactively sweep a global threshold and preview the binary mask.
    # Action : Adjust the slider; press S to save the preview mask as PNG.
    # Note   : This step is for visual tuning; the pipeline below still
    #          produces its own binary using a fixed threshold.
    # ============================
    if THRESHOLD_TUNER_ENABLE_FLAG:
        tuner = ThresholdTuner()
        bw, T_final, invert_flag = tuner.run(
            os.path.join(images_folder, "2_projected.png"),
            out_path=os.path.join(images_folder, "safe_zone_binary_manual.png")
        )
        # 供后续计算使用
        threshold_selected_manually = T_final
    else:
        threshold_selected_manually = 125

    # ============================
    # Block 4 — Fixed-threshold binarization + morphology
    # Purpose: Produce a clean “white = free space” mask from the rectified image.
    # Config : threshold=160 (global), closing (kernel=5, iter=1),
    #          keep only the largest white component, fill interior holes.
    # Output : Binary mask saved to disk.
    # ============================
    sz = SafeZone(expect_size=(2160,2160), auto_resize=False,
              threshold=threshold_selected_manually, close_kernel=5, close_iter=1,
              keep_largest=True, fill_holes=True)
    # out_file = sz.binarize("MicromouseMazeCamera_projected_2160x2160.png", "./safe_zone_binary.png")
    out_file = sz.binarize(os.path.join(images_folder, "2_projected.png"), os.path.join(images_folder, "3_safe_zone_binary.png"))


    # generate the grid part
    gg = Grid_Graph(os.path.join(images_folder, "3_safe_zone_binary.png"), rows=9, cols=9, connectivity=4)
    G = gg.build()
    kept, removed = gg.filter_edges_by_mask(os.path.join(images_folder, "3_safe_zone_binary.png"),
                                            white_thresh=200,      
                                            line_thickness=3,      
                                            require_ratio=1.0)     
    print(f"kept={kept}, removed={removed}")
    save_path = gg.render()   
    print("Saved to:", save_path)

    # delete those nodes inside continuous maze
    cg = gg.graph
    for i in range(2, 7):
        for j in range(2, 7):
            cg.remove_node(cg.find_node_id(i, j))


    # 读取图片
    img = cv2.imread(os.path.join(images_folder, "3_safe_zone_binary.png"))

    if img is None:
        raise IOError(f"无法读取图像: {image_path}")

    # 定义裁剪区域 (y1:y2, x1:x2)
    x1, y1 = 468, 468
    x2, y2 = 1638, 1638

    # 裁剪
    cropped = img[y1:y2, x1:x2]
    cv2.imwrite(os.path.join(images_folder, "6_cropped.png"), cropped)

    # 生成unsafe_zone
    continuous_original_bin_image = cv2.imread(os.path.join(images_folder, "6_cropped.png"), cv2.IMREAD_COLOR) 

    gray_map = cv2.cvtColor(continuous_original_bin_image, cv2.COLOR_BGR2GRAY)
    _, binary_map = cv2.threshold(gray_map, 127, 255, cv2.THRESH_BINARY_INV)
    
    unsafe_kernel_size = 30 # obtained by intuition, could be too large
    unsafe_iterations = 3

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (unsafe_kernel_size, unsafe_kernel_size))
    dilated_map = cv2.dilate(binary_map, kernel, iterations=unsafe_iterations)

    H, W = binary_map.shape
    cropped_unsafe = np.ones((H, W, 3), dtype=np.uint8) * 255

    dilated_mask = dilated_map == 255
    obstacle_mask = binary_map == 255

    cropped_unsafe[dilated_mask] = [0, 0, 255]    # Red
    cropped_unsafe[obstacle_mask] = [0, 0, 0]     # Black

    # cropped_unsafe = cv2.cvtColor(cropped_unsafe, cv2.COLOR_BGR2RGB)
    cv2.imwrite(os.path.join(images_folder, "7_cropped_unsafe.png"), cropped_unsafe)

if __name__ == '__main__':
    main()