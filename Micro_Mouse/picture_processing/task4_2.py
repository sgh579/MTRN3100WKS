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
import random
from tools.continuous_graph import draw_graph_on_image, process_continuous_maze


THRESHOLD_TUNER_ENABLE_FLAG = True

out_path_1 = os.path.join(IMAGE_FOLER, '1_corner.png')
out_path_2 = os.path.join(IMAGE_FOLER, '2_projected.png')
out_path_3 = os.path.join(IMAGE_FOLER, '3_safe_zone_binary.png')
out_path_4 = os.path.join(IMAGE_FOLER, '4_grid_graph.png')
# out_path_5 = os.path.join(IMAGE_FOLER, '1_corner.png')
out_path_6 = os.path.join(IMAGE_FOLER, '6_cropped.png')
out_path_7 = os.path.join(IMAGE_FOLER, '7_cropped_unsafe.png')
out_path_8 = os.path.join(IMAGE_FOLER, '8_combined_continuous_maze.png')
out_path_9 = os.path.join(IMAGE_FOLER, '9_mixed_graph_on_img.png')

def main():
    print(' ############# 4.2 starts #############')

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
    corners = CornerF.pick_corners_by_clicks(os.path.join(images_folder, raw_image))
    if corners is None:
        print("Cancelled or failed.")
        return


    # ============================
    # Block 2 — Perspective warp (projection)
    # Purpose: Map the original image to a fixed-size top-down view.
    # Config : out_size=(2160,2160), margin=120 ⇒ TL maps to (120,120),
    #          other 3 corners placed symmetrically.
    # Output : A rectified image saved to disk; returns the output path.
    # ============================
    proj = Projection(out_size=(2160,2160), margin=120)
    output_img = proj.warp_from_image(os.path.join(images_folder, raw_image), corners)
    cv2.imwrite(out_path_2, output_img)


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

    output_img = sz.binarize(out_path_2)
    cv2.imwrite(out_path_3, output_img)


    # generate the grid part
    gg = Grid_Graph(out_path_3, rows=9, cols=9, connectivity=4)
    G = gg.build()
    kept, removed = gg.filter_edges_by_mask(os.path.join(images_folder, "3_safe_zone_binary.png"),
                                            white_thresh=200,      
                                            line_thickness=3,      
                                            require_ratio=1.0)     
    print(f"kept={kept}, removed={removed}")
    output_img = gg.render()   
    cv2.imwrite(out_path_4, output_img)

    # move it upside
    continuous_maze_top_left_cell = (2, 2)
    continuous_maze_bottom_right_cell = (continuous_maze_top_left_cell[0]+4, continuous_maze_top_left_cell[1]+4)

    # delete those nodes inside continuous maze
    cg = gg.graph
    for i in range(continuous_maze_top_left_cell[0], continuous_maze_bottom_right_cell[0]+1):
        for j in range(continuous_maze_top_left_cell[1], continuous_maze_bottom_right_cell[1]+1):
            cg.remove_node(cg.find_node_id(i, j))


    paths = process_continuous_maze(
        graph=cg,
        in_image_path=out_path_3,
        out_crop_path=out_path_6,
        out_unsafe_path=out_path_7,
        out_combined_path=out_path_8,
        top_left_cell=continuous_maze_top_left_cell,
        bottom_right_cell=continuous_maze_bottom_right_cell,
        cell_px=240,
        unsafe_kernel_size=30,
        unsafe_iterations=3,
        node_margin_px=50,
        division=20,
        fully_connect=True,     # 大图请改 False，或替换成 k-NN
        edge_weight=10.0
    )

    print("Done:", paths)

    output_img = draw_graph_on_image(cg, out_path_8)
    cv2.imwrite(out_path_9, output_img)


if __name__ == '__main__':
    main()

