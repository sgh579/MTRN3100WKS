import cv2
import numpy as np
from tools.image_projection import CornerFinder, Projection
from tools.grid_graph import ThresholdTuner, SafeZone, DisplayGridOnImg, Grid_Graph

if __name__ == '__main__':
    # CornerF = CornerFinder()
    # corners = CornerF.pick_corners_with_roi("Micro_Mouse\\picture_processing\\MicromouseMazeCamera.jpg")
    # if corners is None:
    #     print("Cancelled or failed.")
    # else:
    #     print(corners)
    # proj = Projection(out_size=(2160,2160), margin=120)
    # out_file = proj.warp_from_image("Micro_Mouse\\picture_processing\\MicromouseMazeCamera.jpg", corners)
    # print("Saved to:", out_file)

    # tuner = ThresholdTuner()  # 如需固定窗口大小或禁用自适应，传参调整
    # bw = tuner.run("MicromouseMazeCamera_projected_2160x2160.png", out_path="./safe_zone_binary_manual.png")

    # sz = SafeZone(expect_size=(2160,2160), auto_resize=False,
    #           threshold=160, close_kernel=5, close_iter=1,
    #           keep_largest=True, fill_holes=True)
    # out_file = sz.binarize("MicromouseMazeCamera_projected_2160x2160.png", "./safe_zone_binary.png")

    # gridder = DisplayGridOnImg(rows=9, cols=9, color=(0, 0, 255), thickness=2, draw_border=False)
    # saved = gridder.draw_and_save("safe_zone_binary.png")  # 替换成你的图片路径
    # print("Saved:", saved)

    gg = Grid_Graph("safe_zone_binary.png", rows=9, cols=9, connectivity=4)
    G = gg.build()

    # 2) 用二值图（白=可通行）筛边，严格“全白”
    kept, removed = gg.filter_edges_by_mask("safe_zone_binary.png",
                                            white_thresh=200,      # 白阈值
                                            line_thickness=3,      # 检查线宽
                                            require_ratio=1.0)     # 严格全白
    print(f"kept={kept}, removed={removed}")

    # 3) 只画保留下来的边
    save_path = gg.render()   # 或 gg.filter_and_render("safe_zone_binary.png")
    print("Saved to:", save_path)


# read a standard maze image file



# create a gird_map object from the image

# it knows the start and goal on grid_map

# use BFS to generate a series of motion command to complete task 4.1 in format of extended cmd
# it is expected to be copied into the source code of arduino
# command list example: f18|o90|f18|f18|o0|