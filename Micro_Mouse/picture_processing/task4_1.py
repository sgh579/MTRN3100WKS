import cv2
import numpy as np
from collections import deque
import networkx as nx
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
        #[x, y]
    start = []
    end = []


    ### using ind assignment


    def bfs(graph, start, end):
        if start not in graph.nodes() or end not in graph.nodes():
            return []
        
        queue = deque([start])
        visited = set([start])
        parent = {start: None}
        
        while queue:
            current_node_id = queue.popleft()
            
            if current_node_id == end:
                path = []
                while current_node_id is not None:
                    path.append(current_node_id)
                    current_node_id = parent[current_node_id]
                path.reverse()
                return path
            
            adjacent_nodes = []
            for (node1, node2) in graph.edges.keys():
                if node1 == current_node_id and node2 not in visited:
                    adjacent_nodes.append(node2)
                elif node2 == current_node_id and node1 not in visited:
                    adjacent_nodes.append(node1)

            for adjacent_node in adjacent_nodes:
                if adjacent_node not in visited:
                    visited.add(adjacent_node)
                    queue.append(adjacent_node)
                    parent[adjacent_node] = current_node_id
        
        return []


    path = bfs(bfs_graph,start,end)

    ## convert this to instructions using
    # commands = path_to_commands(path) # defined below


    ## Claude Answer ##

    def bfs_shortest_path(graph, start, finish):
        """
        Find the shortest path between two nodes using BFS algorithm.
        
        Args:
            graph: NetworkX graph object (from your Grid_Graph.graph)
            start: tuple (row, col) - starting position
            finish: tuple (row, col) - ending position
        
        Returns:
            list: Path as list of (row, col) tuples, or None if no path exists
        """
        if start not in graph.nodes or finish not in graph.nodes:
            print(f"Error: Start {start} or finish {finish} not in graph")
            return None
        
        if start == finish:
            return [start]
        
        # BFS implementation
        queue = deque([(start, [start])])  # (current_node, path_to_current)
        visited = {start}
        
        while queue:
            current_node, path = queue.popleft()
            
            # Check all neighbors of current node
            for neighbor in graph.neighbors(current_node):
                if neighbor not in visited:
                    new_path = path + [neighbor]
                    
                    if neighbor == finish:
                        return new_path
                    
                    visited.add(neighbor)
                    queue.append((neighbor, new_path))
        
        return None  # No path found

    def path_to_commands(path, cell_size_mm=18):
        """
        Convert a path of grid coordinates to movement commands for Arduino.
        
        Args:
            path: list of (row, col) tuples representing the path
            cell_size_mm: size of each grid cell in millimeters (default 18mm)
        
        Returns:
            str: Command string in format like "f18|o90|f18|f18|o0|"
        """
        if not path or len(path) < 2:
            return ""
        
        commands = []
        current_direction = 0  # 0=North, 90=East, 180=South, 270=West
        
        for i in range(len(path) - 1):
            current = path[i]
            next_pos = path[i + 1]
            
            # Calculate direction needed
            row_diff = next_pos[0] - current[0]
            col_diff = next_pos[1] - current[1]
            
            # Determine target direction
            if row_diff == -1 and col_diff == 0:  # Moving up (north)
                target_direction = 0
            elif row_diff == 1 and col_diff == 0:  # Moving down (south)
                target_direction = 180
            elif row_diff == 0 and col_diff == 1:  # Moving right (east)
                target_direction = 90
            elif row_diff == 0 and col_diff == -1:  # Moving left (west)
                target_direction = 270
            else:
                print(f"Warning: Invalid move from {current} to {next_pos}")
                continue
            
            # Calculate rotation needed
            rotation_needed = (target_direction - current_direction) % 360
            if rotation_needed > 180:
                rotation_needed -= 360
            
            # Add rotation command if needed
            if rotation_needed != 0:
                new_direction = (current_direction + rotation_needed) % 360
                commands.append(f"o{new_direction}")
                current_direction = new_direction
            
            # Add forward movement command
            commands.append(f"f{cell_size_mm}")
        
        return "|".join(commands) + "|"

    # Example usage with your grid graph
def find_path_and_generate_commands(bfs_graph, start, finish):
    """
    Complete function to find path and generate Arduino commands.
    
    Args:
        bfs_graph: Your Grid_Graph.graph object
        start: tuple (row, col) - starting position
        finish: tuple (row, col) - ending position
    
    Returns:
        tuple: (path, commands) where path is list of coordinates and commands is string
    """
    print(f"Finding path from {start} to {finish}")
    
    # Find shortest path using BFS
    path = bfs_shortest_path(bfs_graph, start, finish)
    
    if path is None:
        print("No path found!")
        return None, ""
    
    print(f"Path found: {path}")
    print(f"Path length: {len(path) - 1} steps")
    
    # Generate movement commands
    commands = path_to_commands(path)
    print(f"Arduino commands: {commands}")
    
    return path, commands
