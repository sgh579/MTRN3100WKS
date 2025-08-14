#new file: bfs_pathfinding.py

from collections import deque
import cv2
import numpy as np
from pathlib import Path
from typing import List, Tuple, Optional, Dict

class BFSPathfinder:
    def __init__(self, graph, grid_rows=9, grid_cols=9):
        """
        Initialize BFS pathfinder
        
        Args:
            graph: The Grid_Graph.graph object (Graph class instance)
            grid_rows: Number of rows in the grid
            grid_cols: Number of columns in the grid
        """
        self.graph = graph
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        
    def bfs_path(self, start_grid_pos: Tuple[int, int], end_grid_pos: Tuple[int, int]) -> List[int]:
        """
        Find shortest path using BFS
        
        Args:
            start_grid_pos: (grid_x, grid_y) starting position
            end_grid_pos: (grid_x, grid_y) ending position
            
        Returns:
            List of node IDs representing the path, empty list if no path found
        """
        # Find node IDs for start and end positions
        start_node = self.graph.find_node_id(start_grid_pos[0], start_grid_pos[1])
        end_node = self.graph.find_node_id(end_grid_pos[0], end_grid_pos[1])
        
        if start_node is None:
            print(f"Start position {start_grid_pos} not found in graph")
            return []
        if end_node is None:
            print(f"End position {end_grid_pos} not found in graph")
            return []
        
        if start_node == end_node:
            return [start_node]
        
        # BFS implementation
        queue = deque([start_node])
        visited = {start_node}
        parent = {start_node: None}
        
        while queue:
            current = queue.popleft()
            
            if current == end_node:
                # Reconstruct path
                path = []
                while current is not None:
                    path.append(current)
                    current = parent[current]
                return path[::-1]  # Reverse to get start->end
            
            # Explore neighbors
            for neighbor in self.graph.edges[current].keys():
                if neighbor not in visited:
                    visited.add(neighbor)
                    parent[neighbor] = current
                    queue.append(neighbor)
        
        print(f"No path found from {start_grid_pos} to {end_grid_pos}")
        return []
    
    def path_to_commands(self, path: List[int], cell_size_mm: int = 180) -> str:
        """
        Convert path to motion commands
        
        Args:
            path: List of node IDs representing the path
            cell_size_mm: Size of each grid cell in mm (default 18mm for micromouse)
            
        Returns:
            Command string in format like "f18|o90|f18|f18|o0|"
        """
        if len(path) < 2:
            return ""
        
        commands = []
        current_direction = 0  # 0=North, 90=East, 180=South, 270=West
        
        for i in range(len(path) - 1):
            current_node = self.graph.nodes[path[i]]
            next_node = self.graph.nodes[path[i + 1]]
            
            # Calculate direction from current to next
            dx = next_node.grid_x - current_node.grid_x
            dy = next_node.grid_y - current_node.grid_y
            
            # Convert grid movement to direction
            if dx == 1 and dy == 0:      # Move right (East)
                target_direction = 270
            elif dx == -1 and dy == 0:   # Move left (West)
                target_direction = 90
            elif dx == 0 and dy == -1:   # Move up (North)
                target_direction = 0
            elif dx == 0 and dy == 1:    # Move down (South)
                target_direction = 180
            else:
                print(f"Warning: Invalid move from ({current_node.grid_x},{current_node.grid_y}) to ({next_node.grid_x},{next_node.grid_y})")
                continue
            
            # Calculate turn needed
            turn_angle = (target_direction - current_direction) % 360
            if turn_angle > 180:
                turn_angle -= 360  # Convert to -180 to +180 range
            
            # Add turn command if needed
            if turn_angle != 0:
                commands.append(f"o{target_direction}")
            
            # Add forward command
            commands.append(f"f{cell_size_mm}")
            
            current_direction = target_direction
        
        # Add final orientation command if needed
        commands.append("o0")  # Return to North orientation
        
        return "|".join(commands) + "|"
    
    def visualize_path(self, 
                      image_path: str, 
                      path: List[int], 
                      start_grid_pos: Tuple[int, int],
                      end_grid_pos: Tuple[int, int],
                      output_path: Optional[str] = None) -> str:
        """
        Visualize the path on the grid graph image
        
        Args:
            image_path: Path to the grid graph image
            path: List of node IDs representing the path
            start_grid_pos: Starting position for labeling
            end_grid_pos: Ending position for labeling
            output_path: Output path for the visualization
            
        Returns:
            Path to the saved visualization
        """
        # Load the existing grid graph image
        img = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if img is None:
            raise IOError(f"Failed to read image: {image_path}")
        
        canvas = img.copy()
        
        if len(path) < 2:
            print("Path too short to visualize")
            return image_path
        
        # Draw path as thick blue line
        path_color = (255, 0, 0)  # Blue in BGR
        path_thickness = 6
        
        for i in range(len(path) - 1):
            current_node = self.graph.nodes[path[i]]
            next_node = self.graph.nodes[path[i + 1]]
            
            cv2.line(canvas,
                    (int(current_node.pixel_x), int(current_node.pixel_y)),
                    (int(next_node.pixel_x), int(next_node.pixel_y)),
                    path_color, path_thickness, cv2.LINE_AA)
        
        # Mark start and end points
        start_node = self.graph.nodes[path[0]]
        end_node = self.graph.nodes[path[-1]]
        
        # Start point (green circle)
        cv2.circle(canvas, (int(start_node.pixel_x), int(start_node.pixel_y)), 
                  15, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.putText(canvas, "START", 
                   (int(start_node.pixel_x) - 25, int(start_node.pixel_y) - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        
        # End point (red circle)
        cv2.circle(canvas, (int(end_node.pixel_x), int(end_node.pixel_y)), 
                  15, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(canvas, "END", 
                   (int(end_node.pixel_x) - 15, int(end_node.pixel_y) - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
        
        # Save the visualization
        if output_path is None:
            p = Path(image_path)
            output_path = str(p.with_name(f"{p.stem}_path_visualization{p.suffix}"))
        
        cv2.imwrite(output_path, canvas)
        return output_path

# Add this to your main script after building the graph:
def run_pathfinding_example(bfs_graph, image_path: str):
    """
    Example function showing how to use the BFS pathfinder
    """
    # Initialize pathfinder
    pathfinder = BFSPathfinder(bfs_graph, grid_rows=9, grid_cols=9)
    
    # Define start and end positions (grid coordinates)
    start_pos = (1, 7)  # Grid position (x=1, y=7)
    end_pos = (7, 1)    # Grid position (x=7, y=1)
    
    print(f"Finding path from {start_pos} to {end_pos}")
    
    # Find path using BFS
    path = pathfinder.bfs_path(start_pos, end_pos)
    
    if not path:
        print("No path found!")
        return
    
    print(f"Path found with {len(path)} nodes:")
    for i, node_id in enumerate(path):
        node = bfs_graph.nodes[node_id]
        print(f"  {i+1}: Node {node_id} at grid ({node.grid_x}, {node.grid_y})")
    
    # Convert path to commands
    commands = pathfinder.path_to_commands(path)
    print(f"Motion commands: {commands}")
    
    # Visualize the path
    viz_path = pathfinder.visualize_path(image_path, path, start_pos, end_pos)
    print(f"Path visualization saved to: {viz_path}")
    
    return commands, viz_path

# Modified section to add to your main script:
# if __name__ == '__main__':
#     # ... your existing code up to building the graph ...
    
#     # After this line in your original code:
#     # bfs_graph = gg.graph # this is the graph to be used in bfs
    
#     # Add the BFS pathfinding:
#     print("\n" + "="*50)
#     print("RUNNING BFS PATHFINDING")
#     print("="*50)
    
#     # Run pathfinding example
#     commands, viz_path = run_pathfinding_example(bfs_graph, save_path)
    
#     print(f"\nFinal Arduino commands to copy: {commands}")
#     print(f"Path visualization: {viz_path}")