// Arduino Nano compatible micromouse implementation with full maze exploration
// No STL containers - uses fixed arrays and manual memory management

// Direction constants
enum Direction
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// Configuration constants - adjust based on your maze size and memory constraints
const int MAX_MAZE_WIDTH = 9;
const int MAX_MAZE_HEIGHT = 9;
const int MAX_QUEUE_SIZE = 256;
const int MAX_STACK_SIZE = 256;
const int WALL_THRESHOLD = 100;

// Cell structure to represent maze cells
struct Cell
{
    bool north_wall = false;
    bool east_wall = false;
    bool south_wall = false;
    bool west_wall = false;
    bool visited = false;
    int flood_value = -1;

    bool hasWall(Direction dir) const
    {
        switch (dir)
        {
        case NORTH:
            return north_wall;
        case EAST:
            return east_wall;
        case SOUTH:
            return south_wall;
        case WEST:
            return west_wall;
        default:
            return true;
        }
    }

    void setWall(Direction dir, bool hasWall)
    {
        switch (dir)
        {
        case NORTH:
            north_wall = hasWall;
            break;
        case EAST:
            east_wall = hasWall;
            break;
        case SOUTH:
            south_wall = hasWall;
            break;
        case WEST:
            west_wall = hasWall;
            break;
        }
    }
};

// Position structure
struct Position
{
    int x, y;
    Position(int x = 0, int y = 0) : x(x), y(y) {}
    bool operator==(const Position &other) const
    {
        return x == other.x && y == other.y;
    }
};

// Simple queue implementation using arrays
class SimpleQueue
{
private:
    Position data[MAX_QUEUE_SIZE];
    int front_idx;
    int rear_idx;
    int size;

public:
    SimpleQueue() : front_idx(0), rear_idx(0), size(0) {}

    bool empty() const { return size == 0; }
    bool full() const { return size == MAX_QUEUE_SIZE; }

    void push(Position pos)
    {
        if (!full())
        {
            data[rear_idx] = pos;
            rear_idx = (rear_idx + 1) % MAX_QUEUE_SIZE;
            size++;
        }
    }

    Position front() const
    {
        return data[front_idx];
    }

    void pop()
    {
        if (!empty())
        {
            front_idx = (front_idx + 1) % MAX_QUEUE_SIZE;
            size--;
        }
    }

    void clear()
    {
        front_idx = 0;
        rear_idx = 0;
        size = 0;
    }
};

// Simple stack implementation using arrays
class SimpleStack
{
private:
    Position data[MAX_STACK_SIZE];
    int top_idx;

public:
    SimpleStack() : top_idx(-1) {}

    bool empty() const { return top_idx == -1; }
    bool full() const { return top_idx == MAX_STACK_SIZE - 1; }

    void push(Position pos)
    {
        if (!full())
        {
            data[++top_idx] = pos;
        }
    }

    Position top() const
    {
        if (!empty())
            return data[top_idx];
        return Position(0, 0);
    }

    void pop()
    {
        if (!empty())
            top_idx--;
    }

    void clear()
    {
        top_idx = -1;
    }
};

class MicromouseSolver
{
private:
    const int maze_width = MAX_MAZE_WIDTH;
    const int maze_height = MAX_MAZE_HEIGHT;
    Cell maze[MAX_MAZE_HEIGHT][MAX_MAZE_WIDTH];
    Position current_pos;
    Position start_pos;
    Position target_pos;
    Direction current_direction = NORTH;

    // Maze array
    char displayMaze [10][17] = {
        {' ', ' ', ' ', ' ', ' ', '_', ' ', '_', ' ', '_', ' ', '_', ' ', ' ', ' ', ' ', ' '},   // row 0
        {' ', ' ', ' ', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', ' ', ' ', ' '},   // row 1
        {' ', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', ' '},   // row 2
        {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'},   // row 3
        {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'},   // row 4
        {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'},   // row 5
        {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'},   // row 6
        {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'},   // row 7
        {' ', ' ', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', ' ', ' '},   // row 8
        {' ', ' ', ' ', ' ', '|', '_', '|', '_', '|', '_', '|', '_', '|', ' ', ' ', ' ', ' '}    // row 9
    };

    // LIDAR sensor readings (in cm or your preferred unit)
    double left_sensor;
    double right_sensor;
    double front_sensor;

    // Exploration state
    bool exploration_complete = false;
    SimpleStack backtrack_stack;
    SimpleQueue bfs_queue;

public:
    MicromouseSolver(Position start, Position target)
        : current_pos(start), start_pos(start),
          target_pos(target), current_direction(NORTH)
    {
        // Initialize maze grid to default values
        for (int y = 0; y < MAX_MAZE_HEIGHT; y++)
        {
            for (int x = 0; x < MAX_MAZE_WIDTH; x++)
            {
                maze[y][x] = Cell(); // Initialize with default constructor
            }
        }

        // Set boundary walls
        initializeBoundaryWalls();
    }

    void initializeBoundaryWalls()
    {
        // Set boundary walls for the entire maze
        for (int y = 0; y < maze_height; y++)
        {
            for (int x = 0; x < maze_width; x++)
            {
                if (x == 0)
                    maze[y][x].setWall(WEST, true);
                if (x == maze_width - 1)
                    maze[y][x].setWall(EAST, true);
                if (y == 0)
                    maze[y][x].setWall(SOUTH, true);
                if (y == maze_height - 1)
                    maze[y][x].setWall(NORTH, true);
            }
        }
    }

    // Update sensor readings
    void updateSensorReadings(double left, double right, double front)
    {
        left_sensor = left;
        right_sensor = right;
        front_sensor = front;
    }

    // Detect walls based on LIDAR readings
    void detectWalls()
    {
        Position pos = current_pos;

        // Check front wall
        bool front_wall = (front_sensor < WALL_THRESHOLD);
        Direction front_dir = current_direction;
        setWallBidirectional(pos, front_dir, front_wall);

        // Check left wall
        bool left_wall = (left_sensor < WALL_THRESHOLD);
        Direction left_dir = static_cast<Direction>((current_direction + 3) % 4);
        setWallBidirectional(pos, left_dir, left_wall);

        // Check right wall
        bool right_wall = (right_sensor < WALL_THRESHOLD);
        Direction right_dir = static_cast<Direction>((current_direction + 1) % 4);
        setWallBidirectional(pos, right_dir, right_wall);
    }

    // Set wall bidirectionally (both cells)
    void setWallBidirectional(Position pos, Direction dir, bool hasWall)
    {
        if (!isValidPosition(pos))
            return;

        // Set wall in current cell
        maze[pos.y][pos.x].setWall(dir, hasWall);

        // Set corresponding wall in adjacent cell
        Position adj = getAdjacentPosition(pos, dir);
        if (isValidPosition(adj))
        {
            Direction opposite_dir = static_cast<Direction>((dir + 2) % 4);
            maze[adj.y][adj.x].setWall(opposite_dir, hasWall);
        }
    }

    // Get adjacent position in given direction
    Position getAdjacentPosition(Position pos, Direction dir)
    {
        switch (dir)
        {
        case NORTH:
            return Position(pos.x, pos.y + 1);
        case EAST:
            return Position(pos.x + 1, pos.y);
        case SOUTH:
            return Position(pos.x, pos.y - 1);
        case WEST:
            return Position(pos.x - 1, pos.y);
        default:
            return pos;
        }
    }

    // Check if position is valid
    bool isValidPosition(Position pos)
    {
        return pos.x >= 0 && pos.x < maze_width &&
               pos.y >= 0 && pos.y < maze_height;
    }

    // Check if all accessible cells have been visited
    bool isExplorationComplete()
    {
        // Use BFS to find all reachable cells from start position
        bool reachable[MAX_MAZE_HEIGHT][MAX_MAZE_WIDTH];

        // Initialize reachable array
        for (int y = 0; y < MAX_MAZE_HEIGHT; y++)
        {
            for (int x = 0; x < MAX_MAZE_WIDTH; x++)
            {
                reachable[y][x] = false;
            }
        }

        bfs_queue.clear();
        bfs_queue.push(start_pos);
        reachable[start_pos.y][start_pos.x] = true;

        while (!bfs_queue.empty())
        {
            Position current = bfs_queue.front();
            bfs_queue.pop();

            // Check all four directions
            for (int dir = 0; dir < 4; dir++)
            {
                Direction direction = static_cast<Direction>(dir);

                // Skip if there's a wall
                if (maze[current.y][current.x].hasWall(direction))
                    continue;

                Position next = getAdjacentPosition(current, direction);

                // Skip if invalid position or already marked reachable
                if (!isValidPosition(next) || reachable[next.y][next.x])
                    continue;

                reachable[next.y][next.x] = true;
                bfs_queue.push(next);
            }
        }

        // Check if all reachable cells have been visited
        for (int y = 0; y < maze_height; y++)
        {
            for (int x = 0; x < maze_width; x++)
            {
                if (reachable[y][x] && !maze[y][x].visited)
                {
                    return false; // Found an unvisited reachable cell
                }
            }
        }

        return true; // All reachable cells have been visited
    }

    // Get number of unvisited accessible neighbors
    int getUnvisitedNeighbors(Position pos, Direction *unvisited_dirs)
    {
        int count = 0;

        for (int dir = 0; dir < 4; dir++)
        {
            Direction direction = static_cast<Direction>(dir);

            // Skip if there's a wall
            if (maze[pos.y][pos.x].hasWall(direction))
                continue;

            Position next = getAdjacentPosition(pos, direction);

            // Skip if invalid position
            if (!isValidPosition(next))
                continue;

            // Add if unvisited
            if (!maze[next.y][next.x].visited)
            {
                unvisited_dirs[count++] = direction;
            }
        }

        return count;
    }

    // DFS-based exploration algorithm
    bool exploreStep()
    {
        // Update current cell as visited
        maze[current_pos.y][current_pos.x].visited = true;

        // Detect walls with current sensor readings
        detectWalls();

        // Check if exploration is complete
        if (isExplorationComplete())
        {
            exploration_complete = true;
            Serial.println("Full maze exploration completed!");
            return false;
        }

        // Get unvisited neighbors
        Direction unvisited_dirs[4];
        int unvisited_count = getUnvisitedNeighbors(current_pos, unvisited_dirs);

        if (unvisited_count > 0)
        {
            // Push current position for backtracking
            backtrack_stack.push(current_pos);

            // Choose first unvisited neighbor
            Direction next_dir = unvisited_dirs[0];

            // Move to the unvisited cell
            moveToDirection(next_dir);
        }
        else
        {
            // No unvisited neighbors, backtrack
            if (!backtrack_stack.empty())
            {
                Position backtrack_target = backtrack_stack.top();
                backtrack_stack.pop();

                // Find path back to backtrack target
                moveToPosition(backtrack_target);
            }
            else
            {
                // No more places to backtrack, exploration complete
                exploration_complete = true;
                Serial.println("Exploration completed - all accessible areas mapped!");
                return false;
            }
        }

        return true; // Continue exploration
    }

    // Move to a specific position (for backtracking)
    void moveToPosition(Position target)
    {
        // Simple pathfinding to target position using known walls
        while (!(current_pos == target))
        {
            Direction best_dir = NORTH;
            int min_distance = 32767; // Use int max for Arduino

            // Check all four directions
            for (int dir = 0; dir < 4; dir++)
            {
                Direction direction = static_cast<Direction>(dir);

                // Skip if there's a wall
                if (maze[current_pos.y][current_pos.x].hasWall(direction))
                    continue;

                Position next = getAdjacentPosition(current_pos, direction);

                // Skip if invalid position
                if (!isValidPosition(next))
                    continue;

                // Calculate Manhattan distance to target
                int distance = abs(next.x - target.x) + abs(next.y - target.y);

                if (distance < min_distance)
                {
                    min_distance = distance;
                    best_dir = direction;
                }
            }

            moveToDirection(best_dir);
        }
    }

    // Flood fill algorithm to calculate distances from target
    void floodFill()
    {
        // Reset all flood values
        for (int y = 0; y < maze_height; y++)
        {
            for (int x = 0; x < maze_width; x++)
            {
                maze[y][x].flood_value = -1;
            }
        }

        // BFS from target position using our simple queue
        bfs_queue.clear();
        bfs_queue.push(target_pos);
        maze[target_pos.y][target_pos.x].flood_value = 0;

        while (!bfs_queue.empty())
        {
            Position current = bfs_queue.front();
            bfs_queue.pop();

            int current_value = maze[current.y][current.x].flood_value;

            // Check all four directions
            for (int dir = 0; dir < 4; dir++)
            {
                Direction direction = static_cast<Direction>(dir);

                // Skip if there's a wall
                if (maze[current.y][current.x].hasWall(direction))
                    continue;

                Position next = getAdjacentPosition(current, direction);

                // Skip if invalid position
                if (!isValidPosition(next))
                    continue;

                // Skip if already processed
                if (maze[next.y][next.x].flood_value != -1)
                    continue;

                maze[next.y][next.x].flood_value = current_value + 1;
                bfs_queue.push(next);
            }
        }
    }

    // Find the best direction to move based on flood fill
    Direction getBestDirection()
    {
        int min_value = 32767; // Arduino-friendly int max
        Direction best_dir = current_direction;

        for (int dir = 0; dir < 4; dir++)
        {
            Direction direction = static_cast<Direction>(dir);

            // Skip if there's a wall
            if (maze[current_pos.y][current_pos.x].hasWall(direction))
                continue;

            Position next = getAdjacentPosition(current_pos, direction);

            // Skip if invalid position
            if (!isValidPosition(next))
                continue;

            int flood_val = maze[next.y][next.x].flood_value;
            if (flood_val != -1 && flood_val < min_value)
            {
                min_value = flood_val;
                best_dir = direction;
            }
        }

        return best_dir;
    }

    // Move robot to specified direction
    void moveToDirection(Direction target_dir)
    {
        // Calculate turn needed
        int turn = (target_dir - current_direction + 4) % 4;

        // Execute turn
        switch (turn)
        {
        case 0: // Go straight
            // No turn needed
            break;
        case 1: // Turn right
            turnRight();
            break;
        case 2: // Turn around
            turnRight();
            turnRight();
            break;
        case 3: // Turn left
            turnLeft();
            break;
        }

        // Move forward
        moveForward();
        current_direction = target_dir;
    }

    // Robot movement commands (implement these based on your hardware)
    void moveForward()
    {
        Position next = getAdjacentPosition(current_pos, current_direction);
        if (isValidPosition(next))
        {
            current_pos = next;
            Serial.print("Moving forward to (");
            Serial.print(current_pos.x);
            Serial.print(", ");
            Serial.print(current_pos.y);
            Serial.println(")");
        }
        // TODO: Implement actual motor control
        delay(100); // Small delay for robot movement
    }

    void turnLeft()
    {
        current_direction = static_cast<Direction>((current_direction + 3) % 4);
        Serial.println("Turning left");
        // TODO: Implement actual motor control
        delay(50); // Small delay for turning
    }

    void turnRight()
    {
        current_direction = static_cast<Direction>((current_direction + 1) % 4);
        Serial.println("Turning right");
        // TODO: Implement actual motor control
        delay(50); // Small delay for turning
    }

    // Get shortest path length from current position to target
    int getShortestPathLength()
    {
        // First run flood fill to calculate optimal distances
        floodFill();

        int path_length = 0;
        Position pos = current_pos;

        // Follow flood fill values to count path length
        while (!(pos == target_pos) && path_length < 1000) // Safety limit
        {
            path_length++;

            Direction best_dir = NORTH;
            int min_value = 32767;

            // Find direction with minimum flood value
            for (int dir = 0; dir < 4; dir++)
            {
                Direction direction = static_cast<Direction>(dir);

                if (maze[pos.y][pos.x].hasWall(direction))
                    continue;

                Position next = getAdjacentPosition(pos, direction);
                if (!isValidPosition(next))
                    continue;

                int flood_val = maze[next.y][next.x].flood_value;
                if (flood_val != -1 && flood_val < min_value)
                {
                    min_value = flood_val;
                    best_dir = direction;
                }
            }

            pos = getAdjacentPosition(pos, best_dir);
        }

        return path_length;
    }

    // Execute the shortest path to target
    void executeShortestPath()
    {
        if (!exploration_complete)
        {
            Serial.println("Cannot execute shortest path - exploration not complete!");
            return;
        }

        Serial.println("\n=== EXECUTING SHORTEST PATH ===");

        int path_length = getShortestPathLength();
        Serial.print("Shortest path calculated (");
        Serial.print(path_length);
        Serial.println(" steps)");

        // Execute the path by following flood fill values
        while (!(current_pos == target_pos))
        {
            // Run flood fill to get current distances
            floodFill();

            // Get best direction
            Direction best_dir = getBestDirection();

            // Move in best direction
            moveToDirection(best_dir);
        }

        Serial.println("Shortest path execution completed!");
    }

    // Return to start position after reaching target
    void returnToStart()
    {
        if (!exploration_complete)
        {
            Serial.println("Cannot return to start - exploration not complete!");
            return;
        }

        Serial.println("\n=== RETURNING TO START ===");

        // Navigate back to start using flood fill with start as target
        Position original_target = target_pos;
        target_pos = start_pos; // Temporarily change target

        executeShortestPath(); // Reuse the pathfinding logic

        target_pos = original_target; // Restore original target

        Serial.println("Successfully returned to start position!");
    }

    // Complete maze solving process with full exploration
    void solveMaze()
    {
        Serial.println("Starting full maze exploration...");
        Serial.print("Start: (");
        Serial.print(current_pos.x);
        Serial.print(", ");
        Serial.print(current_pos.y);
        Serial.println(")");
        Serial.print("Target: (");
        Serial.print(target_pos.x);
        Serial.print(", ");
        Serial.print(target_pos.y);
        Serial.println(")");

        // Phase 1: Full exploration
        Serial.println("\n=== EXPLORATION PHASE ===");
        bool exploring = true;
        int step_count = 0;

        while (exploring && step_count < 1000)
        { // Safety limit for Arduino
            exploring = exploreStep();
            step_count++;

            // Give some time between steps
            delay(10);
        }

        Serial.print("Exploration completed in ");
        Serial.print(step_count);
        Serial.println(" steps");

        // Print exploration statistics
        int visited_cells = 0;
        for (int y = 0; y < maze_height; y++)
        {
            for (int x = 0; x < maze_width; x++)
            {
                if (maze[y][x].visited)
                    visited_cells++;
            }
        }
        Serial.print("Visited ");
        Serial.print(visited_cells);
        Serial.print(" out of ");
        Serial.print(maze_width * maze_height);
        Serial.println(" total cells");

        // Phase 2: Navigate to target using shortest path
        if (exploration_complete)
        {
            executeShortestPath();

            // Optional: Return to start for competition format
            // returnToStart();
        }
        else
        {
            Serial.println("Warning: Exploration may not be complete!");
        }
    }

    // Print maze state (for debugging) - simplified for Arduino Serial
    void printMazeSimple()
    {
        Serial.println("Maze State (R=Robot, T=Target, .=Visited):");

        for (int y = maze_height - 1; y >= 0; y--)
        {
            for (int x = 0; x < maze_width; x++)
            {
                if (Position(x, y) == current_pos)
                    Serial.print("R");
                else if (Position(x, y) == target_pos)
                    Serial.print("T");
                else if (maze[y][x].visited)
                    Serial.print(".");
                else
                    Serial.print(" ");
            }
            Serial.println("");
        }
    }

    // Getter for exploration status
    bool isExplorationCompleted() const
    {
        return exploration_complete;
    }

    char getDisplayMaze[10][17] {
        for (int x=0;x<=size;x++) {
            for (int y=0;y<=size;y++) {
                int row, col;
                Cell curr = maze[x][y];
                if (!curr.hasWall(NORTH)) {
                    row = x;
                    col = 1+2y;
                    displayMaze[row][col]=' '
                }
                if (!curr.hasWall(SOUTH))  {
                    row = x + 1;
                    col = 1+2y;
                    displayMaze[row][col]=' '
                }
                if (!curr.hasWall(EAST))  {
                    row = x + 1;
                    col = 2y;
                    displayMaze[row][col]=' '
                }
                if (!curr.hasWall(WEST))  {
                    row = x + 1;
                    col = 2y;
                    displayMaze[row][col]=' '
                }

            }
        }
    }

    return displayMaze;
};
