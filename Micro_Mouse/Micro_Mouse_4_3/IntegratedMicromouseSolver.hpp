#include <math.h>

// Maze settings
#define CELL_SIZE 180
#define MAX_MAZE_WIDTH 9 // Reduced for Arduino Nano memory
#define MAX_MAZE_HEIGHT 9
#define MAX_QUEUE_SIZE 69
#define MAX_STACK_SIZE 69
#define TOTAL_CELLS 69

// Control thresholds
#define CMD_COMPLETE_STABLE_CYCLES 20
#define POSITION_ERROR_THRESHOLD 5.0f
#define ANGLE_ERROR_THRESHOLD 10.0f
#define MOTOR_OUTPUT_THRESHOLD 40
#define YAW_OUTPUT_THRESHOLD 30.0f
#define WALL_DISTANCE_THRESHOLD 80.0f
#define DESIRED_WALL_DISTANCE 50.0f

// Masks
#define NORTH_MASK 0b0001
#define EAST_MASK 0b0010
#define SOUTH_MASK 0b0100
#define WEST_MASK 0b1000

// Direction constants
enum Direction
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// Maze solving states
enum MazeState
{
    EXPLORING,
    MOVING_TO_TARGET,
    COMPLETED
};

// Cell structure
struct Cell
{
    unsigned char walls = 0;
    bool visited = false;
    int8_t flood_value = -1;

    bool hasWall(Direction dir) const
    {
        switch (dir)
        {
        case NORTH:
            return walls & NORTH_MASK;
        case EAST:
            return walls & EAST_MASK;
        case SOUTH:
            return walls & SOUTH_MASK;
        case WEST:
            return walls & WEST_MASK;
        default:
            return true;
        }
    }

    void setWall(Direction dir, bool hasWall)
    {
        uint8_t mask = 0;
        switch (dir)
        {
        case NORTH:
            mask = NORTH_MASK;
            break;
        case EAST:
            mask = EAST_MASK;
            break;
        case SOUTH:
            mask = SOUTH_MASK;
            break;
        case WEST:
            mask = WEST_MASK;
            break;
        }

        if (hasWall)
        {
            walls |= mask; // set bit
        }
        else
        {
            walls &= ~mask; // clear bit
        }
    }
};

struct Position
{
    uint8_t x, y;
    Position(uint8_t x = 0, uint8_t y = 0) : x(x), y(y) {}
    bool operator==(const Position &other) const
    {
        return x == other.x && y == other.y;
    }
};

// Simple queue for BFS
class SimpleQueue
{
private:
    Position data[MAX_QUEUE_SIZE];
    int front_idx, rear_idx, size;

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
    Position front() const { return data[front_idx]; }
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

// Simple stack for DFS
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
            data[++top_idx] = pos;
    }
    Position top() const { return empty() ? Position(0, 0) : data[top_idx]; }
    void pop()
    {
        if (!empty())
            top_idx--;
    }
    void clear() { top_idx = -1; }
};

// Integrated Micromouse Solver
class IntegratedMicromouseSolver
{
private:
    int maze_width = MAX_MAZE_WIDTH;
    int maze_height = MAX_MAZE_HEIGHT;
    Cell maze[MAX_MAZE_HEIGHT][MAX_MAZE_WIDTH];
    Position current_grid_pos;
    Position start_grid_pos;
    Position target_grid_pos;
    Direction current_direction = NORTH;
    int num_visited = 0;

    // // Maze array
    // char displayMaze[10][17] = {
    //     {' ', ' ', ' ', ' ', ' ', '_', ' ', '_', ' ', '_', ' ', '_', ' ', ' ', ' ', ' ', ' '}, // row 0
    //     {' ', ' ', ' ', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', ' ', ' ', ' '}, // row 1
    //     {' ', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', ' '}, // row 2
    //     {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'}, // row 3
    //     {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'}, // row 4
    //     {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'}, // row 5
    //     {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'}, // row 6
    //     {'|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|'}, // row 7
    //     {' ', ' ', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', '_', '|', ' ', ' '}, // row 8
    //     {' ', ' ', ' ', ' ', '|', '_', '|', '_', '|', '_', '|', '_', '|', ' ', ' ', ' ', ' '}  // row 9
    // };

    MazeState state = EXPLORING;
    SimpleStack backtrack_stack;
    SimpleQueue bfs_queue;

    // Movement state for integration with your controller
    bool movement_in_progress = false;
    char current_movement_command;
    float movement_target_value;
    int stable_cycles;

public:
    IntegratedMicromouseSolver(Position start, Position target)
        : current_grid_pos(start), start_grid_pos(start), target_grid_pos(target),
          current_movement_command('\0'),
          movement_target_value(0), stable_cycles(0)
    {
        // TODO: fix for actual shape
        // Initialize all cells
        for (int y = 0; y < MAX_MAZE_HEIGHT; y++)
        {
            for (int x = 0; x < MAX_MAZE_WIDTH; x++)
            {
                maze[y][x] = Cell();
            }
        }

        // Set boundary walls
        // TODO: ADJUST FOR ACTUAL MAP
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

    // Convert real-world position to grid coordinates
    Position worldToGrid(float x_mm, float y_mm)
    {
        int grid_x = (int)(x_mm / CELL_SIZE);
        int grid_y = (int)(y_mm / CELL_SIZE);
        return Position(grid_x, grid_y);
    }

    // Update maze with sensor readings
    void updateMazeFromSensors(int left_dist, int front_dist, int right_dist)
    {
        Position pos = current_grid_pos;

        // Detect walls based on sensor readings
        bool front_wall = (front_dist < WALL_DISTANCE_THRESHOLD);
        bool left_wall = (left_dist < WALL_DISTANCE_THRESHOLD);
        bool right_wall = (right_dist < WALL_DISTANCE_THRESHOLD);

        // Set walls in maze
        setWallBidirectional(pos, current_direction, front_wall);

        Direction left_dir = static_cast<Direction>((current_direction + 3) % 4);
        setWallBidirectional(pos, left_dir, left_wall);

        Direction right_dir = static_cast<Direction>((current_direction + 1) % 4);
        setWallBidirectional(pos, right_dir, right_wall);
    }

    void setWallBidirectional(Position pos, Direction dir, bool hasWall)
    {
        if (!isValidPosition(pos))
            return;

        maze[pos.y][pos.x].setWall(dir, hasWall);

        Position adj = getAdjacentPosition(pos, dir);
        if (isValidPosition(adj))
        {
            Direction opposite_dir = static_cast<Direction>((dir + 2) % 4);
            maze[adj.y][adj.x].setWall(opposite_dir, hasWall);
        }
    }

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

    bool isValidPosition(Position pos)
    {
        return pos.x >= 0 && pos.x < maze_width && pos.y >= 0 && pos.y < maze_height;
    }

    // Check if current movement command is complete
    bool isMovementComplete(float curr_x, float curr_y, float curr_angle,
                            int motor1_output, int motor2_output)
    {
        if (!movement_in_progress)
            return true;

        bool completed = false;

        if (current_movement_command == 'f')
        {
            float delta_x = curr_x - movement_start_x;
            float delta_y = curr_y - movement_start_y;
            float distance_traveled = sqrt(delta_x * delta_x + delta_y * delta_y);
            float position_error = movement_target_value - distance_traveled;

            if (abs(position_error) <= POSITION_ERROR_THRESHOLD &&
                abs(motor1_output) < MOTOR_OUTPUT_THRESHOLD &&
                abs(motor2_output) < MOTOR_OUTPUT_THRESHOLD)
            {
                stable_cycles++;
            }
            else
            {
                stable_cycles = 0;
            }
            completed = (stable_cycles >= CMD_COMPLETE_STABLE_CYCLES);
        }
        else if (current_movement_command == 'o')
        {
            float angle_error = angleDifference(movement_target_value, curr_angle);

            if (angle_error <= ANGLE_ERROR_THRESHOLD &&
                abs(motor1_output) < YAW_OUTPUT_THRESHOLD &&
                abs(motor2_output) < YAW_OUTPUT_THRESHOLD)
            {
                stable_cycles++;
            }
            else
            {
                stable_cycles = 0;
            }
            completed = (stable_cycles >= CMD_COMPLETE_STABLE_CYCLES);
        }

        if (completed)
        {
            movement_in_progress = false;
            stable_cycles = 0;
        }

        return completed;
    }

    // Start a movement command
    void startMovement(char command, float value, float start_x, float start_y)
    {
        current_movement_command = command;
        movement_target_value = value;
        movement_start_x = start_x;
        movement_start_y = start_y;
        movement_in_progress = true;
        stable_cycles = 0;
    }

    // Main maze solving step - call this from your main loop
    bool processMazeStep(float curr_x, float curr_y, float curr_angle,
                         int left_sensor, int front_sensor, int right_sensor,
                         int motor1_output, int motor2_output,
                         char &next_command, float &next_value)
    {

        // Update current grid position
        current_grid_pos = worldToGrid(curr_x, curr_y);

        // Update maze with sensor data
        updateMazeFromSensors(left_sensor, front_sensor, right_sensor);

        // Mark current cell as visited
        if (!maze[current_grid_pos.y][current_grid_pos.x].visited) num_visited++;
        maze[current_grid_pos.y][current_grid_pos.x].visited = true;

        // Check if movement is complete before planning next move
        if (movement_in_progress)
        {
            return isMovementComplete(curr_x, curr_y, curr_angle, motor1_output, motor2_output);
        }

        // Plan next move based on current state
        switch (state)
        {
        case EXPLORING:
            return planExplorationMove(curr_x, curr_y, next_command, next_value);

        case MOVING_TO_TARGET:
            return planOptimalMove(next_command, next_value);

        case COMPLETED:
            next_command = '\0';
            return false;
        }

        return false;
    }

    MazeState getState() const { return state; }
    bool isMovementInProgress() const { return movement_in_progress; }
    int getPercentage() const { return static_cast<int>(100.0f * num_visited / TOTAL_CELLS); }

    void getDisplayMaze(char ret[10][17])
    {
        // initialise
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 17; j++) {
                ret[i][j] = ' ';
            }
        }

        for (int y = 0; y < MAX_MAZE_HEIGHT; y++) {
            ret[y][16] = '\0'; 
            for (int x = 0; x < MAX_MAZE_WIDTH; x++) {
                int row, col;
                Cell curr = maze[x][y];
                if (curr.hasWall(NORTH))
                {
                    row = y;
                    col = 2 * x + 1;
                    ret[row][col] = '_';
                }
                if (curr.hasWall(SOUTH))
                {
                    row = y + 1;
                    col = 2 * x + 1;
                    ret[row][col] = '_';
                }
                if (curr.hasWall(EAST))
                {
                    row = y + 1;
                    col = 2 * x + 2;
                    ret[row][col] = ' ';
                }
                if (!curr.hasWall(WEST))
                {
                    row = y + 1;
                    col = 2 * x;
                    ret[row][col] = ' ';
                }
            }
        }
    }

private:
    float movement_start_x, movement_start_y;

    bool isExplorationComplete()
    {
        if (num_visited == TOTAL_CELLS) return true; 

        // Check if all reachable cells are visited
        bool reachable[MAX_MAZE_HEIGHT][MAX_MAZE_WIDTH];

        for (int y = 0; y < MAX_MAZE_HEIGHT; y++)
        {
            for (int x = 0; x < MAX_MAZE_WIDTH; x++)
            {
                reachable[y][x] = false;
            }
        }

        bfs_queue.clear();
        bfs_queue.push(start_grid_pos);
        reachable[start_grid_pos.y][start_grid_pos.x] = true;

        while (!bfs_queue.empty())
        {
            Position current = bfs_queue.front();
            bfs_queue.pop();

            for (int dir = 0; dir < 4; dir++)
            {
                Direction direction = static_cast<Direction>(dir);

                if (maze[current.y][current.x].hasWall(direction))
                    continue;

                Position next = getAdjacentPosition(current, direction);
                if (!isValidPosition(next) || reachable[next.y][next.x])
                    continue;

                if (!maze[next.y][next.x].visited) return false;
                bfs_queue.push(next);
            }
        }

        return true;
    }

    int getUnvisitedNeighbors(Position pos, Direction *unvisited_dirs)
    {
        int count = 0;
        for (int dir = 0; dir < 4; dir++)
        {
            Direction direction = static_cast<Direction>(dir);
            if (maze[pos.y][pos.x].hasWall(direction))
                continue;

            Position next = getAdjacentPosition(pos, direction);
            if (!isValidPosition(next))
                continue;

            if (!maze[next.y][next.x].visited)
            {
                unvisited_dirs[count++] = direction;
            }
        }
        return count;
    }

    bool planExplorationMove(float curr_x, float curr_y, char &next_command, float &next_value)
    {
        if (isExplorationComplete()) {
            state = MOVING_TO_TARGET;
            return planOptimalMove(next_command, next_value);
        }

        Direction unvisited_dirs[4];
        int unvisited_count = getUnvisitedNeighbors(current_grid_pos, unvisited_dirs);

        if (unvisited_count > 0) {
            // Move to unvisited neighbor
            backtrack_stack.push(current_grid_pos);
            Direction target_dir = unvisited_dirs[0];

            planMoveToDirection(target_dir, curr_x, curr_y, next_command, next_value);
        }
        else
        {
            // Backtrack
            if (!backtrack_stack.empty())
            {
                Position target_pos = backtrack_stack.top();
                backtrack_stack.pop();
                planMoveToGridPosition(target_pos, next_command, next_value);
            }
            else
            {
                // TODO: RETURN TO START FIRST, THEN GO TO TARGET
                // Exploration complete
                state = MOVING_TO_TARGET;
                return planOptimalMove(next_command, next_value);
            }
        }

        return true;
    }

    bool planOptimalMove(char &next_command, float &next_value)
    {
        if (current_grid_pos == target_grid_pos)
        {
            state = COMPLETED;
            return false;
        }

        // Run flood fill to find optimal direction
        floodFill();
        Direction best_dir = getBestDirection();

        planMoveToDirection(best_dir, 0, 0, next_command, next_value); // x,y not used for direction planning
        return true;
    }

    void planMoveToDirection(Direction target_dir, float curr_x, float curr_y,
                             char &next_command, float &next_value)
    {
        // Calculate turn needed
        if (current_direction == target_dir)
        {
            // Go straight - move one cell
            next_command = 'f';
            next_value = CELL_SIZE;
            startMovement(next_command, next_value, curr_x, curr_y);

            // Update position and direction
            current_grid_pos = getAdjacentPosition(current_grid_pos, current_direction);
        }
        else
        {
            // Turn first
            switch (target_dir)
            {
            case NORTH:
                next_value = 0;
                break; // Turn right
            case EAST:
                next_value = 270;
                break; // Turn around
            case SOUTH:
                next_value = 180;
                break; // Turn left
            case WEST:
                next_value = 90;
                break; // Turn left
            }

            next_command = 'o';
            startMovement(next_command, next_value, curr_x, curr_y);

            current_direction = target_dir;
        }
    }

    void planMoveToGridPosition(Position target, char &next_command, float &next_value)
    {
        // Simple pathfinding - move toward target using Manhattan distance
        Direction best_dir = NORTH;
        int min_distance = 32767;

        for (int dir = 0; dir < 4; dir++)
        {
            Direction direction = static_cast<Direction>(dir);

            if (maze[current_grid_pos.y][current_grid_pos.x].hasWall(direction))
                continue;

            Position next = getAdjacentPosition(current_grid_pos, direction);
            if (!isValidPosition(next))
                continue;

            int distance = abs(next.x - target.x) + abs(next.y - target.y);
            if (distance < min_distance)
            {
                min_distance = distance;
                best_dir = direction;
            }
        }

        planMoveToDirection(best_dir, 0, 0, next_command, next_value);
    }

    void floodFill()
    {
        // Reset flood values
        for (int y = 0; y < maze_height; y++)
        {
            for (int x = 0; x < maze_width; x++)
            {
                maze[y][x].flood_value = -1;
            }
        }

        bfs_queue.clear();
        bfs_queue.push(target_grid_pos);
        maze[target_grid_pos.y][target_grid_pos.x].flood_value = 0;

        while (!bfs_queue.empty())
        {
            Position current = bfs_queue.front();
            bfs_queue.pop();

            int current_value = maze[current.y][current.x].flood_value;

            for (int dir = 0; dir < 4; dir++)
            {
                Direction direction = static_cast<Direction>(dir);

                if (maze[current.y][current.x].hasWall(direction))
                    continue;

                Position next = getAdjacentPosition(current, direction);
                if (!isValidPosition(next))
                    continue;
                if (maze[next.y][next.x].flood_value != -1)
                    continue;

                maze[next.y][next.x].flood_value = current_value + 1;
                bfs_queue.push(next);
            }
        }
    }

    Direction getBestDirection()
    {
        int min_value = 32767;
        Direction best_dir = current_direction;

        for (int dir = 0; dir < 4; dir++)
        {
            Direction direction = static_cast<Direction>(dir);

            if (maze[current_grid_pos.y][current_grid_pos.x].hasWall(direction))
                continue;

            Position next = getAdjacentPosition(current_grid_pos, direction);
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

    float getCurrentAngle()
    {
        return fmod(current_direction * 90.0f, 360.0f);
    }

    float normalizeAngle(float angle) const
    {
        float result = fmod(angle, 360.0f);
        if (result < 0)
            result += 360.0f;
        return result;
    }

    float angleDifference(float a, float b) const
    {
        float diff = normalizeAngle(a - b);
        if (diff > 180.0f)
            diff = 360.0f - diff;
        return diff;
    }
};