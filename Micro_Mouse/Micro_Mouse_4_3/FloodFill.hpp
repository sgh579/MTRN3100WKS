// this is an AI implementation of 4.3 A lot of chnages need to be made. This could be the skelton as to how we approach it

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <cmath>

// bring in imports to run motors

// Direction constants
enum Direction
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

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

class MicromouseSolver
{
private:
    int maze_width;
    int maze_height;
    std::vector<std::vector<Cell>> maze;
    Position current_pos;
    Position target_pos;
    Direction current_direction;

    // LIDAR sensor readings (in cm or your preferred unit)
    double left_sensor;
    double right_sensor;
    double front_sensor;
    double wall_threshold; // Distance threshold to detect walls

public:
    MicromouseSolver(int width, int height, Position start, Position target)
        : maze_width(width), maze_height(height), current_pos(start),
          target_pos(target), current_direction(NORTH), wall_threshold(10.0)
    {

        // Initialize maze grid
        maze.resize(maze_height, std::vector<Cell>(maze_width));

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
        bool front_wall = (front_sensor < wall_threshold);
        Direction front_dir = current_direction;
        setWallBidirectional(pos, front_dir, front_wall);

        // Check left wall
        bool left_wall = (left_sensor < wall_threshold);
        Direction left_dir = static_cast<Direction>((current_direction + 3) % 4);
        setWallBidirectional(pos, left_dir, left_wall);

        // Check right wall
        bool right_wall = (right_sensor < wall_threshold);
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

        // BFS from target position
        std::queue<Position> q;
        q.push(target_pos);
        maze[target_pos.y][target_pos.x].flood_value = 0;

        while (!q.empty())
        {
            Position current = q.front();
            q.pop();

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

                // Skip if already processed with smaller value
                if (maze[next.y][next.x].flood_value != -1)
                    continue;

                maze[next.y][next.x].flood_value = current_value + 1;
                q.push(next);
            }
        }
    }

    // Find the best direction to move based on flood fill
    Direction getBestDirection()
    {
        int min_value = INT_MAX;
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

    // Main exploration algorithm (Right-hand rule with flood fill optimization)
    bool exploreStep()
    {
        // Update current cell as visited
        maze[current_pos.y][current_pos.x].visited = true;

        // Detect walls with current sensor readings
        detectWalls();

        // Update flood fill values
        floodFill();

        // Check if we've reached the target
        // TODO: ADJUST CONDITIONS TO LEAVE
        if (current_pos == target_pos)
        {
            std::cout << "Target reached during exploration!" << std::endl;
            return false; // Stop exploration
        }

        // Get best direction based on flood fill
        Direction best_dir = getBestDirection();

        // Move to the best direction
        moveToDirection(best_dir);

        return true; // Continue exploration
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
            std::cout << "Moving forward to (" << current_pos.x << ", " << current_pos.y << ")" << std::endl;
        }
        // TODO: Implement actual motor control
    }

    void turnLeft()
    {
        current_direction = static_cast<Direction>((current_direction + 3) % 4);
        std::cout << "Turning left" << std::endl;
        // TODO: Implement actual motor control
    }

    void turnRight()
    {
        current_direction = static_cast<Direction>((current_direction + 1) % 4);
        std::cout << "Turning right" << std::endl;
        // TODO: Implement actual motor control
    }

    // Get shortest path after exploration
    std::vector<Position> getShortestPath()
    {
        std::vector<Position> path;
        Position pos = current_pos;

        // Follow flood fill values to get shortest path
        while (!(pos == target_pos))
        {
            path.push_back(pos);

            Direction best_dir = NORTH;
            int min_value = INT_MAX;

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

        path.push_back(target_pos);
        return path;
    }

    // Complete maze solving process
    void solveMaze()
    {
        std::cout << "Starting maze exploration..." << std::endl;
        std::cout << "Start: (" << current_pos.x << ", " << current_pos.y << ")" << std::endl;
        std::cout << "Target: (" << target_pos.x << ", " << target_pos.y << ")" << std::endl;

        // Exploration phase
        bool exploring = true;
        int step_count = 0;

        while (exploring && step_count < 1000)
        { // Safety limit
            exploring = exploreStep();
            step_count++;

            // Optional: Add delay for real robot
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Exploration completed in " << step_count << " steps" << std::endl;

        // Get and display shortest path
        auto shortest_path = getShortestPath();
        std::cout << "\nShortest path (" << shortest_path.size() << " steps):" << std::endl;

        for (const auto &pos : shortest_path)
        {
            std::cout << "(" << pos.x << ", " << pos.y << ") -> ";
        }
        std::cout << "GOAL!" << std::endl;
    }

    // Print maze state (for debugging)
    void printMaze()
    {
        for (int y = maze_height - 1; y >= 0; y--)
        {
            // Print horizontal walls
            for (int x = 0; x < maze_width; x++)
            {
                std::cout << "+";
                if (maze[y][x].hasWall(NORTH))
                    std::cout << "---";
                else
                    std::cout << "   ";
            }
            std::cout << "+" << std::endl;

            // Print vertical walls and cells
            for (int x = 0; x < maze_width; x++)
            {
                if (maze[y][x].hasWall(WEST))
                    std::cout << "|";
                else
                    std::cout << " ";

                // Print cell content
                if (Position(x, y) == current_pos)
                    std::cout << " R ";
                else if (Position(x, y) == target_pos)
                    std::cout << " T ";
                else if (maze[y][x].visited)
                    std::cout << " . ";
                else
                    std::cout << "   ";
            }
            if (maze_width > 0 && maze[y][maze_width - 1].hasWall(EAST))
                std::cout << "|";
            else
                std::cout << " ";
            std::cout << std::endl;
        }

        // Print bottom border
        for (int x = 0; x < maze_width; x++)
        {
            std::cout << "+";
            if (x < maze_width && maze[0][x].hasWall(SOUTH))
                std::cout << "---";
            else
                std::cout << "   ";
        }
        std::cout << "+" << std::endl;
    }
};

// // Example usage
// int main() {
//     // Create a 16x16 maze (standard micromouse size)
//     // Start at (0,0), target at (7,7) - center of maze
//     MicromouseSolver solver(16, 16, Position(0, 0), Position(7, 7));

//     // Simulation of sensor readings (you'll replace this with actual sensor data)
//     // Example: detecting walls on left and right, no wall in front
//     solver.updateSensorReadings(5.0, 5.0, 20.0); // left_wall, right_wall, no_front_wall

//     // Start maze solving
//     solver.solveMaze();

//     // Print final maze state
//     std::cout << "\nFinal maze state:" << std::endl;
//     solver.printMaze();

//     return 0;
// }