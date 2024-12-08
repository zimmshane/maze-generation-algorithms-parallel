#include <omp.h>
#include <queue>
#include <mutex>
#include <vector>
#include <set>
#include "maze_generator.h"
#include "solver.h"

using std::vector;
using std::set;
using std::queue;
using std::pair;


enum struct Direction{
    UP,
    DOWN,
    LEFT,
    RIGHT
};

enum struct Cell{
    WALL = maze::maze_generator::wall,
    HOLE = maze::maze_generator::hole,
    SOLUTION = maze::maze_generator::solution
};

struct Point{
    Point(int x, int y){
        this->x=x;
        this->y=y;
    }
    int x;
    int y;
    Direction direction;
};

struct thread_boundry{
    thread_boundry(int rstart, int rend, int cstart, int cend){
        rowStart=rstart;
        rowEnd=rend;
        colStart=cstart;
        colEnd=cend;
    }
    int rowStart;
    int rowEnd;
    int colStart;
    int colEnd;
};

struct Node {
    int x, y;
    Node* parent;
    vector<pair<Node*, vector<Direction>>> connections;

    Node(int x_, int y_, Node* parent_ = nullptr) 
        : x(x_), y(y_), parent(parent_) {}

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

struct PathSegment {
    Node* start;
    Node* end;
    vector<Direction> directions;
};

class ParallelSolver {
private:
    vector<thread_boundry> submaze_bounds;
    vector<Node*> boundary_nodes;
    Node* root;
    omp_lock_t tree_lock;

    bool isValidMove(int x, int y, const vector<vector<Cell>>& maze) {
        return x >= 0 && x < maze.size() && y >= 0 && y < maze[0].size() 
               && maze[x][y] != Cell::WALL;
    }

    bool isBoundary(int x, int y, const thread_boundry& bounds) {
        return x == bounds.colStart || x == bounds.colEnd ||
               y == bounds.rowStart || y == bounds.rowEnd;
    }
    
    //referenced this
    //https://stackoverflow.com/questions/6190019/split-a-rectangle-into-equal-sized-rectangles
    vector<thread_boundry> partitionMaze(int mazeHeight, int mazeWidth){

    int threadCount = omp_get_num_threads();
    int cols = ceil(sqrt(threadCount));
    int rows = threadCount/cols;
    int leftovers = threadCount % cols;

    int width = mazeHeight / cols;
    int height = mazeWidth / (leftovers == 0 ? rows : (rows + 1));
    vector<thread_boundry> partitions;

    for (int y = 0; y < rows; y++){
        for (int x=0; x < cols; x++){
            thread_boundry bound(x*width, (x*width)+width, y*height, (y*height)+height);
            partitions.push_back(bound);
        }
    }
    if (leftovers > 0){
        int leftoverWidth = width/leftovers;
        for (int x = 0; x < leftovers; x++){
            thread_boundry bound(x*leftoverWidth, (x+1)*leftoverWidth,(rows-1)*height, rows*height);
            partitions.push_back(bound);
        }
    }
    }
    vector<PathSegment> exploreSubmaze(const vector<vector<uint32_t>>& maze, const thread_boundry& bounds, int thread_id) {
        vector<PathSegment> paths;
        vector<Node*> local_boundary_nodes;

        // Find boundary points in this submaze
        for(int x = bounds.colStart; x <= bounds.colEnd; x++) {
            for(int y = bounds.rowStart; y <= bounds.rowEnd; y++) {
                if(maze[x][y] != Cell::WALL && isBoundary(x, y, bounds)) {
                    local_boundary_nodes.push_back(new Node(x, y));
                }
            }
        }

        // For each pair of boundary points, try to find a path
        for(size_t i = 0; i < local_boundary_nodes.size(); i++) {
            for(size_t j = i + 1; j < local_boundary_nodes.size(); j++) {
                vector<vector<bool>> visited(maze.size(), 
                    vector<bool>(maze[0].size(), false));

                vector<Direction> current_path;
                if(findPath(maze, bounds, local_boundary_nodes[i]->x, 
                           local_boundary_nodes[i]->y,
                           local_boundary_nodes[j]->x, 
                           local_boundary_nodes[j]->y,
                           visited, current_path)) {
                    paths.push_back({
                        local_boundary_nodes[i],
                        local_boundary_nodes[j],
                        current_path
                    });
                }
            }
        }

        return paths;
    }

    bool findPath(const vector<vector<Cell>>& maze, const thread_boundry& bounds,
                 int start_x, int start_y, int target_x, int target_y,
                 vector<vector<bool>>& visited, vector<Direction>& path) {
        if(start_x == target_x && start_y == target_y) {
            return true;
        }

        visited[start_x][start_y] = true;

        // Define possible moves (UP, RIGHT, DOWN, LEFT)
        const int dx[] = {-1, 0, 1, 0};
        const int dy[] = {0, 1, 0, -1};
        const Direction dirs[] = {Direction::UP, Direction::RIGHT, 
                                Direction::DOWN, Direction::LEFT};

        for(int i = 0; i < 4; i++) {
            int new_x = start_x + dx[i];
            int new_y = start_y + dy[i];

            if(new_x >= bounds.colStart && new_x <= bounds.colEnd &&
               new_y >= bounds.rowStart && new_y <= bounds.rowEnd &&
               isValidMove(new_x, new_y, maze) && !visited[new_x][new_y]) {

                path.push_back(dirs[i]);
                if(findPath(maze, bounds, new_x, new_y, target_x, target_y, 
                           visited, path)) {
                    return true;
                }
                path.pop_back();
            }
        }

        return false;
    }

    void connectPaths(const vector<PathSegment>& paths) {
        omp_set_lock(&tree_lock);
        for(const auto& path : paths) {
            path.start->connections.push_back({path.end, path.directions});
            path.end->connections.push_back({path.start, reverseDirections(path.directions)});
        }
        omp_unset_lock(&tree_lock);
    }

    vector<Direction> reverseDirections(const vector<Direction>& dirs) {
        vector<Direction> reversed;
        for(auto it = dirs.rbegin(); it != dirs.rend(); ++it) {
            reversed.push_back(getOppositeDirection(*it));
        }
        return reversed;
    }

    Direction getOppositeDirection(Direction dir) {
        switch(dir) {
            case Direction::UP: return Direction::DOWN;
            case Direction::DOWN: return Direction::UP;
            case Direction::LEFT: return Direction::RIGHT;
            case Direction::RIGHT: return Direction::LEFT;
            default: return dir;
        }
    }

public:
    vector<Direction> solveMaze(const vector<vector<uint32_t>>& maze, 
                              const Point& start, const Point& end) {
        root = new Node(start.x, start.y);
        Node* exit_node = new Node(end.x, end.y);

        vector<vector<PathSegment>> all_paths(submaze_bounds.size());
        
        #pragma omp parallel num_threads(submaze_bounds.size())
        {
            int thread_id = omp_get_thread_num();
            all_paths[thread_id] = exploreSubmaze(maze, submaze_bounds[thread_id], 
                                                thread_id);
        }
        // Connect all discovered paths
        for(const auto& paths : all_paths) {
            connectPaths(paths); //should be doing this during discovery, not after
        }

        // Find path from entrance to exit using BFS
        vector<Direction> final_path;
        if(findPathInTree(root, exit_node, final_path)) {
            return final_path;
        }

        return vector<Direction>(); // No solution found
    }

    bool findPathInTree(Node* current, Node* target, vector<Direction>& path) {
        queue<pair<Node*, vector<Direction>>> q;
        set<Node*> visited;

        q.push({current, vector<Direction>()});
        visited.insert(current);

        while(!q.empty()) {
            auto [node, current_path] = q.front();
            q.pop();

            if(*node == *target) {
                path = current_path;
                return true;
            }

            for(const auto& [next, directions] : node->connections) {
                if(visited.find(next) == visited.end()) {
                    vector<Direction> new_path = current_path;
                    new_path.insert(new_path.end(), directions.begin(), 
                                  directions.end());
                    q.push({next, new_path});
                    visited.insert(next);
                }
            }
        }

        return false;
    }
};
