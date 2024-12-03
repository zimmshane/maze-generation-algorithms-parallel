#include "pSolver.h"

maze::MazeNode::MazeNode(const maze::Coord *enter, const maze::Coord *exit, std::vector<uint8_t> *path){
    this->enter=enter;
    this->exit=exit;
    this->path=path;
}

maze::ParallelSolver::ParallelSolver(maze_generator *mazey, int &mazeEntrenceX, int &mazeEntrenceY, int &mazeExitX, int &mazeExitY){
    maze::Coord enter(mazeEntrenceX,mazeEntrenceY);
    maze::Coord exit(mazeExitX,mazeExitY);

    this->mazey=mazey;
    this->mazeEntrence = enter;
    this->mazeExit = exit;
}


//referenced this
//https://stackoverflow.com/questions/6190019/split-a-rectangle-into-equal-sized-rectangles
std::vector<maze::Bounds> maze::ParallelSolver::partitionMaze(){
    int threadCount = omp_get_num_threads();
    int cols = ceil(sqrt(threadCount));
    int rows = threadCount/cols;
    int leftovers = threadCount % cols;

    int width = this->mazey->get_width() / cols;
    int height = this->mazey->get_height() / (leftovers == 0 ? rows : (rows + 1));
    std::vector<maze::Bounds> partitions;

    for (int y = 0; y < rows; y++){
        for (int x=0; x < cols; x++){
            Bounds bound(x*width, (x*width)+width, y*height, (y*height)+height);
            partitions.push_back(bound);
        }
    }
    if (leftovers > 0){
        int leftoverWidth = width/leftovers;
        for (int x = 0; x < leftovers; x++){
            Bounds bound(x*leftoverWidth, (x+1)*leftoverWidth,(rows-1)*height, rows*height);
            partitions.push_back(bound);
        }
    }

}
void maze::ParallelSolver::solveSubmazes(){


}