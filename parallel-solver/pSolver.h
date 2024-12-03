/**
 * @file    pSolver.h
 * @author  Shane Zimmerman
 * @author  Lindsay Adams
 * @date    11/21/2024
 * @brief   Solving maze in parallel with pthreads!
 */ 
#ifndef P_SOLVER_H_
#define P_SOLVER_H_

#include <omp.h>
#include <vector>
#include <random>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include "maze_generator.h"

namespace maze {

  static constexpr uint32_t wall        = maze_generator::wall;
  static constexpr uint32_t hole        = maze_generator::hole;
  static constexpr uint32_t solution    = maze_generator::solution;
  static constexpr uint32_t dead        = 3u;
  static constexpr uint32_t never_dead  = 4u;

  static constexpr uint8_t north  = 0u;
  static constexpr uint8_t south  = 1u;
  static constexpr uint8_t west   = 2u;
  static constexpr uint8_t east   = 3u;

struct Coord{
  Coord(){}
  Coord(uint32_t x, uint32_t y, uint8_t direction){
    this->x=x;
    this->y=y;
    this->direction=direction;
  }
  Coord(int x, int y){
    this->x = x;
    this->y=y;
  }
  uint32_t x;
  uint32_t y;
  uint8_t direction;
};

struct Bounds{
  Bounds(uint32_t rstart, uint32_t rend, uint32_t cstart, uint32_t cend){
    this->rowStart=rstart;
    this->rowEnd=rend;
    this->colStart=cstart;
    this->colEnd=cend;
  }
  int rowStart;
  int rowEnd;
  int colStart;
  int colEnd;
};

class MazeNode{
  explicit MazeNode(const maze::Coord *enter, const maze::Coord *exit, std::vector<uint8_t> *path);
  private:
  const Coord* enter;
  const Coord* exit;
  std::vector<uint8_t>* path; //directions to get from enter to exit
};

class MazeTree{
  private:
  std::shared_ptr<maze::MazeNode> root; //maze entrence
  std::unordered_map<const Coord, std::vector<std::shared_ptr<MazeNode>>> parents; //tree only provides path up
  public:
  MazeTree(maze::MazeNode);
  std::vector<uint8_t> reversePath(std::vector<uint8_t> path); // reverse the directions of a node's path
  void insert(maze::MazeNode); //add maze node to tree

};

class ParallelSolver{
public:
ParallelSolver(maze_generator *mazey, int &mazeEntrenceX, int &mazeEntrenceY, int &mazeExitX, int &mazeExitY);
std::vector<Bounds> partitionMaze();
void solveSubmazes();
void constructSolution();

private:
maze::maze_generator* mazey;
std::vector<Bounds> partitions;
maze::Coord mazeEntrence;
maze::Coord mazeExit;

};

};
#endif /* P_SOLVER_H_ */