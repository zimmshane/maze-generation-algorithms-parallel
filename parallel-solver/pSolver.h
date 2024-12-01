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
  explicit Coord(u_int32_t x, u_int32_t y, uint8_t direction);
  uint32_t x;
  uint32_t y;
  uint8_t direction;
};

class MazeNode{
  explicit MazeNode(Coord enter, Coord exit, std::vector<uint8_t> path);
  private:
  const Coord enter;
  const Coord exit;
  std::vector<uint8_t> path; //directions to get from enter to exit
};

class MazeTree{
  private:
  std::shared_ptr<maze::MazeNode> root; //maze entrence
  std::unordered_map<const Coord, std::vector<std::shared_ptr<MazeNode>>> parents; //tree only provides path up
  public:
  MazeTree(maze::MazeNode);
  std::vector<uint8_t> reversePath(std::vector<u_int8_t> path); // reverse the directions of a node's path
  void insert(maze::MazeNode); //add maze node to tree

};

class ParallelSolver{
public:
explicit ParallelSolver();
void partitionMaze(uint32_t thread_count);
void solveSubmaze();
void constructSolution();

private:
uint16_t threadCount;
Coord mazeEntrence;
Coord mazeExit;

};

};
#endif /* P_SOLVER_H_ */