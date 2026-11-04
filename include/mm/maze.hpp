#pragma once
// Fully-walled snapshot of the maze. Wall bits are symmetric between adjacent
// cells (out-of-bounds counts as wall). Goal cells come from the file's G tags
// (default: center 2x2).
#include <cstdint>
#include <cstring>
#include <vector>
#include "mm/types.hpp"

namespace mm {

class Maze {
 public:
  Maze() : goals_{{7, 7}, {8, 7}, {7, 8}, {8, 8}} {}

  bool wall(int x, int y, Dir d) const {
    if (x < 0 || y < 0 || x >= MAZE_W || y >= MAZE_H) return true;  // border
    return (walls_[idx(x, y)] & kDirBit[d]) != 0;
  }
  void setWall(int x, int y, Dir d) {
    if (x < 0 || y < 0 || x >= MAZE_W || y >= MAZE_H) return;
    walls_[idx(x, y)] |= kDirBit[d];
    int nx = x + kDx[d], ny = y + kDy[d];
    if (nx >= 0 && ny >= 0 && nx < MAZE_W && ny < MAZE_H)
      walls_[idx(nx, ny)] |= kDirBit[opposite(d)];
  }
  void setGoals(const std::vector<Cell>& gs) {
    goals_ = gs;
  }
  bool isGoal(int x, int y) const {
    for (auto& g : goals_)
      if (g.x == x && g.y == y) return true;
    return false;
  }
  Cell goalCell(int i) const { return goals_[i % (int)goals_.size()]; }
  int goalCount() const { return (int)goals_.size(); }
  int hash() const {
    uint32_t h = 2166136261u;
    for (auto w : walls_) { h ^= w; h *= 16777619u; }
    return static_cast<int>(h);
  }
  int unknownEdges() const { return 0; }

 private:
  WallMask walls_[MAZE_W * MAZE_H] = {};
  std::vector<Cell> goals_;
};

}  // namespace mm
