#pragma once
// KnownMaze: what the mouse has discovered so far. Per (cell, direction):
//   0 = unknown, 1 = known open, 2 = known wall. Symmetric updates.
// Snapshots into a `Maze` (walls only) for A*/Dijkstra: that snapshot is
// automatically "optimistic" because unknown walls are simply absent.
#include <cstring>
#include <vector>
#include "mm/maze.hpp"

namespace mm {

using WallStatus = uint8_t;  // 0 unknown, 1 open, 2 wall
constexpr WallStatus kUnknown = 0, kOpen = 1, kWall = 2;

class KnownMaze {
 public:
  WallStatus status(int x, int y, Dir d) const {
    if (x < 0 || y < 0 || x >= MAZE_W || y >= MAZE_H) return kWall;
    return (st_[idx(x, y)] >> (2 * d)) & 0x3;
  }
  void setStatus(int x, int y, Dir d, WallStatus s) {
    if (x < 0 || y < 0 || x >= MAZE_W || y >= MAZE_H) return;
    st_[idx(x, y)] &= ~(0x3 << (2 * d));
    st_[idx(x, y)] |= (s << (2 * d));
    // symmetric neighbor side
    int nx = x + kDx[d], ny = y + kDy[d];
    if (nx >= 0 && ny >= 0 && nx < MAZE_W && ny < MAZE_H)
      setStatusRaw(nx, ny, opposite(d), s);
  }
  // Update from onboard sensors (front/right/left relative to heading).
  // Back side is known-open because we entered the cell through it.
  void senseFront(int x, int y, Dir d, bool wall) {
    setStatus(x, y, d, wall ? kWall : kOpen);
  }
  // Snapshot into walls-only Maze (optimistic when walls unknown)
  Maze snapshot() const {
    Maze m;
    for (int y = 0; y < MAZE_H; ++y)
      for (int x = 0; x < MAZE_W; ++x)
        for (int d = 0; d < kDirCount; ++d)
          if (status(x, y, static_cast<Dir>(d)) == kWall) m.setWall(x, y, static_cast<Dir>(d));
    return m;
  }
  bool fullyKnown() const { return unknownCount() == 0; }
  int unknownCount() const {
    int n = 0;
    // count unknown *edges*: N edges per cell double-count symmetric; count
    // only N (up) & W (left) edges + outer S/E handled by borders
    for (int y = 0; y < MAZE_H; ++y)
      for (int x = 0; x < MAZE_W; ++x) {
        if (y < MAZE_H - 1 && status(x, y, D_S) == kUnknown) ++n;
        if (x < MAZE_W - 1 && status(x, y, D_E) == kUnknown) ++n;
      }
    return n;
  }

 private:
  void setStatusRaw(int x, int y, Dir d, WallStatus s) {
    st_[idx(x, y)] &= ~(0x3 << (2 * d));
    st_[idx(x, y)] |= (s << (2 * d));
  }
  uint8_t st_[MAZE_W * MAZE_H] = {0};
};

}  // namespace mm
