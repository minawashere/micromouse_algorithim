#pragma once
// Precomputed per-cell Cost tables: turn-ignoring Dijkstra/BFS over the wall
// graph. Turn-ignoring => admissible as a heuristic for the turn-aware A*.
#include <algorithm>
#include <climits>
#include <queue>
#include <vector>
#include "mm/maze.hpp"
#include "mm/types.hpp"

namespace mm {

// Multi-source Dijkstra, unit cell cost (kCostUnit per cell). sources: cell idx.
inline void dijkstraCells(const Maze& maze, const int* sources, int n,
                          Cost out[MAZE_W * MAZE_H]) {
  struct Item { Cost d; int c; };
  struct Cmp { bool operator()(const Item& a, const Item& b) const { return a.d > b.d; } };
  std::priority_queue<Item, std::vector<Item>, Cmp> pq;
  for (int i = 0; i < MAZE_W * MAZE_H; ++i) out[i] = INT32_MAX;
  for (int i = 0; i < n; ++i) {
    if (out[sources[i]] == INT32_MAX) { out[sources[i]] = 0; pq.push({0, sources[i]}); }
  }
  while (!pq.empty()) {
    Item it = pq.top(); pq.pop();
    int c = it.c, x = c & 0xF, y = (c >> 4) & 0xF;
    if (it.d > out[c]) continue;
    for (int di = 0; di < kDirCount; ++di) {
      Dir d = static_cast<Dir>(di);
      int nx = x + kDx[d], ny = y + kDy[d];
      if (nx < 0 || ny < 0 || nx >= MAZE_W || ny >= MAZE_H) continue;  // border
      if (maze.wall(x, y, d)) continue;
      int nc = idx(nx, ny);
      Cost nd = it.d + kCostUnit;
      if (nd < out[nc]) { out[nc] = nd; pq.push({nd, nc}); }
    }
  }
}

// "True distance" table on known walls from the goal set (any center cell).
inline void hTrueDistance(const Maze& maze, Cost out[MAZE_W * MAZE_H]) {
  int sources[4];
  for (int i = 0; i < 4; ++i) { Cell g = maze.goalCell(i); sources[i] = idx(g.x, g.y); }
  dijkstraCells(maze, sources, 4, out);
}

// Same but over the *optimistic* map: all unknown walls treated open — indoor
// pass a maze holding only known walls? here we open everything (Maze() =
// the fully open field). The `known` arg is unused (kept for API symmetry).
inline void hOptimisticDistance(const Maze& known, Cost out[MAZE_W * MAZE_H]) {
  static const Maze open;
  int sources[4];
  for (int i = 0; i < 4; ++i) { Cell g = open.goalCell(i); sources[i] = idx(g.x, g.y); }
  dijkstraCells(open, sources, 4, out);
  // NOTE: doesn't depend on `known`; kept for API symmetry / future refinement
  // (per-direction lookup tables can use known walls of the reverse trip).
}

inline void hManhattanAnyGoal(const Maze& maze, Cost out[MAZE_W * MAZE_H]) {
  for (int y = 0; y < MAZE_H; ++y)
    for (int x = 0; x < MAZE_W; ++x) {
      int best = INT_MAX;
      for (int i = 0; i < 4; ++i) {
        Cell g = maze.goalCell(i);
        int d = std::abs(x - g.x) + std::abs(y - g.y);
        if (d < best) best = d;
      }
      out[idx(x, y)] = kCostUnit * best;
    }
}

}  // namespace mm
