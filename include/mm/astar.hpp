#pragma once
// A* over a turn-aware state graph. States are (cell, heading) so turn costs
// are first-class. Integer fixed-point costs, static-array memory, flat binary
// heap. Heuristic = per-cell Cost table (turn-ignoring => admissible),
// optionally scaled by integer epsilon for weighted A*.
#include <cstdint>
#include <algorithm>
#include <cstring>
#include <vector>
#include "mm/maze.hpp"
#include "mm/types.hpp"

namespace mm {

struct AstarConfig {
  Cost turnCost = kCostUnit;  // cost of one 90-deg turn
  bool stopAtFirstGoal = true;
  int epsilonDen = 1;         // weighted A*: h * epsilonNum / epsilonDen
  int epsilonNum = 1;
};

struct PathResult {
  std::vector<State> states;  // states[i]: position+heading after i-th move
  Cost cost = -1;
  int expansions = 0;
  bool found = false;
};

namespace detail {

struct Scratch {
  // lazily-grown static scratch (single-threaded use)
  static constexpr int kStates = MAZE_W * MAZE_H * kDirCount;
  int32_t g[kStates];
  int32_t parent[kStates];
  int16_t stamp[kStates];
  int16_t closed[kStates];
  int16_t gen = 0;
  bool inited = false;
  void reset() {
    if (!inited) {
      std::memset(g, 0x7F, sizeof(g));
      std::memset(parent, 0xFF, sizeof(parent));
      std::memset(stamp, 0, sizeof(stamp));
      std::memset(closed, 0, sizeof(closed));
      inited = true;
    }
    ++gen;
  }
};

}  // namespace detail

class Astar {
 public:
  // h: Cost table over cells (idx), admissible w.r.t. turn-aware cost.
  // atGoal: bool(int x, int y).
  template <class GoalFn>
  PathResult search(const Maze& maze, int sx, int sy, Dir sdir,
                    const AstarConfig& cfg, const Cost* h, GoalFn atGoal) const {
    PathResult out;
    detail::Scratch& S = scratch();
    S.reset();

    struct Item { int state; Cost f; };
    struct Cmp { bool operator()(const Item& a, const Item& b) const { return a.f > b.f; } };
    std::vector<Item> heap;
    auto push = [&](int s, Cost f) {
      heap.push_back({s, f});
      std::push_heap(heap.begin(), heap.end(), Cmp{});
    };
    auto hscale = [&](int s) -> Cost {
      int x = s & 0xF, y = (s >> 4) & 0xF;
      return (static_cast<int64_t>(h[idx(x, y)]) * cfg.epsilonNum) / cfg.epsilonDen;
    };

    int start = stateIdx(sx, sy, sdir);
    S.g[start] = 0;
    S.parent[start] = -1;
    S.stamp[start] = S.gen;
    push(start, hscale(start));

    int goalState = -1;
    while (!heap.empty()) {
      std::pop_heap(heap.begin(), heap.end(), Cmp{});
      Item it = heap.back();
      heap.pop_back();
      int s = it.state;
      if (S.closed[s] == S.gen) continue;  // stale heap entry
      S.closed[s] = S.gen;
      ++out.expansions;
      int x = s & 0xF, y = (s >> 4) & 0xF;
      Dir dir = static_cast<Dir>(s >> 8);
      if (atGoal(x, y)) { goalState = s; break; }
      // successors: all 3 turn options in place (left=1, u=2, right=1),
      // then forward
      for (int t = 0; t < 3; ++t) {
        int rel = (t == 0) ? 1 : (t == 1 ? 2 : 3);  // step count: L,U,R
        Dir d = static_cast<Dir>((static_cast<int>(dir) + rel) % kDirCount);
        int ns = stateIdx(x, y, d);
        if (S.closed[ns] == S.gen) continue;
        Cost cost = turnPenaltySteps(dir, d) * cfg.turnCost;
        (void)rel;
        int32_t ng = S.g[s] + cost;
        if (S.stamp[ns] != S.gen || ng < S.g[ns]) {
          S.stamp[ns] = S.gen;
          S.g[ns] = ng;
          S.parent[ns] = s;
          push(ns, ng + hscale(ns));
        }
      }
      if (!maze.wall(x, y, dir)) {
        int nx = x + kDx[dir], ny = y + kDy[dir];
        if (nx < 0 || ny < 0 || nx >= MAZE_W || ny >= MAZE_H) continue;  // border
        int ns = stateIdx(nx, ny, dir);
        if (S.closed[ns] != S.gen) {
          int32_t ng = S.g[s] + kCostUnit;
          if (S.stamp[ns] != S.gen || ng < S.g[ns]) {
            S.stamp[ns] = S.gen;
            S.g[ns] = ng;
            S.parent[ns] = s;
            push(ns, ng + hscale(ns));
          }
        }
      }
    }
    if (goalState < 0) return out;
    out.found = true;
    std::vector<State> rev;
    for (int s = goalState; s >= 0; s = S.parent[s]) {
      int x = s & 0xF, y = (s >> 4) & 0xF;
      rev.push_back({static_cast<uint8_t>(x), static_cast<uint8_t>(y),
                     static_cast<Dir>(s >> 8)});
    }
    out.states.assign(rev.rbegin(), rev.rend());
    out.cost = S.g[goalState];
    return out;
  }

 private:
  static detail::Scratch& scratch() {
    static detail::Scratch s;
    return s;
  }
};

}  // namespace mm
