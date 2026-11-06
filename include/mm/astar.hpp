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
  // SLALOM/ZIG-ZAG DISCOUNT (see SolverConfig::enableDiagonal): when set, a
  // 90-deg turn OPPOSITE to the previous 90-deg turn charges zigzagCost
  // instead of turnCost (stair-step diagonal drive; the motor controller
  // handles the physical sine-wave slalom). U-turns and first turns are
  // never discounted. Off = bit-identical plain turn-cost graph.
  bool enableDiagonal = false;
  Cost zigzagCost = kCostUnit;
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
  // lazily-grown static scratch (single-threaded use): 1024 (cell, heading)
  // states x 3 previous-turn classes = 3072 (the slalom-state axis, kPrevT)
  static constexpr int kStates = MAZE_W * MAZE_H * kDirCount * kPrevT;
  int32_t g[kStates];
  int32_t parent[kStates];
  // 32-bit generation stamps: int16 stamps would silently corrupt results
  // (stale entries treated as current) once a long-lived process wraps past
  // 32768 searches.
  int32_t stamp[kStates];
  int32_t closed[kStates];
  int32_t gen = 0;
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

    // Item: state, cost f, and diagonal step count for cost-tie breaking
    struct Item { int state; Cost f; int16_t diag; };
    struct Cmp {
      bool operator()(const Item& a, const Item& b) const {
        if (a.f != b.f) return a.f > b.f;
        return a.diag < b.diag; // tie-breaker: prefer higher diagonal count when enableDiagonal
      }
    };
    std::vector<Item> heap;
    auto push = [&](int s, Cost f, int16_t diag) {
      heap.push_back({s, f, diag});
      std::push_heap(heap.begin(), heap.end(), Cmp{});
    };
    auto hscale = [&](int s) -> Cost {
      int x = s & 0xF, y = (s >> 4) & 0xF;
      return (static_cast<int64_t>(h[idx(x, y)]) * cfg.epsilonNum) / cfg.epsilonDen;
    };

    int start = stateIdx(sx, sy, sdir, kPrevStraight);
    S.g[start] = 0;
    S.parent[start] = -1;
    S.stamp[start] = S.gen;
    push(start, hscale(start), 0);

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
      Dir dir = static_cast<Dir>((s >> 8) & 3);
      const int cls = (s >> 10) & 3; // armed previous-turn class of this state
      if (atGoal(x, y)) { goalState = s; break; }

      // successors: all 3 turn options in place (rel 1 = right, 2 = u-turn,
      // 3 = left), then forward
      for (int rel = 1; rel <= 3; ++rel) {
        Dir d = static_cast<Dir>((static_cast<int>(dir) + rel) % kDirCount);
        int ns = stateIdx(x, y, d, prevTurnClass(dir, d));
        bool zig = false;
        if (cfg.enableDiagonal && rel != 2)
        {
          if ((rel == 1 && cls == kPrevLeft) || (rel == 3 && cls == kPrevRight))
            zig = true;
        }
        Cost cost = rel == 2 ? 2 * cfg.turnCost
                             : (zig ? cfg.zigzagCost : cfg.turnCost);
        int32_t ng = S.g[s] + cost;
        if (S.stamp[ns] != S.gen || ng < S.g[ns]) {
          S.stamp[ns] = S.gen;
          S.g[ns] = ng;
          S.parent[ns] = s;
          S.closed[ns] = 0; // reopen a closed state on strict g improvement
          int16_t ndiag = cfg.enableDiagonal ? static_cast<int16_t>(it.diag + (zig ? 1 : 0)) : 0;
          push(ns, ng + hscale(ns), ndiag);
        }
      }
      if (!maze.wall(x, y, dir)) {
        int nx = x + kDx[dir], ny = y + kDy[dir];
        if (nx < 0 || ny < 0 || nx >= MAZE_W || ny >= MAZE_H) continue;  // border
        if (maze.isGoal(x, y) && maze.isGoal(nx, ny)) continue;
        int ns = stateIdx(nx, ny, dir, cls); // forward glide: class carries over
        int32_t ng = S.g[s] + kCostUnit;
        if (S.stamp[ns] != S.gen || ng < S.g[ns]) {
          S.stamp[ns] = S.gen;
          S.g[ns] = ng;
          S.parent[ns] = s;
          S.closed[ns] = 0; // reopen a closed state on strict g improvement
          int16_t ndiag = cfg.enableDiagonal ? it.diag : 0;
          push(ns, ng + hscale(ns), ndiag);
        }
      }
    }
    if (goalState < 0) return out;
    out.found = true;
    std::vector<State> rev;
    for (int s = goalState; s >= 0; s = S.parent[s]) {
      int x = s & 0xF, y = (s >> 4) & 0xF;
      rev.push_back({static_cast<uint8_t>(x), static_cast<uint8_t>(y),
                     static_cast<Dir>((s >> 8) & 3)});
    }
    // Clean up consecutive in-place turns at the same cell
    std::vector<State> fwd(rev.rbegin(), rev.rend());
    out.states.clear();
    for (size_t i = 0; i < fwd.size(); ++i) {
      if (i + 1 < fwd.size() && fwd[i].x == fwd[i + 1].x && fwd[i].y == fwd[i + 1].y) {
        // Look ahead: find the last state at this cell
        size_t k = i;
        while (k + 1 < fwd.size() && fwd[k + 1].x == fwd[i].x && fwd[k + 1].y == fwd[i].y)
          ++k;
        if (fwd[i].dir != fwd[k].dir) {
          out.states.push_back(fwd[i]);
          out.states.push_back(fwd[k]);
        } else {
          out.states.push_back(fwd[i]);
        }
        i = k;
      } else {
        out.states.push_back(fwd[i]);
      }
    }
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