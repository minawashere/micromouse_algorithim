// Minimal smoke tests: maze parsing + A* correctness on hand-built mazes.
#include <cassert>
#include <cstdio>
#include "mm/astar.hpp"
#include "mm/heuristics.hpp"
#include "mm/maze_io.hpp"
#include "mm/sim_world.hpp"
#include "mm/solver.hpp"

using namespace mm;

static void testEmptyMazeAstar() {
  Maze m;  // no walls: goal at (7..8,7..8); from (0,0) distance ~ 14 cells +
           // 15 turns
  Cost h[kMAZE];
  hManhattanAnyGoal(m, h);
  Astar astar;
  AstarConfig ac;
  ac.turnCost = kCostUnit;
  auto r = astar.search(m, 0, 0, D_N, ac, h, [&](int x, int y) { return m.isGoal(x, y); });
  assert(r.found);
  // path length: 15 moves minimum (sqrt of corner distance ceil: to (7,7): 14)
  printf("empty maze: cost=%d units=%d expansions=%d states=%zu\n", r.cost,
         r.cost / kCostUnit, r.expansions, r.states.size());
  assert(r.cost >= 14);
}

static void testLoadRealMaze() {
  Maze m;
  std::string err;
  bool ok = loadMaze("mazes/mazefiles/classic/001.txt", m, &err);
  assert(ok && err.empty());
  printf("001.txt hash=%d\n", m.hash());
  // start cell must be open to its east (classic file has open row to east)
  // enclosure: border walls everywhere except entrance? classic 001 is closed.
  assert(m.wall(0, 0, D_S));
  assert(m.wall(0, 0, D_W));
}

static void testDijkstra() {
  Maze m;
  Cost out[kMAZE];
  int src[1] = {idx(0, 0)};
  dijkstraCells(m, src, 1, out);
  assert(out[idx(16 - 1, 16 - 1)] == 30 * kCostUnit);
}

static void testSolverSimple() {
  // corridor maze: go east 3 cells, then south 3, goal = center? build a
  // deliberate wall layout isolating a path to the center
  Maze truth;
  for (int x = 0; x < MAZE_W; ++x) {
    truth.setWall(x, MAZE_H - 1, D_N);  // top border (y-up)
    truth.setWall(x, 0, D_S);           // bottom border
  }
  for (int y = 0; y < MAZE_H; ++y) {
    truth.setWall(0, y, D_W);           // left border
    truth.setWall(MAZE_W - 1, y, D_E);  // right border
  }
  // simple spiral corridor to (7,7)
  truth.setWall(0, 0, D_S);
  truth.setWall(1, 0, D_S);
  truth.setWall(1, 0, D_N);  // open down to row1? (already top border of row1)
  SolverConfig cfg;
  SimWorld world(truth);
  Solver solver(world, truth, cfg);
  bool ok = solver.solve();
  printf("spiral solve ok=%d score=%.1f\n", ok ? 1 : 0,
         solver.stats().bestRunEff >= 0 ? solver.stats().bestRunEff : -1.0);
  assert(ok);
}

int main() {
  testEmptyMazeAstar();
  testDijkstra();
  testLoadRealMaze();
  testSolverSimple();
  printf("ALL TESTS PASSED\n");
  return 0;
}
