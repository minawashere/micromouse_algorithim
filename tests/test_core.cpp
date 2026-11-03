// Minimal smoke tests: maze parsing + A* correctness on hand-built mazes.
//
// NOTE: assert() must stay ACTIVE even in Release (the default build type
// defines NDEBUG, which silently compiled every assertion out, making the
// whole suite vacuous). Undefine before the first <cassert> include.
#ifdef NDEBUG
#undef NDEBUG
#endif
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

static void testWallSymmetry() {
  // Shared-wall propagation into UNEXPLORED cells: learning one endpoint of a
  // wall instantly writes the opposite face of the neighbor, visited or not.
  // NOTE coordinates: y points UP (D_S = y-1), so the south face of (2,2)
  // is the border between (2,2) and (2,1) -- its symmetric twin is the
  // NORTH face of (2,1).
  KnownMaze km;
  km.setStatus(3, 4, D_E, kWall);
  assert(km.status(4, 4, D_W) == kWall);
  km.setStatus(2, 2, D_S, kOpen);
  assert(km.status(2, 1, D_N) == kOpen);
  // ...and the pathfinder must respect inferred walls of unvisited cells:
  // snapshot() copies every known WALL face (symmetric), so A* / BFS floods
  // over the snapshot can never route through a deduced wall.
  Maze snap = km.snapshot();
  assert(snap.wall(4, 4, D_W));
  assert(!snap.wall(2, 1, D_N));
  // border sanity: a BORDER face (south of (0,0): neighbor (0,-1) is out of
  // bounds) is a wall -- and must survive re-snapshotting (snap is stale the
  // moment km changes, so re-take it)
  km.setStatus(0, 0, D_S, kWall);
  snap = km.snapshot();
  assert(snap.wall(0, 0, D_S));
  assert(km.status(0, 0, D_S) == kWall);
  // out-of-bounds reads are ALWAYS walls by the KnownMaze contract
  assert(km.status(0, -1, D_N) == kWall);
  printf("wall symmetry: OK\n");
}

int main() {
  testWallSymmetry();
  testEmptyMazeAstar();
  testDijkstra();
  testLoadRealMaze();
  testSolverSimple();
  printf("ALL TESTS PASSED\n");
  return 0;
}
