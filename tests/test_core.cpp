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
#include <tuple>
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

static void testSlalomAstar() {
  // ---- SLALOM/ZIG-ZAG DISCOUNT live invariants ---------------------------
  // Builder: walls on every face (symmetric, borders included) EXCEPT the
  // listed open faces, so tests can force exact unique routes. setWall is
  // additive and symmetrizes -- therefore: paint every face NOT on the open
  // list, canonicalized to E/N form, instead of "paint all then open".
  auto solidExcept = [](const std::vector<std::tuple<int, int, Dir>>& open) {
    auto canon = [](int x, int y, int d) -> std::tuple<int, int, int> {
      // y-up convention: S face of a cell is the border to its y-1 neighbor
      if (d == D_W) return {x - 1, y, D_E};
      if (d == D_S) return {x, y - 1, D_N};
      return {x, y, d};
    };
    std::vector<std::tuple<int, int, int>> oc;
    for (const auto& o : open) oc.push_back(canon(std::get<0>(o), std::get<1>(o), std::get<2>(o)));
    std::sort(oc.begin(), oc.end());
    Maze m;
    for (int x = 0; x < MAZE_W; ++x) m.setWall(x, 0, D_S);
    for (int y = 0; y < MAZE_H; ++y) m.setWall(0, y, D_W);
    for (int x = 0; x < MAZE_W; ++x)
      for (int y = 0; y < MAZE_H; ++y)
        for (int d : {D_E, D_N})
          if (!std::binary_search(oc.begin(), oc.end(), canon(x, y, d)))
            m.setWall(x, y, static_cast<Dir>(d));
    return m;
  };
  Cost h[kMAZE] = {};  // zero heuristic == admissible (plain Dijkstra)
  Astar astar;
  AstarConfig acOff;  // slalom off
  acOff.turnCost = 70;
  AstarConfig acOn = acOff;  // slalom on: alternating pairs charged 64
  acOn.enableDiagonal = true;
  acOn.zigzagCost = 64;

  // FORCED zig-zag corridor (heading convention here: y-up, so E then N is a
  // LEFT turn, N then E is a RIGHT turn). Footprint:
  //   (0,0) -E-> (1,0) -N-> (1,1) -E-> (2,1) -N-> (2,2) -E-> (3,2)
  // The A* legs below start AT (1,1) facing E, so the corridor legs are:
  //   E, turn LEFT (isolated), N, turn RIGHT (alternates!), E -> goal (3,2).
  // (The (0,0)/(1,0) spur exists so the flood-mood smoke can walk in.)
  Maze zig = solidExcept({{0, 0, D_E}, {1, 0, D_N}, {1, 1, D_E}, {2, 1, D_N},
                          {2, 2, D_E}});
  auto atEnd = [](int x, int y) { return x == 3 && y == 2; };
  auto rOff = astar.search(zig, 1, 1, D_E, acOff, h, atEnd);
  assert(rOff.found);
  assert(rOff.cost == 3 * kCostUnit + 70 + 70);  // 765 + 140 = 905
  auto rOn = astar.search(zig, 1, 1, D_E, acOn, h, atEnd);
  assert(rOn.found);
  // same route; the second turn is priced as the slalom pair member.
  assert(rOn.cost == 3 * kCostUnit + 70 + acOn.zigzagCost);  // 765 + 134 = 899
  // PathResult.states: [0] = start pose, turn-in-place = own state entries.
  assert(rOn.states.size() == 6);
  assert(rOn.states[0].x == 1 && rOn.states[0].y == 1 && rOn.states[0].dir == D_E);
  assert(rOn.states[1].x == 2 && rOn.states[1].y == 1 && rOn.states[1].dir == D_E);
  assert(rOn.states[2].x == 2 && rOn.states[2].y == 1 && rOn.states[2].dir == D_N);
  assert(rOn.states[3].x == 2 && rOn.states[3].y == 2 && rOn.states[3].dir == D_N);
  assert(rOn.states[4].x == 2 && rOn.states[4].y == 2 && rOn.states[4].dir == D_E);
  assert(rOn.states[5].x == 3 && rOn.states[5].y == 2 && rOn.states[5].dir == D_E);

  // SAME-side repeat earns NO discount: two consecutive LEFT turns (E->N->W)
  // must stay at 2 * turnCost with the discount ON. Goal = (1,2) (west of the
  // N cell): unreachable... from anywhere else, a distinct atGoal.
  Maze sameSide = solidExcept({{1, 1, D_E}, {2, 1, D_N}, {2, 2, D_W}});
  auto atWest = [](int x, int y) { return x == 1 && y == 2; };
  auto rSameOff = astar.search(sameSide, 1, 1, D_E, acOff, h, atWest);
  auto rSameOn = astar.search(sameSide, 1, 1, D_E, acOn, h, atWest);
  assert(rSameOff.found && rSameOn.found);
  assert(rSameOff.cost == 3 * kCostUnit + 2 * 70);
  assert(rSameOn.cost == rSameOff.cost);  // no alternation -> no discount
  assert(rSameOn.states.size() == 6);
  assert(rSameOn.states[5].x == 1 && rSameOn.states[5].y == 2);

  // INVARIANT: slalom is never cheaper than a pure straight line. On the open
  // portion of the zig maze, a straight E run to (3,2)?? -- instead: the
  // straight corridor itself must price identically with the discount on/off.
  Maze straight = solidExcept({{2, 2, D_E}, {3, 2, D_E}, {4, 2, D_E}, {5, 2, D_E}});
  auto atEndS = [](int x, int y) { return x == 6 && y == 2; };
  auto sOff = astar.search(straight, 2, 2, D_E, acOff, h, atEndS);
  auto sOn = astar.search(straight, 2, 2, D_E, acOn, h, atEndS);
  assert(sOff.found && sOn.found);
  assert(sOff.cost == 4 * kCostUnit);  // 4 straight cells, zero turns
  assert(sOn.cost == sOff.cost);       // discount changes nothing here

  // Flood-mood smoke: the kinematic flood with its slalom state axis solves
  // a center-goal maze with the discount ON and OFF (kinZig = 64*64/70 = 58).
  // A connected lead-in row then an alternating staircase enters the standard
  // center 2x2 at (7,7) -- Solver's endgame (`isGoalHere`) hardcodes those
  // goals, so the smoke must use them (no custom goal set).
  Maze cc = solidExcept({{0, 0, D_E}, {1, 0, D_E}, {2, 0, D_N}, {2, 1, D_N},
                         {2, 2, D_N}, {2, 3, D_E}, {3, 3, D_E}, {4, 3, D_N},
                         {4, 4, D_E}, {5, 4, D_N}, {5, 5, D_E}, {6, 5, D_N},
                         {6, 6, D_E}, {7, 6, D_N}});
  {
    SolverConfig c;
    c.floodFill = true;
    c.ffStraight = true;
    c.enableDiagonal = true;
    c.zigzagCost = 64;
    SimWorld w1(cc);
    Solver sOn(w1, cc, c);
    assert(sOn.solve());
  }
  {
    SolverConfig c;
    c.floodFill = true;
    c.ffStraight = true;
    SimWorld w2(cc);
    Solver sOff2(w2, cc, c);
    assert(sOff2.solve());
  }
  printf("slalom invariants: OK\n");
}

int main() {
  testWallSymmetry();
  testEmptyMazeAstar();
  testDijkstra();
  testLoadRealMaze();
  testSlalomAstar();
  testSolverSimple();
  printf("ALL TESTS PASSED\n");
  return 0;
}
