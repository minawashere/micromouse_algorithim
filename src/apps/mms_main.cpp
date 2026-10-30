// Entry point for use inside the mms simulator (https://github.com/mackorone/mms):
// in mms, set the maze file and point "Build command"/"Run command" at this
// binary. Speaks the mms stdin/stdout protocol via MmsWorld.
// Note: goal cells default to the center 2x2 (standard competition layout).
//
// Optional tuning flags (identical to ./build/run):
//   --turncost N          A* turn cost (default 256 = one cell step)
//   --allow-lazy          stop exploring when no frontier edge repays its trip
//                         (drops the exact best-path guarantee)
//   --p1eps N --p2eps N   weighted-A* epsilon per phase (1 = strict optimal
//                         trips, 2/4 = greedier shortcuts; default 2)
//   --p1heur K --p2heur K heuristic kind: manhattan | true | opt (default opt)
// e.g. runCommand = ./scripts/mms_wrap.sh --turncost 128 --allow-lazy
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "mm/mms_world.hpp"
#include "mm/solver.hpp"

int main(int argc, char** argv) {
  mm::SolverConfig cfg;
  cfg.phase2.epsNum = 3;
  mm::MmsWorld world;
  if (!world.init()) return 1;
  mm::Maze assumedGoals;  // default: center 2x2
  mm::Solver solver(world, assumedGoals, cfg);
  return solver.solve() ? 0 : 2;
}
