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
  for (int i = 1; i < argc; ++i) {
    auto next = [&]() { return std::string(argv[++i]); };
    if (!strcmp(argv[i], "--turncost")) {
      cfg.turnCost = atoi(next().c_str());
    } else if (!strcmp(argv[i], "--allow-lazy")) {
      cfg.allowLazy = true;
    } else if (!strcmp(argv[i], "--p1eps")) {
      cfg.phase1.epsNum = atoi(next().c_str());
    } else if (!strcmp(argv[i], "--p2eps")) {
      cfg.phase2.epsNum = atoi(next().c_str());
    } else if (!strcmp(argv[i], "--p1heur")) {
      std::string v = next();
      cfg.phase1.kind = v == "manhattan" ? mm::HeurKind::kManhattan
                       : v == "true"     ? mm::HeurKind::kTrueDistance
                                         : mm::HeurKind::kOptimistic;
    } else if (!strcmp(argv[i], "--p2heur")) {
      std::string v = next();
      cfg.phase2.kind = v == "manhattan" ? mm::HeurKind::kManhattan
                       : v == "true"     ? mm::HeurKind::kTrueDistance
                                         : mm::HeurKind::kOptimistic;
    } else {
      std::fprintf(stderr, "mm_algo: unknown argument '%s'\n", argv[i]);
      return 2;
    }
  }
  std::fprintf(stderr,
               "mm_algo cfg: turncost=%d eps1=%d eps2=%d lazy=%d "
               "p1heur=%d p2heur=%d\n",
               static_cast<int>(cfg.turnCost), cfg.phase1.epsNum,
               cfg.phase2.epsNum, cfg.allowLazy ? 1 : 0,
               static_cast<int>(cfg.phase1.kind),
               static_cast<int>(cfg.phase2.kind));
  mm::MmsWorld world;
  if (!world.init()) return 1;
  mm::Maze assumedGoals;  // default: center 2x2
  mm::Solver solver(world, assumedGoals, cfg);
  return solver.solve() ? 0 : 2;
}
