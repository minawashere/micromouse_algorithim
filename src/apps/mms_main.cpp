// Entry point for use inside the mms simulator (https://github.com/mackorone/mms):
// in mms, set the maze file and point "Build command"/"Run command" at this
// binary. Speaks the mms stdin/stdout protocol via MmsWorld.
// Note: goal cells default to the center 2x2 (standard competition layout).
#include "mm/mms_world.hpp"
#include "mm/solver.hpp"

int main() {
  mm::MmsWorld world;
  if (!world.init()) return 1;
  mm::Maze assumedGoals;  // default: center 2x2
  mm::SolverConfig cfg;
  mm::Solver solver(world, assumedGoals, cfg);
  solver.solve();
  return 0;
}
