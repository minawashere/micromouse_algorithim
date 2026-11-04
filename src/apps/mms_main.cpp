// Entry point for use inside the mms simulator (https://github.com/mackorone/mms):
// in mms, set the maze file and point "Build command"/"Run command" at this
// binary. Speaks the mms stdin/stdout protocol via MmsWorld.
// Note: goal cells default to the center 2x2 (standard competition layout).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <string>
#include "mm/mms_world.hpp"
#include "mm/solver.hpp"

using namespace mm;

int main(int argc, char** argv)
{
    mm::SolverConfig cfg;
    mm::MmsWorld world;

    for (int i = 1; i < argc; ++i) {
        auto next = [&]() { return std::string(argv[++i]); };
        if (!strcmp(argv[i], "--sweep")) { cfg.sweep = true; }
        else if (!strcmp(argv[i], "--turncost")) { cfg.turnCost = atoi(next().c_str()); }
        else if (!strcmp(argv[i], "--enable-diagonal")) { cfg.enableDiagonal = true; }
        else if (!strcmp(argv[i], "--zigzag-cost")) { cfg.zigzagCost = std::max(0, atoi(next().c_str())); }
        else if (!strcmp(argv[i], "--floodfill")) { cfg.floodFill = true; }
        else if (!strcmp(argv[i], "--ff-straight")) { cfg.ffStraight = true; }
        else if (!strcmp(argv[i], "--ff-avoid")) { cfg.ffAvoid = true; }
        else if (!strcmp(argv[i], "--ff-curio")) { cfg.ffCurio = atoi(next().c_str()); }
        else if (!strcmp(argv[i], "--ff-legs")) { cfg.ffLegs = atoi(next().c_str()); }
        else if (!strcmp(argv[i], "--mix") || !strcmp(argv[i], "--mix-mode")) { cfg.mixMode = true; }
        else if (!strcmp(argv[i], "--look-ahead")) { cfg.lookAhead = true; }
        else if (!strcmp(argv[i], "--look-ahead-diag")) { cfg.lookAheadDiag = true; }
        else if (!strcmp(argv[i], "--debug")) { cfg.dbgCrit = true; }
        else if (!strcmp(argv[i], "--trace")) { cfg.dbgTrace = true; }
        else if (!strcmp(argv[i], "--trace2")) { cfg.dbgTrace2 = true; }
        else if (!strcmp(argv[i], "--p1heur")) {
            std::string v = next();
            cfg.phase1.kind = v == "manhattan" ? HeurKind::kManhattan
                             : v == "true"     ? HeurKind::kTrueDistance
                                               : HeurKind::kOptimistic;
        } else if (!strcmp(argv[i], "--p2heur")) {
            std::string v = next();
            cfg.phase2.kind = v == "manhattan" ? HeurKind::kManhattan
                             : v == "true"     ? HeurKind::kTrueDistance
                                               : HeurKind::kOptimistic;
        } else {
            std::fprintf(stderr, "unknown arg %s\n", argv[i]);
            return 2;
        }
    }
    if (!world.init())
        return 1;
    mm::Maze assumedGoals; // default: center 2x2
    mm::Solver solver(world, assumedGoals, cfg);
    return solver.solve() ? 0 : 2;
}
