// Entry point for use inside the mms simulator (https://github.com/mackorone/mms):
// in mms, set the maze file and point "Build command"/"Run command" at this
// binary. Speaks the mms stdin/stdout protocol via MmsWorld.
// Note: goal cells default to the center 2x2 (standard competition layout).
//
// Optional tuning flags (identical to ./build/run):
//   --turncost N          A* turn cost (default 64 = 0.25 cell-time per turn)
//   --floodfill           flood-fill mood: 2-leg exploration descending a
//                         directional kinematic flood (straight 64, 90° turn +
//                         64, u-turn +128; curiosity weight on unexplored
//                         faces via --ff-curio). Default off.
//   --ff-curio N          curiosity weight; sweepable (0 = optimistic flood)
//   --ff-straight / --ff-avoid  walk tie-breaks (default off)
//   --mix / --mix-mode    competitive 5-stage state machine: SearchCenter
//                         (flood) -> OptimizedReturn (curio-64 flood) ->
//                         Sprint1 (locked pessimistic map, no exploration) ->
//                         DeepExplore (improvement-band trips) -> Sprint2
//                         (max-velocity, from start). Default off.
//   --ff-legs N           exploration legs (even, >= 2; odd coerced down).
//                         Even ends the walk at the start, so the sprint
//                         begins immediately. Default 2.
//   --look-ahead          real-HW 2-cell IR rig (diagonal side sensors +
//                         one-cell front read). NO such rig exists inside
//                         the mms protocol: accepted but inert here.
//   --look-ahead-diag     diagonal channels of that rig: read the remote
//                         cell's left/right faces. Also inert in mms.
//   --trip-worth X        phase-2 trip economics ratio (default -1 = exact best
//                         path certificate; optimal per 519-maze sweep — any
//                         X > 0 collapses to instant certification with
//                         slower runs); 1.0 == legacy --allow-lazy
//   --edge-prior P        open-wall prior applied to potential savings when
//                         trip-worth is active (default 1.0 = neutral)
//   --cluster-radius R    frontier tie-break neighborhood radius (default 0 =
//                         same-cell; legacy tie-break)
//   --p1eps N --p2eps N   weighted-A* epsilon per phase (1 = strict optimal
//                         trips, 2/4 = greedier shortcuts; default 2)
//   --p1heur K --p2heur K heuristic kind: manhattan | true | opt (default opt)
// e.g. runCommand = ./scripts/mms_wrap.sh   (defaults are the tuned optimum)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "mm/mms_world.hpp"
#include "mm/solver.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include "mm/maze_io.hpp"
#include "mm/sim_world.hpp"
#include "mm/solver.hpp"

using namespace mm;


int main(int argc, char** argv)
{
    mm::SolverConfig cfg;
    mm::MmsWorld world;

    std::string mazePath;
    bool json = false, check = false, verbose = false;
    for (int i = 1; i < argc; ++i) {
        auto next = [&]() { return std::string(argv[++i]); };
        if (!strcmp(argv[i], "--json")) json = true;
        else if (!strcmp(argv[i], "--check")) check = true;
        else if (!strcmp(argv[i], "--p1eps")) { cfg.phase1.epsNum = atoi(next().c_str()); }
        else if (!strcmp(argv[i], "--p2eps")) { cfg.phase2.epsNum = atoi(next().c_str()); }
        else if (!strcmp(argv[i], "--vband")) { cfg.vband = atof(next().c_str()); }
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
        } else if (!strcmp(argv[i], "--turncost")) {
            cfg.turnCost = atoi(next().c_str());
        } else if (!strcmp(argv[i], "--floodfill")){ cfg.floodFill = true; }
        else if (!strcmp(argv[i], "--ff-straight")){ cfg.ffStraight = true; }
        else if (!strcmp(argv[i], "--ff-avoid")){ cfg.ffAvoid = true; }
        else if (!strcmp(argv[i], "--look-ahead")){ cfg.lookAhead = true; } // no rig in mms: inert
        else if (!strcmp(argv[i], "--look-ahead-diag")){ cfg.lookAheadDiag = true; } // no rig in mms: inert
        else if (!strcmp(argv[i], "--ff-curio")){ cfg.ffCurio = atof(next().c_str()); }
        else if (!strcmp(argv[i], "--ff-legs")){ cfg.ffLegs = atoi(next().c_str()); }
        else if (!strcmp(argv[i], "--mix")){ cfg.mixMode = true; }
        else if (!strcmp(argv[i], "--mix-mode")){ cfg.mixMode = true; }
        else if (!strcmp(argv[i], "--trip-worth")){ cfg.tripWorth = atof(next().c_str()); }
        else if (!strcmp(argv[i], "--edge-prior")){ cfg.edgePrior = atof(next().c_str()); }
        else if (!strcmp(argv[i], "--cluster-radius")){ cfg.clusterRadius = atoi(next().c_str()); }
        else if (!strcmp(argv[i], "--allow-lazy")){ cfg.tripWorth = 1.0; }
        else if (!strcmp(argv[i], "--debug")){ cfg.dbgCrit = true; }
        else if (!strcmp(argv[i], "--trace")){ cfg.dbgTrace = true; }
        else if (!strcmp(argv[i], "--trace2")){ cfg.dbgTrace2 = true; }
        else {
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

// run command inside mms simulator
// to run the floodfill
// --floodfill --ff-straight --ff-curio 64
