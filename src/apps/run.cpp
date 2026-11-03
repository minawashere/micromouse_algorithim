// run.cpp: CLI benchmark runner.
//   ./run --maze <file> [--recipe p1=N/D,p2=N/D,tc=<cost>] [--json]
// Executes the solver on the offline simulator and prints result JSON.
// Also: --check to validate maze parsing (prints dump + hash).
#include <cstdio>
#include <cstring>
#include <string>
#include "mm/maze_io.hpp"
#include "mm/sim_world.hpp"
#include "mm/solver.hpp"

using namespace mm;

int main(int argc, char** argv) {
  std::string mazePath;
  SolverConfig cfg;
  bool json = false, check = false, verbose = false;
  for (int i = 1; i < argc; ++i) {
    auto next = [&]() { return std::string(argv[++i]); };
    if (!strcmp(argv[i], "--maze")) mazePath = next();
    else if (!strcmp(argv[i], "--json")) json = true;
    else if (!strcmp(argv[i], "--check")) check = true;
    else if (!strcmp(argv[i], "--verbose")) verbose = true;
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
    } else if (!strcmp(argv[i], "--floodfill")) { cfg.floodFill = true; }
    else if (!strcmp(argv[i], "--ff-straight")) { cfg.ffStraight = true; }
    else if (!strcmp(argv[i], "--ff-avoid")) { cfg.ffAvoid = true; }
    else if (!strcmp(argv[i], "--look-ahead")) { cfg.lookAhead = true; }
    else if (!strcmp(argv[i], "--ff-curio")) { cfg.ffCurio = atoi(next().c_str()); }
    else if (!strcmp(argv[i], "--ff-legs")) { cfg.ffLegs = atoi(next().c_str()); }
    else if (!strcmp(argv[i], "--mix")) { cfg.mixMode = true; }
    else if (!strcmp(argv[i], "--mix-mode")) { cfg.mixMode = true; }
    else if (!strcmp(argv[i], "--trip-worth")) {
      cfg.tripWorth = atof(next().c_str());
    } else if (!strcmp(argv[i], "--edge-prior")) {
      cfg.edgePrior = atof(next().c_str());
    } else if (!strcmp(argv[i], "--cluster-radius")) {
      cfg.clusterRadius = atoi(next().c_str());
    } else if (!strcmp(argv[i], "--allow-lazy")) {
      cfg.tripWorth = 1.0;
    } else if (!strcmp(argv[i], "--debug")) { cfg.dbgCrit = true; }
    else if (!strcmp(argv[i], "--trace")) { cfg.dbgTrace = true; }
    else if (!strcmp(argv[i], "--trace2")) { cfg.dbgTrace2 = true; }
    else {
      std::fprintf(stderr, "unknown arg %s\n", argv[i]);
      return 2;
    }
  }


  if (mazePath.empty()) {
    std::fprintf(stderr, "usage: run --maze <file> [--p1eps N] ...\n");
    return 2;
  }
  Maze truth;
  std::string err;
  if (!loadMaze(mazePath, truth, &err)) {
    std::fprintf(stderr, "maze load error: %s\n", err.c_str());
    return 1;
  }
  if (check) {
    std::printf("%s", dumpMaze(truth).c_str());
    std::printf("hash: %d\n", truth.hash());
    return 0;
  }
  SimWorld world(truth, verbose);
  Solver solver(world, truth, cfg);
  bool ok = solver.solve();
  // Score & run stats come from the SIMULATOR's own accounting (same numbers
  // the real mms GUI would report), not from the solver's shadow counters.
  const ScoreStats& ws = world.stats();
  const auto& st = solver.stats();
  if (json) {
    std::printf(
        "{\"ok\":%s,\"score\":%.2f,\"total_dist\":%.1f,\"total_eff\":%.1f,"
        "\"total_turns\":%d,\"best_run_dist\":%.1f,\"best_run_eff\":%.1f,"
        "\"best_run_turns\":%.0f,\"sprint_dist\":%d,\"sprint_turns\":%d,"
        "\"astar_calls\":%d,\"astar_exp\":%d,\"trips\":%d,\"replans\":%d,"
        "\"cert_at_cells\":%d,\"cert_known_cost\":%.3f,\"cert_bound\":%.3f}\n",
        ok ? "true" : "false", ws.score(),
        ws.totalDistance, ws.totalEffDistance, ws.totalTurns, ws.bestRunDistance,
        ws.bestRunEff, ws.bestRunTurns, st.sprintDist, st.sprintTurns,        st.aStarCalls, st.aStarExpansions, st.trips, st.replans,
        st.certifiedAfterCells, st.certBestKnown, st.certBound);
  } else {
    std::printf("ok=%d score=%.2f\n", ok ? 1 : 0, ws.score());
    std::printf("total: dist=%.1f eff=%.1f turns=%d\n", ws.totalDistance,
                ws.totalEffDistance, ws.totalTurns);
    std::printf("best-run: dist=%.1f eff=%.1f turns=%.0f\n", ws.bestRunDistance,
                ws.bestRunEff, ws.bestRunTurns);
    std::printf("sprint: dist=%d turns=%d\n", st.sprintDist, st.sprintTurns);
    std::printf("astar: calls=%d expansions=%d trips=%d replans=%d\n",
                st.aStarCalls, st.aStarExpansions, st.trips, st.replans);
    std::printf("cert: at_cells=%d known_cost=%.3f\n", st.certifiedAfterCells,
                st.certBestKnown);
  }
  return ok ? 0 : 1;
}
