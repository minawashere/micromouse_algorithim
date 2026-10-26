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
    } else {
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
  const auto& st = solver.stats();
  if (json) {
    std::printf(
        "{\"ok\":%s,\"score\":%.2f,\"total_dist\":%.1f,\"total_eff\":%.1f,"
        "\"total_turns\":%d,\"best_run_dist\":%.1f,\"best_run_eff\":%.1f,"
        "\"best_run_turns\":%d,\"sprint_dist\":%d,\"sprint_turns\":%d,"
        "\"astar_calls\":%d,\"astar_exp\":%d,\"trips\":%d,\"replans\":%d,"
        "\"cert_at_cells\":%d,\"cert_known_cost\":%.3f,\"cert_bound\":%.3f}\n",
        ok ? "true" : "false", st.bestRunEff >= 0 ? st.bestRunTurns + st.bestRunEff + 0.1 * (st.totalTurns + st.totalEff) : 2000.0,
        st.totalDistance, st.totalEff, st.totalTurns, st.bestRunDist,
        st.bestRunEff, st.bestRunTurns, st.sprintDist, st.sprintTurns,
        st.aStarCalls, st.aStarExpansions, st.trips, st.replans,
        st.certifiedAfterCells, st.certBestKnown, st.certBound);
  } else {
    std::printf("ok=%d score=%.2f\n", ok ? 1 : 0,
                st.bestRunEff >= 0 ? st.bestRunTurns + st.bestRunEff + 0.1 * (st.totalTurns + st.totalEff) : 2000.0);
    std::printf("total: dist=%.1f eff=%.1f turns=%d\n", st.totalDistance,
                st.totalEff, st.totalTurns);
    std::printf("best-run: dist=%.1f eff=%.1f turns=%d\n", st.bestRunDist,
                st.bestRunEff, st.bestRunTurns);
    std::printf("sprint: dist=%d turns=%d\n", st.sprintDist, st.sprintTurns);
    std::printf("astar: calls=%d expansions=%d trips=%d replans=%d\n",
                st.aStarCalls, st.aStarExpansions, st.trips, st.replans);
    std::printf("cert: at_cells=%d known_cost=%.3f\n", st.certifiedAfterCells,
                st.certBestKnown);
  }
  return ok ? 0 : 1;
}
