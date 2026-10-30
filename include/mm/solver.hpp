#pragma once
// Solver: the 4-phase micromouse state machine on top of A* + heuristics.
//
//  PHASE 1  EXPLORE_START_TO_CENTER : weighted A* (optimistic map) to center
//  PHASE 2  CERTIFIED_DEEP_SEARCH   : exploration trips toward the
//           "improvement band" (cells that could lie on a cheaper path),
//           until the best start<->center path is PROVABLY optimal
//  PHASE 3  RUN_BEST_TO_START       : run the certified best path to start
//  PHASE 4  SPRINT                  : start->center, optimal A*, long runs
//
// The certificate: let C_P = cost of best path on the known map, and for each
// cell c compute d_s(c) (optimistic dist. start->c) and d_c(c) (optimistic
// dist. c->center), both turn-free => admissible lower bounds. Any real path
// crossing its FIRST unknown edge (a->b) costs >= ds[a]+1+dc[b]. The set of
// edges with bound <= C_P is the "improvement band": only unknowns inside it
// can still improve the run. Certify when the band is empty.
#include "mm/astar.hpp"
#include "mm/heuristics.hpp"
#include "mm/known_maze.hpp"
#include "mm/maze_io.hpp"
#include "mm/world.hpp"
#include <algorithm>
#include <cstdlib>
#include <vector>

namespace mm {

constexpr int kMAZE = MAZE_W * MAZE_H;

enum struct HeurKind : int { kManhattan = 0, kTrueDistance, kOptimistic };

struct PhaseHeur {
  HeurKind kind = HeurKind::kOptimistic;
  int epsNum = 2; // weighted A* default: h * 2 (explore fast, path may cost +
  int epsDen = 1;
};

struct SolverConfig {
  PhaseHeur phase1, phase2;
  Cost turnCost = 64;
  bool visualize = false;
  // opt-in: stop phase 2 as soon as no frontier edge repays its trip cost
  // (saving <= driving distance). Trades the exact best-path guarantee for
  // less exploration. Default: off (certificate = optimal path).
  bool allowLazy = false;
};

struct SolverStats {
  double totalDistance = 0, totalEff = 0;
  int totalTurns = 0;
  int phase = 0;
  double bestRunDist = -1, bestRunEff = -1;
  int bestRunTurns = -1;
  int aStarCalls = 0, aStarExpansions = 0;
  int trips = 0, replans = 0;
  int certifiedAfterCells = 0;
  double certBestKnown = -1;
  double certBound = -1;
  double certOptimistic = -1;
  int sprintDist = 0, sprintTurns = 0;
};

class Solver {
public:
  Solver(IWorld &w, const Maze &truth, const SolverConfig &cfg)
      : w_(w), truth_(truth), cfg_(cfg), allowLazy_(cfg.allowLazy) {}

  bool solve() { // false on crash / planning failure
    st_ = SolverStats{};
    mx_ = 0;
    my_ = 0;
    mdir_ = D_N;
    known_ = KnownMaze{};
    sealed_ = false;
    vizInit();
    if (!phase1()) {
      if (getenv("MM_DEBUG"))
        fprintf(stderr, "PHASE1 FAILED\n");
      return false;
    }
    if (!phase2()) {
      if (getenv("MM_DEBUG"))
        fprintf(stderr, "PHASE2 FAILED\n");
      return false;
    }
    if (!phase3()) {
      if (getenv("MM_DEBUG"))
        fprintf(stderr, "PHASE3 FAILED\n");
      return false;
    }
    if (!phase4()) {
      if (getenv("MM_DEBUG"))
        fprintf(stderr, "PHASE4 FAILED\n");
      return false;
    }
    return true;
  }
  const SolverStats &stats() const { return st_; }

private:
  enum struct Phase { P1, P2 };

  // ---------- planning ----------
  // Run stats phase; see phase docs above.
  template <class GoalFn>
  PathResult plan(int sx, int sy, Dir sdir, GoalFn atGoal, const Maze &snap,
                  const PhaseHeur &ph, const Cost *hOverride = nullptr) {
    Cost tbl[kMAZE];
    const Cost *h = hOverride;
    if (!h) {
      switch (ph.kind) {
      case HeurKind::kManhattan:
        hManhattanAnyGoal(snap, tbl);
        h = tbl;
        break;
      case HeurKind::kTrueDistance:
        hTrueDistance(snap, tbl);
        h = tbl;
        break;
      default:
        hOptimisticDistance(snap, tbl);
        h = tbl;
        break;
      }
    }
    AstarConfig ac;
    ac.turnCost = cfg_.turnCost;
    ac.epsilonNum = ph.epsNum;
    ac.epsilonDen = ph.epsDen;
    ++st_.aStarCalls;
    PathResult r = astar_.search(snap, sx, sy, sdir, ac, h, atGoal);
    st_.aStarExpansions += r.expansions;
    return r;
  }

  // ---------- sensing & motion ----------
  // Sense 3 faces + infer back-side open. Returns true if anything changed.
  bool sense() {
    bool changed = false;
    struct Q {
      bool wall;
      Dir d;
    } qs[3] = {{w_.wallFront(), mdir_},
               {w_.wallRight(), static_cast<Dir>((mdir_ + 1) % kDirCount)},
               {w_.wallLeft(), static_cast<Dir>((mdir_ + 3) % kDirCount)}};
    for (auto &q : qs) {
      WallStatus s = q.wall ? kWall : kOpen;
      if (known_.status(mx_, my_, q.d) != s) {
        known_.setStatus(mx_, my_, q.d, s);
        changed = true;
      }
    }
    // back side: open (we came through it) unless still unknown
    Dir back = static_cast<Dir>((mdir_ + 2) % kDirCount);
    if (known_.status(mx_, my_, back) == kUnknown) {
      known_.setStatus(mx_, my_, back, kOpen);
      changed = true;
    }
    return changed;
  }

  // How many forward *moves* we can execute in one continuous batch starting
  // at path[pi] (we are at path[pi]). Crossing out of path[j] into path[j+1]
  // requires the exit face of path[j] to be known-open. On unknown exit we
  // stop (arrive-sense-replan continues in the caller). A wall exit means the
  // plan is stale; stopping is still safe (no crash).
  int maxRun(const std::vector<State> &path, size_t pi) const {
    int run = 0;
    size_t j = pi;
    while (j + 1 < path.size()) {
      if (path[j].dir != path[j + 1].dir)
        break; // turn at path[j]
      WallStatus st = known_.status(path[j].x, path[j].y, path[j].dir);
      if (st != kOpen)
        break; // unknown or wall: cannot cross blindly
      ++j;
    }
    return static_cast<int>(j - pi);
  }

  // Follow a goal-directed plan from current pose with sense+replan.
  enum struct RunStatus { kDone, kStuck };

  // Snapshot with unknown faces treated as walls (for sealed open-loop plans).
  Maze pessimisticSnapshot() const {
    Maze m = known_.snapshot();
    for (int y = 0; y < MAZE_H; ++y)
      for (int x = 0; x < MAZE_W; ++x)
        for (int di = 0; di < kDirCount; ++di)
          if (known_.status(x, y, static_cast<Dir>(di)) == kUnknown)
            m.setWall(x, y, static_cast<Dir>(di));
    return m;
  }

  // (F) Open-loop executor for the sealed maze: drive `states` from the
  // current pose in one pass -- batching consecutive straight cells into single
  // moveForward(N) calls, turning in place for dir changes, no sensing and no
  // replanning. All faces along a certified path are known-open by
  // construction, so a crash reply cannot happen (kStuck on it anyway,
  // defensively).
  template <class GoalFn>
  RunStatus driveOpen(const std::vector<State> &plan, GoalFn atGoal) {
    size_t pi = SIZE_MAX;
    for (size_t j = 0; j < plan.size(); ++j)
      if (plan[j].x == mx_ && plan[j].y == my_ && plan[j].dir == mdir_) {
        pi = j;
        break;
      }
    if (pi == SIZE_MAX)
      return RunStatus::kStuck; // pose not on plan: replan
    if (atGoal(mx_, my_))
      return RunStatus::kDone;
    while (pi + 1 < plan.size()) {
      const State &a = plan[pi];
      const State &b = plan[pi + 1];
      if (a.dir == b.dir) {
        // straight batch across plans[j..j+n]
        int run = 1;
        while (pi + run + 1 < plan.size() &&
               plan[pi + run].dir == plan[pi + run + 1].dir) {
          ++run;
        }
        if (!w_.moveForward(run))
          return RunStatus::kStuck;
        advanceStats(run);
        mx_ += static_cast<int>(kDx[mdir_]) * run;
        my_ += static_cast<int>(kDy[mdir_]) * run;
        pi += run;
        onCellVisit();
      } else {
        // turn in place until facing b.dir (up to 2 x 90 deg for a U-turn)
        int rel =
            (static_cast<int>(b.dir) - static_cast<int>(a.dir) + kDirCount) %
            kDirCount;
        while (mdir_ != b.dir) {
          if (rel == 1 || rel == 2) {
            w_.turnRight();
            mdir_ = static_cast<Dir>((mdir_ + 1) % kDirCount);
          } else {
            w_.turnLeft();
            mdir_ = static_cast<Dir>((mdir_ + 3) % kDirCount);
          }
          ++st_.totalTurns;
          if (runAtStart_)
            ++curRunTurns_;
        }
        ++pi;
      }
      if (atGoal(mx_, my_))
        return RunStatus::kDone;
    }
    return atGoal(mx_, my_) ? RunStatus::kDone : RunStatus::kStuck;
  }

  // Follow a goal-directed plan from current pose with sense+replan.
  // kStuck = planning failed mid-trip (partner may succeed from new pose).
  template <class GoalFn>
  RunStatus runTo(GoalFn atGoal, const PhaseHeur &ph,
                  const Cost *hOverride = nullptr) {
    // (F) sealed maze: no sensing/replanning during the endgame runs -- drive
    // open-loop on a *pessimistic* snapshot (unknown faces treated as walls),
    // so every crossed face is provably known-open and a crash is impossible.
    if (sealed_) {
      Maze pes = pessimisticSnapshot();
      PathResult fin = plan(mx_, my_, mdir_, atGoal, pes, ph, hOverride);
      if (!fin.found)
        return RunStatus::kStuck;
      return driveOpen(fin.states, atGoal);
    }
    const int kMaxSteps = 100000; // safety breaker
    int steps = 0;
    PathResult cur =
        plan(mx_, my_, mdir_, atGoal, known_.snapshot(), ph, hOverride);
    if (!cur.found)
      return RunStatus::kStuck;
    while (steps++ < kMaxSteps) {
      bool changed = sense(); // sense at every cell visit (3 faces + back)
      vizVisit();
      if (getenv("MM_TRACE2"))
        fprintf(stderr, "RUNTO pos=(%d,%d,%d) chg=%d faces=%d%d%d%d\n", mx_,
                my_, (int)mdir_, changed, (int)known_.status(mx_, my_, D_N),
                (int)known_.status(mx_, my_, D_E),
                (int)known_.status(mx_, my_, D_S),
                (int)known_.status(mx_, my_, D_W));
      if (atGoal(mx_, my_)) {
        if (!changed)
          return RunStatus::kDone;
        // arrived + fresh walls: replan once, then confirm
        ++st_.replans;
        cur = plan(mx_, my_, mdir_, atGoal, known_.snapshot(), ph, hOverride);
        if (!cur.found || cur.states.size() <= 1)
          return RunStatus::kDone;
        continue;
      }
      if (changed) {
        ++st_.replans;
        Maze snap = known_.snapshot(); // keep for debugging
        cur = plan(mx_, my_, mdir_, atGoal, snap, ph, hOverride);
        if (!cur.found) {
          if (getenv("MM_DEBUG")) {
            Cost dd[kMAZE];
            int tsrcD[1] = {idx(dbgTx_, dbgTy_)};
            dijkstraCells(snap, tsrcD, 1, dd);
            // retry with zero heuristic to isolate h vs graph bug
            AstarConfig zc;
            zc.turnCost = cfg_.turnCost;
            Cost zero[kMAZE];
            std::memset(zero, 0, sizeof(zero));
            Astar ax;
            PathResult rz = ax.search(snap, mx_, my_, mdir_, zc, zero, atGoal);
            fprintf(stderr,
                    "STUCK at (%d,%d,%d) goal=%d,%d lead=%d unknown=%d\n%s",
                    mx_, my_, (int)mdir_, truth_.goalCell(0).x,
                    truth_.goalCell(0).y, dd[idx(mx_, my_)] == INT32_MAX,
                    known_.unknownCount(), dumpMaze(snap).c_str());
            FILE *f = fopen("/tmp/opencode/dumpsnap.txt", "w");
            fprintf(f, "%s", dumpMaze(snap).c_str());
            fclose(f);
          }
          return RunStatus::kStuck;
        }
      }
      // invariant: solver pose == world pose
      {
        int wx, wy, wd;
        w_.debugPos(&wx, &wy, &wd);
        if (wx >= 0 && (wx != mx_ || wy != my_ || wd != (int)mdir_)) {
          fprintf(stderr, "POSE DESYNC solver=(%d,%d,%d) world=(%d,%d,%d)\n",
                  mx_, my_, (int)mdir_, wx, wy, wd);
          return RunStatus::kStuck;
        }
      }
      // execute one batch along cur.states starting at our index
      size_t pi = SIZE_MAX;
      for (size_t j = 0; j < cur.states.size(); ++j)
        if (cur.states[j].x == mx_ && cur.states[j].y == my_ &&
            cur.states[j].dir == mdir_) {
          pi = j;
          break;
        }
      if (pi ==
          SIZE_MAX) { // plan didn't include our (state,dir): replan next iter
        ++st_.replans;
        cur = plan(mx_, my_, mdir_, atGoal, known_.snapshot(), ph, hOverride);
        if (!cur.found || cur.states.empty())
          return RunStatus::kStuck;
        continue;
      }
      int run = 0;
      if (pi + 1 < cur.states.size()) {
        const State &a = cur.states[pi];
        const State &b = cur.states[pi + 1];
        if (a.dir == b.dir)
          run = maxRun(cur.states, pi);
      }
      if (run > 0) {
        if (getenv("MM_TRACE"))
          fprintf(stderr, "MOVE run=%d from=(%d,%d) dir=%d to=(%d,%d)\n", run,
                  mx_, my_, (int)mdir_, mx_ + kDx[mdir_] * run,
                  my_ + kDy[mdir_] * run);
        if (!w_.moveForward(run))
          return RunStatus::kStuck; // crash / sim end
        advanceStats(run);
        mx_ += kDx[mdir_] * run;
        my_ += kDy[mdir_] * run;
      } else {
        // turn according to plan's next move
        if (pi + 1 >= cur.states.size()) {
          // at goal-turn state? shouldn't happen: atGoal checked above
          return RunStatus::kDone;
        }
        Dir nd = cur.states[pi + 1].dir;
        while (mdir_ != nd) {
          int rel =
              (static_cast<int>(nd) - static_cast<int>(mdir_) + kDirCount) %
              kDirCount;
          if (rel == 3) {
            w_.turnLeft();
            mdir_ = static_cast<Dir>((mdir_ + 3) % kDirCount);
          } else {
            w_.turnRight();
            mdir_ = static_cast<Dir>((mdir_ + 1) % kDirCount);
          }
          ++st_.totalTurns;
          if (runAtStart_)
            ++curRunTurns_;
        }
      }
      // run-boundary bookkeeping (start / goal visits)
      onCellVisit();
    }
    return atGoal(mx_, my_) ? RunStatus::kDone : RunStatus::kStuck;
  }

  void advanceStats(int run) {
    st_.totalDistance += run;
    st_.totalEff += (run <= 2) ? run : 2.0 + 0.5 * (run - 2);
    if (runAtStart_) {
      curRunDist_ += run;
      curRunEff_ += (run <= 2) ? run : 2.0 + 0.5 * (run - 2);
    }
  }
  void onCellVisit() {
    if (mx_ == 0 && my_ == 0) {
      runAtStart_ = true;
      curRunDist_ = 0;
      curRunEff_ = 0;
      curRunTurns_ = 0;
    }
    if (runAtStart_ && isGoalHere()) {
      runAtStart_ = false;
      double total = curRunEff_ + curRunTurns_;
      if (st_.bestRunEff < 0 || total < st_.bestRunEff + st_.bestRunTurns) {
        st_.bestRunDist = curRunDist_;
        st_.bestRunEff = curRunEff_;
        st_.bestRunTurns = curRunTurns_;
      }
    }
  }
  bool isGoalHere() const {
    return (mx_ == MAZE_W / 2 || mx_ == MAZE_W / 2 - 1) &&
           (my_ == MAZE_H / 2 || my_ == MAZE_H / 2 - 1);
  }

  // ---------- visualization (worlds that ignore setColor/setText: no-op)
  // ---------- Colors: r=goal, g=explored, o=frontier trip target, b=certified
  // best path. Text: number in each visited cell = optimistic steps-to-center
  // at visit time; refreshed with the final true distance once the path is
  // certified.
  bool cellVis_[MAZE_W * MAZE_H] = {};

  void vizInit() {
    w_.clearAllColor();
    w_.clearAllText();
    std::memset(cellVis_, 0, sizeof(cellVis_));
    vizGoals();
  }

  void vizGoals() {
    Maze snap = known_.snapshot();
    for (int i = 0; i < snap.goalCount() && i < 4; ++i) {
      Cell g = snap.goalCell(i);
      w_.setColor(g.x, g.y, 'r');
    }
  }

  // distance-to-center table on the current known map (unknown=optimistic)
  void vizDistanceTable(const Maze &snap, Cost out[kMAZE]) {
    int src[4], n = 0;
    for (int i = 0; i < 4 && i < snap.goalCount(); ++i) {
      Cell g = snap.goalCell(i);
      src[n++] = idx(g.x, g.y);
    }
    dijkstraCells(snap, src, n, out);
  }

  void vizVisit() {
    int c = idx(mx_, my_);
    if (cellVis_[c])
      return;
    cellVis_[c] = true;
    w_.setColor(mx_, my_, 'g');
    Maze snap = known_.snapshot();
    Cost dc[kMAZE];
    vizDistanceTable(snap, dc);
    if (dc[c] < INT32_MAX) {
      char b[8];
      std::snprintf(b, sizeof b, "%d", dc[c] / kCostUnit);
      w_.setText(mx_, my_, b);
    }
  }

  void vizTripTarget(int tx, int ty) { w_.setColor(tx, ty, 'o'); }

  void vizFinal(const PathResult &best) {
    Maze snap = known_.snapshot();
    w_.clearAllColor();
    vizGoals();
    // final true distances on every visited cell
    Cost dc[kMAZE];
    vizDistanceTable(snap, dc);
    for (int y = 0; y < MAZE_H; ++y)
      for (int x = 0; x < MAZE_W; ++x) {
        int c = idx(x, y);
        if (!cellVis_[c] || dc[c] == INT32_MAX)
          continue;
        char b[8];
        std::snprintf(b, sizeof b, "%d", dc[c] / kCostUnit);
        w_.setText(x, y, b);
      }
    // certified best path in faint blue (cyan)
    for (auto s : best.states)
      w_.setColor(s.x, s.y, 'c');
  }

  // ---------- phases ----------
  bool phase1() {
    st_.phase = 1;
    auto atGoal = [&](int x, int y) { return isGoalTruth(x, y); };
    if (runTo(atGoal, cfg_.phase1) == RunStatus::kStuck)
      return false;
    return true;
  }

  struct FrontierEdge {
    int ax, ay; // from-cell
    Dir d;      // direction of unknown edge a->b
    Cost bound; // ds[a] + 1 + dc[b] (potential best path through it)
    int tx, ty; // cell to travel to for sensing this edge: a
  };

  // compute the improvement band frontier; false => certified (band empty)
  bool improvementFrontier(std::vector<FrontierEdge> &out, double C_P,
                           Cost ds[kMAZE], Cost dc[kMAZE]) {
    out.clear();
    Maze snap = known_.snapshot();
    int srcS[1] = {idx(0, 0)};
    int srcC[4];
    for (int i = 0; i < 4; ++i) {
      Cell g = snap.goalCell(i);
      srcC[i] = idx(g.x, g.y);
    }
    dijkstraCells(snap, srcS, 1, ds);
    dijkstraCells(snap, srcC, 4, dc);
    const Cost LIM = static_cast<Cost>(C_P);
    for (int y = 0; y < MAZE_H; ++y)
      for (int x = 0; x < MAZE_W; ++x) {
        for (int di = 1; di < kDirCount; ++di) { // E,S only: dedupe symmetric
          Dir d = static_cast<Dir>(di);
          if (known_.status(x, y, d) != kUnknown)
            continue;
          int nx = x + kDx[d], ny = y + kDy[d];
          if (nx < 0 || ny < 0 || nx >= MAZE_W || ny >= MAZE_H)
            continue;
          // check both traversal orders for an improving shortcut
          Cost b1 = ds[idx(x, y)];
          Cost b2 = dc[idx(nx, ny)];
          Cost b3 = ds[idx(nx, ny)];
          Cost b4 = dc[idx(x, y)];
          if (b1 == INT32_MAX || b2 == INT32_MAX || b3 == INT32_MAX ||
              b4 == INT32_MAX)
            continue; // side disconnected even optimistically (irrelevant)
          if (b1 + kCostUnit + b2 <= LIM || b3 + kCostUnit + b4 <= LIM) {
            FrontierEdge e;
            e.ax = x;
            e.ay = y;
            e.d = d;
            e.bound = std::min(b1 + kCostUnit + b2, b3 + kCostUnit + b4);
            // travel to whichever side is more informative; use a (from-cell)
            e.tx = x;
            e.ty = y;
            out.push_back(e);
          }
        }
      }
    return !out.empty();
  }

  bool phase2() {
    st_.phase = 2;
    std::memset(skipTarget_, 0, sizeof(skipTarget_));
    int guard = 0;
    iter_ = 0;
    while (guard++ < 400) {
      ++iter_;
      if (getenv("MM_DEBUG"))
        fprintf(stderr, "P2-top iter=%d pos=(%d,%d,%d)\n", iter_, mx_, my_,
                (int)mdir_);
      // 1: best known path center<->start (A* from start pose? mouse may be
      // anywhere; evaluate the *run* start->center independent of pose)
      Maze snap = known_.snapshot();
      PathResult best = plan(
          0, 0, D_N, [&](int x, int y) { return isGoalTruth(x, y); }, snap,
          PhaseHeur{HeurKind::kTrueDistance, 1, 1});
      if (!best.found && known_.unknownCount() == 0) {
        // maze dead-ends? keep exploring (trip toward frontier) or fail:
        // if no unknown and no path, maze can't be solved: report failure
        return false;
      }
      if (best.found) {
        st_.certBestKnown = best.cost / static_cast<double>(kCostUnit);
        // 2: improvement band
        Cost ds[kMAZE], dc[kMAZE];
        std::vector<FrontierEdge> frontier;
        bool exist = improvementFrontier(frontier, best.cost, ds, dc);
        if (getenv("MM_DEBUG")) {
          fprintf(
              stderr,
              "P2 iter=%d pos=(%d,%d,%d) CP=%.2f frontier=%zu unknownEdges=%d "
              "opt=%.2f\n",
              iter_, mx_, my_, (int)mdir_, best.cost / (double)kCostUnit,
              frontier.size(), known_.unknownCount(), st_.certOptimistic);
        }
        st_.certOptimistic =
            (dc[idx(0, 0)] < INT32_MAX ? dc[idx(0, 0)] / (double)kCostUnit
                                       : -1);
        if (!exist) {
          // CERTIFIED
          st_.certifiedAfterCells = static_cast<int>(st_.totalDistance);
          st_.certBound = st_.certBestKnown;
          certified_ = best;
          vizFinal(best);
          sealed_ = true; // (F): endgame drives open-loop, no sense/replan
          return true;
        }
        // 3: score frontier edges (A): one flood from the mouse gives the exact
        // turn-free driving distance to every edge target at once (the cell
        // graph is undirected, so the old per-edge flood from each edge gave
        // the same number): O(|frontier|) floods become exactly one.
        Cost dm[kMAZE];
        int msrc[1] = {idx(mx_, my_)};
        dijkstraCells(snap, msrc, 1, dm);
        double vband_ = 1.0; // cells; (B) tie window, tunable via MM_VBAND
        if (const char *v = getenv("MM_VBAND"))
          vband_ = atof(v);
        const double VBAND = vband_;
        const FrontierEdge *bestEdge = nullptr;
        double bestVal = -1e18;
        for (const auto &e : frontier) {
          if (skipTarget_[idx(e.tx, e.ty)])
            continue;
          if (dm[idx(e.tx, e.ty)] == INT32_MAX)
            continue; // sealed-off target
          double trip = dm[idx(e.tx, e.ty)] / (double)kCostUnit;
          double saving = (best.cost - e.bound) / (double)kCostUnit;
          double val = saving - trip;
          if (val > bestVal)
            bestVal = val;
        }
        if (bestVal <= -1e17) {
          // nowhere reachable with unknowns in-band: cannot improve: certify
          st_.certifiedAfterCells = static_cast<int>(st_.totalDistance);
          st_.certBound = st_.certBestKnown;
          certified_ = best;
          vizFinal(best);
          return true;
        }
        // (B) tie-break among the value plateau: nearest to the mouse first,
        // then more promising bound, then more in-band faces sharing the same
        // target cell (one trip services several edges).
        int bestTrip = INT32_MAX;
        Cost bestBound = INT32_MAX;
        int bestShared = -1;
        for (const auto &e : frontier) {
          if (skipTarget_[idx(e.tx, e.ty)])
            continue;
          if (dm[idx(e.tx, e.ty)] == INT32_MAX)
            continue;
          double trip = dm[idx(e.tx, e.ty)] / (double)kCostUnit;
          double saving = (best.cost - e.bound) / (double)kCostUnit;
          double val = saving - trip;
          if (val < bestVal - VBAND)
            continue;
          // how many other in-band edges share this target cell?
          int shared = 0;
          for (const auto &o : frontier)
            if (o.tx == e.tx && o.ty == e.ty)
              ++shared;
          bool better = false;
          if (trip < bestTrip)
            better = true;
          else if (trip == bestTrip) {
            if (e.bound < bestBound)
              better = true;
            else if (e.bound == bestBound && shared > bestShared)
              better = true;
          }
          if (better) {
            bestEdge = &e;
            bestTrip = static_cast<int>(trip);
            bestBound = e.bound;
            bestShared = shared;
          }
        }
        // (D) lazy mode: nothing pays for itself (saving <= trip for all) and
        // the user accepts possibly non-optimal best paths => certify now.
        if (allowLazy_ && bestVal <= 0) {
          st_.certifiedAfterCells = static_cast<int>(st_.totalDistance);
          st_.certBound = st_.certBestKnown;
          certified_ = best;
          vizFinal(best);
          sealed_ = true;
          return true;
        }
        // 4: exploration trip to the frontier cell
        int tx = bestEdge->tx, ty = bestEdge->ty;
        vizTripTarget(tx, ty);
        if (mx_ == tx && my_ == ty) {
          faceForUnknown(bestEdge->d);
        } else {
          ++st_.trips;
          PhaseHeur tripHeur;
          tripHeur.epsNum = cfg_.phase2.epsNum;
          tripHeur.epsDen = cfg_.phase2.epsDen;
          // exact turn-free optimistic h toward the trip target (admissible,
          // computed once on the open map; walls discovered en route don't
          // invalidate its admissibility)
          std::vector<Cost> tripH(kMAZE);
          int tsrc[1] = {idx(tx, ty)};
          dijkstraCells(Maze(), tsrc, 1, tripH.data());
          dbgTx_ = tx;
          dbgTy_ = ty;
          auto atT = [&](int x, int y) { return x == tx && y == ty; };
          if (runTo(atT, tripHeur, tripH.data()) == RunStatus::kStuck) {
            // sealed en route: give up on this target this maze (its unknown
            // faces can only be sensed if some route there appears later? no:
            // mark skip; the cert may still close via other edges)
            skipTarget_[idx(tx, ty)] = true;
            continue;
          }
          // unknown edge may have been sensed en route; if not, orient & sense
          if (known_.status(bestEdge->tx, bestEdge->ty, bestEdge->d) ==
              kUnknown)
            faceForUnknown(bestEdge->d);
        }
        continue;
      }
      // No known path (start disconnected on known map): explore toward the
      // nearest unknown to open connectivity (fallback: plain optimistic A*)
      PhaseHeur fallback;
      fallback.kind = HeurKind::kOptimistic;
      fallback.epsNum = cfg_.phase2.epsNum;
      fallback.epsDen = cfg_.phase2.epsDen;
      auto atUnknownNeighbor = [&](int x, int y) {
        for (int di = 0; di < kDirCount; ++di) {
          Dir d = static_cast<Dir>(di);
          // own face unknown => sensing here will reveal it => real progress
          if (known_.status(x, y, d) == kUnknown)
            return true;
        }
        return false;
      };
      if (runTo(atUnknownNeighbor, fallback) == RunStatus::kStuck)
        return false;
    }
    return false; // loop guard tripped
  }

  void faceDir(Dir d) {
    while (mdir_ != d) {
      int rel = (static_cast<int>(d) - static_cast<int>(mdir_) + kDirCount) %
                kDirCount;
      if (rel == 1 || rel == 2) {
        w_.turnRight();
        mdir_ = static_cast<Dir>((mdir_ + 1) % kDirCount);
      } else {
        w_.turnLeft();
        mdir_ = static_cast<Dir>((mdir_ + 3) % kDirCount);
      }
      ++st_.totalTurns;
      if (runAtStart_)
        ++curRunTurns_;
      if (!sealed_)
        sense();
    }
  }
  // Rotate in place (up to 4 x 90°) until face d is in front, sensing sides.
  void faceForUnknown(Dir d) {
    for (int k = 0; k < 4; ++k) {
      if (known_.status(mx_, my_, d) != kUnknown)
        return;
      w_.turnRight();
      mdir_ = static_cast<Dir>((mdir_ + 1) % kDirCount);
      ++st_.totalTurns;
      if (runAtStart_)
        ++curRunTurns_;
      sense();
    }
  }

  bool phase3() {
    st_.phase = 3;
    // run certified best path toward start (from wherever we are)
    PhaseHeur ph;
    ph.kind = HeurKind::kTrueDistance;
    ph.epsNum = 1;
    ph.epsDen = 1;
    auto atStart = [&](int x, int y) { return x == 0 && y == 0; };
    if (mx_ == 0 && my_ == 0)
      return true;
    return runTo(atStart, ph) == RunStatus::kDone;
  }

  bool phase4() {
    st_.phase = 4;
    // sprint: optimal A* on fully known map, exact h, batching enabled
    PhaseHeur ph;
    ph.kind = HeurKind::kTrueDistance;
    ph.epsNum = 1;
    ph.epsDen = 1;
    auto atGoal = [&](int x, int y) { return isGoalTruth(x, y); };
    // face the first direction of the sprint plan
    Maze snap = known_.snapshot();
    PathResult p = plan(mx_, my_, mdir_, atGoal, snap, ph);
    if (!p.found || p.states.size() < 2)
      return false;
    faceDir(p.states[1].dir);
    if (runTo(atGoal, ph) != RunStatus::kDone)
      return false;
    st_.sprintDist = static_cast<int>(curRunDist_);
    st_.sprintTurns = curRunTurns_;
    return isGoalHere();
  }

  bool isGoalTruth(int x, int y) const { return truth_.isGoal(x, y); }
  int dbgTx_ = -1, dbgTy_ = -1;
  bool skipTarget_[MAZE_W * MAZE_H] = {}; // trip targets proven unreachable
  bool allowLazy_ = false;                // mirrors cfg_.allowLazy
  bool sealed_ = false; // (F) true after certification: endgame is open-loop

  IWorld &w_;
  const Maze &truth_;
  SolverConfig cfg_;
  Astar astar_;
  KnownMaze known_;
  int mx_ = 0, my_ = 0;
  Dir mdir_ = D_N;
  SolverStats st_;
  PathResult certified_;
  int iter_ = 0;
  bool runAtStart_ = false;
  double curRunDist_ = 0, curRunEff_ = 0;
  int curRunTurns_ = 0;
};

} // namespace mm
