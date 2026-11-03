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
#include <algorithm>
#include <cstdlib>
#include <vector>
#include "mm/astar.hpp"
#include "mm/heuristics.hpp"
#include "mm/known_maze.hpp"
#include "mm/maze_io.hpp"
#include "mm/world.hpp"

namespace mm
{

    constexpr int kMAZE = MAZE_W * MAZE_H;

    enum struct HeurKind : int
    {
        kManhattan = 0,
        kTrueDistance,
        kOptimistic
    };

    struct PhaseHeur
    {
        HeurKind kind = HeurKind::kOptimistic;
        int epsNum = 2; // weighted A* default: h * 2 (explore fast, path may cost +
        int epsDen = 1;
    };

    struct SolverConfig
    {
        PhaseHeur phase1, phase2;
        // Turn-aware planning cost: 64 == 0.25 cell-time per 90-deg turn.
        Cost turnCost = 64;
        double vband = 10;
        bool visualize = false;
        // Flood-fill "mood": ignore the improvement-band certificate entirely and
        // explore the classic beginner way -- a start->center->start scheme with
        // the greedy sense-per-cell walker. Default: off (exact certificate mode).
        bool floodFill = false;
        // Flood-fill walk refinements:
        bool ffStraight = false; // on pick ties, prefer continuing straight
        bool ffAvoid = false;    // leg 1: on pick ties, prefer cells off the outbound path
        // CURIOSITY_WEIGHT: penalty (in flood units) added to the flood cost of
        // crossing an UNEXPLORED face during the directional kinematic flood.
        // Unexplored walls are neither open nor closed: the wave only pushes
        // into unknown territory while a shortcut outweighs the curiosity.
        // DEFAULT 64 (sweep optimum: one unknown edge ~= one extra cell or
        // turn); 0 = classic optimistic flood. Sweepable: --ff-curio.
        int ffCurio = 64;
        // Mix mood: flood to center, optimized flood back to start, unsealed
        // phase-4 sprint to center, then phase-2 certificate exploration, then
        // the sealed sprint straight to center. See solveMix().
        bool mixMode = false;
        // Number of exploration legs (even: the walk ends at the start, so the
        // sprint starts right away; odd values are coerced down to even).
        // Default 2 = center, then start. Sweepable: --ff-legs.
        int ffLegs = 2;
        // Trip economics (phase 2): chase a frontier edge only if its upper-bound
        // saving exceeds tripWorth * trip distance. Ratio 0 = chase everything the
        // improvement band offers; ratio 1 = legacy --allow-lazy. DEFAULT: -1 =
        // exact mode (certificate = provably best path) — measured optimal on
        // 519 classic mazes: any chase-rate discount collapses to instant
        // certification with strictly worse runs (median run 84.5 vs 70.5).
        double tripWorth = -1.0;
        // Expected-value chase rate (ratio mode only): a frontier edge's potential
        // saving counts as edgePrior * saving when testing "does it repay
        // worthRatio * trip?". DEFAULT: 1.0 = neutral (no effect in exact mode,
        // and sweep-invariant otherwise); experimental — left for scoring
        // formulas where exploration is not counted.
        double edgePrior = 1.0;
        // Tie-break scope (0..3): count in-band edges whose chosen endpoint lies
        // within Manhattan radius R of the candidate trip target (0 = legacy
        // same-cell tie-break). DEFAULT: 0 — sweep showed R>0 is score-neutral
        // (trips re-planned locally anyway), kept for hardware-maze tuning.
        int clusterRadius = 0;
        // Debug/trace channels folded into config: getenv() probing sat in the
        // per-step hot loop before (real cost on MCU; cache line misses on desktop).
        bool dbgCrit = false;   // was MM_DEBUG
        bool dbgTrace = false;  // was MM_TRACE
        bool dbgTrace2 = false; // was MM_TRACE2
    };

    const int g_vband = 1;

    struct SolverStats
    {
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

    class Solver
    {
    public:
        Solver(IWorld& w, const Maze& truth, const SolverConfig& cfg) :
            wTrip_(cfg.tripWorth),
            edgePrior_(cfg.edgePrior),
            clusterRadius_(cfg.clusterRadius),
            vDebug_(cfg.dbgCrit),
            vTrace_(cfg.dbgTrace),
            vTrace2_(cfg.dbgTrace2),
            w_(w),
            truth_(truth),
            cfg_(cfg)
        {
        }

        bool solve()
        { // false on crash / planning failure
            st_ = SolverStats{};
            mx_ = 0;
            my_ = 0;
            mdir_ = D_N;
            arrivalFace_ = -1;
            known_ = KnownMaze{};
            vizPathColor_ = 'g'; // phase 1 / flood legs draw green
            sealed_ = false;
            vizInit();
            if (cfg_.floodFill)
                return solveFloodFill();
            if (cfg_.mixMode)
                return solveMix();
            if (!phase1())
            {
                if (vDebug_)
                    fprintf(stderr, "PHASE1 FAILED\n");
                return false;
            }
            if (!phase2())
            {
                if (vDebug_)
                    fprintf(stderr, "PHASE2 FAILED\n");
                return false;
            }
            if (!phase3())
            {
                if (vDebug_)
                    fprintf(stderr, "PHASE3 FAILED\n");
                return false;
            }
            if (!phase4())
            {
                if (vDebug_)
                    fprintf(stderr, "PHASE4 FAILED\n");
                return false;
            }
            return true;
        }
        const SolverStats& stats() const { return st_; }

    private:
        enum struct Phase
        {
            P1,
            P2
        };

        // ---------- planning ----------
        // Run stats phase; see phase docs above.
        template <class GoalFn>
        PathResult plan(int sx, int sy, Dir sdir, GoalFn atGoal, const Maze& snap, const PhaseHeur& ph,
                        const Cost* hOverride = nullptr)
        {
            Cost tbl[kMAZE];
            const Cost* h = hOverride;
            if (!h)
            {
                switch (ph.kind)
                {
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
        // Changed faces are bit-packed into chFaces_ (bit per Dir; every sensed
        // face belongs to the current cell, so 4 bits suffice). The relevance
        // filter in runTo() derives the neighbor endpoints from (mx_,my_) — no
        // parallel change arrays needed (cache/RAM conscious).
        uint8_t chFaces_ = 0;
        bool sense()
        {
            bool changed = false;
            chFaces_ = 0;
            struct Q
            {
                bool wall;
                Dir d;
            } qs[3] = {{w_.wallFront(), mdir_},
                       {w_.wallRight(), static_cast<Dir>((mdir_ + 1) % kDirCount)},
                       {w_.wallLeft(), static_cast<Dir>((mdir_ + 3) % kDirCount)}};
            for (auto& q : qs)
            {
                WallStatus s = q.wall ? kWall : kOpen;
                if (known_.status(mx_, my_, q.d) != s)
                {
                    known_.setStatus(mx_, my_, q.d, s);
                    changed = true;
                    chFaces_ |= static_cast<uint8_t>(1u << q.d);
                }
            }
            // back side: open (we came through it) ONLY if we actually entered
            // this cell through it: after turn-in-place, or in the initial pose
            // (start corner: the back is the border wall!), the 3-sensor rig
            // cannot see the back -- leave it unknown (firmware-honest).
            Dir back = static_cast<Dir>((mdir_ + 2) % kDirCount);
            if (static_cast<int>(back) == arrivalFace_ &&
                known_.status(mx_, my_, back) == kUnknown)
            {
                known_.setStatus(mx_, my_, back, kOpen);
                changed = true;
                chFaces_ |= static_cast<uint8_t>(1u << back);
            }
            return changed;
        }

        // How many forward *moves* we can execute in one continuous batch starting
        // at path[pi] (we are at path[pi]). Crossing out of path[j] into path[j+1]
        // requires the exit face of path[j] to be known-open. On unknown exit we
        // stop (arrive-sense-replan continues in the caller). A wall exit means the
        // plan is stale; stopping is still safe (no crash).
        int maxRun(const std::vector<State>& path, size_t pi) const
        {
            int run = 0;
            size_t j = pi;
            while (j + 1 < path.size())
            {
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
        enum struct RunStatus
        {
            kDone,
            kStuck
        };

        // Snapshot with unknown faces treated as walls (for sealed open-loop plans).
        Maze pessimisticSnapshot() const
        {
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
        RunStatus driveOpen(const std::vector<State>& plan, GoalFn atGoal)
        {
            size_t pi = SIZE_MAX;
            for (size_t j = 0; j < plan.size(); ++j)
                if (plan[j].x == mx_ && plan[j].y == my_ && plan[j].dir == mdir_)
                {
                    pi = j;
                    break;
                }
            if (pi == SIZE_MAX)
                return RunStatus::kStuck; // pose not on plan: replan
            if (atGoal(mx_, my_))
                return RunStatus::kDone;
            while (pi + 1 < plan.size())
            {
                const State& a = plan[pi];
                const State& b = plan[pi + 1];
                if (a.dir == b.dir)
                {
                    // straight batch across plans[j..j+n]
                    int run = 1;
                    while (pi + run + 1 < plan.size() && plan[pi + run].dir == plan[pi + run + 1].dir)
                    {
                        ++run;
                    }
                    if (!w_.moveForward(run))
                    {
                        if (vDebug_)
                        {
                            int wx, wy, wd;
                            w_.debugPos(&wx, &wy, &wd);
                            fprintf(stderr,
                                    "DRIVE-FAIL run=%d pose=(%d,%d,%d) world=(%d,%d,%d) faceSt=%d "
                                    "nextSt=%d\n",
                                    run, mx_, my_, (int)mdir_, wx, wy, wd,
                                    (int)known_.status(a.x, a.y, a.dir),
                                    (int)known_.status(b.x, b.y, b.dir));
                        }
                        return RunStatus::kStuck;
                    }
                    advanceStats(run);
                    mx_ += static_cast<int>(kDx[mdir_]) * run;
                    my_ += static_cast<int>(kDy[mdir_]) * run;
                    arrivalFace_ = (mdir_ + 2) % kDirCount; // face crossed entering the new cell
                    pi += run;
                    onCellVisit();
                }
                else
                {
                    // turn in place until facing b.dir (up to 2 x 90 deg for a U-turn)
                    int rel = (static_cast<int>(b.dir) - static_cast<int>(a.dir) + kDirCount) % kDirCount;
                    while (mdir_ != b.dir)
                    {
                        if (rel == 1 || rel == 2)
                        {
                            w_.turnRight();
                            mdir_ = static_cast<Dir>((mdir_ + 1) % kDirCount);
                        }
                        else
                        {
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
        RunStatus runTo(GoalFn atGoal, const PhaseHeur& ph, const Cost* hOverride = nullptr)
        {
            // (F) sealed maze: no sensing/replanning during the endgame runs -- drive
            // open-loop on a *pessimistic* snapshot (unknown faces treated as walls),
            // so every crossed face is provably known-open and a crash is impossible.
            if (sealed_)
            {
                Maze pes = pessimisticSnapshot();
                PathResult fin = plan(mx_, my_, mdir_, atGoal, pes, ph, hOverride);
                if (!fin.found)
                    return RunStatus::kStuck;
                if (vTrace_)
                {
                    fprintf(stderr, "RUNTO-sealed plan:");
                    for (auto& s : fin.states)
                        fprintf(stderr, " (%d,%d,%d)", s.x, s.y, (int)s.dir);
                    fprintf(stderr, " pose=(%d,%d,%d)\n", mx_, my_, (int)mdir_);
                }
                return driveOpen(fin.states, atGoal);
            }
            const int kMaxSteps = 100000; // safety breaker
            int steps = 0;
            int noops = 0; // consecutive iterations with nothing executed
            PathResult cur = plan(mx_, my_, mdir_, atGoal, known_.snapshot(), ph, hOverride);
            if (!cur.found)
                return RunStatus::kStuck;
            while (steps++ < kMaxSteps)
            {
                bool changed = sense(); // sense at every cell visit (3 faces + back)
                vizVisit();
                if (vTrace2_)
                    fprintf(stderr, "RUNTO pos=(%d,%d,%d) chg=%d faces=%d%d%d%d\n", mx_, my_, (int)mdir_, changed,
                            (int)known_.status(mx_, my_, D_N), (int)known_.status(mx_, my_, D_E),
                            (int)known_.status(mx_, my_, D_S), (int)known_.status(mx_, my_, D_W));
                if (atGoal(mx_, my_))
                {
                    if (!changed)
                        return RunStatus::kDone;
                    // arrived + fresh walls: replan once, then confirm
                    ++st_.replans;
                    cur = plan(mx_, my_, mdir_, atGoal, known_.snapshot(), ph, hOverride);
                    if (!cur.found || cur.states.size() <= 1)
                        return RunStatus::kDone;
                    continue;
                }
                // Relevance filter (cheap incremental replanning): a wall freshly
                // sensed *off* the remaining plan cannot invalidate the part we are
                // about to drive -- skip the replan and keep the current one. A
                // changed face is an endpoint pair (mx_,my_)(ex,ey); it matters only
                // if that same unordered pair is a crossing edge of the plan ahead.
                if (changed)
                {
                    size_t qi = SIZE_MAX;
                    for (size_t j = 0; j < cur.states.size(); ++j)
                        if (cur.states[j].x == mx_ && cur.states[j].y == my_ &&
                            cur.states[j].dir == mdir_)
                        {
                            qi = j;
                            break;
                        }
                    bool relevant = qi != SIZE_MAX;
                    if (relevant)
                    {
                        relevant = false;
                        for (int di = 0; di < kDirCount && !relevant; ++di)
                        {
                            if (!(chFaces_ & (1u << di)))
                                continue;
                            int ex = mx_ + kDx[di], ey = my_ + kDy[di];
                            for (size_t k = qi; k + 1 < cur.states.size(); ++k)
                            {
                                const State& u = cur.states[k];
                                const State& v = cur.states[k + 1];
                                if (u.x == v.x && u.y == v.y)
                                    continue; // turn-in-place state, no crossing
                                if ((u.x == ex && u.y == ey && v.x == mx_ && v.y == my_) ||
                                    (u.x == mx_ && u.y == my_ && v.x == ex && v.y == ey))
                                {
                                    relevant = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (relevant)
                    {
                        ++st_.replans;
                        Maze snap = known_.snapshot(); // keep for debugging
                        cur = plan(mx_, my_, mdir_, atGoal, snap, ph, hOverride);
                        if (!cur.found)
                        {
                            if (vDebug_)
                            {
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
                                fprintf(stderr, "STUCK at (%d,%d,%d) goal=%d,%d lead=%d unknown=%d\n%s", mx_, my_,
                                        (int)mdir_, truth_.goalCell(0).x, truth_.goalCell(0).y,
                                        dd[idx(mx_, my_)] == INT32_MAX, known_.unknownCount(), dumpMaze(snap).c_str());
                                FILE* f = fopen("/tmp/opencode/dumpsnap.txt", "w");
                                fprintf(f, "%s", dumpMaze(snap).c_str());
                                fclose(f);
                            }
                            return RunStatus::kStuck;
                        }
                    }
                }
                // invariant: solver pose == world pose
                {
                    int wx, wy, wd;
                    w_.debugPos(&wx, &wy, &wd);
                    if (wx >= 0 && (wx != mx_ || wy != my_ || wd != (int)mdir_))
                    {
                        fprintf(stderr, "POSE DESYNC solver=(%d,%d,%d) world=(%d,%d,%d)\n", mx_, my_, (int)mdir_, wx,
                                wy, wd);
                        return RunStatus::kStuck;
                    }
                }
                // execute one batch along cur.states starting at our index
                size_t pi = SIZE_MAX;
                for (size_t j = 0; j < cur.states.size(); ++j)
                    if (cur.states[j].x == mx_ && cur.states[j].y == my_ && cur.states[j].dir == mdir_)
                    {
                        pi = j;
                        break;
                    }
                if (pi == SIZE_MAX)
                { // plan didn't include our (state,dir): replan next iter
                    ++st_.replans;
                    cur = plan(mx_, my_, mdir_, atGoal, known_.snapshot(), ph, hOverride);
                    if (!cur.found || cur.states.empty())
                        return RunStatus::kStuck;
                    continue;
                }
                bool acted = false;
                int run = 0;
                if (pi + 1 < cur.states.size())
                {
                    const State& a = cur.states[pi];
                    const State& b = cur.states[pi + 1];
                    if (a.dir == b.dir)
                        run = maxRun(cur.states, pi);
                }
                if (run > 0)
                {
                    if (vTrace_)
                        fprintf(stderr, "MOVE run=%d from=(%d,%d) dir=%d to=(%d,%d)\n", run, mx_, my_, (int)mdir_,
                                mx_ + kDx[mdir_] * run, my_ + kDy[mdir_] * run);
                    if (!w_.moveForward(run))
                        return RunStatus::kStuck; // crash / sim end
                    advanceStats(run);
                    mx_ += kDx[mdir_] * run;
                    my_ += kDy[mdir_] * run;
                    arrivalFace_ = (mdir_ + 2) % kDirCount; // face crossed entering the new cell
                    acted = true;
                }
                else
                {
                    // turn according to plan's next move
                    if (pi + 1 >= cur.states.size())
                    {
                        // at goal-turn state? shouldn't happen: atGoal checked above
                        return RunStatus::kDone;
                    }
                    Dir nd = cur.states[pi + 1].dir;
                    while (mdir_ != nd)
                    {
                        int rel = (static_cast<int>(nd) - static_cast<int>(mdir_) + kDirCount) % kDirCount;
                        if (rel == 3)
                        {
                            w_.turnLeft();
                            mdir_ = static_cast<Dir>((mdir_ + 3) % kDirCount);
                        }
                        else
                        {
                            w_.turnRight();
                            mdir_ = static_cast<Dir>((mdir_ + 1) % kDirCount);
                        }
                        ++st_.totalTurns;
                        if (runAtStart_)
                            ++curRunTurns_;
                        acted = true;
                    }
                }
                // no-op guard: the stale plan orders nothing executable from this pose
                // (e.g., its next edge stops at an unknown face more than once). One
                // forced replan fixes it; a few no-progress rounds in a row means the
                // target is unreachable: report kStuck instead of spinning forever.
                if (!acted)
                {
                    if (++noops > 8)
                        return RunStatus::kStuck;
                    ++st_.replans;
                    cur = plan(mx_, my_, mdir_, atGoal, known_.snapshot(), ph, hOverride);
                    if (!cur.found)
                        return RunStatus::kStuck;
                    continue;
                }
                noops = 0;
                // run-boundary bookkeeping (start / goal visits)
                onCellVisit();
            }
            return atGoal(mx_, my_) ? RunStatus::kDone : RunStatus::kStuck;
        }

        void advanceStats(int run)
        {
            st_.totalDistance += run;
            st_.totalEff += (run <= 2) ? run : 2.0 + 0.5 * (run - 2);
            if (runAtStart_)
            {
                curRunDist_ += run;
                curRunEff_ += (run <= 2) ? run : 2.0 + 0.5 * (run - 2);
            }
        }
        void onCellVisit()
        {
            if (mx_ == 0 && my_ == 0)
            {
                runAtStart_ = true;
                curRunDist_ = 0;
                curRunEff_ = 0;
                curRunTurns_ = 0;
            }
            if (runAtStart_ && isGoalHere())
            {
                runAtStart_ = false;
                double total = curRunEff_ + curRunTurns_;
                if (st_.bestRunEff < 0 || total < st_.bestRunEff + st_.bestRunTurns)
                {
                    st_.bestRunDist = curRunDist_;
                    st_.bestRunEff = curRunEff_;
                    st_.bestRunTurns = curRunTurns_;
                }
            }
        }
        bool isGoalHere() const
        {
            return (mx_ == MAZE_W / 2 || mx_ == MAZE_W / 2 - 1) && (my_ == MAZE_H / 2 || my_ == MAZE_H / 2 - 1);
        }

        // ---------- visualization (worlds that ignore setColor/setText: no-op)
        // ---------- Colors: r=goal, g=phase-1/flood-legs path, a=light gray
        // exploration (phase 2 / trips), o=frontier trip target,
        // b=blue speed-run highlight (Sprint1 + Sprint2, drawn == driven).
        // Text: number in each visited cell = optimistic steps-to-center
        // at visit time; refreshed with the final true distance once the path is
        // certified.
        bool cellVis_[MAZE_W * MAZE_H] = {};
        char cellPaint_[MAZE_W * MAZE_H] = {}; // last stage color painted (0 = never)
        char vizPathColor_ = 'g'; // color for newly visited cells (g = phase 1, a = exploration)

        void vizInit()
        {
            w_.clearAllColor();
            w_.clearAllText();
            std::memset(cellVis_, 0, sizeof(cellVis_));
            std::memset(cellPaint_, 0, sizeof(cellPaint_));
            vizGoals();
        }

        void vizGoals()
        {
            Maze snap = known_.snapshot();
            for (int i = 0; i < snap.goalCount() && i < 4; ++i)
            {
                Cell g = snap.goalCell(i);
                w_.setColor(g.x, g.y, 'r');
            }
        }

        // distance-to-center table on the current known map (unknown=optimistic)
        void vizDistanceTable(const Maze& snap, Cost out[kMAZE])
        {
            int src[4], n = 0;
            for (int i = 0; i < 4 && i < snap.goalCount(); ++i)
            {
                Cell g = snap.goalCell(i);
                src[n++] = idx(g.x, g.y);
            }
            dijkstraCells(snap, src, n, out);
        }

        void vizVisit()
        {
            int c = idx(mx_, my_);
            const bool fresh = !cellVis_[c];
            // recolor when this cell was last painted in another stage's color
            // (e.g. a phase-1 green cell re-walked during exploration turns light
            // gray); skip only when it already carries the current stage color.
            if (!fresh && cellPaint_[c] == vizPathColor_)
                return;
            cellVis_[c] = true;
            cellPaint_[c] = vizPathColor_;
            w_.setColor(mx_, my_, vizPathColor_);
            if (!fresh)
                return; // distance label already present
            Maze snap = known_.snapshot();
            Cost dc[kMAZE];
            vizDistanceTable(snap, dc);
            if (dc[c] < INT32_MAX)
            {
                char b[8];
                std::snprintf(b, sizeof b, "%d", dc[c] / kCostUnit);
                w_.setText(mx_, my_, b);
            }
        }

        void vizTripTarget(int tx, int ty) { w_.setColor(tx, ty, 'o'); }

        void vizFinal(const PathResult& best)
        {
            Maze snap = known_.snapshot();
            w_.clearAllColor();
            vizGoals();
            // repaint the stage history: the green phase-1/flood path and the
            // light-gray exploration trail survive the clear (clearing alone
            // blanked every previously visited cell, so exploration after a
            // sprint looked uncolored).
            for (int y = 0; y < MAZE_H; ++y)
                for (int x = 0; x < MAZE_W; ++x)
                {
                    int c = idx(x, y);
                    if (cellPaint_[c])
                        w_.setColor(x, y, cellPaint_[c]);
                }
            // final true distances on every visited cell
            Cost dc[kMAZE];
            vizDistanceTable(snap, dc);
            for (int y = 0; y < MAZE_H; ++y)
                for (int x = 0; x < MAZE_W; ++x)
                {
                    int c = idx(x, y);
                    if (!cellVis_[c] || dc[c] == INT32_MAX)
                        continue;
                    char b[8];
                    std::snprintf(b, sizeof b, "%d", dc[c] / kCostUnit);
                    w_.setText(x, y, b);
                }
            // speed-run highlight in blue: Sprint1 and Sprint2 both draw exactly
            // the path they will drive (pessimistic, map locked), so the
            // highlight doubles as the driven-path record
            for (auto s : best.states)
            {
                w_.setColor(s.x, s.y, 'b');
                cellPaint_[idx(s.x, s.y)] = 'b';
            }
        }

        // ---------- flood-fill "mood" (cfg_.floodFill) ----------
        // Starts from the beginner firmware scheme but plans every leg with a
        // DIRECTIONAL KINEMATIC FLOOD (ffKinFlood): a backward Dijkstra over
        // (cell, heading) states with straight=64 / 90°-turn=64 / u-turn=128
        // and a curiosity penalty (cfg_.ffCurio) on unexplored faces -- no
        // diagonals (10.5 cm mouse). Each leg (center, then back to start)
        // descends that flood cell by cell, sensing fresh cells and never
        // moving blind. Endgame identical to phases 3/4: return to start,
        // seal, sprint on the best route.
        uint8_t ffVis_[kMAZE / 8 + 1] = {}; // bit-packed per-solve visited map
        uint8_t ffAvoid_[kMAZE / 8 + 1] = {}; // leg-0 outbound cells (bit-packed)

        // Directional kinematic flood scratch (single-threaded, no allocation).
        // Heap size bound: <= 16 goal-heading seeds + <= 4 relaxations per one
        // of the 1024 states = 4112 pushes worst case; 4480 is safe headroom.
        struct FFHE
        {
            Cost k;
            int s; // state = cell*4 + heading
        };
        FFHE ffHeap_[4480];
        int ffHn_ = 0;

        void ffHeapPush(Cost k, int s)
        {
            ffHeap_[ffHn_] = {k, s};
            std::push_heap(ffHeap_, ffHeap_ + ++ffHn_,
                           [](const FFHE& a, const FFHE& b) { return a.k > b.k; });
        }
        FFHE ffHeapPop()
        {
            std::pop_heap(ffHeap_, ffHeap_ + ffHn_,
                          [](const FFHE& a, const FFHE& b) { return a.k > b.k; });
            return ffHeap_[--ffHn_];
        }
        static constexpr Cost kFfStraight = 64; // kinematic scale (planning only)
        static constexpr Cost kFfTurn = 64;     // 90° penalty; straight+turn = 128
        static constexpr Cost kFfUturn = 128;   // u-turn penalty stands alone
        static constexpr Cost kFfInf = INT32_MAX;

        // Backward Dijkstra from the goal seeds over the (cell, heading) state
        // graph: kin[c][d] = min real cost to travel from (c, facing d) to any
        // goal. Edge cost of arriving at c heading d (from predecessor p):
        //   kFfStraight + turn(dh -> d) + curiosity(face c<->p)
        // where turn is 0 / kFfTurn (90°) / kFfUturn, and curiosity
        // (cfg_.ffCurio) is added only when that face is still UNEXPLORED.
        // A kWall face can never be crossed; kUnknown faces can.
        void ffKinFlood(const int* src, int nSrc, Cost kin[kMAZE][kDirCount])
        {
            for (int c = 0; c < kMAZE; ++c)
                for (int d = 0; d < kDirCount; ++d)
                    kin[c][d] = kFfInf;
            ffHn_ = 0;
            for (int i = 0; i < nSrc; ++i)
                for (int d = 0; d < kDirCount; ++d)
                    if (kin[src[i]][d] == kFfInf)
                    {
                        kin[src[i]][d] = 0; // any arrival heading ends the leg
                        ffHeapPush(0, src[i] * kDirCount + d);
                    }
            while (ffHn_)
            {
                FFHE e = ffHeapPop();
                int c = e.s / kDirCount, d = e.s % kDirCount;
                if (kin[c][d] < e.k)
                    continue; // stale entry
                // the physical face crossed on arrival into c heading d
                Dir back = static_cast<Dir>((d + 2) % kDirCount);
                int cx = c & 0xF, cy = c >> 4;
                WallStatus st = known_.status(cx, cy, back);
                if (st == kWall)
                    continue; // cannot arrive heading d through a wall
                Cost curios = st == kUnknown ? static_cast<Cost>(cfg_.ffCurio) : 0;
                if (cx + kDx[back] < 0 || cx + kDx[back] >= MAZE_W || cy + kDy[back] < 0 ||
                    cy + kDy[back] >= MAZE_H)
                    continue; // border: no predecessor cell
                int p = idx(cx + kDx[back], cy + kDy[back]);
                for (int dh = 0; dh < kDirCount; ++dh)
                {
                    Cost turn = dh == d ? 0
                                : dh == (d + 2) % kDirCount ? kFfUturn
                                                            : kFfTurn;
                    Cost nk = e.k + kFfStraight + turn + curios;
                    if (nk < kin[p][dh])
                    {
                        kin[p][dh] = nk;
                        ffHeapPush(nk, p * kDirCount + dh);
                    }
                }
            }
        }
        void ffVisReset() { std::memset(ffVis_, 0, sizeof(ffVis_)); }
        bool ffVis(int c) const { return (ffVis_[c >> 3] >> (c & 7)) & 1; }
        void ffVis(int c, bool v)
        {
            if (v)
                ffVis_[c >> 3] |= (1u << (c & 7));
            else
                ffVis_[c >> 3] &= ~(1u << (c & 7));
        }

        // One leg: descend toward atGoal on the DIRECTIONAL KINEMATIC flood of
        // the CURRENT known map (ffKinFlood) -- firmware-style: plan a whole
        // kinematic descent, then DRIVE it, re-planning only when a wall is
        // revealed ON the plan (senses land during every rotation and on fresh
        // cell arrivals). Per-step re-picking (the old walker) thrashed: every
        // new bit of information re-decided the heading ("keeps switching
        // between looking forward and sideways") and re-steered around the
        // trail (re-exploring known cells). Notable pieces:
        //   * no blind move: an unknown crossing face is revealed by facing it
        //     first (rotations sense); a revealed wall replans from the pose
        //   * faces physically crossed are marked open (KnownMaze symmetry) --
        //     revisited-cell moves skip sense(), so without this the trail's
        //     own faces stayed kUnknown and the curiosity weight kept taxing
        //     proven-open edges (re-explore + re-route artifact)
        //   * the plan is the pure Dijkstra descent from the current state:
        //     value(d) = turn(h -> d) + 64 + kin[nc][d]  (kin already contains
        //     the crossing curiosity), so kin-to-go strictly decreases by >= 64
        //     per step: the plan is cycle-free, and ties prefer non-outbound
        //     neighbor (ffAvoid), then straight continuation (ffStraight),
        //     then the first of N,E,S,W (firmware)
        //   * kin[cur][mdir_] == INF means even the optimistic map has no
        //     route: the leg fails
        // avoid: bit-packed cells to deprioritize (return legs); null disables.
        // record: bit-packed set accumulated with this leg's cells (leg 0).
        // Returns a debug FNV-1a hash of the walked cells, 0 on failure.
        template <class GoalFn>
        uint32_t ffWalkLeg(GoalFn atGoal, const int* floodSrc, int nSrc,
                           const uint8_t* avoid, uint8_t* record)
        {
            auto gbb = [](const uint8_t* m, int c) { return (m[c >> 3] >> (c & 7)) & 1; };
            auto sbb = [](uint8_t* m, int c) { m[c >> 3] |= static_cast<uint8_t>(1u << (c & 7)); };
            const bool useAv = avoid && cfg_.ffAvoid;
            Cost kin[kMAZE][kDirCount];
            uint32_t h = 2166136261u;
            int flops = 0;
            int moves = 0;
            for (int round = 0; round < 4096; ++round)
            {
                if (atGoal(mx_, my_))
                    return h; // leg complete
                int c = idx(mx_, my_);
                // Unconditional sense: the live side sensors read front/right/
                // left for FREE at every cell visit -- gating them behind
                // ffVis (first-visit-since-solve) left stale unknowns around
                // REVISITED cells, so the plan kept steering toward unknown
                // faces beside known corridors and the mouse rotated to face
                // a wall it should already know, only to turn straight back.
                sense();
                ffVis(c, true);
                ffKinFlood(floodSrc, nSrc, kin);
                const int mh = static_cast<int>(mdir_);
                if (kin[c][mh] == kFfInf)
                    return 0; // no optimistic route to the goal from here
                // ---- extract the committed kinematic descent (cycle-free: the
                // greedy argmin equals the Dijkstra edge, kin drops >= 64/step)
                Dir plan[kMAZE];
                int px = mx_, py = my_, pd = mh, pn = 0;
                bool stuck = false;
                while (!atGoal(px, py) && pn < kMAZE)
                {
                    int pc = idx(px, py);
                    const Cost kinCur = kin[pc][pd];
                    Cost bestV = kFfInf;
                    int bestRw = 3, bestAv = 1;
                    Dir pick = D_N;
                    bool any = false;
                    for (int d = 0; d < kDirCount; ++d)
                    {
                        Dir dir = static_cast<Dir>(d);
                        if (known_.status(px, py, dir) == kWall)
                            continue;
                        if (px + kDx[d] < 0 || px + kDx[d] >= MAZE_W || py + kDy[d] < 0 ||
                            py + kDy[d] >= MAZE_H)
                            continue;
                        int nc = idx(px + kDx[d], py + kDy[d]);
                        if (kin[nc][d] == kFfInf)
                            continue; // no route from there either
                        if (kin[nc][d] >= kinCur)
                            continue; // no strict improvement: keep the plan cycle-free
                        Cost turnv = d == pd ? 0 : d == (pd + 2) % kDirCount ? kFfUturn : kFfTurn;
                        Cost v = turnv + kFfStraight + kin[nc][d];
                        int rw = cfg_.ffStraight
                                     ? (d == pd ? 0 : d == (pd + 2) % kDirCount ? 2 : 1)
                                     : 0;
                        int avp = useAv ? gbb(avoid, nc) : 0;
                        bool take;
                        if (!any)
                            take = true;
                        else if (v != bestV)
                            take = v < bestV;
                        else if (avp != bestAv)
                            take = avp < bestAv;
                        else if (rw != bestRw)
                            take = rw < bestRw;
                        else
                            take = false; // full tie: first direction (firmware)
                        if (take)
                        {
                            bestV = v;
                            bestRw = rw;
                            bestAv = avp;
                            pick = dir;
                            any = true;
                        }
                    }
                    if (!any)
                    {
                        if (pn == 0)
                            stuck = true; // still-stepping fallback handles it
                        break;            // drive the extracted prefix
                    }
                    plan[pn++] = pick;
                    px += static_cast<int>(kDx[(int)pick]);
                    py += static_cast<int>(kDy[(int)pick]);
                    pd = static_cast<int>(pick);
                }
                if (pn == 0)
                {
                    if (!stuck || ++flops > 1)
                        return 0; // still trapped on a fresh flood: give up
                    continue;     // one re-flood round
                }
                flops = 0;
                // ---- drive the committed plan; replan on any wall reveal
                for (int i = 0; i < pn; ++i)
                {
                    Dir pick = plan[i];
                    // PRE-TURN wall check: arrival sense() mid-plan may have
                    // revealed a new wall AFTER this plan was extracted.
                    // Rotating first and checking after is too late -- the
                    // mouse then turns into a face its own side sensors just
                    // proved to be a wall. Abort to a fresh replan instead.
                    if (known_.status(mx_, my_, pick) == kWall)
                        break; // replan from the current pose
                    faceDir(pick); // turn to pick -- rotations sense, unknowns revealed
                    if (known_.status(mx_, my_, pick) == kUnknown)
                        sense(); // no rotation happened: read the front face
                    if (known_.status(mx_, my_, pick) == kWall)
                        break; // revealed wall ON the plan: replan from here
                    if (++moves > 1200)
                        return 0; // total-move cap
                    if (!w_.moveForward(1))
                        return 0; // stall/crash detection analog
                    // We PHYSICALLY crossed the pick face: record it open (and
                    // its symmetric twin via KnownMaze).
                    known_.setStatus(mx_, my_, pick, kOpen);
                    advanceStats(1);
                    mx_ += static_cast<int>(kDx[pick]);
                    my_ += static_cast<int>(kDy[pick]);
                    arrivalFace_ = (static_cast<int>(pick) + 2) % kDirCount;
                    if (record)
                        sbb(record, idx(mx_, my_)); // track the outbound path
                    h = h * 16777619u ^ static_cast<uint32_t>(idx(mx_, my_) + 1);
                    if (!h)
                        h = 0x5EEDu; // 0 is the "failure" sentinel — never let the hash land on it
                    onCellVisit();
                    vizVisit();
                    // Unconditional arrival sense too: same live-sensor
                    // reasoning as the round-top sense -- a revisit's
                    // front/right/left faces must land or stale unknowns
                    // beside known corridors later force wasted rotations.
                    sense();
                    ffVis(idx(mx_, my_), true);
                }
            }
            return 0; // replan-round cap
        }

        bool solveFloodFill()
        {
            st_.phase = 1;
            auto atCenter = [&](int x, int y) { return isGoalTruth(x, y); };
            auto atStart = [&](int x, int y) { return x == 0 && y == 0; };
            // flood sources depend on the leg direction
            Maze snap = known_.snapshot();
            int startSrc[1];
            int centerSrc[4];
            for (int i = 0; i < 4; ++i)
            {
                Cell g = snap.goalCell(i);
                centerSrc[i] = idx(g.x, g.y);
            }
            startSrc[0] = idx(0, 0);
            ffVisReset();
            std::memset(ffAvoid_, 0, sizeof(ffAvoid_));
            const uint8_t* avoidPtr = cfg_.ffAvoid ? ffAvoid_ : nullptr;
            // N exploration legs: even-indexed legs go to the center, odd legs
            // back to the start (an even count ends at the start, where the
            // sprint already begins). Leg 0 records its cell trail into
            // ffAvoid_; return legs may use it for the tie-break (ffAvoid).
            const int legs =
                cfg_.ffLegs < 2 ? 2 : (cfg_.ffLegs & ~1); // even, >= 2
            for (int legNo = 0; legNo < legs; ++legNo)
            {
                bool toCenter = (legNo % 2 == 0);
                uint32_t hp = toCenter ? ffWalkLeg(atCenter, centerSrc, 4, nullptr, ffAvoid_)
                                       : ffWalkLeg(atStart, startSrc, 1, avoidPtr, nullptr);
                if (vDebug_)
                    fprintf(stderr, "FF leg=%d toCenter=%d hash=%.8x unknown=%d\n", legNo, toCenter, hp,
                            known_.unknownCount());
                if (hp == 0)
                    return false; // leg gave up: report failure
            }
            Maze end = known_.snapshot();
            PathResult best = plan(0, 0, D_N, atCenter, end, PhaseHeur{HeurKind::kTrueDistance, 1, 1});
            st_.certBestKnown = best.found ? best.cost / static_cast<double>(kCostUnit) : -1;
            st_.certBound = st_.certBestKnown;
            st_.certifiedAfterCells = static_cast<int>(st_.totalDistance);
            // endgame: back to start, seal, then the drawn best-run sprint
            if (mx_ != 0 || my_ != 0)
                if (runTo(atStart, cfg_.phase1) == RunStatus::kStuck)
                    return false;
            sealed_ = true;
            return phase4();
        }

        // ---------- mix mood (cfg_.mixMode, competitive --mix-mode) ----------
        // 5-stage state machine; each transition fires on reaching Start or
        // Center, verbatim reuse of the existing phases:
        //   1. State_SearchCenter: cautious kinematic flood leg to the center
        //      (floodfill walker; curiosity + tie-breaks apply)
        //   2. State_OptimizedReturn: same walker back to start (--ff-curio 64
        //      directional flood; sweeps for initial shortcuts, avoid trail)
        //   3. State_Sprint1: LOCKED map -- open-loop pessimistic-planned
        //      speed run Start -> Center, no sensing/exploration (a scored run)
        //   4. State_DeepExplore: exact improvement-band explorer (phase 2
        //      trip economics) hunting the walls that can improve the sprint,
        //      then the pessimistic return to start (phase 3) so the next
        //      sprint DEPARTS from the start cell and is scored
        //   5. State_Sprint2: max-velocity run straight to the center, sealed
        //      (map locked, drawn == driven)
        bool solveMix()
        {
            st_.phase = 1;
            auto atCenter = [&](int x, int y) { return isGoalTruth(x, y); };
            auto atStart = [&](int x, int y) { return x == 0 && y == 0; };
            Maze snap = known_.snapshot();
            int centerSrc[4];
            for (int i = 0; i < 4; ++i)
            {
                Cell g = snap.goalCell(i);
                centerSrc[i] = idx(g.x, g.y);
            }
            int startSrc[1] = {idx(0, 0)};
            ffVisReset();
            std::memset(ffAvoid_, 0, sizeof(ffAvoid_));
            const uint8_t* avoidPtr = cfg_.ffAvoid ? ffAvoid_ : nullptr;
            // Competitively scored stages; each transition fires when the mouse
            // reaches Start or Center. Debug prints use the firmware state names.
            for (int legNo = 0; legNo < 2; ++legNo)
            {
                const bool toCenter = (legNo == 0); // State_SearchCenter / State_OptimizedReturn
                uint32_t hp = toCenter ? ffWalkLeg(atCenter, centerSrc, 4, nullptr, ffAvoid_)
                                       : ffWalkLeg(atStart, startSrc, 1, avoidPtr, nullptr);
                if (vDebug_)
                    fprintf(stderr, "MIX %s leg=%d hash=%.8x unknown=%d\n",
                            toCenter ? "State_SearchCenter" : "State_OptimizedReturn", legNo, hp,
                            known_.unknownCount());
                if (hp == 0)
                    return false; // leg gave up: report failure
            }
            // State_Sprint1: fast smooth-turn speed run Start -> Center on the
            // currently known optimal path. The map is LOCKED: open-loop drive on
            // a pessimistic plan (no optimistic heuristics, no exploration).
            sealed_ = true;
            if (!phase4())
            {
                if (vDebug_)
                    fprintf(stderr, "MIX FAIL State_Sprint1\n");
                return false;
            }
            if (vDebug_)
                fprintf(stderr, "MIX State_Sprint1 done pos=(%d,%d) unknown=%d\n", mx_, my_,
                        known_.unknownCount());
            // no clearAllColor here: the blue Sprint1 line stays as history while
            // the light-gray State_DeepExplore path + orange waypoints overlay it
            // State_DeepExplore: hunt for the missing walls that can improve the
            // sprint -- the exact improvement-band explorer (trip economics),
            // which senses every candidate shortcut edge. It leaves the mouse at
            // its last trip target and sets sealed_.
            // NOTE: unseal first -- Sprint1 ran sealed; a sealed phase 2 would
            // drive open-loop on the pessimistic map without sensing (inert
            // exploration).
            sealed_ = false;
            vizPathColor_ = 'a'; // exploration trail: light gray (phase-1 green stays visible)
            if (!phase2())
            {
                if (vDebug_)
                    fprintf(stderr, "MIX FAIL State_DeepExplore\n");
                return false;
            }
            // Return to start (pessimistic drive; the wall-hunting is done):
            // Sprint 2 must DEPART from the start cell to be scored as a run.
            if (!phase3())
            {
                if (vDebug_)
                    fprintf(stderr, "MIX FAIL State_DeepExplore-return\n");
                return false;
            }
            // State_Sprint2: final maximum-velocity run straight to the center.
            // Sealed (map locked, drawn == driven) whenever the pessimistic
            // route exists from here -- it does from the start; the probe stays
            // as a defensive fallback that would finish with an optimistic
            // replanned drive instead.
            if (isGoalHere())
                return true;
            {
                // AUDIT: known-open faces must agree with truth
                int wrong = 0;
                for (int y = 0; y < MAZE_H && wrong < 8; ++y)
                    for (int x = 0; x < MAZE_W && wrong < 8; ++x)
                        for (int di = 0; di < kDirCount; ++di)
                        {
                            Dir d = static_cast<Dir>(di);
                            if (known_.status(x, y, d) == kOpen &&
                                truth_.wall(x, y, d))
                            {
                                if (vDebug_)
                                    fprintf(stderr, "AUDIT wrong-open (%d,%d,d=%d)\n", x, y, di);
                                ++wrong;
                            }
                        }
                if (vDebug_)
                    fprintf(stderr, "AUDIT wrong-opens=%d\n", wrong);
                Maze pes = pessimisticSnapshot();
                int csrc[4];
                for (int i = 0; i < 4; ++i)
                {
                    Cell g = pes.goalCell(i);
                    csrc[i] = idx(g.x, g.y);
                }
                Cost tmp[kMAZE];
                dijkstraCells(pes, csrc, 4, tmp);
                sealed_ = tmp[idx(mx_, my_)] < INT32_MAX;
                if (vDebug_)
                    fprintf(stderr, "MIX State_Sprint2 sealed=%d pos=(%d,%d)\n", (int)sealed_, mx_, my_);
            }
            if (!phase4())
            {
                if (vDebug_)
                    fprintf(stderr, "MIX FAIL State_Sprint2\n");
                return false;
            }
            return true;
        }

        bool phase1()
        {
            st_.phase = 1;
            auto atGoal = [&](int x, int y) { return isGoalTruth(x, y); };
            if (runTo(atGoal, cfg_.phase1) == RunStatus::kStuck)
                return false;
            return true;
        }

        struct FrontierEdge
        {
            int ax, ay; // from-cell
            int bx, by; // across-the-edge neighbor (either endpoint cell senses
                        // the shared face, so the trip may target the nearer one)
            Dir d; // direction of unknown edge a->b
            Cost bound; // ds[a] + 1 + dc[b] (potential best path through it)
            int tx, ty; // cell to travel to for sensing this edge: a
        };

        // compute the improvement band frontier; false => certified (band empty)
        bool improvementFrontier(std::vector<FrontierEdge>& out, double C_P, Cost ds[kMAZE], Cost dc[kMAZE])
        {
            out.clear();
            Maze snap = known_.snapshot();
            int srcS[1] = {idx(0, 0)};
            int srcC[4];
            for (int i = 0; i < 4; ++i)
            {
                Cell g = snap.goalCell(i);
                srcC[i] = idx(g.x, g.y);
            }
            dijkstraCells(snap, srcS, 1, ds);
            dijkstraCells(snap, srcC, 4, dc);
            const Cost LIM = static_cast<Cost>(C_P);
            for (int y = 0; y < MAZE_H; ++y)
                for (int x = 0; x < MAZE_W; ++x)
                {
                    for (int di = 1; di < kDirCount; ++di)
                    { // E,S only: dedupe symmetric
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
                        if (b1 == INT32_MAX || b2 == INT32_MAX || b3 == INT32_MAX || b4 == INT32_MAX)
                            continue; // side disconnected even optimistically (irrelevant)
                        if (b1 + kCostUnit + b2 <= LIM || b3 + kCostUnit + b4 <= LIM)
                        {
                            FrontierEdge e;
                            e.ax = x;
                            e.ay = y;
                            e.bx = nx;
                            e.by = ny;
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

        bool phase2()
        {
            st_.phase = 2;
            vizPathColor_ = 'a'; // exploration path: light gray - phase-1 green stays visible
            std::memset(skipTarget_, 0, sizeof(skipTarget_));
            int guard = 0;
            iter_ = 0;
            while (guard++ < 400)
            {
                ++iter_;
                if (vDebug_)
                    fprintf(stderr, "P2-top iter=%d pos=(%d,%d,%d)\n", iter_, mx_, my_, (int)mdir_);
                // 1: best known path center<->start (A* from start pose? mouse may be
                // anywhere; evaluate the *run* start->center independent of pose)
                Maze snap = known_.snapshot();
                PathResult best = plan(
                    0, 0, D_N, [&](int x, int y) { return isGoalTruth(x, y); }, snap,
                    PhaseHeur{HeurKind::kTrueDistance, 1, 1});
                if (!best.found && known_.unknownCount() == 0)
                {
                    // maze dead-ends? keep exploring (trip toward frontier) or fail:
                    // if no unknown and no path, maze can't be solved: report failure
                    return false;
                }
                if (best.found)
                {
                    st_.certBestKnown = best.cost / static_cast<double>(kCostUnit);
                    // 2: improvement band
                    Cost ds[kMAZE], dc[kMAZE];
                    std::vector<FrontierEdge> frontier;
                    bool exist = improvementFrontier(frontier, best.cost, ds, dc);
                    if (vDebug_)
                    {
                        fprintf(stderr,
                                "P2 iter=%d pos=(%d,%d,%d) CP=%.2f frontier=%zu unknownEdges=%d "
                                "opt=%.2f\n",
                                iter_, mx_, my_, (int)mdir_, best.cost / (double)kCostUnit, frontier.size(),
                                known_.unknownCount(), st_.certOptimistic);
                    }
                    st_.certOptimistic = (dc[idx(0, 0)] < INT32_MAX ? dc[idx(0, 0)] / (double)kCostUnit : -1);
                    if (!exist)
                    {
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
                    // the same number): O(|frontier|) flo`ods become exactly one.
                    Cost dm[kMAZE];
                    int msrc[1] = {idx(mx_, my_)};
                    dijkstraCells(snap, msrc, 1, dm);

                    const double VBAND = cfg_.vband;
                    const double W = wTrip_; // <0 = exact mode; 0..1 = trip-economics ratio
                    const double P = edgePrior_; // gamble rate applied to potential saving

                    // Either endpoint cell of an unknown edge senses the shared face
                    // (sensing is a 3-face+back scan of the cell we stand in). Pick the
                    // reachable, not-previously-failed endpoint with the smaller driving
                    // distance: halves the average trip to each edge.
                    struct EP
                    {
                        bool ok = false;
                        int tx = 0, ty = 0;
                        Dir face = D_N; // heading that scans the edge from the endpoint
                        double trip = 0;
                    };
                    auto pickEP = [&](const FrontierEdge& e) {
                        EP r;
                        int dA = dm[idx(e.ax, e.ay)];
                        int dB = dm[idx(e.bx, e.by)];
                        bool aOk = dA != INT32_MAX && !skipTarget_[idx(e.ax, e.ay)];
                        bool bOk = dB != INT32_MAX && !skipTarget_[idx(e.bx, e.by)];
                        if (!aOk && !bOk)
                            return r; // both sealed off
                        bool useB = bOk && (!aOk || dB < dA);
                        r.ok = true;
                        r.tx = useB ? e.bx : e.ax;
                        r.ty = useB ? e.by : e.ay;
                        r.face = useB ? opposite(e.d) : e.d;
                        r.trip = (useB ? dB : dA) / (double)kCostUnit;
                        return r;
                    };
                    auto savingOf = [&](const FrontierEdge& e) {
                        return (best.cost - e.bound) / (double)kCostUnit;
                    };
                    auto certify = [&]() {
                        st_.certifiedAfterCells = static_cast<int>(st_.totalDistance);
                        st_.certBound = st_.certBestKnown;
                        certified_ = best;
                        vizFinal(best);
                        sealed_ = true; // (F): endgame drives open-loop, no sense/replan
                        return true;
                    };

                    const FrontierEdge* bestEdge = nullptr;
                    double bestVal = -1e18;
                    double bestValW = -1e18;
                    for (const auto& e : frontier)
                    {
                        EP c = pickEP(e);
                        if (!c.ok)
                            continue;
                        double val = savingOf(e) - c.trip;
                        if (val > bestVal)
                            bestVal = val;
                        if (W >= 0.0)
                        {
                            double valW = P * savingOf(e) - W * c.trip;
                            if (valW > bestValW)
                                bestValW = valW;
                        }
                    }
                    if (bestVal <= -1e17)
                    {
                        // nowhere reachable with unknowns in-band: cannot improve: certify
                        return certify();
                    }
                    // (D) trip economics: no frontier edge repays (W * trip) when its
                    // saving is discounted by the open-wall prior P => chasing marginal
                    // gambles is not worth it under the score formula: certify now.
                    // W=1, P=1 reproduces the legacy lazy behavior exactly; smaller W
                    // or P keeps more edges (W=0, P=1 is the exact certificate).
                    if (W >= 0.0 && bestValW <= 0.0)
                        return certify();
                    // (B) tie-break among the value plateau: nearest to the mouse first,
                    // then more promising bound, then more in-band faces sharing the same
                    // target cell (one trip services several edges).
                    int bestTrip = INT32_MAX;
                    Cost bestBound = INT32_MAX;
                    int bestShared = -1;
                    for (const auto& e : frontier)
                    {
                        EP c = pickEP(e);
                        if (!c.ok)
                            continue;
                        double val = savingOf(e) - c.trip;
                        if (val < bestVal - VBAND)
                            continue;
                        // how many unresolved in-band edges lie near this target?
                        // R=0 == legacy same-cell count; R>0 sweeps a neighborhood per
                        // trip (one journey resolves several edges).
                        int shared = 0;
                        for (const auto& o : frontier)
                        {
                            EP co = pickEP(o);
                            if (!co.ok)
                                continue;
                            int dx = co.tx - c.tx, dy = co.ty - c.ty;
                            if (std::abs(dx) + std::abs(dy) <= clusterRadius_)
                                ++shared;
                        }
                        bool better = false;
                        if (c.trip < bestTrip)
                            better = true;
                        else if (c.trip == bestTrip)
                        {
                            if (e.bound < bestBound)
                                better = true;
                            else if (e.bound == bestBound && shared > bestShared)
                                better = true;
                        }
                        if (better)
                        {
                            bestEdge = &e;
                            bestTrip = static_cast<int>(c.trip);
                            bestBound = e.bound;
                            bestShared = shared;
                        }
                    }
                    // 4: exploration trip to the frontier edge's nearer endpoint
                    EP use = pickEP(*bestEdge);
                    int tx = use.tx, ty = use.ty;
                    vizTripTarget(tx, ty);
                    if (mx_ == tx && my_ == ty)
                    {
                        faceForUnknown(use.face);
                    }
                    else
                    {
                        ++st_.trips;
                        PhaseHeur tripHeur;
                        tripHeur.epsNum = cfg_.phase2.epsNum;
                        tripHeur.epsDen = cfg_.phase2.epsDen;
                        // exact turn-free optimistic h toward the trip target (admissible,
                        // computed once on the open map; walls discovered en route don't
                        // invalidate its admissibility)
                        Cost tripH[kMAZE];
                        int tsrc[1] = {idx(tx, ty)};
                        dijkstraCells(Maze(), tsrc, 1, tripH);
                        dbgTx_ = tx;
                        dbgTy_ = ty;
                        auto atT = [&](int x, int y) { return x == tx && y == ty; };
                        if (runTo(atT, tripHeur, tripH) == RunStatus::kStuck)
                        {
                            // sealed en route: give up on this target this maze (its unknown
                            // faces can only be sensed if some route there appears later? no:
                            // mark skip; the cert may still close via other edges)
                            skipTarget_[idx(tx, ty)] = true;
                            continue;
                        }
                        // unknown edge may have been sensed en route; if not, orient & sense
                        if (known_.status(tx, ty, use.face) == kUnknown)
                            faceForUnknown(use.face);
                    }
                    continue;
                }
                // No known path (start disconnected on known map): explore toward the
                // nearest unknown to open connectivity (fallback: plain optimistic A*)
                PhaseHeur fallback;
                fallback.kind = HeurKind::kOptimistic;
                fallback.epsNum = cfg_.phase2.epsNum;
                fallback.epsDen = cfg_.phase2.epsDen;
                auto atUnknownNeighbor = [&](int x, int y)
                {
                    for (int di = 0; di < kDirCount; ++di)
                    {
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

        void faceDir(Dir d)
        {
            while (mdir_ != d)
            {
                int rel = (static_cast<int>(d) - static_cast<int>(mdir_) + kDirCount) % kDirCount;
                if (rel == 1 || rel == 2)
                {
                    w_.turnRight();
                    mdir_ = static_cast<Dir>((mdir_ + 1) % kDirCount);
                }
                else
                {
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
        void faceForUnknown(Dir d)
        {
            for (int k = 0; k < 4; ++k)
            {
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

        bool phase3()
        {
            st_.phase = 3;
            // Drop the cert-time highlight: the return trip drives the *pessimistic*
            // plan (unknowns = walls), not the optimistic vizFinal(best) drawing.
            // Phase 4 redraws the path it actually drives. Goals stay marked, and
            // the stage-history record is reset too, so the stale certified-plan
            // blue cannot be repainted by the final sprint's vizFinal.
            w_.clearAllColor();
            std::memset(cellPaint_, 0, sizeof(cellPaint_));
            vizGoals();
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

        bool phase4()
        {
            st_.phase = 4;
            // sprint: optimal A* on fully known map, exact h, batching enabled
            PhaseHeur ph;
            ph.kind = HeurKind::kTrueDistance;
            ph.epsNum = 1;
            ph.epsDen = 1;
            auto atGoal = [&](int x, int y) { return isGoalTruth(x, y); };
            // face the first direction of the sprint plan
            // Post-certification the solver is sealed: runTo() will plan on the
            // *pessimistic* snapshot (unknown faces = walls) and drive open-loop on
            // it. Plan the same way here so the drawn highlight IS the driven path
            // (A* is deterministic on identical inputs). Under allow-lazy the
            // regular snapshot may still route through unresolved unknowns, so
            // drawing that plan would diverge from the mouse again.
            if (sealed_)
            {
                Maze pes = pessimisticSnapshot();
                PathResult fin = plan(mx_, my_, mdir_, atGoal, pes, ph);
                if (!fin.found || fin.states.size() < 2)
                    return false;
                vizFinal(fin);
                if (vTrace_)
                {
                    fprintf(stderr, "SPRINT path (pessimistic):");
                    for (auto& s : fin.states)
                        fprintf(stderr, " (%d,%d)", s.x, s.y);
                    fprintf(stderr, "\n");
                }
                faceDir(fin.states[1].dir);
            }
            else
            {
                Maze snap = known_.snapshot();
                PathResult p = plan(mx_, my_, mdir_, atGoal, snap, ph);
                if (!p.found || p.states.size() < 2)
                    return false;
                vizFinal(p);
                if (vTrace_)
                {
                    fprintf(stderr, "SPRINT path:");
                    for (auto& s : p.states)
                        fprintf(stderr, " (%d,%d)", s.x, s.y);
                    fprintf(stderr, "\n");
                }
                faceDir(p.states[1].dir);
            }
            RunStatus rs = runTo(atGoal, ph);
            if (vDebug_)
                fprintf(stderr, "PH4 runTo=%d pos=(%d,%d) truthGoal=%d center=%d\n", (int)rs, mx_, my_,
                        (int)isGoalTruth(mx_, my_), (int)isGoalHere());
            if (rs != RunStatus::kDone)
                return false;
            st_.sprintDist = static_cast<int>(curRunDist_);
            st_.sprintTurns = curRunTurns_;
            return isGoalHere();
        }

        bool isGoalTruth(int x, int y) const { return truth_.isGoal(x, y); }
        int dbgTx_ = -1, dbgTy_ = -1;
        bool skipTarget_[MAZE_W * MAZE_H] = {}; // trip targets proven unreachable
        double wTrip_ = -1.0; // mirrors cfg_.tripWorth (-1 = exact mode)
        double edgePrior_ = 1.0; // mirrors cfg_.edgePrior
        int clusterRadius_ = 0; // mirrors cfg_.clusterRadius
        bool vDebug_ = false, vTrace_ = false, vTrace2_ = false; // cfg debug channels
        bool sealed_ = false; // (F) true after certification: endgame is open-loop

        IWorld& w_;
        const Maze& truth_;
        SolverConfig cfg_;
        Astar astar_;
        KnownMaze known_;
        int mx_ = 0, my_ = 0;
        Dir mdir_ = D_N;
        int arrivalFace_ = -1; // Dir face last PHYSICALLY crossed into this cell; -1 = never moved (no back inference)
        SolverStats st_;
        PathResult certified_;
        int iter_ = 0;
        bool runAtStart_ = false;
        double curRunDist_ = 0, curRunEff_ = 0;
        int curRunTurns_ = 0;
    };

} // namespace mm
