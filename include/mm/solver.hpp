#pragma once
// Solver: Micromouse state machine on top of A* and directional heuristics.
//
//   PHASE 1  EXPLORE_START_TO_CENTER : weighted A* (optimistic map) to center
//   PHASE 2  CERTIFIED_DEEP_SEARCH   : exploration trips toward the
//            "improvement band" (cells that could lie on a cheaper path),
//            until the best start<->center path is PROVABLY optimal
//   PHASE 3  RUN_BEST_TO_START       : run the certified best path to start
//   PHASE 4  SPRINT                  : start->center, optimal A*, long runs
//
// In --sweep mode:
//   Nearest-candidate-first exploration using a sound relaxation-tie certificate.
//   LIM = best path cost on the optimistic map (every unknown face open).
//   Executable route = best path on the pessimistic map (unknown faces = walls).
//   When pessimistic cost == LIM, the route is provably optimal.
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <queue>
#include <tuple>
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
        int epsNum = 2; // weighted A* default: h * 2 (explore fast)
        int epsDen = 1;
    };

    struct SolverConfig
    {
        PhaseHeur phase1, phase2;
        Cost turnCost = 70;              // Turn penalty in A* (70 = 0.275 cell-time)
        bool enableDiagonal = false;     // Slalom planning: alternating L/R turns discounted
        Cost zigzagCost = 64;            // Cost of one alternating slalom turn
        bool sweep = false;              // Nearest-candidate-first exploration mode
        bool visualize = false;

        // Flood-fill exploration mood
        bool floodFill = false;
        bool ffStraight = false;         // Prefer going straight on ties
        bool ffAvoid = false;            // Prefer unvisited cells on return
        int ffCurio = 64;                // Curiosity penalty on unexplored faces
        int ffLegs = 2;                  // Exploration legs (even, >= 2)

        // Real-hardware sensor rig options (inert in standard simulators)
        bool lookAhead = false;
        bool lookAheadDiag = false;

        bool mixMode = false;            // 5-stage mix mode

        // Debug channels
        bool dbgCrit = false;
        bool dbgTrace = false;
        bool dbgTrace2 = false;
    };

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
            vDebug_(cfg.dbgCrit), vTrace_(cfg.dbgTrace), vTrace2_(cfg.dbgTrace2), w_(w),
            truth_(truth), cfg_(cfg)
        {
        }

        bool solve()
        {
            st_ = SolverStats{};
            mx_ = 0;
            my_ = 0;
            mdir_ = D_N;
            arrivalFace_ = -1;
            known_ = KnownMaze{};
            vizPathColor_ = 'g';
            sealed_ = false;
            std::fill(std::begin(ffSeal_), std::end(ffSeal_), 0);
            sealQn_ = 0;
            knownVer_ = 0;
            ffLastSrc_ = nullptr;
            ffLastNSrc_ = -1;
            ffLastVer_ = -1;
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
            ac.enableDiagonal = cfg_.enableDiagonal;
            ac.zigzagCost = cfg_.zigzagCost;
            ac.epsilonNum = ph.epsNum;
            ac.epsilonDen = ph.epsDen;
            ++st_.aStarCalls;
            PathResult r = astar_.search(snap, sx, sy, sdir, ac, h, atGoal);
            st_.aStarExpansions += r.expansions;
            return r;
        }

        uint8_t chFaces_ = 0;
        void knowSet(int x, int y, Dir d, WallStatus s)
        {
            known_.setStatus(x, y, d, s);
            ++knownVer_;
        }

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
                    knowSet(mx_, my_, q.d, s);
                    changed = true;
                    chFaces_ |= static_cast<uint8_t>(1u << q.d);
                    int nx = mx_ + kDx[q.d], ny = my_ + kDy[q.d];
                    if (nx >= 0 && ny >= 0 && nx < MAZE_W && ny < MAZE_H)
                    {
                        sealEnqueue(idx(mx_, my_));
                        sealEnqueue(idx(nx, ny));
                    }
                }
            }

            Dir back = static_cast<Dir>((mdir_ + 2) % kDirCount);
            if (static_cast<int>(back) == arrivalFace_ && known_.status(mx_, my_, back) == kUnknown)
            {
                knowSet(mx_, my_, back, kOpen);
                changed = true;
                chFaces_ |= static_cast<uint8_t>(1u << back);
                sealEnqueue(idx(mx_, my_));
                int nx = mx_ + kDx[back], ny = my_ + kDy[back];
                if (nx >= 0 && ny >= 0 && nx < MAZE_W && ny < MAZE_H)
                    sealEnqueue(idx(nx, ny));
            }

            Dir fdir = mdir_;
            if (cfg_.lookAhead && known_.status(mx_, my_, fdir) == kOpen)
            {
                int fx = mx_ + kDx[fdir], fy = my_ + kDy[fdir];
                if (fx >= 0 && fy >= 0 && fx < MAZE_W && fy < MAZE_H && known_.status(fx, fy, fdir) == kUnknown)
                {
                    int far2 = w_.wallAhead2();
                    if (far2 >= 0)
                    {
                        WallStatus s2 = far2 == 1 ? kWall : kOpen;
                        knowSet(fx, fy, fdir, s2);
                        changed = true;
                        sealEnqueue(idx(fx, fy));
                        int gx = fx + kDx[fdir], gy = fy + kDy[fdir];
                        if (gx >= 0 && gy >= 0 && gx < MAZE_W && gy < MAZE_H)
                            sealEnqueue(idx(gx, gy));
                    }
                }
            }

            if (cfg_.lookAheadDiag && known_.status(mx_, my_, fdir) == kOpen)
            {
                int fx = mx_ + kDx[fdir], fy = my_ + kDy[fdir];
                if (fx >= 0 && fy >= 0 && fx < MAZE_W && fy < MAZE_H)
                {
                    const Dir sd[2] = {static_cast<Dir>((fdir + 3) % kDirCount),
                                       static_cast<Dir>((fdir + 1) % kDirCount)};
                    const int sv[2] = {w_.wallLeft2(), w_.wallRight2()};
                    for (int ch = 0; ch < 2; ++ch)
                    {
                        int v = sv[ch];
                        if (v < 0) continue;
                        WallStatus s = v == 1 ? kWall : kOpen;
                        if (known_.status(fx, fy, sd[ch]) == s) continue;
                        knowSet(fx, fy, sd[ch], s);
                        changed = true;
                        sealEnqueue(idx(fx, fy));
                        int gx = fx + kDx[sd[ch]], gy = fy + kDy[sd[ch]];
                        if (gx >= 0 && gy >= 0 && gx < MAZE_W && gy < MAZE_H)
                            sealEnqueue(idx(gx, gy));
                    }
                }
            }

            changed = sealCascade() > 0 || changed;
            return changed;
        }

        uint16_t ffSeal_[kMAZE] = {};
        bool isSealed(int c, Dir d) const { return (ffSeal_[c] >> d) & 1; }
        bool isSealed(int x, int y, Dir d) const { return isSealed(idx(x, y), d); }
        void sealSet(int x, int y, Dir d)
        {
            ffSeal_[idx(x, y)] |= static_cast<uint16_t>(1u << d);
        }
        bool wallLike(int x, int y, Dir d) const { return known_.status(x, y, d) == kWall || isSealed(x, y, d); }
        int sealQ_[1024];
        int sealQn_ = 0;
        void sealEnqueue(int c)
        {
            if (sealQn_ < 1024)
                sealQ_[sealQn_++] = c;
        }
        int sealCascade()
        {
            int masked = 0;
            const int px = mx_, py = my_;
            auto nearPose = [&](int x, int y)
            {
                if (x == px && y == py) return true;
                for (int dd = 0; dd < kDirCount; ++dd)
                    if (x == px + kDx[dd] && y == py + kDy[dd]) return true;
                return false;
            };
            for (int q = 0; q < sealQn_; ++q)
            {
                int c = sealQ_[q];
                int cx = c & 0xF, cy = c >> 4;
                if (isGoalTruth(cx, cy) || nearPose(cx, cy)) continue;
                Dir other = D_N;
                int blocked = 0;
                for (int d = 0; d < kDirCount; ++d)
                {
                    Dir dd = static_cast<Dir>(d);
                    if (wallLike(cx, cy, dd)) ++blocked;
                    else if (known_.status(cx, cy, dd) == kOpen) other = dd;
                }
                if (blocked != 3 || known_.status(cx, cy, other) != kOpen) continue;
                int nx = cx + kDx[other], ny = cy + kDy[other];
                if (nx < 0 || ny < 0 || nx >= MAZE_W || ny >= MAZE_H) continue;
                if (isGoalTruth(nx, ny) || nearPose(nx, ny)) continue;
                sealSet(cx, cy, other);
                sealSet(nx, ny, static_cast<Dir>((other + 2) % kDirCount));
                ++masked;
                sealEnqueue(idx(nx, ny));
            }
            sealQn_ = 0;
            return masked;
        }

        int maxRun(const std::vector<State>& path, size_t pi) const
        {
            size_t j = pi;
            while (j + 1 < path.size())
            {
                if (path[j].dir != path[j + 1].dir) break;
                WallStatus st = known_.status(path[j].x, path[j].y, path[j].dir);
                if (st != kOpen) break;
                ++j;
            }
            return static_cast<int>(j - pi);
        }

        enum struct RunStatus
        {
            kDone,
            kStuck
        };

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
                return RunStatus::kStuck;
            if (atGoal(mx_, my_))
                return RunStatus::kDone;
            while (pi + 1 < plan.size())
            {
                const State& a = plan[pi];
                const State& b = plan[pi + 1];
                if (a.dir == b.dir)
                {
                    int run = 1;
                    while (pi + run + 1 < plan.size() && plan[pi + run].dir == plan[pi + run + 1].dir)
                        ++run;
                    if (!w_.moveForward(run))
                        return RunStatus::kStuck;
                    advanceStats(run);
                    mx_ += static_cast<int>(kDx[mdir_]) * run;
                    my_ += static_cast<int>(kDy[mdir_]) * run;
                    arrivalFace_ = (mdir_ + 2) % kDirCount;
                    pi += run;
                    onCellVisit();
                }
                else
                {
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

        template <class GoalFn>
        RunStatus runTo(GoalFn atGoal, const PhaseHeur& ph, const Cost* hOverride = nullptr)
        {
            if (sealed_)
            {
                Maze pes = pessimisticSnapshot();
                PathResult fin = plan(mx_, my_, mdir_, atGoal, pes, ph, hOverride);
                if (!fin.found)
                    return RunStatus::kStuck;
                return driveOpen(fin.states, atGoal);
            }
            const int kMaxSteps = 100000;
            int steps = 0;
            int noops = 0;
            PathResult cur = plan(mx_, my_, mdir_, atGoal, known_.snapshot(), ph, hOverride);
            if (!cur.found)
                return RunStatus::kStuck;
            while (steps++ < kMaxSteps)
            {
                bool changed = sense();
                vizVisit();
                if (atGoal(mx_, my_))
                {
                    if (!changed)
                        return RunStatus::kDone;
                    ++st_.replans;
                    cur = plan(mx_, my_, mdir_, atGoal, known_.snapshot(), ph, hOverride);
                    if (!cur.found || cur.states.size() <= 1)
                        return RunStatus::kDone;
                    continue;
                }
                if (changed)
                {
                    size_t qi = SIZE_MAX;
                    for (size_t j = 0; j < cur.states.size(); ++j)
                        if (cur.states[j].x == mx_ && cur.states[j].y == my_ && cur.states[j].dir == mdir_)
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
                            if (!(chFaces_ & (1u << di))) continue;
                            int ex = mx_ + kDx[di], ey = my_ + kDy[di];
                            for (size_t k = qi; k + 1 < cur.states.size(); ++k)
                            {
                                const State& u = cur.states[k];
                                const State& v = cur.states[k + 1];
                                if (u.x == v.x && u.y == v.y) continue;
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
                        Maze snap = known_.snapshot();
                        cur = plan(mx_, my_, mdir_, atGoal, snap, ph, hOverride);
                        if (!cur.found)
                        {
                            if (vDebug_)
                                fprintf(stderr, "STUCK at (%d,%d,%d)\n", mx_, my_, (int)mdir_);
                            return RunStatus::kStuck;
                        }
                    }
                }

                size_t pi = SIZE_MAX;
                for (size_t j = cur.states.size(); j-- > 0;)
                    if (cur.states[j].x == mx_ && cur.states[j].y == my_ && cur.states[j].dir == mdir_)
                    {
                        pi = j;
                        break;
                    }
                if (pi == SIZE_MAX)
                {
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
                    if (!w_.moveForward(run))
                        return RunStatus::kStuck;
                    advanceStats(run);
                    mx_ += kDx[mdir_] * run;
                    my_ += kDy[mdir_] * run;
                    arrivalFace_ = (mdir_ + 2) % kDirCount;
                    acted = true;
                }
                else
                {
                    if (pi + 1 >= cur.states.size())
                        return RunStatus::kDone;
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

        bool cellVis_[MAZE_W * MAZE_H] = {};
        char cellPaint_[MAZE_W * MAZE_H] = {};
        char vizPathColor_ = 'g';

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
            if (!fresh && cellPaint_[c] == vizPathColor_) return;
            cellVis_[c] = true;
            cellPaint_[c] = vizPathColor_;
            w_.setColor(mx_, my_, vizPathColor_);
            if (!fresh) return;
            Maze snap = known_.snapshot();
            Cost dc[kMAZE];
            vizDistanceTable(snap, dc);
            if (dc[c] < INT32_MAX)
            {
                char b[16];
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
            for (int y = 0; y < MAZE_H; ++y)
                for (int x = 0; x < MAZE_W; ++x)
                {
                    int c = idx(x, y);
                    if (cellPaint_[c]) w_.setColor(x, y, cellPaint_[c]);
                }
            Cost dc[kMAZE];
            vizDistanceTable(snap, dc);
            for (int y = 0; y < MAZE_H; ++y)
                for (int x = 0; x < MAZE_W; ++x)
                {
                    int c = idx(x, y);
                    if (!cellVis_[c] || dc[c] == INT32_MAX) continue;
                    char b[16];
                    std::snprintf(b, sizeof b, "%d", dc[c] / kCostUnit);
                    w_.setText(x, y, b);
                }
            for (auto s : best.states)
            {
                w_.setColor(s.x, s.y, 'b');
                cellPaint_[idx(s.x, s.y)] = 'b';
            }
        }

        uint8_t ffVis_[kMAZE / 8 + 1] = {};
        uint8_t ffAvoid_[kMAZE / 8 + 1] = {};
        static constexpr int kFfNodeCap = 13440;
        static constexpr int kFfStates = kMAZE * kDirCount * kPrevT;
        static constexpr int kFfRingCap = 4096;
        int16_t ffDialSt_[kFfNodeCap];
        int16_t ffDialNx_[kFfNodeCap];
        int16_t ffRingHead_[kFfRingCap];
        int ffDialN_ = 0;
        int ffRem_ = 0;
        Cost ffDialLo_ = 0;
        Cost ffKin_[kMAZE][kDirCount][kPrevT];
        uint32_t ffStamp_[kFfStates] = {};
        bool ffStampLive_ = false;
        uint16_t ffDirty_[kFfStates + 8];
        int ffDirtyN_ = 0;
        bool ffDirtyOvf_ = false;
        uint32_t ffGen_ = 1;
        uint8_t ffClosed_[kFfStates / 8 + 1] = {};
        int knownVer_ = 0;

        struct FFHE { Cost k; int s; };
        FFHE ffHeap_[kFfNodeCap];
        int ffHn_ = 0;
        void ffHeapPush(Cost k, int s)
        {
            ffHeap_[ffHn_] = {k, s};
            std::push_heap(ffHeap_, ffHeap_ + ++ffHn_, [](const FFHE& a, const FFHE& b) { return a.k > b.k; });
        }
        FFHE ffHeapPop()
        {
            std::pop_heap(ffHeap_, ffHeap_ + ffHn_, [](const FFHE& a, const FFHE& b) { return a.k > b.k; });
            return ffHeap_[--ffHn_];
        }
        void ffRingPush(Cost k, int s)
        {
            if (ffDialN_ >= kFfNodeCap) return;
            int node = ffDialN_++;
            ffDialSt_[node] = static_cast<int16_t>(s);
            int b = static_cast<int>(k % kFfRingCap);
            ffDialNx_[node] = ffRingHead_[b];
            ffRingHead_[b] = static_cast<int16_t>(node);
            ++ffRem_;
        }
        static constexpr Cost kFfStraight = 64;
        static constexpr Cost kFfTurn = 64;
        static constexpr Cost kFfUturn = 128;
        static constexpr Cost kFfInf = INT32_MAX;

        const int* ffLastSrc_ = nullptr;
        int ffLastNSrc_ = -1;
        int ffLastVer_ = -1;

        Cost ffZigKin() const
        {
            return cfg_.turnCost > 0
                ? static_cast<Cost>((static_cast<int64_t>(kFfTurn) * cfg_.zigzagCost) / cfg_.turnCost)
                : kFfTurn;
        }

        void ffKinPut(int cell, int d, int cls, Cost v)
        {
            int s = (cell * kDirCount + d) * kPrevT + cls;
            if (ffStamp_[s] != ffGen_)
            {
                if (ffDirtyN_ < static_cast<int>(sizeof(ffDirty_) / sizeof(ffDirty_[0])))
                    ffDirty_[ffDirtyN_++] = static_cast<uint16_t>(s);
                else
                    ffDirtyOvf_ = true;
                ffStamp_[s] = ffGen_;
            }
            ffKin_[cell][d][cls] = v;
        }

        void ffKinFlood(const int* src, int nSrc)
        {
            if (src == ffLastSrc_ && nSrc == ffLastNSrc_ && ffLastVer_ == knownVer_) return;
            if (!ffStampLive_)
            {
                ffStampLive_ = true;
                std::fill(ffRingHead_, ffRingHead_ + kFfRingCap, -1);
                for (int i = 0; i < kMAZE; ++i)
                    for (int d = 0; d < kDirCount; ++d)
                        for (int cls = 0; cls < kPrevT; ++cls)
                            ffKin_[i][d][cls] = kFfInf;
            }
            else
            {
                for (int i = 0; i < ffDirtyN_; ++i)
                {
                    int s = ffDirty_[i];
                    int t = s / kPrevT;
                    ffKin_[t >> 2][t & 3][s % kPrevT] = kFfInf;
                    ffClosed_[s >> 3] &= static_cast<uint8_t>(~(1u << (s & 7)));
                }
            }
            ffDirtyN_ = 0;
            ffDirtyOvf_ = false;
            ++ffGen_;
            const bool dial = static_cast<int>(kFfStraight + kFfUturn + (cfg_.ffCurio > 0 ? cfg_.ffCurio : 0)) + 2 <= kFfRingCap;
            ffDialN_ = 0; ffRem_ = 0; ffDialLo_ = 0; ffHn_ = 0;

            for (int i = 0; i < nSrc; ++i)
                for (int d = 0; d < kDirCount; ++d)
                    for (int cls = 0; cls < kPrevT; ++cls)
                        if (ffKin_[src[i]][d][cls] == kFfInf)
                        {
                            ffKinPut(src[i], d, cls, 0);
                            const int ss = (src[i] * kDirCount + d) * kPrevT + cls;
                            if (dial) ffRingPush(0, ss);
                            else ffHeapPush(0, ss);
                        }
            const Cost kinZig = ffZigKin();
            for (;;)
            {
                int c, d, cls;
                Cost ek;
                if (dial)
                {
                    if (ffRem_ == 0) break;
                    int b = static_cast<int>(ffDialLo_ % kFfRingCap);
                    int guard = kFfRingCap;
                    while (ffRingHead_[b] < 0)
                    {
                        ++ffDialLo_;
                        b = static_cast<int>(ffDialLo_ % kFfRingCap);
                        if (--guard == 0) return;
                    }
                    int node = ffRingHead_[b];
                    int s = ffDialSt_[node];
                    ffRingHead_[b] = ffDialNx_[node];
                    --ffRem_;
                    if ((ffClosed_[s >> 3] >> (s & 7)) & 1) continue;
                    ffClosed_[s >> 3] |= static_cast<uint8_t>(1u << (s & 7));
                    int t = s / kPrevT;
                    c = t / kDirCount, d = t % kDirCount;
                    cls = s % kPrevT;
                    ek = ffKin_[c][d][cls];
                }
                else
                {
                    if (ffHn_ == 0) break;
                    FFHE e = ffHeapPop();
                    int th = e.s / kPrevT;
                    c = th / kDirCount, d = th % kDirCount;
                    cls = e.s % kPrevT;
                    if (ffKin_[c][d][cls] < e.k) continue;
                    ek = e.k;
                }
                Dir back = static_cast<Dir>((d + 2) % kDirCount);
                int cx = c & 0xF, cy = c >> 4;
                WallStatus st = known_.status(cx, cy, back);
                if (st == kWall || isSealed(cx, cy, back)) continue;
                Cost curios = st == kUnknown ? static_cast<Cost>(cfg_.ffCurio) : 0;
                if (cx + kDx[back] < 0 || cx + kDx[back] >= MAZE_W || cy + kDy[back] < 0 || cy + kDy[back] >= MAZE_H) continue;
                int p = idx(cx + kDx[back], cy + kDy[back]);
                if (isGoalTruth(cx, cy) && isGoalTruth(cx + kDx[back], cy + kDy[back])) continue;

                for (int dh = 0; dh < kDirCount; ++dh)
                {
                    const int rel = (d - dh + kDirCount) % kDirCount;
                    for (int clsP = 0; clsP < kPrevT; ++clsP)
                    {
                        if (rel == 2 && cls != kPrevStraight) continue;
                        if (rel == 0 && clsP != cls) continue;
                        if (rel == 1 && cls != kPrevRight) continue;
                        if (rel == 3 && cls != kPrevLeft) continue;

                        const bool zig = cfg_.enableDiagonal && rel != 2 &&
                            ((rel == 1 && clsP == kPrevLeft) || (rel == 3 && clsP == kPrevRight));
                        Cost turn = rel == 0 ? 0 : rel == 2 ? kFfUturn : (zig ? kinZig : kFfTurn);
                        Cost nk = ek + kFfStraight + turn + curios;
                        if (nk < ffKin_[p][dh][clsP])
                        {
                            ffKinPut(p, dh, clsP, nk);
                            if (dial) ffRingPush(nk, (p * kDirCount + dh) * kPrevT + clsP);
                            else ffHeapPush(nk, (p * kDirCount + dh) * kPrevT + clsP);
                        }
                    }
                }
            }
            ffLastSrc_ = src;
            ffLastNSrc_ = nSrc;
            ffLastVer_ = knownVer_;
        }
        void ffVisReset() { std::memset(ffVis_, 0, sizeof(ffVis_)); }
        bool ffVis(int c) const { return (ffVis_[c >> 3] >> (c & 7)) & 1; }
        void ffVis(int c, bool v)
        {
            if (v) ffVis_[c >> 3] |= (1u << (c & 7));
            else ffVis_[c >> 3] &= ~(1u << (c & 7));
        }

        template <class GoalFn>
        uint32_t ffWalkLeg(GoalFn atGoal, const int* floodSrc, int nSrc, const uint8_t* avoid, uint8_t* record, int* clsInOut)
        {
            auto gbb = [](const uint8_t* m, int c) { return (m[c >> 3] >> (c & 7)) & 1; };
            auto sbb = [](uint8_t* m, int c) { m[c >> 3] |= static_cast<uint8_t>(1u << (c & 7)); };
            const bool useAv = avoid && cfg_.ffAvoid;
            Cost(&kin)[kMAZE][kDirCount][kPrevT] = ffKin_;
            int walkCls = clsInOut && *clsInOut >= 0 ? *clsInOut : kPrevStraight;
            const Cost kinZig = ffZigKin();
            bool kinValid_ = false;
            int lastFloodVer_ = -1;
            uint32_t h = 2166136261u;
            int flops = 0;
            int moves = 0;
            for (int round = 0; round < 4096; ++round)
            {
                if (atGoal(mx_, my_))
                {
                    if (clsInOut) *clsInOut = walkCls;
                    return h;
                }
                int c = idx(mx_, my_);
                const bool knownChanged = sense();
                ffVis(c, true);
                if (!kinValid_ || knownChanged || knownVer_ != lastFloodVer_)
                {
                    ffKinFlood(floodSrc, nSrc);
                    lastFloodVer_ = knownVer_;
                    kinValid_ = true;
                }
                const int mh = static_cast<int>(mdir_);
                if (kin[c][mh][walkCls] == kFfInf)
                {
                    if (clsInOut) *clsInOut = walkCls;
                    return 0;
                }

                Dir plan[kMAZE];
                int pn = 0;
                bool stuck = false;
                int px = mx_, py = my_, pd = mh;
                int pcl = walkCls;
                while (!atGoal(px, py) && pn < kMAZE)
                {
                    int pc = idx(px, py);
                    const Cost kinCur = kin[pc][pd][pcl];
                    const bool onGoal = isGoalTruth(px, py);
                    Cost bestV = kFfInf;
                    int bestRw = 3, bestAv = 1;
                    Dir pick = D_N;
                    bool any = false;
                    for (int d = 0; d < kDirCount; ++d)
                    {
                        Dir dir = static_cast<Dir>(d);
                        if (known_.status(px, py, dir) == kWall) continue;
                        if (px + kDx[d] < 0 || px + kDx[d] >= MAZE_W || py + kDy[d] < 0 || py + kDy[d] >= MAZE_H) continue;
                        int nc = idx(px + kDx[d], py + kDy[d]);
                        const int relStep = (d - pd + kDirCount) % kDirCount;
                        const int claD = relStep == 0 ? pcl : relStep == 1 ? kPrevRight : relStep == 2 ? kPrevStraight : kPrevLeft;
                        if (kin[nc][d][claD] == kFfInf) continue;
                        if (onGoal && isGoalTruth(px + kDx[d], py + kDy[d])) continue;
                        if (kin[nc][d][claD] > kinCur || (kin[nc][d][claD] == kinCur && !onGoal)) continue;

                        const bool zigStep = cfg_.enableDiagonal && relStep != 2 &&
                            ((relStep == 1 && pcl == kPrevLeft) || (relStep == 3 && pcl == kPrevRight));
                        Cost turnv = relStep == 0 ? 0 : relStep == 2 ? kFfUturn : (zigStep ? kinZig : kFfTurn);
                        Cost v = turnv + kFfStraight + kin[nc][d][claD];
                        int rw = cfg_.ffStraight ? (d == pd ? 0 : d == (pd + 2) % kDirCount ? 2 : 1) : 0;
                        int avp = useAv ? gbb(avoid, nc) : 0;
                        bool take;
                        if (!any) take = true;
                        else if (v != bestV) take = v < bestV;
                        else if (avp != bestAv) take = avp < bestAv;
                        else if (rw != bestRw) take = rw < bestRw;
                        else take = false;
                        if (take)
                        {
                            bestV = v; bestRw = rw; bestAv = avp; pick = dir; any = true;
                        }
                    }
                    if (!any)
                    {
                        if (pn == 0) stuck = true;
                        break;
                    }
                    plan[pn++] = pick;
                    px += static_cast<int>(kDx[(int)pick]);
                    py += static_cast<int>(kDy[(int)pick]);
                    int relC = (static_cast<int>(pick) - pd + kDirCount) % kDirCount;
                    pcl = relC == 0 ? pcl : relC == 1 ? kPrevRight : relC == 2 ? kPrevStraight : kPrevLeft;
                    pd = static_cast<int>(pick);
                }
                if (pn == 0)
                {
                    if (!stuck || ++flops > 1) return 0;
                    continue;
                }
                flops = 0;
                for (int i = 0; i < pn; ++i)
                {
                    Dir pick = plan[i];
                    if (known_.status(mx_, my_, pick) == kWall) break;
                    const int relExec = (static_cast<int>(pick) - static_cast<int>(mdir_) + kDirCount) % kDirCount;
                    faceDir(pick);
                    walkCls = relExec == 0 ? walkCls : relExec == 1 ? kPrevRight : relExec == 2 ? kPrevStraight : kPrevLeft;
                    if (known_.status(mx_, my_, pick) == kUnknown) sense();
                    if (known_.status(mx_, my_, pick) == kWall) break;
                    if (++moves > 1200) { if (clsInOut) *clsInOut = walkCls; return 0; }
                    if (!w_.moveForward(1)) { if (clsInOut) *clsInOut = walkCls; return 0; }
                    if (known_.status(mx_, my_, pick) != kOpen)
                    {
                        known_.setStatus(mx_, my_, pick, kOpen);
                        ++knownVer_;
                    }
                    advanceStats(1);
                    mx_ += static_cast<int>(kDx[pick]);
                    my_ += static_cast<int>(kDy[pick]);
                    arrivalFace_ = (static_cast<int>(pick) + 2) % kDirCount;
                    if (record) sbb(record, idx(mx_, my_));
                    h = h * 16777619u ^ static_cast<uint32_t>(idx(mx_, my_) + 1);
                    if (!h) h = 0x5EEDu;
                    onCellVisit();
                    vizVisit();
                    sense();
                    ffVis(idx(mx_, my_), true);
                }
            }
            if (clsInOut) *clsInOut = walkCls;
            return 0;
        }

        bool solveFloodFill()
        {
            st_.phase = 1;
            auto atCenter = [&](int x, int y) { return isGoalTruth(x, y); };
            auto atStart = [&](int x, int y) { return x == 0 && y == 0; };
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
            const int legs = cfg_.ffLegs < 2 ? 2 : (cfg_.ffLegs & ~1);
            int legCls = kPrevStraight;
            for (int legNo = 0; legNo < legs; ++legNo)
            {
                bool toCenter = (legNo % 2 == 0);
                uint32_t hp = toCenter ? ffWalkLeg(atCenter, centerSrc, 4, nullptr, ffAvoid_, &legCls)
                                       : ffWalkLeg(atStart, startSrc, 1, avoidPtr, nullptr, &legCls);
                if (hp == 0) return false;
            }
            Maze end = known_.snapshot();
            PathResult best = plan(0, 0, D_N, atCenter, end, PhaseHeur{HeurKind::kTrueDistance, 1, 1});
            st_.certBestKnown = best.found ? best.cost / static_cast<double>(kCostUnit) : -1;
            st_.certBound = st_.certBestKnown;
            st_.certifiedAfterCells = static_cast<int>(st_.totalDistance);
            if (mx_ != 0 || my_ != 0)
                if (runTo(atStart, cfg_.phase1) == RunStatus::kStuck) return false;
            sealed_ = true;
            return phase4();
        }

        bool solveMix()
        {
            st_.phase = 1;
            auto atCenter = [&](int x, int y) { return isGoalTruth(x, y); };
            auto atStart = [&](int x, int y) { return x == 0 && y == 0; };
            Maze snap = known_.snapshot();
            int centerSrc[4];
            for (int i = 0; i < 4; ++i)
                centerSrc[i] = idx(snap.goalCell(i).x, snap.goalCell(i).y);
            int startSrc[1] = {idx(0, 0)};
            ffVisReset();
            int legCls = kPrevStraight;
            std::memset(ffAvoid_, 0, sizeof(ffAvoid_));
            const uint8_t* avoidPtr = cfg_.ffAvoid ? ffAvoid_ : nullptr;
            for (int legNo = 0; legNo < 2; ++legNo)
            {
                bool toCenter = (legNo == 0);
                uint32_t hp = toCenter ? ffWalkLeg(atCenter, centerSrc, 4, nullptr, ffAvoid_, &legCls)
                                       : ffWalkLeg(atStart, startSrc, 1, avoidPtr, nullptr, &legCls);
                if (hp == 0) return false;
            }
            sealed_ = true;
            if (!phase4()) return false;
            sealed_ = false;
            vizPathColor_ = 'a';
            if (!phase2()) return false;
            if (!phase3()) return false;
            if (isGoalHere()) return true;
            Maze pes = pessimisticSnapshot();
            int csrc[4];
            for (int i = 0; i < 4; ++i) csrc[i] = idx(pes.goalCell(i).x, pes.goalCell(i).y);
            Cost tmp[kMAZE];
            dijkstraCells(pes, csrc, 4, tmp);
            sealed_ = tmp[idx(mx_, my_)] < INT32_MAX;
            if (!phase4()) return false;
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
            int ax, ay, bx, by;
            Dir d;
            Cost bound;
            int tx, ty;
        };

        bool improvementFrontier(std::vector<FrontierEdge>& out, double C_P, Cost ds[kMAZE], Cost dc[kMAZE])
        {
            out.clear();
            Maze snap = known_.snapshot();
            int srcS[1] = {idx(0, 0)};
            int srcC[4];
            for (int i = 0; i < 4; ++i) srcC[i] = idx(snap.goalCell(i).x, snap.goalCell(i).y);
            dijkstraCells(snap, srcS, 1, ds);
            dijkstraCells(snap, srcC, 4, dc);
            const Cost LIM = static_cast<Cost>(C_P);
            for (int y = 0; y < MAZE_H; ++y)
                for (int x = 0; x < MAZE_W; ++x)
                    for (int di = 1; di < kDirCount; ++di)
                    {
                        Dir d = static_cast<Dir>(di);
                        if (known_.status(x, y, d) != kUnknown) continue;
                        int nx = x + kDx[d], ny = y + kDy[d];
                        if (nx < 0 || ny < 0 || nx >= MAZE_W || ny >= MAZE_H) continue;
                        Cost b1 = ds[idx(x, y)], b2 = dc[idx(nx, ny)];
                        Cost b3 = ds[idx(nx, ny)], b4 = dc[idx(x, y)];
                        if (b1 == INT32_MAX || b2 == INT32_MAX || b3 == INT32_MAX || b4 == INT32_MAX) continue;
                        if (b1 + kCostUnit + b2 <= LIM || b3 + kCostUnit + b4 <= LIM)
                        {
                            FrontierEdge e;
                            e.ax = x; e.ay = y; e.bx = nx; e.by = ny; e.d = d;
                            e.bound = std::min(b1 + kCostUnit + b2, b3 + kCostUnit + b4);
                            e.tx = x; e.ty = y;
                            out.push_back(e);
                        }
                    }
            return !out.empty();
        }

        // Clean nearest-candidate sweep with sound relaxation-tie certification
        bool phase2Sweep()
        {
            st_.phase = 2;
            vizPathColor_ = 'a';
            std::memset(skipTarget_, 0, sizeof(skipTarget_));
            int guard = 0;
            iter_ = 0;
            auto atGoalC = [&](int x, int y) { return isGoalTruth(x, y); };
            auto atStart = [&](int x, int y) { return x == 0 && y == 0; };

            Cost dF[kDirCount][kMAZE];
            int tF[kDirCount][kMAZE];
            Cost dPose[kMAZE];
            int tPose[kMAZE];
            auto poseFlood = [&](const Maze& pmap)
            {
                for (int di = 0; di < kDirCount; ++di)
                    for (int i = 0; i < kMAZE; ++i) {
                        dF[di][i] = INT32_MAX;
                        tF[di][i] = INT32_MAX;
                    }
                using QI = std::tuple<Cost, int, int, int, int>;
                std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
                int s = idx(mx_, my_);
                dF[static_cast<int>(mdir_)][s] = 0;
                tF[static_cast<int>(mdir_)][s] = 0;
                pq.emplace(0, 0, mx_, my_, static_cast<int>(mdir_));
                while (!pq.empty())
                {
                    QI top = pq.top();
                    pq.pop();
                    int x = std::get<2>(top), y = std::get<3>(top), di = std::get<4>(top);
                    int c = idx(x, y);
                    if (std::get<0>(top) > dF[di][c] || (std::get<0>(top) == dF[di][c] && std::get<1>(top) > tF[di][c]))
                        continue;
                    for (int ndi = 0; ndi < kDirCount; ++ndi)
                    {
                        Dir nd = static_cast<Dir>(ndi);
                        int nx = x + kDx[nd], ny = y + kDy[nd];
                        if (nx < 0 || ny < 0 || nx >= MAZE_W || ny >= MAZE_H || pmap.wall(x, y, nd))
                            continue;
                        int rel = (ndi - di + kDirCount) % kDirCount;
                        int tStep = (rel == 0) ? 0 : (rel == 2) ? 2 : 1;
                        int nc = idx(nx, ny);
                        Cost ndv = dF[di][c] + kCostUnit;
                        int ntv = tF[di][c] + tStep;
                        if (ndv < dF[ndi][nc] || (ndv == dF[ndi][nc] && ntv < tF[ndi][nc]))
                        {
                            dF[ndi][nc] = ndv;
                            tF[ndi][nc] = ntv;
                            pq.emplace(ndv, ntv, nx, ny, ndi);
                        }
                    }
                }
                for (int i = 0; i < kMAZE; ++i) {
                    dPose[i] = INT32_MAX;
                    tPose[i] = INT32_MAX;
                }
                for (int di = 0; di < kDirCount; ++di)
                    for (int i = 0; i < kMAZE; ++i)
                        if (dF[di][i] != INT32_MAX && (dF[di][i] < dPose[i] || (dF[di][i] == dPose[i] && tF[di][i] < tPose[i]))) {
                            dPose[i] = dF[di][i];
                            tPose[i] = tF[di][i];
                        }
            };

            struct Cand {
                int ax = 0, ay = 0, bx = 0, by = 0;
                Dir d = D_N;
                Cost bound = 0;
                int tx = -1, ty = -1;
                Dir face = D_N;
                int trip = 0, turns = 0;
            };

            auto finalizeCert = [&](const PathResult& route) {
                st_.certifiedAfterCells = static_cast<int>(st_.totalDistance);
                st_.certBestKnown = route.cost / static_cast<double>(kCostUnit);
                st_.certBound = st_.certBestKnown;
                certified_ = route;
                vizFinal(route);
                sealed_ = true;
            };

            while (guard++ < 2000)
            {
                ++iter_;
                Maze snap = known_.snapshot();
                Maze pes = pessimisticSnapshot();
                PathResult bestO = plan(0, 0, D_N, atGoalC, snap, PhaseHeur{HeurKind::kTrueDistance, 1, 1});
                PathResult bestP = plan(0, 0, D_N, atGoalC, pes, PhaseHeur{HeurKind::kTrueDistance, 1, 1});

                if (!bestO.found) {
                    if (known_.unknownCount() == 0) return false;
                    PhaseHeur fallback{HeurKind::kOptimistic, cfg_.phase2.epsNum, cfg_.phase2.epsDen};
                    auto atUnknown = [&](int x, int y) {
                        for (int di = 0; di < kDirCount; ++di)
                            if (known_.status(x, y, static_cast<Dir>(di)) == kUnknown) return true;
                        return false;
                    };
                    if (runTo(atUnknown, fallback) == RunStatus::kStuck) return false;
                    continue;
                }

                const Cost LIM = bestO.cost;
                st_.certOptimistic = LIM / (double)kCostUnit;
                if (bestP.found)
                    st_.certBestKnown = bestP.cost / (double)kCostUnit;

                // Provable certification: executable pessimistic route equals optimistic lower bound
                if (bestP.found && bestP.cost == LIM) {
                    if (mx_ == 0 && my_ == 0) {
                        finalizeCert(bestP);
                        return true;
                    }
                    PhaseHeur ph{HeurKind::kTrueDistance, 1, 1};
                    runTo(atStart, ph);
                    if (mx_ == 0 && my_ == 0) {
                        finalizeCert(bestP);
                        return true;
                    }
                }

                Cost ds[kMAZE], dc[kMAZE];
                int srcS[1] = {idx(0, 0)};
                int srcC[4];
                for (int i = 0; i < 4; ++i) srcC[i] = idx(snap.goalCell(i).x, snap.goalCell(i).y);
                dijkstraCells(snap, srcS, 1, ds);
                dijkstraCells(snap, srcC, 4, dc);

                poseFlood(pes);
                std::vector<Cand> cands;

                for (int y = 0; y < MAZE_H; ++y)
                    for (int x = 0; x < MAZE_W; ++x)
                        for (int di = 1; di < kDirCount; ++di) {
                            Dir d = static_cast<Dir>(di);
                            if (known_.status(x, y, d) != kUnknown) continue;
                            int nx = x + kDx[d], ny = y + kDy[d];
                            if (nx < 0 || ny < 0 || nx >= MAZE_W || ny >= MAZE_H) continue;

                            Cost b1 = ds[idx(x, y)], b2 = dc[idx(nx, ny)];
                            Cost b3 = ds[idx(nx, ny)], b4 = dc[idx(x, y)];
                            Cost b12 = (b1 != INT32_MAX && b2 != INT32_MAX) ? b1 + kCostUnit + b2 : INT32_MAX;
                            Cost b34 = (b3 != INT32_MAX && b4 != INT32_MAX) ? b3 + kCostUnit + b4 : INT32_MAX;
                            Cost bound = std::min(b12, b34);
                            if (bound > LIM) continue;

                            bool aOk = dPose[idx(x, y)] != INT32_MAX && !skipTarget_[idx(x, y)];
                            bool bOk = dPose[idx(nx, ny)] != INT32_MAX && !skipTarget_[idx(nx, ny)];
                            if (!aOk && !bOk) continue;

                            bool useB = bOk && (!aOk || dPose[idx(nx, ny)] < dPose[idx(x, y)] ||
                                                (dPose[idx(nx, ny)] == dPose[idx(x, y)] && tPose[idx(nx, ny)] < tPose[idx(x, y)]));
                            Cand c;
                            c.ax = x; c.ay = y; c.bx = nx; c.by = ny; c.d = d; c.bound = bound;
                            c.tx = useB ? nx : x; c.ty = useB ? ny : y;
                            c.face = useB ? opposite(d) : d;
                            int ci = idx(c.tx, c.ty);
                            c.trip = static_cast<int>(dPose[ci]);
                            c.turns = tPose[ci];
                            cands.push_back(c);
                        }

                if (cands.empty()) {
                    if (bestP.found) {
                        if (mx_ != 0 || my_ != 0) {
                            PhaseHeur ph{HeurKind::kTrueDistance, 1, 1};
                            runTo(atStart, ph);
                        }
                        finalizeCert(bestP);
                        return true;
                    }
                    PhaseHeur fallback{HeurKind::kOptimistic, cfg_.phase2.epsNum, cfg_.phase2.epsDen};
                    auto atUnknown = [&](int x, int y) {
                        for (int di = 0; di < kDirCount; ++di)
                            if (known_.status(x, y, static_cast<Dir>(di)) == kUnknown) return true;
                        return false;
                    };
                    if (runTo(atUnknown, fallback) == RunStatus::kStuck) return false;
                    continue;
                }

                Cand best = cands[0];
                int bestScore = best.trip + (kCostUnit / 2) * best.turns;
                for (const auto& c : cands) {
                    int sc = c.trip + (kCostUnit / 2) * c.turns;
                    if (sc < bestScore) {
                        best = c;
                        bestScore = sc;
                    }
                }

                vizTripTarget(best.tx, best.ty);
                if (mx_ == best.tx && my_ == best.ty) {
                    if (known_.status(best.tx, best.ty, best.face) == kUnknown)
                        faceForUnknown(best.face);
                    continue;
                }

                ++st_.trips;
                PhaseHeur tripHeur{HeurKind::kOptimistic, cfg_.phase2.epsNum, cfg_.phase2.epsDen};
                Cost tripH[kMAZE];
                int tsrc[1] = {idx(best.tx, best.ty)};
                dijkstraCells(Maze(), tsrc, 1, tripH);
                dbgTx_ = best.tx; dbgTy_ = best.ty;
                auto atT = [&](int x, int y) { return x == best.tx && y == best.ty; };
                if (runTo(atT, tripHeur, tripH) == RunStatus::kStuck) {
                    skipTarget_[idx(best.tx, best.ty)] = true;
                    continue;
                }
                if (known_.status(best.tx, best.ty, best.face) == kUnknown)
                    faceForUnknown(best.face);
            }
            return false;
        }

        // Standing baseline Phase 2 (exact 525-maze byte parity)
        bool phase2Standing()
        {
            st_.phase = 2;
            vizPathColor_ = 'a';
            std::memset(skipTarget_, 0, sizeof(skipTarget_));
            int guard = 0;
            iter_ = 0;
            while (guard++ < 400)
            {
                ++iter_;
                Maze snap = known_.snapshot();
                PathResult best = plan(0, 0, D_N, [&](int x, int y) { return isGoalTruth(x, y); }, snap,
                                       PhaseHeur{HeurKind::kTrueDistance, 1, 1});
                if (!best.found && known_.unknownCount() == 0)
                    return false;
                if (best.found)
                {
                    st_.certBestKnown = best.cost / static_cast<double>(kCostUnit);
                    Cost ds[kMAZE], dc[kMAZE];
                    std::vector<FrontierEdge> frontier;
                    bool exist = improvementFrontier(frontier, best.cost, ds, dc);
                    st_.certOptimistic = (dc[idx(0, 0)] < INT32_MAX ? dc[idx(0, 0)] / (double)kCostUnit : -1);
                    if (!exist)
                    {
                        st_.certifiedAfterCells = static_cast<int>(st_.totalDistance);
                        st_.certBound = st_.certBestKnown;
                        certified_ = best;
                        vizFinal(best);
                        sealed_ = true;
                        return true;
                    }
                    Cost dm[kMAZE];
                    int msrc[1] = {idx(mx_, my_)};
                    dijkstraCells(snap, msrc, 1, dm);

                    const double VBAND = 10.0;
                    struct EP
                    {
                        bool ok = false;
                        int tx = 0, ty = 0;
                        Dir face = D_N;
                        double trip = 0;
                    };
                    auto pickEP = [&](const FrontierEdge& e)
                    {
                        EP r;
                        int dA = dm[idx(e.ax, e.ay)];
                        int dB = dm[idx(e.bx, e.by)];
                        bool aOk = dA != INT32_MAX && !skipTarget_[idx(e.ax, e.ay)];
                        bool bOk = dB != INT32_MAX && !skipTarget_[idx(e.bx, e.by)];
                        if (!aOk && !bOk) return r;
                        bool useB = bOk && (!aOk || dB < dA);
                        r.ok = true;
                        r.tx = useB ? e.bx : e.ax;
                        r.ty = useB ? e.by : e.ay;
                        r.face = useB ? opposite(e.d) : e.d;
                        r.trip = (useB ? dB : dA) / (double)kCostUnit;
                        return r;
                    };
                    auto savingOf = [&](const FrontierEdge& e) { return (best.cost - e.bound) / (double)kCostUnit; };

                    const FrontierEdge* bestEdge = nullptr;
                    double bestVal = -1e18;
                    for (const auto& e : frontier)
                    {
                        EP c = pickEP(e);
                        if (!c.ok) continue;
                        double val = savingOf(e) - c.trip;
                        if (val > bestVal) bestVal = val;
                    }
                    if (bestVal <= -1e17)
                    {
                        st_.certifiedAfterCells = static_cast<int>(st_.totalDistance);
                        st_.certBound = st_.certBestKnown;
                        certified_ = best;
                        vizFinal(best);
                        sealed_ = true;
                        return true;
                    }
                    int bestTrip = INT32_MAX;
                    Cost bestBound = INT32_MAX;
                    int bestShared = -1;
                    for (const auto& e : frontier)
                    {
                        EP c = pickEP(e);
                        if (!c.ok) continue;
                        double val = savingOf(e) - c.trip;
                        if (val < bestVal - VBAND) continue;
                        int shared = 0;
                        for (const auto& o : frontier)
                        {
                            EP co = pickEP(o);
                            if (!co.ok) continue;
                            int dx = co.tx - c.tx, dy = co.ty - c.ty;
                            if (std::abs(dx) + std::abs(dy) <= 0) ++shared;
                        }
                        bool better = false;
                        if (c.trip < bestTrip) better = true;
                        else if (c.trip == bestTrip)
                        {
                            if (e.bound < bestBound) better = true;
                            else if (e.bound == bestBound && shared > bestShared) better = true;
                        }
                        if (better)
                        {
                            bestEdge = &e;
                            bestTrip = static_cast<int>(c.trip);
                            bestBound = e.bound;
                            bestShared = shared;
                        }
                    }
                    auto serviceEdge = [&](const FrontierEdge& e)
                    {
                        EP use = pickEP(e);
                        vizTripTarget(use.tx, use.ty);
                        if (mx_ == use.tx && my_ == use.ty)
                        {
                            faceForUnknown(use.face);
                        }
                        else
                        {
                            ++st_.trips;
                            PhaseHeur tripHeur;
                            tripHeur.epsNum = cfg_.phase2.epsNum;
                            tripHeur.epsDen = cfg_.phase2.epsDen;
                            Cost tripH[kMAZE];
                            int tsrc[1] = {idx(use.tx, use.ty)};
                            dijkstraCells(Maze(), tsrc, 1, tripH);
                            dbgTx_ = use.tx; dbgTy_ = use.ty;
                            auto atT = [&](int x, int y) { return x == use.tx && y == use.ty; };
                            if (runTo(atT, tripHeur, tripH) == RunStatus::kStuck)
                            {
                                skipTarget_[idx(use.tx, use.ty)] = true;
                                return false;
                            }
                            if (known_.status(use.tx, use.ty, use.face) == kUnknown)
                                faceForUnknown(use.face);
                        }
                        return true;
                    };
                    if (!serviceEdge(*bestEdge)) continue;
                    continue;
                }
                PhaseHeur fallback;
                fallback.kind = HeurKind::kOptimistic;
                fallback.epsNum = cfg_.phase2.epsNum;
                fallback.epsDen = cfg_.phase2.epsDen;
                auto atUnknownNeighbor = [&](int x, int y)
                {
                    for (int di = 0; di < kDirCount; ++di)
                        if (known_.status(x, y, static_cast<Dir>(di)) == kUnknown) return true;
                    return false;
                };
                if (runTo(atUnknownNeighbor, fallback) == RunStatus::kStuck) return false;
            }
            return false;
        }

        bool phase2()
        {
            if (cfg_.sweep)
                return phase2Sweep();
            return phase2Standing();
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
                if (runAtStart_) ++curRunTurns_;
                if (!sealed_) sense();
            }
        }

        void faceForUnknown(Dir d)
        {
            for (int k = 0; k < 4; ++k)
            {
                if (known_.status(mx_, my_, d) != kUnknown) return;
                w_.turnRight();
                mdir_ = static_cast<Dir>((mdir_ + 1) % kDirCount);
                ++st_.totalTurns;
                if (runAtStart_) ++curRunTurns_;
                sense();
            }
        }

        bool phase3()
        {
            st_.phase = 3;
            w_.clearAllColor();
            std::memset(cellPaint_, 0, sizeof(cellPaint_));
            vizGoals();
            PhaseHeur ph{HeurKind::kTrueDistance, 1, 1};
            auto atStart = [&](int x, int y) { return x == 0 && y == 0; };
            if (mx_ == 0 && my_ == 0) return true;
            return runTo(atStart, ph) == RunStatus::kDone;
        }

        bool phase4()
        {
            st_.phase = 4;
            PhaseHeur ph{HeurKind::kTrueDistance, 1, 1};
            auto atGoal = [&](int x, int y) { return isGoalTruth(x, y); };
            if (sealed_)
            {
                Maze pes = pessimisticSnapshot();
                PathResult fin = plan(mx_, my_, mdir_, atGoal, pes, ph);
                if (!fin.found || fin.states.size() < 2) return false;
                vizFinal(fin);
                faceDir(fin.states[1].dir);
            }
            else
            {
                Maze snap = known_.snapshot();
                PathResult p = plan(mx_, my_, mdir_, atGoal, snap, ph);
                if (!p.found || p.states.size() < 2) return false;
                vizFinal(p);
                faceDir(p.states[1].dir);
            }
            RunStatus rs = runTo(atGoal, ph);
            if (rs != RunStatus::kDone) return false;
            st_.sprintDist = static_cast<int>(curRunDist_);
            st_.sprintTurns = curRunTurns_;
            return isGoalHere();
        }

        bool isGoalTruth(int x, int y) const { return truth_.isGoal(x, y); }
        int dbgTx_ = -1, dbgTy_ = -1;
        bool skipTarget_[MAZE_W * MAZE_H] = {};
        bool vDebug_ = false, vTrace_ = false, vTrace2_ = false;
        bool sealed_ = false;

        IWorld& w_;
        const Maze& truth_;
        SolverConfig cfg_;
        Astar astar_;
        KnownMaze known_;
        int mx_ = 0, my_ = 0;
        Dir mdir_ = D_N;
        int arrivalFace_ = -1;
        SolverStats st_;
        PathResult certified_;
        int iter_ = 0;
        bool runAtStart_ = false;
        double curRunDist_ = 0, curRunEff_ = 0;
        int curRunTurns_ = 0;
    };

} // namespace mm
