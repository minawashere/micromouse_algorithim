#pragma once
// SimWorld: faithful in-process re-implementation of the mms rules
// (movement, crashes, scorekeeping) for fast automated benchmarking.
#include "mm/maze.hpp"
#include "mm/world.hpp"

namespace mm {

struct ScoreStats {
  double totalDistance = 0, totalEffDistance = 0, bestRunDistance = -1,
         bestRunEff = -1, bestRunTurns = -1, bestRunEffTurns = -1;
  int totalTurns = 0, bestRunTurnsCount = 0;
  double currentRunDistance = 0, currentRunEff = 0;
  int currentRunTurns = 0;
  bool inRun = false;
  double score() const {
    if (bestRunDistance < 0) return 2000.0;
    return bestRunTurns + bestRunEff +
           0.1 * (totalTurns + totalEffDistance);
  }
};

class SimWorld : public IWorld {
 public:
  explicit SimWorld(const Maze& truth, bool verbose = false)
      : truth_(truth), verbose_(verbose) {
    resetMouse();
  }
  int x() const { return x_; }
  int y() const { return y_; }
  Dir dir() const { return dir_; }
  bool crashed() const { return crashed_; }
  const ScoreStats& stats() const { return st_; }

  bool wallFront() override { return truth_.wall(x_, y_, dir_); }
  bool wallRight() override {
    return truth_.wall(x_, y_, static_cast<Dir>((dir_ + 1) % kDirCount));
  }
  bool wallLeft() override {
    return truth_.wall(x_, y_, static_cast<Dir>((dir_ + 3) % kDirCount));
  }
  // Calibrated 2-cell IR: the wall between the next cell and the one after
  // it, along the current heading (line-of-sight: the caller has already
  // proven the immediate front face open). Out-of-range neighbor cell means
  // the face lies on the border -> wall.
  int wallAhead2() override {
    Dir d = dir_;
    int nx = x_ + kDx[d], ny = y_ + kDy[d];
    if (nx < 0 || ny < 0 || nx >= MAZE_W || ny >= MAZE_H)
      return 1;
    return truth_.wall(nx, ny, d) ? 1 : 0;
  }
  // Calibrated diagonal channels: the left / right faces of the REMOTE cell
  // (one cell ahead). Needs the remote cell to exist; only consulted when
  // the face between here and it is known open (the solver's gate), so the
  // sight line is physically unobstructed.
  int wallSide2(int off) {
    Dir d = dir_;
    int fx = x_ + kDx[d], fy = y_ + kDy[d];
    if (fx < 0 || fy < 0 || fx >= MAZE_W || fy >= MAZE_H)
      return -1; // no remote cell to read from inside
    Dir sd = static_cast<Dir>((static_cast<int>(d) + off) % kDirCount);
    return truth_.wall(fx, fy, sd) ? 1 : 0;
  }
  int wallLeft2() override { return wallSide2(3); }
  int wallRight2() override { return wallSide2(1); }

  bool moveForward(int cells) override {
    if (verbose_)
      std::fprintf(stderr, "SIM move n=%d from=(%d,%d) dir=%d\n", cells, x_, y_,
                   (int)dir_);
    if (cells < 1) { crash(); return false; }
    for (int i = 0; i < cells; ++i) {
      if (truth_.wall(x_, y_, dir_)) {
        if (verbose_)
          std::fprintf(stderr, "SIM CRASH at (%d,%d) dir=%d step %d/%d\n", x_, y_,
                       (int)dir_, i + 1, cells);
        crash();
        return false;
      }
      x_ += kDx[dir_];
      y_ += kDy[dir_];
      if (x_ < 0 || y_ < 0 || x_ >= MAZE_W || y_ >= MAZE_H) { crash(); return false; }
    }
    // scorekeeping: distance = cells; effective distance: cells>2 halves the
    // extras (simulates higher speed on straights)
    st_.totalDistance += cells;
    st_.totalEffDistance += cells <= 2 ? cells : 2.0 + 0.5 * (cells - 2);
    if (st_.inRun) {
      st_.currentRunDistance += cells;
      st_.currentRunEff += cells <= 2 ? cells : 2.0 + 0.5 * (cells - 2);
    }
    checkRunBoundaries();
    return true;
  }

  void turnRight() override {
    dir_ = static_cast<Dir>((dir_ + 1) % kDirCount);
    ++st_.totalTurns;
    if (st_.inRun) ++st_.currentRunTurns;
  }
  void turnLeft() override {
    dir_ = static_cast<Dir>((dir_ + 3) % kDirCount);
    ++st_.totalTurns;
    if (st_.inRun) ++st_.currentRunTurns;
  }

  // scorekeeping boundary logic: a run = start visit ... goal visit
  void checkRunBoundaries() {
    if (x_ == 0 && y_ == 0) {
      st_.inRun = true;
      st_.currentRunDistance = 0;
      st_.currentRunEff = 0;
      st_.currentRunTurns = 0;
      return;
    }
    if (st_.inRun && truth_.isGoal(x_, y_)) {
      st_.inRun = false;
      if (st_.bestRunDistance < 0 || st_.currentRunEff + st_.currentRunTurns <
                                         st_.bestRunEff + st_.bestRunTurns) {
        st_.bestRunDistance = st_.currentRunDistance;
        st_.bestRunEff = st_.currentRunEff;
        st_.bestRunTurnsCount = st_.currentRunTurns;
        st_.bestRunTurns = st_.currentRunTurns;
      }
    }
  }
  void debugPos(int* x, int* y, int* dir) override {
    if (x) *x = x_;
    if (y) *y = y_;
    if (dir) *dir = (int)dir_;
  }
  void resetMouse() { x_ = 0; y_ = 0; dir_ = D_N; crashed_ = false; }

 private:
  void crash() { crashed_ = true; }

  const Maze& truth_;
  bool verbose_;
  int x_ = 0, y_ = 0;
  Dir dir_ = D_N;
  bool crashed_ = false;
  ScoreStats st_;
};

}  // namespace mm
