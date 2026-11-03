#pragma once
// IWorld: the abstract robot interface. Two implementations:
//  - SimWorld: in-process offline simulator (mms semantics, fast, scriptable)
//  - MmsWorld: speaks the real mms stdin/stdout protocol
// Keeping both behind one interface means the algorithm (solver) is identical
// for benchmarking and for the GUI.
#include <string>
#include "mm/types.hpp"

namespace mm {

class IWorld {
 public:
  virtual ~IWorld() = default;
  // Sensor queries, relative to current heading
  virtual bool wallFront() = 0;
  virtual bool wallRight() = 0;
  virtual bool wallLeft() = 0;
  // Calibrated 2-cell look-ahead IR: line of sight along the CURRENT heading,
  // reporting the face BETWEEN the next cell and the one after it. Only
  // trustworthy when the immediate front face is open (the solver consults
  // it exclusively in that case, so the sight is physically unobstructed).
  //   0 = no wall detected, 1 = wall detected
  //  -1 = this rig is NOT calibrated for 2-cell reads (no such protocol
  //       command) -- the solver then plans strictly on the 3-face rig.
  // SOLVER-SIDE GATE: the solver only ever calls this when cfg.lookAhead is
  // set (--look-ahead, real hardware with the diagonal-sensor package);
  // simulators run with the flag off and never ask -- no rig to emulate.
  // DIAGONAL CHANNELS (--look-ahead-diag): wallLeft2()/wallRight2() are the
  // same idea for the SIDE faces of the REMOTE cell (see sense()); same
  // return contract, same honesty rule (-1 = rig cannot read that channel).
  virtual int wallAhead2() { return -1; }
  // Diagonal-sensor channels for the REMOTE cell (one cell ahead along the
  // current heading): the wall on the remote cell's LEFT / RIGHT side.
  //   0 = no wall detected, 1 = wall detected, -1 = channel not available.
  virtual int wallLeft2() { return -1; }
  virtual int wallRight2() { return -1; }
  // Motion. Returns false on crash (run is over, abandon and report).
  virtual bool moveForward(int cells = 1) = 0;
  virtual void turnRight() = 0;
  virtual void turnLeft() = 0;
  // GUI helpers (no-ops in SimWorld unless enabled)
  virtual void setColor(int x, int y, char c) { (void)x; (void)y; (void)c; }
  virtual void setText(int x, int y, const char* t) { (void)x; (void)y; (void)t; }
  virtual void clearAllColor() {}
  virtual void clearAllText() {}
  virtual void resetDisplay() {}
  // debug: report true pose (default: not available => -1, check skipped)
  virtual void debugPos(int* x, int* y, int* dir) {
    if (x) *x = -1;
    if (y) *y = -1;
    if (dir) *dir = -1;
  }
};

}  // namespace mm
