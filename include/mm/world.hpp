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
