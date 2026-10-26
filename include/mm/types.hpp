#pragma once
// Muratello core types. Portable C++17, embedded-friendly (no dynamic allocation
// in the hot path, no exceptions, fixed-point costs).
#include <cstdint>

namespace mm {

constexpr int MAZE_W = 16;
constexpr int MAZE_H = 16;
constexpr int GOAL_CELLS = 4;  // center 2x2

// Fixed-point cost: 1 cell forward == kCostUnit.
using Cost = int32_t;
constexpr Cost kCostUnit = 256;

// Direction of travel. Position (x,y): x east, y south (row index).
enum Dir : uint8_t { D_N = 0, D_E = 1, D_S = 2, D_W = 3 };
constexpr int kDirCount = 4;

inline constexpr int kDx[kDirCount] = {0, 1, 0, -1};
inline constexpr int kDy[kDirCount] = {1, 0, -1, 0};  // y UP (mms convention)

// Wall bitmask bits per cell. Order matches Dir.
using WallMask = uint8_t;
constexpr uint8_t kWallBitN = 1 << 0;
constexpr uint8_t kWallBitE = 1 << 1;
constexpr uint8_t kWallBitS = 1 << 2;
constexpr uint8_t kWallBitW = 1 << 3;
constexpr uint8_t kDirBit[kDirCount] = {kWallBitN, kWallBitE, kWallBitS, kWallBitW};

inline constexpr Dir opposite(Dir d) {
  return static_cast<Dir>((d + 2) % kDirCount);
}
inline constexpr int idx(int x, int y) { return y * MAZE_W + x; }
// Turn distance in {0,1,2}: (to - from) mod 4 -> absolute turns right/left.
inline constexpr int turnPenaltySteps(Dir from, Dir to) {
  int d = static_cast<int>(to) - static_cast<int>(from);
  d = ((d % 4) + 4) % 4;
  return (d == 3) ? 1 : d;  // U-turn = 2 steps (two turns); left = 1, right = 1
}

struct State {  // position + heading: the node type of the search graph
  uint8_t x = 0, y = 0;
  Dir dir = D_N;
};
inline constexpr int stateIdx(int x, int y, Dir dir) {
  return (static_cast<int>(dir) << 8) | (y << 4) | x;
}

struct Cell { int x, y; };

}  // namespace mm
