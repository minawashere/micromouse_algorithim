#pragma once
// Maze file loading. Supports:
//  - classic micromouseonline format: 2 rows per cell-line, 4 cols per cell
//    (33 lines for 16x16), 'o' = pillar, '-'= horizontal wall, '|' = vertical,
//    'S' = start (assumed (0,0)), 'G' = goal
//  - mms "num" format: "X Y N E S W" lines
#include <string>
#include <vector>
#include "mm/maze.hpp"

namespace mm {

enum struct MazeFormat { kClassic = 0, kNum };

// Load truth maze from file. Returns false + error msg on failure.
bool loadMaze(const std::string& path, Maze& out, std::string* err = nullptr);

// Generic pretty-printer matching the classic format (for debugging)
std::string dumpMaze(const Maze& m);

}  // namespace mm
