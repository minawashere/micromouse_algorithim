// See maze_io.hpp. Classic format layout (per mazefiles repo):
//   line 0:      top border, every 4th char is a pillar 'o', walls are '-'
//   line 2k+1:   cell row: pillars 'o' every 4 chars, '|' = vertical walls
//   line 2k+2:   horizontal walls between rows ('-' runs between pillars)
// Wall segments: on pillar lines, chars between pillars; on cell lines,
// the pillar column char itself.
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>
#include "mm/maze_io.hpp"

namespace mm {

static bool isWallChar(char c) { return c == '-' || c == '|' || c == '+'; }

bool loadClassic(const std::vector<std::string>& lines, Maze& m, std::string* err) {
  if (lines.empty()) { if (err) *err = "empty file"; return false; }
  int totalRows = static_cast<int>(lines.size());
  size_t w = lines[0].size();
  for (auto& l : lines)
    if (l.size() != w) { if (err) *err = "ragged rows"; return false; }
  if ((totalRows - 1) % 2 != 0) { if (err) *err = "bad row count"; return false; }
  int H = (totalRows - 1) / 2;
  if ((w - 1) % 4 != 0) { if (err) *err = "bad column count"; return false; }
  int W = static_cast<int>((w - 1) / 4);
  if (W != MAZE_W || H != MAZE_H) {
    if (err) {
      std::ostringstream os;
      os << "unexpected maze size " << W << "x" << H;
      *err = os.str();
    }
    return false;
  }
  const int colStep = 4;  // cells are 4 columns wide in classic format
  (void)colStep;
  // file row 0 = maze TOP = mm row MAZE_H-1 (y-up convention, mms parity)
  for (int r = 0; r < MAZE_H; ++r) {
    int y = H - 1 - r;  // mm row for file row r
    // horizontal walls north of mm row y: file line 2r between pillars
    const std::string& hrow = lines[2 * r];
    for (int x = 0; x < MAZE_W; ++x)
      if (hrow[4 * x + 1] == '-') m.setWall(x, y, D_N);
    // vertical wall west of cell x: pillar column of file row 2r+1
    const std::string& vrow = lines[2 * r + 1];
    for (int x = 0; x < MAZE_W; ++x)
      if (vrow[4 * x] == '|') m.setWall(x, y, D_W);
    // southern border handled on the closing row (file bottom)
    if (y == 0) {
      const std::string& last = lines[2 * r + 2];
      for (int x = 0; x < MAZE_W; ++x)
        if (last[4 * x + 1] == '-') m.setWall(x, y, D_S);
    }
  }
  // goal detection: 'G' anywhere in the cell interior of vrow lines
  std::vector<Cell> goals;
  for (int r = 0; r < MAZE_H; ++r) {
    const std::string& vrow = lines[2 * r + 1];
    for (int x = 0; x < MAZE_W; ++x)
      for (int k = 1; k <= 3; ++k)
        if (vrow[4 * x + k] == 'G') goals.push_back({x, MAZE_H - 1 - r});
  }
  if (goals.size() >= 1 && goals.size() <= 4) m.setGoals(goals);
  // outer eastern border: pillar rows char at 4*W
  for (int r = 0; r < MAZE_H; ++r)
    if (lines[2 * r + 1][4 * MAZE_W] == '|')
      m.setWall(MAZE_W - 1, MAZE_H - 1 - r, D_E);
  return true;
}

bool loadNum(const std::vector<std::string>& lines, Maze& m, std::string* err) {
  int count = 0;
  for (auto& line : lines) {
    std::istringstream is(line);
    int x, y, N, E, S, W;
    if (!(is >> x >> y >> N >> E >> S >> W)) continue;
    if (x < 0 || y < 0 || x >= MAZE_W || y >= MAZE_H) {
      if (err) *err = "cell out of bounds";
      return false;
    }
    if (N) m.setWall(x, y, D_N);
    if (E) m.setWall(x, y, D_E);
    if (S) m.setWall(x, y, D_S);
    if (W) m.setWall(x, y, D_W);
    ++count;
  }
  if (count != MAZE_W * MAZE_H) {
    if (err) {
      std::ostringstream os;
      os << "num format expects 256 lines, got " << count;
      *err = os.str();
    }
    return false;
  }
  return true;
}

bool loadMaze(const std::string& path, Maze& out, std::string* err) {
  std::ifstream f(path);
  if (!f) { if (err) *err = "cannot open " + path; return false; }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(f, line)) {
    while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
      line.pop_back();
    if (!line.empty()) lines.push_back(line);
  }
  // detect format: first line contains "num"-style digits vs walls
  bool firstIsNum = false;
  {
    std::istringstream is(lines[0]);
    int x; std::string rest;
    if ((is >> x) && !(is >> rest)) firstIsNum = false;
    std::istringstream is2(lines[0]);
    char c0 = 0; is2 >> c0;
    firstIsNum = isdigit(static_cast<unsigned char>(c0));
  }
  if (firstIsNum) return loadNum(lines, out, err);
  return loadClassic(lines, out, err);
}

std::string dumpMaze(const Maze& m) {
  std::ostringstream os;
  for (int y = MAZE_H - 1; y >= 0; --y) {  // top (max y) first: file parity
    std::string h = "o", v;
    for (int x = 0; x < MAZE_W; ++x) {
      h += m.wall(x, y, D_N) ? "---" : "   ";
      h += "o";
      v += m.wall(x, y, D_W) ? "|" : " ";
      v += "   ";
    }
    v += m.wall(MAZE_W - 1, y, D_E) ? "|" : " ";
    os << h << "\n" << v << "\n";
  }
  std::string h = "o";
  for (int x = 0; x < MAZE_W; ++x) {
    h += m.wall(x, 0, D_S) ? "---" : "   ";
    h += "o";
  }
  os << h << "\n";
  return os.str();
}

}  // namespace mm
