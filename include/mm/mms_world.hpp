#pragma once
// MmsWorld: talks the mms simulator protocol over stdin/stdout
// (run inside the mms GUI; see github.com/mackorone/mms).
#include <cstdio>
#include <string>
#include "mm/world.hpp"

namespace mm {

class MmsWorld : public IWorld {
 public:
  bool init() {
    int w = intValue("mazeWidth");
    int h = intValue("mazeHeight");
    return w == MAZE_W && h == MAZE_H;
  }

  bool wallFront() override { return boolValue("wallFront"); }
  bool wallRight() override { return boolValue("wallRight"); }
  bool wallLeft() override { return boolValue("wallLeft"); }
  // The mms protocol exposes the 3-face rig only (this cell): no 2-cell
  // look-ahead exists in this rig, and cfg.lookAhead (--look-ahead) stays
  // off for simulators -- the solver never asks and plans strictly on
  // measured faces. (Return -1 "not calibrated" if it ever asked anyway.)
  int wallAhead2() override { return -1; }
  // Diagonal-sensor channels (cfg.lookAheadDiag, --look-ahead-diag) do not
  // exist in the mms protocol either: always "not calibrated".
  int wallLeft2() override { return -1; }
  int wallRight2() override { return -1; }

  bool moveForward(int cells) override {
    send("moveForward %d", cells);
    std::string resp = readLine();
    return resp == "ack";
  }
  void turnRight() override { send("turnRight"); readLine(); }
  void turnLeft() override { send("turnLeft"); readLine(); }

  void setColor(int x, int y, char c) override { send("setColor %d %d %c", x, y, c); }
  void setText(int x, int y, const char* t) override {
    send("setText %d %d %s", x, y, t);
  }
  void clearAllColor() override { send("clearAllColor"); }
  void clearAllText() override { send("clearAllText"); }

  double getStat(const char* stat) {
    send("getStat %s", stat);
    std::string r = readLine();
    return r.empty() ? -1.0 : std::atof(r.c_str());
  }
  bool wasReset() {
    send("wasReset");
    return readLine() == "true";
  }

 private:
  template <class... A>
  void send(const char* fmt, A... a) {
    if constexpr (sizeof...(A) == 0) {
      // zero-arg sends pass whole literal lines ("mazeWidth") -- print via
      // %s so the format is always a string literal (-Wformat-security)
      std::printf("%s", fmt);
    } else {
      std::printf(fmt, a...);
    }
    std::printf("\n");
    std::fflush(stdout);
    if (getenv("MM_PROTO")) fprintf(stderr, "SENT: ");
    if (getenv("MM_PROTO")) {
      if constexpr (sizeof...(A) == 0)
        fprintf(stderr, "%s", fmt);
      else
        fprintf(stderr, fmt, a...);
      fprintf(stderr, "\n");
    }
  }
  std::string readLine() {
    std::string s;
    int c;
    while ((c = std::getchar()) != '\n' && c != EOF)
      if (c != '\r') s.push_back(static_cast<char>(c));
    if (getenv("MM_PROTO") && c == EOF)
      fprintf(stderr, "PROTOCOL: EOF on stdin (simulator closed the pipe)\n");
    else if (getenv("MM_PROTO"))
      fprintf(stderr, "RECV: %s\n", s.c_str());
    return s;
  }
  int intValue(const char* cmd) {
    send("%s", cmd);
    std::string r = readLine();
    return atoi(r.c_str());
  }
  bool boolValue(const char* cmd) {
    send("%s", cmd);
    return readLine() == "true";
  }
};

}  // namespace mm
