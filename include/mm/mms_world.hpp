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
    std::printf(fmt, a...);
    std::printf("\n");
    std::fflush(stdout);
  }
  std::string readLine() {
    std::string s;
    int c;
    while ((c = std::getchar()) != '\n' && c != EOF)
      if (c != '\r') s.push_back(static_cast<char>(c));
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
