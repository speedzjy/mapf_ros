#pragma once

#ifndef PRIORITIZED_SIPP_ENV_H
#define PRIORITIZED_SIPP_ENV_H

#include "../utils/utility.hpp"

#include "../utils/neighbor.hpp"
#include "../utils/planresult.hpp"

using mapf::Neighbor;
using mapf::PlanResult;

struct State {
  State(int x, int y) : x(x), y(y) {}

  bool operator==(const State &other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  bool operator!=(const State &other) const {
    return std::tie(x, y) != std::tie(other.x, other.y);
  }

  bool operator<(const State &other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  friend std::ostream &operator<<(std::ostream &os, const State &s) {
    return os << "(" << s.x << "," << s.y << ")";
  }

  int x;
  int y;
};

namespace std {
template <> struct hash<State> {
  size_t operator()(const State &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std

enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

std::ostream &operator<<(std::ostream &os, const Action &a) {
  switch (a) {
  case Action::Up:
    os << "Up";
    break;
  case Action::Down:
    os << "Down";
    break;
  case Action::Left:
    os << "Left";
    break;
  case Action::Right:
    os << "Right";
    break;
  case Action::Wait:
    os << "Wait";
    break;
  }
  return os;
}

class Environment {
public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<State> obstacles,
              State goal)
      : m_dimx(dimx), m_dimy(dimy), m_obstacles(std::move(obstacles)),
        m_goal(goal) {}

  float admissibleHeuristic(const State &s) {
    return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
  }

  bool isSolution(const State &s) { return s == m_goal; }

  State getLocation(const State &s) { return s; }

  void getNeighbors(const State &s,
                    std::vector<Neighbor<State, Action, int>> &neighbors) {
    neighbors.clear();

    State up(s.x, s.y + 1);
    if (stateValid(up)) {
      neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
    }
    State down(s.x, s.y - 1);
    if (stateValid(down)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(down, Action::Down, 1));
    }
    State left(s.x - 1, s.y);
    if (stateValid(left)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(left, Action::Left, 1));
    }
    State right(s.x + 1, s.y);
    if (stateValid(right)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(right, Action::Right, 1));
    }
  }

  void onExpandNode(const State & /*s*/, int /*fScore*/, int /*gScore*/) {
    // std::cout << "expand: " << s << "g: " << gScore << std::endl;
  }

  void onDiscover(const State & /*s*/, int /*fScore*/, int /*gScore*/) {
    // std::cout << "  discover: " << s << std::endl;
  }

  bool isCommandValid(
      const State & /*s1*/, const State & /*s2*/, const Action & /*a*/,
      int earliestStartTime,     // can start motion at this time
      int /*latestStartTime*/,   // must have left s by this time
      int earliestArrivalTime,   // can only arrive at (s+cmd)
      int /*latestArrivalTime*/, // needs to arrive by this time at (s+cmd)
      int &t) {
    t = std::max<int>(earliestArrivalTime, earliestStartTime + 1);

    // TODO(whoenig): need to check for swaps here...

    // return t - 1 <= latestStartTime;
    return true;
  }

private:
  bool stateValid(const State &s) {
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(s) == m_obstacles.end();
  }

private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<State> m_obstacles;
  State m_goal;
};

#endif