#include <lane_change_planner/state/state_base_class.h>

namespace lane_change_planner {
std::ostream& operator<<(std::ostream& ostream, const State& state) {
  switch (state) {
    case State::NO_STATE:
      ostream << std::string("NO_STATE");
      break;
    case State::FOLLOWING_LANE:
      ostream << std::string("FOLLOWING_LANE");
      break;
    case State::EXECUTING_LANE_CHANGE:
      ostream << std::string("EXECUTING_LANE_CHANGE");
      break;
    case State::ABORTING_LANE_CHANGE:
      ostream << std::string("ABORTING_LANE_CHANGE");
      break;
    case State::FORCING_LANE_CHANGE:
      ostream << std::string("FORCING_LANE_CHANGE");
      break;
    case State::BLOCKED_BY_OBSTACLE:
      ostream << std::string("BLOCKED_BY_OBSTACLE");
      break;
    default:
      ostream << std::string("NO_STATE");
      break;
  }
  return ostream;
}

Status StateBase::getStatus() const { return status_; }
}  // namespace lane_change_planner
