#include <turn_signal_decider/turn_signal_decider.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "turn_signal_decider");

  turn_signal_decider::TurnSignalDecider decider;

  ros::spin();

  return 0;
}