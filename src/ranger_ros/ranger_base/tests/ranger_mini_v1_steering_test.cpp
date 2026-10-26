#include <cassert>
#include <cmath>

#include "ugv_sdk/details/robot_base/ranger_base.hpp"

// Inject feedback without opening a CAN socket or sending commands.
class OfflineMiniV1 : public westonrobot::RangerMiniV1Base {
 public:
  void Feedback(uint8_t mode, double steering, double angular) {
    core_state_msgs_ = {};
    core_state_msgs_.motion_mode_state.motion_mode = mode;
    core_state_msgs_.motion_state.linear_velocity = 0.1;
    core_state_msgs_.motion_state.steering_angle = steering;
    core_state_msgs_.motion_state.angular_velocity = angular;
  }
};

int main() {
  using Mode = westonrobot::RangerInterface::MotionMode;
  OfflineMiniV1 robot;
  const double scale = 10.0 / 180.0 * 3.14;
  for (double sign : {-1.0, 1.0}) {
    robot.Feedback(Mode::kDualAckerman, sign, 0.08);
    auto ackermann = robot.GetRobotState();
    assert(std::abs(ackermann.motion_state.steering_angle - sign * scale) < 1e-6);
    assert(std::abs(ackermann.motion_state.linear_velocity - 0.1) < 1e-6);

    robot.Feedback(Mode::kSpinning, sign, sign * 0.08);
    auto spinning = robot.GetRobotState();
    assert(std::abs(spinning.motion_state.angular_velocity - sign * 0.08) < 1e-6);
    assert(spinning.motion_state.linear_velocity == 0.0);
    assert(std::abs(spinning.motion_state.steering_angle + sign * scale) < 1e-6);

    robot.Feedback(Mode::kParallel, sign, 0.0);
    assert(std::abs(robot.GetRobotState().motion_state.steering_angle +
                    sign * scale) < 1e-6);
  }
}
