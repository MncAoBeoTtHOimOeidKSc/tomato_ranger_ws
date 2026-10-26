#ifndef RANGER_COMMAND_BUFFER_HPP
#define RANGER_COMMAND_BUFFER_HPP

#include <chrono>
#include <cmath>
#include <geometry_msgs/Twist.h>

namespace westonrobot {
class CommandBuffer {
 public:
  using Clock = std::chrono::steady_clock;
  explicit CommandBuffer(double timeout) : timeout_(timeout) {}

  void Store(const geometry_msgs::Twist& command,
             Clock::time_point now = Clock::now()) {
    command_ = command;
    if (!std::isfinite(command.linear.x) ||
        !std::isfinite(command.linear.y) ||
        !std::isfinite(command.linear.z) ||
        !std::isfinite(command.angular.x) ||
        !std::isfinite(command.angular.y) ||
        !std::isfinite(command.angular.z)) {
      command_ = geometry_msgs::Twist();
    }
    received_ = now;
    active_ = true;
  }

  geometry_msgs::Twist Read(Clock::time_point now = Clock::now()) const {
    if (!active_ || std::chrono::duration<double>(now - received_).count() >= timeout_) {
      return geometry_msgs::Twist();
    }
    return command_;
  }

  bool Active() const { return active_; }
  void Clear() { active_ = false; }

 private:
  double timeout_;
  bool active_ = false;
  Clock::time_point received_;
  geometry_msgs::Twist command_;
};
}  // namespace westonrobot
#endif
