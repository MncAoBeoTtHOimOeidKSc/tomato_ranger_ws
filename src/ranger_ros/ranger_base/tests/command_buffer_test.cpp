#include <cassert>
#include <limits>
#include "ranger_base/command_buffer.hpp"

int main() {
  using westonrobot::CommandBuffer;
  CommandBuffer buffer(0.6);
  const auto start = CommandBuffer::Clock::time_point();
  assert(!buffer.Active());
  assert(buffer.Read(start).angular.z == 0.0);

  geometry_msgs::Twist command;
  command.angular.z = 0.08;
  buffer.Store(command, start);
  assert(buffer.Active());
  // One 5 Hz input must remain available for repeated 50 Hz CAN sends.
  for (int i = 0; i < 10; ++i) {
    assert(buffer.Read(start + std::chrono::milliseconds(20 * i)).angular.z == 0.08);
  }
  assert(buffer.Read(start + std::chrono::milliseconds(599)).angular.z == 0.08);
  assert(buffer.Read(start + std::chrono::milliseconds(600)).angular.z == 0.0);
  assert(buffer.Read(start + std::chrono::seconds(5)).angular.z == 0.0);

  command.angular.z = -0.08;
  buffer.Store(command, start + std::chrono::seconds(6));
  assert(buffer.Read(start + std::chrono::seconds(6)).angular.z == -0.08);
  buffer.Clear();
  assert(!buffer.Active());
  assert(buffer.Read(start + std::chrono::seconds(6)).angular.z == 0.0);

  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  buffer.Store(command, start);
  assert(buffer.Read(start).linear.x == 0.0);
  assert(buffer.Read(start).angular.z == 0.0);
  command.linear.x = 0.0;
  command.angular.z = std::numeric_limits<double>::infinity();
  buffer.Store(command, start);
  assert(buffer.Read(start).angular.z == 0.0);
}
