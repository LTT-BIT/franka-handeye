// Read-only libfranka telemetry. Deliberately contains no control/setup commands.
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
template <size_t N>
void array_json(const std::array<double, N>& values) {
  std::cout << '[';
  for (size_t i = 0; i < N; ++i) {
    if (i) std::cout << ',';
    std::cout << values[i];
  }
  std::cout << ']';
}
const char* mode_name(franka::RobotMode mode) {
  switch (mode) {
    case franka::RobotMode::kIdle: return "idle";
    case franka::RobotMode::kGuiding: return "guiding";
    case franka::RobotMode::kMove: return "move";
    case franka::RobotMode::kReflex: return "reflex";
    case franka::RobotMode::kUserStopped: return "user_stopped";
    case franka::RobotMode::kAutomaticErrorRecovery: return "recovery";
    default: return "other";
  }
}
}
int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: franka_state_reader ROBOT_IP\n";
    return 2;
  }
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  std::signal(SIGPIPE, stop);
  try {
    // No realtime control thread: scheduling enforcement is unnecessary for read().
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    uint64_t last_ms = 0;
    std::cout << std::setprecision(17);
    robot.read([&](const franka::RobotState& state) {
      if (!running || !std::cout.good()) return false;
      const uint64_t robot_ms = state.time.toMSec();
      if (robot_ms >= last_ms && robot_ms - last_ms < 5) return true;
      last_ms = robot_ms;
      const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      std::cout << "{\"host_stamp_ns\":" << now << ",\"robot_time_ms\":" << robot_ms
                << ",\"robot_mode\":\"" << mode_name(state.robot_mode) << "\",\"O_T_EE\":";
      array_json(state.O_T_EE);
      std::cout << ",\"F_T_EE\":";
      array_json(state.F_T_EE);
      std::cout << ",\"q\":";
      array_json(state.q);
      std::cout << "}\n" << std::flush;
      return static_cast<bool>(running);
    });
  } catch (const std::exception& error) {
    std::cerr << "Read-only Franka connection failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
