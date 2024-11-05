#include <motor/motor_client_imitation.hpp>

namespace motor_client {
MotorClientImitation::MotorClientImitation(double rate_) : rate(rate_) {
  ctime = 1.0 / rate * 1000 * 1000;  // microseconds
  cycle_thread = std::thread(&MotorClientImitation::cycle, this);
}

MotorClientImitation::~MotorClientImitation() {
  terminated.store(true);
  if (cycle_thread.joinable()) cycle_thread.join();
}

void MotorClientImitation::cycle() {
  auto cycletime = std::chrono::microseconds(this->ctime);

  while (!terminated.load()) {
    auto start = std::chrono::high_resolution_clock::now();
    update();
    auto end = std::chrono::high_resolution_clock::now();
    const auto delta = end - start;
    if (delta > cycletime) {
      std::cout << "System too slow for cycle time " << cycletime.count()
                << "ms sending takes " << delta.count() << "ns" << std::endl;
    } else {
      std::this_thread::sleep_for(cycletime - delta);
    }
  }
}

void MotorClientImitation::update() {
  if (actuator.mode == ControlMode::STOPPED) {
    actuator.state.position = actuator.state.position;
    actuator.state.velocity = 0;
    actuator.state.effort = 0;
    return;
  }
  if (actuator.mode == ControlMode::POSITION) {
    double state_pos = actuator.state.position;
    double command_pos = actuator.command.position;
    if (rate == 0) throw std::runtime_error("rate = 0");
    double delta = profile_velocity_ / rate;
    state_pos += (state_pos < command_pos)
                     ? std::min(delta, command_pos - state_pos)
                     : std::max(-delta, command_pos - state_pos);
    actuator.state.velocity = profile_velocity_;
    actuator.state.effort = actuator.command.velocity;
  }
  if (actuator.mode == ControlMode::VELOCITY) {
    actuator.state.position += actuator.command.velocity / rate;
    actuator.state.velocity = actuator.command.velocity;
    actuator.state.effort = actuator.command.velocity;
  }
  if (actuator.mode == ControlMode::EFFORT) {
    actuator.state.position += actuator.command.effort / rate;
    actuator.state.velocity = actuator.command.effort;
    actuator.state.effort = actuator.command.effort;
  }
}

void MotorClientImitation::setVelocity(double velocity) {
  std::lock_guard<std::recursive_mutex> lock(actuator_mutex);
  actuator.mode = ControlMode::VELOCITY;
  actuator.command.velocity = velocity;
}

void MotorClientImitation::setPosition(double target, bool relative) {
  actuator.mode = ControlMode::POSITION;
  actuator.command.position = target;
}

void MotorClientImitation::setEffort(double target) {
  actuator.mode = ControlMode::EFFORT;
  actuator.command.effort = target;
}

void MotorClientImitation::enable() {
  actuator.status = MotorStatus::OK;
  actuator.mode = ControlMode::STOPPED;
}

void MotorClientImitation::disable() { actuator.mode = ControlMode::STOPPED; }

void MotorClientImitation::stop() { actuator.mode = ControlMode::STOPPED; }

void MotorClientImitation::reset() {
  actuator.status = MotorStatus::OK;
  actuator.mode = ControlMode::STOPPED;
}

void MotorClientImitation::home(bool always) {
  actuator.status = MotorStatus::OK;
  actuator.mode = ControlMode::STOPPED;
}

MotorState MotorClientImitation::getState() {
  std::lock_guard<std::recursive_mutex> lock(actuator_mutex);
  return MotorState{
      actuator.state.position,
      actuator.state.velocity,
      actuator.state.effort,
  };
}

MotorStatus MotorClientImitation::getStatus() {
  std::lock_guard<std::recursive_mutex> lock(actuator_mutex);
  return actuator.status;
}
MotorType MotorClientImitation::getType() { return MotorType::IMITATOR; }
unsigned MotorClientImitation::getFault() { return 0; }
bool MotorClientImitation::readyState() { return true; }

}  // namespace motor_client
