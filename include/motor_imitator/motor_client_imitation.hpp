#pragma once

//
#include <motor_imitator/motor_client_interface.hpp>

//
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace motor_client {

enum class ActuatorStatus { DISABLE, MOVING, STOPPED, ERROR };

class ActuatorState {
 public:
  ActuatorState() : name(""), position(0), velocity(0), effort(0) {}

  std::string name;
  double position;
  double velocity;
  double effort;
};

class ActuatorCommand {
 public:
  ActuatorCommand() : position(0), velocity(0), effort(0) {}
  double position;
  bool relative = false;
  double velocity;
  double effort;
};

class Actuator {
 public:
  Actuator() : state(), command() {
    last_culculation_time =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
  }
  ~Actuator() {};

  MotorStatus status = MotorStatus::OK;
  ActuatorState state;
  ActuatorCommand command;
  double profile_velocity_ = 0.1;
  double profile_acceleration_ = 0.1;
  ControlMode mode = ControlMode::VELOCITY;

  double last_culculation_time = 0.0;
};

class MotorClientImitation : public MotorClientInterface {
 public:
  MotorClientImitation(double rate_ = 50);
  ~MotorClientImitation();
  MotorClientImitation(const MotorClientImitation&) = delete;
  MotorClientImitation& operator=(const MotorClientImitation&) = delete;
  MotorClientImitation(const MotorClientImitation&&) = delete;
  MotorClientImitation& operator=(const MotorClientImitation&&) = delete;

  MotorState getState() override;
  MotorStatus getStatus() override;
  MotorType getType() override;
  unsigned getFault() override;
  bool readyState() override;

  void enable() override;
  void disable() override;
  void stop() override;
  void reset() override;
  void home(bool always = false) override;

  void setPosition(double target, bool relative = false) override;
  void setVelocity(double velocity) override;
  void setEffort(double target) override;

  void setMode(ControlMode mode) override {
    std::lock_guard lck(actuator_mutex);
    actuator.mode = mode;
  }
  ControlMode getMode() override {
    std::lock_guard lck(actuator_mutex);
    return actuator.mode;
  }

  void setProfileVelocity(double profile_velocity) override {
    profile_velocity_ = profile_velocity;
  }
  void setMaxPosition(double max_position) override {
    // TODO: Implement
  }
  void setMinPosition(double min_position) override {
    // TODO: Implement
  }
  void setProfileAcceleratoin(double profile_acceleration) override {}
  void setProfileDeceleratoin(double profile_deceleration) override {}

 private:
  void cycle();
  void update();

  std::thread cycle_thread;
  std::atomic<bool> terminated{false};

  Actuator actuator;
  std::recursive_mutex actuator_mutex;

  int actuators_number;
  double rate;
  unsigned ctime;

  unsigned profile_velocity_ = 1;
};

}  // namespace motor_client