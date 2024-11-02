#pragma once

namespace motor_client {

enum class MotorType { RTCRM, FESTO, VECTOR, IMITATOR };

enum class MotorStatus { OK, WARNING, ERROR };

enum class ControlMode {
  STOPPED = 0,
  POSITION = 1,
  VELOCITY = 3,
  EFFORT = 4,
  HOMING = 6
};

struct MotorState {
  double position;
  double velocity;
  double effort;
};

class MotorClientInterface {
 public:
  virtual MotorState getState() = 0;
  virtual MotorStatus getStatus() = 0;
  virtual MotorType getType() = 0;
  virtual unsigned getFault() = 0;
  virtual bool readyState() = 0;

  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual void stop() = 0;
  virtual void reset() = 0;
  virtual void home(bool always = false) = 0;

  virtual void setPosition(double target, bool relative = false) = 0;
  virtual void setVelocity(double velocity) = 0;
  virtual void setEffort(double target) = 0;

  virtual void setProfileVelocity(double profile_velocity) = 0;
  virtual void setMaxPosition(double max_position) = 0;
  virtual void setMinPosition(double min_position) = 0;
  virtual void setProfileAcceleratoin(double profile_acceleration) = 0;
  virtual void setProfileDeceleratoin(double profile_deceleration) = 0;

  virtual void setMode(ControlMode mode) = 0;
  virtual ControlMode getMode() = 0;

  virtual ~MotorClientInterface() {}
};
}  // namespace motor_client
