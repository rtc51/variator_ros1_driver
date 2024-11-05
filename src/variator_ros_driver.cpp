#include <variator_ros_driver.hpp>

VariatorHardwareInterface::VariatorHardwareInterface(ros::NodeHandle& nh_)
    : nh(nh_) {
  // states
  registerStateInterface(steer_left_joint);
  registerStateInterface(steer_right_joint);
  registerStateInterface(front_left_joint);
  registerStateInterface(front_right_joint);
  registerStateInterface(rear_right_joint);
  registerStateInterface(rear_left_joint);

  // position
  registerPositionInterface(steer_left_joint);
  registerPositionInterface(steer_right_joint);

  // velocity
  registerVelocityInterface(front_left_joint);
  registerVelocityInterface(front_right_joint);
  registerVelocityInterface(rear_right_joint);
  registerVelocityInterface(rear_left_joint);

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);
  registerInterface(&jnt_vel_interface);
  registerInterface(&jnt_eff_interface);

  set_ratio_srv = nh.advertiseService(
      "set_ratio", &VariatorHardwareInterface::setRatioCallback, this);

  set_led_srv = nh.advertiseService(
      "set_led", &VariatorHardwareInterface::setLedCallback, this);

  variator_states_pub =
      nh.advertise<variator_driver::VariatorStates>("variator_states", 1);
}

void VariatorHardwareInterface::write(ros::Duration elapsed_time) {
  steer_left_actuator.setPosition(steer_left_joint.position_command);
  steer_right_actuator.setPosition(steer_right_joint.position_command);

  front_left_actuator.setVelocity(front_left_joint.velocity_command);
  front_right_actuator.setVelocity(front_right_joint.velocity_command);
  rear_right_actuator.setVelocity(rear_right_joint.velocity_command);
  rear_left_actuator.setVelocity(rear_left_joint.velocity_command);
}

void VariatorHardwareInterface::read() {
  auto state_steer_left = steer_left_actuator.getState();
  steer_left_joint.position = state_steer_left.position;
  steer_left_joint.velocity = state_steer_left.velocity;
  steer_left_joint.effort = state_steer_left.effort;

  auto state_steer_right = steer_right_actuator.getState();
  steer_right_joint.position = state_steer_right.position;
  steer_right_joint.velocity = state_steer_right.velocity;
  steer_right_joint.effort = state_steer_right.effort;

  auto state_front_left = front_left_actuator.getState();
  front_left_joint.position = state_front_left.position;
  front_left_joint.velocity = state_front_left.velocity;
  front_left_joint.effort = state_front_left.effort;

  auto state_front_right = front_right_actuator.getState();
  front_right_joint.position = state_front_right.position;
  front_right_joint.velocity = state_front_right.velocity;
  front_right_joint.effort = state_front_right.effort;

  auto state_rear_right = rear_right_actuator.getState();
  rear_right_joint.position = state_rear_right.position;
  rear_right_joint.velocity = state_rear_right.velocity;
  rear_right_joint.effort = state_rear_right.effort;

  auto state_rear_left = rear_left_actuator.getState();
  rear_left_joint.position = state_rear_left.position;
  rear_left_joint.velocity = state_rear_left.velocity;
  rear_left_joint.effort = state_rear_left.effort;

  variator_driver::VariatorStates variator_states;
  variator_states.status = variator_driver::VariatorStates::OK;
  variator_states.battery_voltage = battery.getVoltage();
  variator_states.battery_charge = battery.getCharge();
  variator_states.led_state = led.getLedState();
  variator_states.variator_ratio = variator.getRatio();
  variator_states_pub.publish(variator_states);
}

void VariatorHardwareInterface::doSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& stop_list) {
  // TODO: implement (if needed)
}

void VariatorHardwareInterface::registerStateInterface(Joint& joint) {
  // clang-format off
  hardware_interface::JointStateHandle state_handle(
      joint.name,
      &joint.position,
      &joint.velocity,
      &joint.effort
  );
  // clang-format on
  jnt_state_interface.registerHandle(state_handle);
}

void VariatorHardwareInterface::registerPositionInterface(Joint& joint) {
  hardware_interface::JointHandle pos_handle(
      jnt_state_interface.getHandle(joint.name), &joint.position_command);

  jnt_pos_interface.registerHandle(pos_handle);
}

void VariatorHardwareInterface::registerVelocityInterface(Joint& joint) {
  hardware_interface::JointHandle vel_handle(
      jnt_state_interface.getHandle(joint.name), &joint.velocity_command);
  jnt_vel_interface.registerHandle(vel_handle);
}

void VariatorHardwareInterface::registerEffortInterface(Joint& joint) {
  hardware_interface::JointHandle eff_handle(
      jnt_state_interface.getHandle(joint.name), &joint.effort_command);
  jnt_eff_interface.registerHandle(eff_handle);
}

bool VariatorHardwareInterface::setRatioCallback(
    variator_driver::SetRatioRequest& req,
    variator_driver::SetRatioResponse& res) {
  try {
    variator.setRatio(req.ratio);
    res.success = true;
  } catch (const std::exception& e) {
    res.success = false;
    res.error_message = e.what();
  }
  return true;
}

bool VariatorHardwareInterface::setLedCallback(
    variator_driver::SetLedRequest& req, variator_driver::SetLedResponse& res) {
  try {
    led.setLedState(req.led_state);
    res.success = true;
  } catch (const std::exception& e) {
    res.success = false;
    res.error_message = e.what();
  }
  return true;
}