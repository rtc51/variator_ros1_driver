#pragma once

// C++
#include <bitset>
#include <iostream>
#include <string>
#include <unordered_map>

// ROS
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

//
#include <variator_driver/SetLed.h>
#include <variator_driver/SetRatio.h>
#include <variator_driver/VariatorStates.h>

#include <battery/battery_imitation.hpp>
#include <led/led_imitation.hpp>
#include <motor/motor_client_imitation.hpp>
#include <variator/variator_imitation.hpp>

/**
 * @brief Типы контроллеров
 *
 */
enum class ControllerType { INFO, POSITION, VELOCITY, EFFORT };

struct Joint {
  std::string name;
  double position;
  double velocity;
  double effort;
  double position_command;
  double velocity_command;
  double effort_command;
};

class VariatorHardwareInterface : public hardware_interface::RobotHW {
 public:
  VariatorHardwareInterface(ros::NodeHandle& nh_);
  ~VariatorHardwareInterface() {};

  /**
   * @brief Read data from the robot hardware.
   * The read method is part of the control loop cycle
   */
  void read();
  /**
   * @brief Write commands to the robot hardware
   * The write method is part of the control loop cycle
   */
  void write(ros::Duration elapsed_time);

 private:
  /**
   * @brief Функция которую вызывает controller_manager
   * переключает росконтроллеры в соответствии с полученными данными
   * @param start_list - список контроллеров которые нужно запустить
   * @param stop_list - список контроллеров которые нужно остановить
   */
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list);

  ros::NodeHandle& nh;

 private:
  // hardware interfaces
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;

  // actuators
  motor_client::MotorClientImitation steer_left_actuator;
  motor_client::MotorClientImitation steer_right_actuator;

  motor_client::MotorClientImitation front_left_actuator;
  motor_client::MotorClientImitation front_right_actuator;
  motor_client::MotorClientImitation rear_right_actuator;
  motor_client::MotorClientImitation rear_left_actuator;

  // joints
  Joint steer_left_joint{"steer_left_joint"};
  Joint steer_right_joint{"steer_right_joint"};

  Joint front_left_joint{"front_left_joint"};
  Joint front_right_joint{"front_right_joint"};
  Joint rear_right_joint{"rear_right_joint"};
  Joint rear_left_joint{"rear_left_joint"};

  // register hardware interfaces
  void registerStateInterface(Joint& joint);
  void registerPositionInterface(Joint& joint);
  void registerVelocityInterface(Joint& joint);
  void registerEffortInterface(Joint& joint);

  // ROS Interface:

  // set variator ratio
  ros::ServiceServer set_ratio_srv;
  bool setRatioCallback(variator_driver::SetRatioRequest& req,
                        variator_driver::SetRatioResponse& res);
  variator_client::VariatorClientImitation variator;

  // set led
  ros::ServiceServer set_led_srv;
  bool setLedCallback(variator_driver::SetLedRequest& req,
                      variator_driver::SetLedResponse& res);
  led_client::LedClientImitation led;

  // state pub
  ros::Publisher variator_states_pub;
  battery_client::BatteryClientImitation battery;
};
