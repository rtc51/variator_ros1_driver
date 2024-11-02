#pragma once

// ROS
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <motor_imitator/motor_client_imitation.hpp>

// C++
#include <bitset>
#include <iostream>
#include <string>
#include <unordered_map>

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
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;

 private:
  motor_client::MotorClientImitation steer_left_actuator;
  motor_client::MotorClientImitation steer_right_actuator;

  motor_client::MotorClientImitation front_left_actuator;
  motor_client::MotorClientImitation front_right_actuator;
  motor_client::MotorClientImitation rear_right_actuator;
  motor_client::MotorClientImitation rear_left_actuator;

 private:
  Joint steer_left_joint;
  Joint steer_right_joint;

  Joint front_left_joint;
  Joint front_right_joint;
  Joint rear_right_joint;
  Joint rear_left_joint;

 private:
  void registerStateInterface(Joint& joint);
  void registerPositionInterface(Joint& joint);
  void registerVelocityInterface(Joint& joint);
  void registerEffortInterface(Joint& joint);
};
