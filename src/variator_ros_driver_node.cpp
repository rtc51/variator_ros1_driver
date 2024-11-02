#include <controller_manager/controller_manager.h>

#include <variator_ros_driver.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "festo_driver");
  ros::NodeHandle nh;

  VariatorHardwareInterface robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Time ts = ros::Time::now();

  ros::Rate rate(10);

  while (ros::ok()) {
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
    robot.read();
    cm.update(ts, d);
    robot.write(d);

    rate.sleep();
  }

  spinner.stop();

  return 0;
}
