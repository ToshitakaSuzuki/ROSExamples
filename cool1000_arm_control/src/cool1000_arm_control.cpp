#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "cool1000_arm_control/cool1000_arm_hw.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "arm_control");
  ros::NodeHandle nh;

  cool1000_arm_control::Cool1000ArmHW arm;
  arm.Initialize();
  controller_manager::ControllerManager cm(&arm, nh);
 
  ros::Rate rate(1.0 / arm.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  ros::Time start = arm.getTime();
  while(ros::ok())
  {
    ros::Time now = arm.getTime();
    ros::Duration dt = arm.getPeriod();
 
    arm.read(now, dt);
 
    cm.update(now, dt);
 
    ros::Duration diff = now - start;
    if(diff.toSec() > 5) {
      arm.write(now, dt);
    }
 
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
