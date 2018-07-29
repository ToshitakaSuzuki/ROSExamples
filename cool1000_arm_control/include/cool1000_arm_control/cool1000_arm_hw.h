#ifndef _COOL1000_ARM_HW_H_
#define _COOL1000_ARM_HW_H_

#include <ros/ros.h>

// Message (std_msgs)
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std_msgs;

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits_interface.h>

#include <boost/thread.hpp>

#define JOINT_MAX (8)

namespace cool1000_arm_control
{
  class Cool1000ArmHW : public hardware_interface::RobotHW
  {
  public:
    Cool1000ArmHW();
    virtual ~Cool1000ArmHW();

    bool Initialize();

    ros::Time getTime() const
    {
      return ros::Time::now();
    }

    ros::Duration getPeriod() const
    {
      return ros::Duration(0.008);
    }

    void read(ros::Time, ros::Duration);
    void write(ros::Time, ros::Duration);

  private:
    //HRESULT ChangeModeWithClearError(int mode);
    //void Callback_ChangeMode(const Int32::ConstPtr& msg);

  private:
    hardware_interface::JointStateInterface m_JntStInterface;
    hardware_interface::PositionJointInterface m_PosJntInterface;
    double m_cmd[JOINT_MAX];
    double m_pos[JOINT_MAX];
    double m_vel[JOINT_MAX];
    double m_eff[JOINT_MAX];
    std::vector<double> m_joint;

    //ros::Publisher  m_pubRecvUserIO;
    //ros::Publisher  m_pubCurrent;

    boost::mutex m_mtxMode;
  };

}

#endif
