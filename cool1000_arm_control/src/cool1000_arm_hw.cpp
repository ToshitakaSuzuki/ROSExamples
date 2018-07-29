#include <string.h>
#include <sstream>
#include "cool1000_arm_control/cool1000_arm_hw.h"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) / 180.0 * M_PI)

#define M2MM(x) ((x) * 1000.0)
#define MM2M(x) ((x) / 1000.0)

namespace cool1000_arm_control
{

  Cool1000ArmHW::Cool1000ArmHW()
  {
    memset(m_cmd, 0, sizeof(m_cmd));
    memset(m_pos, 0, sizeof(m_pos));
    memset(m_vel, 0, sizeof(m_vel));
    memset(m_eff, 0, sizeof(m_eff));
    m_joint.resize(JOINT_MAX);
  }

  Cool1000ArmHW::~Cool1000ArmHW()
  {

  }

  bool Cool1000ArmHW::Initialize()
  {
    ros::NodeHandle nh;

    for (int i = 0; i < 8; i++) {
      std::stringstream ss;
      ss << "joint" << i+1;

      hardware_interface::JointStateHandle state_handle(ss.str(),
          &m_pos[i], &m_vel[i], &m_eff[i]);
      m_JntStInterface.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(
          m_JntStInterface.getHandle(ss.str()), &m_cmd[i]);
      m_PosJntInterface.registerHandle(pos_handle);
    }

    registerInterface(&m_JntStInterface);
    registerInterface(&m_PosJntInterface);

    // Init Driver

//    m_subChangeMode = nh.subscribe<Int32>(
//        "ChangeMode", 1, &DensoRobotHW::Callback_ChangeMode, this);
//    m_pubCurMode = nh.advertise<Int32>("CurMode", 1);
    
    return true;
  }

//  HRESULT DensoRobotHW::ChangeModeWithClearError(int mode)
//  {
//    return hr;
//  }
//
//  void DensoRobotHW::Callback_ChangeMode(const Int32::ConstPtr& msg)
//  {
//    boost::mutex::scoped_lock lockMode(m_mtxMode);
//
//    ROS_INFO("Change to mode %d.", msg->data);
//    HRESULT hr = ChangeModeWithClearError(msg->data);
//    if(FAILED(hr)) {
//      ROS_ERROR("Failed to change mode. (%X)", hr);
//    }
//  }

  void Cool1000ArmHW::read(ros::Time time, ros::Duration period)
  {
    //boost::mutex::scoped_lock lockMode(m_mtxMode);

    // Read Encoder data  
    
    // Convert pulse to radian  
    for(int i = 0; i < 8; i++){
//      m_pos[i] = 0;
    }

  }

  void Cool1000ArmHW::write(ros::Time time, ros::Duration period)
  {
    //boost::mutex::scoped_lock lockMode(m_mtxMode);
    ROS_INFO_STREAM("Gripper Command: " << m_cmd[7]);
    // Convert rad to pulse

    // Call SRV_UPDATE

  }

//  void DensoRobotHW::Callback_MiniIO(const Int32::ConstPtr& msg)
//  {
//    m_rob->put_MiniIO(msg->data);
//  }
//
//  void DensoRobotHW::Callback_HandIO(const Int32::ConstPtr& msg)
//  {
//    m_rob->put_HandIO(msg->data);
//  }
//  
//  void DensoRobotHW::Callback_SendUserIO(const UserIO::ConstPtr& msg)
//  {
//    m_rob->put_SendUserIO(*msg.get());
//  }
//
//  void DensoRobotHW::Callback_RecvUserIO(const UserIO::ConstPtr& msg)
//  {
//    m_rob->put_RecvUserIO(*msg.get());
//  }

}
