#include "franka_panda_driver-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

#define NR_JOINT 7

void setDefaultBehavior(FrankaComponent::PandaPtr& panda) {
  panda->setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  panda->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  panda->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

FrankaComponent::FrankaComponent(std::string const& name) 
  : TaskContext(name)
  , m_q_actual(NR_JOINT)
  , m_q_des(NR_JOINT)
  , m_t_actual(NR_JOINT)
  , m_qdot_actual(NR_JOINT)
  , p_baseframe("base_link")
  , p_ip_address("172.16.0.2")
  {
    
  /* Setting up Orocos component interface */
  //Adding ports
  /// Input
  this->addPort("event_in",              port_ein).doc("Events IN - eg supervisor");
  this->addPort("q_desired",             port_qdes).doc("desired joint position [rad]");
  this->addPort("JointPositionCommand",  port_joint_pos_command).doc("desired joint positions [rad]");
  this->addPort("JointVelocityCommand",  port_joint_vel_command).doc("desired joint velocities [rad/s]");
  /// Output
  this->addPort("event_out",             port_eout).doc("Events OUT - eg faults to supervisor");
  this->addPort("q_actual",              port_q_actual).doc("current joint positions [rad]");
  this->addPort("qdot_actual",           port_qdot_actual).doc("current joint velocities [rad/s]");
  this->addPort("JointPositionMeasured", port_joint_pos_msr).doc("current joint positions [rad]");
  this->addPort("JointExternalEffort",   port_joint_ext_jnt).doc("external torque on joints [Nm]");
  this->addPort("joint_states",          port_joint_state).doc("joint_states (ROS)");
  // Properties
  this->addProperty("baseframe",  p_baseframe).doc("Frame name of the robot base");
  this->addProperty("ip_address", p_ip_address).doc("Frame name of the robot base");
  
  //
  m_t_ext.names.resize(NR_JOINT);
  m_t_ext.efforts.assign(NR_JOINT,0);

  m_joint_states.name.resize(NR_JOINT);
  m_joint_states.position.resize(NR_JOINT);
  m_joint_states.effort.resize(NR_JOINT);
  m_joint_states.header.frame_id = p_baseframe;

  for (unsigned int i=0; i<NR_JOINT; ++i) {
    std::ostringstream ss;
    ss << "arm_" << i+1 << "_joint";
    m_joint_states.name[i] = ss.str();
    m_t_ext.names[i]       = ss.str();
  }
  port_joint_state.setDataSample(m_joint_states);

  
  try {
    panda = std::make_unique<franka::Robot>(franka::Robot("192.168.0.1"));
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
  std::cout << "Franka_panda_driver constructed !" <<std::endl;
}

bool FrankaComponent::configureHook(){
  setDefaultBehavior(panda);
  std::cout << "Franka_panda_driver configured !" <<std::endl;
  return true;
}

bool FrankaComponent::startHook(){

  std::cout << "Franka_panda_driver started !" <<std::endl;
  panda->control(
  [&  ](const franka::RobotState& state, franka::Duration period) -> franka::JointVelocities {
    
    for(unsigned int i=0;i<NR_JOINT;++i) {
      m_joint_states.position[i]   = state.q[i];
      m_joint_states.velocity[i] = state.dq[i];
    }
    
    port_joint_state.write(m_joint_states);
    
    franka::JointVelocities velocities = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
    return velocities;
  });
  return true;
}

void FrankaComponent::updateHook(){
//   std::cout << "Franka_panda_driver executes updateHook !" <<std::endl;
}

void FrankaComponent::stopHook() {
  std::cout << "Franka_panda_driver executes stopping !" <<std::endl;
}

void FrankaComponent::cleanupHook() {
  std::cout << "Franka_panda_driver cleaning up !" <<std::endl;
}

ORO_CREATE_COMPONENT(FrankaComponent)
