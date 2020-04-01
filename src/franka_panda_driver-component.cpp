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

FrankaComponent::FrankaComponent(std::string const& name): TaskContext(name,PreOperational)
  , p_ip_address("172.16.0.2")
  , sending_setpoints(false)
  {

  /* Setting up Orocos component interface */
  //Adding ports
  /// Input
  this->addPort("control_joint_velocities",  control_joint_velocities).doc("Desired joint velocities [rad/s]");
  /// Output
  this->addPort("sensor_joint_angles",             sensor_joint_angles).doc("Current sensed joint angles [rad]");
  this->addPort("tool_external_wrench",          tool_external_wrench).doc("External wrench calculated from the measured data: three forces [N] and three moments [Nm]");
  // Properties
  this->addProperty("ip_address", p_ip_address).doc("ip_address of the robot's controller. Default: 172.16.0.2");

  //Operations
  this->addOperation("start_sending_setpoints",  &FrankaComponent::start_sending_setpoints, this, OwnThread).doc("Starts sending the setpoints of the joints to the robot");
  this->addOperation("get_joint_angles",  &FrankaComponent::get_joint_angles, this, OwnThread).doc("Get the current sensed joint angles [rad]. Useful to initialize your trajectory planner.");

  //Memory allocation for the size of the vectors done beforehand (real time)
  temporary_desired_vel.resize(NR_JOINT,0.0);
  temporary_actual_pos.resize(NR_JOINT,0.0);
  temporary_actual_wrench.resize(6,0.0);

  sensor_joint_angles.setDataSample( temporary_actual_pos );
  tool_external_wrench.setDataSample( temporary_actual_wrench );



  try {
    panda = std::make_unique<franka::Robot>(franka::Robot(p_ip_address));
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
      // m_joint_states.position[i]   = state.q[i];
      // m_joint_states.velocity[i] = state.dq[i];
      temporary_actual_pos[i] = state.q[i];
    }
    for(unsigned int i=0;i<6;++i) {
      temporary_actual_wrench[i] = state.K_F_ext_hat_K[i];
    }

    sensor_joint_angles.write(temporary_actual_pos);
    tool_external_wrench.write(temporary_actual_wrench);

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

void FrankaComponent::start_sending_setpoints(){
  //TODO: fill
}

std::vector<double> FrankaComponent::get_joint_angles(){
  temporary_robot_state = panda->readOnce();
  for(unsigned int i=0;i<NR_JOINT;++i) {
    temporary_actual_pos[i] = temporary_robot_state.q[i];
  }
  return temporary_actual_pos;
}

ORO_CREATE_COMPONENT(FrankaComponent)
