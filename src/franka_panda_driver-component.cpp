#include "franka_panda_driver-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

#define NR_JOINT 7
#define LOG_SIZE 5

void setDefaultBehavior(FrankaComponent::PandaPtr& panda) {
  panda->setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  // panda->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  panda->setCartesianImpedance({{3000, 3000, 500, 300, 300, 300}});
}

FrankaComponent::FrankaComponent(std::string const& name): TaskContext(name,PreOperational)
  , p_ip_address("172.16.0.2")
  , control_loop_running(false)
  , cutoff_frequency(100)
  , rate_limiters(true)
  {

  /* Setting up Orocos component interface */
  //Adding ports
  /// Input
  this->addPort("control_joint_velocities",  control_joint_velocities).doc("Desired joint velocities [rad/s]");
  /// Output
  this->addPort("sensor_joint_angles",             sensor_joint_angles).doc("Current sensed joint angles [rad]");
  this->addPort("tool_external_wrench",          tool_external_wrench).doc("External wrench calculated from the measured data: three forces [N] and three moments [Nm]");
  this->addPort("events_port",          events_port).doc("eTaSL Event Port (temporary solution)");
  // Properties
  this->addProperty("ip_address", p_ip_address).doc("ip_address of the robot's controller. Default: 172.16.0.2");
  this->addProperty("cartesian_impedance", cartesian_impedance).doc("Cartesian impedance (3 for forces and 3 for torques)");
  this->addProperty("cutoff_frequency", cutoff_frequency).doc("cutoff_frequency [Hz] for built-in filter of frankalib to avoid packet looses due to communication issues. To turn it off set it to 1000 Hz");
  this->addProperty("rate_limiters", rate_limiters).doc("libfranka built-in rate limiters. For velocity control it will limit the acceleration and jerk, while for torque control, it will limit the torque rate");

  //Operations
  this->addOperation("start_sending_setpoints",  &FrankaComponent::start_sending_setpoints, this, OwnThread).doc("Starts sending the setpoints of the joints to the robot");
  this->addOperation("get_joint_angles",  &FrankaComponent::get_joint_angles, this, OwnThread).doc("Get the current sensed joint angles [rad]. Useful to initialize your trajectory planner.");
  this->addOperation("admittance",  &FrankaComponent::admittance, this, OwnThread).doc("Start admittance mode: command torques to zero (gravity compensated)");
  this->addOperation("low_level_velocity",  &FrankaComponent::low_level_velocity, this, OwnThread).doc("Start sending velocity setpoints at 1kHz");
  this->addOperation("stop_control_loop",  &FrankaComponent::stop_control_loop, this, OwnThread).doc("Stop the current control loop (at 1kHz)");
  this->addOperation("error_recovery",  &FrankaComponent::error_recovery, this, OwnThread).doc("Automatic error recovery (e.g. after the robot hits joint limits or collides)");

  //Memory allocation for the size of the vectors done beforehand (real time)
  temporary_desired_vel.resize(NR_JOINT,0.0);
  temporary_actual_pos.resize(NR_JOINT,0.0);
  temporary_actual_wrench.resize(6,0.0);
  cartesian_impedance.resize(6,0.0);
  // cartesian_impedance = {{3000, 3000, 1500, 100, 100, 100}}
  cartesian_impedance[0] = 3000;
  cartesian_impedance[1] = 3000;
  cartesian_impedance[2] = 1500;
  cartesian_impedance[3] = 100;
  cartesian_impedance[4] = 100;
  cartesian_impedance[5] = 100;

  sensor_joint_angles.setDataSample( temporary_actual_pos );
  tool_external_wrench.setDataSample( temporary_actual_wrench );



  try {
    panda = std::make_unique<franka::Robot>(franka::Robot(p_ip_address,franka::RealtimeConfig::kEnforce,LOG_SIZE)); //change kEnforce to kIgnore to stop enforcing realtime mode for a control loop thread.
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
  // panda->control(
  // [&  ](const franka::RobotState& state, franka::Duration period) -> franka::JointVelocities {
  //
  //   for(unsigned int i=0;i<NR_JOINT;++i) {
  //     // m_joint_states.position[i]   = state.q[i];
  //     // m_joint_states.velocity[i] = state.dq[i];
  //     temporary_actual_pos[i] = state.q[i];
  //   }
  //   // for(unsigned int i=0;i<6;++i) {
  //   //   temporary_actual_wrench[i] = state.K_F_ext_hat_K[i];
  //   // }
  //
  //   sensor_joint_angles.write(temporary_actual_pos);
  //   // tool_external_wrench.write(temporary_actual_wrench);
  //
  //   franka::JointVelocities velocities = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
  //   return velocities;
  // });
  // return true;
}

void FrankaComponent::updateHook(){
//   std::cout << "Franka_panda_driver executes updateHook !" <<std::endl;
// TODO: Stop the activity whenever this is called (franka takes care of the 1kHz loop)
}

void FrankaComponent::stopHook() {
  std::cout << "Franka_panda_driver executes stopping !" <<std::endl;
}

void FrankaComponent::cleanupHook() {
  std::cout << "Franka_panda_driver cleaning up !" <<std::endl;
}

void FrankaComponent::start_sending_setpoints(){
  //TODO: Delete this function
}

void FrankaComponent::stop_control_loop(){
  control_loop_running = false;
  //TODO: fill
}
void FrankaComponent::error_recovery(){
  panda->automaticErrorRecovery();
}


void FrankaComponent::low_level_velocity(){
  control_loop_running = true;
  try {

    panda->setCartesianImpedance({{cartesian_impedance[0], cartesian_impedance[1], cartesian_impedance[2], cartesian_impedance[3], cartesian_impedance[4], cartesian_impedance[5]}});
    // panda->setCartesianImpedance({{3000, 3000, 1500, 100, 100, 100}});
    franka::JointVelocities velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    panda->control(
        [&](const franka::RobotState& state, franka::Duration period) -> franka::JointVelocities {
          // std::fill(temporary_actual_pos.begin(), temporary_actual_pos.end(), 0.0); //Assigns zero values to the vector, just in case to prevent the posibility of observing old values. Can be deleted to improve a bit the speed.
          if ( control_joint_velocities.read(temporary_desired_vel) != NoData ){
            velocities = {{temporary_desired_vel[0], temporary_desired_vel[1], temporary_desired_vel[2], temporary_desired_vel[3], temporary_desired_vel[4], temporary_desired_vel[5], temporary_desired_vel[6]}};
            // std::cout << temporary_desired_vel[6] << std::endl;
          }
          else{
            velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
          }
          for(unsigned int i=0;i<NR_JOINT;++i) {
            temporary_actual_pos[i] = state.q[i];
          }
          for(unsigned int i=0;i<6;++i) {
            temporary_actual_wrench[i] = state.K_F_ext_hat_K[i];
          }

          sensor_joint_angles.write(temporary_actual_pos);
          tool_external_wrench.write(temporary_actual_wrench);


          if (events_port.read(events) == NewData) {
            return franka::MotionFinished(velocities);
            std::cout << "eTasL Event received:" << std::endl;
            std::cout << events << std::endl;
            std::cout << "Finished motion" << std::endl;
          }
          // if (!control_loop_running) {
          //   std::cout << "Finished motion" << std::endl;
          //   return franka::MotionFinished(velocities);
          // }
          return velocities;
        },franka::ControllerMode::kCartesianImpedance, rate_limiters, cutoff_frequency);
  }
  catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    std::vector< franka::Record > log = e.log;
    std::cout << "Last joint positions before the exception was captured" << std::endl;
    for (unsigned int i = 0; i < log.size(); i++) {
      log_q = log[i].state.q;
      for (unsigned int n = 0; n < log_q.size(); n++) {
        std::cout << log_q[n] << ' ';
      }
      std::cout << "  " << std::endl;
    }
    std::cout << "Last wrenches before the exception was captured" << std::endl;
    for (unsigned int i = 0; i < log.size(); i++) {
      log_wrench = log[i].state.K_F_ext_hat_K;
      for (unsigned int n = 0; n < log_wrench.size(); n++) {
        std::cout << log_wrench[n] << ' ';
      }
      std::cout << "  " << std::endl;
    }

    // panda->automaticErrorRecovery();
  }
   catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
}


void FrankaComponent::admittance(){
  try {
    panda->control(
      [&](const franka::RobotState& state, franka::Duration) -> franka::Torques {
        for(unsigned int i=0;i<NR_JOINT;++i) {
          temporary_actual_pos[i] = state.q[i];
        }
        for(unsigned int i=0;i<6;++i) {
          temporary_actual_wrench[i] = state.K_F_ext_hat_K[i];
        }
        sensor_joint_angles.write(temporary_actual_pos);
        tool_external_wrench.write(temporary_actual_wrench);
        return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
}

std::vector<double> FrankaComponent::get_joint_angles(){
  temporary_robot_state = panda->readOnce();
  for(unsigned int i=0;i<NR_JOINT;++i) {
    temporary_actual_pos[i] = temporary_robot_state.q[i];
  }
  return temporary_actual_pos;
}

// void print_vec(std::vector<double>  &input){
// 	for (unsigned int i = 0; i < input.size(); i++) {
// 		std::cout << input[i] << ' ';
// 	}
// }

ORO_CREATE_COMPONENT(FrankaComponent)
