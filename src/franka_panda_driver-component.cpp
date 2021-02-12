#include "franka_panda_driver-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

#define NR_JOINT 7
#define LOG_SIZE 5
//Max apperture of the gripper, determined with homing franka::Gripper::homing
#define MAX_APPERTURE 0.079

void setDefaultBehavior(FrankaComponent::PandaPtr& panda) {
  panda->setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  // panda->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  // panda->setCartesianImpedance({{3000, 3000, 500, 300, 300, 300}});
}

FrankaComponent::FrankaComponent(std::string const& name): TaskContext(name,PreOperational)
  , p_ip_address("172.16.0.2")
  , event_stop_loop("stop_loop")
  , control_loop_running(false)
  , cutoff_frequency(100)
  , rate_limiters(true)
  , gripper_connected(true)
  , gripper_constructed(false)
  , cartesian_impedance(6)
  , joint_impedance(NR_JOINT)
  , impedance_mode("joint")
  , lower_torque_thresholds_acceleration({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}})
  , upper_torque_thresholds_acceleration({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}})
  , lower_torque_thresholds_nominal({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}})
  , upper_torque_thresholds_nominal({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}})
  , lower_force_thresholds_acceleration({{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}})
  , upper_force_thresholds_acceleration({{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}})
  , lower_force_thresholds_nominal({{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}})
  , upper_force_thresholds_nominal({{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}})
  , controller_mode(franka::ControllerMode::kJointImpedance)
  {
    Logger::In in(this->getName().data());
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
  this->addProperty("joint_impedance", joint_impedance).doc("Joint impedance: 7 values, one for each joint");
  this->addProperty("cutoff_frequency", cutoff_frequency).doc("cutoff_frequency [Hz] for built-in filter of frankalib to avoid packet looses due to communication issues. To turn it off set it to 1000 Hz");
  this->addProperty("rate_limiters", rate_limiters).doc("libfranka built-in rate limiters. For velocity control it will limit the acceleration and jerk, while for torque control, it will limit the torque rate");
  this->addProperty("gripper_connected", gripper_connected).doc("Use it to indicate that the default gripper is not connected");
  this->addProperty("impedance_mode", impedance_mode).doc("Impedance Mode. Set to: 'joint' (default) or 'cartesian'");
  this->addProperty("event_stop_loop", event_stop_loop).doc("When receiving this string through the 'events_port' port, the control loop will stop.");

  this->addProperty("lower_torque_thresholds_acceleration", lower_torque_thresholds_acceleration).doc("Contact torque thresholds during acceleration/deceleration for each joint in [Nm]. Set before calling the configure hook");
  this->addProperty("upper_torque_thresholds_acceleration", upper_torque_thresholds_acceleration).doc("Collision torque thresholds during acceleration/deceleration for each joint in [Nm]. Set before calling the configure hook");
  this->addProperty("lower_torque_thresholds_nominal", lower_torque_thresholds_nominal).doc("Contact torque thresholds for each joint in [Nm]. Set before calling the configure hook");
  this->addProperty("upper_torque_thresholds_nominal", upper_torque_thresholds_nominal).doc("Collision torque thresholds for each joint in [Nm]. Set before calling the configure hook");
  this->addProperty("lower_force_thresholds_acceleration", lower_force_thresholds_acceleration).doc("Contact wrench thresholds during acceleration/deceleration in [N]. Set before calling the configure hook");
  this->addProperty("upper_force_thresholds_acceleration", upper_force_thresholds_acceleration).doc("Collision wrench thresholds during acceleration/deceleration in [N]. Set before calling the configure hook");
  this->addProperty("lower_force_thresholds_nominal", lower_force_thresholds_nominal).doc("Contact wrench thresholds in [N]. Set before calling the configure hook");
  this->addProperty("upper_force_thresholds_nominal", upper_force_thresholds_nominal).doc("Collision wrench thresholds in [N]. Set before calling the configure hook");

  //Operations
  this->addOperation("start_sending_setpoints",  &FrankaComponent::start_sending_setpoints, this, OwnThread).doc("Starts sending the setpoints of the joints to the robot");
  this->addOperation("get_joint_angles",  &FrankaComponent::get_joint_angles, this, OwnThread).doc("Get the current sensed joint angles [rad]. Useful to initialize your trajectory planner.");
  this->addOperation("stream_sensor_data",  &FrankaComponent::stream_sensor_data, this, OwnThread).doc("Streams sensor data. Meant to be used when there is not a control loop running.");
  this->addOperation("admittance",  &FrankaComponent::admittance, this, OwnThread).doc("Start admittance mode: command torques to zero (gravity compensated)");
  this->addOperation("low_level_velocity",  &FrankaComponent::low_level_velocity, this, OwnThread).doc("Start sending velocity setpoints at 1kHz");
  this->addOperation("stop_control_loop",  &FrankaComponent::stop_control_loop, this, OwnThread).doc("Stop the current control loop (at 1kHz)");
  this->addOperation("error_recovery",  &FrankaComponent::error_recovery, this, OwnThread).doc("Automatic error recovery (e.g. after the robot hits joint limits or collides)");
  this->addOperation("gripper_grasp",  &FrankaComponent::gripper_grasp, this, OwnThread).doc("Grasp object with the franka gripper/hand. Arguments: grasping_width[m], velocity [m/s] and force [N]");
  this->addOperation("gripper_grasp_with_check",  &FrankaComponent::gripper_grasp_with_check, this, OwnThread).doc("Grasp object with the franka gripper/hand. Arguments: grasping_width[m], velocity [m/s] and force [N],epsilon_inner [m], epsilon_outer [m] ");
  this->addOperation("gripper_change_apperture",  &FrankaComponent::gripper_change_apperture, this, OwnThread).doc("Change gripper's apperture (without applying force). Arguments: grasping_width[m], velocity [m/s]");
  this->addOperation("gripper_homing",  &FrankaComponent::gripper_homing, this, OwnThread).doc("Gripper homing (calibration). Warning: the gripper move the whole range");

  //Memory allocation for the size of the vectors done beforehand (real time)
  temporary_desired_vel.resize(NR_JOINT,0.0);
  temporary_actual_pos.resize(NR_JOINT,0.0);
  temporary_actual_wrench.resize(6,0.0);
  cartesian_impedance.resize(6,0.0);
  joint_impedance.resize(7,0.0);
  cartesian_impedance = {3000, 3000, 1500, 100, 100, 100};
  joint_impedance = {3000, 3000, 3000, 2500, 2500, 2000, 2000};


  sensor_joint_angles.setDataSample( temporary_actual_pos );
  tool_external_wrench.setDataSample( temporary_actual_wrench );



  try {
    panda = std::make_unique<franka::Robot>(franka::Robot(p_ip_address,franka::RealtimeConfig::kEnforce,LOG_SIZE)); //change kEnforce to kIgnore to stop enforcing realtime mode for a control loop thread.
    // gripper = nullptr;
    // gripper = std::make_unique<franka::Gripper>(franka::Gripper(p_ip_address));
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
  // std::cout << "Franka_panda_driver constructed !" <<std::endl;
  log( Info ) << "Franka_panda_driver constructed !" << endlog();
}

bool FrankaComponent::configureHook(){
  if (gripper_connected) {
    try {
      gripper = std::make_unique<franka::Gripper>(franka::Gripper(p_ip_address));
      // gripper = new franka::Gripper(franka::Gripper(p_ip_address));
    } catch (const franka::Exception& e) {
      log( Error ) << e.what() << endlog();
      return false;
    }
    log( Info ) << "Gripper communication object constructed!" << endlog();
    gripper_constructed = true;
  }

  if (lower_torque_thresholds_acceleration.size() != 7){
    log( Error ) << "The size of lower_torque_thresholds_acceleration must be 7" << endlog();
    setDefaultBehavior(panda);
    return false;
  }
  else if (upper_torque_thresholds_acceleration.size() != 7){
    log( Error ) << "The size of upper_torque_thresholds_acceleration must be 7" << endlog();
    setDefaultBehavior(panda);
    return false;
  }
  else if (lower_torque_thresholds_nominal.size() != 7){
    log( Error ) << "The size of lower_torque_thresholds_nominal must be 7" << endlog();
    setDefaultBehavior(panda);
    return false;
  }
  else if (upper_torque_thresholds_nominal.size() != 7){
    log( Error ) << "The size of upper_torque_thresholds_nominal must be 7" << endlog();
    setDefaultBehavior(panda);
    return false;
  }
  else if (lower_force_thresholds_acceleration.size() != 6){
    log( Error ) << "The size of lower_force_thresholds_acceleration must be 6" << endlog();
    setDefaultBehavior(panda);
    return false;
  }
  else if (upper_force_thresholds_acceleration.size() != 6){
    log( Error ) << "The size of upper_force_thresholds_acceleration must be 6" << endlog();
    setDefaultBehavior(panda);
    return false;
  }
  else if (lower_force_thresholds_nominal.size() != 6){
    log( Error ) << "The size of lower_force_thresholds_nominal must be 6" << endlog();
    setDefaultBehavior(panda);
    return false;
  }
  else if (upper_force_thresholds_nominal.size() != 6){
    log( Error ) << "The size of upper_force_thresholds_nominal must be 6" << endlog();
    setDefaultBehavior(panda);
    return false;
  }
  // setDefaultBehavior(panda);
  std::array<double,7> array_lower_torque_thresholds_acceleration;
  std::array<double,7> array_upper_torque_thresholds_acceleration;
  std::array<double,7> array_lower_torque_thresholds_nominal;
  std::array<double,7> array_upper_torque_thresholds_nominal;
  std::array<double,6> array_lower_force_thresholds_acceleration;
  std::array<double,6> array_upper_force_thresholds_acceleration;
  std::array<double,6> array_lower_force_thresholds_nominal;
  std::array<double,6> array_upper_force_thresholds_nominal;

  std::copy_n(lower_torque_thresholds_acceleration.begin(), 7, array_lower_torque_thresholds_acceleration.begin());
  std::copy_n(upper_torque_thresholds_acceleration.begin(), 7, array_upper_torque_thresholds_acceleration.begin());
  std::copy_n(lower_torque_thresholds_nominal.begin(), 7, array_lower_torque_thresholds_nominal.begin());
  std::copy_n(upper_torque_thresholds_nominal.begin(), 7, array_upper_torque_thresholds_nominal.begin());
  std::copy_n(lower_force_thresholds_acceleration.begin(), 6, array_lower_force_thresholds_acceleration.begin());
  std::copy_n(upper_force_thresholds_acceleration.begin(), 6, array_upper_force_thresholds_acceleration.begin());
  std::copy_n(lower_force_thresholds_nominal.begin(), 6, array_lower_force_thresholds_nominal.begin());
  std::copy_n(upper_force_thresholds_nominal.begin(), 6, array_upper_force_thresholds_nominal.begin());

  panda->setCollisionBehavior(
        array_lower_torque_thresholds_acceleration, array_upper_torque_thresholds_acceleration,
        array_lower_torque_thresholds_nominal, array_upper_torque_thresholds_nominal,
        array_lower_force_thresholds_acceleration, array_upper_force_thresholds_acceleration,
        array_lower_force_thresholds_nominal, array_upper_force_thresholds_nominal);
  log( Info ) << "Franka_panda_driver configured !" <<endlog();
  return true;
}

bool FrankaComponent::startHook(){

  log( Info )<< "Franka_panda_driver started !" << endlog();
  low_level_velocity();
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

    if(impedance_mode == "cartesian") {
      panda->setCartesianImpedance({{cartesian_impedance[0], cartesian_impedance[1], cartesian_impedance[2], cartesian_impedance[3], cartesian_impedance[4], cartesian_impedance[5]}});
      controller_mode = franka::ControllerMode::kCartesianImpedance;
      log( Info ) << "Set to Cartesian Impedance mode" << endlog();
    }
    else{
      panda->setJointImpedance({{joint_impedance[0], joint_impedance[1], joint_impedance[2], joint_impedance[3], joint_impedance[4], joint_impedance[5], joint_impedance[6]}});
      controller_mode = franka::ControllerMode::kJointImpedance;
      log( Info ) << "Set to Joint Impedance mode" << endlog();
    }

    // panda->setCartesianImpedance({{3000, 3000, 1500, 100, 100, 100}});
    // panda->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
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
            temporary_actual_wrench[i] = -state.K_F_ext_hat_K[i]; //The minus is to be able to use it in admittance (action-reaction)
          }

          sensor_joint_angles.write(temporary_actual_pos);
          tool_external_wrench.write(temporary_actual_wrench);


          if (events_port.read(events) == NewData && events == event_stop_loop) {
            log( Info ) << "Event '" <<event_stop_loop <<"' received through events_port. Control loop stopped" << endlog();
            return franka::MotionFinished(velocities);
          }
          // if (!control_loop_running) {
          //   std::cout << "Finished motion" << std::endl;
          //   return franka::MotionFinished(velocities);
          // }
          return velocities;
        },controller_mode, rate_limiters, cutoff_frequency);
  }
  catch (const franka::ControlException& e) {
    log( Error ) << e.what() << endlog();
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
    log( Error ) << e.what() << endlog();
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
    log( Error ) << e.what() << endlog();
  }
}

std::vector<double> FrankaComponent::get_joint_angles(){
  temporary_robot_state = panda->readOnce();
  for(unsigned int i=0;i<NR_JOINT;++i) {
    temporary_actual_pos[i] = temporary_robot_state.q[i];
  }
  return temporary_actual_pos;
}

void FrankaComponent::stream_sensor_data(){
  panda->read([&](const franka::RobotState& state) {
  // Printing to std::cout adds a delay. This is acceptable for a read loop such as this,
  // but should not be done in a control loop.
  for(unsigned int i=0;i<NR_JOINT;++i) {
    temporary_actual_pos[i] = state.q[i];
  }
  for(unsigned int i=0;i<6;++i) {
    temporary_actual_wrench[i] = state.K_F_ext_hat_K[i];
  }
  sensor_joint_angles.write(temporary_actual_pos);
  tool_external_wrench.write(temporary_actual_wrench);
  if (events_port.read(events) == NewData && events == event_stop_loop) {
    log( Info ) << "Event '" <<event_stop_loop <<"' received through events_port. Control loop stopped" << endlog();
    return false;
  }
  else{
    return true;
  }
});
  // temporary_robot_state = panda->readOnce();
  // for(unsigned int i=0;i<NR_JOINT;++i) {
  //   temporary_actual_pos[i] = temporary_robot_state.q[i];
  // }
  // return temporary_actual_pos;
}

bool FrankaComponent::gripper_change_apperture(double grasping_width, double vel){
  if (gripper_constructed){
    try {
      // gripper->stop();
      // gripper->homing();
      gripper->move(grasping_width,vel);

    } catch (franka::Exception const& e) {
      log( Error ) << e.what() << endlog();
      return false;
    }
    return true;
  }
  else{
    log( Error ) << "The gripper was not configured" << endlog();
    return false;
  }
}
double FrankaComponent::gripper_homing(){
  if (gripper_constructed){
    try {
      gripper->homing();
      franka::GripperState gripper_state = gripper->readOnce();
      return gripper_state.max_width;
    } catch (franka::Exception const& e) {
      log( Error ) << e.what() << endlog();
      return -1;
    }
  }
  else{
    log( Error ) << "The gripper was not configured" << endlog();
    return -1;
  }
}
bool FrankaComponent::gripper_grasp(double grasping_width, double vel, double force){

  if (gripper_constructed){
    try {

      // double grasping_width = std::stod(argv[3]);

      // Do a homing in order to estimate the maximum grasping width with the current fingers.
      // gripper->homing();
      gripper->stop();
      // Check for the maximum grasping width.
      franka::GripperState gripper_state = gripper->readOnce();
      if (MAX_APPERTURE < grasping_width) {
        log( Error ) << "Object is too large for the current fingers on the gripper." << endlog();
        return false;
      }
      if (!gripper->grasp(grasping_width, vel, force,0.005, 0.005)) {
        log( Warning ) << "Failed to grasp object." << endlog();
        return false;
      }
    }
    catch (franka::Exception const& e) {
      log( Error ) << e.what() << endlog();
      return false;
    }
    return true;
  }
  else{
    log( Error ) << "The gripper was not configured" << endlog();
    return false;
  }


}
bool FrankaComponent::gripper_grasp_with_check(double grasping_width, double vel, double force, double epsilon_inner, double epsilon_outer){
  if (gripper_constructed){
    try {

      gripper->stop();
      // Check for the maximum grasping width.
      franka::GripperState gripper_state = gripper->readOnce();

      if (MAX_APPERTURE < grasping_width) {
        log( Error ) << "Object is too large for the current fingers on the gripper." << endlog();
        return false;
      }
      if (!gripper->grasp(grasping_width, vel, force, epsilon_inner,epsilon_outer)) {
        log( Warning ) << "Failed to grasp object." << endlog();
        return false;
      }
    } catch (franka::Exception const& e) {
      log( Error ) << e.what() << endlog();
      return false;
    }
    return true;
  }
  else{
    log( Error ) << "The gripper was not configured" << endlog();
    return false;
  }


}

// void print_vec(std::vector<double>  &input){
// 	for (unsigned int i = 0; i < input.size(); i++) {
// 		std::cout << input[i] << ' ';
// 	}
// }

ORO_CREATE_COMPONENT(FrankaComponent)
