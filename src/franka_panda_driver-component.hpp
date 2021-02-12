#ifndef OROCOS_FRANKA_PANDA_DRIVER_COMPONENT_HPP
#define OROCOS_FRANKA_PANDA_DRIVER_COMPONENT_HPP

#include <rtt/RTT.hpp>

// ros_rttcom
#include <sensor_msgs/JointState.h>
#include <motion_control_msgs/JointAccelerations.h>
#include <motion_control_msgs/JointPositions.h>
#include <motion_control_msgs/JointEfforts.h>
#include <motion_control_msgs/JointVelocities.h>


#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <memory>

using namespace std;
using namespace RTT;

class FrankaComponent : public RTT::TaskContext{
  public:
    typedef std::unique_ptr<franka::Robot> PandaPtr;
    typedef std::unique_ptr<franka::Gripper> GripperPtr;

    FrankaComponent(string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void start_sending_setpoints();
    void admittance();
    void stream_sensor_data();
    void low_level_velocity();
    void stop_control_loop();
    void error_recovery();
    std::vector<double> get_joint_angles();
    bool gripper_change_apperture(double grasping_width, double vel);
    bool gripper_grasp(double grasping_width,double vel, double force);
    bool gripper_grasp_with_check(double grasping_width, double vel, double force, double epsilon_inner, double epsilon_outer);
    double gripper_homing();

  private:
    PandaPtr panda;
    GripperPtr gripper;
    // franka::Gripper gripper;
    bool control_loop_running;

    // Internal, mem alloc
    // motion_control_msgs::JointPositions  m_joint_pos_command;
    // motion_control_msgs::JointVelocities m_joint_vel_command;
    // motion_control_msgs::JointEfforts    m_joint_effort_command;
    // motion_control_msgs::JointEfforts    m_t_ext;
    // sensor_msgs::JointState              m_joint_states;
    // vector<double>                       m_q_des;
    // vector<double>                       m_q_actual;
    // vector<double>                       m_t_actual;
    // vector<double>                       m_qdot_actual;



    // Properties
    // std::string p_baseframe;
    // std::string p_ip_address;
    //
    // // Port Interface
    // InputPort<string>                               port_ein;
    // InputPort<vector<double> >                      port_qdes;
    // InputPort<motion_control_msgs::JointPositions>  port_joint_pos_command;
    // InputPort<motion_control_msgs::JointVelocities> port_joint_vel_command;
    // InputPort<motion_control_msgs::JointEfforts>    port_joint_effort_command;
    // //
    // OutputPort<string>                              port_eout;
    // OutputPort<vector<double> >                     port_q_actual;
    // OutputPort<vector<double> >                     port_t_actual;
    // OutputPort<vector<double> >                     port_qdot_actual;
    // OutputPort<motion_control_msgs::JointPositions> port_joint_pos_msr;
    // OutputPort<motion_control_msgs::JointEfforts>   port_joint_ext_jnt;
    // OutputPort<sensor_msgs::JointState>             port_joint_state;

    // Internal, mem alloc
    std::vector<double>                       temporary_desired_vel;
    std::vector<double>                       temporary_actual_pos;
    std::vector<double>                       temporary_actual_wrench;
    franka::RobotState                        temporary_robot_state;
    std::string                               events;
    franka::ControllerMode                    controller_mode;

    std::array<double, 7>                     log_q;
    std::array<double, 6>                     log_wrench;

    // Properties
    std::string                               p_ip_address;
    std::string                               event_stop_loop;
    std::vector<double>                       cartesian_impedance;
    std::vector<double>                       joint_impedance;
    double                                    cutoff_frequency;
    bool                                      rate_limiters;
    bool                                      gripper_connected;
    bool                                      gripper_constructed;
    std::string                               impedance_mode;

    std::vector<double>                       lower_torque_thresholds_acceleration;
    std::vector<double>                       upper_torque_thresholds_acceleration;
    std::vector<double>                       lower_torque_thresholds_nominal;
    std::vector<double>                       upper_torque_thresholds_nominal;
    std::vector<double>                       lower_force_thresholds_acceleration;
    std::vector<double>                       upper_force_thresholds_acceleration;
    std::vector<double>                       lower_force_thresholds_nominal;
    std::vector<double>                       upper_force_thresholds_nominal;

    // Port Interface
    RTT::InputPort<std::vector<double>>      control_joint_velocities;
    RTT::InputPort<std::string>              events_port;
    RTT::OutputPort<std::vector<double>>     sensor_joint_angles;
    RTT::OutputPort<std::vector<double>>     tool_external_wrench;

    // void print_vec();


};
#endif
