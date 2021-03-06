require "rttlib"
require "rttros"
require "utils"


rttlib.color=true

--If you set it to Info you will get more stuff printed from the driver
rtt.setLogLevel("Warning")

tc=rtt.getTC()
if tc:getName() == "lua" then
  depl=tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
  depl=tc
end

depl:import("rtt_ros")
depl:import("rtt_sensor_msgs")
-- depl:import("franka_panda_driver")
depl:import("rtt_motion_control_msgs")

gs=rtt.provides()
ros = gs:provides("ros")
ros:import("rtt_rospack")

dir = rtt.provides("ros"):find("orocos_franka_panda") .. "/scripts/"
-- dir = "/home/santiregui/etasl-install/ws/etasl-ex/src/orocos_franka_panda/scripts/"
print(dir)

--Load the component generating the trajectory
depl:loadComponent("traj_gen", "OCL::LuaComponent")
traj_gen = depl:getPeer("traj_gen")
traj_gen:exec_file(dir .. "velocity_generator.lua")

ros:import("orocos_franka_panda")
depl:loadComponent("panda", "FrankaComponent")
panda=depl:getPeer("panda")
panda:getProperty('ip_address'):set("172.16.0.2")
depl:setActivity("panda", 0, 99, rtt.globals.ORO_SCHED_RT)

-- =======================Uncomment for Joint Impedance====================================
      joint_impedance = rtt.Variable("array")
      joint_impedance:fromtab{3000, 1000, 3000, 2500, 2500, 2000, 2000}
      panda:getProperty('joint_impedance'):set(joint_impedance)
      panda:getProperty('impedance_mode'):set("joint")


--=======================Uncomment for Cartesian Impedance====================================
      -- cartesian_impedance = rtt.Variable("array")
      -- cartesian_impedance:fromtab{3000, 3000, 500, 100, 100, 100}
      -- panda:getProperty('cartesian_impedance'):set(cartesian_impedance)
      -- panda:getProperty('impedance_mode'):set("cartesian")


--Connect the ports
cp=rtt.Variable("ConnPolicy")
-- depl:connect("panda.tool_external_wrench","traj_gen.measured_angles",cp )
depl:connect("panda.sensor_joint_angles","traj_gen.measured_angles",cp )
depl:connect("panda.control_joint_velocities","traj_gen.desired_velocities",cp )

panda:configure()
traj_gen:configure()

depl:setActivity("traj_gen", 0.01, 0, rtt.globals.ORO_SCHED_OTHER)
-- depl:setActivity("traj_gen", 0.2, 0, rtt.globals.ORO_SCHED_OTHER)
traj_gen:start()

panda:error_recovery()
-- panda:gripper_change_apperture(0.05,0.1)
-- panda:gripper_grasp(0.015,0.1,60)


panda:low_level_velocity()
-- panda:admittance()
