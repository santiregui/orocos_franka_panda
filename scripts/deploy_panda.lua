--[[
  Deploy rttlua script for testing franka panda component
  Authors:
    * Enea Scioni, <enea.scioni@kuleuven.be>
   2019, KU Leuven, Belgium
--]]

require("rttlib")
require("rttros")
rtt.setLogLevel("Warning")
rttlib.color = true

tc=rtt.getTC()
depl=tc:getPeer("Deployer")


depl:import("rtt_ros")
depl:import("rtt_rospack")
depl:import("rtt_sensor_msgs")
depl:import("franka_panda_driver")
depl:import("rtt_motion_control_msgs")
--- Loading ROS-based Libraries
gs = rtt.provides()
ros = gs:provides("ros")
depl:import('rtt_rosnode')
--
depl:loadComponent("panda", "FrankaComponent")
panda=depl:getPeer("panda")


--Configuration
panda:getProperty('ip_address'):set("172.16.0.2")
depl:setActivity("panda", 0, 99, rtt.globals.ORO_SCHED_RT)
-- panda:setPeriod(0.005)

panda:configure()
-- panda:start()

initial_angle = rtt.Variable("array")
initial_angle = panda:get_joint_angles()
print(initial_angle)
-------------------------------------------
---- ROS Streams
-- depl:stream("panda.joint_states", ros:topic("/joint_states"))
