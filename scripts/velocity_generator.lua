require("rttlib")
require("math")
-- require("inspect") --Just for debuging, it allows to print pretty lua tables. Install by: running sudo luarocks install inspect


--This component generates a sine wave for joint velocities

tc=rtt.getTC()



measured_angles = rtt.InputPort("array", "measured_angles")
tc:addPort(measured_angles)

desired_velocities = rtt.OutputPort("array", "desired_velocities")
tc:addPort(desired_velocities)

local joint_setpoints=rtt.Variable("array")

function configureHook()
    return true
end

function startHook()

    return true
end

time = 0
frequency = 0.2
amplitud = 10*math.pi/180 --Radians
function updateHook()

  -- angle = math.sin(2*math.pi*frequency*time )*amplitud
  angle = math.sin(2*math.pi*frequency*time )*amplitud
  mytab = {0,0,0,0,0,0,angle} --For velocities
  -- mytab = {0,0,0,0,0,0,0} --For velocities
  -- mytab = {0,0,0,0,0,0,2*math.pi/180} --For velocities

  joint_setpoints:fromtab(mytab)
  desired_velocities:write(joint_setpoints)
  time = time + tc:getPeriod()

  -- Uncomment for printing the measured angles in the terminal:
  -- local fs,val= measured_angles:read()
  -- print(val)

end


function cleanupHook()

end

function round(num, numDecimalPlaces)
  local mult = 10^(numDecimalPlaces or 0)
  return math.floor(num * mult + 0.5) / mult
end
