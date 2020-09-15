# Orocos driver for Franka Panda



## Requirements

- libfranka
- Orocos toolchain 2.9 installed
- ROS Melodic (only catkin is used for handling the packages. It is possible to modify the CMakeLists for getting rid of this dependency)
- Real Time Kernel (installation instructions found [here](https://frankaemika.github.io/docs/installation_linux.html)). If you don't use the Real Time Kernel you will get communication errors due to the very strict requirements of the real time controller of the Franka.


## Install

(Tested on Ubuntu 18.04 LTS, Orocos 2.9, Ros Melodic)

This package contains the Git submodule libfranka. Clone it using --recursive flag:
```shell
git clone --recursive https://github.com/santiregui/orocos_franka_panda.git
```

 Run the following to build libfranka:
 ```bash
cd libfranka
git checkout 0.7.1
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
 ```


 <!-- ### Frankalib with apt

Right now it is not possible to do this since you get the latest API version, which is not compatible with this driver (we need 0.7.1).

Install the libfranka libraries:
```shell
sudo apt install ros-melodic-libfranka ros-melodic-franka-ros
```
  -->


Compile using `catkin_make` from the root workspace.

## How to use


### Preliminary steps
1. Check that you turned on the computer with the Real Time Kernel. To do that you can run in the terminal the following:
  ```bash
  uname -r
  ```
  If the computer has the real time kernel running, you should see the output "5.0.21-rt16" (or equivalent, depending on the version). If not, the output will have a "-generic" suffix.

  In case you are not using the real time kernel please restart your computer. As soon as it reboots you will see the grub menu (when you choose between ubuntu and windows). Then select "advanced options for ubuntu" and then select the one with real time Kernel (**Not** generic and **not** recovery mode)

2. Check that the computer is in performance mode (in the top left corner, the icon more to the left side). You can **also** run:
```bash
cpufreq-info
```
You should obtain this message within the output:
```bash
  current policy: frequency should be within 800 MHz and 4.30 GHz.
                  The governor "performance" may decide which speed to use
                  within this range.
```

To execute orocos components in real time, you also need to give the necessary permitions to the user. To do this, open the text file in */etc/security/limits.conf* and add this line at the end (by changing my_user_name to your own): *my_user_name rtprio hard 99*

When you are ready to run the task, open Google Chrome and go type the following website/url: 172.16.0.2
It should take you to the Franka Web Interface. Unlock the brakes of the robot.

### Configure Driver
In the Orocos deployer you can configure the driver as described in this section.


Import the driver:
```lua
ros:import("orocos_franka_panda")
depl:loadComponent("panda", "FrankaComponent")
panda=depl:getPeer("panda")
```
Configure properties and setup a non-periodic activity with maximum priority (for real time control loop):
```lua
panda:getProperty('ip_address'):set("172.16.0.2")
depl:setActivity("panda", 0, 99, rtt.globals.ORO_SCHED_RT)
```
If you want to modify the impedance of the joints (impedance in jointspace already by default), include the following:
```lua
joint_impedance = rtt.Variable("array")
joint_impedance:fromtab{5000, 5000, 5000, 5000, 5000, 5000, 5000}
panda:getProperty('joint_impedance'):set(joint_impedance)
panda:getProperty('impedance_mode'):set("joint")
```

If you want to use cartesian impedance mode, include the following:

```lua
cartesian_impedance = rtt.Variable("array")
cartesian_impedance:fromtab{3000, 3000, 2000, 100, 100, 100}
panda:getProperty('cartesian_impedance'):set(cartesian_impedance)
panda:getProperty('impedance_mode'):set("cartesian")
```

**Important remark:** The control loop is handled by libfranka and not by Orocos (Empty updateHook) with a blocking routine. Therefore, the terminal will get blocked when you start the control loop by executing the *start_sending_setpoints()* operation (you can not execute any lua command).

In order to stop the control loop, and therefore perform other operations, you can send a string through the event port. The control loop will stop if the string is equal to the one given in the *event_stop_loop* property. You can modify it by including:
```lua
 panda:getProperty('event_stop_loop'):set("e_finished@etaslcore")
```

If you are using rFSM and eTaSL, it is not necesary to stop the control loop when making the transitions, because the transitions are so fast that the communication with the franka allows such small delays (although it might be a good idea to stop it anyway).


### Ports
  **control_joint_velocities:** Joint velocity commands in a double vector (of size 7).
  **sensor_joint_angles:** Joint angle measurements (for control feedback).
  **tool_external_wrench (optional):** Estimation of the external wrench based on the joint torques (provided by frankalib).
  **panda.events_port (optional):** Port to stop the control loop.

### Run
```lua
panda:configure()
panda:error_recovery()
panda:low_level_velocity() --Starts the control loop at 1 kHz.
```


## Author

- Santiago Iregui, santiago.iregui@kuleuven.be

## Acknowledgement
Thanks to Enea Scioni for providing the skeleton of the package
