# Orocos driver for Franka Panda

## Status

WIP (work-in-progress)



## Requirements

* libfranka
* motion_control (CATKIN package)
* Orocos and ros_rtt installed



## Install

(Tested on Ubuntu 16.04 LTS, Orocos 2.8)

**NOTE** this step won't be necessary in the future (FIXME)

```bash
git submodule init
git submodule update
cd libfranka
git submodule init
git submodule update
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<workspace>/devel/lib
make install
```

and finally, `catkin_make` from root workspace.

## Run

Coming soon





## Contributions

* Enea Scioni, <enea.scioni@kuleuven.be>