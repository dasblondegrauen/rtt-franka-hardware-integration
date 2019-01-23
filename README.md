# rtt-franka-hardware-integration
Integration of Franka Panda arm into Orocos

## Installation
Besides an Orocos toolchain, this project requires *[libfranka](https://github.com/frankaemika/libfranka)* built from source for compilation. Installation instructions can be found at <https://frankaemika.github.io/docs/installation.html>. *libfranka* apparently needs a kernel compiled with the *PREEMPT_RT* patchset, so you can either build your own kernel as stated in the official documentation or simply install one of the lowlatency kernels included with recent versions of Ubuntu:

```
sudo apt install linux-image-lowlatency
```

With `<libfranka>` as your location of the _libfranka_ sources, use

```
mkdir build && cd build
cmake -DFranka_DIR=<libfranka>/build ..
cmake --build .
```
    
to build an Orocos component from the code.

## Usage
To control the Franka Panda arm from Orocos, make sure your machine booted a kernel including the *PREMPT_RT* patch and is connected to the controller hardware as written in the official documentation. Additional, you will need the URDF and SRDF of the manipulator (available for example [here](https://github.com/corlab/cogimon-gazebo-models/tree/master/franka), originally taken from the [franka_ros package](https://github.com/frankaemika/franka_ros/tree/kinetic-devel/franka_description)).

If those prerequisites are fulfilled, you can load and use a component called *franka::franka_robot* from the *deployer*:

```sh
# Import typekits and OS library for convenience
import("eigen_typekit")
import("rst-rt_typekit")
require("os")
    
# Import rtt-franka-hardware-integration
import("rtt-franka-hardware-integration")

# Assume URDF and SRDF files are in $GAZEBO_MODEL_PATH/franka
var string model_path = os.getenv("GAZEBO_MODEL_PATH") + "/franka"

# Load component and schedule it really fast (event driven) with high priority
loadComponent("franka", "franka::franka_robot")
setActivity("franka", 0, 90, ORO_SCHED_RT)

# Initialize kinematic chain using URDF and SRDF files
franka.addChain("franka", "172.16.0.2", model_path + "model.urdf", model_path + "model.srdf")

# Set control mode and start component!
franka.setControlMode("franka", "JointTorqueCtrl")
franka.configure()
franka.start()

# TODO
# Connect your own component to the feedback and command ports of franka and control the robot!
```