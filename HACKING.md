# Hacking
This document refers to the implementation of the Orocos integration for Franka robots in detail. For general information on how to use the robot and its software, please consult the [documentation provided by Franka Emika](https://frankaemika.github.io) and [README.md](README.md).

## Code structure
To split up complexity, source code is distributed over several files and directories serving a specific purpose.

```
include/
├── control_modes.hpp
├── kinematic_chain.hpp
└── rtt-franka-robot.hpp
src/
├── kinematic_chain.cpp
├── rtt-franka-robot.cpp
└── rtt-franka-robot_hooks.cpp
ops/
```

TODO Add libfranka