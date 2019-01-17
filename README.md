# rtt-franka-hardware-integration
Integration of Franka Panda arm into Orocos

## Installation
Besides an Orocos toolchain, this project requires *[libfranka](https://github.com/frankaemika/libfranka)* built from source for compilation. Installation instructions can be found at <https://frankaemika.github.io/docs/installation.html>. *libfranka* apparently needs a kernel compiled with the *PREEMPT_RT* patchset, so you can either build your own kernel as stated in the official documentation or simply install one of the lowlatency kernels included with recent versions of Ubuntu:

    sudo apt install linux-image-lowlatency

With `<libfranka>` as your location of the _libfranka_ sources, use

    mkdir build && cd build
    cmake -DFranka_DIR=<libfranka>/build ..
    cmake --build .
    
to build an Orocos component from the code.
