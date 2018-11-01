# Lazy script to set up environment for testing.
# You probably have to adjust the values for use on your own machine.

source ~/citk/systems/cogimon-minimal-nightly/bin/setup-cogimon-env.sh

export RTT_COMPONENT_PATH=$RTT_COMPONENT_PATH:~/rtt-franka-hardware-integration/build/orocos:~/rtt-franka-hardware-integration/test/robot_data_test/build/src/orocos
export GAZEBO_MODEL_PATH=~/cogimon-gazebo-models

export RTT_COMPONENT_PATH=$RTT_COMPONENT_PATH:~/franka-test/JointPositionControl/build/src/orocos
