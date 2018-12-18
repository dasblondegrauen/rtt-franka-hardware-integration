# Lazy script to set up environment for testing.
# You probably have to adjust the values for use on your own machine.

source ~/citk/systems/cogimon-minimal-nightly/bin/setup-cogimon-env.sh
unset PYTHONPATH

export RTT_COMPONENT_PATH=$RTT_COMPONENT_PATH:~/code/rtt-franka-hardware-integration/build/orocos:~/code/rtt-franka-hardware-integration/test/robot_data_test/build/src/orocos
