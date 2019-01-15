#ifndef OROCOS_ROBOT_DATA_TEST_COMPONENT_HPP
#define OROCOS_ROBOT_DATA_TEST_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <Eigen/Core>
#include "QuinticPolynomial.hpp"
#include "cosine.hpp"


class Robot_data_test : public RTT::TaskContext {
public:
    Robot_data_test(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    RTT::InputPort<rstrt::robot::JointState> joint_state_in_port;
    RTT::FlowStatus joint_state_in_flow;
    rstrt::robot::JointState joint_state_in_data;

    RTT::InputPort<Eigen::VectorXf> grav_in_port;
    RTT::FlowStatus grav_in_flow;
    Eigen::VectorXf grav_in_data;

    RTT::InputPort<Eigen::VectorXf> coriolis_in_port;
    RTT::FlowStatus coriolis_in_flow;
    Eigen::VectorXf coriolis_in_data;

    RTT::InputPort<Eigen::MatrixXf> inertia_in_port;
    RTT::FlowStatus inertia_in_flow;
    Eigen::MatrixXf inertia_in_data; // 7x7

    RTT::InputPort<Eigen::MatrixXf> jacobian_in_port;
    RTT::FlowStatus jacobian_in_flow;
    Eigen::MatrixXf jacobian_in_data; // 6x7

    RTT::OutputPort<rstrt::dynamics::JointTorques> out_trq_port;
    rstrt::dynamics::JointTorques out_trq_data;

    RTT::OutputPort<rstrt::kinematics::JointVelocities> out_vel_port;
    rstrt::kinematics::JointVelocities out_vel_data;

    RTT::OutputPort<rstrt::kinematics::JointAngles> out_pos_port;
    rstrt::kinematics::JointAngles out_pos_data;

    RTT::OutputPort<rstrt::dynamics::JointImpedance> out_imp_port;
    rstrt::dynamics::JointImpedance out_imp_data;

    QuinticPolynomial<float> qp;
    Cosine<float> cos;
    Eigen::VectorXf* ramp_input, *ramp_output;
    double current_time = 0, end_time = 0, start_time = 0;
    bool value_set, single_value_set, lock, gen_cosine, impedance_set;
    void setMode(const std::string mode);
    void setValue(int idx, float val);
    void setSingleValue(int idx, float val);
    void ramp(int idx, float target, double time);
    void cosine(int idx, double amplitude, double period);
    void setImpedance(int idx, float stiffness, float damping);
    void setFullImpedance(const rstrt::dynamics::JointImpedance& impedance);
    void print();

    inline void write() {
        out_trq_port.write(out_trq_data);
        out_vel_port.write(out_vel_data);
        out_pos_port.write(out_pos_data);
    };
};
#endif
