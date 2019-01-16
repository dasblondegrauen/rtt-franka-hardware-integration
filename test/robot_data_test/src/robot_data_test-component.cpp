#include "robot_data_test-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Robot_data_test::Robot_data_test(std::string const& name) : TaskContext(name) {
    joint_state_in_flow = RTT::NoData;
    joint_state_in_data.angles.setZero(7);
    joint_state_in_data.velocities.setZero(7);
    joint_state_in_data.torques.setZero(7);
    this->addPort("joint_vals_in_port", joint_state_in_port);

    grav_in_flow = RTT::NoData;
    grav_in_data.setZero(7);
    this->addPort("grav_in_port", grav_in_port);

    coriolis_in_flow = RTT::NoData;
    coriolis_in_data.setZero(7);
    this->addPort("coriolis_in_port", coriolis_in_port);

    inertia_in_flow = RTT::NoData;
    inertia_in_data.setZero(7, 7);
    this->addPort("inertia_in_port", inertia_in_port);

    jacobian_in_flow = RTT::NoData;
    jacobian_in_data.setZero(6, 7);
    this->addPort("jacobian_in_port", jacobian_in_port);

    addOperation("setMode", &Robot_data_test::setMode, this, RTT::ClientThread);
    addOperation("setValue", &Robot_data_test::setValue, this, RTT::ClientThread);
    addOperation("setSingleValue", &Robot_data_test::setSingleValue, this, RTT::ClientThread);
    addOperation("ramp", &Robot_data_test::ramp, this, RTT::ClientThread);
    addOperation("cosine", &Robot_data_test::cosine, this, RTT::ClientThread);
    addOperation("setImpedance", &Robot_data_test::setImpedance, this, RTT::ClientThread);
    addOperation("setFullImpedance", &Robot_data_test::setFullImpedance, this, RTT::ClientThread);
    addOperation("print", &Robot_data_test::print, this, RTT::ClientThread);

    value_set = false;
    single_value_set = false;
    lock = false;
    gen_cosine = false;
    impedance_set = false;

    this->setMode("torque");
}

bool Robot_data_test::configureHook() {
    out_trq_data.torques.setZero(7);
    out_vel_data.velocities.setZero(7);
    out_pos_data.angles.setZero(7);
    out_imp_data.stiffness.setZero(7);
    out_imp_data.damping.setZero(7);

    joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);
    if(joint_state_in_flow != RTT::NoData) {
        out_pos_data.angles = joint_state_in_data.angles;
        out_vel_data.velocities = joint_state_in_data.velocities;

        out_pos_port.setDataSample(out_pos_data);
        out_vel_port.setDataSample(out_vel_data);
    }

    out_trq_port.setDataSample(out_trq_data);
    this->addPort("joint_trqs_out_port", out_trq_port);

    out_vel_port.setDataSample(out_vel_data);
    this->addPort("joint_vels_out_port", out_vel_port);

    out_pos_port.setDataSample(out_pos_data);
    this->addPort("joint_pos_out_port", out_pos_port);

    out_imp_port.setDataSample(out_imp_data);
    this->addPort("joint_imp_out_port", out_imp_port);

    return true;
}

bool Robot_data_test::startHook() {
    return true;
}

void Robot_data_test::updateHook() {
    // Get current time
    current_time = 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());

    // Read state from input ports
    joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);

    // Modify values
    if(joint_state_in_flow == RTT::NewData) {
        if(value_set) {
            write();
        } if(single_value_set) {
            write();
            single_value_set = false;

            out_pos_data.angles = joint_state_in_data.angles;
            out_vel_data.velocities = joint_state_in_data.velocities;
        } else if(current_time < end_time) {
            *ramp_output = qp.getQ(current_time);
            write();
        } else if(gen_cosine) {
            *ramp_output = cos.getQ(current_time - start_time);
            write();
        } else {
            lock = false;

            out_pos_data.angles = joint_state_in_data.angles;
            out_vel_data.velocities = joint_state_in_data.velocities;
        }

        if(impedance_set) {
            out_imp_port.write(out_imp_data);
        }

        // Do something with *ramp_output, so it does not get optimized out ¯\_(ツ)_/¯
        //RTT::log(RTT::Info) << "Values: " << ramp_output->transpose() << RTT::endlog();
    }
}

void Robot_data_test::stopHook() {

}

void Robot_data_test::cleanupHook() {

}

void Robot_data_test::setMode(const std::string mode) {
    joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);

    if(mode == "torque") {
        out_trq_data.torques.setZero(7);
        ramp_input = &(joint_state_in_data.torques);
        ramp_output = &(out_trq_data.torques);
    } else if(mode == "position") {
        out_pos_data.angles = joint_state_in_data.angles;
        ramp_input = &(joint_state_in_data.angles);
        ramp_output = &(out_pos_data.angles);
    } else if(mode == "velocity") {
        out_vel_data.velocities = joint_state_in_data.velocities;
        ramp_input = &(joint_state_in_data.velocities);
        ramp_output = &(out_vel_data.velocities);
    } else {
        RTT::log(RTT::Warning) << "Could not set unkown mode " << mode
            << "\nFalling back to torque mode" << RTT::endlog();

        out_trq_data.torques.setZero();
        ramp_input = &(joint_state_in_data.torques);
        ramp_output = &(out_trq_data.torques);

        return;
    }

    RTT::log(RTT::Info) << mode << " mode set!" << RTT::endlog();
}

void Robot_data_test::setValue(int idx, float val) {
    (*ramp_output)(idx) = val;
    value_set = true;
}

void Robot_data_test::setSingleValue(int idx, float val) {
    (*ramp_output)(idx) = val;
    single_value_set = true;
}

void Robot_data_test::ramp(int idx, float target, double time) {
    if(lock) {
        return;
    }

    end_time = current_time + time;

    Eigen::VectorXf start_conf = Eigen::VectorXf(*ramp_output);
    Eigen::VectorXf end_conf = Eigen::VectorXf(start_conf);
    end_conf(idx) = target;

    qp = QuinticPolynomial<float>(current_time, end_time, start_conf, end_conf);

    lock = true;
}


void Robot_data_test::cosine(int idx, double amplitude, double period) {
    start_time = current_time;

    Eigen::VectorXf amplitudes = Eigen::VectorXf::Zero(7);
    amplitudes(idx) = amplitude;

    cos = Cosine<float>(Eigen::VectorXf(*ramp_output), amplitudes, period);
    gen_cosine = true;
}


void Robot_data_test::setImpedance(int idx, float stiffness, float damping) {
    out_imp_data.stiffness(idx) = stiffness;
    out_imp_data.damping(idx) = damping;

    impedance_set = true;
}


void Robot_data_test::setFullImpedance(const rstrt::dynamics::JointImpedance &impedance) {
    out_imp_data = impedance;
    impedance_set = true;
}


void Robot_data_test::print() {
    grav_in_flow = grav_in_port.read(grav_in_data);
    coriolis_in_flow = coriolis_in_port.read(coriolis_in_data);
    inertia_in_flow = inertia_in_port.read(inertia_in_data);
    jacobian_in_flow = jacobian_in_port.read(jacobian_in_data);

    std::cout << "Gravity: " << grav_in_data.transpose() << std::endl;
    std::cout << "Coriolis: " << coriolis_in_data.transpose() << std::endl;

    std::cout << "Inertia:\n" << inertia_in_data << std::endl;
    std::cout << "Jacobian:\n" << jacobian_in_data << std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Robot_data_test)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Robot_data_test)
