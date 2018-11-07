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



    addOperation("setValue", &Robot_data_test::setValue, this, RTT::ClientThread);
    addOperation("ramp", &Robot_data_test::ramp, this, RTT::ClientThread);
    addOperation("cosine", &Robot_data_test::cosine, this, RTT::ClientThread);

    lock = false;
    gen_cosine = false;
    ramp_input = &(joint_state_in_data.torques);
    ramp_output = &(out_trq_data.torques);
}

bool Robot_data_test::configureHook() {
    out_trq_data.torques.setZero(7);
    out_vel_data.velocities.setZero(7);
    out_pos_data.angles.setZero(7);

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

    return true;
}

bool Robot_data_test::startHook() {
    if(out_pos_port.connected()) {
        RTT::log(RTT::Info) << "Starting test component in position mode" << RTT::endlog();
        ramp_input = &(joint_state_in_data.angles);
        ramp_output = &(out_pos_data.angles);
    } else if(out_vel_port.connected()) {
        RTT::log(RTT::Info) << "Starting test component in velocity mode" << RTT::endlog();
        ramp_input = &(joint_state_in_data.velocities);
        ramp_output = &(out_vel_data.velocities);
    } else {
        if(!out_trq_port.connected()) {
            RTT::log(RTT::Info) << "No output port connected, falling back to default" << RTT::endlog();
        }

        RTT::log(RTT::Info) << "Starting test component in torque mode" << RTT::endlog();
        ramp_input = &(out_trq_data.torques);
        ramp_output = &(out_trq_data.torques);
    }

    return true;
}

void Robot_data_test::updateHook() {
    // Get current time
    current_time = 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());

    // Read state from input ports
    joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);

    // Feed back joint positions & velocities

    // Modify values
    if(joint_state_in_flow != RTT::NoData) {
        if(current_time < end_time) {
            *ramp_output = qp.getQ(current_time);
            out_trq_port.write(out_trq_data);
            out_vel_port.write(out_vel_data);
            out_pos_port.write(out_pos_data);
        } else if(gen_cosine) {
            *ramp_output = cos.getQ(current_time - start_time);
            RTT::log(RTT::Info) << "Generating cosine: " << out_pos_data.angles.transpose() << RTT::endlog();
            out_trq_port.write(out_trq_data);
            out_vel_port.write(out_vel_data);
            out_pos_port.write(out_pos_data);
        } else {
            lock = false;
            gen_cosine = false;
        }

        // Do something with *ramp_output, so it does not get optimized out ¯\_(ツ)_/¯
        RTT::log(RTT::Info) << "Values: " << ramp_output->transpose() << RTT::endlog();
    }
}

void Robot_data_test::stopHook() {

}

void Robot_data_test::cleanupHook() {
    out_trq_data.torques.setZero();
    out_vel_data.velocities.setZero();
    out_pos_data.angles.setZero();
}

void Robot_data_test::setValue(int idx, float val) {
    (*ramp_output)(idx) = val;
}

void Robot_data_test::ramp(int idx, float target, double time) {
    if(lock) {
        return;
    }

    end_time = current_time + time;

    Eigen::VectorXf start_conf = Eigen::VectorXf(*ramp_input);
    Eigen::VectorXf end_conf = start_conf;
    end_conf(idx) = target;

    qp = QuinticPolynomial<float>(current_time, end_time, start_conf, end_conf);

    lock = true;
}


void Robot_data_test::cosine(int idx, double amplitude, double period) {
    start_time = current_time;

    Eigen::VectorXf amplitudes = Eigen::VectorXf::Zero(7);
    amplitudes(idx) = amplitude;

    cos = Cosine<float>(Eigen::VectorXf(*ramp_input), amplitudes, period);

    gen_cosine = true;
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
