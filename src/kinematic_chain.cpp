#include <kinematic_chain.h>
#include <string>
#include <franka/lowpass_filter.h>
#include <franka/rate_limiting.h>


constexpr research_interface::robot::Move::Deviation KinematicChain::kDefaultDeviation;

KinematicChain::KinematicChain(const std::string &chain_name,
                               const std::vector<std::string> &joint_names,
                               RTT::DataFlowInterface &ports_in,
                               std::unique_ptr < franka::Robot::Impl > franka_control)
    : kinematic_chain_name(chain_name),
      ports(ports_in) {
    dof = joint_names.size();
    franka_model = std::make_unique<franka::Model>(franka_control->loadModel());

    this->franka_control = std::move(franka_control);
    current_control_mode = franka::ControlModes::Torque;
}

bool KinematicChain::initKinematicChain() {
    RTT::log(RTT::Info) << kinematic_chain_name << RTT::endlog();
    setFeedBack();
    setCollisionBehavior(
    {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}}, {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
    {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}}, {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}});
    //{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    //TODO testing, values below should be set from srdf file
    setImpedanceBehavior({3000, 3000, 3000, 2500, 2500, 2000, 2000}, {3000, 3000, 3000, 300, 300, 300});

    return true;
}

bool KinematicChain::resetKinematicChain() {
    return true;
}

std::string KinematicChain::getKinematicChainName() {
    return kinematic_chain_name;
}

unsigned int KinematicChain::getNumberOfDOFs() {
    return dof;
}

const std::string &KinematicChain::getCurrentControlMode() {
    return franka::ControlModeMap.find(current_control_mode)->second;
}

void KinematicChain::setFeedBack() {
    jf = std::make_unique<franka::JointFeedback<rstrt::robot::JointState> >(kinematic_chain_name, this->ports, "JointFeedback", [this](rstrt::robot::JointState &in) -> void {in = rstrt::robot::JointState(dof); in.angles.setZero(); in.velocities.setZero(); in.torques.setZero();});
    inertia_feedback = std::make_unique<franka::DynamicFeedback<Eigen::MatrixXf> >(kinematic_chain_name, this->ports, "InertiaFeedback", [this](Eigen::MatrixXf &in) -> void {in(dof, dof); in.setZero();});
    coriolis_feedback = std::make_unique<franka::DynamicFeedback<Eigen::VectorXf> >(kinematic_chain_name, this->ports, "CoriolisFeedback", [this](Eigen::VectorXf &in) -> void {in(dof); in.setZero();});
    gravity_feedback = std::make_unique<franka::DynamicFeedback<Eigen::VectorXf> >(kinematic_chain_name, this->ports, "GravityFeedback", [this](Eigen::VectorXf &in) -> void {in(dof); in.setZero();});
    jacobian_feedback = std::make_unique<franka::DynamicFeedback<Eigen::MatrixXf> >(kinematic_chain_name, this->ports, "JacobianFeedback", [this](Eigen::MatrixXf &in) -> void {in(6, dof); in.setZero();});
}

bool KinematicChain::setControlMode(const std::string &controlMode) {
    RTT::log(RTT::Info) << "KC set control mode " << this->kinematic_chain_name << RTT::endlog();

    motion_command = {};
    control_command = {};

    RTT::log(RTT::Info) << "fill 0 end " << franka::ControlModeMap.find(franka::ControlModes::Torque)->second << RTT::endlog();

    if (controlMode == franka::ControlModeMap.find(franka::ControlModes::Torque)->second) {
        RTT::log(RTT::Info) << "found map string" << RTT::endlog();
        current_control_mode = franka::ControlModes::Torque;
        RTT::log(RTT::Info) << "To torque!" << RTT::endlog();
        jc = std::make_unique<franka::JointController<rstrt::dynamics::JointTorques> >(kinematic_chain_name,
                                                                                       this->ports,
                                                                                       franka::ControlModes::Torque,
                                                                                       [](rstrt::dynamics::JointTorques &input) -> Eigen::VectorXf & {return input.torques;});
        current_control_input_var = &(control_command.tau_J_d);
        RTT::log(RTT::Info) << "Created relavant controller" << RTT::endlog();
    } else if(controlMode == franka::ControlModeMap.find(franka::ControlModes::Velocity)->second) {
        RTT::log(RTT::Info) << "Found mapping string" << RTT::endlog();
        current_control_mode = franka::ControlModes::Velocity;
        jc = std::make_unique<franka::JointController<rstrt::kinematics::JointVelocities>>(kinematic_chain_name,
                                                                                           this->ports,
                                                                                           franka::ControlModes::Velocity,
                                                                                           [](rstrt::kinematics::JointVelocities &input) -> Eigen::VectorXf& {return input.velocities;});
        current_control_input_var = &(motion_command.dq_c);
        RTT::log(RTT::Info) << "Set control mode to velocity" << RTT::endlog();
    } else if(controlMode == franka::ControlModeMap.find(franka::ControlModes::Position)->second) {
        RTT::log(RTT::Info) << "Found mapping string" << RTT::endlog();
        current_control_mode = franka::ControlModes::Position;
        jc = std::make_unique<franka::JointController<rstrt::kinematics::JointAngles>>(kinematic_chain_name,
                                                                                       this->ports,
                                                                                       franka::ControlModes::Position,
                                                                                       [](rstrt::kinematics::JointAngles &input) -> Eigen::VectorXf& {return input.angles;});
        current_control_input_var = &(motion_command.q_c);
        RTT::log(RTT::Info) << "Set control mode to position" << RTT::endlog();
    } else if(controlMode == franka::ControlModeMap.find(franka::ControlModes::Impedance)->second) {
        RTT::log(RTT::Info) << "Found mapping string" << RTT::endlog();
        current_control_mode = franka::ControlModes::Impedance;
        jc = std::make_unique<franka::JointController<rstrt::dynamics::JointImpedance>>(kinematic_chain_name,
                                                                                       this->ports,
                                                                                       franka::ControlModes::Impedance,
                                                                                       [this](rstrt::dynamics::JointImpedance &input) -> Eigen::VectorXf& {return this->convertImpedance(input); });
        impedance_q_flow = RTT::NoData;
        impedance_q.angles.setZero(dof);
        ports.addPort(kinematic_chain_name + "_JointImpedanceQ", impedance_q_port)
                .doc("Input for JointImpedanceQ-cmds from Orocos to Franka.");
        current_control_input_var = &(control_command.tau_J_d);
        RTT::log(RTT::Info) << "Set control mode to impedance" << RTT::endlog();
    } else {
        RTT::log(RTT::Error) << "Control Mode has not been implemented " << controlMode << RTT::endlog();
        return false;
    }
    return true;
}

bool KinematicChain::startKinematicChain() {
    if (!franka_control) {
        return false;
    }

    //franka_state = franka_control->update(nullptr, nullptr);
    franka_state = static_cast<franka::Robot::Impl*>(franka_control.get())->readOnce();

    switch (current_control_mode) {
    case franka::ControlModes::Torque:
        RTT::log(RTT::Info) << "STARTED KINEMATIC CHAIN IN MODE: " << franka::ControlModeMap.find(franka::ControlModes::Torque)->second << RTT::endlog();
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kExternalController,
                                                franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode,
                                                kDefaultDeviation,
                                                kDefaultDeviation);
        break;
    case franka::ControlModes::Velocity:
        RTT::log(RTT::Info) << "STARTED KINEMATIC CHAIN IN MODE: " << franka::ControlModeMap.find(franka::ControlModes::Velocity)->second << RTT::endlog();
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance,
                                                franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode,
                                                kDefaultDeviation,
                                                kDefaultDeviation);
        motion_command.dq_c = franka_state.dq_d;
        break;
    case franka::ControlModes::Position:
        RTT::log(RTT::Info) << "STARTED KINEMATIC CHAIN IN MODE: " << franka::ControlModeMap.find(franka::ControlModes::Position)->second << RTT::endlog();
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance,
                                                franka::MotionGeneratorTraits<franka::JointPositions>::kMotionGeneratorMode,
                                                kDefaultDeviation,
                                                kDefaultDeviation);
        motion_command.q_c = franka_state.q_d;
        break;
    case franka::ControlModes::Impedance:
        RTT::log(RTT::Info) << "STARTED KINEMATIC CHAIN IN MODE: " << franka::ControlModeMap.find(franka::ControlModes::Impedance)->second << RTT::endlog();
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kExternalController,
                                                franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode,
                                                kDefaultDeviation,
                                                kDefaultDeviation);
        break;
    default:
        return false;
    }

    return true;
}

bool KinematicChain::sense() {
    //auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
    //RTT::log(RTT::Info) << "SENSE: " << time.count() % 60000000 << " ";

    jf->joint_feedback.angles = Eigen::Map<Eigen::VectorXd>(franka_state.q.data(), dof).cast<float>();
    jf->joint_feedback.velocities = Eigen::Map<Eigen::VectorXd>(franka_state.dq.data(), dof).cast<float>();
    jf->joint_feedback.torques = Eigen::Map<Eigen::VectorXd>(franka_state.tau_J.data(), dof).cast<float>();

    if (inertia_feedback->connected())
        inertia_feedback->dynamicFeedback = Eigen::Map<Eigen::MatrixXd>(franka_model->mass(franka_state).data(), dof, dof).cast<float>();

    if (coriolis_feedback->connected())
        coriolis_feedback->dynamicFeedback = Eigen::Map<Eigen::VectorXd>(franka_model->coriolis(franka_state).data(), dof).cast<float>();

    if (gravity_feedback->connected())
        gravity_feedback->dynamicFeedback = Eigen::Map<Eigen::VectorXd>(franka_model->gravity(franka_state).data(), dof).cast<float>();

    if (jacobian_feedback->connected())
        jacobian_feedback->dynamicFeedback = Eigen::Map<Eigen::MatrixXd>(franka_model->zeroJacobian(franka::Frame::kFlange,franka_state).data(), 6, dof).cast<float>();

    //RTT::log(RTT::Info) << "WRITE! Control command success rate: " << franka_state.control_command_success_rate << RTT::endlog();
    jf->write();
    if(franka_state.control_command_success_rate>0.8){
        if (inertia_feedback->connected())
            inertia_feedback->write();
        if (coriolis_feedback->connected())
            coriolis_feedback->write();
        if (gravity_feedback->connected())
            gravity_feedback->write();
        if (jacobian_feedback->connected())
            jacobian_feedback->write();
    }

    return true;
}

void KinematicChain::getCommand() {    
    if (jc->connected() && jc->read() == RTT::NewData) {
        //auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
        //RTT::log(RTT::Info) << "GETCMD " << time.count() % 60000000 << " ";

        for (size_t i = 0; i < 7; i++) {
            (*current_control_input_var)[i] = static_cast<double>(jc->value()(i));
            //RTT::log(RTT::Info) << current_control_input_var->at(i) << " ";
        }
        //RTT::log(RTT::Info) << RTT::endlog();
    } else {
        //RTT::log(RTT::Debug) << "No data from control input " << franka::ControlModeMap.find(current_control_mode)->second << RTT::endlog();
    }
}

void KinematicChain::stop() try {
    if(current_control_mode == franka::ControlModes::Torque || current_control_mode == franka::ControlModes::Impedance) {
        franka_control->finishMotion(motion_id, &motion_command, &control_command);
    } else {
        franka_control->finishMotion(motion_id, &motion_command, nullptr);
    }
} catch (...) {
    try {
        franka_control->cancelMotion(motion_id);
    } catch (...) {}
    throw;
}

void KinematicChain::move() try {
    if (jc->connected() && jc->joint_cmd_fs == RTT::NewData) {
        //auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
        //RTT::log(RTT::Info) << "MOVE: " << time.count() % 60000000 << " ";

        if(current_control_mode == franka::ControlModes::Torque || current_control_mode == franka::ControlModes::Impedance) {
            franka_state = franka_control->update(&motion_command, &control_command);
        } else {
            franka_state = franka_control->update(&motion_command, nullptr);
        }
    } else {
        franka_state = franka_control->update(nullptr, nullptr);
    }

    //auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
    //RTT::log(RTT::Info) << "ERROR? " << time.count() % 60000000 << RTT::endlog();
    franka_control->throwOnMotionError(franka_state, motion_id);
} catch (const franka::NetworkException &exc) {
    RTT::log(RTT::Error) << "NETWORK: " << exc.what() << RTT::endlog();
    throw;
} catch (const franka::ControlException &exc) {
    RTT::log(RTT::Error) << "CONTROL: " << exc.what() << RTT::endlog();
    throw;
} catch (const franka::ProtocolException &exc) {
    RTT::log(RTT::Error) << "PROTOCOL: " << exc.what() << RTT::endlog();
    throw;
}

void KinematicChain::setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                                          const std::array<double, 7>& upper_torque_thresholds,
                                          const std::array<double, 6>& lower_force_thresholds,
                                          const std::array<double, 6>& upper_force_thresholds) {
    static_cast<franka::Robot::Impl*>(franka_control.get())->executeCommand<research_interface::robot::SetCollisionBehavior>(
                lower_torque_thresholds, upper_torque_thresholds, lower_torque_thresholds,
                upper_torque_thresholds, lower_force_thresholds, upper_force_thresholds,
                lower_force_thresholds, upper_force_thresholds);
}

void KinematicChain::setImpedanceBehavior(const std::array<double, 7> &joint_imp, const std::array<double, 6> &cart_imp){
    static_cast<franka::Robot::Impl*>(franka_control.get())->executeCommand<research_interface::robot::SetJointImpedance>(joint_imp);
    static_cast<franka::Robot::Impl*>(franka_control.get())->executeCommand<research_interface::robot::SetCartesianImpedance>(cart_imp);
}

Eigen::VectorXf& KinematicChain::convertImpedance(rstrt::dynamics::JointImpedance& input) {
    Eigen::VectorXf error = this->jf->joint_feedback.angles - impedance_q.angles;
    this->impedance_tau.torques = -1.0 * input.stiffness.cwiseProduct(error)
            - input.damping.cwiseProduct(this->jf->joint_feedback.velocities)
            + Eigen::Map<Eigen::VectorXd>(this->franka_model->coriolis(this->franka_state).data(), this->dof).cast<float>();

    return impedance_tau.torques;
}

std::string KinematicChain::printKinematicChainInformation() {
    // std::stringstream joint_names_stream;
    // for (unsigned int i = 0; i < _joint_names.size(); ++i)
    //     joint_names_stream << _joint_names[i] << " ";
    //
    // auto &controller_names = getControllersAvailable();
    // std::stringstream controller_names_stream;
    // for (unsigned int i = 0; i < controller_names.size(); ++i)
    //     controller_names_stream << controller_names[i] << " ";
    //
    // std::stringstream info;
    // info << "Kinematic Chain: " << _kinematic_chain_name << std::endl;
    // info << "    Number of DOFs: " << _number_of_dofs << std::endl;
    // info << "    Joints:  [" << joint_names_stream.str() << "]" << std::endl;
    // info << "    Control Modes:  [ " << controller_names_stream.str() << "]"
    //      << std::endl;
    // info << "    Current Control Mode: " << _current_control_mode << std::endl;
    // info << "    Impedance: Stiffness = " << trqModeJntImpedance.stiffness << std::endl;
    // info << "    Impedance: Damping = " << trqModeJntImpedance.damping << std::endl;
    // info << "    overrideFakeImpedance = " << _overrideFakeImpedance << std::endl;

    return "TO IMPLEMENT!";
}
