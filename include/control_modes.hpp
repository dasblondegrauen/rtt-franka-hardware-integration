#ifndef _CONTROL_MODES_H_
#define _CONTROL_MODES_H_

#include <string>
#include <rtt/Port.hpp>
#include <Eigen/Core>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>

namespace franka {
    /**
     * Enum of implemented control modes for the kinematic chain
     */
    enum ControlModes {Position, Velocity, Torque, Impedance};

    /**
     * Mapping of ControlModes values to string representations
     */
    static const std::map < ControlModes, std::string > ControlModeMap = {
        {Position, "JointPositionCtrl"},
        {Velocity, "JointVelocityCtrl"},
        {Torque, "JointTorqueCtrl"},
        {Impedance, "JointImpedanceCtrl"}
    };

    /**
     * Abstract base class for joint controllers of the kinematic chain
     */
    class BaseJointController {
        public:
            virtual RTT::FlowStatus &read() = 0;
            virtual bool connected() = 0;
            virtual Eigen::VectorXf &value() = 0;
            RTT::FlowStatus joint_cmd_fs = RTT::NoData;
    };

    /**
     * Generic joint controller.
     * Used for torque control of the kinematic chain
     */
    template < class T > class JointController: public BaseJointController {
        public:
            JointController(const std::string &name, RTT::DataFlowInterface &ports, const ControlModes &control_name, std::function < Eigen::VectorXf & (T &) > conversion_in) : conversion(conversion_in) {
                orocos_port.setName(name + "_" + ControlModeMap.find(control_name)->second);
                orocos_port.doc("Input for " + ControlModeMap.find(control_name)->second + "-cmds from Orocos to Franka.");
                joint_cmd_fs = RTT::NoData;
                ports.addPort(orocos_port);

                joint_cmd_fs = RTT::NoData;
            }

            ~JointController() {}

            RTT::FlowStatus &read() override {
                joint_cmd_fs = orocos_port.read(joint_cmd);
                return joint_cmd_fs;
            }

            RTT::InputPort < T > orocos_port;
            //RTT::FlowStatus joint_cmd_fs;
            T joint_cmd;
            std::function < Eigen::VectorXf & (T &) > conversion;

            bool connected() override {
                return orocos_port.connected();
            }

            Eigen::VectorXf &value() override {
                return conversion(joint_cmd);
            }
    };

    /**
     * Specialized joint controller for impedance control.
     * A generic implementation of the joint congroller does not suffice,
     * since additional input and computation is required for impedance control
     */
    class JointImpedanceController: public BaseJointController {
    public:
        JointImpedanceController(const std::string &name,
                                 RTT::DataFlowInterface &ports,
                                 const ControlModes &control_name,
                                 std::array<double, 7> q_d,
                                 std::function<Eigen::VectorXf& (rstrt::dynamics::JointImpedance&, rstrt::kinematics::JointAngles&, rstrt::dynamics::JointTorques&)> conversion_in)
            : conversion(conversion_in) {
            impedance_port.setName(name + "_JointImpedanceCtrl");
            impedance_port.doc("Input for JointImpedanceCtrl-cmds from Orocos to Franka.");
            impedance_cmd_fs = RTT::NoData;
            impedance_cmd.stiffness.setZero(7);
            impedance_cmd.damping.setZero(7);
            ports.addPort(impedance_port);

            position_port.setName(name + "_JointPosition");
            position_port.doc("Input for JointPosition-cmds from Orocos to Franka while in JointImpedanceCtrl");
            position_cmd_fs = RTT::NoData;
            position_cmd.angles = Eigen::Map<Eigen::VectorXd>(q_d.data(), 7).cast<float>();
            ports.addPort(position_port);

            torque_port.setName(name + "_JointTorque");
            torque_port.doc("Input for JointTorque-cmds from Orocos to Franka while in JointImpedanceCtrl");
            torque_cmd_fs = RTT::NoData;
            torque_cmd.torques.setZero(7);
            ports.addPort(torque_port);

            joint_cmd_fs = RTT::NoData;
        }

        ~JointImpedanceController() {}

        RTT::FlowStatus &read() override {
            if(impedance_port.connected()) {
                impedance_cmd_fs = impedance_port.read(impedance_buf);

                if(impedance_cmd_fs == RTT::NewData) {
                    impedance_cmd.stiffness = impedance_buf.stiffness;
                    impedance_cmd.damping = impedance_buf.damping;
                }

                joint_cmd_fs = impedance_cmd_fs;
            }

            if(position_port.connected()) {
                position_cmd_fs = position_port.read(position_buf);

                if(position_cmd_fs == RTT::NewData) {
                    position_cmd.angles = position_buf.angles;
                    joint_cmd_fs = position_cmd_fs;
                }
            }

            if(torque_port.connected()) {
                torque_cmd_fs = torque_port.read(torque_buf);

                if(torque_cmd_fs == RTT::NewData) {
                    torque_cmd.torques = torque_buf.torques;
                    joint_cmd_fs = torque_cmd_fs;
                }
            }

            return joint_cmd_fs;
        }

        bool connected() override {
            return impedance_port.connected();
        }

        Eigen::VectorXf &value() override {
            return conversion(impedance_cmd, position_cmd, torque_cmd);
        }

        std::function<Eigen::VectorXf& (rstrt::dynamics::JointImpedance&, rstrt::kinematics::JointAngles&, rstrt::dynamics::JointTorques&)> conversion;

        RTT::InputPort<rstrt::dynamics::JointImpedance> impedance_port;
        RTT::FlowStatus impedance_cmd_fs;
        rstrt::dynamics::JointImpedance impedance_buf, impedance_cmd;

        RTT::InputPort<rstrt::kinematics::JointAngles> position_port;
        RTT::FlowStatus position_cmd_fs;
        rstrt::kinematics::JointAngles position_buf, position_cmd;

        RTT::InputPort<rstrt::dynamics::JointTorques> torque_port;
        RTT::FlowStatus torque_cmd_fs;
        rstrt::dynamics::JointTorques torque_buf, torque_cmd;
    };

    template < class T > class JointFeedback {
        public:
            JointFeedback(std::string name, RTT::DataFlowInterface &ports, std::string feedback_name, std::function < void(T &) > initalization) {
                orocos_port.setName(name + "_" + feedback_name);
                orocos_port.doc("Output for " + feedback_name + " messages from Franka to Orocos.");
                ports.addPort(orocos_port);
                initalization(joint_feedback);
                orocos_port.setDataSample(joint_feedback);
            }

            ~JointFeedback() {}

            T joint_feedback;
            RTT::OutputPort < T > orocos_port;

            void write() {
                orocos_port.write(joint_feedback);
            }

            bool connected() {
                return orocos_port.connected();
            }
    };

    template < class T > class DynamicFeedback {
        public:
            DynamicFeedback(std::string name, RTT::DataFlowInterface &ports, std::string feedback_name, std::function < void(T &) > initalization) {
                orocos_port.setName(name + "_" + feedback_name);
                orocos_port.doc("Output for " + feedback_name + " messages from Franka to Orocos.");
                ports.addPort(orocos_port);
                initalization(dynamicFeedback);
                orocos_port.setDataSample(dynamicFeedback);
            }

            ~DynamicFeedback() {}

            T dynamicFeedback;
            RTT::OutputPort < T > orocos_port;

            void write() {
                orocos_port.write(dynamicFeedback);
            }

            bool connected() {
                return orocos_port.connected();
            }
    };
}
#endif
