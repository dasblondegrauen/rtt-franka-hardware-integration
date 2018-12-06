#ifndef _CONTROL_MODES_H_
#define _CONTROL_MODES_H_

#include <string>
#include <rtt/Port.hpp>
#include <rst-rt/dynamics/JointImpedance>
#include <rst-rt/dynamics/JointTorque>
#include <rst-rt/kinematics/JointAngles>

namespace franka {
    enum ControlModes {Position, Velocity, Torque, Impedance};
    static const std::map < ControlModes, std::string > ControlModeMap = {
        {Position, "JointPositionCtrl"},
        {Velocity, "JointVelocityCtrl"},
        {Torque, "JointTorqueCtrl"},
        {Impedance, "JointImpedanceCtrl"}
    };

    class BaseJointController {
        public:
            virtual RTT::FlowStatus &read() = 0;
            virtual bool connected() = 0;
            virtual Eigen::VectorXf &value() = 0;
            RTT::FlowStatus joint_cmd_fs;
    };

    template < class T > class JointController: public BaseJointController {
        public:
            JointController(const std::string &name, RTT::DataFlowInterface &ports, const ControlModes &control_name, std::function < Eigen::VectorXf & (T &) > conversion_in) : conversion(conversion_in) {
                orocos_port.setName(name + "_" + ControlModeMap.find(control_name)->second);
                orocos_port.doc("Input for " + ControlModeMap.find(control_name)->second + "-cmds from Orocos to Franka.");
                joint_cmd_fs = RTT::NoData;
                ports.addPort(orocos_port);
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

    class JointImpedanceController: public BaseJointController {
        public:
            JointImpedanceController(const std::string &name,
                    RTT::DataFlowInterface &ports,
                    const ControlModes &control_name,
                    std::function<Eigen::VectorXf& (rstrt::dynamics::JointImpedance&, rstrt::kinematics::JointAngles&, rstrt::dynamics::JointTorque&)> conversion_in)
                : conversion(conversion_in) {
                    // TODO
                }

            ~JointImpedanceController() {}

            RTT::FlowStatus &read() override {
                // TODO
                return RTT::NoData;
            }

            bool connected() override {
                // TODO
                return false;
            }

            Eigen::VectorXf &value() override {
                // TODO
                return nullptr;
            }
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
