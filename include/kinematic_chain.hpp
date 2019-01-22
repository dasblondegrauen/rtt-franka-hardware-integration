#ifndef _KINEMATIC_FRANKA_CHAIN_H_
#define _KINEMATIC_FRANKA_CHAIN_H_

// RST-RT includes
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/robot/JointState.hpp>

#include <robot_control.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <control_modes.hpp>
#include <robot_impl.h>
#include <motion_generator_traits.h>
#include <research_interface/robot/rbk_types.h>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <memory>
#include <array>

class KinematicChain {
    public:
        /**
         * Contructor to set up a kinematic chain.
         *
         * @param chain_name name of the chain.
         * @param joint_names list o joint names.
         * @param ports pointer to the ports of the enclosing component.
         * @param friInst remote interface of FRI.
         */
        KinematicChain(const std::string &chain_name,
                       const std::vector < std::string > &joint_names,
                       RTT::DataFlowInterface &ports,
                       std::unique_ptr < franka::Robot::Impl > franka_control);

        ~KinematicChain() {
            // delete motion_command;
            // delete control_command;
        }

        /**
         * Gets the name of the kinematic chain
         * @return Name of the kinematic chain
         */
        std::string getKinematicChainName();

        /**
         * Gets the Degree of Freedom of the kinematic chain
         * @return DoF of the kinematic chain
         */
        unsigned int getNumberOfDOFs();

        /**
         * Gets the current control mode of the kinematic chain
         * @return Current control mode of the kinematic chain as string
         */
        const std::string &getCurrentControlMode();

        // std::vector < std::string > &getJointNames();

        /**
         * Detects available controllers of the robot
         * @return Available controllers as vector of strings
         */
        std::vector < std::string > getControllersAvailable() {
            std::vector < std::string > output;
            for (auto const &x : franka::ControlModeMap) {
                output.push_back(x.second);
            }
            return output;
        }

        /**
         * Initializes the kinematic chain
         * @return true if successful, false otherwise
         */
        bool initKinematicChain();

        /**
         * Resets the kinematic chain
         * @return true if successful, false otherwise
         */
        bool resetKinematicChain();

        /**
         * Starts the kinematic chain.
         * Initializes a motion as well as motion/control commands according to the current control mode
         * @return true if successful, false otherwise
         */
        bool startKinematicChain();

        /**
         * Initializes feedback ports.
         * Gets called automatically by initKinematicChain()
         */
        void setFeedBack();

        /**
         * Set control mode of the kinematic chain.
         * Has to be called before the chain is started by startKinematicChain()
         * @return true if successful, false otherwise
         */
        bool setControlMode(const std::string &controlMode);

        /**
         * Receive current state of robot and write to feedback ports
         * @return true if successful, false otherwise
         */
        bool sense();

        /**
         * Read command from JointController/input port and transfer to current_control_input_var
         */
        void getCommand();

        /**
         * Try to execute the previously receibed motion/control command by sending it to the robot
         * May throw franka::NetworkException, franka::ControlException, franka::ProtocolException on error.
         */
        void move();

        /**
         * Try to finish/abort current motion and stop the kinematic chain.
         * May throw any exception occuring during that process.
         */
        void stop();

        /**
         * Print information about the kinematic chain into a string
         * @return A String with information about the kinematic chain
         */
        std::string printKinematicChainInformation();

        /**
         * Set internal collision detection thresholds
         */
	void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                                 const std::array<double, 7>& upper_torque_thresholds,
                                 const std::array<double, 6>& lower_force_thresholds,
                                 const std::array<double, 6>& upper_force_thresholds);

        /**
         * Set internal joint and cartesion impedance values.
         * Only affects operation in position and velocity modes
         */
        void setImpedanceBehavior(const std::array<double,7> & joint_imp,const std::array<double,6> & cart_imp);

    private:
        std::string kinematic_chain_name;
        int dof;
        franka::ControlModes current_control_mode;
        RTT::DataFlowInterface &ports;
        std::unique_ptr < franka::RobotControl > franka_control;
        static constexpr research_interface::robot::Move::Deviation kDefaultDeviation {
            10.0, 3.12, 2 * M_PI
        };
        research_interface::robot::MotionGeneratorCommand motion_command;
        research_interface::robot::ControllerCommand control_command;

        franka::RobotState franka_state;
        std::unique_ptr < franka::Model > franka_model;

        std::unique_ptr < franka::BaseJointController > jc;
        std::unique_ptr < franka::JointFeedback < rstrt::robot::JointState >> jf;
        std::unique_ptr < franka::DynamicFeedback < Eigen::MatrixXf >> inertia_feedback;
        std::unique_ptr < franka::DynamicFeedback < Eigen::VectorXf >> coriolis_feedback;
        std::unique_ptr < franka::DynamicFeedback < Eigen::VectorXf >> gravity_feedback;
        
        std::unique_ptr < franka::DynamicFeedback < Eigen::MatrixXf >> jacobian_feedback;
        std::array < double, 7 > *current_control_input_var;
        uint32_t motion_id;

        Eigen::VectorXf& convertImpedance(rstrt::dynamics::JointImpedance& impedance_input,
                                          rstrt::kinematics::JointAngles& position_input,
                                          rstrt::dynamics::JointTorques& torque_input);
        rstrt::dynamics::JointTorques tau_buffer;
};

#endif
