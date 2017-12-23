/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef XBOT_ROBOTINTERFACE_OROCOS_H
#define XBOT_ROBOTINTERFACE_OROCOS_H

#include <XBotInterface/RobotInterface.h>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <std_msgs/Float64.h>

#include <XBotInterface/SoLib.h>

#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace std;
using namespace RTT;
using namespace Eigen;
using namespace rstrt;

#define LOG(x) RTT::log(x)
#define ENDLOG() RTT::endlog()

namespace XBot
{
///TODO: these are copied from rtt-gazebo-robot-sim, should be aligned with the one of XBotCore
struct ControlModes{
        static constexpr const char* JointPositionCtrl = "JointPositionCtrl";
        static constexpr const char* JointTorqueCtrl = "JointTorqueCtrl";
        static constexpr const char* JointImpedanceCtrl = "JointImpedanceCtrl";
};

template <class T> class Sensor {
public:
    typedef boost::shared_ptr<Sensor<T> > Ptr;

    Sensor(const string& feedback_name,
           const string& kinematic_chain_name,
           shared_ptr<TaskContext> task_ptr,
           shared_ptr<TaskContext> task_peer)
    {
        init(feedback_name, kinematic_chain_name, task_ptr, task_peer);
    }

    Sensor(const string& feedback_name,
           const string& kinematic_chain_name,
           const int size,
           shared_ptr<TaskContext> task_ptr,
           shared_ptr<TaskContext> task_peer):
           feedback(size)
    {
        init(feedback_name, kinematic_chain_name, task_ptr, task_peer);
    }

    ~Sensor(){

    }

    FlowStatus read()
    {
        return port->read(feedback);
    }

    boost::shared_ptr<InputPort<T> > port;
    T feedback;

private:
    void init(const string& feedback_name,
              const string& kinematic_chain_name,
              shared_ptr<TaskContext> task_ptr,
              shared_ptr<TaskContext> task_peer)
    {
        port.reset(new InputPort<T>(kinematic_chain_name + feedback_name));
        task_ptr->addPort(port->doc(kinematic_chain_name+"_"+feedback_name + " port"));
        if(port->connectTo(task_peer->ports()->getPort(kinematic_chain_name+"_"+feedback_name)))
            LOG(Info)<<"Connected to port: "<<kinematic_chain_name+"_"+feedback_name<<ENDLOG();
        else
        {
            LOG(Error)<<"Can not connect port: "<<kinematic_chain_name+"_"+feedback_name<<ENDLOG();
            throw std::runtime_error("Error during port connect");
        }
    }
};

template <class T> class JointCtrl {
public:
    typedef boost::shared_ptr<JointCtrl<T> > Ptr;

    JointCtrl(const string& control_mode,
              const string& kinematic_chain_name,
              const int size,
              shared_ptr<TaskContext> task_ptr,
              shared_ptr<TaskContext> task_peer):
        cmd(size)
    {
        port.reset(new OutputPort<T>(kinematic_chain_name + control_mode));
        task_ptr->addPort(port->doc(kinematic_chain_name+"_"+control_mode + " port"));
        if(port->connectTo(task_peer->ports()->getPort(kinematic_chain_name+"_"+control_mode)))
            LOG(Info)<<"Connected to port: "<<kinematic_chain_name+"_"+control_mode<<ENDLOG();
        else
        {
            LOG(Error)<<"Can not connect port: "<<kinematic_chain_name+"_"+control_mode<<ENDLOG();
            throw std::runtime_error("Error during port connect");
        }
    }

    ~JointCtrl(){

    }

    void write()
    {
        port->write(cmd);
    }

    boost::shared_ptr<OutputPort<T> > port;
    T cmd;
};



struct FeedbackModes {
    static constexpr const char* velocityFeedback = "JointVelocity";
    static constexpr const char* torqueFeedback = "JointTorque";
    static constexpr const char* positionFeedback = "JointPosition";
};

class RobotInterfaceOROCOS : public RobotInterface, os::TimeService
{

    friend RobotInterface;

public:
    typedef JointCtrl<kinematics::JointAngles> JointPositionController;
    typedef JointCtrl<dynamics::JointTorques> JointTorqueController;
    typedef Sensor<robot::JointState> JointFeedback;
    typedef Sensor<dynamics::Wrench> ForceTorqueFeedback;

    typedef string KinematicChainName;
    typedef string JointName;
    typedef vector<string> JointNames;
    typedef string ForceTorqueFrame;


    RobotInterfaceOROCOS() {}

    virtual bool set_control_mode_internal ( int joint_id, const ControlMode& control_mode );

    virtual double getTime() const;

    virtual bool isRunning() const;

    static bool attachToRobot(const string &robot_name, const string &config_path,
                              RobotInterface::Ptr& _robot, shared_ptr<TaskContext> task);

protected:

    virtual bool init_robot(const string &path_to_cfg, AnyMapConstPtr any_map);
    virtual bool move_internal();
    virtual bool sense_internal();
    virtual bool read_sensors();
    virtual bool move_hands();
    virtual bool sense_hands();

private:
    shared_ptr<TaskContext> _task_ptr;
    shared_ptr<TaskContext> _task_peer_ptr;

    map<KinematicChainName, JointNames > _map_kin_chains_joints;

    map<KinematicChainName, JointPositionController::Ptr> _position_ctrl;
    map<KinematicChainName, JointTorqueController::Ptr> _torque_ctrl;

    map<KinematicChainName, JointFeedback::Ptr> _joint_feedback;
    map<ForceTorqueFrame, ForceTorqueFeedback::Ptr> _ft_feedback;

    //For now these variable are motor side AND link side
    VectorXd _q;
    VectorXd _qdot;
    VectorXd _tau;

    VectorXd _q_ref;
    VectorXd _tau_ref;
    VectorXd _stiffness_ref;
    VectorXd _damping_ref;


    ForceTorqueSensor::ConstPtr _ftptr;
    Vector6d _tmp_vector6;

};

}
#endif
