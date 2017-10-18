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

#include <RobotInterfaceOROCOS/RobotInterfaceOROCOS.h>
#include <rtt/Logger.hpp>


SHLIBPP_DEFINE_SHARED_SUBCLASS(robot_interface_orocos, XBot::RobotInterfaceOROCOS, XBot::RobotInterface);

using namespace rstrt::dynamics;
using namespace rstrt::kinematics;
using namespace rstrt::robot;

bool XBot::RobotInterfaceOROCOS::attachToRobot(const std::string &robot_name, const std::string &config_path,
                                         XBot::RobotInterface::Ptr _robot,
                                         std::shared_ptr<RTT::TaskContext> task)
{
    RTT::log(RTT::Info)<<"Robot name: "<<robot_name<<RTT::endlog();

    std::shared_ptr<RTT::TaskContext> task_ptr(task->getPeer(robot_name));
    if(!task_ptr){
        RTT::log(RTT::Error)<<"Can not getPeer("<<robot_name<<")"<<RTT::endlog();
        return false;}

    std::shared_ptr<std::map<std::string, boost::any >> anymap(new std::map<std::string, boost::any >);
    (*anymap)["TaskContextPtr"] = task;
    (*anymap)["TaskPeerContextPtr"] = task_ptr;


    _robot = XBot::RobotInterface::getRobot(config_path, anymap);
    if(_robot)
        return true;
    RTT::log(RTT::Error)<<"CAN NOT LOAD ROBOT INTERFACE OROCOS"<<RTT::endlog();
    return false;

}

bool XBot::RobotInterfaceOROCOS::init_robot(const std::string &path_to_cfg, AnyMapConstPtr any_map)
{
    RTT::log(RTT::Info)<<"Constructing OROCOS implementation of RobotInterface!"<<RTT::endlog();

    _q.setZero(this->getJointNum());
    _qdot.setZero(this->getJointNum());
    _tau.setZero(this->getJointNum());

    if(!any_map){
        RTT::log(RTT::Error) << "ERROR in " << __func__ << "! TBD explain what to do!" << RTT::endlog();
        return false;
    }

    if( any_map->count("TaskContextPtr") ){
        try{
            _task_ptr = boost::any_cast<std::shared_ptr<RTT::TaskContext> >(any_map->at("TaskContextPtr"));
            RTT::log(RTT::Info) << "TaskContextPtr found!" << RTT::endlog();
        }
        catch(...){
            RTT::log(RTT::Error) << "ERROR in " << __func__ << "! Invalid object with key \"TaskContextPtr\" inside the parameter map given as a second argument to getRobot()! Make sure it is a std::shared_ptr<RTT::TaskContext>." << RTT::endlog();
            return false;
        }
    }

    if( any_map->count("TaskPeerContextPtr") ){
        try{
            _task_peer_ptr = boost::any_cast<std::shared_ptr<RTT::TaskContext> >(any_map->at("TaskPeerContextPtr"));
            RTT::log(RTT::Info) << "TaskPeerContextPtr found!" << RTT::endlog();
        }
        catch(...){
            RTT::log(RTT::Error) << "ERROR in " << __func__ << "! Invalid object with key \"TaskPeerContextPtr\" inside the parameter map given as a second argument to getRobot()! Make sure it is a std::shared_ptr<RTT::TaskContext>." << RTT::endlog();
            return false;
        }
    }

    //FIRST KINEMATICH CHAINS
    RTT::OperationCaller<std::map<std::string, std::vector<std::string> >(void) > getKinematicChainsAndJoints
            = _task_peer_ptr->getOperation("getKinematicChainsAndJoints");

    _map_kin_chains_joints = getKinematicChainsAndJoints();

    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::string kin_chain_name = it->first;
        std::vector<std::string> joint_names = it->second;

        //1) Feedback from joints
        _kinematic_chains_feedback_ports[kin_chain_name] =
            boost::shared_ptr<RTT::InputPort<JointState> >(new RTT::InputPort<JointState>(
                            kin_chain_name+"_"+"JointFeedback"));

        _task_ptr->addPort(*(_kinematic_chains_feedback_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointFeedback port");

        _kinematic_chains_feedback_ports.at(kin_chain_name)->connectTo(
                    _task_peer_ptr->ports()->getPort(kin_chain_name+"_"+"JointFeedback"));

        JointState tmp(joint_names.size());
        _kinematic_chains_joint_state_map[kin_chain_name] = tmp;
        RTT::log(RTT::Info)<<"Added "<<kin_chain_name<<" port and data"<<RTT::endlog();


        //2) Position Ctrl
        _kinematic_chains_output_ports[kin_chain_name] =
                boost::shared_ptr<RTT::OutputPort<JointAngles> >(new RTT::OutputPort<JointAngles>(
                                kin_chain_name+"_"+"JointPositionCtrl"));
        _task_ptr->addPort(*(_kinematic_chains_output_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointPositionCtrl port");
        _kinematic_chains_output_ports.at(kin_chain_name)->connectTo(
                    _task_peer_ptr->ports()->getPort(kin_chain_name+"_"+"JointPositionCtrl"));
        JointAngles tmp2(joint_names.size());
        _kinematic_chains_desired_joint_state_map[kin_chain_name] = tmp2;

        //3) Impedance Ctrl
        //...

        //4) Torque Ctrl
        //...
    }

    //SECOND FT SENSORS
    RTT::OperationCaller<std::vector<std::string> (void) > getForceTorqueSensorsFrames
        = _task_peer_ptr->getOperation("getForceTorqueSensorsFrames");
    std::vector<std::string> ft_sensors_frames = getForceTorqueSensorsFrames();
    for(unsigned int i = 0; i < ft_sensors_frames.size(); ++i)
    {
        _frames_ports_map[ft_sensors_frames[i]] =
                boost::shared_ptr<RTT::InputPort<Wrench> >(new RTT::InputPort<Wrench>(
                        ft_sensors_frames[i]+"_SensorFeedback"));
        _task_ptr->addPort(*(_frames_ports_map.at(ft_sensors_frames[i]))).
                doc(ft_sensors_frames[i]+"_SensorFeedback port");

        _frames_ports_map.at(ft_sensors_frames[i])->connectTo(
                    _task_peer_ptr->ports()->getPort(ft_sensors_frames[i]+"_SensorFeedback"));

        Wrench tmp;
        _frames_wrenches_map[ft_sensors_frames[i]] = tmp;

        RTT::log(RTT::Info)<<"Added "<<ft_sensors_frames[i]<<" port and data"<<RTT::endlog();
    }

    return true;
}



bool XBot::RobotInterfaceOROCOS::sense_internal()
{
    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        RTT::FlowStatus fs = _kinematic_chains_feedback_ports.at(it->first)->read(
                    _kinematic_chains_joint_state_map.at(it->first));

        for(unsigned int i = 0; i < it->second.size(); ++i)
        {
            _q[this->getDofIndex(it->second.at(i))] =
                    _kinematic_chains_joint_state_map.at(it->first).angles[i];

            _qdot[this->getDofIndex(it->second.at(i))] =
                    _kinematic_chains_joint_state_map.at(it->first).velocities[i];

            _tau[this->getDofIndex(it->second.at(i))] =
                    _kinematic_chains_joint_state_map.at(it->first).torques[i];
        }
    }

    bool success = true;

    // set sensed positions inside RobotInterface
    success = this->setJointPosition(_q) && success;
    success = this->setJointVelocity(_qdot) && success;
    success = this->setMotorPosition(_q) && success;
    success = this->setMotorVelocity(_qdot) && success;
    success = this->setJointEffort(_tau) && success;
//    success = this->setStiffness(_jointstate_msg_map_stiffness) && success;
//    success = this->setDamping(_jointstate_msg_map_damping) && success;

    return success;

}

double XBot::RobotInterfaceOROCOS::getTime() const
{
    return RTT::nsecs_to_Seconds(this->getNSecs());
}

bool XBot::RobotInterfaceOROCOS::isRunning() const
{
    return _task_ptr->isRunning();
}

bool XBot::RobotInterfaceOROCOS::move_internal(){return false;}
bool XBot::RobotInterfaceOROCOS::read_sensors(){return false;}
bool XBot::RobotInterfaceOROCOS::move_hands(){return false;}
bool XBot::RobotInterfaceOROCOS::sense_hands(){ return false;}
bool XBot::RobotInterfaceOROCOS::set_control_mode_internal ( int joint_id, const ControlMode& control_mode )
{
    return false;
}


