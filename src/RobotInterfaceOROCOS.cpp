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


REGISTER_SO_LIB_(XBot::RobotInterfaceOROCOS, XBot::RobotInterface);

using namespace rstrt::dynamics;
using namespace rstrt::kinematics;
using namespace rstrt::robot;

bool XBot::RobotInterfaceOROCOS::attachToRobot(const string &robot_name, const string &config_path,
                                         RobotInterface::Ptr& _robot,
                                         shared_ptr<TaskContext> task)
{
    LOG(Info)<<"Robot name: "<<robot_name<<ENDLOG();

    shared_ptr<TaskContext> task_ptr(task->getPeer(robot_name));
    if(!task_ptr){
        LOG(Error)<<"Can not getPeer("<<robot_name<<")"<<ENDLOG();
        return false;}

    shared_ptr<map<string, boost::any >> anymap(new map<string, boost::any >);
    (*anymap)["TaskContextPtr"] = task;
    (*anymap)["TaskPeerContextPtr"] = task_ptr;


    _robot = XBot::RobotInterface::getRobot(config_path,robot_name, anymap);
    if(_robot)
    {
        LOG(Warning)<<"ROBOT LOADED IN ROBOT INTERFACE OROCOS"<<ENDLOG();
        return true;
    }
    LOG(Error)<<"CAN NOT LOAD ROBOT INTERFACE OROCOS"<<ENDLOG();
    return false;

}

bool XBot::RobotInterfaceOROCOS::init_robot(const string &path_to_cfg, AnyMapConstPtr any_map)
{
    LOG(Info)<<"Constructing OROCOS implementation of RobotInterface!"<<ENDLOG();
    LOG(Info)<<"Robot has "<<this->getJointNum()<<" dofs"<<ENDLOG();

    _q.setZero(this->getJointNum());
    _qdot.setZero(this->getJointNum());
    _tau.setZero(this->getJointNum());

    _q_ref.setZero(this->getJointNum());
    _tau_ref.setZero(this->getJointNum());
    _stiffness_ref.setZero(this->getJointNum());
    _damping_ref.setZero(this->getJointNum());

    if(!any_map){
        LOG(Error) << "ERROR in " << __func__ << "! TBD explain what to do!" << ENDLOG();
        return false;
    }

    if( any_map->count("TaskContextPtr") ){
        try{
            _task_ptr = boost::any_cast<shared_ptr<TaskContext> >(any_map->at("TaskContextPtr"));
            LOG(Info) << "TaskContextPtr found!" << ENDLOG();
        }
        catch(...){
            LOG(Error) << "ERROR in " << __func__ << "! Invalid object with key \"TaskContextPtr\" inside the parameter map given as a second argument to getRobot()! Make sure it is a shared_ptr<TaskContext>." << ENDLOG();
            return false;
        }
    }

    if( any_map->count("TaskPeerContextPtr") ){
        try{
            _task_peer_ptr = boost::any_cast<shared_ptr<TaskContext> >(any_map->at("TaskPeerContextPtr"));
            LOG(Info) << "TaskPeerContextPtr found!" << ENDLOG();
        }
        catch(...){
            LOG(Error) << "ERROR in " << __func__ << "! Invalid object with key \"TaskPeerContextPtr\" inside the parameter map given as a second argument to getRobot()! Make sure it is a shared_ptr<TaskContext>." << ENDLOG();
            return false;
        }
    }

    //FIRST KINEMATICH CHAINS
    OperationCaller<map<KinematicChainName, JointNames >(void) > getKinematicChainsAndJoints
            = _task_peer_ptr->getOperation("getKinematicChainsAndJoints");

    _map_kin_chains_joints = getKinematicChainsAndJoints();

    map<KinematicChainName, JointNames >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        KinematicChainName kin_chain_name = it->first;
        JointNames joint_names = it->second;

        //1) Feedback from joints
        _joint_feedback[kin_chain_name] = JointFeedback::Ptr(
                new JointFeedback("JointFeedback", kin_chain_name, joint_names.size(),
                                  _task_ptr, _task_peer_ptr));

        //2) Position Ctrl
        _position_ctrl[kin_chain_name] = JointPositionController::Ptr(
            new JointPositionController(ControlModes::JointPositionCtrl,kin_chain_name,
                                        joint_names.size(), _task_ptr, _task_peer_ptr));


        //3) Impedance Ctrl
        //...

        //4) Torque Ctrl
        _torque_ctrl[kin_chain_name] = JointTorqueController::Ptr(
            new JointTorqueController(ControlModes::JointTorqueCtrl, kin_chain_name,
                                      joint_names.size(), _task_ptr, _task_peer_ptr));

        LOG(Info)<<"Added "<<kin_chain_name<<" port and data"<<ENDLOG();

    }

    //SECOND FT SENSORS
    OperationCaller<vector<ForceTorqueFrame> (void) > getForceTorqueSensorsFrames
        = _task_peer_ptr->getOperation("getForceTorqueSensorsFrames");
    vector<ForceTorqueFrame> ft_sensors_frames = getForceTorqueSensorsFrames();
    for(unsigned int i = 0; i < ft_sensors_frames.size(); ++i)
    {
        _frames_ports_map[ft_sensors_frames[i]] =
                WrenchIPort_Ptr(new WrenchIPort(ft_sensors_frames[i]+"_SensorFeedback"));
        _task_ptr->addPort(*(_frames_ports_map.at(ft_sensors_frames[i]))).
                doc(ft_sensors_frames[i]+"_SensorFeedback port");

        _frames_ports_map.at(ft_sensors_frames[i])->connectTo(
                    _task_peer_ptr->ports()->getPort(ft_sensors_frames[i]+"_SensorFeedback"));

        Wrench tmp;
        _frames_wrenches_map[ft_sensors_frames[i]] = tmp;

        LOG(Info)<<"Added "<<ft_sensors_frames[i]<<" port and data"<<ENDLOG();
    }


    //Extra stuffs
    _getControlMode = _task_peer_ptr->getOperation("getControlMode");

    return true;
}



bool XBot::RobotInterfaceOROCOS::sense_internal()
{
    map<KinematicChainName, JointNames >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        FlowStatus fs = _joint_feedback.at(it->first)->read();

        for(unsigned int i = 0; i < it->second.size(); ++i)
        {
            _q[this->getDofIndex(it->second.at(i))] =
                    _joint_feedback.at(it->first)->feedback.angles[i];

            _qdot[this->getDofIndex(it->second.at(i))] =
                    _joint_feedback.at(it->first)->feedback.velocities[i];

            _tau[this->getDofIndex(it->second.at(i))] =
                    _joint_feedback.at(it->first)->feedback.torques[i];
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
    return nsecs_to_Seconds(this->getNSecs());
}

bool XBot::RobotInterfaceOROCOS::isRunning() const
{
    return _task_ptr->isRunning();
}

bool XBot::RobotInterfaceOROCOS::move_internal()
{
    // For now position ctrl
    this->getPositionReference(_q_ref);
    this->getEffortReference(_tau_ref);
    this->getStiffness(_stiffness_ref);
    this->getDamping(_damping_ref);

    map<KinematicChainName, JointNames >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        _control_mode = _getControlMode(it->first);

        if(_control_mode.compare(ControlModes::JointPositionCtrl) == 0 ||
           _control_mode.compare(ControlModes::JointImpedanceCtrl) == 0)
        {
            for(unsigned int i = 0; i < it->second.size(); ++i)
                _position_ctrl.at(it->first)->cmd.angles[i] =
                        _q_ref[this->getDofIndex(it->second.at(i))];
            _position_ctrl.at(it->first)->write();
        }
        if(_control_mode.compare(ControlModes::JointTorqueCtrl) == 0 ||
           _control_mode.compare(ControlModes::JointImpedanceCtrl) == 0)
        {
            for(unsigned int i = 0; i < it->second.size(); ++i)
                _torque_ctrl.at(it->first)->cmd.torques[i] =
                        _tau_ref[this->getDofIndex(it->second.at(i))];
            _torque_ctrl.at(it->first)->write();
        }
        if(_control_mode.compare(ControlModes::JointImpedanceCtrl) == 0)
        {

        }

    }

    return true;
}

bool XBot::RobotInterfaceOROCOS::read_sensors()
{
    bool success = true;

    //For now just FT sensors
    map<ForceTorqueFrame, Wrench>::iterator it2;
    for(it2 = _frames_wrenches_map.begin(); it2 != _frames_wrenches_map.end(); it2++)
    {
        FlowStatus fs = _frames_ports_map.at(it2->first)->read(
                    _frames_wrenches_map.at(it2->first));

        auto it = getForceTorqueInternal().find(it2->first);

        if( it != getForceTorqueInternal().end() )
            _ftptr = it->second;
        if(!_ftptr){
            LOG(Error) << "WARNING in " << __func__ << ": no sensor corresponding to link " << it2->first << " is present in given URDF/SRDF! Check that FTmsg.header.frame_id is the parent link of the sensor name in URDF! Also check that FT fixed joints are defined inside the force_torque_sensors SRDF group" << ENDLOG();
            success = false;
        }
        else{
            _tmp_vector6.setZero();
            _tmp_vector6<<_frames_wrenches_map.at(it2->first).forces.cast <double> (), _frames_wrenches_map.at(it2->first).torques.cast <double> ();
            getForceTorqueInternal().at(_ftptr->getSensorName())->setWrench(
                _tmp_vector6,getTime());
        }
    }

    return success;
}

bool XBot::RobotInterfaceOROCOS::move_hands()
{
    LOG(Debug)<<"move_hands() is not yet implemented!"<<ENDLOG();
    return false;
}

bool XBot::RobotInterfaceOROCOS::sense_hands()
{
    LOG(Debug)<<"sense_hands() is not yet implemented!"<<ENDLOG();
    return false;
}

bool XBot::RobotInterfaceOROCOS::set_control_mode_internal ( int joint_id, const ControlMode& control_mode )
{
    LOG(Debug)<<"set_control_mode_internal(...) is not yet implemented!"<<ENDLOG();
    return false;
}


