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

bool XBot::RobotInterfaceOROCOS::attachToRobot(const string &robot_name,
                                               const string &path_to_urdf,
                                               const string &path_to_srdf,
                                               const bool is_robot_floating_base,
                                               const string &model_type,
                                         RobotInterface::Ptr& _robot,
                                         shared_ptr<TaskContext> task)
{
    LOG(Info)<<"Robot name: "<<robot_name<<ENDLOG();

    XBot::ConfigOptions cfg;
    cfg.set_urdf_path(path_to_urdf);
    cfg.set_srdf_path(path_to_srdf);
    cfg.generate_jidmap();
    std::string framework = "OROCOS";
    cfg.set_parameter("framework", framework);
    cfg.set_parameter("is_model_floating_base", is_robot_floating_base);
    cfg.set_parameter("model_type", model_type);

    shared_ptr<TaskContext> task_ptr(task->getPeer(robot_name));
    if(!task_ptr){
        LOG(Error)<<"Can not getPeer("<<robot_name<<")"<<ENDLOG();
        return false;}

    cfg.set_parameter("TaskContextPtr", task);
    cfg.set_parameter("TaskPeerContextPtr", task_ptr);

    _robot = XBot::RobotInterface::getRobot(cfg, robot_name);
    if(_robot)
    {
        LOG(Warning)<<"ROBOT LOADED IN ROBOT INTERFACE OROCOS"<<ENDLOG();
        return true;
    }
    LOG(Error)<<"CAN NOT LOAD ROBOT INTERFACE OROCOS"<<ENDLOG();
    return false;

}

bool XBot::RobotInterfaceOROCOS::init_robot(const XBot::ConfigOptions& cfg)
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

    if( !cfg.get_parameter("TaskContextPtr",_task_ptr)){
        LOG(Error) << "ERROR in " << __func__ << "! Invalid object with key \"TaskContextPtr\" inside the parameter map given as a second argument to getRobot()! Make sure it is a shared_ptr<TaskContext>." << ENDLOG();
        return false;
    }
    LOG(Info) << "TaskContextPtr found!" << ENDLOG();

    if(!cfg.get_parameter("TaskPeerContextPtr",_task_peer_ptr)){
        LOG(Error) << "ERROR in " << __func__ << "! Invalid object with key \"TaskPeerContextPtr\" inside the parameter map given as a second argument to getRobot()! Make sure it is a shared_ptr<TaskContext>." << ENDLOG();
        return false;
    }
    LOG(Info) << "TaskPeerContextPtr found!" << ENDLOG();

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
        try{_joint_feedback[kin_chain_name] = JointFeedback::Ptr(
                new JointFeedback("JointFeedback", kin_chain_name, joint_names.size(),
                                  _task_ptr, _task_peer_ptr));
        }catch(...){return false;}

        //2) Position Ctrl
        try{_position_ctrl[kin_chain_name] = JointPositionController::Ptr(
            new JointPositionController(ControlModes::JointPositionCtrl,kin_chain_name,
                                        joint_names.size(), _task_ptr, _task_peer_ptr));
        }catch(...){return false;}

        //3) Impedance Ctrl
        //...

        //4) Torque Ctrl
        try{_torque_ctrl[kin_chain_name] = JointTorqueController::Ptr(
            new JointTorqueController(ControlModes::JointTorqueCtrl, kin_chain_name,
                                      joint_names.size(), _task_ptr, _task_peer_ptr));
        }catch(...){return false;}

        LOG(Info)<<"Added "<<kin_chain_name<<" port and data"<<ENDLOG();

    }

    //SECOND FT SENSORS
    OperationCaller<vector<ForceTorqueFrame> (void) > getForceTorqueSensorsFrames
        = _task_peer_ptr->getOperation("getForceTorqueSensorsFrames");
    vector<ForceTorqueFrame> ft_sensors_frames = getForceTorqueSensorsFrames();
    for(unsigned int i = 0; i < ft_sensors_frames.size(); ++i)
    {
        try{_ft_feedback[ft_sensors_frames[i]] = ForceTorqueFeedback::Ptr(
            new ForceTorqueFeedback("SensorFeedback", ft_sensors_frames[i],
                                    _task_ptr, _task_peer_ptr));
        }catch(...){return false;}

        LOG(Info)<<"Added "<<ft_sensors_frames[i]<<" port and data"<<ENDLOG();
    }

    //THIRD IMU SENSORS
    OperationCaller<vector<IMUFrame> (void) > getImuSensorsFrames
        = _task_peer_ptr->getOperation("getIMUSensorsFrames") ;
    vector<IMUFrame> imu_sensors_frames = getImuSensorsFrames();
    for(unsigned int i = 0; i < imu_sensors_frames.size(); ++i)
    {
        try{_imu_feedback[imu_sensors_frames[i]] = IMUFeedback::Ptr(
            new IMUFeedback("SensorFeedback", imu_sensors_frames[i],
                            _task_ptr, _task_peer_ptr));

        }catch(...){return false;}

        LOG(Info)<<"Added "<<imu_sensors_frames[i]<<" port and data"<<ENDLOG();
    }

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

            for(unsigned int i = 0; i < it->second.size(); ++i)
            {
                _position_ctrl.at(it->first)->cmd.angles[i] =
                        _q_ref[this->getDofIndex(it->second.at(i))];
                _torque_ctrl.at(it->first)->cmd.torques[i] =
                        _tau_ref[this->getDofIndex(it->second.at(i))];
            }


            _position_ctrl.at(it->first)->write();
            _torque_ctrl.at(it->first)->write();


    }

    return true;
}

bool XBot::RobotInterfaceOROCOS::read_sensors()
{
    bool success = true;

    map<ForceTorqueFrame, ForceTorqueFeedback::Ptr>::iterator it2;
    for(it2 = _ft_feedback.begin(); it2 != _ft_feedback.end(); it2++)
    {
        FlowStatus fs = _ft_feedback.at(it2->first)->read();

        auto it = getForceTorqueInternal().find(it2->first);

        if( it != getForceTorqueInternal().end() )
            _ftptr = it->second;
        if(!_ftptr){
            LOG(Error) << "WARNING in " << __func__ << ": no sensor corresponding to link " << it2->first << " is present in given URDF/SRDF! Check that FT msg.header.frame_id is the parent link of the sensor name in URDF! Also check that FT fixed joints are defined inside the force_torque_sensors SRDF group" << ENDLOG();
            success = false;
        }
        else{
            _tmp_vector6.setZero();
            _tmp_vector6<<_ft_feedback.at(it2->first)->feedback.forces.cast <double> (), _ft_feedback.at(it2->first)->feedback.torques.cast <double> ();
            getForceTorqueInternal().at(_ftptr->getSensorName())->setWrench(_tmp_vector6,getTime());
        }
    }

    map<IMUFrame, IMUFeedback::Ptr>::iterator it3;
    for(it3 = _imu_feedback.begin(); it3 != _imu_feedback.end(); it3++)
    {
        FlowStatus fs = _imu_feedback.at(it3->first)->read();

        auto it = getImuInternal().find(it3->first);

        if(it != getImuInternal().end() )
            _imuptr = it->second;
        if(!_imuptr){
            LOG(Error) << "WARNING in " << __func__ << ": no sensor corresponding to link " << it3->first << " is present in given URDF/SRDF! Check that IMU msg.header.frame_id is the parent link of the sensor name in URDF!" << ENDLOG();
            success = false;
        }
        else{
            _tmp_quaternion.setIdentity();
            _tmp_vector3d_1.setZero();
            _tmp_vector3d_2.setZero();
            _tmp_vector_4f.setZero();

            _tmp_vector3d_1 = _imu_feedback.at(it3->first)->feedback.linearAcceleration.cast <double> ();
            _tmp_vector3d_2 = _imu_feedback.at(it3->first)->feedback.angularVelocity.cast <double> ();
            _tmp_vector_4f = _imu_feedback.at(it3->first)->feedback.rotation;

            _tmp_quaternion.coeffs() = _tmp_vector_4f.cast <double> ();

            getImuInternal().at(_imuptr->getSensorName())->setImuData(_tmp_quaternion, _tmp_vector3d_1,_tmp_vector3d_2, getTime());
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
    return true;
}


