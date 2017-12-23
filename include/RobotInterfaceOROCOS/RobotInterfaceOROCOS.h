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
#include <rst-rt/dynamics/Wrench.hpp>
#include <std_msgs/Float64.h>

#include <XBotInterface/SoLib.h>

#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace std;
using namespace RTT;
using namespace Eigen;
using namespace rstrt;

namespace XBot
{

class RobotInterfaceOROCOS : public RobotInterface, os::TimeService
{

    friend RobotInterface;

public:
    typedef string KinematicChainName;
    typedef string JointName;
    typedef vector<string> JointNames;

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

    map<KinematicChainName, boost::shared_ptr<InputPort<robot::JointState> > > _kinematic_chains_feedback_ports;
    map<KinematicChainName, boost::shared_ptr<OutputPort<kinematics::JointAngles> > > _kinematic_chains_output_ports;
    map<string, boost::shared_ptr<InputPort<dynamics::Wrench> > > _frames_ports_map;

    map<KinematicChainName, robot::JointState> _kinematic_chains_joint_state_map;
    map<KinematicChainName, kinematics::JointAngles> _kinematic_chains_desired_joint_state_map;
    map<string, dynamics::Wrench> _frames_wrenches_map;


    //For now these variable are motor side AND link side
    VectorXd _q;
    VectorXd _qdot;
    VectorXd _tau;

    VectorXd _q_ref;


    ForceTorqueSensor::ConstPtr _ftptr;
    Vector6d _tmp_vector6;
};

}
#endif
