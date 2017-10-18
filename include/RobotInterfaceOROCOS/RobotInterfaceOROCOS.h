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

#include <SharedLibraryClass.h>

#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

namespace XBot
{

class RobotInterfaceOROCOS : public RobotInterface, RTT::os::TimeService
{

    friend RobotInterface;

public:

    RobotInterfaceOROCOS() {}

    virtual bool set_control_mode_internal ( int joint_id, const ControlMode& control_mode );

    virtual double getTime() const;

    virtual bool isRunning() const;

    static bool attachToRobot(const std::string &robot_name, const std::string &config_path,
                              XBot::RobotInterface::Ptr& _robot, std::shared_ptr<RTT::TaskContext> task);

protected:

    virtual bool init_robot(const std::string &path_to_cfg, AnyMapConstPtr any_map);
    virtual bool move_internal();
    virtual bool sense_internal();
    virtual bool read_sensors();
    virtual bool move_hands();
    virtual bool sense_hands();

private:
    std::shared_ptr<RTT::TaskContext> _task_ptr;
    std::shared_ptr<RTT::TaskContext> _task_peer_ptr;

    std::map<std::string, std::vector<std::string> > _map_kin_chains_joints;

    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> > > _kinematic_chains_feedback_ports;
    std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> > > _kinematic_chains_output_ports;
    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::dynamics::Wrench> > > _frames_ports_map;

    std::map<std::string, rstrt::robot::JointState> _kinematic_chains_joint_state_map;
    std::map<std::string, rstrt::kinematics::JointAngles> _kinematic_chains_desired_joint_state_map;
    std::map<std::string, rstrt::dynamics::Wrench> _frames_wrenches_map;


    //For now these variable are motor side AND link side
    Eigen::VectorXd _q;
    Eigen::VectorXd _qdot;
    Eigen::VectorXd _tau;

    ForceTorqueSensor::ConstPtr _ftptr;
    Eigen::Vector6d _tmp_vector6;
};

}
#endif
