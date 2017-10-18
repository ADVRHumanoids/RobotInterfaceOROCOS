/*
 * Copyright (C) 2017 Cogimon
 * Author: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <XBotInterface/RobotInterface.h>
#include <rtt/Component.hpp>
#include <boost/shared_ptr.hpp>
#include <RobotInterfaceOROCOS/RobotInterfaceOROCOS.h>

class robot_interface_orocos_test: public RTT::TaskContext {
public:
    robot_interface_orocos_test(std::string const & name):
        RTT::TaskContext(name)
    {
        this->setActivity(new RTT::Activity(1, 0.01));

        this->addOperation("attachToRobot", &robot_interface_orocos_test::attachToRobot,
                    this, RTT::ClientThread);
        this->addOperation("sense", &robot_interface_orocos_test::sense,
                    this, RTT::ClientThread);
    }

    bool configureHook()
    {
        return false;
    }

    bool startHook()
    {
        return false;
    }

    void updateHook()
    {

    }

    void stopHook()
    {

    }

    bool sense()
    {
        _robot->sense();
        _robot->getJointPosition(q);
        _robot->getJointVelocity(qdot);
        _robot->getJointEffort(tau);

        std::cout<<"q: "<<q<<std::endl;
        std::cout<<"qdot: "<<qdot<<std::endl;
        std::cout<<"tau: "<<tau<<std::endl;

        return true;
    }

    bool attachToRobot(const std::string &robot_name, const std::string &config_path)
    {
        bool a =  XBot::RobotInterfaceOROCOS::attachToRobot(robot_name, config_path,
            _robot, std::shared_ptr<RTT::TaskContext>(this));

        if(!a)
            RTT::log(RTT::Error)<<"ERROR!!! attachToRobot returned false"<<RTT::endlog();
        if(!_robot)
            RTT::log(RTT::Error)<<"ERROR!!! _robot is invalid pointer"<<RTT::endlog();


        std::cout<<"robot_name has "<<_robot->getJointNum()<<" dofs"<<std::endl;

        q.setZero(_robot->getJointNum());
        qdot.setZero(_robot->getJointNum());
        tau.setZero(_robot->getJointNum());

        return a;
    }


private:
    XBot::RobotInterface::Ptr _robot;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd tau;
};

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(robot_interface_orocos_test)
