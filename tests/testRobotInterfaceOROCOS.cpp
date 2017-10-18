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

    bool attachToRobot(const std::string &robot_name, const std::string &config_path)
    {
        XBot::RobotInterfaceOROCOS::attachToRobot(robot_name, config_path,
            _robot, std::shared_ptr<RTT::TaskContext>(this));

    }


private:
    XBot::RobotInterface::Ptr _robot;
};

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(robot_interface_orocos_test)
