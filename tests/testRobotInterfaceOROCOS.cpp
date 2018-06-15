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

        this->addOperation("startPosTrj", &robot_interface_orocos_test::startPosTrj,
                    this, RTT::ClientThread);
        start_position_trj = false;

        this->addOperation("stopTrj", &robot_interface_orocos_test::stopTrj,
                    this, RTT::ClientThread);
    }

    bool configureHook()
    {
        return false;
    }

    bool startHook()
    {
        return true;
    }

    void updateHook()
    {
        if(start_position_trj)
        {
            double amplitude = 10.*M_PI/180.;
            double period = 0.1;

            sin_traj(q, amplitude, t, period, q_ref);

            _robot->setPositionReference(q_ref);
            _robot->move();

            t += this->getPeriod();
        }
    }

    void stopHook()
    {

    }

    bool stopTrj()
    {
        t = 0.0;
        start_position_trj = false;

        return true;
    }

    bool startPosTrj()
    {
        sense();

        t = 0.0;
        start_position_trj = true;

        return true;
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

        std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > ftmap = _robot->getForceTorque();
        std::map< std::string, XBot::ForceTorqueSensor::ConstPtr >::iterator it;
        for(it = ftmap.begin(); it != ftmap.end(); it++){
            Eigen::Vector6d w;
            it->second->getWrench(w);
            std::cout<<it->first<<": "<<w<<std::endl;
        }

        std::map< std::string,XBot::ImuSensor::ConstPtr > imumap = _robot->getImu();
        std::map< std::string,XBot::ImuSensor::ConstPtr >::iterator it2;
        for(it2 = imumap.begin(); it2 != imumap.end(); it2++){
            Eigen::Quaterniond quat;
            it2->second->getOrientation(quat);
            std::cout<<it2->first<<" quaternion: "<<quat.coeffs()<<std::endl;
            Eigen::Vector3d tmp;
            it2->second->getLinearAcceleration(tmp);
            std::cout<<it2->first<<" linear acc: "<<tmp<<std::endl;
            it2->second->getAngularVelocity(tmp);
            std::cout<<it2->first<<" angular vel: "<<tmp<<std::endl;
        }

        return true;
    }

    bool attachToRobot(const std::string &robot_name, const std::string &urdf_path, const std::string &srdf_path)
    {
        bool a =  XBot::RobotInterfaceOROCOS::attachToRobot(robot_name, urdf_path, srdf_path, true, "RBDL","",
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

    double sin_traj(double q0, double amplitude, double t, double period)
    {
        return q0 + amplitude*std::sin(t/(period*M_PI));
    }

    void sin_traj(const Eigen::VectorXd& q0, double amplitude, double t, double period,
                  Eigen::VectorXd& qref)
    {
        qref.setZero(q0.size());
        for(unsigned int i = 0; i < q0.size(); ++i)
            qref[i] = sin_traj(q0[i], amplitude, t, period);
    }


private:
    XBot::RobotInterface::Ptr _robot;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd tau;

    Eigen::VectorXd q_ref;

    double t;
    bool start_position_trj;
};

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(robot_interface_orocos_test)
