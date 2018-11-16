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

class orocos_test_homing: public RTT::TaskContext {
public:
    orocos_test_homing(std::string const & name):
        RTT::TaskContext(name)
    {
        //this->setActivity(new RTT::Activity(1, 0.01));

        this->addOperation("attachToRobot", &orocos_test_homing::attachToRobot,
                    this, RTT::ClientThread);
        
    }

    bool configureHook()
    {
        return false;
    }

    bool startHook()
    {
        _homing_time = 5;
        _time = 0;
        // first loop time
        _first_loop_time = _robot->getTime();
        
        _robot->sense();
        // retrieve the homing configuration
        _robot->getRobotState("home", _q_home);
        // get starting position
        _robot->getMotorPosition(_q0);
        
        RTT::log(RTT::Info)<<"Starting OROCOS simple homing!"<<RTT::endlog();
        
        return true;
    }

    void updateHook()
    {
        
        _time = _robot->getTime();
        if( (_time - _first_loop_time) <= _homing_time ) {
            _q = _q0 + 0.5*(1-std::cos(3.1415*(_time - _first_loop_time)/_homing_time))*(_q_home-_q0);
            _robot->setPositionReference(_q);
            _robot->move();
            
//             RTT::log(RTT::Info)<<_robot->getTime()<<RTT::endlog();
            return;
        }
        
        // stiffness test
//         Eigen::VectorXd k_ref;
//         k_ref.setZero(7);
//         
//         k_ref(5) = 50.0;
//         
//         _robot->chain("right_arm").setStiffness(k_ref);
//         _robot->move();
//         
           // torque test
//         Eigen::VectorXd tau_ref;
//         tau_ref.setZero(7);
//         
//         tau_ref(4) = 10.0;
//         tau_ref(5) = 10.0;
//         
//         _robot->chain("right_arm").setEffortReference(tau_ref);
//         _robot->move();
        
        
    
    
    }

    void stopHook()
    {

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

    


private:
    XBot::RobotInterface::Ptr _robot;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd tau;

    Eigen::VectorXd q_ref;

    Eigen::VectorXd _q0, _q_home, _q, _qref;
    double _time, _homing_time, _first_loop_time;
    
    double t;
    bool start_position_trj;
};

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(orocos_test_homing)
