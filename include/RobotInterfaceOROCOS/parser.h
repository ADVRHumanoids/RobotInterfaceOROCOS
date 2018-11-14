#ifndef __PARSER_H__
#define __PARSER_H__

#include <RobotInterfaceOROCOS/RobotInterfaceOROCOS.h>
#include <tinyxml.h>
#include <srdfdom_advr/model.h>
#include <urdf/model.h>
#include <algorithm>


/**
 * This is a copy of the parser.h file in rtt-gazebo-robot-sim project without the parsing of the PID gains (only impedance)!
 */
namespace XBot {

struct parsed_words{
        static constexpr const char* robot_tag = "robot";
        static constexpr const char* rtt_gazebo_tag = "rtt-gazebo";
        static constexpr const char* controller_tag = "controller";
        static constexpr const char* reference_attribute = "reference";
        static constexpr const char* type_attribute = "type";
        static constexpr const char* gains_tag = "gains";
        static constexpr const char* stiffness_attribute = "stiffness";
        static constexpr const char* damping_attribute = "damping";

};

struct ImpedanceGain{
    std::string joint_name;
    double stiffness = 0.0;
    double damping = 0.0;
};

struct gains{
    typedef std::string kinematic_chain;
    typedef std::vector<ImpedanceGain> ImpedanceGains;

    std::map<kinematic_chain, std::vector<std::string>> map_controllers;
    std::map<kinematic_chain, ImpedanceGains> map_ImpedanceGains;

    gains()
    {

    }

    gains(const gains& _gains)
    {
        map_controllers = _gains.map_controllers;
        map_ImpedanceGains = _gains.map_ImpedanceGains;
    }

    bool getImpedance(const kinematic_chain& kc, const std::string& joint_name,
                             ImpedanceGain& impedance)
    {
        if(map_controllers.find(kc) == map_controllers.end())
            return false;

        std::vector<std::string> available_controllers = map_controllers.at(kc);
        if(std::find(available_controllers.begin(), available_controllers.end(),
                     std::string(ControlModes::JointImpedanceCtrl)) != available_controllers.end())
        {
            ImpedanceGains impedances= map_ImpedanceGains.at(kc);
            for(unsigned int i = 0; i < impedances.size(); ++i)
            {
                if(joint_name.compare(impedances[i].joint_name) == 0)
                {
                    impedance = impedances[i];
                    return true;
                }
            }
            return false;
        }
        else
            return false;
    }
};

class gain_parser{
public:
    gain_parser();

    bool initFile(const std::string& filename);
    void printGains();

    gains Gains;
private:
    boost::shared_ptr<TiXmlDocument> _doc;



};

}

#endif
