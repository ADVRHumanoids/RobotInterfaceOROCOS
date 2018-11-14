#include <RobotInterfaceOROCOS/parser.h>

using namespace XBot;

gain_parser::gain_parser()
{

}

bool gain_parser::initFile(const std::string &filename)
{
    _doc.reset(new TiXmlDocument(filename));

    if(!_doc->LoadFile())
        return false;

    TiXmlHandle hDoc(_doc.get());
    TiXmlElement *pRoot, *pParm;
    TiXmlElement *ppParam, *pppParam;
    pRoot = _doc->FirstChildElement(XBot::parsed_words::robot_tag);

    if(!pRoot)
        return false;

    pParm = pRoot->FirstChildElement(XBot::parsed_words::rtt_gazebo_tag);
    while(pParm)
    {
         std::string kin_chain_name = pParm->Attribute(XBot::parsed_words::reference_attribute);

         std::vector<std::string> controllers;
         XBot::gains::ImpedanceGains impedances;

         ppParam = pParm->FirstChildElement(XBot::parsed_words::controller_tag);
         while(ppParam)
         {
            std::string controller_type = ppParam->Attribute(XBot::parsed_words::type_attribute);
            controllers.push_back(controller_type);

            pppParam = ppParam->FirstChildElement(XBot::parsed_words::gains_tag);
            while(pppParam)
            {
                std::string joint_name = pppParam->Attribute(XBot::parsed_words::reference_attribute);

                if(controller_type.compare(ControlModes::JointImpedanceCtrl) == 0)
                {
                    std::string stiffness = pppParam->Attribute(XBot::parsed_words::stiffness_attribute);
                    std::string damping = pppParam->Attribute(XBot::parsed_words::damping_attribute);

                    XBot::ImpedanceGain impedance;
                    impedance.joint_name = joint_name;
                    impedance.stiffness = std::atof(stiffness.c_str());
                    impedance.damping = std::atof(damping.c_str());

                    impedances.push_back(impedance);
                }


                pppParam = pppParam->NextSiblingElement(XBot::parsed_words::gains_tag);
            }


            if(controller_type.compare(ControlModes::JointImpedanceCtrl) == 0)
                Gains.map_ImpedanceGains[kin_chain_name] = impedances;


            ppParam = ppParam->NextSiblingElement(XBot::parsed_words::controller_tag);
         }

         if(controllers.size() > 0)
            Gains.map_controllers[kin_chain_name] = controllers;






         pParm = pParm->NextSiblingElement(XBot::parsed_words::rtt_gazebo_tag);
    }

    return true;
}

void gain_parser::printGains()
{
    std::map<XBot::gains::kinematic_chain, std::vector<std::string>>::iterator map_controllers_it;
    std::map<XBot::gains::kinematic_chain, XBot::gains::ImpedanceGains>::iterator map_ImpedanceGains_it;

    for(map_controllers_it = Gains.map_controllers.begin();
        map_controllers_it != Gains.map_controllers.end(); map_controllers_it++)
    {
        std::string kin_chain_name = map_controllers_it->first;
        std::vector<std::string> controllers = map_controllers_it->second;

        std::cout<<kin_chain_name<<": [  ";
        for(unsigned int i = 0; i < controllers.size(); ++i)
            std::cout<<controllers[i]<<"  ";
        std::cout<<"]"<<std::endl;
    }

    for(map_ImpedanceGains_it = Gains.map_ImpedanceGains.begin();
        map_ImpedanceGains_it != Gains.map_ImpedanceGains.end(); map_ImpedanceGains_it++)
    {
        std::string kin_chain_name = map_ImpedanceGains_it->first;
        XBot::gains::ImpedanceGains impedances= map_ImpedanceGains_it->second;

        std::cout<<kin_chain_name<<"  JointImpedanceCtrl:"<<std::endl;
        for(unsigned int i = 0; i < impedances.size(); ++i)
            std::cout<<"    "<<impedances[i].joint_name<<"    stiffness: "<<impedances[i].stiffness
                    <<" damping: "<<impedances[i].damping<<std::endl;
    }
}

