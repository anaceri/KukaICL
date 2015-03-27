#ifndef PARAMETERMANAGER_H
#define PARAMETERMANAGER_H

#include "CtrlParam.h"
#include "task.h"
#include <map>
//load the parameter which are stored in xml file
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"
#include "boost/foreach.hpp"
using boost::property_tree::ptree;

class ParameterManager
{
public:
    ParameterManager();
    ParameterManager(const std::string);
    std::map<TACTaskNameT, taskctrlpara> tac_task_ctrl_param;
    std::map<VISTaskNameT, taskctrlpara> vis_task_ctrl_param;
    std::map<PROTaskNameT, taskctrlpara> pro_task_ctrl_param;
    std::map<FORCETaskNameT, taskctrlpara> force_task_ctrl_param;
    //    static ctrlpara o_ctrlpara;
    stiffpara stiff_ctrlpara;
private:
    void loadCtrlParam(std::string);
    void load(TACTaskNameT,ptree);
    void load(PROTaskNameT,ptree);
    std::map<TACTaskNameT, std::string> tac_map_task_name;
    std::map<PROTaskNameT, std::string> pro_map_task_name;
    std::map<int,std::string> map_name_dim;
    std::map<int,std::string> map_name_dim2;
    std::map<int,std::string> map_name_dim3;
};

#endif // PARAMETERMANAGER_H
