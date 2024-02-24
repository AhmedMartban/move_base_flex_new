// inter_util.cpp
#include "inter_util.h"
#include <vector>
#include <cmath>
#include <unordered_map>
#include <string> 

namespace inter_util
{
    std::string InterUtil::getLocalPlanner(const std::string &keyword)
    {
        // Create a map to store keyword to local planner name mappings
        std::unordered_map<std::string, std::string> plannerMap = {
            {"teb", "TebLocalPlannerROS"},
            {"mpc", "MpcLocalPlannerROS"},
            {"dwa", "DwaLocalPlannerROS"},
            {"cohan", "HATebLocalPlannerROS"},
            {"rosnav", "TebLocalPlannerROS"},
            {"dragon", "DwaLocalPlannerROS"},
            {"applr", "TrajectoryPlannerROS"},
            {"lflh", "TrajectoryPlannerROS"},
            {"trail", "TrajectoryPlannerROS"}
        };

        // Use the map to find the local planner name
        auto it = plannerMap.find(keyword);
        return (it != plannerMap.end()) ? it->second : "";
    }

    double InterUtil::getDangerLevel(const std::vector<double>& terms) {
        double denominator = 0.0;

        for (double term : terms) {
            denominator += 1/pow((term / 3.0), 2);
        }

        double exponent = -1.0 / denominator;

        return exp(exponent);
        }   
    
    void InterUtil::publishSignal(ros::Publisher& publisher)
    {
        std_msgs::String msg;
        msg.data = "SIGNAL";
        publisher.publish(msg);
    }

    void InterUtil::checkDanger(ros::Publisher& publisher, const std::vector<double>& terms, double threshold)
    {
        double dangerLevel = getDangerLevel(terms);

        if (dangerLevel > threshold) {
            publishSignal(publisher);
        }
    }



    void InterUtil::select_heuristic(const std::vector<inter_util::SimAgentInfo> &simAgentInfos)
    {
        for (const auto &agent : simAgentInfos)
        {
            std::string ped_type = agent.type; // Type of ped 
            geometry_msgs::Point32 pedestrian_point = agent.point; // Position of ped 
            std::string id = agent.id; // ID of ped 
            std::string  social_state = agent.social_state; // Social state of ped 

            ROS_INFO("Pedestrian Type: %s, Pedestrian Point: (%f, %f, %f), ID: %u, Social State: %u",
                    ped_type.c_str(), pedestrian_point.x, pedestrian_point.y, pedestrian_point.z, id, social_state);

            //Switch to Polite:
            
            



            //Switch to Sideways:


        }
    }


   
}