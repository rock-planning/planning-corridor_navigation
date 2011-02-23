#include <corridor_navigation/VFHServoing.hpp>
#include <vfh_star/VFHStar.h>

using namespace corridor_navigation;
using namespace vfh_star;

VFHStarDebugData VFHServoing::getVFHStarDebugData(const std::vector< base::Waypoint >& trajectory)
{
    VFHStarDebugData dd_out;
    dd_out.horizonOrigin = getHorizonOrigin();
    dd_out.horizonVector = getHorizonVector();
    for(std::vector<base::Waypoint>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++)
    {
	bool found = false;
	for(std::vector<vfh_star::VFHDebugData>::const_iterator it2 = debugData.steps.begin(); it2 != debugData.steps.end(); it2++) 
	{
	    if(it->position == base::Vector3d(it2->pose.position))
	    {
		dd_out.steps.push_back(*it2);
		found = true;
		break;
	    }
	    
	}
	if(!found && (it + 1) != trajectory.end() )
	{
	    std::cerr << "BAD debug data is fishy" << std::endl;
	    throw std::runtime_error("Could not build VFHStarDebugData");
	}
    }
    return dd_out;
}


std::vector< std::pair< double, double > > VFHServoing::getNextPossibleDirections(const vfh_star::TreeNode& curNode, double obstacleSafetyDist, double robotWidth) const
{
    VFHDebugData dd;
    std::vector< std::pair< double, double > > ret;
    ret = vfh.getNextPossibleDirections(curNode.getPose(), obstacleSafetyDist, robotWidth, &dd);
    debugData.steps.push_back(dd);
    return ret;
}

std::pair<base::Pose, bool> VFHServoing::getProjectedPose(const vfh_star::TreeNode& curNode, double heading, double distance) const
{
    //super omnidirectional robot
    base::Vector3d p(0, distance, 0);
    
    base::Pose ret;
    ret.orientation = Eigen::AngleAxisd(heading, base::Vector3d::UnitZ());
    ret.position = curNode.getPose().position + ret.orientation * p;
    
    return std::make_pair(ret, true);
}

