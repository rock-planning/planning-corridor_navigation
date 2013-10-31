#include "VFHServoing.hpp"
#include <vfh_star/VFHStar.h>
#include <base/angle.h>
#include <assert.h>


using namespace corridor_navigation;
using namespace vfh_star;
using namespace Eigen;

enum VFSDriveMode
{
    FORWARD = 0,
    BACKWARD = 1,
};

VFHServoing::VFHServoing(): vfh(), allowBackwardsDriving(false)
{
}

void VFHServoing::setAllowBackwardDriving(bool allowed)
{
    if(allowed == allowBackwardsDriving)
        return;

    allowBackwardsDriving = allowed;
    
    if(allowBackwardsDriving)
    {
        setMaxDriveModes(2);
    }
    else
    {
        setMaxDriveModes(1);
    }
}

void VFHServoing::setNewTraversabilityGrid(const envire::Grid< Traversability >* tr)
{
    vfh.setNewTraversabilityGrid(tr);
}


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

void VFHServoing::clearDebugData()
{
    debugData.generatedTrajectory.clear();
    debugData.steps.clear();
}

void VFHServoing::setCostConf(const corridor_navigation::VFHServoingConf& conf)
{
    cost_conf = conf;
    search_conf.angularSamplingMin = asin((search_conf.stepDistance / 5.0) / search_conf.stepDistance);
    vfh.setSenseRadius(conf.obstacleSenseRadius);
    std::cout << "setting min sampling to " << search_conf.angularSamplingMin / M_PI * 180 << std::endl;
}

bool VFHServoing::validateNode(const vfh_star::TreeNode& node) const
{
    if(node.getCost() == std::numeric_limits<double>::infinity())
	return false;
    
    if(!vfh.validPosition(node.getPose())) {
	return false;
    }
    return true;
}

std::vector< std::pair< double, double > > VFHServoing::getNextPossibleDirections(const vfh_star::TreeNode& curNode, double obstacleSafetyDist, double robotWidth) const
{
    std::vector< std::pair< double, double > > ret;
    ret = vfh.getNextPossibleDirections(curNode.getPose(), obstacleSafetyDist, robotWidth);
    std::vector< std::pair< double, double > > frontIntervals;

    if(curNode.isRoot() && ret.empty())
    {
	ret.push_back(std::make_pair(0, 2*M_PI));
    }
    
    //oversampling interval
    double intervalHalf = cost_conf.oversamplingWidth / 2.0;
    
    double curDir = curNode.getDirection();
    if(curDir < 0)
	curDir += 2*M_PI;
    
    if(curDir > 2*M_PI)
	curDir -= 2*M_PI;
    
    double start = curDir - intervalHalf;
    if(start < 0)
	start += 2 * M_PI;
    
    double end = curDir + intervalHalf;
    if(end > 2 * M_PI)
	end -= 2 * M_PI;
    
    for(std::vector< std::pair< double, double > >::iterator it = ret.begin(); it != ret.end(); it++)
    {
	bool startInInterval = false;
	bool endInInterval = false;
	
	//check for wrapping case
	if(it->first > it->second) 
	{
// 	    std::cout << "Wrapping case " << it->first << " " << it->second << " dir " << curNode.getDirection() << " start " << start << " end " << end << std::endl;
	    
	    if((it->first < start && start < 2 * M_PI) || (0 < start && start < it->second))
	    {
		startInInterval = true;
	    }

	    if((it->first < end && end < 2 * M_PI) || (0 < end && end < it->second))
	    {
		endInInterval = true;
	    }    
	} else {
	    //test if start is in interval
	    if(it->first < start && start < it->second)
	    {
		startInInterval = true;
	    }
	    
	    //test if end is in interval
	    if(it->first < end && end < it->second)
	    {
		endInInterval = true;
	    }
	}
	
	if(startInInterval && endInInterval)
	{
	    ret.push_back(std::make_pair<double, double>(start, end));
	    return ret;
	}
	
	//start but no end
	if(startInInterval)
	{
	    frontIntervals.push_back(std::make_pair<double, double>(start, it->second));
	}
	
	//only end in interval
	if(endInInterval)
	{
	    frontIntervals.push_back(std::make_pair<double, double>(it->first, end));
	}
    }
    
    ret.insert(ret.end(), frontIntervals.begin(), frontIntervals.end());
    
    return ret;
}

bool VFHServoing::isTerminalNode(const vfh_star::TreeNode& node) const
{
    //test if unknown terrain is near
//     double securityRadius = search_conf.robotWidth + search_conf.obstacleSafetyDistance; 
    
    double d = algebraicDistanceToGoalLine(node.getPose().position);
    
    return d<=0;
}

std::vector< ProjectedPose > VFHServoing::getProjectedPoses(const vfh_star::TreeNode& curNode, double moveDirection, double distance) const
{
    std::vector< ProjectedPose > ret;
    
    //super omnidirectional robot
    base::Vector3d p(0, distance, 0);
    
    base::Pose pose;

    double curHeading = curNode.getYaw();

    // Compute rate of turn for front to wanted heading
    double angleDiffForward = angleDiff(moveDirection , curHeading);
    // Compute rate of turn for back to wanted heading
    double angleDiffBackward = angleDiff(moveDirection - M_PI, curHeading);
    

    //forward case
    ProjectedPose fwProj;
    fwProj.driveMode = FORWARD;
    fwProj.pose.orientation = Eigen::AngleAxisd(moveDirection, base::Vector3d::UnitZ());
    fwProj.pose.position = curNode.getPose().position + fwProj.pose.orientation * p;
    fwProj.angleTurned = angleDiffForward;
    fwProj.nextPoseExists = true;
    ret.push_back(fwProj);
    
    if(allowBackwardsDriving)
    {
        ProjectedPose bwProj;
        bwProj.driveMode = BACKWARD;
        bwProj.pose.orientation = Eigen::AngleAxisd(moveDirection - M_PI, base::Vector3d::UnitZ());
        bwProj.pose.position = curNode.getPose().position - bwProj.pose.orientation * p;
        bwProj.angleTurned = angleDiffBackward;
        bwProj.nextPoseExists = true;
        ret.push_back(bwProj);
    }
    
    return ret;
}

double VFHServoing::getHeuristic(const vfh_star::TreeNode& node) const
{   
    double d_to_goal = algebraicDistanceToGoalLine(node.getPose().position);
    if (d_to_goal < 0)
        return 0;

    return d_to_goal / cost_conf.speedProfile[0];
}

bool lineIntersection(const Vector3d &p1, const Vector3d &p2, const Vector3d &p3, const Vector3d &p4, Vector3d &intersectionPoint) {
    double a1 = p2.y()-p1.y();
    double b1 = p1.x()-p2.x();
    double c1 = p2.x()*p1.y() - p1.x() * p2.y();// x2*y1 - x1*y2;  { a1*x + b1*y + c1 = 0 is line 1 }

    double a2 = p4.y() - p3.y(); //y4-y3;
    double b2 = p3.x() - p4.x(); //x3-x4;
    double c2 = p4.x()*p3.y() - p3.x()*p4.y(); //x4*y3 - x3*y4;  { a2*x + b2*y + c2 = 0 is line 2 }

    double denom = a1*b2 - a2*b1;
    if( denom == 0 ) {
	return false;
    }

    intersectionPoint.x() = (b1*c2 - b2*c1)/denom;
    intersectionPoint.y() = (a2*c1 - a1*c2)/denom;
    intersectionPoint.z() = 0;
    
    return true;
}

std::vector< base::Waypoint > VFHServoing::getWaypoints(const base::Pose& start, double mainHeading, double horizon)
{
    std::vector< base::Waypoint > wholeTrajectory = VFHStar::getWaypoints(start, mainHeading, horizon);
    std::vector< base::Waypoint > ret;
    
    for(std::vector< base::Waypoint >::const_iterator it = wholeTrajectory.begin(); it != wholeTrajectory.end(); it++)
    {
	base::Pose pos;
	pos.position = it->position;
	pos.orientation = AngleAxisd(it->heading, Vector3d::UnitZ());
	if(vfh.getWorstTerrainInRadius(pos, search_conf.robotWidth / 2.0 + search_conf.obstacleSafetyDistance) == TRAVERSABLE)
	    ret.push_back(*it);
	else
	    break;
    }
    
    return ret;
}

VFHServoing::ServoingStatus VFHServoing::getTrajectories(std::vector< base::Trajectory >& result, const base::Pose& start, double mainHeading, double horizon, const Eigen::Affine3d& body2Trajectory)
{    
    TreeNode const* curNode = computePath(start, mainHeading, horizon, body2Trajectory);
    if (!curNode)
    {
	result = std::vector<base::Trajectory>();
        return NO_SOLUTION;
    }
    
    std::vector<const vfh_star::TreeNode *> nodes;
    const vfh_star::TreeNode* nodeTmp = curNode;
    int size = curNode->getDepth() + 1;
    for (int i = 0; i < size; ++i)
    {
	nodes.insert(nodes.begin(), nodeTmp);
	if (nodeTmp->isRoot() && i != size - 1)
            throw std::runtime_error("internal error in buildTrajectoryTo: found a root node even though the trajectory is not finished");
	nodeTmp = nodeTmp->getParent();
    }    

    //look for anything that is not traversable on
    //the computed route, e.g unknow or shadow
    int i;
    for (i = 0; i < size; ++i)
    {
	if(vfh.getWorstTerrainInRadius(nodes[i]->getPose(), search_conf.robotWidth / 2.0 + search_conf.obstacleSafetyDistance) != TRAVERSABLE)
	{
	    std::cout << "Cutting trajectory at pos " << i << " from " << size << std::endl;
	    break;
	}
    }
    
    //cut off the unknown part
    nodes.resize(i);
    
    if(i == 0)
    {
	result = std::vector<base::Trajectory>();
	return TRAJECTORY_THROUGH_UNKNOWN;
    }
     
    result = tree.buildTrajectoriesTo(nodes, body2Trajectory);
    return TRAJECTORY_OK;
}

double VFHServoing::getCostForNode(const vfh_star::ProjectedPose& projection, double direction, const vfh_star::TreeNode& parentNode) const
{
    //cost in time
    double cost = 0;
    double distance = search_conf.stepDistance;
    Vector3d intersectionPoint;
	
    const base::Pose &p(projection.pose);
    
    if(lineIntersection(parentNode.getPose().position, p.position, targetLinePoint, targetLinePoint + targetLine, intersectionPoint))
    {
	intersectionPoint.z() = parentNode.getPose().position.z();
	double distToGoal = (parentNode.getPose().position - intersectionPoint).norm();
// 	std::cout << "D " << direction << " P " << parentNode.getPose().position.transpose() << " PF " << p.position.transpose() << std::endl;
// 	std::cout << "TLP " << targetLinePoint.transpose() << " TL " << targetLine.transpose() << std::endl;
// 	std::cout << "IP " << intersectionPoint.transpose() << std::endl;
// 	std::cout << "dist to goal " << distToGoal << std::endl;
	if(distToGoal < distance)
	{
	    distance = distToGoal;
	}
    }
    
    double current_speed = 0;
    double speedPenaltyForTerrain = 0;
    const double outer_radius = 0.3;
    const double inner_radius = search_conf.robotWidth / 2.0 + search_conf.obstacleSafetyDistance;
    std::pair<TerrainStatistic, TerrainStatistic> stats = vfh.getTerrainStatisticsForRadius(p, inner_radius, outer_radius);
    
    const TerrainStatistic &innerStats(stats.first);
    const TerrainStatistic &outerStats(stats.second);
    
    if(innerStats.getObstacleCount() > 0)
	//TODO this is perhaps bad, as it makes the robot stop inside a obstacle
	return std::numeric_limits<double>::infinity();

/*    const double innerCnt = innerStats.getTerrainCount();
    double innerSpeedPenalty = 	innerStats.getUnknownCount() / innerCnt * cost_conf.unknownSpeedPenalty + 
      innerStats.getUnknownObstacleCount() / innerCnt * cost_conf.shadowSpeedPenalty;*/

/*    const double outerCnt = outerStats.getTerrainCount();
    double outerSpeedPenalty = 	outerStats.getObstacleCount() / outerCnt * cost_conf.shadowSpeedPenalty + 
				outerStats.getUnknownCount() / outerCnt * cost_conf.unknownSpeedPenalty +
				outerStats.getUnknownObstacleCount() / outerCnt * cost_conf.shadowSpeedPenalty;*/
    double innerSpeedPenalty = 0;
    if(innerStats.getUnknownObstacleCount())
	innerSpeedPenalty += cost_conf.shadowSpeedPenalty;

    double outerSpeedPenalty = 0;
    if(outerStats.getObstacleCount())
	outerSpeedPenalty += cost_conf.shadowSpeedPenalty * (outer_radius - (outerStats.getMinDistanceToTerrain(OBSTACLE) - inner_radius)) / outer_radius;
    if(outerStats.getUnknownObstacleCount())
	outerSpeedPenalty += cost_conf.shadowSpeedPenalty * (outer_radius - (outerStats.getMinDistanceToTerrain(UNKNOWN_OBSTACLE) - inner_radius)) / outer_radius;
    
//     std::cout << "Outer Pen " << outerSpeedPenalty << " cnt " << outerStats.getTerrainCount() << " Inner " << innerSpeedPenalty << " cnt " << innerStats.getTerrainCount() << " safe dist " << search_conf.obstacleSafetyDistance << std::endl;
				
    speedPenaltyForTerrain = innerSpeedPenalty + outerSpeedPenalty;
   
        
    // Check if we must point turn
    if (cost_conf.pointTurnThreshold > 0 && projection.angleTurned > cost_conf.pointTurnThreshold)
    {
        current_speed = cost_conf.speedAfterPointTurn;
        //add time needed to turn
        cost += projection.angleTurned / cost_conf.pointTurnSpeed;
    }
    else
    {
        //calculate speed reducttion for driving curves
        current_speed = cost_conf.speedProfile[0] - projection.angleTurned * cost_conf.speedProfile[1];
    }

    current_speed -=speedPenaltyForTerrain;
    
    if(current_speed < cost_conf.minimalSpeed) {
	std::cout << "Error speed is negative " << current_speed << std::endl;
	current_speed = cost_conf.minimalSpeed;
    }
    
    //make direction changes expensive
    if(!parentNode.isRoot() && (parentNode.getDriveMode() != projection.driveMode))
    {
        cost += 0.05;
    }


    return cost + distance / current_speed;
} 

