#include "VFHServoing.hpp"
#include <vfh_star/VFHStar.h>
#include <base/angle.h>
#include <assert.h>
#include <boost/bind.hpp>


using namespace corridor_navigation;
using namespace vfh_star;
using namespace Eigen;

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

class corridor_navigation::VFHServoingDriveMode : public vfh_star::DriveMode
{
    const VFHServoing &servoing;
public:
    VFHServoingDriveMode(const std::string &identifier, const VFHServoing &servoing) : DriveMode(identifier),  servoing(servoing)
    {
        
    }

    virtual ~VFHServoingDriveMode()
    {
    }

    virtual double getCostForNode(const ProjectedPose& projection, const base::Angle& direction, const TreeNode& parentNode) const
    {
        //cost in time
        double cost = 0;
        double distance = servoing.search_conf.stepDistance;
        Vector3d intersectionPoint;
            
        const base::Pose &p(projection.pose);
        
        if(lineIntersection(parentNode.getPose().position, p.position, servoing.targetLinePoint, servoing.targetLinePoint + servoing.targetLine, intersectionPoint))
        {
            intersectionPoint.z() = parentNode.getPose().position.z();
            double distToGoal = (parentNode.getPose().position - intersectionPoint).norm();
    //      std::cout << "D " << direction << " P " << parentNode.getPose().position.transpose() << " PF " << p.position.transpose() << std::endl;
    //      std::cout << "TLP " << targetLinePoint.transpose() << " TL " << targetLine.transpose() << std::endl;
    //      std::cout << "IP " << intersectionPoint.transpose() << std::endl;
    //      std::cout << "dist to goal " << distToGoal << std::endl;
            if(distToGoal < distance)
            {
                distance = distToGoal;
            }
        }
        
        double current_speed = 0;
        double speedPenaltyForTerrain = 0;
        const double outer_radius = servoing.cost_conf.outerRadiusSize;
        
        envire::TraversabilityStatistic innerStats;
        envire::TraversabilityStatistic outerStats;
        base::Pose2D p2d(p);
        
        const VFHServoingConf &cost_conf(servoing.cost_conf);
        
        const double &virtualSizeX(servoing.virtualSizeX);
        const double &virtualSizeY(servoing.virtualSizeY);
        
        servoing.traversabilityGrid->computeStatistic(p2d, virtualSizeX, virtualSizeY, outer_radius, innerStats, outerStats);
            
        double worstInnerDrivability = 1.0;
        for(uint8_t i = 0; i <= innerStats.getHighestTraversabilityClass(); i++)
        {
            size_t count = innerStats.getClassCount(i);
            if(count)
            {
                const envire::TraversabilityClass &klass(servoing.traversabilityGrid->getTraversabilityClass(i));

                if(!klass.isTraversable())
                {
                    //TODO this is perhaps bad, as it makes the robot stop inside a obstacle
                    return std::numeric_limits<double>::infinity();
                }
                
                if(worstInnerDrivability > klass.getDrivability())
                    worstInnerDrivability = klass.getDrivability();                
            }
        }
        
        double maxOuterPenalty = 0.0;
        
        for(uint8_t i = 0; i <= outerStats.getHighestTraversabilityClass(); i++)
        {
            size_t count;
            double minDistToRobot;
            outerStats.getStatisticForClass(i, minDistToRobot, count);
            
            if(count)
            {
		if(minDistToRobot < 0)
		{
		    std::cout << "Warning, minDistToRobot < 0" << std::endl;
		    minDistToRobot = 0.0;
		}
		if(minDistToRobot > outer_radius)
		{
		    std::cout << "Warning, minDistToRobot > outer_radius" << std::endl;
		    minDistToRobot = outer_radius;
		}
                double impactFactor = (outer_radius - minDistToRobot) / outer_radius;
//                 std::cout << "Impact factor  " << impactFactor << " outer_radius " << outer_radius << " minDistToRobot " << minDistToRobot << std::endl;
                const envire::TraversabilityClass &klass(servoing.traversabilityGrid->getTraversabilityClass(i));
                double curPenalty = (1.0 - klass.getDrivability()) * impactFactor;
                maxOuterPenalty = std::max(maxOuterPenalty, curPenalty);
                assert(impactFactor < 1.001 && impactFactor >= 0);
            }
        }
    
    
        double innerSpeedPenalty;
        double outerSpeedPenalty;

        innerSpeedPenalty = cost_conf.maxInnerSpeedPenalty * (1 - worstInnerDrivability);
        outerSpeedPenalty = cost_conf.maxOuterSpeedPenalty * (maxOuterPenalty);
        
//         std::cout << "Outer Pen " << outerSpeedPenalty << " Inner " << innerSpeedPenalty << std::endl;
                                    
        speedPenaltyForTerrain = std::max(innerSpeedPenalty, outerSpeedPenalty);
    

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
            current_speed = cost_conf.baseSpeed - projection.angleTurned * cost_conf.speedReductionForTurning;
        }

        
        current_speed -= speedPenaltyForTerrain;
        
        if(current_speed < cost_conf.minimalSpeed) {
//             std::cout << "Error speed is negative " << current_speed << std::endl;
            current_speed = cost_conf.minimalSpeed;
        }
        
        //make direction changes expensive
        if(!parentNode.isRoot() && (parentNode.getDriveMode() != projection.driveMode))
        {
            cost += cost_conf.driveModeChangeCost;
        }

//         std::cout << "Cost is " << cost << " distance " << distance << " current_speed " << current_speed << " base speed " << cost_conf.baseSpeed << std::endl;
        

        return cost + distance / current_speed;
    }
};

class DriveModeForward : public VFHServoingDriveMode
{
public:
    DriveModeForward(const VFHServoing& servoing) : VFHServoingDriveMode("DriveModeForward", servoing)
    {
    }
    
    virtual bool projectPose(ProjectedPose &result, const TreeNode& curNode, const base::Angle& moveDirection, double distance) const
    {
        //super omnidirectional robot
        base::Vector3d p(distance, 0, 0);
//         base::Angle curHeading = curNode.getYaw();

        //forward case
        result.pose.orientation = curNode.getPose().orientation * Eigen::AngleAxisd(moveDirection.getRad(), base::Vector3d::UnitZ());
        result.pose.position = curNode.getPose().position + result.pose.orientation * p;
        result.angleTurned = fabs((moveDirection).getRad());
        result.nextPoseExists = true;
        
        return true;
    }
    virtual void setTrajectoryParameters(base::Trajectory& tr) const
    {
        tr.speed = 1.0;
    };
    
};

class DriveModeBackward : public VFHServoingDriveMode
{
public:
    DriveModeBackward(const VFHServoing& servoing) :VFHServoingDriveMode("DriveModeBackward", servoing)
    {
    }
    
    virtual bool projectPose(ProjectedPose &result, const TreeNode& curNode, const base::Angle& moveDirection, double distance) const
    {
        //super omnidirectional robot
        base::Vector3d p(distance, 0, 0);
//         base::Angle curHeading = curNode.getYaw();

        result.pose.orientation = curNode.getPose().orientation * Eigen::AngleAxisd(moveDirection.flipped().getRad(), base::Vector3d::UnitZ());
        result.pose.position = curNode.getPose().position - result.pose.orientation * p;
        result.angleTurned = fabs((moveDirection.flipped()).getRad());
        result.nextPoseExists = true;
        return true;;
    }
    virtual void setTrajectoryParameters(base::Trajectory& tr) const
    {
        tr.speed = -1.0;
    };
};

VFHServoing::VFHServoing(): allowBackwardsDriving(false), forward(new DriveModeForward(*this)), backward(new DriveModeBackward(*this)), env(NULL), traversabilityGrid(NULL)
{
    addDriveMode(*forward);
}

VFHServoing::~VFHServoing()
{
    delete forward;
    delete backward;
}

void VFHServoing::setAllowBackwardDriving(bool allowed)
{
    if(allowed == allowBackwardsDriving)
        return;

    clearDriveModes();
    
    addDriveMode(*forward);
    if(allowBackwardsDriving)
        addDriveMode(*backward);    
}

void VFHServoing::setCostConf(const corridor_navigation::VFHServoingConf& conf)
{
    vfh_star::VFHConf vfhConf;
    vfhConf = conf.vfhConfig;
    vfhConf.obstacleSafetyDistance = conf.obstacleSafetyRadius;
    vfhConf.robotWidth = std::min(conf.robotLength, conf.robotWidth);
    vfh.setConfig(vfhConf);
    
    //TODO add oversampling for front
    cost_conf = conf;
    
    virtualSizeX = cost_conf.robotLength + cost_conf.vfhConfig.obstacleSafetyDistance * 2.0;
    virtualSizeY = cost_conf.robotWidth + cost_conf.vfhConfig.obstacleSafetyDistance * 2.0;
//     search_conf.angularSamplingMin = asin((search_conf.stepDistance / 5.0) / search_conf.stepDistance);
//     std::cout << "setting min sampling to " << search_conf.angularSamplingMin / M_PI * 180 << std::endl;
}

void VFHServoing::setNewTraversabilityGrid(envire::TraversabilityGrid* trGrid)
{
    setTreeToWorld(trGrid->getFrameNode()->relativeTransform(trGrid->getEnvironment()->getRootNode()));
    originalTraversabilityGrid = trGrid;
    traversabilityGrid = trGrid;
}

const envire::TraversabilityGrid& VFHServoing::getInternalTraversabilityGrid()
{
    return *traversabilityGrid;
}

envire::Environment* VFHServoing::getInternalEnvironment()
{
    return env;
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

bool VFHServoing::isTerminalNode(const vfh_star::TreeNode& node) const
{
    double d = algebraicDistanceToGoalLine(node.getPose().position);
    
    return d<=0;
}

double VFHServoing::getHeuristic(const vfh_star::TreeNode& node) const
{   
    double d_to_goal = algebraicDistanceToGoalLine(node.getPose().position);
    if (d_to_goal < 0)
        return 0;

    return d_to_goal / cost_conf.baseSpeed;
}

void VFHServoing::setUnknownToObstacle(size_t x, size_t y)
{
    if(traversabilityGrid->getProbability(x, y) < 0.0001)
    {
        (*traversabilityData)[y][x] = obstacleClassNumber;
        //can't use helper function here, cause bug with copy constructor...
        (*probabilityData)[y][x] = 255;
    }
}

void VFHServoing::markUnkownTerrainOnStartAsObstacle(base::Pose start_world)
{
    //first copy the original Grid
    if(env)
    {
        delete env;
    }
    
    env = new envire::Environment();
    
    traversabilityGrid = new envire::TraversabilityGrid(*originalTraversabilityGrid);
    envire::FrameNode *fr = new envire::FrameNode(originalTraversabilityGrid->getFrameNode()->relativeTransform(originalTraversabilityGrid->getEnvironment()->getRootNode()));
    
    env->attachItem(fr);
    env->attachItem(traversabilityGrid);
    env->setFrameNode(traversabilityGrid, fr);
    env->getRootNode()->addChild(fr);

    if(!cost_conf.makeUnkownTerrainOnStartObstacles)
    {
        return;
    }
    
    bool found = false;
    uint8_t numClasses = traversabilityGrid->getTraversabilityClasses().size();
    //search for obstacle class
    for(uint8_t i = 0; i < numClasses; i++)
    {
        if(traversabilityGrid->getTraversabilityClass(i).getDrivability() < 0.001)
        {
            found = true;
            obstacleClassNumber = i;
            break;
        }
    }
    
    if(!found)
        throw std::runtime_error("VFHServoing::TraversabilityGrid does not contain an obstacle class");
    
    traversabilityData = &(traversabilityGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    probabilityData = &(traversabilityGrid->getGridData(envire::TraversabilityGrid::PROBABILITY));

    //compute position half robot length back
    base::Pose2D rectPos(getTreeToWorld().inverse() * start_world.toTransform());
    rectPos.position += Eigen::Rotation2D<double>(rectPos.orientation) * Eigen::Vector2d(-virtualSizeX/2.0,0); 

    //mark unknown in radius as obstacle
    if(!traversabilityGrid->forEachInRectangle(rectPos, virtualSizeX * 2.0, virtualSizeY * 2.0, boost::bind(&VFHServoing::setUnknownToObstacle, this, _1, _2)))
    {
        std::cout << "Each in rect failed : rectPos " << rectPos.position.transpose() << std::endl;
    };
    
}

VFHServoing::ServoingStatus VFHServoing::getTrajectories(std::vector< base::Trajectory >& result, const base::Pose& start, const base::Angle &mainHeading, 
                                                         double horizon, const Eigen::Affine3d& world2Trajectory, double minTrajectoryLenght)
{   
    size_t xi, yi;
    if(!traversabilityGrid->toGrid(start.position, xi, yi, traversabilityGrid->getEnvironment()->getRootNode()))
    {
        std::cout << "Warning, pose is not in grid" <<std::endl;
        result = std::vector<base::Trajectory>();
        return NO_SOLUTION;
    }
    
    markUnkownTerrainOnStartAsObstacle(start);
    
    vfh.setNewTraversabilityGrid(traversabilityGrid);
    
    TreeNode const* curNode = computePath(start, mainHeading, horizon, world2Trajectory);
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

    if(cost_conf.cutTrajectoryOnUnknownTerrain)
    {
        int minSteps = ceil(minTrajectoryLenght / search_conf.stepDistance);
        
        //look for anything that is not traversable on
        //the computed route, e.g unknow or shadow
        int i;
        for (i = 1; i < size; ++i)
        {
            //not this pose is in map coordinates, which is just right in this case
            base::Pose2D pose(nodes[i]->getPose());
            
            const envire::TraversabilityClass &klass = traversabilityGrid->getWorstTraversabilityClassInRectangle(pose, virtualSizeX, virtualSizeY);
            if(!klass.isTraversable())
            {
                std::cout << "Cutting trajectory at pos " << i << " from " << size << " reason: Obstacle " << std::endl;
                break;
            }
            
            if(traversabilityGrid->getWorstProbabilityInRectangle(pose, virtualSizeX, virtualSizeY) < 0.01)
            {
                std::cout << "Cutting trajectory at pos " << i << " from " << size << " reason: Unknown " << std::endl;
                break;
            }
        }
        
        //cut off the unknown part
        nodes.resize(i);
        
        if(i <= minSteps)
        {
            result = std::vector<base::Trajectory>();
            return TRAJECTORY_THROUGH_UNKNOWN;
        }
    }
     
    result = buildTrajectoriesTo(nodes, world2Trajectory);
    return TRAJECTORY_OK;
}

TreeSearch::AngleIntervals VFHServoing::getNextPossibleDirections(const TreeNode& curNode) const
{
    return vfh.getNextPossibleDirections(curNode.getPose());
}


