#ifndef CORRIDOR_NAVIGATION_VFHSERVOING_HPP
#define CORRIDOR_NAVIGATION_VFHSERVOING_HPP

#include <vfh_star/Types.h>
#include <vfh_star/VFHStar.h>
#include <vfh_star/TraversabilityGrid.h>
#include "VFHServoingConf.hpp"
#include "VFHStarDebugData.hpp"
#include <envire/maps/Grid.hpp>
#include "VFHServoingConf.hpp"

namespace corridor_navigation
{
    class VFHServoing: public vfh_star::VFHStar
    {
    public:
	
        VFHServoing();

        virtual ~VFHServoing() {};

        VFHStarDebugData getVFHStarDebugData(const std::vector< base::Waypoint >& trajectory);

	void clearDebugData();
	
	void setCostConf(const VFHServoingConf &conf);

	std::vector<base::Waypoint> getWaypoints(base::Pose const& start, double mainHeading, double horizon);
	
	void setNewTraversabilityGrid(const envire::Grid<vfh_star::Traversability> *tr);
	
	std::vector< base::Trajectory > getTrajectories(const base::Pose& start, double mainHeading, double horizon, const Eigen::Affine3d& body2Trajectory);
    private:
	VFHServoingConf cost_conf;
	
	virtual double getHeuristic(const vfh_star::TreeNode& node) const;
	
	virtual double getCostForNode(const base::Pose& p, double direction, const vfh_star::TreeNode& parentNode) const;
	
	virtual bool validateNode(const vfh_star::TreeNode& node) const;
	
	virtual bool isTerminalNode(const vfh_star::TreeNode& node) const; 
	
        vfh_star::VFH vfh;
        mutable VFHStarDebugData debugData;
        AngleIntervals getNextPossibleDirections(const vfh_star::TreeNode& curNode, double obstacleSafetyDist, double robotWidth) const;
        std::pair<base::Pose, bool> getProjectedPose(
                const vfh_star::TreeNode& curNode, double heading, double distance) const;
    };
}

#endif

