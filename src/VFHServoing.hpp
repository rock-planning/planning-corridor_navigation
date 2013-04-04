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
	
	enum ServoingStatus
	{
	    NO_SOLUTION,
	    TRAJECTORY_THROUGH_UNKNOWN,
	    TRAJECTORY_OK,
	};
	
        VFHServoing();

        virtual ~VFHServoing() {};

        VFHStarDebugData getVFHStarDebugData(const std::vector< base::Waypoint >& trajectory);

	void clearDebugData();
	
	void setCostConf(const VFHServoingConf &conf);

        /**
         * Computes a path from the start pose to a 
         * line that is 'horizon' meters away from
         * the start pose and ortogonal to 
         * mainHeading.
         * 
         * Returns a series of waypoints that lead to the horizon
         *      or an emptry vector is no path could be found.
         * */
	std::vector<base::Waypoint> getWaypoints(base::Pose const& start, double mainHeading, double horizon);
	
        /**
         * Sets a new traversability map.
         * The map is used while computing the optimal path
         * to the horizon.
         * */
	void setNewTraversabilityGrid(const envire::Grid<vfh_star::Traversability> *tr);
	
        /**
         * Computes a path from the start pose to a 
         * line that is 'horizon' meters away from
         * the start pose and ortogonal to 
         * mainHeading.
         * 
         * The trajectory is returned in the 'result' parameter.
         * The result is transformed into the 'trajectory' frame using 
         * the given parameter 'body2Trajectory'.
         * 
         * Returns wether the planning was successfull.
         * */
	ServoingStatus getTrajectories(std::vector< base::Trajectory > &result, const base::Pose& start, double mainHeading, double horizon, const Eigen::Affine3d& body2Trajectory);
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

