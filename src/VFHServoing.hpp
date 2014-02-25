#ifndef CORRIDOR_NAVIGATION_VFHSERVOING_HPP
#define CORRIDOR_NAVIGATION_VFHSERVOING_HPP

#include <vfh_star/Types.h>
#include <vfh_star/HorizonPlanner.hpp>
#include <vfh_star/VFH.h>
#include "VFHServoingConf.hpp"
#include <envire/maps/Grid.hpp>
#include "VFHServoingConf.hpp"

namespace corridor_navigation
{
    class VFHServoingDriveMode;
    class VFHServoing: public vfh_star::HorizonPlanner
    {
        friend class VFHServoingDriveMode;
    public:
	
	enum ServoingStatus
	{
	    NO_SOLUTION,
	    TRAJECTORY_THROUGH_UNKNOWN,
	    TRAJECTORY_OK,
	};
	
        VFHServoing();

        virtual ~VFHServoing();

	void setCostConf(const VFHServoingConf &conf);

        void setAllowBackwardDriving(bool allowed);
        
        /**
         * Computes a path from the start pose to a 
         * line that is 'horizon' meters away from
         * the start pose and ortogonal to 
         * mainHeading.
         * 
         * @param start_map position and orientation of the start in map frame
         * @param mainHeading_map target heading in map frame
         * 
         * The trajectory is returned in the 'result' parameter.
         * The result is transformed into the 'trajectory' frame using 
         * the given parameter 'world2Trajectory'.
         * 
         * Returns wether the planning was successfull.
         * */
	ServoingStatus getTrajectories(std::vector< base::Trajectory > &result, const base::Pose& start, const base::Angle &mainHeading, double horizon, const Eigen::Affine3d& world2Trajectory, double minTrajectoryLenght = 0);

        /**
         * Sets a new traversability map.
         * The map is used while computing the optimal path
         * to the horizon.
         * */
        void setNewTraversabilityGrid(const envire::TraversabilityGrid *trGrid);
        
    private:
	VFHServoingConf cost_conf;
        vfh_star::VFH vfh;
        bool allowBackwardsDriving;        
        VFHServoingDriveMode *forward;
        VFHServoingDriveMode *backward;
        
        ///Height plus obstacle safety radius
        double virtualHeight;
        ///widht plus obstacle safety radius
        double virtualWidth;

	virtual double getHeuristic(const vfh_star::TreeNode& node) const;
	
	virtual bool validateNode(const vfh_star::TreeNode& node) const;
	
	virtual bool isTerminalNode(const vfh_star::TreeNode& node) const; 
	
        virtual AngleIntervals getNextPossibleDirections(const vfh_star::TreeNode& curNode) const;
    };
}

#endif

