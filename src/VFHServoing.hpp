#ifndef CORRIDOR_NAVIGATION_VFHSERVOING_HPP
#define CORRIDOR_NAVIGATION_VFHSERVOING_HPP

#include <vfh_star/Types.h>
#include <vfh_star/VFHStar.h>
#include <vfh_star/TraversabilityGrid.h>
#include <corridor_navigation/VFHServoingConf.hpp>
#include <corridor_navigation/VFHStarDebugData.hpp>
#include <envire/maps/Grid.hpp>

namespace corridor_navigation
{
    class VFHServoing: public vfh_star::VFHStar
    {
    public:
        VFHServoing(const envire::Grid<vfh_star::Traversability> *tr)
            : vfh(tr) {};

        virtual ~VFHServoing() {};

        VFHStarDebugData getVFHStarDebugData(const std::vector< base::Waypoint >& trajectory);

	void clearDebugData();
	
    private:
        vfh_star::VFH vfh;
        mutable VFHStarDebugData debugData;
        AngleIntervals getNextPossibleDirections(const vfh_star::TreeNode& curNode, double obstacleSafetyDist, double robotWidth) const;
        std::pair<base::Pose, bool> getProjectedPose(
                const vfh_star::TreeNode& curNode, double heading, double distance) const;
    };
}

#endif

