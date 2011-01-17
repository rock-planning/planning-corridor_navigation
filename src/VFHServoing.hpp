#ifndef CORRIDOR_NAVIGATION_VFHSERVOING_HPP
#define CORRIDOR_NAVIGATION_VFHSERVOING_HPP

#include <vfh_star/Types.h>
#include <vfh_star/VFHStar.h>
#include <vfh_star/TraversabilityGrid.h>
#include <corridor_navigation/Types.hpp>
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

    private:
        vfh_star::VFH vfh;
        mutable VFHStarDebugData debugData;
        AngleIntervals getNextPossibleDirections(const base::Pose& curPose, double obstacleSafetyDist, double robotWidth) const;
        std::pair<base::Pose, bool> getProjectedPose(
                const base::Pose& curPose, double heading, double distance) const;
    };
}

#endif

