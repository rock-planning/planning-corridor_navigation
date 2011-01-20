#ifndef CORRIDOR_NAVIGATION_VFH_DEBUG_DATA_HPP
#define CORRIDOR_NAVIGATION_VFH_DEBUG_DATA_HPP

#include <vfh_star/VFH.h>

namespace corridor_navigation {
    struct VFHStarDebugData {
	std::vector<vfh_star::VFHDebugData> steps;
	std::vector<base::Waypoint> generatedTrajectory;
    };
}

#endif

