#ifndef CORRIDOR_NAVIGATION_TYPES_HPP
#define CORRIDOR_NAVIGATION_TYPES_HPP

#include <vfh_star/VFH.h>

namespace corridor_navigation {
    struct VFHStarDebugData {
	std::vector<vfh_star::VFHDebugData> steps;
	std::vector<base::Waypoint> generatedTrajectory;
    };

    typedef vfh_star::VFHStarConf VFHServoingConf;

    /** Configuration structure for VFHFollowing
     *
     * This mostly configures different parts of the cost function. See the
     * documentation of each field for more information
     */
    struct VFHFollowingConf {
        /** If the system is closer than this distance to the corridor boundary,
         * a cost of (safetyDistanceToBorder - d) * distanceToBorderWeight will
         * be added
         */
        double safetyDistanceToBorder;
        /** See comment of safetyDistanceToBorder
         */
        double distanceToBorderWeight;

        /** This factor gives us the linear speed w.r.t. the rate of turn
         * (in rad/m). Speed is given by
         *   
         *   speed = speedProfile[0] - rate_of_turn * speedProfile[1]
         */
        double speedProfile[2];
        /** This is the threshold rate of turn (in rad/m) above which the robot
         * will use point turning instead of going forward
         *
         * The speed after the point turn is given by speedAfterPointTurn
         */
        double pointTurnThreshold;
        /** Speed at which a point turn is done, in rad/s
         */
        double pointTurnSpeed;
        /** Speed after a point turn
         */
        double speedAfterPointTurn;
        
        double pointTurnAperture;

        VFHFollowingConf()
            : safetyDistanceToBorder(0.5)
            , distanceToBorderWeight(1)
            , pointTurnThreshold(0)
            , pointTurnSpeed(0)
            , speedAfterPointTurn(1)
            , pointTurnAperture(M_PI/4)
        {
            speedProfile[0] = 1;
            speedProfile[1] = 0;
        }
    };
}

#endif

