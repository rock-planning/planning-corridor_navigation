#ifndef CORRIDOR_NAVIGATION_VFHFOLLOWING_CONF_HPP
#define CORRIDOR_NAVIGATION_VFHFOLLOWING_CONF_HPP

namespace corridor_navigation {
    /** Configuration structure for VFHFollowing
     *
     * This mostly configures different parts of the cost function. See the
     * documentation of each field for more information
     */
    struct VFHFollowingConf {
        /** The cost of being close to the border is calculated with two linear
         * parts
         *
         * Let's d be the distance to the closest border, positive values being
         * towards the center
         *
         * inside_width is defined as the distance between the median point and
         * the border, minus the safety distance
         *
         * in d=[safetyDistanceToBorder, local_width], the cost is
         *
         *   c=(1 - (d - inside_width) / inside_width)*distanceToBorderWeight[0]
         *
         * in d=[-Inf, safetyDistanceToBorder], the cost is
         *
         *   c=(0.5 - safetyDistanceToBorder / local_width)*distanceToBorderWeight[0] +
         *      (safetyDistanceToBorder - d) * distanceToBorderWeight[1]
         */
        double safetyDistanceToBorder;
        /** See comment of safetyDistanceToBorder
         */
        double distanceToBorderWeight[2];

        /** This factor gives us the linear speed based on the rate of turn
         *
         * <ul>
         *  <li>speedProfile[0] is the base speed
         *  <li>speedProfile[1] is a speed vs. the rate of turn.
         * </ul>
         *
         * Speed at a ceertain point is given by
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
        
        /** When adding the possible travel directions, we always add an
         * aperture of [-pointTurnAperture, +pointTurnAperture] around the local
         * direction.
         *
         * This avoids searching in all directions: if we point turn, our best
         * bet is to try to be aligned with the corridor ...
         */
        double pointTurnAperture;

        /** The cost of having the terminal node not being aligned to the
         * corridor
         *
         * The additional cost is
         *
         *   (terminal_node_direction - local_direction) * finalDirectionCost
         */
        double finalDirectionCost;

        /** Some systems are really bad at turning. For those, it is best to
         * force going straight as much as possible.
         *
         * This cost is added to all movements that are not perfectly straight
         */
        double baseTurnCost;

        VFHFollowingConf()
            : safetyDistanceToBorder(0.5)
            , pointTurnThreshold(0)
            , pointTurnSpeed(0)
            , speedAfterPointTurn(1)
            , pointTurnAperture(M_PI/4)
            , finalDirectionCost(0)
            , baseTurnCost(0.5)
        {
            distanceToBorderWeight[0] = 0;
            distanceToBorderWeight[1] = 1;
            speedProfile[0] = 1;
            speedProfile[1] = 0;
        }
    };
}

#endif

