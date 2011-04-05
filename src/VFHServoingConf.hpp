#ifndef CORRIDOR_NAVIGATION_VFHSERVOING_CONF_HPP
#define CORRIDOR_NAVIGATION_VFHSERVOING_CONF_HPP

namespace corridor_navigation {
    struct VFHServoingConf
    {
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

	/**
	 * Width of the area in front of the robot, 
	 * were more directions are sampled 
	 * */
	double oversamplingWidth;
	
	/**
	 * Radius in which obstacles are sensed
	 * per step.
	 * */
	double obstacleSenseRadius;
	
	/** Some systems are really bad at turning. For those, it is best to
         * force going straight as much as possible.
         *
         * This cost is added to all movements that are not perfectly straight
         */
        double baseTurnCost;
	
	/**
	 * Penaltys for driving over unknown or
	 * terrain inside a sensor shadow
	 * */
	double unknownSpeedPenalty;
	double shadowSpeedPenalty;
	
	/**
	 * This is the minimal speed of the robot
	 * */
	double minimalSpeed;
	
	VFHServoingConf() : unknownSpeedPenalty(0.1), shadowSpeedPenalty(0.1), minimalSpeed(0.01)
	{}
    };
}

#endif


