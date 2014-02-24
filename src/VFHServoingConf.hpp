#ifndef CORRIDOR_NAVIGATION_VFHSERVOING_CONF_HPP
#define CORRIDOR_NAVIGATION_VFHSERVOING_CONF_HPP

#include <vfh_star/Types.h>

namespace corridor_navigation {
    struct VFHServoingConf
    {
        /**
         * Normal speed of the robot
         * */
        double baseSpeed;
        
        /**
         * Factor for reduction the speed in respect ot the angle turned
         * */
        double speedReductionForTurning;
        
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
         * Width of the robot
         * */
        double robotWidth;
        
        /**
         * Height of the robot.
         * Note, this is a 2D model, so height is NOT in Z direction.
         * */
        double robotHeight;
        
        /**
         * Safety radius. The robots dimensions will be extended
         * by this value to every side.
         * */
        double obstacleSafetyRadius;
	
	/** Some systems are really bad at turning. For those, it is best to
         * force going straight as much as possible.
         *
         * This cost is added to all movements that are not perfectly straight
         */
        double baseTurnCost;
	
	/**
         * Maximumt speed penalty caused by
         * bad terrain under the robot. 
	 * */
	double maxInnerSpeedPenalty;
        
        /**
         * Maximum speed penalty caused by bad
         * terrain around the robot.
         * */
	double maxOuterSpeedPenalty;
	
        /**
         * Cost for changing the drive mode
         * */
        double driveModeChangeCost;
        
	/**
	 * This is the minimal speed of the robot
	 * */
	double minimalSpeed;
        
        /**
         * Configuration for the VFH algorithm used
         * for sampling space reduction
         * */
        vfh_star::VFHConf vfhConfig;
	
	VFHServoingConf() : baseSpeed(0), speedReductionForTurning(0), pointTurnThreshold(0), pointTurnSpeed(0), 
                            speedAfterPointTurn(0), robotWidth(0), robotHeight(0),
                            baseTurnCost(0), maxInnerSpeedPenalty(0), maxOuterSpeedPenalty(0), driveModeChangeCost(0), 
                            minimalSpeed(0)
	{}
    };
}

#endif


