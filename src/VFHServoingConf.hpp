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
         * Lenght of the robot.
         * */
        double robotLength;

        /**
         * Size of the radius around the robot, that is used
         * for obstacle avoidance. This parameter is related to
         * maxOuterSpeedPenalty.
         * 
         * */
        double outerRadiusSize;
        
        /**
         * Safety radius. The robots dimensions will be extended
         * by this value to every side.
         * */
        double obstacleSafetyRadius;
	
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
	
        /**
         * If set to true, the planned trajectory will 
         * be cut as soon, as the robot enters unknown terrain.
         * */
        bool cutTrajectoryOnUnknownTerrain;

        /**
         * If set to true, the planner will make all unknown
         * terrain patche BEHIND him an obstacle. This is usefull,
         * if the cutTrajectoryOnUnknownTerrain is also set, and there
         * is no sensor coverage behind the robot.
         * */
        bool makeUnkownTerrainOnStartObstacles;

	VFHServoingConf() : baseSpeed(0), speedReductionForTurning(0), pointTurnThreshold(0), pointTurnSpeed(0), 
                            speedAfterPointTurn(0), robotWidth(0), robotLength(0), outerRadiusSize(0.3), obstacleSafetyRadius(0.0),
                            maxInnerSpeedPenalty(0), maxOuterSpeedPenalty(0), driveModeChangeCost(0), 
                            minimalSpeed(0), cutTrajectoryOnUnknownTerrain(true), makeUnkownTerrainOnStartObstacles(true)
	{}
    };
}

#endif


