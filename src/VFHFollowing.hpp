#ifndef CORRIDOR_NAVIGATION_VFHFOLLOWING_HPP
#define CORRIDOR_NAVIGATION_VFHFOLLOWING_HPP

#include <base/eigen.h>
#include <vfh_star/TreeSearch.h>
#include <corridor_planner/corridors.hh>
#include <corridor_navigation/VFHFollowingConf.hpp>

namespace corridor_navigation
{

    /** A behaviour that follows a corridor given to it explicitely
     *
     * It uses the VFH* algorithm for its planning step
     */
    class VFHFollowing : public vfh_star::TreeSearch
    {
    public:
        VFHFollowing();

        void setCostConf(const VFHFollowingConf& conf);

        // Sets the corridor that should be followed
        void setCorridor(const corridors::Corridor& corridor, double desired_terminal_heading = base::unset<double>());

        std::pair<base::geometry::Spline<3>, bool> getTrajectory(
                const base::Pose& current_position, double horizon);

        std::pair<base::Vector3d, base::Vector3d> getHorizon() const;

        double getInitialHorizonDistance() const;

        void computeHorizon(double median_t, base::Vector3d const& current_position);

        /** Distance-to-goal below which the corridor follower announces that we
         * reached our goal
         */
        void setDistanceToGoal(double value);

    private:
        // Configuration of the cost function
        VFHFollowingConf cost_conf;

        double distance_to_goal;

        bool findHorizon(const base::Position& current_position,
                double desired_distance);
        void updateHorizonParameters(base::Position const& current_position);
        void findBoundaryInterpolation(double median_t, base::Position const& current_position, double desired_distance);

        // Used as a cached value in getNextPossibleDirections to avoid memory
        // allocation
        mutable AngleIntervals possible_directions;

        // Discretized version of the corridor curves
        std::vector<base::Position> median_curve;
        std::vector<double> width_curve;
        std::vector<base::Vector3d> median_tangents;
        struct NodeInfo
        {
            int reference_point;
            bool inside;
        };
        mutable std::vector<NodeInfo> node_info;
        std::vector<base::Position> boundary_curves[2];

        // Definition of the planning horizon
        double horizon_boundaries_t[2];
        base::Position horizon_boundaries[2];
        base::Position horizon_normal;
        base::Position horizon_tangent;
        double horizon_length;
        /** The outward direction of the horizon
         */
        double horizon_direction;
        /** The desired heading at the end of the corridor. Use
         * base::unset<double>() to disable
         */
        double desired_terminal_heading;
        /** During search, this is the desired heading at the horizon. It is
         * either set to horizon_direction, desired_terminal_heading or to the
         * goal's direction if the goal is within our line of sight
         */
        double desired_final_heading;
        double initial_horizon_distance;

        // Cached values for getProjectedPose
        mutable bool hasLastProjectedPosition;
        mutable base::Position lastProjectedPosition;
        mutable bool projectionComputeIntersections[2];

        // Used in getNextPossibleDirections. Cached here to reduce the number
        // of memory allocations
        mutable std::vector<double> curve_points;
        mutable std::vector< std::pair<double, double> > curve_segments;

        // The corridor we have to follow
        corridors::Corridor corridor;

        // Computes the distance to the goal line, with positive distances
        // if \c pos is behind the horizon line w.r.t. the direction defined by
        // horizon_normal
        std::pair<double, bool> algebraicDistanceToGoal(const base::Position& pos) const;

        //! Method required by vfh_star::TreeSearch
        bool isTerminalNode(const vfh_star::TreeNode& node) const;

        //! Method required by vfh_star::TreeSearch
        double getHeuristic(const vfh_star::TreeNode &node) const;

        //! Method required by vfh_star::TreeSearch
        virtual AngleIntervals getNextPossibleDirections(const vfh_star::TreeNode& curNode) const;
        
        virtual double getCostForNode(const vfh_star::ProjectedPose& projection, const base::Angle& direction, const vfh_star::TreeNode& parentNode) const;
        virtual std::vector< vfh_star::ProjectedPose > getProjectedPoses(const vfh_star::TreeNode& curNode, const base::Angle& heading, double distance) const;
    
        //! Method required by vfh_star::TreeSearch
        bool updateCost(vfh_star::TreeNode& node, bool is_terminal) const;
        //! Method required by vfh_star::TreeSearch
        bool validateNode(const vfh_star::TreeNode& node) const;
    };
}

#endif

