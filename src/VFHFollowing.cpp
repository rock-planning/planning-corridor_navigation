/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "corridor_navigation/VFHFollowing.hpp"
#include <boost/tuple/tuple.hpp>
#include <set>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <base/float.h>
#include <base/angle.h>

using namespace corridor_navigation;
using namespace std;
using boost::lexical_cast;
using vfh_star::TreeNode;
using boost::bind;
using boost::ref;

static double vector_angles(base::Vector3d const& from, base::Vector3d const& to)
{
    // WARNING: this works because from and to are in 2D !!!!
    double result = atan2(to.y(), to.x()) - atan2(from.y(), from.x());
    return base::Angle::normalizeRad(result);
}

VFHFollowing::VFHFollowing()
    : distance_to_goal(0.5)
{
    possible_directions.resize(1);
}

void VFHFollowing::setDistanceToGoal(double value)
{
    distance_to_goal = value;
}

void VFHFollowing::setCostConf(const VFHFollowingConf& conf)
{
    this->cost_conf = conf;
}

static void computeCurveTangents(std::vector<base::Vector3d>& tangents, std::vector<base::Position> const& curve)
{
    tangents.clear();
    tangents.reserve(curve.size());
    int size = curve.size();
    for (int i = 0; i < size; ++i)
    {
        int tg1_i = std::max(0, i - 4);
        base::Vector3d tg1 = curve[i] - curve[tg1_i];
        int tg2_i = std::min(size - 1, i + 4);
        base::Vector3d tg2 = curve[tg2_i] - curve[i];
        tangents.push_back((tg2 + tg1) / 2);
    }
}

static int getReferencePoint(base::Position p, int start_t, int search_dir, std::vector<base::Position> const& curve, std::vector<base::Vector3d> const& tangents)
{
    if (search_dir == 0)
    {
        base::Position curve_p = curve[start_t];
        base::Vector3d curve_t = tangents[start_t];
        float current_dir = (p - curve_p).dot(curve_t);
        if (current_dir > 0) search_dir = 1;
        else search_dir = -1;

        int result = getReferencePoint(p, start_t, search_dir, curve, tangents);
        if (result == -1)
            return getReferencePoint(p, start_t, -search_dir, curve, tangents);
        else
            return result;
    }

    for (unsigned int i = start_t; i >= 0 && i < curve.size(); i += search_dir)
    {
        base::Position curve_p = curve[i];
        base::Vector3d curve_t = tangents[i];
        float new_dir = (p - curve_p).dot(curve_t);
        if (new_dir * search_dir < 0)
            return i;
    }
    return -1;
}

static int getClosest(std::vector<base::Position> const& curve, base::Position p)
{
    int min_t = 0;
    double min_d = (curve[0] - p).norm();
    for (unsigned int i = 1; i < curve.size(); ++i)
    {
        double d = (curve[i] - p).norm();
        if (d < min_d)
        {
            min_d = d;
            min_t = i;
        }
    }
    return min_t;
}

std::pair<base::Vector3d, base::Vector3d> VFHFollowing::getHorizon() const
{
    return std::make_pair(horizon_boundaries[0], horizon_boundaries[1]);
}

double VFHFollowing::getInitialHorizonDistance() const
{
    return initial_horizon_distance;
}

std::pair<base::geometry::Spline<3>, bool> VFHFollowing::getTrajectory(const base::Pose& current_pose, double horizon)
{
    if (search_conf.discountFactor != 1)
    {
        std::cerr << "VFHFollowing needs TreeSearch to be configured with a discountFactor of 1" << std::endl;
        search_conf.discountFactor = 1;
    }

    node_info.resize(search_conf.maxTreeSize);

    bool within_range = findHorizon(current_pose.position, horizon);

    hasLastProjectedPosition = false;
    return std::make_pair(TreeSearch::getTrajectory(current_pose), within_range);
}

void VFHFollowing::setCorridor(const corridors::Corridor& corridor, double desired_terminal_heading)
{
    this->corridor = corridor;
    this->desired_terminal_heading = desired_terminal_heading;

    std::vector<double> sampling_t;
    median_curve = corridor.median_curve.sample(search_conf.stepDistance / 4, &sampling_t);

    // Account for rounding errors in the two curves ...
    if (sampling_t.size() > 1)
        sampling_t[sampling_t.size() - 1] = sampling_t[sampling_t.size() - 2];

    std::vector< base::geometry::Spline<1>::vector_t > widths = corridor.width_curve.getPoints(sampling_t);
    width_curve.resize(widths.size());
    for (unsigned int i = 0; i < widths.size(); ++i)
        width_curve[i] = widths[i](0, 0);
    computeCurveTangents(median_tangents, median_curve);
    boundary_curves[0]  = corridor.boundary_curves[0].sample(search_conf.stepDistance / 4);
    boundary_curves[1]  = corridor.boundary_curves[1].sample(search_conf.stepDistance / 4);
}

void VFHFollowing::computeHorizon(double median_t, base::Vector3d const& current_position)
{
    base::Vector3d base_point, travel_direction;
    boost::tie(base_point, travel_direction) =
        corridor.median_curve.getPointAndTangent(median_t);

    for (int curve_idx = 0; curve_idx < 2; ++curve_idx)
    {
        curve_points.clear();
        curve_segments.clear();
        corridor.boundary_curves[curve_idx].
             findClosestPoints(base_point, curve_points, curve_segments);

        double boundary_t = 0;
        if (!curve_points.empty())
            boundary_t = curve_points[0];
        else if (!curve_segments.empty())
            boundary_t = curve_segments[0].first;
        else
            throw std::runtime_error("no closest point (!)");

        for (unsigned int i = 1; i < curve_points.size(); ++i)
        {
            if (boundary_t > curve_points[i])
                boundary_t = curve_points[i];
        }
        for (unsigned int i = 0; i < curve_segments.size(); ++i)
        {
            if (boundary_t > curve_segments[i].first)
                boundary_t = curve_segments[i].first;
        }

        horizon_boundaries_t[curve_idx] = boundary_t;
    }

    updateHorizonParameters(current_position);
}

void VFHFollowing::updateHorizonParameters(base::Position const& current_position)
{
    for (int i = 0; i < 2; ++i)
    {
        horizon_boundaries[i] = corridor.boundary_curves[i].
            getPoint(horizon_boundaries_t[i]);
    }

    base::Vector3d tangent = horizon_boundaries[1] - horizon_boundaries[0];
    horizon_length  = tangent.norm();
    horizon_tangent = tangent / horizon_length;
    // normal must be oriented in the direction opposite of travel so that
    // algebraicDistanceToGoal is > 0 for non-terminal nodes
    horizon_normal    = base::Vector3d::UnitZ().cross(horizon_tangent);
    horizon_direction = vector_angles(base::Vector3d::UnitY(), -horizon_normal);
    initial_horizon_distance = algebraicDistanceToGoal(current_position).first;
}


static std::pair<bool, double> horizonSearchTest(VFHFollowing& follower, base::Vector3d const& initial_position, double desired_distance, double t0, double t1, base::geometry::Spline<3> const& curve)
{
    follower.computeHorizon(t0, initial_position);
    double initial0 = follower.getInitialHorizonDistance();
    follower.computeHorizon(t1, initial_position);
    double initial1 = follower.getInitialHorizonDistance();
    return std::make_pair(initial0 < desired_distance && initial1 > desired_distance, initial1 - initial0);
}

void VFHFollowing::findBoundaryInterpolation(double median_t, base::Position const& current_position, double desired_distance)
{
    double before_t[2] = { horizon_boundaries_t[0], horizon_boundaries_t[1] };
    computeHorizon(median_t, current_position);
    double after_t[2] =  { horizon_boundaries_t[0], horizon_boundaries_t[1] };

    while (true)
    {
        for (int i = 0; i < 2; ++i)
            horizon_boundaries_t[i] = (before_t[i] + after_t[i]) / 2;
        updateHorizonParameters(current_position);

        if (initial_horizon_distance > desired_distance * 1.1)
            copy(horizon_boundaries_t, horizon_boundaries_t + 2, after_t);
        else if (initial_horizon_distance < desired_distance * 0.9)
            copy(horizon_boundaries_t, horizon_boundaries_t + 2, before_t);
        else
            break;
    }
}

bool VFHFollowing::findHorizon(const base::Position& current_position, double desired_distance)
{
    double t0 = corridor.median_curve.findOneClosestPoint(current_position);

    double t1, advance;
    boost::tie(t1, advance) =
        corridor.median_curve.advance(t0, desired_distance, search_conf.stepDistance / 5);

    // Now look for a good horizon
    computeHorizon(t1, current_position);
    std::cerr << "advanced " << advance << " on the median curve" << std::endl;
    std::cerr << "searching between " << t0 << " and " << t1 << std::endl;

    if (initial_horizon_distance > desired_distance * 1.1)
    {
        double before, after;
        boost::tie(before, after) = corridor.median_curve.
            dichotomic_search(t0, t1, bind(horizonSearchTest, ref(*this), current_position, desired_distance, _1, _2, _3), search_conf.stepDistance / 2, 0.0001);
        if (before == after)
        {
            // The horizon at +t0+ is already too far ... Just take it
            computeHorizon(t0, current_position);
            std::cerr << "t0_horizon too far: initial_horizon_distance=" << initial_horizon_distance << std::endl;
        }
        else
        {
            computeHorizon(before, current_position);
            std::cerr << "before=" << before << ", initial_d=" << initial_horizon_distance << std::endl;
            if (fabs(initial_horizon_distance - desired_distance) / desired_distance > 0.1)
            {
                // We have to find an interpolation between after and before, but we
                // have to do it by working on the boundaries, *not* by working on the
                // median curve
                findBoundaryInterpolation(after, current_position, desired_distance);
                std::cerr << "interpolated, initial_d=" << initial_horizon_distance << std::endl;
            }
        }
    }

    base::Vector3d base_point = corridor.median_curve.getPoint(t0);
    node_info[0].reference_point = getClosest(median_curve, base_point);
    node_info[0].inside = true;

    desired_final_heading = base::unset<double>();
    if (t1 == corridor.median_curve.getEndParam())
    {
        if (initial_horizon_distance < this->distance_to_goal)
            return true;
        else if (!base::isUnset(desired_terminal_heading))
            desired_final_heading = desired_terminal_heading;
    }
    
    if (base::isUnset(desired_final_heading))
    {
        // Two options left: either we use the horizon direction, or -- if we
        // have a direct line of sight to the goal -- we use the goal's
        // direction
        base::Vector3d goal = corridor.median_curve.getEndPoint();

        bool hasLineOfSight = true;
        for (int boundary_idx = 0; hasLineOfSight && boundary_idx < 2; ++boundary_idx)
        {
            std::vector<double> points;
            std::vector< std::pair<double, double> > curves;
            corridor.boundary_curves[boundary_idx].findSegmentIntersections(current_position, goal, points, curves, 0.1);
            for (size_t i = 0; i < curves.size(); ++i)
            {
                points.push_back(curves[i].first);
                points.push_back(curves[i].second);
            }

            for (size_t i = 0; i < points.size(); ++i)
            {
                Eigen::Vector3d p = corridor.boundary_curves[boundary_idx].getPoint(points[i]);
                if ((p - goal).norm() > this->distance_to_goal)
                {
                    hasLineOfSight = false;
                    break;
                }
            }
        }

        if (hasLineOfSight)
            desired_final_heading = vector_angles(Eigen::Vector3d::UnitY(), goal - current_position);
        else
            desired_final_heading = horizon_direction;

    }

    std::cerr << "planning horizon" << std::endl;
    std::cerr << "  from=" << current_position.x() << " " << current_position.y() << " " << current_position.z() << std::endl;
    std::cerr << "  to=" << base_point.x() << " " << base_point.y() << " " << base_point.z() << std::endl;
    std::cerr << "  from_parameter=" << t0 << " to_parameter=" << t1 << std::endl;
    std::cerr << "  d=" << desired_distance << std::endl;
    for (int i = 0; i < 2; ++i)
    {
        std::cerr << "  b" << i << "=" << horizon_boundaries[i].x() << " " << horizon_boundaries[i].y() << " " << horizon_boundaries[i].z() << std::endl;
    }
    std::cerr << "  n=" << horizon_normal.x() << " " << horizon_normal.y() << " " << horizon_normal.z() << std::endl;
    std::cerr << "  t=" << horizon_tangent.x() << " " << horizon_tangent.y() << " " << horizon_tangent.z() << std::endl;
    std::cerr << "  l=" << horizon_length << std::endl;
    std::cerr << "  dir=" << horizon_direction << std::endl;
    std::cerr << "  final_dir=" << desired_final_heading << std::endl;
    std::cerr << "  initial point-to-horizon: " << initial_horizon_distance << std::endl;

    return false;
}

std::pair<double, bool> VFHFollowing::algebraicDistanceToGoal(const base::Position& pos) const
{
    base::Vector3d v = pos - horizon_boundaries[0];
    double x    = v.dot(horizon_tangent);
    if (x > horizon_length)
        return std::make_pair((pos - horizon_boundaries[1]).norm(), false);
    else if (x < 0)
        return std::make_pair(v.norm(), false);
    else
        return std::make_pair(v.dot(horizon_normal), true);
}

bool VFHFollowing::isTerminalNode(const TreeNode& node) const
{
    if (!node_info[node.getIndex()].inside)
        return false;

    double d;
    bool in_segment;
    boost::tie(d, in_segment) = algebraicDistanceToGoal(node.getPose().position);
    return in_segment && (d <= 0);
}

// enum INTERVAL_INTERSECTION
// {
//     INTER_NONE,
//     INTER_I2_I1,
//     INTER_I1_I2,
//     INTER_I2,
//     INTER_I1
// };

// static INTERVAL_INTERSECTION interval_intersection(pair<double, double> i1, pair<double, double> i2)
// {
//     float i2_second_in_i1 = (i2.second - i1.first) * (i2.second - i1.second);
//     float i2_first_in_i1  = (i2.first - i1.first) * (i2.first - i1.second);
//     if (i2_second_in_i1 < 0)
//     {
//         if (i2_first_in_i1 < 0)
//             return INTER_I1;
//         else
//             return INTER_I2_I1;
//     }
//     else if (i2_first_in_i1 < 0)
//         return INTER_I1_I2;
//     else
//     {
//         float i1_first_in_i2 = (i1.first - i2.first) * (i1.first - i2.second);
//         if (i1_first_in_i2 < 0)
//             return INTER_I2;
//         return INTER_NONE;
//     }
// }

// static pair<double, double> interval_merge(INTERVAL_INTERSECTION mode, pair<double, double> i1, pair<double, double> i2)
// {
//     switch(mode)
//     {
//         case INTER_I1: return i1;
//         case INTER_I2: return i2;
//         case INTER_I1_I2: return make_pair(i1.first, i2.second);
//         case INTER_I2_I1: return make_pair(i2.first, i1.second);
//         default:;
//     }
//     // never reached
//     return make_pair(0, 0);
// }

vfh_star::TreeSearch::AngleIntervals VFHFollowing::getNextPossibleDirections(const vfh_star::TreeNode& current_node, double safetyDistance, double robotWidth) const
{
    base::Pose current_pose = current_node.getPose();

    possible_directions.clear();

    double current_heading = current_pose.getYaw();
    double threshold = cost_conf.pointTurnThreshold;
    if (threshold == 0)
    {
        possible_directions.push_back(std::make_pair(0, 2 * M_PI));
        return possible_directions;
    }

    double distance = search_conf.stepDistance;
    double angular_threshold = threshold * distance;

    pair<double, double> normal = std::make_pair(current_heading - angular_threshold, current_heading + angular_threshold);

    int reference_index = node_info[current_node.getIndex()].reference_point;
    base::Vector3d local_tangent = median_tangents[reference_index];
    double local_heading = vector_angles(base::Vector3d::UnitY(), local_tangent);

    pair<double, double> point_turn = make_pair(local_heading - cost_conf.pointTurnAperture, local_heading + cost_conf.pointTurnAperture);

    double b0_heading = vector_angles(base::Vector3d::UnitY(), horizon_boundaries[0] - current_node.getPose().position);
    double b0b1 = vector_angles(horizon_boundaries[0] - current_node.getPose().position, horizon_boundaries[1] - current_node.getPose().position);
    pair<double, double> horizon;
    if (b0b1 > 0)
        horizon = make_pair(b0_heading, b0_heading + b0b1);
    else
        horizon = make_pair(b0_heading + b0b1, b0_heading);

    possible_directions.push_back(normal);
    possible_directions.push_back(point_turn);
    possible_directions.push_back(horizon);
    return possible_directions;
}

bool VFHFollowing::updateCost(TreeNode& node, bool is_terminal) const
{
    int reference_point = node_info[node.getIndex()].reference_point;

    // WARN: this is an approximation of the half-width of the corridor. It is
    // an approximation, as it depends on the choice of the reference point on
    // the median.
    double local_width = width_curve[reference_point];
    double distance_to_median = (median_curve[reference_point] - node.getPose().position).norm();

    bool is_inside = node_info[node.getIndex()].inside;

    // See comment above about local_width being an approximation
    //
    // d is the distance from the local border to the current position, positive
    // being inside and negative outside
    double d = 0;
    if (is_inside)
    {
        if (local_width > distance_to_median)
            d = (local_width - distance_to_median);
        else
            d = 0;
    }
    else
    {
        if (distance_to_median > local_width)
            d = (local_width - distance_to_median);
        else
            d = 0;
    }

    // Width of the corridor "inside", i.e. the part of the corridor that is not
    // the safety margin.
    double inside_width = local_width - cost_conf.safetyDistanceToBorder;
    if (inside_width < 0)
        inside_width = 0;

    double cost = 0;
    if (d > cost_conf.safetyDistanceToBorder)
    {
        if (local_width > d)
        {
            // we are inside the corridor and outside the safety margin
            cost += (local_width - d) / inside_width * cost_conf.distanceToBorderWeight[0];
        }
    }
    if (d < cost_conf.safetyDistanceToBorder)
    {
        // We are too close to the border, or outside the corridor
        cost += cost_conf.distanceToBorderWeight[0] +
            (cost_conf.safetyDistanceToBorder - d)
                * cost_conf.distanceToBorderWeight[1];
    }
    if (is_terminal)
    {
        // Add some cost for the final direction (should be aligned with the
        // horizon)
        double angle_to_horizon = base::Angle::normalizeRad(node.getDirection() - desired_final_heading);
        cost += fabs(angle_to_horizon) * cost_conf.finalDirectionCost;
    }

    if (cost != 0)
    {
        node.setCost( node.getCost() + cost );
        return true;
    }
    return false;
}

bool VFHFollowing::validateNode(TreeNode const& node) const
{
    if (node.isRoot())
        return true;

    const base::Position parent = node.getParent()->getPose().position;
    const base::Position child  = node.getPose().position;

    // Get the index of the closest point on the median. We compute it here as
    // it is being used in updateCost() and getNextPossibleDirections(), and
    // validateNode() is the first method called on a node
    node_info[node.getIndex()].reference_point =
        getReferencePoint(node.getPose().position,
            node_info[node.getParent()->getIndex()].reference_point, 0,
            median_curve, median_tangents);

    base::Vector3d parent_child = (child - parent);
    double parent_child_dist = parent_child.norm();
    parent_child /= parent_child_dist;

    // Compute the normal to the line between parent and child
    base::Vector3d n = base::Vector3d::UnitZ().cross(parent_child);

    // Later, we compute in which direction we cross the boundaries by looking
    // at the order between the parent->child vector and the boundary tangent at
    // the intersection point
    //
    // This array encodes whether we have an inside2outside crossing if the Z
    // coordinate of the cross product is positive (inside_outside[i] == 1) or
    // negative (inside_outside[i] == -1)
    double inside2outsideSign[2] = { 1, -1 };

    // We find the intersection closest to parent to know if parent is outside.
    // We also are looking for the intersection closest to child to know if
    // child is outside
    //
    bool parent_is_outside = false, child_is_outside = false;
    double closest_parent_inter = -1, closest_child_inter = -1;
    for (int curve_idx = 0; curve_idx < 2; ++curve_idx)
    {
        curve_points.clear();
        curve_segments.clear();

        base::geometry::Spline<3> const& curve = corridor.boundary_curves[curve_idx];
        curve.findLineIntersections(parent, n,
                curve_points, curve_segments, 0.01);
        for (unsigned int i = 0; i < curve_segments.size(); ++i)
        {
            curve_points.push_back(curve_segments[i].first);
            curve_points.push_back(curve_segments[i].second);
        }
        
        for (unsigned int i = 0; i < curve_points.size(); ++i)
        {
            base::Vector3d intersection_p, intersection_t;
            boost::tie(intersection_p, intersection_t) = curve.
                getPointAndTangent(curve_points[i]);

            double line_x = (intersection_p - parent).dot(parent_child);
            double dist = fabs(line_x);
            double inside2outside = parent_child.cross(intersection_t).z() * inside2outsideSign[curve_idx];
            if (closest_parent_inter < 0 || closest_parent_inter > dist)
            {
                closest_parent_inter = dist;
                parent_is_outside = (inside2outside * line_x < 0);
            }

            line_x -= parent_child_dist;
            dist = fabs(line_x);
            if (closest_child_inter < 0 || closest_child_inter > dist)
            {
                closest_child_inter = dist;
                child_is_outside = (inside2outside * line_x < 0);
            }
        }
    }
    node_info[node.getIndex()].inside = !child_is_outside;

    return (parent_is_outside || !child_is_outside);
}

std::pair<base::Pose, bool> VFHFollowing::getProjectedPose(const vfh_star::TreeNode& curNode,
        double heading, double distance) const
{
    //super omnidirectional robot
    base::Vector3d p(0, distance, 0);
    base::Pose ret;
    ret.orientation = Eigen::AngleAxisd(heading, base::Vector3d::UnitZ());
    ret.position = curNode.getPose().position + ret.orientation * p;
    return std::make_pair(ret, true);
}

double VFHFollowing::getHeuristic(const TreeNode &node) const
{
    double d;
    bool in_segment;
    boost::tie(d, in_segment) = algebraicDistanceToGoal(node.getPose().position);
    if (in_segment && d <= 0)
        return 0;

    return fabs(d) / cost_conf.speedProfile[0];
}

static bool lineIntersection(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, const Eigen::Vector3d &p4, Eigen::Vector3d &intersectionPoint) {
    double a1 = p2.y()-p1.y();
    double b1 = p1.x()-p2.x();
    double c1 = p2.x()*p1.y() - p1.x() * p2.y();// x2*y1 - x1*y2;  { a1*x + b1*y + c1 = 0 is line 1 }

    double a2 = p4.y() - p3.y(); //y4-y3;
    double b2 = p3.x() - p4.x(); //x3-x4;
    double c2 = p4.x()*p3.y() - p3.x()*p4.y(); //x4*y3 - x3*y4;  { a2*x + b2*y + c2 = 0 is line 2 }

    double denom = a1*b2 - a2*b1;
    if( denom == 0 ) {
	return false;
    }

    intersectionPoint.x() = (b1*c2 - b2*c1)/denom;
    intersectionPoint.y() = (a2*c1 - a1*c2)/denom;
    intersectionPoint.z() = 0;
    
    return true;
}


double VFHFollowing::getCostForNode(const base::Pose& pose, double direction, const TreeNode& parentNode) const
{
    double cost = 0;
    double distance = search_conf.stepDistance;

    Eigen::Vector3d intersection_point;
    if (lineIntersection(parentNode.getPose().position, pose.position, horizon_boundaries[0], horizon_boundaries[1], intersection_point))
    {
	intersection_point.z() = parentNode.getPose().position.z();
	double distToGoal = (parentNode.getPose().position - intersection_point).norm();
	if(distToGoal < distance)
	    distance = distToGoal;
    }

    // Compute rate of turn
    double angle_diff = base::Angle::normalizeRad(direction - parentNode.getDirection());
    double desired_speed;
    double rate_of_turn = angle_diff / distance;

    // Check if we must point turn
    if (distance == 0 || (cost_conf.pointTurnThreshold > 0 && rate_of_turn > cost_conf.pointTurnThreshold))
    {
        desired_speed = cost_conf.speedAfterPointTurn;
        cost += angle_diff / cost_conf.pointTurnSpeed;
    }
    else
    {
        desired_speed = cost_conf.speedProfile[0] - rate_of_turn * cost_conf.speedProfile[1];
        if (rate_of_turn != 0)
            cost += cost_conf.baseTurnCost;
    }

    return cost + distance / desired_speed;
} 

