/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "corridor_navigation/VFHFollowing.hpp"
#include <boost/tuple/tuple.hpp>
#include <set>
#include <stdexcept>
#include <boost/lexical_cast.hpp>

using namespace corridor_navigation;
using namespace std;
using vfh_star::TreeNode;

VFHFollowing::VFHFollowing()
{
    possible_directions.resize(1);
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
        int tg2_i = std::min(size, i + 4);
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


base::geometry::Spline<3> VFHFollowing::getTrajectory(const base::Pose& current_pose, double horizon)
{
    if (search_conf.discountFactor != 1)
    {
        std::cerr << "VFHFollowing needs TreeSearch to be configured with a discountFactor of 1" << std::endl;
        search_conf.discountFactor = 1;
    }

    node_info.resize(search_conf.maxTreeSize);

    findHorizon(current_pose.position, horizon);
    hasLastProjectedPosition = false;
    return TreeSearch::getTrajectory(current_pose);
}

void VFHFollowing::setCorridor(const corridors::Corridor& corridor)
{
    this->corridor = corridor;

    median_curve = corridor.median_curve.sample(search_conf.stepDistance / 4);
    computeCurveTangents(median_tangents, median_curve);
    boundary_curves[0]  = corridor.boundary_curves[0].sample(search_conf.stepDistance / 4);
    boundary_curves[1]  = corridor.boundary_curves[1].sample(search_conf.stepDistance / 4);
    t_to_d = 1.0 / this->corridor.median_curve.getUnitParameter();
}

void VFHFollowing::findHorizon(const base::Position& current_position, double desired_distance)
{
    double t0 = corridor.median_curve.findOneClosestPoint(current_position);
    double t1, advance;
    boost::tie(t1, advance) =
        corridor.median_curve.advance(t0, desired_distance, search_conf.stepDistance / 5);
    std::cerr << "advanced " << advance << " on the median curve" << std::endl;

    base::Vector3d travel_direction;
    boost::tie(horizon_center, travel_direction) =
        corridor.median_curve.getPointAndTangent(t1);
    node_info[0].reference_point = getClosest(median_curve, horizon_center);
    for (int i = 0; i < 2; ++i)
    {
        double boundary_t = corridor.boundary_curves[i].
            findOneClosestPoint(horizon_center);
        horizon_boundaries[i] = corridor.boundary_curves[i].
            getPoint(boundary_t);
        horizon_tangents[i] = (horizon_boundaries[i] - horizon_center);
        horizon_tangents[i].normalize();

        base::Vector3d normal = horizon_tangents[i].cross(base::Vector3d::UnitZ());
        if (normal.dot(travel_direction) < 0)
            horizon_normals[i] = -normal;
        else
            horizon_normals[i] = normal;

        horizon_lengths[i] = (horizon_boundaries[i] - horizon_center).norm();
    }

    std::cerr << "planning horizon" << std::endl;
    std::cerr << "  from=" << current_position.x() << " " << current_position.y() << " " << current_position.z() << std::endl;
    std::cerr << "  to=" << horizon_center.x() << " " << horizon_center.y() << " " << horizon_center.z() << std::endl;
    std::cerr << "  from_parameter=" << t0 << " to_parameter=" << t1 << std::endl;
    std::cerr << "  d=" << desired_distance << std::endl;
    std::cerr << "  p=" << horizon_center.x() << " " << horizon_center.y() << " " << horizon_center.z() << std::endl;
    for (int i = 0; i < 2; ++i)
    {
        std::cerr << "  b" << i << "=" << horizon_boundaries[i].x() << " " << horizon_boundaries[i].y() << " " << horizon_boundaries[i].z() << std::endl;
        std::cerr << "  n" << i << "=" << horizon_normals[i].x() << " " << horizon_normals[i].y() << " " << horizon_normals[i].z() << std::endl;
        std::cerr << "  t" << i << "=" << horizon_tangents[i].x() << " " << horizon_tangents[i].y() << " " << horizon_tangents[i].z() << std::endl;
        std::cerr << "  l" << i << "=" << horizon_lengths[i] << std::endl;
    }
}

std::pair<double, bool> VFHFollowing::algebraicDistanceToGoal(const base::Position& pos) const
{
    bool on_segment[2] = { false, false };
    double distances[2];
    for (int i = 0; i < 2; ++i)
    {
        base::Vector3d normal   = horizon_normals[i];
        base::Vector3d tangent  = horizon_tangents[i];
        base::Vector3d boundary = horizon_boundaries[i];

        base::Vector3d v = horizon_center - pos;
        double dist = v.dot(normal);
        double x    = v.dot(tangent);
        if (x < -horizon_lengths[i])
            distances[i] = (boundary - pos).norm();
        else if (x > 0)
            distances[i] = v.norm();
        else
        {
            distances[i] = dist;
            on_segment[i] = true;
        }
    }

    if (distances[0] > distances[1])
        return std::make_pair(distances[1], on_segment[1]);
    else
        return std::make_pair(distances[0], on_segment[0]);
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

static double vector_angles(base::Vector3d const& from, base::Vector3d const& to)
{
    // WARNING: this works because from and to are in 2D !!!!
    double result = atan2(to.y(), to.x()) - atan2(from.y(), from.x());
    if (result < -M_PI) result += 2 * M_PI;
    else if (result > M_PI) result -= 2*M_PI;
    return result;
}

enum INTERVAL_INTERSECTION
{
    INTER_NONE,
    INTER_I2_I1,
    INTER_I1_I2,
    INTER_I2,
    INTER_I1
};

static INTERVAL_INTERSECTION interval_intersection(pair<double, double> i1, pair<double, double> i2)
{
    float i2_second_in_i1 = (i2.second - i1.first) * (i2.second - i1.second);
    float i2_first_in_i1  = (i2.first - i1.first) * (i2.first - i1.second);
    if (i2_second_in_i1 < 0)
    {
        if (i2_first_in_i1 < 0)
            return INTER_I1;
        else
            return INTER_I2_I1;
    }
    else if (i2_first_in_i1 < 0)
        return INTER_I1_I2;
    else
    {
        float i1_first_in_i2 = (i1.first - i2.first) * (i1.first - i2.second);
        if (i1_first_in_i2 < 0)
            return INTER_I2;
        return INTER_NONE;
    }
}

static pair<double, double> interval_merge(INTERVAL_INTERSECTION mode, pair<double, double> i1, pair<double, double> i2)
{
    switch(mode)
    {
        case INTER_I1: return i1;
        case INTER_I2: return i2;
        case INTER_I1_I2: return make_pair(i1.first, i2.second);
        case INTER_I2_I1: return make_pair(i2.first, i1.second);
        default:;
    }
    // never reached
    return make_pair(0, 0);
}

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

    base::Vector3d closest, closest_tangent;
    int reference_index = getReferencePoint(current_pose.position,
            node_info[current_node.getParent()->getIndex()].reference_point, 0,
            median_curve, median_tangents);
    node_info[current_node.getIndex()].reference_point = reference_index;
    base::Vector3d local_tangent = median_tangents[reference_index];

    double local_heading = vector_angles(base::Vector3d::UnitY(), local_tangent);

    pair<double, double> point_turn = make_pair(local_heading - cost_conf.pointTurnAperture, local_heading + cost_conf.pointTurnAperture);

    INTERVAL_INTERSECTION intersection =
        interval_intersection(normal, point_turn);
    if (intersection != INTER_NONE)
        possible_directions.push_back(interval_merge(intersection, normal, point_turn));
    else
    {
        possible_directions.push_back(normal);
        possible_directions.push_back(point_turn);
    }

    return possible_directions;
}

bool VFHFollowing::updateCost(TreeNode& node) const
{
    double d = node_info[node.getIndex()].distance_to_border;
    double cost = 0;
    if (d < cost_conf.safetyDistanceToBorder)
    {
        cost = (cost_conf.safetyDistanceToBorder - d)
            * cost_conf.distanceToBorderWeight;
    }
    else if (!node_info[node.getIndex()].inside)
    {
        cost = (cost_conf.safetyDistanceToBorder + d)
            * cost_conf.distanceToBorderWeight;
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
    const base::Position parent = node.getParent()->getPose().position;
    const base::Position child  = node.getPose().position;

    // Compute the normal to the line between parent and child
    base::Vector3d n = base::Vector3d::UnitZ().cross(child - parent);

    // Later, we compute in which direction we cross the boundaries by looking
    // at the order between the parent->child vector and the boundary tangent at
    // the intersection point
    //
    // This array encodes whether we have an inside2outside crossing if the Z
    // coordinate of the cross product is positive (inside_outside[i] == 1) or
    // negative (inside_outside[i] == -1)
    double inside2outsideSign[2] = { 1, -1 };

    base::Vector3d parent_child = (child - parent);
    double parent_child_dist = parent_child.norm();

    // We find the intersection closest to parent to know if parent is outside.
    // We also are looking for the intersection closest to child to know if
    // child is outside
    //
    bool parent_is_outside = false, child_is_outside = false;
    double closest_parent_inter = -1, closest_child_inter = -1;
    for (int curve_idx = 0; curve_idx < 2; ++curve_idx)
    {
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
    node_info[node.getIndex()].distance_to_border = closest_child_inter;
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

static double getMotionCost(VFHFollowingConf const& cost_conf, double angle_diff, double distance)
{
    double rate_of_turn = angle_diff / distance;
    if (distance == 0 || (cost_conf.pointTurnThreshold > 0 && rate_of_turn > cost_conf.pointTurnThreshold))
    {
        // We have to point turn
        return (angle_diff / cost_conf.pointTurnSpeed)
            + distance / cost_conf.speedAfterPointTurn;
    }
    else
    {
        // Normal movement
        double speed = cost_conf.speedProfile[0] - rate_of_turn * cost_conf.speedProfile[1];
        return distance / speed;
    }

    // never reached
}

double VFHFollowing::getHeuristic(const TreeNode &node) const
{
    double d;
    bool in_segment;
    boost::tie(d, in_segment) = algebraicDistanceToGoal(node.getPose().position);
    if (in_segment && d <= 0)
        return 0;

    return fabs(d);
}


double VFHFollowing::getCostForNode(const base::Pose& pose, double direction, const TreeNode& parentNode) const
{
    double cost = 0;
    double const distance = search_conf.stepDistance;

    // Compute rate of turn
    double angle_diff = direction - parentNode.getDirection();
    if (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;

    angle_diff = fabs(angle_diff);
    cost += getMotionCost(cost_conf, angle_diff, distance);
    return cost;
} 

