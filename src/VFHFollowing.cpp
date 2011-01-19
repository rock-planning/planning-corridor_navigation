/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "corridor_navigation/VFHFollowing.hpp"
#include <boost/tuple/tuple.hpp>
#include <set>

#include <boost/tuple/tuple.hpp>
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

base::geometry::Spline<3> VFHFollowing::getTrajectory(const base::Pose& current_pose, double horizon)
{
    if (search_conf.discountFactor != 1)
    {
        std::cerr << "VFHFollowing needs TreeSearch to be configured with a discountFactor of 1" << std::endl;
        search_conf.discountFactor = 1;
    }

    findHorizon(current_pose.position, horizon);
    hasLastProjectedPosition = false;
    return TreeSearch::getTrajectory(current_pose);
}

void VFHFollowing::setCorridor(const corridors::Corridor& corridor)
{
    this->corridor = corridor;
    t_to_d = 1.0 / this->corridor.median_curve.getUnitParameter();
}

void VFHFollowing::findHorizon(const base::Position& current_position, double desired_distance)
{
    double t0 = corridor.median_curve.findOneClosestPoint(current_position);
    double t1 = t0 + desired_distance + t_to_d;
    if (t1 > corridor.median_curve.getEndParam())
        t1 = corridor.median_curve.getEndParam();

    base::Vector3d travel_direction;
    boost::tie(horizon_center, travel_direction) =
        corridor.median_curve.getPointAndTangent(t1);
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
    std::cerr << "  from_parameter=" << t0 << " to_parameter=" << t1 << std::endl;
    std::cerr << "  d=" << desired_distance << std::endl;
    std::cerr << "  p=" << horizon_center.x() << " " << horizon_center.y() << " " << horizon_center.z() << std::endl;
    for (int i = 0; i < 2; ++i)
    {
        std::cerr << "  b" << i << "=" << horizon_boundaries[i].x() << " " << horizon_boundaries[i].y() << " " << horizon_boundaries[i].z() << std::endl;
        std::cerr << "  n" << i << "=" << horizon_normals[i].x() << " " << horizon_normals[i].y() << " " << horizon_normals[i].z() << std::endl;
        std::cerr << "  t" << i << "=" << horizon_tangents[i].x() << " " << horizon_tangents[i].y() << " " << horizon_tangents[i].z() << std::endl;
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
        if (x > horizon_lengths[i])
            distances[i] = (boundary - pos).norm();
        else if (x < 0)
            distances[i] = v.norm();
        else
        {
            distances[i] = dist;
            on_segment[i] = true;
        }
    }

    // std::cerr << "distance of " << pos.x() << " " << pos.y() << " " << pos.z() << " to goal" << std::endl;
    // std::cerr << "  " << distances[0] << "(" << on_segment[0] << ") " << distances[1] << "(" << on_segment[1] << ")" << std::endl;

    if (distances[0] > distances[1])
        return std::make_pair(distances[1], on_segment[1]);
    else
        return std::make_pair(distances[0], on_segment[0]);
}

bool VFHFollowing::isTerminalNode(const TreeNode& node) const
{
    double d;
    bool in_segment;
    boost::tie(d, in_segment) = algebraicDistanceToGoal(node.getPose().position);
    return in_segment && (d <= 0);
}

static double vector_angles(base::Vector3d const& from, base::Vector3d const& to)
{
    // WARNING: this works because from and to are in 2D !!!!
    double result = atan2(from.y(), from.x()) - atan2(to.y(), to.x());
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

    double closest_t = corridor.median_curve.findOneClosestPoint(current_pose.position);
    base::Vector3d closest, closest_tangent;
    boost::tie(closest, closest_tangent) =
        corridor.median_curve.getPointAndTangent(closest_t);
    double local_heading = vector_angles(base::Vector3d::UnitY(), closest_tangent - closest);

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

    typedef std::vector< std::pair<double, double> > CurveSegments;
    typedef std::set< std::pair<double, double> > RejectedCurveSegments;
    RejectedCurveSegments rejected_segments;

    static const double geores = 0.01;

    base::Position current_p = current_pose.position;

    //std::cerr << "looking for possible directions from " << current_p.x() << " " << current_p.y() << " " << current_p.z() << std::endl;

    // Get the intersection of a stepDistance sphere centered on \c current_pose
    // with the corridor boundaries. Reject every direction that lead us out of
    // the corridor
    for (int i = 0; i < 2; ++i)
    {
        base::geometry::Spline<3> const& boundary = corridor.boundary_curves[i];
        boundary.findSphereIntersections(current_p, search_conf.stepDistance,
                curve_points, curve_segments, 0.001);

        // std::cerr << curve_points.size() << " point intersects with boundary " << i << std::endl;
        //std::cerr << curve_segments.size() << " segments intersects with boundary " << i << std::endl;

        if (curve_points.empty() && curve_segments.empty())
            continue;

        std::set<double> sorted_points(curve_points.begin(), curve_points.end());
        for (CurveSegments::const_iterator it = curve_segments.begin();
                it != curve_segments.end(); ++it)
        {
            sorted_points.insert(it->first);
            sorted_points.insert(it->second);
            // std::cerr << "  segment: " << it->first << " " << it->second << std::endl;
        }
        sorted_points.insert(boundary.getEndParam());

        // std::cerr << sorted_points.size() - 1 << " intersection points in sorted_points" << std::endl;

        // If true, +from+ has already been added to a rejected_segment element.
        // Otherwise, it has never been added. If a point is added neither as
        // 'from' or 'to' in a segment, it gets added as a singular (p, p) pair
        //
        // The first point is artificial, and should therefore be considered as
        // "added"
        bool used_from = true;
        double from_t = boundary.getStartParam();
        base::Vector3d from = boundary.getPoint(from_t);
        for (set<double>::const_iterator it = sorted_points.begin(); it != sorted_points.end(); ++it)
        {
            base::Vector3d to = boundary.getPoint(*it);
            base::Vector3d median = boundary.getPoint((*it + from_t) / 2);

            double from_theta = vector_angles(base::Vector3d::UnitX(), from - current_p);
            double diff_theta = vector_angles(from - current_p, to - current_p);
            // std::cerr << "looking at pair " << from_t << " " << from.x() << " " << from.y() << " " << from.z() << " h=" << from_theta << std::endl;
            // std::cerr << "            and " << *it << " " << to.x() << " " << to.y() << " " << to.z() << " h=" << from_theta + diff_theta << std::endl;
            // std::cerr << "        opening " << diff_theta * 180 / M_PI << std::endl;

            if ((current_p - median).norm() - geores < search_conf.stepDistance)
            {
                //std::cerr << "   is within disk boundaries" << std::endl;
                if (diff_theta > 0)
                    rejected_segments.insert( std::make_pair(from_theta, from_theta + diff_theta) );
                else
                    rejected_segments.insert( std::make_pair(from_theta + diff_theta, from_theta) );
                used_from = true;
            }
            else if (!used_from)
            {
                // The point in +from+ has not been added at the last iteration,
                // and is not going to be added at this one. Add it as a
                // singular pair. See comment on the declaration of used_from
                // for more explanations.
                double from_theta = vector_angles(base::Vector3d::UnitX(), from - current_p);
                //std::cerr << "   intersection at single point: " << from_theta << std::endl;
                rejected_segments.insert( std::make_pair(from_theta, from_theta) );
            }
            else
            {
                used_from = false;
            }
            from = to;
            from_t = *it;
        }
    }

    //std::cerr << "  " << rejected_segments.size() << " rejected segments" << std::endl;
    if (rejected_segments.empty())
    {
        possible_directions.push_back(std::make_pair(0, 2 * M_PI));
        return possible_directions;
    }

    std::pair<double, double> current_rejected = *rejected_segments.begin();
    for (RejectedCurveSegments::const_iterator it = rejected_segments.begin(); it != rejected_segments.end(); ++it)
    {
        std::pair<double, double> new_rejected = *it;
        if (current_rejected.second >= new_rejected.first)
        {
            if (current_rejected.second < new_rejected.second)
                current_rejected.second = new_rejected.second;
            continue;
        }

        // std::cerr << "  [" << current_rejected.second << ", " << new_rejected.first << "]" << std::endl;
        possible_directions.push_back(std::make_pair(current_rejected.second, new_rejected.first));
        current_rejected = new_rejected;
    }

    //std::cerr << "  [" << current_rejected.second << ", " << rejected_segments.begin()->first + 2 * M_PI << "]" << std::endl;
    possible_directions.push_back(
            std::make_pair(current_rejected.second, rejected_segments.begin()->first + 2 * M_PI));
    return possible_directions;
}

bool VFHFollowing::validateNode(const TreeNode& node) const
{
    const base::Position parent = node.getParent()->getPose().position;
    const base::Position child  = node.getPose().position;

    // if (!hasLastProjectedPosition || (lastProjectedPosition != parent))
    // {
    //     // We use the fact, here, that getProjectedPose gets called repeatedly
    //     // with the same pose.
    //     //
    //     // We cache the min distance between curPose and the boundaries, so as
    //     // to not test intersections unnecessarily
    //     hasLastProjectedPosition = true;
    //     lastProjectedPosition = parent;
    //     projectionComputeIntersections[0] =
    //         (corridor.boundary_curves[0].distanceTo(lastProjectedPosition) <= search_conf.stepDistance);
    //     projectionComputeIntersections[1] =
    //         (corridor.boundary_curves[1].distanceTo(lastProjectedPosition) <= search_conf.stepDistance);
    // }

    // Compute the normal to the line between parent and child
    base::Quaterniond q;
    q = Eigen::AngleAxisd(M_PI / 2, base::Vector3d::UnitZ());
    base::Vector3d n = q * (child - parent);

    for (int i = 0; i < 2; ++i)
    {
        // if (!projectionComputeIntersections[i])
        //     continue;

        base::geometry::Spline<3> const& boundary = corridor.boundary_curves[i];
        boundary.findLineIntersections(parent, n,
                curve_points, curve_segments, 0.01);

        for (unsigned int i = 0; i < curve_segments.size(); ++i)
        {
            curve_points.push_back(curve_segments[i].first);
            curve_points.push_back(curve_segments[i].second);
        }

        for (unsigned int i = 0; i < curve_points.size(); ++i)
        {
            double intersection_t = curve_points[i];
            // std::cerr << intersection_t << " " << boundary.getStartParam() << " " << boundary.getEndParam() << std::endl;
            base::Vector3d intersection_p = boundary.
                getPoint(curve_points[i]);

            double d_cur_intersection  = (intersection_p - parent).norm();
            double d_next_intersection = (intersection_p - child).norm();

            if (d_cur_intersection < 0.01 || d_next_intersection < 0.01)
                return false;
            double dir = (intersection_p - parent).
                dot(intersection_p - child);

            if (dir < 0)
                return false;
        }

        //if (has_intersection)
        //{
        //    double d_cur_intersection = (intersection_p - curPose.position).norm();
        //    double d_next_intersection = (intersection_p - ret.position).norm();

        //    if (d_cur_intersection < 0.01 || d_next_intersection < 0.01)
        //        return std::make_pair(ret, false);

        //    double dir = (intersection_p - curPose.position).dot(intersection_p - ret.position);
        //    if (dir < 0)
        //    {
        //        std::cerr << "found intersection at " << intersection_t << " " << intersection_p.x() << " " << intersection_p.y() << std::endl;
        //        std::cerr << "curPose is  " << curPose.position.x() << " " << curPose.position.y() << std::endl;
        //        std::cerr << "nextPose is " << ret.position.x() << " " << ret.position.y() << std::endl;
        //        std::cerr << "heading is  " << heading << std::endl;
        //        std::cerr << "        d(curPose, intersection) = " << (curPose.position - intersection_p).norm() << std::endl;
        //        std::cerr << "        d(curPose, nextPose) = " << (curPose.position - ret.position).norm() << std::endl;
        //        std::cerr << "        d(intersection, nextPose) = " << (intersection_p - ret.position).norm() << std::endl;
        //        return std::make_pair(ret, false);
        //    }

        //    if (false)
        //    {
        //        std::cerr << "found intersection at " << intersection_t << " " << intersection_p.x() << " " << intersection_p.y() << std::endl;
        //        std::cerr << "curPose is  " << curPose.position.x() << " " << curPose.position.y() << std::endl;
        //        std::cerr << "nextPose is " << ret.position.x() << " " << ret.position.y() << std::endl;
        //        std::cerr << "heading is  " << heading << std::endl;
        //        std::cerr << "        d(curPose, intersection) = " << (curPose.position - intersection_p).norm() << std::endl;
        //        std::cerr << "        d(curPose, nextPose) = " << (curPose.position - ret.position).norm() << std::endl;
        //        std::cerr << "        d(intersection, nextPose) = " << (intersection_p - ret.position).norm() << std::endl;
        //        throw std::runtime_error("crossing corridor boundary " + boost::lexical_cast<string>(i));
        //    }
        //}
    }
    return true;
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
        // std::cerr << "point turn " << angle_diff << " " << cost << std::endl;
    }
    else
    {
        // Normal movement
        double speed = cost_conf.speedProfile[0] - rate_of_turn * cost_conf.speedProfile[1];
        return distance / speed;
        // std::cerr << "normal " << rate_of_turn << " " << cost << std::endl;
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

    // Check distance to boundaries
    // base::Position p = pose.position;
    // for (int i = 0; i < 2; ++i)
    // {
    //     double d_boundary = corridor.boundary_curves[i].distanceTo(p);
    //     if (d_boundary < cost_conf.safetyDistanceToBorder)
    //     {
    //         cost += (cost_conf.safetyDistanceToBorder - d_boundary)
    //             * cost_conf.distanceToBorderWeight;
    //     }
    // }

    // Compute rate of turn
    double angle_diff = direction - parentNode.getDirection();
    if (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;

    angle_diff = fabs(angle_diff);
    cost += getMotionCost(cost_conf, angle_diff, distance);
    return cost;
} 

