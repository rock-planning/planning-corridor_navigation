#ifndef CORRIDOR_NAVIGATION_INTERSECTION_HPP
#define CORRIDOR_NAVIGATION_INTERSECTION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry> 

namespace corridor_navigation {
    inline bool segmentIntersection(
            const Eigen::Vector3d &_a1,
            const Eigen::Vector3d &_a2,
            const Eigen::Vector3d &_b1,
            const Eigen::Vector3d &_b2,
            Eigen::Vector3d &intersectionPoint) {

        Eigen::Vector2d
            a1(_a1.x(), _a1.y()),
            a2(_a2.x(), _a2.y()),
            b1(_b1.x(), _b1.y()),
            b2(_b2.x(), _b2.y());

        // Normal of the two lines
        Eigen::Vector2d n1(-(a2.y() - a1.y()), a2.x() - a1.x());
        Eigen::Vector2d n2(-(b2.y() - b1.y()), b2.x() - b1.x());

        // Inverted matrix of the 2-equation system
        Eigen::Matrix2d m_inv;
        m_inv << n2.y(), -n1.y(), -n2.x(), n1.x();

        // Determinant
        double det = n1.x() * n2.y() - n1.y() * n2.x();

        if (det < 1e-6)
            return false;

        // Intersection point
        double n1a1 = n1.dot(a1);
        double n2b1 = n2.dot(b1);
        Eigen::Vector2d inter = 1/det * m_inv * Eigen::Vector2d(n1a1, n2b1);

        // Now check that the intersection point is on the segment
        if ((inter - a1).dot(inter - a2) > 0)
            return false;
        if ((inter - b1).dot(inter - b2) > 0)
            return false;

        intersectionPoint.x() = inter.x();
        intersectionPoint.y() = inter.y();
        intersectionPoint.z() = 0;

        return true;
    }
}

#endif

