#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE "test_tools"
#define BOOST_AUTO_TEST_MAIN

#include <corridor_navigation/Geometry.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/test/execution_monitor.hpp>  

using namespace corridor_navigation;

BOOST_AUTO_TEST_CASE( segment_intersection_test )
{
    Eigen::Vector3d p1(1, 0, 0);
    Eigen::Vector3d p2(-1, 0, 0);

    {
        Eigen::Vector3d result(10, 10, 10);
        Eigen::Vector3d p3(0, 1, 0);
        Eigen::Vector3d p4(0, -1, 0);
        BOOST_CHECK(segmentIntersection(p1, p2, p3, p4, result));
        BOOST_CHECK_SMALL(result.x(), 1e-3);
        BOOST_CHECK_SMALL(result.y(), 1e-3);
    }

    {
        Eigen::Vector3d result(0, 1.5, 0);
        Eigen::Vector3d p3(0, 2, 0);
        Eigen::Vector3d p4(0, 1, 0);
        BOOST_CHECK(!segmentIntersection(p1, p2, p3, p4, result));
    }


    {
        Eigen::Vector3d result(10, 1.5, 0);
        Eigen::Vector3d p3(1, 1, 0);
        Eigen::Vector3d p4(1, -1, 0);
        BOOST_CHECK(segmentIntersection(p1, p2, p3, p4, result));
        BOOST_CHECK_CLOSE(result.x(), 1, 1e-3);
        BOOST_CHECK_SMALL(result.y(), 1e-3);
    }

    {
        Eigen::Vector3d result(1, 1.5, 0);
        Eigen::Vector3d p3(1, 2, 0);
        Eigen::Vector3d p4(1, 1, 0);
        BOOST_CHECK(!segmentIntersection(p1, p2, p3, p4, result));
    }
}




