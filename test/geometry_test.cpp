#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE "test_tools"
#define BOOST_AUTO_TEST_MAIN

#include <corridor_navigation/Geometry.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/test/execution_monitor.hpp>  
#include <corridor_navigation/VFHFollowing.hpp>

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


BOOST_AUTO_TEST_CASE( vector_tests )
{
    
    double cosDirections = Eigen::Vector3d::UnitY().dot(Eigen::Vector3d::UnitY().normalized());
    
    double lastDirection = acos(cosDirections);
    if(cosDirections <0)
	lastDirection *= -1;
    
    BOOST_CHECK(lastDirection == 0);

    {
	Eigen::Vector3d v1 = Eigen::Vector3d::UnitY();
	Eigen::Vector3d v2 = Eigen::Vector3d(1,1,0);
	double cosDirections = v1.dot(v2.normalized());
    
	double lastDirection = acos(cosDirections);
	if(cosDirections <0)
	    lastDirection *= -1;
	
	double lastdir2 = atan2(v2.y(),v2.x()) - atan2(v1.y(),v1.x());
	std::cout << v2.transpose() << " dir " << lastDirection << " " << lastdir2 << std::endl;
	
	BOOST_CHECK(lastDirection < 0);	
    }

        {
	Eigen::Vector3d v1 = Eigen::Vector3d::UnitY();
	Eigen::Vector3d v2 = Eigen::Vector3d(-1,1,0);
	double cosDirections = v1.dot(v2.normalized());
    
	double lastDirection = acos(cosDirections);
	if(cosDirections <0)
	    lastDirection *= -1;
	
	double lastdir2 = atan2(v2.y(),v2.x()) - atan2(v1.y(),v1.x());
	std::cout << v2.transpose() << " dir " << lastDirection << " " << lastdir2 << std::endl;
	
	BOOST_CHECK(lastDirection > 0);	
    }


}

BOOST_AUTO_TEST_CASE( following_algebraic_distance )
{
    VFHFollowing following;
    corridors::Corridor corridor;
    
    std::vector<base::Vector3d> points;
    points.push_back(base::Vector3d(0,0,0));
    points.push_back(base::Vector3d(10,0,0));
   
    corridor.boundary_curves[0].interpolate(points);

    points.clear();
    points.push_back(base::Vector3d(0,5,0));
    points.push_back(base::Vector3d(10,5,0));

    corridor.boundary_curves[1].interpolate(points);

    std::cout << "Test 1" << std::endl;
    typedef base::geometry::Spline<1>::vector_t point_t;
    std::vector<point_t > points1d;
    point_t width_p;
    width_p(0, 0) = 0;
    points1d.push_back(width_p);
    width_p(0, 0) = 10;
    points1d.push_back(width_p);
    corridor.width_curve.interpolate(points1d);

    points.clear();
    points.push_back(base::Vector3d(0,2.5,0));
    points.push_back(base::Vector3d(10,2.5,0));
    
    corridor.median_curve.interpolate(points);
    
    std::cout << "Test 6" << std::endl;

    following.setCorridor(corridor);
    
    std::pair<base::Vector3d, base::Vector3d> horizon = following.getHorizon();
    std::cout << horizon.first.transpose() << " " << horizon.second.transpose() << std::endl;
//     {
//         Eigen::Vector3d result(10, 10, 10);
//         Eigen::Vector3d p3(0, 1, 0);
//         Eigen::Vector3d p4(0, -1, 0);
//         BOOST_CHECK(segmentIntersection(p1, p2, p3, p4, result));
//         BOOST_CHECK_SMALL(result.x(), 1e-3);
//         BOOST_CHECK_SMALL(result.y(), 1e-3);
//     }

}



