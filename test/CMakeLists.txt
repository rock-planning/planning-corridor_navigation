rock_executable(corridir_servoing_test CorridorServoingTest.cpp
    DEPS corridor_navigation
    DEPS_PKGCONFIG vfh_star-viz vizkit3d vizkit3d-viz envire-viz)

rock_testsuite(test_suite geometry_test.cpp DEPS corridor_navigation)

