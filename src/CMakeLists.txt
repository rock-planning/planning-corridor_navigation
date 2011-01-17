rock_library(corridor_navigation
    SOURCES VFHServoing.cpp VFHFollowing.cpp
    DEPS_PKGCONFIG vfh_star corridor_planner
    HEADERS VFHServoing.hpp VFHFollowing.hpp Types.hpp)

