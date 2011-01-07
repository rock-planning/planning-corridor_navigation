rock_library(corridor_navigation
    SOURCES dummy.cpp)

rock_executable(corridor_navigation_bin main.cpp
    DEPS corridor_navigation)

