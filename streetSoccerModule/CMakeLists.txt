cmake_minimum_required(VERSION 2.8)

# Create a executable named mymodule
project(streetSoccerModule)

find_package(qibuild)
find_package(OpenCV REQUIRED)


# Create a list of source files
set(_srcs
    CameraStuff.cpp
    CameraStuff.h
    FindBall.cpp
    FindBall.h
    Misc.h
    Movement.cpp
    Movement.h
    main.cpp)

# with the source file: main.cpp
qi_create_bin(streetSoccerModule ${_srcs})

# Add a simple test:
#enable_testing()
#qi_create_test(test_mymodule "test.cpp")

qi_use_lib(streetSoccerModule ALCOMMON ALPROXIES ALVISION OPENCV OPENCV2)
