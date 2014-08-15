cmake_minimum_required(VERSION 2.8)
project(streetSoccerModule)

find_package(qibuild)

# Create a executable named streetSoccerModule
# with the source file: main.cpp
qi_create_bin(streetSoccerModule "main.cpp")

# Add a simple test:
enable_testing()
qi_create_test(test_streetSoccerModule "test.cpp")
