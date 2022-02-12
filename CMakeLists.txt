cmake_minimum_required(VERSION 3.2.0)

Project(Liberty)

include_directories(${PROJECT_SOURCE_DIR} include)

add_executable(${PROJECT_NAME} src/main.cpp src/Astar_algo.cpp)

