cmake_minimum_required(VERSION 3.20)
project(RP_LiDAR_Interface_Cpp)

set(CMAKE_CXX_STANDARD 23)

file(GLOB_RECURSE SOURCES src/*.cpp src/*.h)

add_subdirectory(vendor/rplidar_sdk)
add_subdirectory(vendor/boost EXCLUDE_FROM_ALL)

add_executable(RP_LiDAR_Interface_Cpp ${SOURCES})

target_link_libraries(RP_LiDAR_Interface_Cpp PRIVATE rplidar_driver)
target_link_libraries(RP_LiDAR_Interface_Cpp PRIVATE Boost::asio)

target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${Boost_INCLUDE_DIRS})

target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/rplidar_sdk/sdk/include)
