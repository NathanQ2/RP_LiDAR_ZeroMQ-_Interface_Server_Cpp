cmake_minimum_required(VERSION 3.28)
project(RP_LiDAR_Interface_Cpp)

set(CMAKE_CXX_STANDARD 26)

file(GLOB_RECURSE SOURCES src/*.cpp src/*.h)

add_executable(RP_LiDAR_Interface_Cpp ${SOURCES})

target_link_libraries(RP_LiDAR_Interface_Cpp ${PROJECT_SOURCE_DIR}/vendor/rplidar_sdk/output/x64/${CMAKE_BUILD_TYPE}/rplidar_driver.lib)


target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/Boost_1_85_0)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/rplidar_sdk/sdk/include)
