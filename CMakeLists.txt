cmake_minimum_required(VERSION 3.20)
project(RP_LiDAR_Interface_Cpp)

set(CMAKE_CXX_STANDARD 23)

file(GLOB_RECURSE SOURCES src/*.cpp src/*.h)

add_subdirectory(vendor/rplidar_sdk)

add_executable(RP_LiDAR_Interface_Cpp ${SOURCES})

target_link_libraries(RP_LiDAR_Interface_Cpp PRIVATE rplidar_driver)

# There has to be a better way to include all these boost headers...
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/core/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/winapi/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/predef/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/move/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/intrusive/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/container/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/static_assert/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/detail/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/assert/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/config/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/detail/include)
target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/boost/libs/interprocess/include)

target_include_directories(RP_LiDAR_Interface_Cpp PRIVATE ${PROJECT_SOURCE_DIR}/vendor/rplidar_sdk/sdk/include)
