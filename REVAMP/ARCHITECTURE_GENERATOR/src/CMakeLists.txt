cmake_minimum_required(VERSION 3.5)
FILE(GLOB MyCPPSources *.cpp)
FILE(GLOB MyHSources *.h)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(PROJ "generator")

find_package(nlohmann_json CONFIG REQUIRED)

add_executable(${PROJ} ${MyCPPSources} ${MyHSources})

# Use C++11 to compile our pass (i.e., supply -std=c++11).
target_compile_features(${PROJ} PRIVATE cxx_range_for cxx_auto_type)
target_link_libraries(${PROJ} nlohmann_json::nlohmann_json)

