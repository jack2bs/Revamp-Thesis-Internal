cmake_minimum_required(VERSION 3.5)
FILE(GLOB MyCPPSources *.cpp)
FILE(GLOB MyHSources *.h)
set(CMAKE_CXX_STANDARD 11)
set(PROJ "homogeneous_compiler")

find_package(nlohmann_json CONFIG REQUIRED)

add_executable(${PROJ} ${MyCPPSources} ${MyHSources})

# Use C++11 to compile our pass (i.e., supply -std=c++11).
target_compile_features(${PROJ} PRIVATE cxx_range_for cxx_auto_type)
target_link_libraries(${PROJ} nlohmann_json::nlohmann_json)


set(PROJ1 "heterogeneous_compiler_n")

find_package(nlohmann_json CONFIG REQUIRED)

add_executable(${PROJ1} ${MyCPPSources} ${MyHSources})
#set_target_properties(${PROJ1} PROPERTIES compile_flags "-DDUAL_STREAMING")
target_compile_definitions(${PROJ1} PUBLIC DUAL_STREAMING)

# Use C++11 to compile our pass (i.e., supply -std=c++11).
target_compile_features(${PROJ1} PRIVATE cxx_range_for cxx_auto_type)
target_link_libraries(${PROJ1} nlohmann_json::nlohmann_json)


set(PROJ2 "heterogeneous_compiler_c")

find_package(nlohmann_json CONFIG REQUIRED)

add_executable(${PROJ2} ${MyCPPSources} ${MyHSources})
#set_target_properties(${PROJ1} PROPERTIES compile_flags "-DROUTER_CONFIG -DOPCODE_CONST_CONFIG")
target_compile_definitions(${PROJ2} PUBLIC OPCODE_CONST_CONFIG)

# Use C++11 to compile our pass (i.e., supply -std=c++11).
target_compile_features(${PROJ2} PRIVATE cxx_range_for cxx_auto_type)
target_link_libraries(${PROJ2} nlohmann_json::nlohmann_json)

set(PROJ3 "heterogeneous_compiler_nc")

find_package(nlohmann_json CONFIG REQUIRED)

add_executable(${PROJ3} ${MyCPPSources} ${MyHSources})
#set_target_properties(${PROJ1} PROPERTIES compile_flags "-DDUAL_STREAMING -DROUTER_CONFIG -DOPCODE_CONST_CONFIG")
target_compile_definitions(${PROJ3} PUBLIC DUAL_STREAMING OPCODE_CONST_CONFIG)

# Use C++11 to compile our pass (i.e., supply -std=c++11).
target_compile_features(${PROJ3} PRIVATE cxx_range_for cxx_auto_type)
target_link_libraries(${PROJ3} nlohmann_json::nlohmann_json)
