# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/cmake-3.28.1-linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.28.1-linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build

# Include any dependencies generated for this target.
include CMakeFiles/heterogeneous_compiler_nc.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/heterogeneous_compiler_nc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/heterogeneous_compiler_nc.dir/flags.make

CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/CGRA.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/CGRA.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/CGRA.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/CGRA.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/CGRA_xml_compiler.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/CGRA_xml_compiler.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/CGRA_xml_compiler.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/CGRA_xml_compiler.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFG.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFG.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFG.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFG.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFGEdge.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFGEdge.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFGEdge.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFGEdge.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFGNode.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFGNode.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFGNode.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DFGNode.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DataPath.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DataPath.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DataPath.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/DataPath.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/FU.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/FU.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/FU.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/FU.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/HeuristicMapper.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/HeuristicMapper.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/HeuristicMapper.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/HeuristicMapper.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/Module.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/Module.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/Module.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/Module.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/PE.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/PE.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/PE.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/PE.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/PathFinderMapper.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/PathFinderMapper.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/PathFinderMapper.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/PathFinderMapper.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/Port.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/Port.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/Port.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/Port.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/RegFile.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/RegFile.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/RegFile.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/RegFile.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.s

CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/flags.make
CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.o: /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/tinyxml2.cpp
CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.o: CMakeFiles/heterogeneous_compiler_nc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.o -MF CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.o.d -o CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.o -c /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/tinyxml2.cpp

CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/tinyxml2.cpp > CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.i

CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/tinyxml2.cpp -o CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.s

# Object files for target heterogeneous_compiler_nc
heterogeneous_compiler_nc_OBJECTS = \
"CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.o" \
"CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.o"

# External object files for target heterogeneous_compiler_nc
heterogeneous_compiler_nc_EXTERNAL_OBJECTS =

heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/CGRA.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/CGRA_xml_compiler.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/DFG.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/DFGEdge.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/DFGNode.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/DataPath.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/FU.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/HeuristicMapper.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/Module.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/PE.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/PathFinderMapper.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/Port.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/RegFile.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/tinyxml2.cpp.o
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/build.make
heterogeneous_compiler_nc: CMakeFiles/heterogeneous_compiler_nc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking CXX executable heterogeneous_compiler_nc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/heterogeneous_compiler_nc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/heterogeneous_compiler_nc.dir/build: heterogeneous_compiler_nc
.PHONY : CMakeFiles/heterogeneous_compiler_nc.dir/build

CMakeFiles/heterogeneous_compiler_nc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/heterogeneous_compiler_nc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/heterogeneous_compiler_nc.dir/clean

CMakeFiles/heterogeneous_compiler_nc.dir/depend:
	cd /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build /home/jack2bs/Documents/REVAMP-2/REVAMP/HETEROGENEOUS_MAPPER/src/build/CMakeFiles/heterogeneous_compiler_nc.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/heterogeneous_compiler_nc.dir/depend
