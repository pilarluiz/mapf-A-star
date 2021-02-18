cmake_minimum_required(VERSION 3.10)
project(mapf-adapters)
set (CMAKE_CXX_STANDARD 17)


add_subdirectory(library/libMultiRobotPlanning)
#add_subdirectory(library/yaml-cpp)
add_subdirectory(library/bcp-mapf-mirror)
add_subdirectory(library/icts)
add_subdirectory(library/SAT_solver/reLOC-0.x)
add_subdirectory(library/EPEA_solver)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

add_library(mapf-adapters INTERFACE)
target_include_directories(
  mapf-adapters
  INTERFACE 
  include 
  library/libMultiRobotPlanning/include 
 # library/yaml-cpp/include
  library/bcp-mapf-mirror/library
  library/bcp-mapf-mirror
  library/bcp-mapf-mirror/bcp
  library/icts/include
  library/SAT_solver/reLOC-0.x
library/SAT_solver/reLOC-0.x/src
library/SAT_solver/reLOC-0.x/src/core
library/SAT_solver/reLOC-0.x/src/glucose
library/EPEA_solver/include
)


# Include CPLEX.
set(LPS cpx CACHE STRING "options for LP solver")
find_path(CPLEX_INCLUDE_DIR
        NAMES cplex.h
        HINTS "${CPLEX_DIR}/include/ilcplex" "$ENV{CPLEX_DIR}/include/ilcplex")
find_library(CPLEX_LIBRARY
        NAMES cplex
        HINTS "${CPLEX_DIR}/lib/*/static_pic" "$ENV{CPLEX_DIR}/lib/*/static_pic")
if (NOT CPLEX_INCLUDE_DIR)
    message(FATAL_ERROR "CPLEX includes not found")
endif ()
if (NOT CPLEX_LIBRARY)
    message(FATAL_ERROR "CPLEX library not found")
endif ()
include_directories(${CPLEX_INCLUDE_DIR})

# Set stuff for SCIP 6.0.2.
set(WITH_SCIPDEF on)
set(TPI none)
set(tpisources tpi/tpi_none.c)
set(THREAD_LIBRARIES "")
set(TPI_NONE on)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/scipoptsuite-6.0.2/scip/src/scip/config.h.in ${CMAKE_BINARY_DIR}/scip/config.h @ONLY)
include_directories(${CMAKE_BINARY_DIR})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ldl")

# Include SCIP.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/scipoptsuite-6.0.2/scip)
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/scipoptsuite-6.0.2/scip/src)

# Link to math library.
find_library(LIBM m)
if (NOT LIBM)
    set(LIBM "")
endif ()

# Include fmt.
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/fmt/include)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/fmt)

include_directories(SYSTEM library/bcp-mapf-mirror/bcp)

# Include cxxopts.
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/cxxopts/include)
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/robin-hood-hashing/src/include)

# Enable the families of cuts.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_WAITEDGE_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_RECTANGLE_KNAPSACK_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CORRIDOR_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_WAITDELAY_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_EXITENTRY_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_TWOEDGE_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_GOAL_CONFLICTS")

# Set flags.
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -m64")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-ignored-attributes")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -funroll-loops -fstrict-aliasing")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} -g")

# Turn on link-time optimization for Linux.
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif ()



# Include Truffle Hog.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/library)
set(TRUFFLEHOG_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/robin-hood-hashing/src/include/robin_hood.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/Includes.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/Debug.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/Coordinates.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/Map.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/AgentsData.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/Instance.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/Instance.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/PriorityQueue.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/LabelPool.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/LabelPool.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/Heuristic.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/Heuristic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/EdgePenalties.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/Crossings.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/AStar.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/AStar.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/ReservationTable.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/library/trufflehog/ReservationTable.cpp
)
set(SAT_SOURCE_FILES
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/compile.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/config.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/defs.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/insolver_main.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/result.h

${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/cnf.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/cnf.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/complete.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/complete.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/compress.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/compress.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/encodings_binary.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/encodings_derived.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/encodings_direct.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/encodings_extract.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/encodings_mdd.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/graph.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/graph.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/hierarch.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/hierarch.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/multirobot.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/multirobot.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/reloc.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/reloc.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/search.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/search.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/statistics.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/statistics.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/types.cpp
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/core/types.h

${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Alg.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Alloc.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/BoundedQueue.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Constants.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Dimacs.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Heap.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/IntTypes.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Map.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Options.cc
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Options.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/ParseUtils.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Queue.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Solver.cc
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Solver.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/SolverTypes.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Sort.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/System.cc
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/System.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/Vec.h
${CMAKE_CURRENT_SOURCE_DIR}/library/SAT_solver/reLOC-0.x/src/glucose/XAlloc.h)
# Set source files.
set(MAPF_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Includes.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Debug.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Coordinates.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Reader.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Reader.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/ProblemData.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/ProblemData.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/VariableData.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/VariableData.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Pricer_TruffleHog.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Pricer_TruffleHog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/ConstraintHandler_VertexConflicts.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/ConstraintHandler_VertexConflicts.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/ConstraintHandler_EdgeConflicts.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/ConstraintHandler_EdgeConflicts.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_RectangleConflicts.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_RectangleConflicts.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_RectangleKnapsackConflicts.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_RectangleKnapsackConflicts.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_CorridorConflicts.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_CorridorConflicts.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_WaitDelayConflicts.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_WaitDelayConflicts.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_ExitEntryConflicts.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_ExitEntryConflicts.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_TwoEdgeConflicts.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_TwoEdgeConflicts.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_GoalConflicts.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Separator_GoalConflicts.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/BranchingRule.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/BranchingRule.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/BranchingRule_Fractional.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Constraint_VertexBranching.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Constraint_VertexBranching.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Constraint_LengthBranching.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Constraint_LengthBranching.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Output.h
        ${CMAKE_CURRENT_SOURCE_DIR}/library/bcp-mapf-mirror/bcp/Output.cpp
)



add_executable(
  mapf
  example/mapf_test.cpp
  ${MAPF_SOURCE_FILES} ${TRUFFLEHOG_SOURCE_FILES} ${SAT_SOURCE_FILES}
)

#target_compile_options(mapf PUBLIC "-pthread")

target_link_libraries(
  mapf
  mapf-adapters
  ${Boost_LIBRARIES}
  yaml-cpp
  fmt::fmt-header-only 
  libscip 
  ${LIBM}
  pthread
)
