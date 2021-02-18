cmake_minimum_required(VERSION 3.2)
project(bcp-mapf)

# Verbose.
#set(CMAKE_VERBOSE_MAKEFILE on)

# Set C version.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED 1)

# Set C++ version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

# Set release build.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

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

include_directories(SYSTEM library/scipoptsuite-6.0.2/scip/src)
include_directories(${CMAKE_BINARY_DIR})

# Link to math library.
find_library(LIBM m)
if (NOT LIBM)
    set(LIBM "")
endif ()

# Include fmt.
include_directories(SYSTEM library/fmt/include)

# Include cxxopts.
include_directories(SYSTEM library/cxxopts/include)
include_directories(SYSTEM library/robin-hood-hashing/src/include)


# Include Truffle Hog.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/library)
set(TRUFFLEHOG_SOURCE_FILES
        library/robin-hood-hashing/src/include/robin_hood.h
        library/trufflehog/Includes.h
        library/trufflehog/Debug.h
        library/trufflehog/Coordinates.h
        library/trufflehog/Map.h
        library/trufflehog/AgentsData.h
        library/trufflehog/Instance.h
        library/trufflehog/Instance.cpp
        library/trufflehog/PriorityQueue.h
        library/trufflehog/LabelPool.h
        library/trufflehog/LabelPool.cpp
        library/trufflehog/Heuristic.h
        library/trufflehog/Heuristic.cpp
        library/trufflehog/EdgePenalties.h
        library/trufflehog/Crossings.h
        library/trufflehog/AStar.h
        library/trufflehog/AStar.cpp
        library/trufflehog/ReservationTable.h
        library/trufflehog/ReservationTable.cpp
)

# Set source files.
set(MAPF_SOURCE_FILES
        bcp/Main.cpp
        bcp/Includes.h
        bcp/Debug.h
        bcp/Coordinates.h
        bcp/Reader.h
        bcp/Reader.cpp
        bcp/ProblemData.h
        bcp/ProblemData.cpp
        bcp/VariableData.h
        bcp/VariableData.cpp
        bcp/Pricer_TruffleHog.h
        bcp/Pricer_TruffleHog.cpp
        bcp/ConstraintHandler_VertexConflicts.h
        bcp/ConstraintHandler_VertexConflicts.cpp
        bcp/ConstraintHandler_EdgeConflicts.h
        bcp/ConstraintHandler_EdgeConflicts.cpp
        bcp/Separator.h
        bcp/Separator_RectangleConflicts.h
        bcp/Separator_RectangleConflicts.cpp
        bcp/Separator_RectangleKnapsackConflicts.h
        bcp/Separator_RectangleKnapsackConflicts.cpp
        bcp/Separator_CorridorConflicts.h
        bcp/Separator_CorridorConflicts.cpp
        bcp/Separator_WaitDelayConflicts.h
        bcp/Separator_WaitDelayConflicts.cpp
        bcp/Separator_ExitEntryConflicts.h
        bcp/Separator_ExitEntryConflicts.cpp
        bcp/Separator_TwoEdgeConflicts.h
        bcp/Separator_TwoEdgeConflicts.cpp
        bcp/Separator_GoalConflicts.h
        bcp/Separator_GoalConflicts.cpp
        bcp/BranchingRule.h
        bcp/BranchingRule.cpp
        bcp/BranchingRule_Fractional.cpp
        bcp/Constraint_VertexBranching.h
        bcp/Constraint_VertexBranching.cpp
        bcp/Constraint_LengthBranching.h
        bcp/Constraint_LengthBranching.cpp
        bcp/Output.h
        bcp/Output.cpp
)
include_directories(bcp/)


# Add executable.
#add_executable(bcp-mapf ${MAPF_SOURCE_FILES} ${TRUFFLEHOG_SOURCE_FILES})
#target_link_libraries(bcp-mapf fmt::fmt-header-only libscip ${LIBM})

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
