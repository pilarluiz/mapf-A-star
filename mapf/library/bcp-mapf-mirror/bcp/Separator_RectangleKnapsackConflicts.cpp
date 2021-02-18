/*
This file is part of BCP-MAPF.

BCP-MAPF is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BCP-MAPF is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with BCP-MAPF.  If not, see <https://www.gnu.org/licenses/>.

Author: Edward Lam <ed@ed-lam.com>
*/

#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS

//#define PRINT_DEBUG

#include "Separator_RectangleKnapsackConflicts.h"
#include "Separator_RectangleConflicts.h"
#include "Coordinates.h"
#include "ProblemData.h"
#include "VariableData.h"
#include <algorithm>

#define SEPA_NAME                 "rectangle_knapsack_conflicts"
#define SEPA_DESC   "Separator for rectangle knapsack conflicts"
#define SEPA_PRIORITY                                    +500000 // priority of the constraint handler for separation
#define SEPA_FREQ                                              1 // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                    1.0
#define SEPA_USESSUBSCIP                                   FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                         FALSE // should separation method be delayed, if other separators found cuts? */

// Data for rectangle knapsack cuts
struct RectangleKnapsackSepaData
{
    Vector<RectangleKnapsackCut> cuts;
};

SCIP_RETCODE rectangle_knapsack_conflicts_create_cut(
    SCIP* scip,                               // SCIP
    SCIP_ProbData* probdata,                  // Problem data
    SCIP_SEPA* sepa,                          // Separator
    RectangleKnapsackSepaData& sepadata,      // Separator data
#if defined(DEBUG) or defined(PRINT_DEBUG)
    const Time start_t,                       // Time before entry
    const Position start_x1,                  // Start coordinate of agent 1
    const Position start_y1,                  // Start coordinate of agent 1
    const Position start_x2,                  // Start coordinate of agent 2
    const Position start_y2,                  // Start coordinate of agent 2
    const Time end_t,                         // Time after exit
    const Position end_x1,                    // End coordinate of agent 1
    const Position end_y1,                    // End coordinate of agent 1
    const Position end_x2,                    // End coordinate of agent 2
    const Position end_y2,                    // End coordinate of agent 2
#endif
    const RectangleConflict& conflict,        // Rectangle conflict
    SCIP_Result* result                       // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    auto name = fmt::format("rectangle_knapsack_conflict("
                            "{},{},"
                            "(({},{}),({},{}),{}),(({},{}),({},{}),{}))",
                            conflict.a1, conflict.a2,
                            start_x1, start_y1, start_x2, start_y2, start_t,
                            end_x1, end_y1, end_x2, end_y2, end_t);
#endif

    // Create data for the cut.
    const auto [a1_begin, a1_end] = conflict.agent1_edges();
    const auto [a2_begin, a2_end] = conflict.agent2_edges();
    TwoAgentRobustCut cut(scip, conflict.a1, conflict.a2, a1_end - a1_begin, a2_end - a2_begin
#ifdef DEBUG
        , std::move(name)
#endif
    );
    std::copy(a1_begin, a2_end, cut.edge_times_a1().first);

    // Store the cut.
    Int idx;
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip,
                                               probdata,
                                               sepa,
                                               std::move(cut),
                                               3,
                                               result,
                                               &idx));
    sepadata.cuts.push_back({idx, conflict.out1_begin_, conflict.in2_begin_, conflict.out2_begin_});

    // Done.
    return SCIP_OKAY;
}

static
RectangleConflict find_rectangle(
    SCIP* scip,                                                   // SCIP
    const Map& map,                                               // Map
    const Vector<HashTable<EdgeTime, SCIP_Real>>& agent_edges,    // Edge weights for each agent
    const NodeTime nt,                                            // Node-time of the conflict
    const Agent a1,                                               // Agent 1
    const Agent a2,                                               // Agent 2
    SCIP_VAR* var1,                                               // Path of agent 1
    SCIP_VAR* var2                                                // Path of agent 2
#if defined(DEBUG) or defined(PRINT_DEBUG)
  , SCIP_ProbData* probdata,                                      // Problem data
    Time& output_start_t,                                         // Time before entry
    Time& output_end_t,                                           // Time after exit
    Position& output_start_x1,                                    // Start coordinate of agent 1
    Position& output_start_y1,                                    // Start coordinate of agent 1
    Position& output_start_x2,                                    // Start coordinate of agent 2
    Position& output_start_y2,                                    // Start coordinate of agent 2
    Position& output_end_x1,                                      // End coordinate of agent 1
    Position& output_end_y1,                                      // End coordinate of agent 1
    Position& output_end_x2,                                      // End coordinate of agent 2
    Position& output_end_y2,                                      // End coordinate of agent 2
    SCIP_Real& output_lhs                                         // LHS
#endif
)
{
    // Create output.
    RectangleConflict conflict;
    conflict.a1 = a1;
    conflict.a2 = a2;

    // Get the path of agent 1.
    auto vardata1 = SCIPvarGetData(var1);
    const auto path_length1 = SCIPvardataGetPathLength(vardata1);
    const auto path1 = SCIPvardataGetPath(vardata1);

    // Get the path of agent 2.
    auto vardata2 = SCIPvarGetData(var2);
    const auto path_length2 = SCIPvardataGetPathLength(vardata2);
    const auto path2 = SCIPvardataGetPath(vardata2);

    // Print.
#ifdef PRINT_DEBUG
    {
        auto probdata = SCIPgetProbData(scip);
        debugln("---------");
        fmt::print("                  ");
        for (Time t = 0; t < std::max(path_length1, path_length2); ++t)
            fmt::print("{:10d}", t);
        debugln("");
        debugln("   agent {:2d}, path {}",
                a1, format_path_spaced(probdata, path_length1, path1));
        debugln("   agent {:2d}, path {}",
                a2, format_path_spaced(probdata, path_length2, path2));
    }
#endif

    // Get the length of the shorter path.
    const auto min_path_length = std::min(path_length1, path_length2);

    // Get time of the vertex conflict.
    const auto conflict_time = nt.t;

    // A rectangle conflict can only occur if the two paths visit the same vertex.
    if (!(0 < conflict_time && conflict_time < min_path_length - 1 &&
          path1[conflict_time].n == nt.n &&
          path2[conflict_time].n == nt.n))
    {
        return conflict;
    }

    // Get the movement directions.
    Direction y_dir = Direction::INVALID;
    Direction x_dir = Direction::INVALID;
    for (Time t = conflict_time; t < min_path_length; ++t)
    {
        if (path1[t].d == Direction::NORTH || path1[t].d == Direction::SOUTH)
        {
            y_dir = path1[t].d;
            break;
        }
        else if (path2[t].d == Direction::NORTH || path2[t].d == Direction::SOUTH)
        {
            y_dir = path2[t].d;
            break;
        }
        else if (path1[t].d == Direction::WAIT || path2[t].d == Direction::WAIT)
        {
            return conflict;
        }
    }
    for (Time t = conflict_time; t < min_path_length; ++t)
    {
        if (path1[t].d == Direction::EAST || path1[t].d == Direction::WEST)
        {
            x_dir = path1[t].d;
            break;
        }
        else if (path2[t].d == Direction::EAST || path2[t].d == Direction::WEST)
        {
            x_dir = path2[t].d;
            break;
        }
        else if (path1[t].d == Direction::WAIT || path2[t].d == Direction::WAIT)
        {
            return conflict;
        }
    }
    if (y_dir == Direction::INVALID || x_dir == Direction::INVALID)
    {
        return conflict;
    }

    // Find the first time when the direction changes.
    Time start_t = conflict_time;
    Time end_t = conflict_time;
    for (; start_t >= 0; --start_t)
        if ((path1[start_t].d != y_dir && path1[start_t].d != x_dir) ||
            (path2[start_t].d != y_dir && path2[start_t].d != x_dir))
        {
            break;
        }
    start_t++;
    for (; end_t < min_path_length - 1; ++end_t)
        if ((path1[end_t].d != y_dir && path1[end_t].d != x_dir) ||
            (path2[end_t].d != y_dir && path2[end_t].d != x_dir))
        {
            break;
        }
    if (end_t <= start_t + 2)
    {
        return conflict;
    }
    release_assert(0 <= start_t && end_t > start_t + 2 && end_t < min_path_length);

    // Cannot find a rectangle conflict if the start location of the two agents are the
    // same.
    if (path1[start_t].n == path2[start_t].n)
    {
        return conflict;
    }

    // Check.
#ifdef DEBUG
    for (Time t = start_t; t < end_t; ++t)
    {
        debug_assert(path1[t].d != Direction::WAIT);
        debug_assert(path2[t].d != Direction::WAIT);
    }
#endif

    // Get the coordinates of those times.
    const auto [start_x1, start_y1] = map.get_xy(path1[start_t].n);
    const auto [start_x2, start_y2] = map.get_xy(path2[start_t].n);
    const auto [end_x1, end_y1] = map.get_xy(path1[end_t].n);
    const auto [end_x2, end_y2] = map.get_xy(path2[end_t].n);

    // Compute the rectangle.
    compute_rectangle(map,
                      start_t, end_t,
                      start_x1, start_y1, start_x2, start_y2,
                      end_x1, end_y1, end_x2, end_y2,
                      conflict);

    // Determine if the cut is violated.
    SCIP_Real lhs = 0.0;
    if (!conflict.empty())
    {
        for (auto [it, end] = conflict.agent1_edges(); it != end; ++it)
        {
            const auto et = *it;
            auto it2 = agent_edges[a1].find(et);
            if (it2 != agent_edges[a1].end())
                lhs += it2->second;
        }
        for (auto [it, end] = conflict.agent2_edges(); it != end; ++it)
        {
            const auto et = *it;
            auto it2 = agent_edges[a2].find(et);
            if (it2 != agent_edges[a2].end())
                lhs += it2->second;
        }
        debug_assert(SCIPisSumLE(scip, lhs, 4.0));
        if (SCIPisSumLE(scip, lhs, 3.0))
        {
            conflict.edges.clear();
        }
    }

    // Store coordinates.
#if defined(DEBUG) or defined(PRINT_DEBUG)
    output_start_t = start_t;
    output_end_t = end_t;
    output_start_x1 = start_x1;
    output_start_y1 = start_y1;
    output_start_x2 = start_x2;
    output_start_y2 = start_y2;
    output_end_x1 = end_x1;
    output_end_y1 = end_y1;
    output_end_x2 = end_x2;
    output_end_y2 = end_y2;
    output_lhs = lhs;
#endif

    // Return.
    return conflict;
}

// Separator
static
SCIP_RETCODE rectangle_knapsack_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for rectangle knapsack conflicts on solution with obj "
            "{:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get separator data.
    auto sepadata = reinterpret_cast<RectangleKnapsackSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);

    // Get variables.
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Force cuts for debugging.
//    {
//        Vector<Agent> a1s{};
//        Vector<Vector<EdgeTime>> a1_ins{};
//        Vector<Vector<EdgeTime>> a1_outs{};
//
//        Vector<Agent> a2s{};
//        Vector<Vector<EdgeTime>> a2_ins{};
//        Vector<Vector<EdgeTime>> a2_outs{};
//
//        for (size_t idx = 0; idx < a1s.size(); ++idx)
//        {
//            const auto a1 = a1s[idx];
//            const auto a2 = a2s[idx];
//
//            RectangleConflict conflict;
//            conflict.a1 = a1;
//            conflict.a2 = a2;
//
//            conflict.edges = a1_ins[idx];
//            conflict.out1_begin_ = conflict.edges.size();
//            std::copy(a1_outs[idx].begin(), a1_outs[idx].end(), std::back_inserter(conflict.edges));
//            conflict.in2_begin_ = conflict.edges.size();
//            std::copy(a2_ins[idx].begin(), a2_ins[idx].end(), std::back_inserter(conflict.edges));
//            conflict.out2_begin_ = conflict.edges.size();
//            std::copy(a2_outs[idx].begin(), a2_outs[idx].end(), std::back_inserter(conflict.edges));
//
//            SCIP_CALL(rectangle_knapsack_conflicts_create_cut(scip,
//                                                              sepa,
//                                                              *sepadata,
//                                                              agent_vars[a1],
//                                                              agent_vars[a2],
//                                                              conflict,
//                                                              result));
//        }
//
//    }

    // Get conflicting paths and vertices of each agent.
    Vector<Vector<SCIP_VAR*>> agent_paths(N);
    Vector<Vector<NodeTime>> agent_vertices(N);
    Vector<HashTable<EdgeTime, SCIP_Real>> agent_edges(N);
    for (Agent a = 0; a < N; ++a)
    {
        // Get agent-specific data.
        auto& agent_paths_a = agent_paths[a];
        auto& agent_vertices_a = agent_vertices[a];
        auto& agent_edges_a = agent_edges[a];

        // Calculate the number of times a vertex is used by summing the columns.
        for (auto var : agent_vars[a])
        {
            // Get the path.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Get the variable value.
            const auto var_val = SCIPgetSolVal(scip, nullptr, var);

            // Append the path.
            if (SCIPisPositive(scip, var_val))
            {
                agent_paths_a.push_back(var);

                {
                    const EdgeTime et{path[0], 0};
                    agent_edges_a[et] += var_val;
                }

                for (Time t = 1; t < path_length; ++t)
                {
                    {
                        const NodeTime nt{path[t].n, t};
                        if (std::find(agent_vertices_a.begin(),
                                      agent_vertices_a.end(),
                                      nt) == agent_vertices_a.end())
                        {
                            agent_vertices_a.push_back(nt);
                        }
                    }

                    {
                        const EdgeTime et{path[t], t};
                        agent_edges_a[et] += var_val;
                    }
                }
            }
        }

        // Sort.
        std::sort(agent_vertices_a.begin(),
                  agent_vertices_a.end(),
                  [](const NodeTime a, const NodeTime b)
                  {
                      return (a.t < b.t) || (a.t == b.t && a.n < b.n);
                  });

        // Print.
//#ifdef PRINT_DEBUG
//        if (!agent_vertices_a.empty())
//        {
//            debugln("   Used vertices for agent {}:", a);
//            for (const auto nt : agent_vertices_a)
//            {
//                const auto [x, y] = map.get_xy(nt.n);
//                debugln("      agent {:2d}, x {:2d}, y {:2d}, t {:3d}",
//                        a, x, y, nt.t);
//            }
//        }
//#endif
    }

    // Find conflicts.
    Vector<NodeTime> common_vertices;
    for (Agent a1 = 0; a1 < N - 1; ++a1)
        for (Agent a2 = a1 + 1; a2 < N; ++a2)
        {
            // Find common vertices.
            common_vertices.clear();
            std::set_intersection(agent_vertices[a1].begin(), agent_vertices[a1].end(),
                                  agent_vertices[a2].begin(), agent_vertices[a2].end(),
                                  std::back_inserter(common_vertices),
                                  [](const NodeTime a, const NodeTime b)
                                  {
                                      return (a.t < b.t) || (a.t == b.t && a.n < b.n);
                                  });

            // Find a conflict.
            for (const auto nt : common_vertices)
            {
                // Print.
                debugln("Checking conflict at ({},{}) time {}",
                        map.get_x(nt.n), map.get_y(nt.n), nt.t);

                // Check every path.
                for (auto p1 : agent_paths[a1])
                    for (auto p2 : agent_paths[a2])
                    {
#if defined(DEBUG) or defined(PRINT_DEBUG)
                        Time start_t;
                        Time end_t;
                        Position start_x1;
                        Position start_y1;
                        Position start_x2;
                        Position start_y2;
                        Position end_x1;
                        Position end_y1;
                        Position end_x2;
                        Position end_y2;
                        SCIP_Real lhs;
#endif
                        auto conflict = find_rectangle(scip,
                                                       map,
                                                       agent_edges,
                                                       nt,
                                                       a1,
                                                       a2,
                                                       p1,
                                                       p2
#if defined(DEBUG) or defined(PRINT_DEBUG)
                                                     , probdata,
                                                       start_t,
                                                       end_t,
                                                       start_x1,
                                                       start_y1,
                                                       start_x2,
                                                       start_y2,
                                                       end_x1,
                                                       end_y1,
                                                       end_x2,
                                                       end_y2,
                                                       lhs
#endif
                                                       );

                        // Stop if no rectangle was found.
                        if (conflict.empty())
                            continue;

                        // Print.
#ifdef PRINT_DEBUG
                        String a1_in_str("{");
                        for (auto it = conflict.in1_begin(); it != conflict.in1_end(); ++it)
                        {
                            const auto& [e, t] = *it;
                            a1_in_str += fmt::format("EdgeTime({},{}),",
                                                     NodeTime(e.n, t).nt,
                                                     e.d == Direction::NORTH ? "Direction::NORTH" :
                                                     e.d == Direction::SOUTH ? "Direction::SOUTH" :
                                                     e.d == Direction::EAST ? "Direction::EAST" :
                                                     e.d == Direction::WEST ? "Direction::WEST" :
                                                     e.d == Direction::WAIT ? "Direction::WAIT" : "???");
                        }
                        a1_in_str.pop_back();
                        a1_in_str += "}";
                        String a1_out_str("{");
                        for (auto it = conflict.out1_begin(); it != conflict.out1_end(); ++it)
                        {
                            const auto& [e, t] = *it;
                            a1_out_str += fmt::format("EdgeTime({},{}),",
                                                     NodeTime(e.n, t).nt,
                                                     e.d == Direction::NORTH ? "Direction::NORTH" :
                                                     e.d == Direction::SOUTH ? "Direction::SOUTH" :
                                                     e.d == Direction::EAST ? "Direction::EAST" :
                                                     e.d == Direction::WEST ? "Direction::WEST" :
                                                     e.d == Direction::WAIT ? "Direction::WAIT" : "???");
                        }
                        a1_out_str.pop_back();
                        a1_out_str += "}";
                        String a2_in_str("{");
                        for (auto it = conflict.in2_begin(); it != conflict.in2_end(); ++it)
                        {
                            const auto& [e, t] = *it;
                            a2_in_str += fmt::format("EdgeTime({},{}),",
                                                     NodeTime(e.n, t).nt,
                                                     e.d == Direction::NORTH ? "Direction::NORTH" :
                                                     e.d == Direction::SOUTH ? "Direction::SOUTH" :
                                                     e.d == Direction::EAST ? "Direction::EAST" :
                                                     e.d == Direction::WEST ? "Direction::WEST" :
                                                     e.d == Direction::WAIT ? "Direction::WAIT" : "???");
                        }
                        a2_in_str.pop_back();
                        a2_in_str += "}";
                        String a2_out_str("{");
                        for (auto it = conflict.out2_begin(); it != conflict.out2_end(); ++it)
                        {
                            const auto& [e, t] = *it;
                            a2_out_str += fmt::format("EdgeTime({},{}),",
                                                      NodeTime(e.n, t).nt,
                                                      e.d == Direction::NORTH ? "Direction::NORTH" :
                                                      e.d == Direction::SOUTH ? "Direction::SOUTH" :
                                                      e.d == Direction::EAST ? "Direction::EAST" :
                                                      e.d == Direction::WEST ? "Direction::WEST" :
                                                      e.d == Direction::WAIT ? "Direction::WAIT" : "???");
                        }
                        a2_out_str.pop_back();
                        a2_out_str += "}";
                        debugln("   Creating rectangle knapsack cut for agent {} in "
                                "{} out {} and agent {} in {} out {} with value {} in "
                                "branch-and-bound node {}",
                                conflict.a1, a1_in_str, a1_out_str,
                                conflict.a2, a2_in_str, a2_out_str,
                                lhs,
                                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
#endif

                        // Create cut.
                        SCIP_CALL(rectangle_knapsack_conflicts_create_cut(scip,
                                                                          probdata,
                                                                          sepa,
                                                                          *sepadata,
#if defined(DEBUG) or defined(PRINT_DEBUG)
                                                                          start_t,
                                                                          start_x1,
                                                                          start_y1,
                                                                          start_x2,
                                                                          start_y2,
                                                                          end_t,
                                                                          end_x1,
                                                                          end_y1,
                                                                          end_x2,
                                                                          end_y2,
#endif
                                                                          conflict,
                                                                          result));
                        goto NEXT_AGENT;
                    }
            }
        NEXT_AGENT:;
        }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
static
SCIP_DECL_SEPACOPY(sepaCopyRectangleKnapsackConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_SEPA* sepa_copy;
    SCIP_CALL(SCIPincludeSepaRectangleKnapsackConflicts(scip, &sepa_copy));

    // Done.
    return SCIP_OKAY;
}

// Free separator data
static
SCIP_DECL_SEPAFREE(sepaFreeRectangleKnapsackConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Get separator data.
    auto sepadata = reinterpret_cast<RectangleKnapsackSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Free memory.
    sepadata->~RectangleKnapsackSepaData();
    SCIPfreeBlockMemory(scip, &sepadata);

    // Done.
    return SCIP_OKAY;
}

// Separation method for LP solutions
static
SCIP_DECL_SEPAEXECLP(sepaExeclpRectangleKnapsackConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(rectangle_knapsack_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}

// Creates separator for rectangle knapsack conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaRectangleKnapsackConflicts(
    SCIP* scip,         // SCIP
    SCIP_SEPA** sepa    // Output pointer to separator
)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);

    // Create separator data.
    RectangleKnapsackSepaData* sepadata = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &sepadata));
    debug_assert(sepadata);
    new(sepadata) RectangleKnapsackSepaData;
    sepadata->cuts.reserve(500);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaBasic(scip,
                                   sepa,
                                   SEPA_NAME,
                                   SEPA_DESC,
                                   SEPA_PRIORITY,
                                   SEPA_FREQ,
                                   SEPA_MAXBOUNDDIST,
                                   SEPA_USESSUBSCIP,
                                   SEPA_DELAY,
                                   sepaExeclpRectangleKnapsackConflicts,
                                   nullptr,
                                   reinterpret_cast<SCIP_SEPADATA*>(sepadata)));
    debug_assert(*sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, *sepa, sepaCopyRectangleKnapsackConflicts));
    SCIP_CALL(SCIPsetSepaFree(scip, *sepa, sepaFreeRectangleKnapsackConflicts));

    // Done.
    return SCIP_OKAY;
}

// Get additional data about rectangle knapsack cuts
const Vector<RectangleKnapsackCut>& rectangle_knapsack_get_cuts(
    SCIP_ProbData* probdata    // Problem data
)
{
    auto sepa = SCIPprobdataGetRectangleKnapsackConflictsSepa(probdata);
    debug_assert(sepa);
    auto sepadata = reinterpret_cast<RectangleKnapsackSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);
    return sepadata->cuts;
}

#endif
