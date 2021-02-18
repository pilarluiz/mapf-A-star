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

#include "Output.h"
#include "Includes.h"
#include "ProblemData.h"
#include "VariableData.h"
#include <sys/stat.h>
#include <vector>

std::pair<float, std::vector<std::string> > write_best_solution(
    SCIP* scip    // SCIP
)
{
    // Check.
    debug_assert(scip);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);

    // Get variables.
    const auto& dummy_vars = SCIPprobdataGetDummyVars(probdata);
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    float objective_value = 0.0;
    std::vector<std::string> paths;    

    // Get best solution.
    auto sol = SCIPgetBestSol(scip);
    SCIP_Real obj = 0;
    if (!sol)
    {
        goto EXIT;
    }

    // Exit if no solution within objective limit is found.
    obj = SCIPgetSolOrigObj(scip, sol);
    if (obj >= ARTIFICIAL_VAR_COST)
    {
        goto EXIT;
    }

    // Print paths.
    println("");
    //print_used_paths(scip, sol);

    // Check if dummy variables are used.
    for (Agent a = 0; a < N; ++a)
    {
        // Get the variable.
        auto var = dummy_vars[a];
        debug_assert(var);
        debug_assert(!SCIPvarGetData(var));

        // Get the variable value.
        const auto var_val = SCIPgetSolVal(scip, sol, var);

        // Check
        if (SCIPisPositive(scip, var_val))
        {
            goto EXIT;
        }
    }

    // Write objective value.
    //fmt::print(f, "{:.0f}\n\n", SCIPround(scip, obj));
    objective_value = SCIPround(scip, obj);

    // Write paths.
    for (Agent a = 0; a < N; ++a)
    {
        bool found = false;
        for (auto var : agent_vars[a])
        {
            // Get the variable value.
            debug_assert(var);
            const auto var_val = SCIPgetSolVal(scip, sol, var);

            // Write the path.
            if (SCIPisPositive(scip, var_val))
            {
                // Get the path.
                auto vardata = SCIPvarGetData(var);
                const auto path_length = SCIPvardataGetPathLength(vardata);
                const auto path = SCIPvardataGetPath(vardata);

                // Write.
                //fmt::print(f, "{:.0f}: {}\n",
                //          SCIPround(scip, SCIPvarGetObj(var)),
                //           format_path(probdata, path_length, path));
                paths.push_back(format_path(probdata, path_length, path));

                // Move to next agent.
                release_assert(!found, "Agent {} is using more than one path");
                found = true;
                break;
            }
        }
        release_assert(found, "Agent {} has no path in the solution", a);
    }

    // Close file.
    EXIT:
    
    // Done.
    return make_pair(objective_value, paths);
}
