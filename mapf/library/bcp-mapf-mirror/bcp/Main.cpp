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

#include "Includes.h"
#include "Reader.h"
#include "Output.h"
#include "scip/scip.h"
#include "scip/scipshell.h"
#include "scip/scipdefplugins.h"
#include "cxxopts.hpp"
#include <vector>

static
SCIP_RETCODE start_solver(
    int x, 
    int y, 
    std::vector<std::pair<int, int> > obstacles, 
    std::vector<std::pair<int, int> > starts,
    std::vector<std::pair<int, int> > goals,
    std::pair<float, std::vector<std::string> > *solution
)
{
    // Parse program options.
    SCIP_Real time_limit = 0;
   
    // Initialize SCIP.
    SCIP* scip = nullptr;
    SCIP_CALL(SCIPcreate(&scip));

    // Set up plugins.
    {
        // Include default SCIP plugins.
        SCIP_CALL(SCIPincludeDefaultPlugins(scip));

        // Disable parallel solve.
        SCIP_CALL(SCIPsetIntParam(scip, "parallel/maxnthreads", 1));
        SCIP_CALL(SCIPsetIntParam(scip, "lp/threads", 1));

        // Set parameters.
        SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrounds", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "propagating/rootredcost/freq", -1));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/maxaddrounds", -1));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/maxstallrounds", 5));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/maxstallroundsroot", 20));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/cutagelimit", -1));

        // Turn off all separation algorithms.
        SCIP_CALL(SCIPsetSeparating(scip, SCIP_PARAMSETTING_OFF, TRUE));

        // Turn on aggressive primal heuristics.
        SCIP_CALL(SCIPsetHeuristics(scip, SCIP_PARAMSETTING_AGGRESSIVE, TRUE));

        // Turn off some primal heuristics.
        {
            const auto nheurs = SCIPgetNHeurs(scip);
            auto heurs = SCIPgetHeurs(scip);
            for (Int idx = 0; idx < nheurs; ++idx)
            {
                auto heur = heurs[idx];
                const String name(SCIPheurGetName(heur));
                if (name == "alns" ||
                    name == "bound" ||
                    name == "coefdiving" ||
                    name == "crossover" ||
                    name == "dins" ||
                    name == "fixandinfer" ||
                    name == "gins" ||
                    name == "guideddiving" ||
                    name == "intdiving" ||
                    name == "localbranching" ||
                    name == "locks" ||
                    name == "mutation" ||
                    name == "oneopt" ||
                    name == "rens" ||
                    name == "repair" ||
                    name == "rins" ||
                    name == "trivial" ||
                    name == "zeroobj" ||
                    name == "zirounding" ||
                    name == "proximity" || // Buggy
                    name == "twoopt")      // Buggy
                {
                    SCIPheurSetFreq(heur, -1);
                }
            }
        }
    }

    // Read instance.
    SCIP_CALL(read_instance(scip, x, y, obstacles, starts, goals));

    // Set time limit.
    if (time_limit > 0)
    {
        SCIP_CALL(SCIPsetRealParam(scip, "limits/time", time_limit));
    }

    // Solve.
    SCIP_CALL(SCIPsolve(scip));

    // Output.
    {
        // Print.
        println("");
        //SCIP_CALL(SCIPprintStatistics(scip, NULL));

        // Write best solution to file.
        *solution = write_best_solution(scip);
    }

    // Free memory.
    SCIP_CALL(SCIPfree(&scip));

    // Check if memory is leaked.
    BMScheckEmptyMemory();

    // Done.
    return SCIP_OKAY;
}

/*
int main(int argc, char** argv)
{
    const SCIP_RETCODE retcode = start_solver(argc, argv);
    if (retcode != SCIP_OKAY)
    {
        SCIPprintError(retcode);
        return -1;
    }
    return 0;
}

*/