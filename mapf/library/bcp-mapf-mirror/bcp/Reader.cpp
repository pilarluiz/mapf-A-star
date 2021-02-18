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

#include "Reader.h"
#include "Includes.h"
#include "ProblemData.h"
#include <regex>

#include "trufflehog/Instance.h"
#include "trufflehog/AStar.h"

// Read instance from file
SCIP_RETCODE read_instance(
    SCIP* scip,                   // SCIP
    int x, 
    int y, 
    std::vector<std::pair<int, int> > obstacles, 
    std::vector<std::pair<int, int> > starts,
    std::vector<std::pair<int, int> > goals)
{
    // Get instance name.

    // Load instance.
    auto instance = std::make_shared<Instance>(x, y, obstacles, starts, goals);

    // Create pricing solver.
    auto astar = std::make_shared<AStar>(instance->map);

    // Create the problem.
    SCIP_CALL(SCIPprobdataCreate(scip, "instance_name", instance, astar));

    // Done.
    return SCIP_OKAY;
}
