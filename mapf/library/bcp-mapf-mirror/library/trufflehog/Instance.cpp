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

#include <fstream>
#include <iostream>
#include "Instance.h"

namespace TruffleHog
{

struct AgentMapData
{
    String map_path;
    Position map_width;
    Position map_height;
};

void read_map(int x, int y, std::vector<std::pair<int, int> > obstacles, Map& map)
{

    // Read map size.
    Position width = x+2;
    Position height = y+2;

    // Create map.
    map.resize(width, height);

    for(int i=1; i<height-1;i++){
        for(int j=1; j<width-1;j++){
            Node n = map.get_id(i, j);
            map.set_passable(n);       
        }
    }
    for(auto it = obstacles.begin(); it!=obstacles.end(); ++it){
        int i = /*x - */it->second + 1;
        int j = it->first + 1;

        Node n = map.get_id(i, j);

        map.set_obstacle(n);    
    }

}

Instance::Instance(
    int x, 
    int y, 
    std::vector<std::pair<int, int> > obstacles, 
    std::vector<std::pair<int, int> > starts,
    std::vector<std::pair<int, int> > goals)
{
    
    // Read agents.
    Vector<AgentMapData> agents_map_data;
    {
        // Open scenario file.
        
        // Read agents data.
        {
            Position start_x;
            Position start_y;
            Position goal_x;
            Position goal_y;
            Float tmp;
            AgentMapData agent_map_data;

            agent_map_data.map_width = x+2;
            agent_map_data.map_height = y+2;

            for(int i = 0; i < goals.size();i++){
                start_x = /*x - */starts[i].second + 1;
                start_y = starts[i].first + 1;
                goal_x = /*x - */goals[i].second + 1;
                goal_y = goals[i].first + 1;
                
                if(map.empty()){
                    read_map(x, y, obstacles, map);
                }
                agents.add_agent(start_x, start_y, goal_x, goal_y, map);
                agents_map_data.push_back(agent_map_data);
            }
        }
        
        if (agents.empty())
        {
            err("No agents in scenario");
        }
    }

    // Check.
    
    for (Agent a = 0; a < agents.size(); ++a)
    {
        const auto [start_id, goal_id, start_x, start_y, goal_x, goal_y] = agents[a];
        const auto [map_path, map_width2, map_height2] = agents_map_data[a];

        release_assert(map_path == agents_map_data[0].map_path,
                       "Agent {} uses a different map");
        release_assert(map_width2 == map.width(),
                       "Map width of agent {} does not match actual map size", a);
        release_assert(map_height2 == map.height(),
                       "Map height of agent {} does not match actual map size", a);

        const auto start_tile = start_x * map.width() + start_y;
        release_assert(start_tile < map.size() && map[start_tile],
                       "Agent {} starts at an obstacle", a);

        const auto goal_tile = goal_x * map.width() + goal_y;
        release_assert(goal_tile < map.size() && map[goal_tile],
                       "Agent {} ends at an obstacle", a);
    }
}

}

