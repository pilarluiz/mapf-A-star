#ifndef MAPFADAPTERS_CBS_HPP
#define MAPFADAPTERS_CBS_HPP


#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/cbs.hpp>
#include "mapf.hpp"
#include "definitions.hpp"
#include <chrono>



using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using mapf_adapters::mapf;

namespace cbs{
	class solver{
	public:
		solver()	{}

		bool solve(mapf_adapters::mapf map, std::vector<std::pair<int, int> > starts){
			
			std::vector<std::pair<int, int> > obs = map.get_obstacles();
			std::vector<std::pair<int, int> > gl = map.get_goals();

			std::unordered_set<Location> obstacles;
			std::vector<Location> goals;
			std::vector<State> startStates;

			for(auto it = obs.begin(); it != obs.end(); ++it){
				obstacles.insert(Location(it->first, it->second));
			}
			
			for(auto it = gl.begin(); it != gl.end(); ++it){
				goals.emplace_back(Location(it->first, it->second));
			}

			for(auto it = starts.begin(); it != starts.end(); ++it){
				startStates.emplace_back(State(0, it->first, it->second));
			}

			Environment mapf(obstacles, goals, map.get_graph());
			CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);
			std::vector<PlanResult<State, Action, int> > solution;


			auto cbs_start = std::chrono::system_clock::now();
			bool success = cbs.search(startStates, solution);
			auto cbs_end = std::chrono::system_clock::now();

			if (success) {
				std::cout << "Planning successful! " << std::endl;
				int cost = 0;
				int makespan = 0;
				for (const auto& s : solution) {
					cost += s.cost;
					makespan = std::max<int>(makespan, s.cost);
				}

				std::ofstream out("../example/output_cbs.yaml");
				out << "statistics:" << std::endl;
				out << "  cost: " << cost << std::endl;
				out << "  makespan: " << makespan << std::endl;
				out << "  runtime: " << std::chrono::duration<double>(cbs_end - cbs_start).count() << std::endl;
				out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
				out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
				out << "schedule:" << std::endl;
				for (size_t a = 0; a < solution.size(); ++a) {
					out << "  agent" << a << ":" << std::endl;
					for (const auto& state : solution[a].states) {
						out << "    - x: " << state.first.x << std::endl
							<< "      y: " << state.first.y << std::endl
							<< "      t: " << state.second << std::endl;
					}
				}
				return true;
			} else {
				std::cout << "Planning NOT successful!" << std::endl;
				return false;
			}
		}
	};
}

#endif
