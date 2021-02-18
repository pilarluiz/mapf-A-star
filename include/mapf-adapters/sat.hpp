#ifndef MAPFADAPTERS_SAT_HPP
#define MAPFADAPTERS_SAT_HPP

#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include "mapf.hpp"

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <chrono>

#ifndef __INSOLVER_MAIN_TEMP__
#define __INSOLVER_MAIN_TEMP__

#include <stdio.h>
#include <stdlib.h>
#include <sys/times.h>
#include <unistd.h>

#include "config.h"
#include "compile.h"
#include "complete.h"
#include "defs.h"
#include "reloc.h"
#include "multirobot.h"
#include "compress.h"
#include "statistics.h"
#include "version.h"
#include "search.h"

#include "insolver_main.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    sResult solve_MultirobotInstance_SAT(const std::vector<std::pair<int, int> > &obstacles, const std::vector<std::pair<int, int> >& goals, const std::vector<std::pair<int, int> > &starts, const int &x, const int &y, const std::vector<std::vector<int> > &graph, std::pair<float, std::vector<std::vector<std::tuple<int, int, int> > > > *solution)
    {
	sResult result;
	sUndirectedGraph environment(false);	

	sRobotArrangement initial_arrangement;
	sRobotArrangement goal_arrangement;
	sRobotGoal robot_goal;

	environment.from_vector(obstacles, goals, starts, x, y, graph);

	initial_arrangement.from_vector_initial(starts, x, y);
	robot_goal.from_vector_goal(goals, x, y);

	goal_arrangement.from_vector_goal(goals, x, y);
	

	sMultirobotInstance instance_tmp;

	sMultirobotInstance instance(environment, initial_arrangement, robot_goal);
	sMultirobotInstance instance_whca(environment, initial_arrangement, goal_arrangement);
	int optimal_makespan, optimal_cost, optimal_fuel;
	sMultirobotSolution optimal_solution;
	   	
	sMultirobotInstance::MDD_vector MDD;

	sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH, -1, 600, 65536, 4, sMultirobotSolutionCompressor::ENCODING_MDD);
	compressor.set_Ratio(-1);
	compressor.set_Robustness(1);		

	Glucose::Solver *solver;
	result = compressor.incompute_CostOptimalSolution(&solver, initial_arrangement, robot_goal, environment, instance.m_sparse_environment, MDD, 65536, optimal_cost, optimal_solution);

	printf("Computed sum of costs:%d\n", optimal_cost);
	std::map<int, std::vector<std::pair<int, int> > > sol = optimal_solution.get_sol();
	solution->first = optimal_cost;
	solution->second.resize(starts.size());
	
	for(auto it=sol.begin(); it!=sol.end(); it++){
		for(auto it2=sol[it->first].begin(); it2!=sol[it->first].end(); it2++){
			solution->second[it->first-1].push_back(make_tuple(it2->first/y, it2->first%y, it2->second));
		}
	}
		
	return sRESULT_SUCCESS;
    }

/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif
using namespace sReloc;
namespace SAT_solver{
  class solver{
  public:
    solver()  {}

    bool solve(mapf_adapters::mapf map_obj, std::vector<std::pair<int, int> > starts){
	
	std::pair<float, std::vector<std::vector<std::tuple<int, int, int> > > > solution;
	auto sat_start = std::chrono::system_clock::now();
	::sReloc::solve_MultirobotInstance_SAT(map_obj.get_obstacles(),map_obj.get_goals(),starts,map_obj.get_x(),map_obj.get_y(),map_obj.get_graph().get_adj(), &solution);
	auto sat_end = std::chrono::system_clock::now();
	
	std::ofstream out("../example/output_sat.yaml");
	out << "statistics:" << std::endl;
	out << "  cost: " << solution.first << std::endl;
	out << "  runtime: " << std::chrono::duration<double>(sat_end - sat_start).count() << std::endl;
	out << "schedule:" << std::endl;

	for(int i=0; i<solution.second.size(); i++){
		out << "  agent" << i << ":" << std::endl;
		for(int j=0; j<solution.second[i].size(); j++){
			out << "    - x: " << std::get<0>(solution.second[i][j]) << std::endl;
			out << "      y: " << std::get<1>(solution.second[i][j]) << std::endl;
			out << "      t: " << std::get<2>(solution.second[i][j]) << std::endl;
		}
	}
      return true;
    }
  };
}

#endif
