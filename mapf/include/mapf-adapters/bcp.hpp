#ifndef MAPFADAPTERS_BCP_HPP
#define MAPFADAPTERS_BCP_HPP

#include <iostream>
#include "mapf.hpp"

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <string.h> 

#include <bcp/Main.cpp>

using mapf_adapters::mapf;

namespace bcp{

	class solver{
		public:
		solver()	{}

		bool solve(mapf_adapters::mapf map, std::vector<std::pair<int, int> > starts){
			

			/*
			Easy option

			std::vector<std::string> arguments = {"--dir", "../example/input.scen"};

			std::vector<char*> argv;
			for (const auto& arg : arguments)
			    argv.push_back((char*)arg.data());
			argv.push_back(nullptr);
			const SCIP_RETCODE retcode = start_solver(argv.size() - 1, argv.data());
			return retcode;
			*/

			std::vector<std::pair<int, int> > obstacles = map.get_obstacles();
			std::vector<std::pair<int, int> > goals = map.get_goals();
			int x = map.get_x(), y = map.get_y();

			std::pair<float, std::vector<std::string> > solution;

			auto bcp_start = std::chrono::system_clock::now();
			bool success = start_solver(x, y, obstacles, starts, goals, &solution);
			auto bcp_end = std::chrono::system_clock::now();

			float cost = solution.first;

			
			if (success) {
				std::cout << "Planning successful! " << std::endl;

				std::ofstream out("../example/output_bcp.yaml");
				out << "statistics:" << std::endl;
				out << "  cost: " << cost << std::endl;
				out << "  runtime: " << std::chrono::duration<double>(bcp_end - bcp_start).count() << std::endl;
				out << "schedule:" << std::endl;

				int count = 0;

				for(auto it=solution.second.begin(); it!=solution.second.end();++it){
					out << "  agent" << count << ":" << std::endl;
					std::stringstream ss(*it);

					std::string s;
					std::vector<std::string> output;
					while (std::getline(ss, s, ',')) {
						if(s[0] == '('){
							output.push_back(s.substr(1));
						}
						else if(s[s.length()-1] == ')'){
							output.push_back(s.substr(0, s.length()-1));	
						}
					}
					for(int i=0; i<output.size(); i+=2){
						out << "    - x: " << output[i] << std::endl
							<< "      y: " << std::stoi(output[i+1]) << std::endl
							<< "      t: " << i/2 << std::endl;
					}
					count++;
				}
				return true;
			} else {
				std::cout << "Planning NOT successful!" << std::endl;
				return false;
			}
			
			return true;
		}

	};
}


#endif