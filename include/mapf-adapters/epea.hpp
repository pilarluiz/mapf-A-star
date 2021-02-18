#ifndef MAPFADAPTERS_EPEA_HPP
#define MAPFADAPTERS_EPEA_HPP

#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include <epea/epea.hpp>
#include "mapf.hpp"

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <chrono>

using namespace EPEA;

namespace epea{
  class solver{
  public:
    solver()  {}

    bool solve(mapf_adapters::mapf map_obj, std::vector<std::pair<int, int> > starts){

      std::pair<int, std::vector< std::vector< std::pair<int, int> > > > solution;
      EPEA::EPEAStar<mapf_adapters::mapf> mapf_epea;

      auto epea_start = std::chrono::system_clock::now();
      bool success = mapf_epea.search(map_obj, starts, &solution);
      auto epea_end = std::chrono::system_clock::now();
      auto epea_time = std::chrono::duration<double>(epea_end - epea_start).count();

      if (success) {
        std::cout << "Planning successful! " << std::endl;

        std::ofstream out("../example/output_epea.yaml");
        out << "statistics:" << std::endl;
        out << "  cost: " << solution.first << std::endl;
        out << "  runtime: " << epea_time << std::endl;
        out << "schedule:" << std::endl;

        int count = 0;

        for(auto it=solution.second.begin(); it!=solution.second.end();++it){
          out << "  agent" << count << ":" << std::endl;
          std::vector<std::pair<int, int> > output;
          output = *it;
          for(int i=0; i<output.size(); i++){
            out << "    - x: " << output[i].first << std::endl
              << "      y: " << output[i].second << std::endl
              << "      t: " << i << std::endl;
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
