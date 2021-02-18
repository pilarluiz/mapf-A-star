#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include <mapf-adapters/mapf.hpp>

#include <mapf-adapters/cbs.hpp>
#include <mapf-adapters/ecbs.hpp>
#include <mapf-adapters/bcp.hpp>
#include <mapf-adapters/icts.hpp>
#include <mapf-adapters/sat.hpp>
#include <mapf-adapters/epea.hpp>

#include <chrono>

int main(int argc, char* argv[]) {

  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");

  std::string inputFile;
  float w;
  int j;
  int e;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")(
      "suboptimality,w", po::value<float>(&w)->default_value(1.0),
      "suboptimality bound")(
      "icts_include,j", po::value<int>(&j)->default_value(1),
      "Include ICTS 0/1")(
      "epea_include,e", po::value<int>(&e)->default_value(1),
      "Include EPEA 0/1");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  std::vector<std::pair<int, int> > obstacles;
  std::vector<std::pair<int, int> > goals;
  std::vector<std::pair<int, int> > starts;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.emplace_back(std::make_pair(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    starts.emplace_back(std::make_pair(start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(std::make_pair(goal[0].as<int>(), goal[1].as<int>()));
  }

  mapf_adapters::mapf mapf(dimx, dimy, obstacles, goals);
  
  cbs::solver mapf_cbs;

  auto cbs_start = std::chrono::system_clock::now();
  mapf_cbs.solve(mapf, starts);
  auto cbs_end = std::chrono::system_clock::now();
  auto cbs_time = std::chrono::duration<double>(cbs_end - cbs_start).count();
  

  bcp::solver mapf_bcp;

  auto bcp_start = std::chrono::system_clock::now();
  mapf_bcp.solve(mapf, starts);
  auto bcp_end = std::chrono::system_clock::now();
  auto bcp_time = std::chrono::duration<double>(bcp_end - bcp_start).count();

  ecbs::solver mapf_ecbs;

  auto ecbs_start = std::chrono::system_clock::now();
  mapf_ecbs.solve(mapf, starts, w);
  auto ecbs_end = std::chrono::system_clock::now();
  auto ecbs_time = std::chrono::duration<double>(ecbs_end - ecbs_start).count();

  SAT_solver::solver mapf_sat;

  auto sat_start = std::chrono::system_clock::now();
  mapf_sat.solve(mapf, starts);
  auto sat_end = std::chrono::system_clock::now();
  auto sat_time = std::chrono::duration<double>(sat_end - sat_start).count();


  float icts_time;
  if(j){
    icts::solver mapf_icts;
    
    auto icts_start = std::chrono::system_clock::now();
    mapf_icts.solve(mapf, starts);
    auto icts_end = std::chrono::system_clock::now();
    icts_time = std::chrono::duration<double>(icts_end - icts_start).count(); 
  }else{
    icts_time = 0;
  }

  float epea_time;
  if(e){
    epea::solver mapf_epea;
    
    auto epea_start = std::chrono::system_clock::now();
    mapf_epea.solve(mapf, starts);
    auto epea_end = std::chrono::system_clock::now();
    epea_time = std::chrono::duration<double>(epea_end - epea_start).count(); 
  }else{
    epea_time = 0;
  }

  std::cout<<std::endl<<std::endl;

  std::cout<<"TIME TAKEN TO COMPLETE THE TASK ::"<<std::endl
            <<"CBS :: "<<cbs_time<<std::endl
            <<"BCP :: "<<bcp_time<<std::endl
            <<"ECBS :: "<<ecbs_time<<std::endl
	    <<"SAT :: "<<sat_time<<std::endl
            <<"ICTS :: "<<icts_time<<std::endl
	    <<"EPEA :: "<<epea_time<<std::endl<<std::endl<<std::endl;

  return 0;
}
