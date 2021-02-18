#include <iostream>
#include <string.h>
#include <fstream>

#include "astar_gpu.h"
#include "sliding_puzzle.h"
#include "pathfinding.h"

struct config {
	version_value version;
	std::string input_file;
	std::string output_file;
};

config parse_args(int argc, const char *argv[]);

char** expand(const char *str) {
	int node = atoi(str);
	char **result = (char**)malloc(4 * sizeof(char*));
	char *arr = (char*)malloc(3 * 11 * sizeof(char*));
	int sibling = node + 1;
	int lchild = 2 * node;
	int rchild = 2 * node + 1;
	sprintf(arr, "%04d %04d %04d ", sibling, lchild, rchild);
	for (int i = 0; i < 3; i++) {
		result[i] = &(arr[5 * i]);
		arr[5 * i - 1] = '\0';
	}
	if (node == 1) {
		result[2] = NULL;
	}
	result[3] = NULL;
	return result;
}

int h(const char *x, const char *t) {
	int res = 0;
	if (atoi(x) > atoi(t)) return 20000;
	int dist = abs(atoi(x) - atoi(t));
	while (dist > 0) {
		dist /= 2;
		res++;
	}
	return res;
}

int map_cpu[10];

#define SLIDING_N  5
#define SLIDING_STATE_LEN (SLIDING_N * SLIDING_N;

int main(int argc, const char *argv[]) {
	config config;
	try {
		config = parse_args(argc, argv);
	} catch (std::string error) {
		std::cout << error << std::endl;
		return 1;
	}
	std::ifstream file(config.input_file);
	std::fstream file_out(config.output_file, std::fstream::out | std::fstream::trunc);
	if (config.version == SLIDING) {
		std::string s, t;
		std::getline(file, s);
		std::getline(file, t);
		astar_gpu(s.c_str(), t.c_str(), SLIDING, file_out);
	} else if (config.version == PATHFINDING) {
		std::string s, t;
		pathfinding_read_input(file, s, t);
		astar_gpu(s.c_str(), t.c_str(), PATHFINDING, file_out);
	}
	return 0;
}

std::string usage(std::string filename) {
	return "Usage: " + filename + " --version [sliding | pathfinding]" +
		" --input-data input.txt --output-data output.txt";
}

config parse_args(int argc, const char *argv[]) {
	config result = {};
	std::string filename = std::string(argv[0]);
	if (argc != 7) throw usage(filename);

	if (std::string(argv[1]) != "--version") throw usage(filename);
	std::string version = std::string(argv[2]);
	if (version == "sliding") result.version = SLIDING;
	else if (version == "pathfinding") result.version = PATHFINDING;
	else throw usage(filename);

	if (std::string(argv[3]) != "--input-data") throw usage(filename);
	result.input_file = std::string(argv[4]);

	if (std::string(argv[5]) != "--output-data") throw usage(filename);
	result.output_file = std::string(argv[6]);
	return result;
}

