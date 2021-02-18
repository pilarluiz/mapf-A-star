#ifndef ASTAR_GPU
#define ASTAR_GPU
#include <fstream>

struct state;

enum version_value {
	SLIDING, PATHFINDING
};


typedef void(*expand_fun)(const char *x, char **result);
typedef int(*heur_fun)(const char *x, const char *t);
typedef int(*states_delta_fun)(const char *src, const char *dst);
void astar_gpu(const char *s_in, const char *t_in, version_value version, std::fstream &output);

struct state {
	const char *node;
	int f;
	int g;
	state *prev;
};

#endif //ASTAR_GPU
