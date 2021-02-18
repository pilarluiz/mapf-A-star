#include <stdio.h>
#include "astar_gpu.h"
#include "heap.h"
#include "list.h"
#include "sliding_puzzle.h"
#include "pathfinding.h"
#include "cuda_utils.h"
#include "assert.h"
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <fstream>
#include <chrono>

#define STATES (32 * 1024ll * 1024)
#define HASH_SIZE  (1024 * 1024)
#define HASH_FUNS 128

__global__ void init_heap(const char *s, heap **Q, state *states_pool, char *nodes_pool, int state_len);
__global__ void clear_list(list *S);
__global__ void fill_list(const char *t, int k, int state_len,
		heap **Q, list *S, state *states_pool, char *nodes_pool,
		char ***expand_buf, expand_fun expand, heur_fun h, states_delta_fun states_delta);
__global__ void deduplicate(state **H, list *T, const char *t, heur_fun h);
__global__ void push_to_queues(const char *t, int k, heap **Q, list *S, heur_fun h, int off);

__device__ int f(const state *x, const char *t, heur_fun h);
__device__ int calculate_id();
__device__ state *state_create(const char *node, int f, int g, state *prev,
		state *states_pool, char *nodes_pool, int state_len);

char ***expand_bufs_create(int bufs, int elements, int element_size);
char **expand_buf_create(int elements, int element_size);

void states_pool_create(state **states, char **nodes, int node_size);
void states_pool_destroy(state *states_pool, char *nodes_pool);

#define THREADS_PER_BLOCK  1024
#define BLOCKS 16
#define RESULT_LEN (1024 * 1024)

__device__ int total_Q_size = 0;
__device__ int found = 0;
__device__ int out_of_memory = 0;
__device__ char result_path[RESULT_LEN];

void astar_gpu(const char *s_in, const char *t_in, version_value version, std::fstream &output) {
	char *s_gpu, *t_gpu;
	int k = THREADS_PER_BLOCK * BLOCKS;
	expand_fun expand_fun_cpu;
	heur_fun h_cpu;
	states_delta_fun states_delta_cpu;
	int expand_elements;
	int expand_element_size;

	auto start = std::chrono::high_resolution_clock::now();
	if (version == SLIDING) {
		sliding_puzzle_preprocessing(s_in, t_in, &s_gpu, &t_gpu, &expand_fun_cpu, &h_cpu, &states_delta_cpu,
				&expand_elements, &expand_element_size);
	} else if (version == PATHFINDING) {
		pathfinding_preprocessing(s_in, t_in, &s_gpu, &t_gpu, &expand_fun_cpu, &h_cpu, &states_delta_cpu,
				&expand_elements, &expand_element_size);
	}

	state **H;
	char ***expand_buf = expand_bufs_create(THREADS_PER_BLOCK * BLOCKS, expand_elements, expand_element_size);
	HANDLE_RESULT(cudaMalloc(&H, HASH_SIZE * sizeof(state*)));
	HANDLE_RESULT(cudaMemset(H, 0, HASH_SIZE * sizeof(state*)));
	heap **Q = heaps_create(k);
	list **Ss = lists_create(BLOCKS, 1000000);
	list *S = list_create(1024 * 1024);
	state *states_pool;
	char *nodes_pool;
	states_pool_create(&states_pool, &nodes_pool, expand_element_size);
	int total_Q_size_cpu;
	int found_cpu;
	int out_of_memory_cpu;

	init_heap<<<1, 1>>>(s_gpu, Q, states_pool, nodes_pool, expand_element_size);
	int step = 0;
	do {
		clear_list<<<1, 1>>>(S);
		HANDLE_RESULT(cudaDeviceSynchronize());
		fill_list<<<BLOCKS, THREADS_PER_BLOCK>>>(t_gpu, k, expand_element_size, Q, S, states_pool, nodes_pool,
				expand_buf, expand_fun_cpu, h_cpu, states_delta_cpu);
		HANDLE_RESULT(cudaMemcpyFromSymbol(&found_cpu, found, sizeof(int)));
		HANDLE_RESULT(cudaMemcpyFromSymbol(&out_of_memory, found, sizeof(int)));
		if (found_cpu) break;
		if (out_of_memory_cpu) break;
		HANDLE_RESULT(cudaDeviceSynchronize());
		deduplicate<<<BLOCKS, THREADS_PER_BLOCK>>>(H, S, t_gpu, h_cpu);
		HANDLE_RESULT(cudaDeviceSynchronize());
		push_to_queues<<<1, THREADS_PER_BLOCK>>>(t_gpu, k, Q, S, h_cpu, step) ;
		HANDLE_RESULT(cudaDeviceSynchronize());
		HANDLE_RESULT(cudaMemcpyFromSymbol(&total_Q_size_cpu, total_Q_size, sizeof(int)));
		step++;
	} while (total_Q_size_cpu > 0);

	auto end = std::chrono::high_resolution_clock::now();

	auto duration = end - start;
	output << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "\n";

	if (found_cpu) {
		char result_path_cpu[RESULT_LEN];
		HANDLE_RESULT(cudaMemcpyFromSymbol(result_path_cpu, result_path, RESULT_LEN));

		std::string path_str = std::string(result_path_cpu);
		std::istringstream path_stream;
		path_stream.str(result_path_cpu);

		std::vector<std::string> v;
		for (std::string line; std::getline(path_stream, line); ) {
			v.push_back(line);
		}
		std::reverse(v.begin(), v.end());
		if (version == SLIDING) {
			output << sliding_puzzle_postprocessing(v);
		} else if (version == PATHFINDING) {
			for (std::string path_el: v) {
				output << path_el << "\n";
			}
		}
	}

	states_pool_destroy(states_pool, nodes_pool);
	lists_destroy(Ss, BLOCKS);
	heaps_destroy(Q, k);
	HANDLE_RESULT(cudaFree(H));
	HANDLE_RESULT(cudaDeviceSynchronize());
}


__global__ void init_heap(const char *s, heap **Q, state *states_pool, char *nodes_pool, int state_len) {
	heap_insert(Q[0], state_create(s, 0, 0, NULL, states_pool, nodes_pool, state_len));
	atomicAdd(&total_Q_size, 1);
}

__device__ int processed = 0;
__device__ int steps = 0;
__device__ int heaps_min_before;

__global__ void clear_list(list *S) {
	list_clear(S);
}

__global__ void fill_list(const char *t, int k, int state_len,
		heap **Q, list *S, state *states_pool, char *nodes_pool,
		char ***expand_buf, expand_fun expand, heur_fun h, states_delta_fun states_delta) {
	state *m = NULL;
	int id = calculate_id();
	char **my_expand_buf = expand_buf[id];
	if (id == 0)steps++;
	for (int i = id; i < k; i += blockDim.x * gridDim.x) {
		if (Q[i]->size == 0) continue;
		state *q = heap_extract(Q[i]);
		atomicSub(&total_Q_size, 1);
		if (cuda_str_eq(q->node, t)) {
			if (m == NULL || f(q, t, h) < f(m, t, h)) {
				m = q;
			}
			continue;
		}
		expand(q->node, my_expand_buf);
		for (int j = 0; my_expand_buf[j][0] != '\0'; j++) {
			int delta = states_delta(q->node, my_expand_buf[j]);
			state *new_state = state_create(my_expand_buf[j], -1, q->g + delta, q, states_pool, nodes_pool, state_len);
			if (new_state == NULL) return;
			list_insert(S, new_state);
		}
	}
	if (m != NULL && f(m, t, h) <= heaps_min(Q, k)) {
		int found_before = atomicCAS(&found, 0, 1);
		if (found_before == 1) return;

		state *cur = m;
		int result_len = 0;
		while (cur != NULL) {
			int len = cuda_strlen(cur->node) + 1;
			memcpy(result_path + result_len, cur->node, len);
			result_len += len;
			result_path[result_len-1] = '\n';
			cur = cur->prev;
		}
		result_path[result_len-1] = '\0';
		return;
	}
}

__global__ void deduplicate(state **H, list *T, const char *t, heur_fun h) {
	int id = calculate_id();
	for (int i = id; i < T->length; i += blockDim.x * gridDim.x) {
		int z = 0;
		state *t1 = list_get(T, i);
		for (int j = 0; j < HASH_FUNS; j++) {
			assert(t1->node != NULL);
			state *el = H[jenkins_hash(j, t1->node) % HASH_SIZE];
			if (el == NULL || cuda_str_eq(t1->node, el->node)) {
				z = j;
				break;
			}
		}
		int index = jenkins_hash(z, t1->node) % HASH_SIZE;
		t1 = (state*)atomicExch((unsigned long long*)&(H[index]), (unsigned long long)t1);
		if (t1 != NULL && cuda_str_eq(t1->node, list_get(T, i)->node) &&
				f(list_get(T, i), t, h) >= f(t1, t, h)) {
			list_remove(T, i);
			continue;
		}
		t1 = list_get(T, i);
		for (int j = 0; j < HASH_FUNS; j++) {
			if (j != z) {
				state *el = H[jenkins_hash(j, t1->node) % HASH_SIZE];
				if (el != NULL && cuda_str_eq(el->node, t1->node) &&
						f(list_get(T, i), t, h) >= f(el, t, h)) {
					list_remove(T, i);
					break;
				}
			}
		}
	}
}

__global__ void push_to_queues(const char *t, int k, heap **Q, list *S, heur_fun h, int off) {
	for (int i = threadIdx.x; i < S->length; i += blockDim.x) {
		state *t1 = list_get(S, i);
		if (t1 != NULL) {
			t1->f = f(t1, t, h);
			heap_insert(Q[(i + off) % k], t1);
			atomicAdd(&processed, 1);
			atomicAdd(&total_Q_size, 1);
		}
		__syncthreads();
	}
}

__device__ int f(const state *x, const char *t, heur_fun h) {
	return x->g + h(x->node, t);
}

void states_pool_create(state **states, char **nodes, int node_size) {
	HANDLE_RESULT(cudaMalloc(states, STATES * sizeof(state)));
	HANDLE_RESULT(cudaMalloc(nodes, 3 * STATES * node_size * sizeof(char)));
	HANDLE_RESULT(cudaMemset(*states, 0, STATES * sizeof(state)));
	HANDLE_RESULT(cudaMemset(*nodes, 0, 3 * STATES * node_size * sizeof(char)));
}

void states_pool_destroy(state *states_pool, char *nodes_pool) {
	HANDLE_RESULT(cudaFree(states_pool));
	HANDLE_RESULT(cudaFree(nodes_pool));
}

char ***expand_bufs_create(int bufs, int elements, int element_size) {
	int bufs_size = bufs * sizeof(char**);
	char ***bufs_cpu = (char***)malloc(bufs_size);
	for (int i = 0; i < bufs; i++) {
		bufs_cpu[i] = expand_buf_create(elements, element_size);
	}
	char ***bufs_gpu;
	HANDLE_RESULT(cudaMalloc(&bufs_gpu, bufs_size));
	HANDLE_RESULT(cudaMemcpy(bufs_gpu, bufs_cpu, bufs_size, cudaMemcpyDefault));
	free(bufs_cpu);
	return bufs_gpu;

}

char **expand_buf_create(int elements, int element_size) {
	char **buf_cpu = (char**)malloc(elements * sizeof(char*));
	for (int i = 0; i < elements; i++) {
		HANDLE_RESULT(cudaMalloc(&(buf_cpu[i]), element_size));
	}
	char **buf_gpu;
	HANDLE_RESULT(cudaMalloc(&buf_gpu, elements * sizeof(char*)));
	HANDLE_RESULT(cudaMemcpy(buf_gpu, buf_cpu, elements * sizeof(char*),
				cudaMemcpyDefault));
	free(buf_cpu);
	return buf_gpu;

}

__device__ int used_states = 0;
__device__ state *state_create(const char *node, int f, int g, state *prev,
		state *states_pool, char *nodes_pool, int state_len) {
	int index = atomicAdd(&used_states, 1);
	if (index >= STATES || (long long)state_len * index >= (1<<30)) {
		out_of_memory = 1;
		return NULL;
	}
	state *result = &(states_pool[index]);
	memcpy(&(nodes_pool[(unsigned long long)state_len * index]), node, state_len);
	result->node = &(nodes_pool[state_len * index]);
	result->f = f;
	result->g = g;
	result->prev = prev;
	return result;
}

__device__ int calculate_id() {
	return threadIdx.x + blockIdx.x * blockDim.x;
}
