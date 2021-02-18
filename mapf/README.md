# mapf-adapters

# After cloning the repo
```
cd mapf-adapters
git submodule update --init --recursive
```
## Building

Tested on Ubuntu 18.04.

```
mkdir build
cd build
cmake -DCPLEX_DIR=<cplex directory> ..
make
```
(Remember to copy the scipoptsuite-6.0.2 folder to library/bcp-mapf-mirror/library and comment yaml-cpp include directory line in EPEA* CMakeLists.txt)

## Run example instances

### CBS

```
./mapf -i ../example/input.yaml 
python3 ../example/visualize.py ../example/input.yaml ../example/output.yaml
```

## Misc Info

Use the following command to update the submodule. Then commit the changes.
```
git submodule foreach git pull origin master
```

Running using the following command will skip ICTS solver.
```
./mapf -i ../example/input.yaml -j 0
```

w tag can be given as input for ECBS for finding paths with suboptimal cost.
```
./mapf -i ../example/input.yaml -w 1.4
```
