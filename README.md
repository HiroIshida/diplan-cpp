#  Kinodynamic sampling based motion planning
This repository implements Fast Marching Tree (FMT) [1] and Rapidly-Exploring Random Tree (RRT) algorithms for a double integrator system. Kinodynamic planning can be complex, as connecting states isn't always straightforward. However, leveraging the nilpotent dynamics of the double integrator, we can easily compute optimal state-to-state connections, as pointed out in [2].

This implementation is essentially a port of my previous Julia implementation, which can be found at https://github.com/HiroIshida/julia_motion_planning. Significant refactoring and bug fixes have been carried out.

build cpp example and run
```bash
git clone git@github.com:HiroIshida/diplan-cpp.git
cd diplan-cpp
git submodule update --init --recursive --recommend-shallow --depth 1
mkdir build && cd build
cmake .. -DWITH_MATPLOTLIB=ON && make -j4
./fmt_example  # run
```

build python binding
```bash
git clone git@github.com:HiroIshida/diplan-cpp.git
cd diplan-cpp
git submodule update --init --recursive --recommend-shallow --depth 1
pip3 install -e .
```


[1] Janson, Lucas, et al. "Fast marching tree: A fast marching sampling-based method for optimal motion planning in many dimensions." The International journal of robotics research 34.7 (2015): 883-921.

[2] Webb, Dustin J., and Jur van den Berg. "Kinodynamic RRT*: Optimal motion planning for systems with linear differential constraints." arXiv preprint arXiv:1205.5088 (2012).
