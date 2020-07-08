![CI](https://github.com/JeroenDM/elion/workflows/CI/badge.svg)

# elion

Constrained planning in MoveIt using OMPL's constrained planning interface

Refactored version of: [https://github.com/JeroenDM/moveit_constrained_planning_plugin](https://github.com/JeroenDM/moveit_constrained_planning_plugin)

Work in progress.

[API documentation for elion_planner package.](https://jeroendm.github.io/elion/docs/html/index.html)

## Build instructions

Start with the usual commands:
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/JeroenDM/elion.git
rosdep install --from-paths . --ignore-src
```

and add the unreleased test dependencies using [vcstool](https://github.com/dirk-thomas/vcstool).
```
vcs import < elion/test_dependencies.repos
```

At the moment I'm mostly building in Debug mode:
```bash
cd ..
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
catkin build
source devel/setup.bash
```

**Note**: for the examples below, I recommend building in release mode. Especially planning with collision objects is slow (+100 seconds) at the moment. In debug mode you need a lot of patience to get a solution :)
```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run examples
Planning problems are configured through a json file. Examples are present in the folder `elion_examples/config/...`. To run an example, first launch the MoveIt configuration for a specific robot. All examples with a name `panda_xxxx.json` use the panda robot. The examples package contains a specific launch file to properly configure Rviz.

```bash
roslaunch elion_examples panda_example.launch 
```

Examples with a name `kuka_xxxx.json` use the Kuka KR5 Arc robot. The configuration is available in [kuka_test_resources](https://github.com/JeroenDM/kuka_test_resources.git).

```bash
roslaunch kuka_kr5_moveit_config demo.launch 
```

When Rviz is visible and the move group node is ready, you can run an example, for example `panda_pos_con.json` does some simple planning with position constraints:

```bash
rosrun elion_examples elion_run_example panda_1.json
```

![panda_example](doc/panda_example.gif)

## Run tests
Build the workspace again with the tests argument added.
```
catkin build --make-args tests
```

`elion_planner` has a rostest that initialized different constriants for the panda robot:
```bash
rostest elion_planner test_constraints_panda.rostest
```

Some basic tests exist to check the conversion of an angular velocity vector to roll, pitch, way velocity. I like to run this test directly from the target in the devel space to get readable output.
```bash
./devel/lib/elion_planner/test_elion_conversions
```





