# Steps to Replicate Workspace

1. Follow the steps in the [F1tenth Gym repository](https://github.com/f1tenth/f1tenth_gym_ros) to install the simulation environment.
2. `roscd f1tenth_gym_ros`
3. `rm -r scripts` (The existing files in the *scripts* folder have been moved to our repository)
4. `git clone --branch dev https://github.com/trunc8/F1tenth_IROS2020.git src`
5. Replace the `params.yaml` file with the one provided here in this setup directory. The only change in this file is the name of the map file.
6. `roscd f1tenth_gym_ros/launch`
7. Replace the `gym_bridge.launch` file with the one provided here in this setup directory. The only change in this file is the name of the map file.
8. `roscd f1tenth_gym_ros/maps`
9. Copy the files `vegas*` and `berlin*` provided here in this setup directory into the `maps` directory.
10. `cd /path/to/catkin_ws/src`
11. Clone the [obstacle_detector](https://github.com/yangfuyuan/obstacle_detector) as an external package:  
	`git clone https://github.com/yangfuyuan/obstacle_detector.git`
12. `roscd obstacle_detector`
13. Replace the `CMakeLists.txt` file with the one provided here in this setup directory.
14. `roscd obstacle_detector/src`
15. Replace the `obstacle_detector.cpp` file with `obstacle_detector_src.cpp` provided here in this setup directory.
16. Roscd f1tenth_gym_ros
17. Create src and include folders
18. Add MPC_Node.cpp and MPC.cpp to the f1tenth_gym_ros/src folder
19. Add MPC.h (inside setup/include) to the f1tenth_gym_ros/include folder
20. `cd /path/to/catkin_ws`
21. `catkin_make -j4`
22. `roscd f1tenth_gym_ros`
23. `sudo ./build_docker.sh` (**Note**: This must be run every time you change the map file path [Recall steps 5 and 7] )


