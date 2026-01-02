# Laika-Software
A repository for the software packages for the Laika quadruped robot.

# Wiki
Check out the wiki: [Wiki](../../wiki)

## Build
To finish the install, all packages provided have to be build.

**IMPORTANT:** This is also necessary after updating the repository or making any changes to the code.

The build command has to be run in the \<path to repository\>/laika_ws folder.
```sh
cd Laika-Software/laika_ws
```
Build Command:
```sh
colcon build
```
Don't forget to source to workspace:
```sh
source ./install/local_setup.sh
```
## Run the Simulation
After building and sourcing the workspace:
```sh
ros2 launch laika_sim simulation.launch.py
```
To start the simulation with the robot flying (good for some testing):
```sh
ros2 launch laika_sim simulation.launch.py fly:=true
```
To start the simulation with the robot on a vertical slider (good for some testing):
```sh
ros2 launch laika_sim simulation.launch.py slide:=true
```
## Make the joints move
After starting the simulation, run these in a seperate terminal:

Make the joints go up and down smothly (sin wave):
```sh
ros2 run laika_control set_joints_sin
```

Make the joints go to random positions:
```sh
ros2 run laika_control set_joints_random
```

Make the joints extended:
```sh
ros2 run laika_control set_joints_extended
```

