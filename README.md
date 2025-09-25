# Laika-Software
A repository for the software packages for the Laika quadruped robot.

## Dependencies
Follow these instructions to avoid dependency hell.

### Setup rosdep
To automatically install all necessary dependencies, this projects uses rosdep (https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html). 
Install Rosdep using these commands:
```sh
sudo apt install python3-rosdep
```
```sh
sudo rosdep init
rosdep update
```

### Dependencies
Now, clone this repository and use rosdep to install all dependencies automatically:
```sh
cd laika_ws && rosdep install --from-paths src -y --ignore-src
```

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
