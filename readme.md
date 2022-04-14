# Dev environment setup
Development utilises this repository https://github.com/devrt/ros-devcontainer-vscode

Ensure you have Docker, vscode and remote development extension for vscode installed.

0. (Optional) The development container extracts the information from the .gitconfig file in the host system. It is important to ensure that the file exists and it is up to date. If the file is missing, create it and add the relevant information to it, e.g.:

```
[user]
	name = Your Name
	email = you@helgen.tech
```

1. Clone the repository:
```shell
git clone git@github.com:devrt/ros-devcontainer-vscode.git
```

2. Open the folder in vscode
```shell
cd ros-devcontainer-vscode
code .
```

3. The option to reopen the container will pop up in the bottom right of the screen. Click it and you will enter the workspace of a ROS docker image

# Installing the Simulation Packages

### 1. Clone this repository into the workspace

Ensure there are no other files in the src folder, then run:
```shell
git clone --branch=simulation-test-fix --recurse-submodules -j8 --remote-submodules git@github.com:ciaran-helgen/wat-beacon-pub.git src
```
### This will clone several packages:
- The beacon_pub gazebo plugin which simulates comms propagation delay and provides ground truth beacon distance measurements
- The waypoint_gen package which was a simple test of calling the /go_to_waypoint(s) service(s)
- A fork of the waypoint_navigator package which includes a model of our drone with 4 rx beacons
### Explanation of arguments
- --recurse-submodules ensures the submodules are cloned.
- --remote-submodules ensures the specified branch in .gitmodules is cloned
- src is specified as the target directory. Otherwise a wat-beacon-bub folder would be created at /workspace

### 1.1 Older Git version
if the --remote-submodules option is unavailable, run the following instead:
```shell
git clone --branch=simulation-test-fix --recurse-submodules -j8 git@github.com:ciaran-helgen/wat-beacon-pub.git src
cd src/Trilateration-Python
git checkout ros
```

### 2. Install the simulation packages using rosinstall

Note: You do not need to clone this repository as it is included as a submodule.
The instructions are available here https://github.com/ciaran-helgen/waypoint_navigator/tree/master

# Running the Demos
Remember to source the catkin workspace whenever you open a new terminal
``` shell
source devel/setup.bash
```
## Start the simulation
```shell
roslaunch waypoint_navigator mav_sim.launch
```
Open a web browser and go to http://localhost:3000/ to view the GUI.
## Start the waypoint navigator
Open a new terminal, source the workspace and run:
```shell
roslaunch waypoint_navigator waypoint_navigator.launch
```
## Use the navigation services
Open a new terminal and source the workspace. 
### To navigate to a waypoint, e.g. (0, 0, 1.0)
```shell
rosservice call /firefly/go_to_waypoint "point: x: 0.0 y: 0.0 z: 1.0"
```
### To navigate to waypoints (The easier way)

Construct your command in a text editor and paste it into the command line.
```shell
rosservice call /firefly/go_to_waypoints "points:
- x: 0.0
  y: 0.0
  z: 0.0
- x: 1.0
  y: 0.0
  z: 1.0"
```
### To navigate to waypoints (finicky), e.g. (0, 0, 1.0), (1.0, 0, 1.0)

The specific format above is important. To add a new line without executing the command prematurely add a '\' then hit enter. I'm using the ▮ symbol to show where your caret is:
1. Start typing out the service call
```shell
rosservice call /firefly/go_to_waypoints ▮
```
2. then hit the tab key. ROS will autofill a blank message for you with one point
```shell
rosservice call /firefly/go_to_waypoints "points:
- x: 0.0
  y: 0.0
  z: 0.0▮
```
3. add a / and hit enter
```shell
rosservice call /firefly/go_to_waypoints "points:
- x: 0.0
  y: 0.0
  z: 0.0/▮
```
4. You will be on a new line with a preceding >
```shell
rosservice call /firefly/go_to_waypoints "points:
- x: 0.0
  y: 0.0
  z: 0.0
>▮
```
5. Type / followed by return to populate the resto of the message. The leading spaces and returns are important as we are constructing a formatted YAML string
```shell
rosservice call /firefly/go_to_waypoints "points:
- x: 0.0
  y: 0.0
  z: 0.0
>- x: 1.0
>  y: 0.0
>  z: 1.0"▮
7. Hit enter to execute the command. If you use the up arrow to access this command again the preceding >'s will not show.
