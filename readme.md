# Dev environment setup
Development utilises this repository https://github.com/devrt/ros-devcontainer-vscode

Ensure you have Docker, vscode and remote development extension for vscode installed.

1. Clone the repository:
```shell
$ git clone git@github.com:devrt/ros-devcontainer-vscode.git
```

2. Open the folder in vscode
```shell
$ cd ros-devcontainer-vscode
$ code .
```

3. The option to reopen the container will pop up in the bottom right of the screen. Cllck it and you will enter the workspace of a ROS docker image

# Installing the Simulation Packages

1. Clone this repository into src
```shell
$ cd src
$ git clone --recurse-submodules -j8 git@github.com:ciaran-helgen/wat-beacon-pub.git
```
This will clone several packages:
- The beacon_pub gazebo plugin which simulates comms propagation delay and provides ground truth beacon distance measurements
- The waypoint_gen package which was a simple test of calling the /go_to_waypoint(s) service(s)
- A fork of the waypoint_navigator package which includes a model of our drone with 4 rx beacons

2. Install the simulation packages using rosinstall
The instructions are available here https://github.com/ciaran-helgen/waypoint_navigator/tree/master

# Running the Demos
TODO
