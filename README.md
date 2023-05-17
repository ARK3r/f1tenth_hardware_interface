# A ros2 package for testing the ackermann steering controller with the f1tenth

Much of the f1tenth components are from [f1tenth-dev/simulator](https://github.com/f1tenth-dev/simulator)

## launch files
`sim.launch.py` launches rviz with odom and rqt for sending commmands to the ackermann steering controller.

`f1.launch.py` launches rviz and `state_publisher_gui` to view the urdf and manualy handle joints.

## Usage
Build the docker image using the `build.sh` script.

Then run a container using the `run.sh` script. This allows for the container to be run with the correct permissions to access the display.

Finally from inside the container run:
```bash
source install/setup.bash
ros2 launch f1tenth_hardware_interface sim.launch.py
```