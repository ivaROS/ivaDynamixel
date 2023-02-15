# ivaDynamixel

(IVALab-tailored) ROS stack for interfacing with Robotis Dynamixel line of servo motors.
This package is compatible with Python 2 and Python 3.

## quickstart

Add the package to your catkin workspace and build it.

```bash
# where ~/catkin_ws is your catkin workspace
cd ~/catkin_ws/src
git clone https://github.com/ivaROS/ivaDynamixel.git
cd ..
catkin build
```

Then include it in your package:

```xml
  <run_depend>dynamixel_controllers</run_depend>
```

See the [dynamixel_tutorials](./dynamixel_tutorials) package for examples on how to use the package.

### general testing

We've provided a few docker containers to test library package functionality across a few versions of ROS i.e., kinetic, melodic, and noetic.
The purpose is to validate behavior of the module in both Python 2 and 3.
Containers take advantage of containerd, which uses Linux namespaces and control groups to run isolated images on the host kernel.
We mount `/dev/ttyUSB0` onto the container, and validate that we can communicate with dynamixel motors from each ROS distribution.

Ensure your user is part of the `dialout` and `docker` groups.

We can build all container images:

```bash
docker compose build
```

This may take some time, since this will pull down the base images for each of the images.
Instead, you can specify a single container to build and run:

```bash
docker compose build kinetic

# script to verify information from motors
docker compose run --rm kinetic python src/ivaDynamixel/dynamixel_driver/scripts/info_dump.py 1 2 3

# to start the controller manager node
docker compose run --rm kinetic roslaunch dynamixel_tutorials controller_manager.launch
```

We can run any arbitrary script or launch file using the docker images.
If you run into any issues, make sure that there are no other programs actively communicating on the serial device on `/dev/ttyUSB0`.

Note that workspace is set under `/app` and the source for the repository under `/app/src/ivaDynamixel`.
The containers run using the host ip stack, so all ROS nodes will be available under `rostopic` or `rosservice`.

### verifying behavior on python 2 and 3

#### basic serialization - info_dump.py

As an example, here is motor information run from kinetic on python 2, melodic on python 2, and noetic on python 3:

```bash
$ docker compose run --rm noetic python src/ivaDynamixel/dynamixel_driver/scripts/info_dump.py 1
Pinging motors:
1 ... done
    Motor 1 is connected:
        Freespin: False
        Model ------------------- EX-106+ (firmware version: 28)
        Min Angle --------------- 0
        Max Angle --------------- 4095
        Current Position -------- 798
        Current Speed ----------- 0
        Current Temperature ----- 37°C
        Current Voltage --------- 8.1v
        Current Load ------------ 0
        Moving ------------------ False

$ docker compose run --rm melodic python src/ivaDynamixel/dynamixel_driver/scripts/info_dump.py 1
Pinging motors:
1 ... done
    Motor 1 is connected:
        Freespin: False
        Model ------------------- EX-106+ (firmware version: 28)
        Min Angle --------------- 0
        Max Angle --------------- 4095
        Current Position -------- 798
        Current Speed ----------- 0
        Current Temperature ----- 37\u00B0C
        Current Voltage --------- 8.1v
        Current Load ------------ 0
        Moving ------------------ False

$ docker compose run --rm kinetic python src/ivaDynamixel/dynamixel_driver/scripts/info_dump.py 1
Pinging motors:
1 ... done
    Motor 1 is connected:
        Freespin: False
        Model ------------------- EX-106+ (firmware version: 28)
        Min Angle --------------- 0
        Max Angle --------------- 4095
        Current Position -------- 798
        Current Speed ----------- 0
        Current Temperature ----- 37\u00B0C
        Current Voltage --------- 8.1v
        Current Load ------------ 0
        Moving ------------------ False
```

Testing on melodic is tricky, since this is the first version of ROS that supported python 2 and 3, via `ROS_PYTHON_VERSION`.
At time of writing, we can simplify assumptions that most code and modules from this point is written for ROS noetic (or ROS 2), and thus sets `ROS_PYTHON_VERSION=3`.

#### ros nodes - controller manager and spawner

We also test the behavior of the controller manager and spawner on python 2 vs python 3.
The main difference is a change in import mechanism, but there is refactoring of the code too.

We will run `controller_manager.launch` and `controller_spawner.launch` from the `dynamixel_tutorials` package.
These need to get run in separate terminals in order to verify behavior.

Via kinetic:

```bash
docker compose run --rm kinetic roslaunch dynamixel_tutorials controller_manager.launch
docker compose run --rm kinetic roslaunch dynamixel_tutorials controller_spawner.launch

# from the dynamixel manager
process[dynamixel_manager-2]: started with pid [67]
[INFO] [1676480719.748477]: pan_tilt_port: Pinging motor IDs 1 through 25...
[INFO] [1676480722.229833]: pan_tilt_port: Found 9 motors - 1 MX-106 [2], 3 EX-106+ [1, 3, 4], 2 AX-12 [8, 9], 3 MX-28 [5, 6, 7], initialization complete.
[INFO] [1676480727.857555]: JointPositionController::JointPositionController() - Motor 5 bias set to 0.0000

[INFO] [1676480727.903481]: JointPositionController::JointPositionController() - Motor 6 bias set to 0.0000

# from the dynamixel spawner
process[dynamixel_controller_spawner-1]: started with pid [39]
[INFO] [1676480727.815928]: pan_tilt_port controller_spawner: waiting for controller_manager dxl_manager to startup in global namespace...
[INFO] [1676480727.820646]: pan_tilt_port controller_spawner: All services are up, spawning controllers...
[INFO] [1676480727.873546]: Controller pan_controller successfully started.
[INFO] [1676480727.916967]: Controller tilt_controller successfully started.
```

Via noetic:

```bash
docker compose run --rm noetic roslaunch dynamixel_tutorials controller_manager.launch
docker compose run --rm noetic roslaunch dynamixel_tutorials controller_spawner.launch

# from the dynamixel manager
process[dynamixel_manager-2]: started with pid [49]
[INFO] [1676480834.207882]: pan_tilt_port: Pinging motor IDs 1 through 25...
[INFO] [1676480836.688936]: pan_tilt_port: Found 9 motors - 3 EX-106+ [1, 3, 4], 1 MX-106 [2], 3 MX-28 [5, 6, 7], 2 AX-12 [8, 9], initialization complete.
[INFO] [1676480841.728839]: JointPositionController::JointPositionController() - Motor 5 bias set to 0.0000

[INFO] [1676480841.763914]: JointPositionController::JointPositionController() - Motor 6 bias set to 0.0000

# from the dynamixel spawner
process[dynamixel_controller_spawner-1]: started with pid [32]
[INFO] [1676480841.695405]: pan_tilt_port controller_spawner: waiting for controller_manager dxl_manager to startup in global namespace...
[INFO] [1676480841.704013]: pan_tilt_port controller_spawner: All services are up, spawning controllers...
[INFO] [1676480841.739664]: Controller pan_controller successfully started.
[INFO] [1676480841.775393]: Controller tilt_controller successfully started.
[dynamixel_controller_spawner-1] process has finished cleanly
```
