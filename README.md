# ivaDynamixel

(IVALab-tailored) ROS stack for interfacing with Robotis Dynamixel line of servo motors.
This package is compatible with Python 2 and Python 3.

## quickstart

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
docker compose kinetic

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
