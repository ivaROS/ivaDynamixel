# ivaDynamixel

(IVALab-tailored) ROS stack for interfacing with Robotis Dynamixel line of servo motors.

## quickstart

### testing

We've provided a few docker containers to test library package functionality across a few versions of ROS i.e., kinetic, melodic, and noetic.
The purpose is to validate behavior of the module in both Python 2 and 3.
Containers take advantage of containerd, which uses Linux namespaces and control groups to run isolated images on the host kernel.
We mount `/dev/ttyUSB0` onto the container, and validate that we can communicate with dynamixel motors from each ROS distribution.

We can build all container images:

```bash
docker compose build
```

This may take some time, since this will pull down the base images for each of the images.
Instead, you can specify a single container to build and run:

```bash
docker compose kinetic

# script to verify information from motors
docker compose run kinetic python src/ivaDynamixel/dynamixel_driver/scripts/info_dump.py 1 2 3

# to start the controller manager node
docker compose run kinetic roslaunch dynamixel_tutorials controller_manager.launch
```

Note that workspace is set under `/app` and the source for the repository under `/app/src/ivaDynamixel`.
The containers run using the host ip stack, so all ROS nodes will be available under `rostopic` or `rosservice`.
