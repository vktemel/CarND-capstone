This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Summary

In order to run this project, please follow the installation and usage instructions written below. By following the usage instructions written below, and running the Udacity Simulator with Camera enabled and Manual mode disabled, the car will run autonomously in the loop using the pre-determined waypoints. The waypoints are based on the `data\sim_waypoints.csv` file. 

The implementations for this project are as following: 
- In `ros\src\waypoint_updater\waypoint_updater.py`, the next 200 waypoints are loaded to vehicle path for vehicle to follow. Also, if a traffic light is detected to be red, the vehicle is instructed to stop before reaching the intersection. After the light turns green, it will follow the waypoints again. 
- In `ros\src\twist_controller\dbw_node.py`, the function subscribes to the `dbw_enabled`, which indicates if autonomous operation is requested, `twist_cmd`, which contains what the vehicle target linear and angular speeds are, and `current_velocity`, which contains the vehicle's current linear and angular speeds. It will also publish the throttle, brake and steer commands received from twist_controller. 
- In `ros\src\twist_controller\twist_controller.py`, the function will receive the current and target linear and angular velocities, and calculate the steering, throttle and brake commands of the vehicle based on these targets.
- In `ros\src\tl_detector\tl_detector.py`, the traffic lights on the path of the vehicle are searched based on the previously provided waypoints. If a light is found ahead of the vehicle, then the state of the light is obtained via the ros message that carries the light status. 


### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
