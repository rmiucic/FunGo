# Capstone Project
## Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car

The goals of this project are :

* Vehicle follow a predetermined path
* Stops at the intersection when the traffic light is RED
* Classify the state of traffic lights 
* Summarize the results with a written report


[//]: # (References)
[capstone project]: https://github.com/iburris/CarND-Capstone
[labeling img]: ./imgs/labeling_example.png
[path folowing img]: ./imgs/path_folowing.png
[system overview img]: ./imgs/system_overview.png
[real video]: ./imgs/real.mp4
[simulation video]: ./imgs/sim.mp4
[udacity simulator]: https://github.com/udacity/CarND-Capstone/releases
[dbw bag]: https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/reference.bag.zip

## Table of Contents
1. [Introduction](#introduction)
2. [Car/Simulator](#Car/Simulator)
3. [System](#system)
    * [Simulator](#Simulator)
    * [Control](#Control)
    * [Planning](#Planning)
    * [Perception](#Perception)
4. [Useful ROS Commands](#ROS-Commands)


## Introduction
First, let me start with the hardest part: aligning stars for TensorFlow to work with GPU! This took me a while. The rest of the project was also time consuming but more straight forward.  Overall it was a lot of fun.
## Car/Simulator
This block represents either a Carla vehicle or Simulator. My development was done using Udacity simulator. 
1. Download Udacity simulator [linux_sys_int.zip][udacity simulator].
2. Extract the content of linux_sys_int.zip into new location e.g ``~/Desktop/linux_sys_int``
3. Open new terminal and go to the simulation folder ``$cd ~/Desktop/linux_sys_int``
4. Change execution permission ``sudo chmod a+x``
5. Run the simulator (you will need machine with GPU in order to run the simulator) ``./sys_int.x86_64``

## System
In general, I followed steps suggested in udacity class. the picture bellow shows the data flow of publishing and subscribing nodes and topics names. 
![publishers and subscribers][system overview img] 

### Simulator
The simulator publishes the following topics
* ``/image_color`` - front view of camera image
* ``/current_pose`` - x,y,and heading of the vehicle
* ``/current_velocity`` - speed of the vehicle
* ``/vehicle/dbw_enabled`` - drive by wire enabled 
#### The simulator subsrcibesto the following topics
* ``/vehicle/brake_cmd`` - command to brake (from DBW node)
* ``/vehicle/steering_cmd`` - command to steer (from DBW node)
* ``/vehicle/throttle_cmd`` - command to throttle (from DBW node)

### Control
This block translates twist commands to the commands that vehicle understands. The vehicle's waypoint follower will publish twist commands to the ``/twist_cmd`` topic. The goal for this part of the project is to implement the drive-by-wire node (dbw_node.py) which will subscribe to ``/twist_cmd`` and use various controllers to provide appropriate throttle, brake, and steering commands. These commands can then be published to the following topics:
* ``/vehicle/throttle_cmd``
* ``/vehicle/brake_cmd``
* ``/vehicle/steering_cmd``

#### ``dbw_node.py ``
publishes the commands at 50Hz, and only does that when DBW is enabled.
```python
def loop(self):
    rate = rospy.Rate(50) # 50Hz
    while not rospy.is_shutdown():
        if self.dbw_enabled and not None in (self.current_vel, self.linear_vel, self.angular_vel):

            self.throttle, self.brake, self.steering = self.controller.control(self.current_vel, 
                                                                                self.dbw_enabled, 
                                                                                self.linear_vel, 
                                                                                self.angular_vel)
            self.publish(self.throttle, self.brake, self.steering)
        rate.sleep()
```
#### ``twist_controller.py``
This file contains a stub of the Controller class. You can use this class to implement vehicle control. For example, the control method can take twist data as input and return throttle, brake, and steering values. Within this class, you can import and use the provided pid.py and lowpass.py if needed for acceleration, and yaw_controller.py for steering. Note that it is not required for you to use these, and you are free to write and import other controllers.

#### ``yaw_controller.py``
A controller that used in ``twist_controller.py`` to convert target linear and angular velocity to steering commands.

#### ``pid.py``
A generic PID controller used in ``twist_controller.py``.

#### ``lowpass.py``
A generic low pass filter used in ``twist_controller.py`` to filter velocity.

#### ``dbw_test.py``
You can use this file to test your DBW code against a bag recorded with a reference implementation. The bag can be found at [dbw bag]. Detailed use instructions can be found in the dbw_test.py file.

### Planning
#### ``waypoint_updater.py``
This node publishes a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles.
subsrcibes to
* ``/base_waypoints``
* ``/current_pose``
* `/traffic_waypoint`

publishes
* ``/final_waypoints`` 
* ``/obstacle_waypoint`` (not implemented in my code)

#### ``waypoint_loader.py``
This node loads waypoints from the CSV file. It should load only once and latch the topic.
publishes
* ``/base_waypoints`` -> all the points from the track a vehicle is to follow

#### ``waypoint_follower``
Implementation of pure persuit in CPP. Here I forced ``following_flag`` to be ``false`` so that the algorithm always follows waypoints.

```CPP
geometry_msgs::Twist PurePursuit::calcTwist(double curvature, double cmd_velocity) const
{
  //rmiucic: the flag is forced to false in order for comands to be recomputed every time. 
  // bool following_flag = verifyFollowing();
  bool following_flag = false;
  static double prev_angular_velocity = 0;

...
```
#### ``twist_controller.py``
This nodes prepare variables for the ``/twist_cmd`` published in ``dbw_node.py``

### Perception


### ROS-Commands
**rostopic info /current_pose** -> get more info on the topic
```shell
radovan@radovan-GS63VR-6RF:~/git/rmiucic/FunGo$ rostopic info /current_pose 
Type: geometry_msgs/PoseStamped

Publishers: 
 * /styx_server (http://radovan-GS63VR-6RF:33792/)

Subscribers: 
 * /pure_pursuit (http://radovan-GS63VR-6RF:33230/)
 * /waypoint_updater (http://radovan-GS63VR-6RF:46200/)
 * /tl_detector (http://radovan-GS63VR-6RF:40336/)
 ```
**rosmsg info geometry_msgs/PoseStamped** -> get more info on the message type
```shell
radovan@radovan-GS63VR-6RF:~/git/rmiucic/FunGo$ rosmsg info geometry_msgs/PoseStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```


1. System Setup
1.1 Hardware
1.2 Software

---
# Original Udacity Read.me
---
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

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
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
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
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

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

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
