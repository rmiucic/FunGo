# Capstone Project
## Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car

The goals of this project are :

* Vehicle follow a predetermined path
* Stops at the intersection when the traffic light is RED
* Classify the state of traffic lights for Simulation and Real Life 
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
[SiliconCar]:https://github.com/SiliconCar/CarND-Capstone
[Alex]:https://github.com/alex-lechner/Traffic-Light-Classification
[alex lechner dataset]: https://www.dropbox.com/s/vaniv8eqna89r20/alex-lechner-udacity-traffic-light-dataset.zip?dl=0
[coldknight dataset]: https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI#get-the-dataset
[labelImg]: https://github.com/tzutalin/labelImg
[ssd inception 171117]: http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2017_11_17.tar.gz
[sim test]: ./imgs/sim_test.png
[real test]: ./imgs/real_test.png

## Table of Contents
1. [Introduction](#introduction)
2. [Car and Simulator](#2-Car-and-Simulator)
3. [System](#3-System)
    1. [Simulator](#31-Simulator)
    2. [Control](#32-Control)
    3. [Planning](#33-Planning)
    4. [Perception](#34-Perception)
4. [Performance Videos](#4-Performance-Videos)
    1. [Simulation Test](#41-Simulation-Test)
    2. [ROSBag Test](#42-ROSBag-Test)
5. [Useful ROS Commands](#5-ROS-Commands)

## Introduction

## 2. Car and Simulator
This block represents either a Carla vehicle or Simulator. My development was done using Udacity simulator. 
1. Download Udacity simulator [linux_sys_int.zip][udacity simulator].
2. Extract the content of linux_sys_int.zip into new location e.g ``~/Desktop/linux_sys_int``
3. Open new terminal and go to the simulation folder ``$cd ~/Desktop/linux_sys_int``
4. Change execution permission ``sudo chmod a+x``
5. Run the simulator (you will need machine with GPU in order to run the simulator) ``./sys_int.x86_64``

## 3. System
In general, I followed steps suggested in udacity class. the picture bellow shows the data flow of publishing and subscribing nodes and topics. 
![publishers and subscribers][system overview img] 

### 3.1 Simulator
The simulator publishes the following topics
* ``/image_color`` - front view of camera image
* ``/current_pose`` - x,y,and heading of the vehicle
* ``/current_velocity`` - speed of the vehicle
* ``/vehicle/dbw_enabled`` - drive by wire enabled 
The simulator subsrcibes to the following topics
* ``/vehicle/brake_cmd`` - command to brake (from DBW node)
* ``/vehicle/steering_cmd`` - command to steer (from DBW node)
* ``/vehicle/throttle_cmd`` - command to throttle (from DBW node)

### 3.2 Control
This block translates twist commands to the commands that vehicle understands. The vehicle's waypoint follower will publish twist commands to the ``/twist_cmd`` topic. The goal for this part of the project is to implement the drive-by-wire node (dbw_node.py) which will subscribe to ``/twist_cmd`` and use various controllers to provide appropriate throttle, brake, and steering commands. These commands are then published to the following topics:
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
This file contains the Controller class. This class implements vehicle control. The control method can take twist data as input and return throttle, brake, and steering values. Within this class, ``pid.py`` and ``lowpass.py`` are used to control acceleration, and ``yaw_controller.py`` controls steering. If vehicle is in not autonomous mode ``not dbw_enabled`` then reset the throttle PID controller.
```python
def control(self,current_vel, dbw_enabled, linear_vel, angular_vel):
    if not dbw_enabled:
        self.throttle_controller.reset()
        return 0.0, 0.0, 0.0
...
```

#### ``yaw_controller.py``
A controller used in ``twist_controller.py`` to convert target linear and angular velocity to steering commands.

#### ``pid.py``
A generic PID controller used in ``twist_controller.py``.

#### ``lowpass.py``
A generic low pass filter used in ``twist_controller.py`` namely to filter velocity.

#### ``dbw_test.py``
Use this file to test your DBW code against a bag recorded with a reference implementation. The bag can be found at [dbw bag]. Detailed use instructions can be found in the dbw_test.py file.

### 3.3 Planning
#### ``waypoint_updater.py``
This node publishes a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles.<br>
the node subsrcibes to
* ``/base_waypoints``
* ``/current_pose``
* `/traffic_waypoint`

publishes
* ``/final_waypoints`` 
* ``/obstacle_waypoint`` (not implemented in my code)

As sugested in the udacity class the development of this module was in several phases. First, I implemented a publishing  ``/final_waypoints`` as a subset of waypoints copmosed of points from ``/base_waypoints``, from host vehicle closest point to some number of points  ``LOOKAHEAD_WPS = 200`` ahead.
```python
def publish_waypoints(self):
    lane = Lane()
    lane.header = self.base_waypoints.header
    lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
    self.final_waypoints_pub.publish(lane)
```
After figuring out traffic ligth state (againg first from simulator and later properly with Classifier) I modified publishing ``/final_waypoints`` to take into accont the state of the traffic light. Where ``generate_lane()`` returns a lane from ``/base_waypoints`` if traffic light is not RED or return the lane with decelerating points if the traffic light is RED (``stopline_wp_idx`` is not -1) and traffic light stop line is with in the lookahead points. Topic ``/traffic_waypoint`` sets the ``stopline_wp_idx``. Topic ``/traffic_waypoint`` is being published by Traffic Ligth detector node.
```python
def publish_waypoints(self):
    final_lane = self.generate_lane()
    self.final_waypoints_pub.publish(final_lane)
```

#### ``waypoint_loader.py``
This node loads waypoints from the CSV file. It should load only once and latch the topic.<br>
publishes
* ``/base_waypoints`` -> all the points from the track a vehicle is to follow

#### ``waypoint_follower``
Implementation of pure persuit in CPP. The algorithm follows trajectory setup in ``\final_waypoints``. The only change here I did was to forced ``following_flag`` to be ``false`` so that the algorithm always follows waypoints.

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

### 3.4 Perception
First, let me start with the hardest part: aligning stars for TensorFlow to work with GPU! This took me a while. The rest of the project was also time consuming but more straight forward.  Overall it was a lot of fun. I did the following:
* Internet research on whatis the best way to detect traffic light state. One of the questions was is it better to use classical CV or neural networks (NN).
* Finally I settled on using neural networks but then I had to decide the structure. 
* Now, do I use one NN for detecting the traffic signs boxes and feed the cropped image to anothe NN to do the state detection or use Single Shot Detection - SSD? 
* In the end I decided to use a single NN (). Reading [SiliconCar][SiliconCar] README helped me a lot in this decision. 

#### ``tl_detector.py``
prepares image for the classiffier and publishes the traffic waypoint if the state is RED or -1 otherwise. This node schedules the frequency of traffic light classifier implemented in ``tl_classifier.py``.
subscribes to:
``/current_pose``
``/base_waypoints``
``/vehicle/traffic_lightshtArray``
``/image_color``

publishes:
``/traffic_waypoint``

I empirically found that classifying traffic light at 5Hz is good enough rate to be able to stop on a RED light in the simulation.  
```python
def ros_spin(self):
    rate = rospy.Rate(5)
...
    if self.pose is not None and self.waypoints is not None and self.camera_image is not None:
        light_wp, state = self.process_traffic_lights()
...
```
#### ``tl_classifier.py``
This is where the classifier is implemented. I was influenced and inspiered reading Alexander Lechner [README][Alex] file for the traffic light classifier. This tutorial helped me a lot in with chosing the suitable NN, making and getting the datasets ready for training and finally training the NN. I prety much followed the instruction from Alexander's tutorial. In training I used combination of small self generated datasets and datasets found on internet, namely:
1. [Vatsal's dataset][coldknight dataset]
2. [Alex's dataset][alex lechner dataset] 

#### Extract images from a ROSbag file
For the simulator data, I drove around the track in the simulator and recorded a ROSbag file (sim.bag). For real data, Udacity provides ROSbag file from Carla and extracting images will be the same. 

1. Open a terminal and launch ROS 
    ```bash
    roscore
    ```
2. Open another terminal and play the ROSbag file
    ```bash
    rosbag play -l path/to/sim.bag
    ```
3. Create a directory where you want to save the images e.g. sim_img
    ```bash
    sudo mkdir sim_img
    ```

4. Open another, third terminal and navigate to the newly created directory sim_img and... 
    ```bash
    cd sim_img
    ```
    
    1. ...execute the following statement if you have a ROSbag file from Udacity's simulator:
        ```bash
        rosrun image_view image_saver _sec_per_frame:=0.01 image:=/image_color
        ```
    

I used [labelImg][labelImg]. It's very user-friendly and easy to set up. Here is an example of a labeled image
![labeling a traffic light][labeling img] 

All the tarining and validation files are stored in TFRecord file. TFRecord file is needed in order to retrain a TensorFlow model. A TFRecord is a binary file that stores images and ground truth labels. I have used [ssd_inception_v2_coco_2017_11_17][ssd inception 171117] model and retrained it. I took about 6 hours to train the NN. I am using two different instances of the classifier: one for simulation environment and one for the real life environment. 

### 4. Performance Videos

#### 4.1 Simulation Test
In the [simulation video][simulation video] you can see performance of autonomous driving in the Udacity simulation. Below the simulation screen is the terminal output showing the SSD classification results and time it took TensorFlow to compute it.

1. Start the simulation: in a new terminal from where the simulator is installed execute
```bash
./sys_int.x86_64
```
2. Start the simulation launch script: in new terminal execute
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

![sim test][sim test]

#### 4.2 ROSBag Test
In the rog bag testing [video][real video], you can see camera output of the prerecorded bag file, and a terminal showing the output of the classifier.
1. roscore: in one terminal execute
```bash
rocore
```
2. ROSBag playback: in another terminal execute
```bash
rosbag play -l loop_with_traffic_light.bag
```
3. Camera output: in yet another terminal execute:
```bash
rqt_image_view /image_raw
```
4. Start the site launch script: in new and final terminal execute
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/site.launch
```

![ROSBag test][real test]

### 5. ROS-Commands
Useful ROS commands<br>
**rostopic info /current_pose** -> get more info on the topic
```bash
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
```bash
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
**rosbag info 1767290_2019-03-28-21-05-48.bag** -> get the information about the ROS bag 
```shell
radovan@radovan-GS63VR-6RF:/media/radovan/Linux_HD/udacity/FunGo_review$ rosbag info 1767290_2019-03-28-21-05-48.bag 
path:        1767290_2019-03-28-21-05-48.bag
version:     2.0
duration:    1:23s (83s)
start:       Mar 29 2019 00:05:48.57 (1553832348.57)
end:         Mar 29 2019 00:07:12.37 (1553832432.37)
size:        2.8 GB
messages:    213610
compression: none [2054/2054 chunks]
types:       actionlib_msgs/GoalStatusArray           [8b2b82f13216d0a8ea88bd3af735e619]
             bond/Status                              [eacc84bf5d65b6777d4c50f463dfb9c8]
             can_msgs/Frame                           [64ae5cebf967dc6aae4e78f5683a5b25]
             dbw_mkz_msgs/BrakeCmd                    [c0d20e1056976680942e85ab0959826c]
             dbw_mkz_msgs/BrakeInfoReport             [fc88af128b5b3213ea25ab325a9b3bbb]
             dbw_mkz_msgs/BrakeReport                 [a92bad28c400885f36170c1cab44618e]
...
```


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
